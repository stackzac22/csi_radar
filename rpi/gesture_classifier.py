#!/usr/bin/env python3
"""
Gesture classifier — DTW template matching over the CSI radar WebSocket feed.

Record a reference gesture (3-2-1 countdown, then captures for --duration):
    python3 gesture_classifier.py record swipe_left --duration 1.5

Classify live gestures against everything saved in gestures/:
    python3 gesture_classifier.py classify

Detections are POSTed to the dashboard's own /alert endpoint, so they show up
in the web dashboard's Alerts panel and the Android app's CSI Radar screen —
the same path used for motion/gesture alerts.
"""

import argparse
import json
import os
import threading
import time
from collections import deque

import requests
import websocket

GESTURE_DIR = os.path.join(os.path.dirname(__file__), "gestures")
os.makedirs(GESTURE_DIR, exist_ok=True)

parser = argparse.ArgumentParser()
parser.add_argument("mode", choices=["record", "classify"])
parser.add_argument("label", nargs="?", help="gesture name (record mode only)")
parser.add_argument("--ws-url", default="ws://127.0.0.1:5000/ws")
parser.add_argument("--alert-url", default="http://127.0.0.1:5000/alert")
parser.add_argument("--duration", default=1.5, type=float, help="seconds per gesture window")
parser.add_argument("--max-distance", default=12.0, type=float,
                     help="DTW distance threshold below which a match counts")
parser.add_argument("--cooldown", default=2.0, type=float, help="seconds between detections")
parser.add_argument("--rest-margin", default=1.0, type=float,
                    help="gesture DTW must beat the idle 'rest' baseline by this much to count")
parser.add_argument("--use-motion-gate", action="store_true",
                    help="only classify when the dashboard flags motion (mean-variance). "
                         "Off by default: subtle hand gestures don't move the mean magnitude, "
                         "so this gate suppresses them. DTW trajectory matching works without it.")
args = parser.parse_args()


def vec_dist(a: list, b: list) -> float:
    n = min(len(a), len(b))
    return sum((a[i] - b[i]) ** 2 for i in range(n)) ** 0.5


def dtw_distance(seq_a: list, seq_b: list) -> float:
    n, m = len(seq_a), len(seq_b)
    if n == 0 or m == 0:
        return float("inf")
    INF = float("inf")
    prev = [INF] * (m + 1)
    prev[0] = 0.0
    for i in range(1, n + 1):
        cur = [INF] * (m + 1)
        cur[0] = INF
        for j in range(1, m + 1):
            cost = vec_dist(seq_a[i - 1], seq_b[j - 1])
            cur[j] = cost + min(prev[j], cur[j - 1], prev[j - 1])
        prev = cur
    # normalize by path length so templates/windows of different lengths are comparable
    return prev[m] / (n + m)


def load_templates() -> dict:
    templates = {}
    for fname in os.listdir(GESTURE_DIR):
        if fname.endswith(".json"):
            label = fname[:-5]
            with open(os.path.join(GESTURE_DIR, fname)) as f:
                templates[label] = json.load(f)
    return templates


class FrameStream:
    """Connects to the dashboard WebSocket and exposes incoming mags frames."""

    def __init__(self, ws_url: str):
        self.ws_url = ws_url
        self.on_frame = None
        self._ws = None

    def _on_message(self, ws, message):
        try:
            data = json.loads(message)
        except json.JSONDecodeError:
            return
        if "ping" in data or data.get("mags") is None:
            return
        if self.on_frame:
            self.on_frame(data)

    def run_forever(self):
        while True:
            self._ws = websocket.WebSocketApp(self.ws_url, on_message=self._on_message)
            self._ws.run_forever()
            print("[stream] disconnected, retrying in 2s...")
            time.sleep(2)

    def start(self):
        t = threading.Thread(target=self.run_forever, daemon=True)
        t.start()


def record_mode():
    if not args.label:
        parser.error("record mode requires a label, e.g. 'record swipe_left'")

    buffer = []
    state = {"recording": False}

    def on_frame(frame):
        if state["recording"]:
            buffer.append(frame["mags"])

    stream = FrameStream(args.ws_url)
    stream.on_frame = on_frame
    stream.start()

    print(f"Recording '{args.label}' — get ready to perform the gesture.")
    for n in (5, 4, 3, 2, 1):
        print(n)
        time.sleep(1)
    print(f"GO — recording for {args.duration}s...")
    state["recording"] = True
    time.sleep(args.duration)
    state["recording"] = False

    if len(buffer) < 2:
        print(f"Only captured {len(buffer)} frames — too few (check the dashboard is "
              "running and streaming). Not saving.")
        return

    out_path = os.path.join(GESTURE_DIR, f"{args.label}.json")
    with open(out_path, "w") as f:
        json.dump(buffer, f)
    print(f"Saved {len(buffer)} frames to {out_path}")


def classify_mode():
    all_templates = load_templates()
    if not all_templates:
        print(f"No templates found in {GESTURE_DIR} — record some first with "
              "'record <label>'.")
        return

    # "rest" is an idle baseline (record it standing still), not a gesture.
    # A window only counts as a gesture if it matches a gesture template AND is
    # clearly more gesture-like than the idle baseline — this kills the constant
    # false firing you get when every window just picks its nearest gesture.
    rest_template = all_templates.pop("rest", None)
    templates = all_templates
    if not templates:
        print("Only a 'rest' template found — record at least one gesture too.")
        return
    print(f"Loaded gestures: {', '.join(templates.keys())}"
          + ("  + rest baseline" if rest_template else "  (no rest baseline — "
             "record one with 'record rest' to cut false positives)"))

    lengths = [len(t) for t in templates.values()]
    if rest_template:
        lengths.append(len(rest_template))
    max_template_len = max(lengths)
    window = deque(maxlen=max_template_len + 5)
    motion_window = deque(maxlen=max_template_len + 5)
    last_detection = {"label": None, "ts": 0.0}

    def on_frame(frame):
        window.append(frame["mags"])
        motion_window.append(frame.get("motion", "still"))
        if len(window) < 3:
            return

        # Optional motion gate (off by default). Subtle hand gestures barely move
        # the mean magnitude, so this gate tends to suppress them; the DTW
        # trajectory match below is the real discriminator. Enable only for
        # large/close motions where the mean-variance flag is reliable.
        if args.use_motion_gate and not any(m and m != "still" for m in motion_window):
            return

        best_label, best_dist = None, float("inf")
        for label, template in templates.items():
            k = len(template)
            if len(window) < k:
                continue
            candidate = list(window)[-k:]
            dist = dtw_distance(candidate, template)
            if dist < best_dist:
                best_label, best_dist = label, dist

        if best_label is None:
            return

        # Compare against the idle baseline: the window must be more like the
        # gesture than like rest (by --rest-margin) to count.
        rest_dist = float("inf")
        if rest_template is not None:
            k = len(rest_template)
            if len(window) >= k:
                rest_dist = dtw_distance(list(window)[-k:], rest_template)

        now = time.time()
        gesture_wins = best_dist < rest_dist - args.rest_margin
        if (best_dist < args.max_distance
                and gesture_wins
                and now - last_detection["ts"] > args.cooldown):
            last_detection["label"] = best_label
            last_detection["ts"] = now
            rest_str = f", rest={rest_dist:.2f}" if rest_dist != float("inf") else ""
            print(f"DETECTED: {best_label}  (dtw={best_dist:.2f}{rest_str})")
            try:
                requests.post(args.alert_url, json={
                    "trigger": f"gesture:{best_label}",
                    "id": "gesture_classifier",
                }, timeout=2)
            except Exception as e:
                print(f"alert post failed: {e}")

    stream = FrameStream(args.ws_url)
    stream.on_frame = on_frame
    stream.start()

    print("Classifying live... Ctrl+C to stop.")
    while True:
        time.sleep(1)


if __name__ == "__main__":
    if args.mode == "record":
        record_mode()
    else:
        classify_mode()
