#!/usr/bin/env python3
"""
CSI Radar — RPi 4B serial reader, live dashboard, and alert forwarder.

Reads newline-delimited JSON frames from XIAO ESP32-S3 over USB serial,
streams live CSI magnitude data via WebSocket to a browser dashboard,
detects motion/gestures, and forwards alerts to RaspyJack over Tailscale.

Install:  pip3 install pyserial flask flask-sock requests
Run:      python3 csi_reader.py
          python3 csi_reader.py --port /dev/ttyACM1 --alert-url http://100.x.x.x:5000/alert
"""

import argparse
import json
import logging
import math
import queue
import threading
import time
from collections import deque
from datetime import datetime

import requests
import serial
from flask import Flask, jsonify, render_template_string
from flask_sock import Sock

# ── CLI args ───────────────────────────────────────────────────────────────────
parser = argparse.ArgumentParser()
parser.add_argument("--port",      default="/dev/ttyACM0", help="Serial port for XIAO")
parser.add_argument("--baud",      default=115200, type=int)
parser.add_argument("--host",      default="0.0.0.0",  help="Dashboard bind address")
parser.add_argument("--web-port",  default=5000, type=int, help="Dashboard HTTP port")
parser.add_argument("--alert-url", default=None, help="RaspyJack/RPi alert endpoint")
parser.add_argument("--motion-thresh", default=60.0, type=float,
                    help="Subcarrier-variance-sum threshold for motion. Calibrated "
                         "2026-06-16: idle tops out ~40, motion starts ~90. 'gesture' "
                         "(strong motion) fires at 4x this. Re-measure if geometry changes.")
# ── Coarse control: fire an action on confirmed sustained motion ─────────────────
parser.add_argument("--action-url", default=None,
                    help="URL to fire on confirmed motion (e.g. a Home Assistant webhook). "
                         "POSTed JSON by default; use --action-method GET for plain webhooks.")
parser.add_argument("--action-method", default="POST", choices=["POST", "GET"],
                    help="HTTP method for --action-url")
parser.add_argument("--action-cmd", default=None,
                    help="Shell command to run on confirmed motion (e.g. a CLI to toggle a plug)")
parser.add_argument("--action-min-frames", default=3, type=int,
                    help="Consecutive motion frames required before firing (debounces noise)")
parser.add_argument("--action-cooldown", default=8.0, type=float,
                    help="Minimum seconds between fired actions")
args = parser.parse_args()

# ── Logging ────────────────────────────────────────────────────────────────────
logging.basicConfig(level=logging.INFO,
                    format="%(asctime)s [%(levelname)s] %(message)s")
log = logging.getLogger("csi")

# ── Shared state ───────────────────────────────────────────────────────────────
HISTORY     = 300                        # frames kept in ring buffer
WIN_MOTION  = 20                         # sliding window for motion variance

frame_lock    = threading.Lock()
frame_history: deque = deque(maxlen=HISTORY)
mean_history:  deque = deque(maxlen=WIN_MOTION)
mags_history:  deque = deque(maxlen=WIN_MOTION)
latest: dict         = {}
motion_state: str    = "init"

# WebSocket subscribers: each connected client gets its own queue
_ws_lock    = threading.Lock()
_ws_clients: dict[int, queue.Queue] = {}
_ws_id      = 0

# ── CSI maths ──────────────────────────────────────────────────────────────────
def iq_to_mag(csi: list) -> list:
    """Convert flat [I, Q, I, Q, ...] list → per-subcarrier magnitudes."""
    mags = []
    for i in range(0, len(csi) - 1, 2):
        r, im = csi[i], csi[i + 1]
        mags.append(math.sqrt(r * r + im * im))
    return mags


def variance(vals: list) -> float:
    if not vals:
        return 0.0
    mu = sum(vals) / len(vals)
    return sum((v - mu) ** 2 for v in vals) / len(vals)


def subcarrier_var_sum(window: list) -> float:
    """Sum of each subcarrier's temporal variance across the window.

    This is the real coarse-motion signal. The scalar mean magnitude collapses
    all subcarriers into one number, so motion that pushes subcarriers in
    opposite directions cancels out (measured 2026-06-16: mean-variance does NOT
    separate motion from idle, but this does — still tops out ~40, motion ~90+).
    """
    if len(window) < 2:
        return 0.0
    L = min(len(m) for m in window)
    total = 0.0
    for i in range(L):
        col = [m[i] for m in window]
        total += variance(col)
    return total


def classify_motion(mags: list) -> str:
    mags_history.append(mags)
    mean_history.append(sum(mags) / len(mags) if mags else 0.0)  # kept for display
    if len(mags_history) < 5:
        return "init"
    score = subcarrier_var_sum(list(mags_history))
    if score > args.motion_thresh * 4:
        return "gesture"
    if score > args.motion_thresh:
        return "motion"
    return "still"


# ── Alert forwarding ───────────────────────────────────────────────────────────
_last_alert = 0.0
ALERT_COOLDOWN = 5.0   # seconds between outgoing alerts

def send_alert(state: str, frame: dict):
    global _last_alert
    if not args.alert_url:
        return
    now = time.time()
    if now - _last_alert < ALERT_COOLDOWN:
        return
    _last_alert = now
    payload = {
        "trigger": state,
        "id":      frame.get("id", "?"),
        "chip":    frame.get("chip", "?"),
        "rssi":    frame.get("rssi", 0),
        "ts":      frame.get("ts_wall", ""),
    }
    try:
        requests.post(args.alert_url, json=payload, timeout=2)
        log.info("Alert sent → %s (%s)", args.alert_url, state)
    except Exception as e:
        log.warning("Alert failed: %s", e)


# ── Coarse control: action trigger ──────────────────────────────────────────────
import subprocess

_motion_run   = 0       # consecutive motion/gesture frames seen
_action_armed = True    # must return to "still" before firing again
_last_action  = 0.0

def fire_action(frame: dict):
    """Fire the user-configured webhook and/or shell command on confirmed motion."""
    global _last_action
    now = time.time()
    if now - _last_action < args.action_cooldown:
        return
    _last_action = now
    fired = []
    if args.action_url:
        try:
            if args.action_method == "GET":
                requests.get(args.action_url, timeout=3)
            else:
                requests.post(args.action_url, json={
                    "event":    "motion",
                    "state":    frame.get("motion"),
                    "mean_mag": frame.get("mean_mag"),
                    "id":       frame.get("id"),
                    "ts":       frame.get("ts_wall"),
                }, timeout=3)
            fired.append(f"{args.action_method} {args.action_url}")
        except Exception as e:
            log.warning("action-url failed: %s", e)
    if args.action_cmd:
        try:
            subprocess.Popen(args.action_cmd, shell=True)
            fired.append(f"cmd: {args.action_cmd}")
        except Exception as e:
            log.warning("action-cmd failed: %s", e)
    if fired:
        log.info("ACTION fired → %s", "; ".join(fired))


def update_action_trigger(new_state: str, frame: dict):
    """Edge+sustain trigger: fire once after N consecutive motion frames, then
    re-arm only after motion settles back to still."""
    global _motion_run, _action_armed
    if new_state in ("motion", "gesture"):
        _motion_run += 1
        if _action_armed and _motion_run >= args.action_min_frames:
            fire_action(frame)
            _action_armed = False
    elif new_state == "still":
        _motion_run = 0
        _action_armed = True


# ── Frame processing ───────────────────────────────────────────────────────────
def process(raw: dict) -> dict | None:
    global motion_state

    csi = raw.get("csi")
    if not csi or not isinstance(csi, list):
        return None

    mags = iq_to_mag(csi)
    active = [m for m in mags if m > 0.1]   # skip DC null subcarriers
    if not active:
        return None

    mean_mag = sum(active) / len(active)
    new_state = classify_motion(mags)
    motion_score = subcarrier_var_sum(list(mags_history))

    frame = {
        **raw,
        "mags":         [round(m, 2) for m in mags],
        "mean_mag":     round(mean_mag, 2),
        "motion_score": round(motion_score, 1),
        "n_active":     len(active),
        "motion":       new_state,
        "ts_wall":      datetime.utcnow().isoformat(timespec="milliseconds"),
    }

    if new_state in ("motion", "gesture") and motion_state == "still":
        log.info("STATE → %s (score=%.1f mean=%.2f)", new_state, motion_score, mean_mag)
        send_alert(new_state, frame)

    update_action_trigger(new_state, frame)

    motion_state = new_state
    return frame


def publish(frame: dict):
    """Push frame to all WebSocket subscribers."""
    msg = json.dumps({
        "t":      frame.get("t"),
        "id":     frame.get("id"),
        "rssi":   frame.get("rssi"),
        "ch":     frame.get("ch"),
        "mags":   frame.get("mags"),
        "mean":   frame.get("mean_mag"),
        "motion": frame.get("motion"),
    })
    with _ws_lock:
        dead = []
        for cid, q in _ws_clients.items():
            try:
                q.put_nowait(msg)
            except queue.Full:
                dead.append(cid)
        for cid in dead:
            del _ws_clients[cid]


# ── Serial reader thread ───────────────────────────────────────────────────────
def serial_reader():
    log.info("Opening %s @ %d baud", args.port, args.baud)
    while True:
        try:
            with serial.Serial(args.port, args.baud, timeout=2) as ser:
                log.info("Serial connected")
                buf = b""
                while True:
                    chunk = ser.read(ser.in_waiting or 1)
                    if not chunk:
                        continue
                    buf += chunk
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        line = line.strip()
                        if not line:
                            continue
                        try:
                            raw = json.loads(line)
                        except json.JSONDecodeError:
                            continue
                        frame = process(raw)
                        if frame is None:
                            continue
                        with frame_lock:
                            frame_history.append(frame)
                            latest.update(frame)
                        publish(frame)
        except serial.SerialException as e:
            log.warning("Serial error: %s — retrying in 3s", e)
            time.sleep(3)
        except Exception as e:
            log.error("Reader crash: %s", e)
            time.sleep(3)


# ── Flask app + WebSocket ──────────────────────────────────────────────────────
app  = Flask(__name__)
wsock = Sock(app)

DASHBOARD_HTML = """<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<title>CSI Radar</title>
<script src="https://cdn.jsdelivr.net/npm/chart.js@4"></script>
<style>
  body { background:#0d1117; color:#e6edf3; font-family:monospace; margin:0; padding:16px; }
  h1   { color:#58a6ff; margin:0 0 4px; font-size:1.2em; }
  #status { font-size:.85em; color:#8b949e; margin-bottom:12px; }
  #motion { font-size:1.1em; font-weight:bold; margin-bottom:12px; }
  .still   { color:#3fb950; }
  .motion  { color:#d29922; }
  .gesture { color:#f85149; }
  .init    { color:#8b949e; }
  .grid { display:grid; grid-template-columns:1fr 1fr; gap:12px; }
  .card { background:#161b22; border:1px solid #30363d; border-radius:6px; padding:12px; }
  canvas { width:100%!important; }
  #stats { font-size:.8em; color:#8b949e; margin-top:8px; }
</style>
</head>
<body>
<h1>CSI Radar — Live Dashboard</h1>
<div id="status">connecting...</div>
<div id="motion" class="init">■ INIT</div>
<div class="grid">
  <div class="card">
    <b>Subcarrier Magnitude</b>
    <canvas id="specChart"></canvas>
  </div>
  <div class="card">
    <b>Mean Magnitude (history)</b>
    <canvas id="meanChart"></canvas>
  </div>
</div>
<div id="stats"></div>
<div class="card" style="margin-top:12px">
  <b>Alerts</b>
  <div id="alertLog" style="font-size:.8em;color:#f85149;max-height:120px;overflow-y:auto;margin-top:6px"></div>
</div>
<script>
const MEAN_HISTORY = 120;
const specCtx  = document.getElementById('specChart').getContext('2d');
const meanCtx  = document.getElementById('meanChart').getContext('2d');

const specChart = new Chart(specCtx, {
  type: 'bar',
  data: { labels: [], datasets: [{ label: 'Magnitude', data: [],
    backgroundColor: '#1f6feb', borderColor: '#388bfd', borderWidth: 0 }] },
  options: { animation: false, plugins: { legend: { display: false } },
    scales: { x: { display: false }, y: { min: 0, max: 50,
      ticks: { color: '#8b949e' }, grid: { color: '#21262d' } } } }
});

const meanData  = Array(MEAN_HISTORY).fill(null);
const meanChart = new Chart(meanCtx, {
  type: 'line',
  data: { labels: Array(MEAN_HISTORY).fill(''),
    datasets: [{ label: 'Mean', data: meanData, borderColor: '#58a6ff',
      borderWidth: 1.5, pointRadius: 0, fill: false, tension: 0.2 }] },
  options: { animation: false, plugins: { legend: { display: false } },
    scales: { x: { display: false }, y: { min: 0, max: 40,
      ticks: { color: '#8b949e' }, grid: { color: '#21262d' } } } }
});

let frames = 0, lastSeen = Date.now();

const ws = new WebSocket(`ws://${location.host}/ws`);
ws.onopen  = () => document.getElementById('status').textContent = 'connected';
ws.onclose = () => document.getElementById('status').textContent = 'disconnected — refresh to reconnect';
ws.onmessage = (ev) => {
  const d = JSON.parse(ev.data);
  frames++;
  lastSeen = Date.now();

  // Spectrum chart
  if (d.mags) {
    specChart.data.labels   = d.mags.map((_, i) => i);
    specChart.data.datasets[0].data = d.mags;
    specChart.update();
  }

  // Mean history
  meanData.shift(); meanData.push(d.mean ?? 0);
  meanChart.data.datasets[0].data = meanData;
  meanChart.update();

  // Motion badge
  const el = document.getElementById('motion');
  const labels = { still:'■ STILL', motion:'▲ MOTION', gesture:'★ GESTURE', init:'■ INIT' };
  el.textContent = labels[d.motion] ?? d.motion;
  el.className = d.motion;

  // Alert log
  if (d.alert) {
    const el = document.getElementById('alertLog');
    const line = document.createElement('div');
    line.textContent = `[${new Date().toLocaleTimeString()}] ${d.alert.source ?? 'fw'} ${d.alert.type ?? d.alert.trigger ?? '?'}: ${d.alert.text ?? d.alert.id ?? ''}`;
    el.prepend(line);
    while (el.children.length > 20) el.removeChild(el.lastChild);
  }

  // Stats
  document.getElementById('stats').textContent =
    `id:${d.id}  rssi:${d.rssi} dBm  ch:${d.ch}  frames:${frames}  ` +
    `mean:${d.mean?.toFixed(1)}  ${new Date().toLocaleTimeString()}`;
};
</script>
</body>
</html>"""

@app.route("/")
def dashboard():
    return render_template_string(DASHBOARD_HTML)

@app.route("/api/latest")
def api_latest():
    with frame_lock:
        return jsonify(latest)

@app.route("/api/history")
def api_history():
    with frame_lock:
        return jsonify(list(frame_history)[-50:])

@app.route("/api/motion")
def api_motion():
    return jsonify({"state": motion_state, "ts": datetime.utcnow().isoformat()})

# Ring buffer for inbound alerts (kismet + firmware)
_alerts: deque = deque(maxlen=50)
_alerts_lock = threading.Lock()

@app.route("/alert", methods=["POST"])
def alert_in():
    from flask import request
    data = request.get_json(silent=True) or {}
    log.warning("ALERT: %s", data)
    with _alerts_lock:
        _alerts.append({**data, "ts_wall": datetime.utcnow().isoformat(timespec="milliseconds")})
    # Broadcast alert to all WS clients
    publish({"t": None, "id": data.get("id", data.get("bssid", "?")),
             "rssi": 0, "ch": data.get("channel", "?"),
             "mags": None, "mean_mag": 0, "motion": "alert",
             "alert": data})
    return jsonify({"ok": True})

@app.route("/api/alerts")
def api_alerts():
    with _alerts_lock:
        return jsonify(list(_alerts))

@wsock.route("/ws")
def ws_handler(ws):
    global _ws_id
    with _ws_lock:
        _ws_id += 1
        cid = _ws_id
        _ws_clients[cid] = queue.Queue(maxsize=30)
    log.info("WS client #%d connected", cid)
    try:
        while True:
            try:
                msg = _ws_clients[cid].get(timeout=5)
                ws.send(msg)
            except queue.Empty:
                ws.send('{"ping":1}')   # keepalive
    except Exception:
        pass
    finally:
        with _ws_lock:
            _ws_clients.pop(cid, None)
        log.info("WS client #%d disconnected", cid)


# ── Entry point ────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    t = threading.Thread(target=serial_reader, daemon=True, name="serial")
    t.start()
    log.info("Dashboard → http://%s:%d", args.host, args.web_port)
    app.run(host=args.host, port=args.web_port, threaded=True)
