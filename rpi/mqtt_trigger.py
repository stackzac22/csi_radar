#!/usr/bin/env python3
"""
CSI Radar — MQTT motion subscriber + camera snapshot grabber (RPi 4B).

The XIAO ESP32-S3 Sense (RX) publishes a JSON motion event to the broker
(mosquitto on this Pi) whenever its CSI motion detector fires. This script:

  1. subscribes to the motion topic,
  2. pulls the JPEG the XIAO serves (the snapshot URL is in the payload),
  3. saves it under --save-dir with a timestamped filename,
  4. optionally forwards the event to the csi_reader.py dashboard /alert
     endpoint so it shows up in the live UI alert log.

Broker setup (once):  sudo apt install mosquitto mosquitto-clients
Install:              pip3 install paho-mqtt requests
Run:                  python3 mqtt_trigger.py --broker localhost
"""

import argparse
import json
import logging
import os
import time
from datetime import datetime

import requests
import paho.mqtt.client as mqtt

# ── CLI args ───────────────────────────────────────────────────────────────────
parser = argparse.ArgumentParser()
parser.add_argument("--broker", default="localhost", help="MQTT broker host")
parser.add_argument("--port",   default=1883, type=int, help="MQTT broker port")
parser.add_argument("--topic",  default="csi/motion", help="Topic to subscribe to")
parser.add_argument("--save-dir", default="captures",
                    help="Directory to save motion snapshots into")
parser.add_argument("--dashboard-alert", default="http://localhost:5000/alert",
                    help="csi_reader.py /alert endpoint to forward events to "
                         "(pass '' to skip)")
parser.add_argument("--snapshot-timeout", default=5.0, type=float,
                    help="HTTP timeout (s) when grabbing the XIAO snapshot")
parser.add_argument("--cooldown", default=3.0, type=float,
                    help="Minimum seconds between snapshot grabs")
args = parser.parse_args()

logging.basicConfig(level=logging.INFO,
                    format="%(asctime)s [%(levelname)s] %(message)s")
log = logging.getLogger("mqtt-trigger")

os.makedirs(args.save_dir, exist_ok=True)
_last_grab = 0.0


def grab_snapshot(url: str):
    """Fetch the JPEG the XIAO serves and save it. Returns the path or None."""
    try:
        r = requests.get(url, timeout=args.snapshot_timeout)
        r.raise_for_status()
        ts = datetime.now().strftime("%Y%m%d-%H%M%S")
        path = os.path.join(args.save_dir, f"motion-{ts}.jpg")
        with open(path, "wb") as f:
            f.write(r.content)
        log.info("snapshot saved → %s (%d bytes)", path, len(r.content))
        return path
    except Exception as e:
        log.warning("snapshot grab failed (%s): %s", url, e)
        return None


def forward_to_dashboard(payload: dict, snapshot_path):
    if not args.dashboard_alert:
        return
    try:
        requests.post(args.dashboard_alert, json={
            "source":   "mqtt",
            "type":     payload.get("trigger", "motion"),
            "id":       payload.get("id"),
            "diff":     payload.get("diff"),
            "snapshot": snapshot_path,
        }, timeout=2)
    except Exception as e:
        log.warning("dashboard forward failed: %s", e)


def on_connect(client, userdata, flags, reason_code, properties=None):
    log.info("connected to %s:%d (rc=%s) — subscribing to %s",
             args.broker, args.port, reason_code, args.topic)
    client.subscribe(args.topic)


def on_message(client, userdata, msg):
    global _last_grab
    try:
        payload = json.loads(msg.payload.decode())
    except Exception:
        log.warning("bad payload on %s: %r", msg.topic, msg.payload[:80])
        return

    log.info("MOTION %s", payload)

    now = time.time()
    if now - _last_grab < args.cooldown:
        return
    _last_grab = now

    snapshot_path = None
    url = payload.get("snapshot")
    if url:
        snapshot_path = grab_snapshot(url)
    forward_to_dashboard(payload, snapshot_path)


def main():
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    client.on_connect = on_connect
    client.on_message = on_message
    log.info("connecting to broker %s:%d …", args.broker, args.port)
    client.connect(args.broker, args.port, keepalive=60)
    client.loop_forever()


if __name__ == "__main__":
    main()
