#!/usr/bin/env python3
"""
Kismet → CSI dashboard + RaspyJack alert bridge.

Polls Kismet's REST API for new alerts and forwards them to:
  - Local CSI dashboard  (/alert endpoint)
  - RaspyJack over Tailscale (/alert endpoint)

Run:  python3 kismet_bridge.py
      python3 kismet_bridge.py --kismet-url http://localhost:2501 \
              --dashboard-url http://localhost:5000/alert \
              --raspyjack-url http://100.x.x.x:5000/alert
"""

import argparse
import json
import logging
import time

import requests
from requests.auth import HTTPBasicAuth

parser = argparse.ArgumentParser()
parser.add_argument("--kismet-url",     default="http://localhost:2501")
parser.add_argument("--kismet-user",    default="kismet")
parser.add_argument("--kismet-pass",    default="kismet")
parser.add_argument("--dashboard-url",  default="http://localhost:5000/alert")
parser.add_argument("--raspyjack-url",  default=None,
                    help="RaspyJack Tailscale alert URL (optional)")
parser.add_argument("--poll-interval",  default=3.0, type=float,
                    help="Seconds between Kismet alert polls")
# Alert types to forward (Kismet alert type strings)
parser.add_argument("--alert-types",    default="DEAUTHFLOOD,APSPOOF,BSSTIMESTAMP,NEWADHOC,PROBECLIENTFLOOD",
                    help="Comma-separated Kismet alert types to forward")
args = parser.parse_args()

logging.basicConfig(level=logging.INFO,
                    format="%(asctime)s [%(levelname)s] %(message)s")
log = logging.getLogger("kismet_bridge")

ALERT_TYPES = set(args.alert_types.upper().split(","))
auth = HTTPBasicAuth(args.kismet_user, args.kismet_pass)

_seen_ts = 0   # track last alert timestamp to avoid re-sending


def kismet_get(path: str) -> dict | list | None:
    try:
        r = requests.get(f"{args.kismet_url}{path}", auth=auth, timeout=5)
        r.raise_for_status()
        return r.json()
    except Exception as e:
        log.warning("Kismet API error (%s): %s", path, e)
        return None


def forward(payload: dict):
    targets = [args.dashboard_url]
    if args.raspyjack_url:
        targets.append(args.raspyjack_url)
    for url in targets:
        try:
            requests.post(url, json=payload, timeout=3)
            log.info("Forwarded → %s | %s", url, payload.get("type", "?"))
        except Exception as e:
            log.warning("Forward failed → %s: %s", url, e)


def poll_alerts():
    global _seen_ts
    data = kismet_get("/alerts/last-time/0/alerts.json")
    if not data:
        return
    alerts = data if isinstance(data, list) else data.get("alerts", [])
    for alert in alerts:
        ts = alert.get("kismet.alert.timestamp", 0)
        if ts <= _seen_ts:
            continue
        alert_type = alert.get("kismet.alert.header", "UNKNOWN").upper()
        if alert_type not in ALERT_TYPES:
            log.debug("Skipping alert type: %s", alert_type)
            continue
        payload = {
            "source":   "kismet",
            "type":     alert_type,
            "text":     alert.get("kismet.alert.text", ""),
            "bssid":    alert.get("kismet.alert.bssid", ""),
            "channel":  alert.get("kismet.alert.channel", ""),
            "ts":       ts,
        }
        log.warning("KISMET ALERT [%s]: %s", alert_type, payload["text"])
        forward(payload)
        _seen_ts = max(_seen_ts, ts)


def poll_new_devices():
    """Log newly seen devices — useful for detecting unknown clients."""
    data = kismet_get("/devices/last-time/-30/devices.json")
    if not data:
        return
    devices = data if isinstance(data, list) else []
    for dev in devices:
        mac  = dev.get("kismet.device.base.macaddr", "?")
        ssid = dev.get("kismet.device.base.name", "")
        mfr  = dev.get("kismet.device.base.manuf", "")
        ch   = dev.get("kismet.device.base.channel", "?")
        sig  = dev.get("kismet.device.base.signal", {}).get("kismet.common.signal.last_signal", 0)
        log.info("Device: %s  '%s'  %s  ch:%s  sig:%d dBm", mac, ssid, mfr, ch, sig)


def check_kismet_alive() -> bool:
    info = kismet_get("/system/status.json")
    if info:
        version = info.get("kismet.system.version", "?")
        sources = info.get("kismet.system.datasources.running", 0)
        log.info("Kismet OK — version:%s sources_running:%s", version, sources)
        return True
    return False


if __name__ == "__main__":
    log.info("Kismet bridge starting — polling %s every %.1fs", args.kismet_url, args.poll_interval)
    log.info("Alert types: %s", ", ".join(sorted(ALERT_TYPES)))

    # Wait for Kismet to be ready
    for attempt in range(10):
        if check_kismet_alive():
            break
        log.info("Waiting for Kismet... (%d/10)", attempt + 1)
        time.sleep(3)
    else:
        log.error("Kismet not reachable after 30s — exiting")
        raise SystemExit(1)

    log.info("Forwarding alerts → dashboard: %s", args.dashboard_url)
    if args.raspyjack_url:
        log.info("Forwarding alerts → RaspyJack: %s", args.raspyjack_url)

    while True:
        try:
            poll_alerts()
            poll_new_devices()
        except Exception as e:
            log.error("Poll error: %s", e)
        time.sleep(args.poll_interval)
