# CSI Radar → Home Assistant

The coarse-motion trigger can reach Home Assistant two ways. Both hit the **same
webhook**, so set up the HA side once.

## 1. HA side — create the webhook automation

Paste into `automations.yaml` (or the Automations UI → edit in YAML), then reload
automations. Change `light.living_room` to whatever you want to control.

```yaml
- alias: CSI Radar motion
  trigger:
    - platform: webhook
      webhook_id: csi_motion        # → URL path
      local_only: true              # set false only if hitting from off-LAN
      allowed_methods: [POST, GET]
  action:
    - service: light.toggle
      target:
        entity_id: light.living_room
```

Webhook URL (LAN): `http://<HA-IP>:8123/api/webhook/csi_motion`
(no auth token needed — the random-ish `webhook_id` is the secret).

## 2a. Phone fires it directly (chosen path)

In the app's **CSI Radar** screen:
- Put the webhook URL in the "Home Assistant webhook URL" field.
- Flip **ARMED** on. Each motion/gesture event (5 s cooldown) buzzes the phone and
  POSTs the webhook. "Fire now" tests it manually.

## 2b. Pi dashboard fires it (no phone needed)

```bash
.venv/bin/python -u csi_reader.py --port /dev/ttyACM0 \
  --alert-url http://127.0.0.1:5000/alert \
  --action-url http://<HA-IP>:8123/api/webhook/csi_motion \
  --action-method POST \
  --action-min-frames 3 --action-cooldown 8
```

Motion detection uses the calibrated subcarrier-variance-sum feature
(`--motion-thresh 60`, "gesture"/strong-motion at 4×). Re-measure if the room
geometry changes.
