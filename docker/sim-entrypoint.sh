#!/usr/bin/env bash
# Webots serves the streaming viewer and every extern controller on one port.
set -euo pipefail

WORLD="${WORLD:-worlds/track.wbt}"
MODE="${MODE:-realtime}"

exec xvfb-run -a webots \
  --stream \
  --port=1234 \
  --batch \
  --stdout \
  --stderr \
  --mode="${MODE}" \
  "${WORLD}"
