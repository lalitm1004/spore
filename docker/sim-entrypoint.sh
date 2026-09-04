#!/usr/bin/env bash
# Webots serves the streaming viewer and every extern controller on one port.
set -euo pipefail

WORLD="${WORLD:-worlds/track.wbt}"
MODE="${MODE:-realtime}"
# w3d renders in the browser; mjpeg renders server-side and streams JPEG frames
# (useful for weak clients, and the only mode that can be captured headlessly).
STREAM_MODE="${STREAM_MODE:-w3d}"
DISPLAY_NUM="${DISPLAY_NUM:-99}"

# Start Xvfb directly rather than via xvfb-run. xvfb-run waits for Xvfb to
# signal readiness with SIGUSR1 to its parent; when the entrypoint is PID 1 in a
# container that signal is not delivered, and xvfb-run hangs forever with no
# output at all -- the container looks healthy and the port never opens.
Xvfb ":${DISPLAY_NUM}" -screen 0 1280x1024x24 -nolisten tcp &
export DISPLAY=":${DISPLAY_NUM}"

for _ in $(seq 1 100); do
  [ -e "/tmp/.X11-unix/X${DISPLAY_NUM}" ] && break
  sleep 0.1
done

# w3d streaming renders in the browser, so server-side rendering is redundant.
# Set RENDERING=off to skip it; verified to leave the viewer and the IR sensors
# working, and it is the main lever on simulator CPU.
RENDER_FLAG=()
if [ "${RENDERING:-on}" = "off" ]; then
  RENDER_FLAG=(--no-rendering)
fi

exec webots \
  "${RENDER_FLAG[@]}" \
  --stream="${STREAM_MODE:-w3d}" \
  --port=1234 \
  --batch \
  --stdout \
  --stderr \
  --mode="${MODE}" \
  "${WORLD}"
