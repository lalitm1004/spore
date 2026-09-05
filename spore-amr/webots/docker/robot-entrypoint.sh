#!/usr/bin/env bash
# One robot = two processes, mirroring the Pi Zero + Arduino split.
# socat gives them a pair of linked pseudo-terminals, so each opens a serial
# device path exactly as it would on the real hardware.
set -euo pipefail

ROBOT_NAME="${ROBOT_NAME:?ROBOT_NAME is required}"
CONFIG="${CONFIG:-/project/config/${ROBOT_NAME}.yaml}"
TELEMETRY="${TELEMETRY:-/project/out/${ROBOT_NAME}.csv}"
SIM_HOST="${SIM_HOST:-sim}"
MISSION_DURATION="${MISSION_DURATION:-120}"

# The supervisor is a peer with no motion half: no serial link, no companion,
# no line to follow. It only watches and draws.
if [ "${ROLE:-robot}" = "supervisor" ]; then
  exec /usr/local/webots/webots-controller \
    --protocol=tcp \
    --ip-address="${SIM_HOST}" \
    --port=1234 \
    --robot-name=supervisor \
    /project/robot/supervisor.py \
    --duration "${MISSION_DURATION}"
fi

FIRMWARE_TTY="/tmp/${ROBOT_NAME}-firmware"
COMPANION_TTY="/tmp/${ROBOT_NAME}-companion"

socat -d pty,raw,echo=0,link="${FIRMWARE_TTY}" pty,raw,echo=0,link="${COMPANION_TTY}" &
for _ in $(seq 1 100); do
  [ -e "${FIRMWARE_TTY}" ] && [ -e "${COMPANION_TTY}" ] && break
  sleep 0.1
done

# One network layer for the whole fleet, reached over gRPC -- not one process
# per robot. It holds the global fleet state, reconciles outstanding commands
# against what robots report, and journals both, none of which is possible from
# inside a single robot's process.
python3 /project/robot/companion.py \
  --link "${COMPANION_TTY}" \
  --config "${CONFIG}" \
  --map "${WAREHOUSE_MAP:-/project/config/warehouse.json}" \
  --network "${NETWORK_ADDRESS:-network:50051}" \
  --mission-duration "${MISSION_DURATION}" &

exec /usr/local/webots/webots-controller \
  --protocol=tcp \
  --ip-address="${SIM_HOST}" \
  --port=1234 \
  --robot-name="${ROBOT_NAME}" \
  /project/robot/main.py \
  --config "${CONFIG}" \
  --telemetry "${TELEMETRY}" \
  --link "${FIRMWARE_TTY}"
