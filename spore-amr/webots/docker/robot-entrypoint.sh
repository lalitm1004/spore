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
    --duration "${MISSION_DURATION}" \
    --replay "${REPLAY:-/project/out/replay.csv}"
fi

FIRMWARE_TTY="/tmp/${ROBOT_NAME}-firmware"
COMPANION_TTY="/tmp/${ROBOT_NAME}-companion"

socat -d pty,raw,echo=0,link="${FIRMWARE_TTY}" pty,raw,echo=0,link="${COMPANION_TTY}" &
for _ in $(seq 1 100); do
  [ -e "${FIRMWARE_TTY}" ] && [ -e "${COMPANION_TTY}" ] && break
  sleep 0.1
done

# Each robot runs its own network-layer bot, and it runs in its own container:
# see the `bot_*` services in compose.fleet.yml. One shared service answering
# every robot would be a control plane wearing a hat, and per-robot means
# killing a single robot's coordinator is a real thing to demonstrate. The
# argument, against the fleet-wide service this branch merged past, is in
# spore-amr/network-layer/docs/boundary.md.
#
# It used to run *here*, beside the companion, because the robot link was a unix
# socket and a socket forces co-location. An address does not, and that is what
# lets the bot live on an image that can actually run it: this one is Ubuntu
# 22.04, so Python 3.10, and the planner needs 3.11+ for `enum.StrEnum`. Every
# robot in this fleet has been starting a bot that raised ImportError on its
# first import, which is why the network layer has never once run here.
#
# The companion waits for the bot rather than assuming it: gRPC's
# `wait_for_ready` handles a slow start, and a robot with nobody to ask sits
# still rather than inventing somewhere to go.
NETWORK_ADDRESS="${NETWORK_ADDRESS:-${ROBOT_NAME}-bot:50051}"
export PYTHONPATH="/network-layer:${PYTHONPATH:-}"

python3 /project/robot/companion.py \
  --link "${COMPANION_TTY}" \
  --config "${CONFIG}" \
  --map "${WAREHOUSE_MAP:-/project/config/warehouse.json}" \
  --network "${NETWORK_ADDRESS}" \
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
