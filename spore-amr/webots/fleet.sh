#!/usr/bin/env bash
# Run the fleet. One place for the flags that are easy to get wrong.
#
#   ./fleet.sh up          bring the fleet up (regenerates if fleet.yaml changed)
#   ./fleet.sh chunk       cut a small piece of the map and run it, so the
#                          browser viewer works (the whole warehouse is 924 MB
#                          of marker texture; a browser cannot expand that)
#   ./fleet.sh view        server-side rendering, for watching the whole map
#   ./fleet.sh down        tear it down, orphans and all
#   ./fleet.sh restart     down then up
#   ./fleet.sh status      what is running, and how far through the run
#   ./fleet.sh score       score the run against the supervisor's ground truth
#   ./fleet.sh logs [svc]  follow the logs (default: every robot, no STATUS spam)
#   ./fleet.sh where       where the fleet thinks its robots are (roster + claims)
#   ./fleet.sh dump [n]    print the last n log lines once and exit
#   ./fleet.sh order A B   place a cargo order: pickup node A, dropoff node B
#   ./fleet.sh goals       what the network layer has told each robot to do
#   ./fleet.sh robots      per-robot: distance, state, and whether it is moving
#   ./fleet.sh fleet       leaders, jobs and claims -- the coordination layer
#   ./fleet.sh replay      build a flat replay of the run just recorded
#   ./fleet.sh replay3d    build a 3D replay you can orbit, fly and follow
#   ./fleet.sh gen         regenerate the world from fleet.yaml
#   ./fleet.sh build       rebuild the controller image
#
# Environment, with the defaults that are almost always right:
#   MODE=fast             run as fast as the host allows (realtime paces it)
#   RENDERING=off         server-side rendering off; the viewer renders in the
#                         browser anyway, and this is the main lever on CPU
#   MISSION_DURATION=3600 simulation seconds, not wall seconds
set -euo pipefail

# Every path here is relative to this directory, because the volume mount is
# `./:/project` -- `./` is the *shell's* directory, not the compose file's, so
# running from anywhere else silently mounts the wrong tree. Anchoring to the
# script's own location makes `./fleet.sh` work from wherever you happen to be.
cd "$(dirname "$(readlink -f "$0")")"

COMPOSE=(docker compose -f compose.yml -f compose.fleet.yml)
# Hand the simulator the host GPU only where there is one to hand it. Compose
# fails to create a container that names a missing device, so this cannot be
# unconditional -- see compose.gpu.yml.
if [ -d /dev/dri ]; then
  COMPOSE+=(-f compose.gpu.yml)
fi

# The network-layer containers, one per robot: `bot_01-bot` and friends. Their
# logs are the coordination layer; the robot services' logs are the driving.
bot_services() {
  for config in config/bot_*.yaml; do
    printf '%s-bot ' "$(basename "$config" .yaml)"
  done
}
VIEWER="http://localhost:1234/index.html"

export MODE="${MODE:-fast}"
export RENDERING="${RENDERING:-off}"
export STREAM_MODE="${STREAM_MODE:-w3d}"
export MISSION_DURATION="${MISSION_DURATION:-3600}"

say() { printf '\033[1m%s\033[0m\n' "$*"; }

# The header comment above is the help text. Read it up to the first line that
# is not a comment, so editing the block cannot drift out of sync with a
# hardcoded line range.
usage() {
  awk 'NR > 1 { if (!/^#/) exit; sub(/^# ?/, ""); print }' "$0"
}

generate() {
  say "regenerating the world from fleet.yaml"
  uv run python -m tools.gen_fleet
}

# fleet.yaml is the single source of truth, and every output is derived from it.
# Editing it and forgetting to regenerate leaves the world, the compose file and
# the per-robot configs disagreeing, which looks like a bug in the robots.
generate_if_stale() {
  if [ ! -f compose.fleet.yml ] || [ fleet.yaml -nt compose.fleet.yml ]; then
    say "fleet.yaml is newer than the generated files"
    generate
  fi
}

case "${1:-help}" in
  up|start)
    generate_if_stale
    # Clear the previous run's artefacts. `out/*.status.json` is how each robot
    # tells the supervisor what it last read, and a leftover file is read as
    # this run's first marker: the supervisor scores a stale node against a
    # robot standing somewhere else entirely and reports metres of localisation
    # error that never happened. Measured once at 18 m, which looks exactly
    # like the fleet being broken.
    rm -f out/*.csv out/*.status.json out/*.summary.json out/fleet.jsonl \
          out/replay.html
    # --remove-orphans on the way up too: a previous run with more robots
    # leaves containers attached to a world that no longer has them, and a
    # synchronized robot whose controller never attaches freezes the simulator
    # for everyone else.
    "${COMPOSE[@]}" up -d --remove-orphans
    say "fleet up  --  MODE=$MODE RENDERING=$RENDERING STREAM_MODE=$STREAM_MODE MISSION_DURATION=${MISSION_DURATION}s"
    say "viewer:   $VIEWER"
    say "orders:   http://localhost:8000/   (or ./fleet.sh order <pickup> <dropoff>)"
    ;;

  chunk)
    # The streaming viewer renders in the *browser*, so it has to expand every
    # texture in the world. The whole warehouse is 881 marker tiles -- 924 MB
    # -- and no tab will do that. A 32 x 16 m piece is 83 tiles and 101 MB,
    # which it will.
    #
    # Everything else still comes from fleet.yaml. This is the same fleet on a
    # smaller map, not a different configuration.
    shift
    window="${1:-8200,600,32,16}"
    say "cutting a chunk: $window  (x_cm,y_cm,width_m,height_m)"
    uv run python -m tools.gen_fleet --window "$window"
    "$0" up
    say ""
    say "Set Mode to W3D in the viewer -- this is small enough for it now."
    ;;

  view)
    # W3D streaming renders in the *browser*, so the browser downloads every
    # texture in the world: 83 marker tiles at 1024x1024 plus an 8192x4096
    # floor is 482 MB once decompressed, and a tab typically gives up part way
    # through -- the loader sits at "Downloading assets: 97%" for ever. Those
    # tiles are 1024x1024 so the simulator's camera never resamples a QR
    # module, which is right for the robot and hopeless for a viewer.
    #
    # mjpeg renders server-side and streams frames instead, so the browser
    # downloads no textures at all. It costs the simulator CPU that
    # RENDERING=off saves, which is the honest trade for being able to watch:
    # use this to look at the fleet, and plain `up` to measure it.
    RENDERING=on STREAM_MODE=mjpeg "$0" up
    say ""
    say "In the viewer, set Mode to MJPEG before connecting."
    say "W3D renders in the browser and pulls every texture across: 83 marker"
    say "tiles plus the floor is 482 MB decompressed, and the tab stalls at"
    say "\"Downloading assets: 97%\". MJPEG renders here and sends frames."
    ;;

  down|stop)
    "${COMPOSE[@]}" down --remove-orphans
    ;;

  restart)
    "${COMPOSE[@]}" down --remove-orphans
    exec "$0" up
    ;;

  status)
    "${COMPOSE[@]}" ps --format 'table {{.Name}}\t{{.Status}}'
    if [ -f out/bot_01.csv ]; then
      printf '\nsimulation clock: %s s of %s\n' \
        "$(tail -1 out/bot_01.csv | cut -d, -f1)" "$MISSION_DURATION"
    fi
    printf 'turns executed:   %s\n' \
      "$("${COMPOSE[@]}" logs 2>/dev/null | grep -c 'turning to' || true)"
    ;;

  score)
    # The supervisor is the only thing that knows where the robots really are.
    "${COMPOSE[@]}" logs 2>&1 | uv run python -m tools.spike_truth
    ;;

  where)
    # Where the *fleet* thinks its robots are, which is a different question
    # from where they are: this is the roster every bot holds, built from QR
    # reads that travelled the wire. `robots` reads telemetry and answers
    # "which one is stuck"; this answers "does the coordination layer know".
    #
    # Asked of one bot over its own AdminService, from inside its container --
    # the bot containers publish no ports, and nothing outside the fleet needs
    # to reach them. `--json` for anything reading this rather than looking at
    # it (the webots test tier).
    shift
    fmt="${1:-text}"
    docker exec "webots-$(basename config/bot_01.yaml .yaml)-bot-1" sh -c \
      "cd /app && .venv/bin/python -c \"
import grpc, json, sys
from proto import fleet_pb2, fleet_pb2_grpc
from bus.policy import rpc_metadata
s = fleet_pb2_grpc.AdminServiceStub(grpc.insecure_channel('localhost:50051'))
st = s.GetState(fleet_pb2.Empty(), timeout=5, metadata=rpc_metadata(999,0,'admin'))
state = {
    'bot_id': st.bot_id, 'role': st.role, 'region': st.region_id,
    'roster': [{'bot_id': p.bot_id, 'node': p.latest_node_id,
                'trail': list(p.node_trail)} for p in st.roster],
    'claims': [[c.bot_id, c.node_id] for c in st.reservations],
}
if '${fmt}' == 'json':
    print(json.dumps(state))
else:
    print('asked bot-%d (%s of region %d)' % (state['bot_id'], state['role'], state['region']))
    for p in sorted(state['roster'], key=lambda p: p['bot_id']):
        print('   bot-%-2d node %-6s trail %s' % (p['bot_id'], p['node'] or '-', p['trail']))
    print('claims:', state['claims'] or 'none')
\""
    ;;

  order)
    # Place a cargo order on the running fleet, through the control plane's own
    # HTTP surface -- the same POST its web form makes, so there is one client.
    shift
    [ $# -eq 2 ] || { say "usage: ./fleet.sh order <pickup_node> <dropoff_node>"; exit 2; }
    if curl -sf -X POST "http://localhost:8000/orders" \
         -F "pickup_node=$1" -F "dropoff_node=$2" >/dev/null; then
      say "order placed: node $1 -> node $2"
    else
      say "the control plane did not accept it -- is the fleet up, and are both nodes on this map?"
      exit 1
    fi
    ;;

  dump)
    # Logs once and exit, where `logs` follows. For anything reading the run
    # rather than watching it -- the webots test tier, mostly -- so it does not
    # have to rebuild the compose invocation and get the GPU overlay wrong.
    shift
    "${COMPOSE[@]}" logs --tail="${1:-2000}" 2>&1
    ;;

  logs)
    if [ $# -gt 1 ]; then
      shift
      "${COMPOSE[@]}" logs -f "$@"
    else
      # STATUS is a once-a-second heartbeat and drowns everything else.
      "${COMPOSE[@]}" logs -f | grep --line-buffered -v STATUS
    fi
    ;;

  goals)
    say "node each robot was last told to head for, and why"
    for config in config/bot_*.yaml; do
      name="$(basename "$config" .yaml)"
      line="$("${COMPOSE[@]}" logs "$name" 2>/dev/null \
              | grep -oE "'node': [0-9]+" | tail -1 | grep -oE '[0-9]+' || true)"
      printf '  %-8s -> %s\n' "$name" "${line:-nothing yet}"
    done
    ;;

  fleet)
    # The coordination layer, which is a different question from where the
    # robots are: who leads each region, who holds a job, who is claiming what.
    say "the network layer: leaders, jobs and claims"
    "${COMPOSE[@]}" logs --tail=400 $(bot_services) 2>/dev/null \
      | grep -E "became leader|accepted job|assigned|giving way|obstacle|migrat" \
      || echo "  nothing yet"
    ;;

  robots)
    # A robot jammed behind another logs nothing at all -- it is following its
    # line perfectly, just not going anywhere -- so the log tells you less than
    # the telemetry does. This is the view that answers "which one is stuck".
    uv run python - <<'PYTHON'
import csv, glob, os

print("%-9s %8s %9s %-11s %-9s %-6s %s" % (
    "robot", "t", "distance", "crossing", "obstacle", "lost", "moved recently"))
print("-" * 74)
for path in sorted(glob.glob("out/bot_*.csv")):
    name = os.path.basename(path).split(".")[0]
    try:
        rows = list(csv.DictReader(open(path)))
    except OSError:
        rows = []
    if not rows:
        print("%-9s %8s" % (name, "no telemetry yet"))
        continue
    last = rows[-1]
    # ~10 s of history at 62.5 Hz: enough to tell stopped from slow.
    window = rows[-625:] if len(rows) > 625 else rows
    travelled = float(last["distance"]) - float(window[0]["distance"])
    print("%-9s %8.1f %8.2fm %-11s %-9s %-6s %s" % (
        name, float(last["t"]), float(last["distance"]),
        last["crossing"], last["obstacle"], last["lost"],
        "yes  %.2f m" % travelled if travelled > 0.01 else "NO   held/stuck"))
PYTHON
    ;;

  replay)
    # Draws the run rather than replaying the world, so it needs none of the
    # textures that stop a browser rendering the live stream. Ground truth
    # from the supervisor: what you watch is what actually happened.
    shift
    uv run python -m tools.make_replay "$@"
    ;;

  replay3d)
    # The same recording, lit and in three dimensions: orbit, zoom, follow one
    # robot. Everything is geometry generated from warehouse.json, so the whole
    # 881-node warehouse is about a megabyte -- against the 924 MB of marker
    # texture Webots' own viewer would ask the browser to expand for the same
    # map. It is a recording, so how slowly the run went does not matter either.
    shift
    uv run python -m tools.make_replay3d "$@"
    ;;

  journal)
    # There is no journal. The fleet has no central service to keep one -- see
    # spore-amr/network-layer/docs/boundary.md for why that is the design and
    # what it costs. Ask a bot instead: each one holds its own whole picture.
    say "no fleet journal: state lives in the bots. Try 'fleet' or 'robots'."
    exit 1
    ;;

  gen)
    generate
    ;;

  build)
    # docker/robot-entrypoint.sh is COPY'd into the image, so editing it changes
    # nothing until this runs. Python is volume-mounted and needs no rebuild.
    say "rebuilding the controller image"
    docker build -f docker/controller.Dockerfile -t sih2026/controller:dev .
    ;;

  help|--help|-h)
    usage
    ;;

  *)
    printf 'unknown command: %s\n\n' "$1" >&2
    usage >&2
    exit 2
    ;;
esac
