#!/usr/bin/env bash
# Run the fleet. One place for the flags that are easy to get wrong.
#
#   ./fleet.sh up          bring the fleet up (regenerates if fleet.yaml changed)
#   ./fleet.sh down        tear it down, orphans and all
#   ./fleet.sh restart     down then up
#   ./fleet.sh status      what is running, and how far through the run
#   ./fleet.sh score       score the run against the supervisor's ground truth
#   ./fleet.sh logs [svc]  follow the logs (default: every robot, no STATUS spam)
#   ./fleet.sh goals       what the network layer has told each robot to do
#   ./fleet.sh journal     follow the network layer's durable state
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
VIEWER="http://localhost:1234/index.html"

export MODE="${MODE:-fast}"
export RENDERING="${RENDERING:-off}"
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
    rm -f out/*.csv out/*.status.json out/*.summary.json out/fleet.jsonl
    # --remove-orphans on the way up too: a previous run with more robots
    # leaves containers attached to a world that no longer has them, and a
    # synchronized robot whose controller never attaches freezes the simulator
    # for everyone else.
    "${COMPOSE[@]}" up -d --remove-orphans
    say "fleet up  --  MODE=$MODE RENDERING=$RENDERING MISSION_DURATION=${MISSION_DURATION}s"
    say "viewer:   $VIEWER"
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
    say "destination the network layer last set for each robot"
    for config in config/bot_*.yaml; do
      name="$(basename "$config" .yaml)"
      goal="$("${COMPOSE[@]}" logs "$name" 2>/dev/null \
              | grep -oE "'goal': [0-9]+" | tail -1 | grep -oE '[0-9]+' || true)"
      printf '  %-8s -> %s\n' "$name" "${goal:-none yet}"
    done
    ;;

  journal)
    # What the network layer has persisted: every status and every command.
    tail -f out/fleet.jsonl
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
