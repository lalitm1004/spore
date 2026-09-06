"""Kill a section's leader mid-mission and watch the section carry on.

The claim this fleet makes is that no process is required for a robot to keep
driving. `PROTOCOL.md` §7 states it as two things a leader must not do -- hold
the only copy of any state, and be required for a safety-critical path -- and
`docs/boundary.md` says the whole decentralised design stands or falls on it.

This is that claim, tested by breaking it on purpose: find the bot that
actually won the election for a region with real followers, kill its container
while the fleet is mid-job, and measure three things.

    1. A new leader is elected, by whom and how fast.
    2. The followers keep driving -- their odometry keeps advancing across the
       kill, which is only possible because reservations go bot to bot and
       never through the leader.
    3. Nothing halts.

Run it against a live fleet. It kills a container: `./fleet.sh up` brings it
back, and the fleet re-elects again on its own either way.

Usage:
    uv run python -m tools.spike_leader_down
    uv run python -m tools.spike_leader_down --watch 90
"""

import argparse
import csv
import json
import pathlib
import subprocess
import sys
import time

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

COMPOSE = ["docker", "compose", "-f", "compose.yml", "-f", "compose.fleet.yml"]

#: Asked over AdminService, which every bot serves when ADMIN_ENABLED is set.
#: Read-only: it reports the bot's own picture and changes nothing.
STATE_PROBE = """cd /app && uv run python -c "
import grpc
from proto import fleet_pb2, fleet_pb2_grpc
ch = grpc.insecure_channel('localhost:50051')
st = fleet_pb2_grpc.AdminServiceStub(ch)
s = st.GetState(fleet_pb2.Empty(), metadata=(
    ('bot-id','9000'), ('region-id','0'), ('role','control')), timeout=5)
print('%d|%s|%d|%d' % (s.bot_id, s.role, s.region_id, s.leader_bot_id))
" """


def bots():
    return [p.stem for p in sorted((ROOT / "config").glob("bot_*.yaml"))]


def state_of(name):
    """(bot_id, role, region, leader) for one bot, or None if it cannot answer."""
    out = subprocess.run(
        COMPOSE + ["exec", "-T", "{}-bot".format(name), "sh", "-lc", STATE_PROBE],
        cwd=ROOT, capture_output=True, text=True, timeout=40)
    for line in reversed(out.stdout.strip().splitlines()):
        if line.count("|") == 3:
            bot_id, role, region, leader = line.split("|")
            return int(bot_id), role, int(region), int(leader)
    return None


def survey():
    found = {}
    for name in bots():
        try:
            state = state_of(name)
        except subprocess.TimeoutExpired:
            state = None
        if state is not None:
            found[name] = state
    return found


def sim_clock():
    """The simulation clock, from any robot's telemetry.

    Wall time is not what the replay is indexed by, so a kill recorded in wall
    seconds cannot be found in it. The supervisor is writing ground truth the
    whole time this runs, which means the kill is already in `out/replay.csv`
    -- the only thing missing was a timestamp in the replay's own units.
    """
    for name in bots():
        path = ROOT / "out" / "{}.csv".format(name)
        if not path.exists():
            continue
        with path.open() as handle:
            rows = list(csv.DictReader(handle))
        if rows:
            return float(rows[-1]["t"])
    return None


def distance(name):
    """How far this robot's odometry says it has driven, right now."""
    path = ROOT / "out" / "{}.csv".format(name)
    if not path.exists():
        return None
    with path.open() as handle:
        rows = list(csv.DictReader(handle))
    return float(rows[-1]["distance"]) if rows else None


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--watch", type=float, default=60.0,
                        help="seconds to watch the section after the kill")
    parser.add_argument("--out", type=pathlib.Path,
                        default=ROOT / "out" / "leader_down.json")
    args = parser.parse_args(argv)

    before = survey()
    if not before:
        print("no bot answered -- is the fleet up?")
        return 1

    print("before:")
    for name, (bot_id, role, region, leader) in sorted(before.items()):
        print("  {:8} bot-{} {:8} region {} leader bot-{}".format(
            name, bot_id, role, region, leader))

    # The interesting leader is one with followers. Killing a leader of one is
    # a demonstration of nothing: there is no section left to keep moving.
    sections = {}
    for name, (bot_id, role, region, _) in before.items():
        sections.setdefault(region, []).append((name, bot_id, role))
    target = None
    for region, members in sorted(sections.items()):
        leaders = [m for m in members if m[2] == "leader"]
        if leaders and len(members) > 1:
            target = (region, leaders[0][0], leaders[0][1],
                      [m[0] for m in members if m[2] != "leader"])
            break
    if target is None:
        print("\nno region has a leader with followers; nothing to demonstrate")
        return 1

    region, victim, victim_id, followers = target
    print("\nregion {} -- leader {} (bot-{}), followers {}".format(
        region, victim, victim_id, ", ".join(followers)))

    moved_before = {n: distance(n) for n in followers}
    killed_at_sim = sim_clock()
    killed_at = time.time()
    subprocess.run(COMPOSE + ["kill", "{}-bot".format(victim)],
                   cwd=ROOT, capture_output=True, timeout=60)
    print("\nkilled {}-bot at t=0".format(victim))

    # Watch for a follower to declare a new leader. Nothing is nudged: the
    # election runs off missed heartbeats, so this is just waiting.
    successor, elected_after = None, None
    while time.time() - killed_at < args.watch:
        time.sleep(5)
        for name in followers:
            try:
                state = state_of(name)
            except subprocess.TimeoutExpired:
                continue
            if state and state[3] not in (victim_id, 0) or (state and state[1] == "leader"):
                successor = state[3] if state[1] != "leader" else state[0]
                elected_after = time.time() - killed_at
                break
        if successor is not None:
            break

    moved_after = {n: distance(n) for n in followers}
    result = {
        "killed_at_sim_s": round(killed_at_sim, 1) if killed_at_sim else None,
        "region": region,
        "victim": victim, "victim_bot_id": victim_id,
        "followers": followers,
        "successor_bot_id": successor,
        "elected_after_s": round(elected_after, 1) if elected_after else None,
        "driven_m": {
            n: round((moved_after[n] or 0) - (moved_before[n] or 0), 2)
            for n in followers
        },
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(result, indent=2))

    print("\nafter:")
    if successor is not None:
        print("  new leader: bot-{}, elected {:.0f}s after the kill".format(
            successor, elected_after))
    else:
        print("  NO new leader within {:.0f}s".format(args.watch))
    for name in followers:
        print("  {:8} drove {:.2f} m across the kill".format(
            name, result["driven_m"][name]))
    kept_going = all(v > 0.1 for v in result["driven_m"].values())
    print("\n{}".format(
        "the section kept moving with its leader dead"
        if kept_going and successor is not None else
        "the section did NOT carry on -- this is the claim failing"))
    if killed_at_sim is not None:
        print("\nthe kill is at t={:.0f}s in out/replay.csv -- the supervisor was"
              " recording throughout, so it is already in the replay."
              .format(killed_at_sim))
        print("  ./fleet.sh replay3d      then scrub to t={:.0f}s".format(killed_at_sim))
    print("{}".format(args.out))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
