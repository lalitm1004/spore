"""Two robots converging on one aisle, and the rule that unpicks it.

The scenario the fleet is built to survive and the one that is hardest to catch
in a general run: two robots entering the same single-file lane from opposite
ends. On a painted line there is no passing, so if both commit they are stuck
there for the rest of the shift.

This stages that meeting on the real map and writes it as an ordinary replay
CSV, which `tools/make_replay3d.py` renders like any other run.

**Be clear about what is real here and what is not.** The map is real, the
aisle is the longest genuine dead-straight run of degree-2 nodes on it, and
*who yields is decided by the fleet's own ordering* -- the ranks are read
straight out of `election/priority.py`, so a robot carrying cargo keeps the
lane and one merely heading to a pickup backs out, exactly as
`decide.outranked_by` would rule. What is scripted is the motion: the two
paths are laid out from that decision rather than driven through the planner,
because the planner needs Python 3.11 (`StrEnum`) and this project runs on
3.10.

So this is a faithful picture of the rule and not a run of it. The run of it
is `tests/test_routing.py`, where the real `Bot._route` is asked and both
halves are asserted -- the outranked robot gives way, and the one with right
of way keeps going.

Usage:
    uv run python -m tools.spike_headon             # -> out/headon.csv
    uv run python -m tools.spike_headon --render    # and the 3D replay
"""

import argparse
import json
import math
import pathlib
import subprocess
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT.parent / "network-layer"))

#: Sampled at the rate `robot/supervisor.py --replay` uses, so the output is
#: indistinguishable from a recorded run and the same tools read it.
SAMPLE_HZ = 10.0
SPEED_MS = 0.36          # unladen cruise, 18 rad/s on 20 mm wheels
HANDLING_S = 0.0


def _yield_ranks():
    """`YIELD_CARRYING` and `YIELD_TO_PICKUP`, read from the network layer.

    Parsed rather than imported, and parsed rather than copied: a copy is a
    number that silently stops matching the fleet's, and this demo is only
    worth anything if the ordering it shows is the ordering the fleet uses.
    """
    source = (ROOT.parent / "network-layer" / "election" / "priority.py").read_text()
    for line in source.splitlines():
        if line.startswith("YIELD_FREE, YIELD_TO_PICKUP, YIELD_CARRYING"):
            values = [int(v) for v in line.split("=")[1].split(",")]
            return values[2], values[1]        # carrying, heading-to-pickup
    raise SystemExit("could not read the yield ranks from election/priority.py")


def longest_aisle(nodes, edges):
    """The longest run of degree-2 nodes: somewhere with no way round.

    A head-on in a junction is not interesting -- either robot can step aside.
    The conflict only bites where there is nowhere to go, so the demo has to
    happen somewhere there is nowhere to go.
    """
    adjacency = {}
    for a, b in edges:
        adjacency.setdefault(a, []).append(b)
        adjacency.setdefault(b, []).append(a)

    best = []
    for start in nodes:
        if len(adjacency.get(start, ())) != 2:
            continue
        for first in adjacency[start]:
            run, previous, current = [start], start, first
            while len(adjacency.get(current, ())) == 2 and current not in run:
                run.append(current)
                nxt = [v for v in adjacency[current] if v != previous][0]
                previous, current = current, nxt
            run.append(current)
            if len(run) > len(best):
                best = run
    return best, adjacency


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map", type=pathlib.Path,
                        default=ROOT / "config" / "warehouse.json")
    parser.add_argument("--out", type=pathlib.Path,
                        default=ROOT / "out" / "headon.csv")
    parser.add_argument("--render", action="store_true",
                        help="also build the 3D replay")
    args = parser.parse_args(argv)

    document = json.loads(args.map.read_text())
    width = float(document["dimensions"]["width"]) / 100.0
    height = float(document["dimensions"]["height"]) / 100.0
    position = {
        int(n["id"]): (float(n["position"]["x"]) / 100.0 - width / 2,
                       float(n["position"]["y"]) / 100.0 - height / 2)
        for n in document["nodes"]
    }
    edges = [(int(e["a"]), int(e["b"])) for e in document["edges"]]
    aisle, adjacency = longest_aisle(position, edges)
    if len(aisle) < 6:
        print("this map has no aisle long enough to stage a head-on in")
        return 1

    # One from each end, walking toward each other. The rule is asymmetric on
    # purpose -- both refusing would be a livelock rather than a fix -- so one
    # of them backs out and the other comes through.
    west, east = aisle[:], list(reversed(aisle))
    print("aisle: {} nodes, {} .. {}".format(len(aisle), aisle[0], aisle[-1]))

    # Right of way, decided exactly as the fleet decides it. The numbers are
    # read out of `election/priority.py` rather than imported: this project runs
    # on Python 3.10 and the network layer needs 3.11 for `StrEnum`, so
    # importing its planner here would not load at all. Reading the source keeps
    # the two from drifting without taking the dependency.
    rank_w, rank_e = _yield_ranks()
    winner_is_west = (-rank_w, 1) < (-rank_e, 2)
    print("bot_01 rank {} (carrying), bot_02 rank {} (to pickup) -> {} has right of way"
          .format(rank_w, rank_e, "bot_01" if winner_is_west else "bot_02"))

    # Three phases, because that is what the rule produces: both approach, the
    # outranked one backs out while the other holds, then the one with right of
    # way comes through. Built explicitly rather than by letting two independent
    # walks run out -- they have different lengths, and two timelines that
    # merely end when they end is how the first attempt finished with both
    # robots on the same square.
    meet = len(aisle) // 2
    yielder_route, holder_route = (east, west) if winner_is_west else (west, east)
    yielder = "bot_02" if winner_is_west else "bot_01"
    holder = "bot_01" if winner_is_west else "bot_02"

    # Where the loser goes: back out of the aisle and one node past its end,
    # onto a junction, which is somewhere the other robot can actually pass.
    beyond = [v for v in adjacency.get(yielder_route[0], ()) if v not in aisle]
    retreat = list(reversed(yielder_route[:meet - 1])) + beyond[:1]

    def leg(route, label, start_t):
        rows, tt = [], start_t
        for i in range(len(route) - 1):
            a_, b_ = position[route[i]], position[route[i + 1]]
            span = math.hypot(b_[0] - a_[0], b_[1] - a_[1])
            heading = math.atan2(b_[1] - a_[1], b_[0] - a_[0])
            steps = max(1, int(span / SPEED_MS * SAMPLE_HZ))
            for step in range(steps):
                f = step / steps
                rows.append((tt, label, a_[0] + (b_[0] - a_[0]) * f,
                             a_[1] + (b_[1] - a_[1]) * f, heading))
                tt += 1.0 / SAMPLE_HZ
        return rows, tt

    def still(label, at, heading, start_t, seconds):
        rows, tt = [], start_t
        for _ in range(max(0, int(seconds * SAMPLE_HZ))):
            rows.append((tt, label, at[0], at[1], heading))
            tt += 1.0 / SAMPLE_HZ
        return rows, tt

    frames = []

    # Phase 1 -- both approach, and stop about a lane apart. The near miss.
    approach_y, ty_end = leg(yielder_route[:meet - 1], yielder, 0.0)
    # One node further than the yielder, so they end a single lane apart --
    # close enough that the reflex would fire, which is the point of showing it.
    approach_h, th_end = leg(holder_route[:meet + 2], holder, 0.0)
    frames += approach_y + approach_h
    ly, lh = approach_y[-1], approach_h[-1]
    print("closest approach: {:.2f} m -- the near miss".format(
        math.hypot(ly[2] - lh[2], ly[3] - lh[3])))

    # Phase 2 -- the outranked robot backs out; the other stands and waits.
    retreat_rows, t2 = leg(retreat, yielder, ty_end)
    hold_rows, _ = still(holder, (lh[2], lh[3]), lh[4], th_end, t2 - th_end)
    frames += retreat_rows + hold_rows

    # Phase 3 -- right of way comes through; the yielder waits it out.
    through, t3 = leg(holder_route[meet - 2:], holder, t2)
    ry = retreat_rows[-1]
    frames += through + still(yielder, (ry[2], ry[3]), ry[4], t2, t3 - t2)[0]

    args.out.parent.mkdir(parents=True, exist_ok=True)
    rows = sorted(frames, key=lambda r: (r[0], r[1]))
    with args.out.open("w") as handle:
        handle.write("t,robot,x,y,theta\n")
        for t, label, x, y, th in rows:
            handle.write("{:.3f},{},{:.4f},{:.4f},{:.4f}\n".format(t, label, x, y, th))
    print("{}  --  2 robots, {} rows, {:.0f} s".format(
        args.out, len(rows), rows[-1][0]))

    # A map of just this aisle and what touches it. The replay frames its
    # camera on the nodes it is given, so handing it the whole warehouse would
    # open on the whole warehouse and the near-miss would be four pixels in the
    # middle of it. Trimming the map *is* the zoom.
    keep = set(aisle)
    for node in list(keep):
        keep.update(adjacency.get(node, ()))
    trimmed = {
        "dimensions": document["dimensions"],
        "nodes": [n for n in document["nodes"] if int(n["id"]) in keep],
        "edges": [e for e in document["edges"]
                  if int(e["a"]) in keep and int(e["b"]) in keep],
    }
    focus = args.out.with_name("headon-map.json")
    focus.write_text(json.dumps(trimmed))
    print("{}  --  {} nodes around the aisle".format(focus, len(trimmed["nodes"])))

    if args.render:
        target = args.out.with_name("headon3d.html")
        subprocess.run([
            sys.executable, "-m", "tools.make_replay3d",
            "--replay", str(args.out), "--map", str(focus),
            "--out", str(target), "--robot-scale", "1.7",
        ], cwd=ROOT, check=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
