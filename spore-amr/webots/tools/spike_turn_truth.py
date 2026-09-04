"""Ground truth for tools/spike_turn.py.

Runs as the supervisor. Samples the robot's true heading every step and, once
the turn log is complete, joins it against what the robot believed. The robot
cannot do this itself -- it has no privileged sensor, which is the point.
"""

import argparse
import json
import math
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from controller import Supervisor  # noqa: E402


def true_heading(node):
    m = node.getOrientation()
    return math.atan2(m[3], m[0])


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot", default="BOT_01")
    parser.add_argument("--turns", type=pathlib.Path, default=ROOT / "out" / "turns.jsonl")
    parser.add_argument("--report", type=pathlib.Path,
                        default=ROOT / "out" / "turn_report.txt",
                        help="results go here as well as to stdout, so a "
                             "container that removes itself does not take "
                             "the measurement with it")
    parser.add_argument("--expect", type=int, default=10,
                        help="stop once the turn log has this many rows")
    args = parser.parse_args(argv)

    supervisor = Supervisor()
    timestep = int(supervisor.getBasicTimeStep())

    node = supervisor.getFromDef(args.robot)
    if node is None:
        print("no DEF {} in the world".format(args.robot), flush=True)
        return 2

    # Heading at every step, so a turn's start and end can be looked up by the
    # timestamps the robot recorded rather than sampled on a guess.
    #
    # Stopping is keyed to the log rather than to elapsed time: bot_01 is
    # synchronized, so when its controller exits the world stops advancing and
    # any sim-time deadline here would never arrive.
    history = []
    while supervisor.step(timestep) != -1:
        history.append((supervisor.getTime(), true_heading(node)))
        if args.turns.exists():
            rows = [l for l in args.turns.read_text().splitlines() if l.strip()]
            if len(rows) >= args.expect:
                break

    def heading_at(t):
        best = min(history, key=lambda row: abs(row[0] - t))
        return best[1]

    if not args.turns.exists():
        print("no turn log at {}".format(args.turns), flush=True)
        return 1

    rows = [json.loads(line) for line in args.turns.read_text().splitlines() if line.strip()]
    if not rows:
        print("turn log is empty", flush=True)
        return 1

    lines = [" asked   believed     true      error",
             " -----   --------   --------   --------"]

    errors = []
    for row in rows:
        believed = math.degrees(wrap(row["believed_end"] - row["believed_start"]))
        actual = math.degrees(wrap(heading_at(row["t"]) - heading_at(row["t_start"])))
        error = actual - believed
        errors.append(error)
        lines.append("{:+6.0f}   {:+8.2f}   {:+8.2f}   {:+8.2f}".format(
            row["requested_deg"], believed, actual, error))

    mean = sum(errors) / len(errors)
    worst = max(errors, key=abs)
    lines.append("")
    # A consistent ratio between what the wheels claim and what the body did is
    # a track-width error and can be calibrated out. Scatter around zero is
    # slip, and cannot.
    ratios = [
        math.degrees(wrap(heading_at(r["t"]) - heading_at(r["t_start"])))
        / math.degrees(wrap(r["believed_end"] - r["believed_start"]))
        for r in rows
        if abs(wrap(r["believed_end"] - r["believed_start"])) > 0.05
    ]
    lines.append("mean error {:+.2f} deg, worst {:+.2f} deg".format(mean, worst))
    if ratios:
        spread = max(ratios) - min(ratios)
        lines.append("true/believed ratio {:.4f} (spread {:.4f}) -> {}".format(
            sum(ratios) / len(ratios), spread,
            "systematic, calibratable" if spread < 0.05
            else "scattered: slip, not geometry"))

    report = "\n".join(lines)
    print(report, flush=True)
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(report + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
