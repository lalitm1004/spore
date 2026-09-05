"""Score a run against the supervisor's ground truth.

The supervisor is the only thing in the system that knows where a robot really
is. It prints one line per marker read with the robot's own belief scored
against the truth, and this turns a run's worth of those lines into the four
numbers that say whether the fleet is working:

  fix error     how far the robot's believed position was from its real one
  heading       how far its believed heading was from its real one
  lost line     how long it spent with nothing under the IR array
  turns         how many completed, and how many timed out

Heading is the one to watch. A `TURN` carries an absolute bearing off the map
and the turn controller's only feedback is odometry, so a heading error is not
cosmetic -- it is the robot turning onto a lane nobody chose. Every robot on
the warehouse window was out by exactly its charging bay's bearing until the
frame was seeded, which is the kind of thing that looks like "bad line
following" from the outside.

Usage:
    docker compose -f compose.yml -f compose.fleet.yml logs \\
      | uv run python -m tools.spike_truth
"""

import argparse
import collections
import pathlib
import re
import statistics
import sys

LABEL = re.compile(
    r"label: (?P<name>bot_\d+)\s+node (?P<node>\d+).*?"
    r"fix error (?P<fix>[\d.]+) mm"
    r"(?:\s+wheels (?P<wheels>[-+][\d.]+) deg)?"
    r"(?:\s+heading (?P<heading>[-+][\d.]+) deg)?")
RELEASED = re.compile(r"(?P<name>bot_\d+): released after (?P<delay>[\d.]+)s")
TURNED = re.compile(r"(?P<name>bot_\d+): turning to (?P<bearing>-?\d+) deg")
TIMEOUT = re.compile(r"(?P<name>bot_\d+): turn timed out")
HALTED = re.compile(r"(?P<name>bot_\d+): line lost for [\d.]+s, halted")
RESUMED = re.compile(r"(?P<name>bot_\d+): line found, resuming")


def parse(lines):
    """Per-robot samples, keyed by robot name."""
    fixes = collections.defaultdict(list)
    headings = collections.defaultdict(list)
    turns = collections.Counter()
    timeouts = collections.Counter()
    halts = collections.Counter()
    resumes = collections.Counter()
    nodes = collections.defaultdict(list)
    released = {}

    for line in lines:
        match = LABEL.search(line)
        if match:
            name = match.group("name")
            # The same label is reprinted whenever any part of it changes, so
            # a read only counts once -- otherwise a robot sitting at a node
            # while its neighbour's error updates would be counted repeatedly.
            node = int(match.group("node"))
            if not nodes[name] or nodes[name][-1] != node:
                nodes[name].append(node)
                fixes[name].append(float(match.group("fix")))
                if match.group("heading") is not None:
                    headings[name].append(abs(float(match.group("heading"))))
            continue
        for pattern, counter in ((TURNED, turns), (TIMEOUT, timeouts),
                                 (HALTED, halts), (RESUMED, resumes)):
            match = pattern.search(line)
            if match:
                counter[match.group("name")] += 1
                break
        else:
            match = RELEASED.search(line)
            if match:
                released[match.group("name")] = float(match.group("delay"))

    return {"fixes": fixes, "headings": headings, "turns": turns,
            "timeouts": timeouts, "halts": halts, "resumes": resumes,
            "nodes": nodes, "released": released}


def median(values):
    return statistics.median(values) if values else float("nan")


def report(data, out=sys.stdout):
    names = sorted(set(data["fixes"]) | set(data["turns"]) | set(data["halts"]))
    if not names:
        print("no ground-truth lines found -- is the supervisor running?",
              file=out)
        return 1

    print("{:<8} {:>6} {:>10} {:>10} {:>9} {:>9} {:>6} {:>8} {:>6}".format(
        "robot", "reads", "fix med", "fix max", "head med", "head max",
        "turns", "timeouts", "halts"), file=out)
    print("-" * 78, file=out)

    for name in names:
        fixes = data["fixes"][name]
        headings = data["headings"][name]
        print("{:<8} {:>6} {:>8.0f}mm {:>8.0f}mm {:>7.1f}d {:>7.1f}d "
              "{:>6} {:>8} {:>6}".format(
                  name, len(fixes), median(fixes), max(fixes or [0]),
                  median(headings), max(headings or [0]),
                  data["turns"][name], data["timeouts"][name],
                  data["halts"][name]), file=out)

    every_fix = [v for values in data["fixes"].values() for v in values]
    every_heading = [v for values in data["headings"].values() for v in values]
    print("-" * 78, file=out)
    print("fleet    {:>6} {:>8.0f}mm {:>8.0f}mm {:>7.1f}d {:>7.1f}d "
          "{:>6} {:>8} {:>6}".format(
              len(every_fix), median(every_fix), max(every_fix or [0]),
              median(every_heading), max(every_heading or [0]),
              sum(data["turns"].values()), sum(data["timeouts"].values()),
              sum(data["halts"].values())), file=out)

    # A heading error is the one that silently ruins a run: the turn still
    # completes, it just completes onto the wrong lane.
    worst = max(every_heading or [0])
    if worst > 10.0:
        print("\nheading error above 10 deg: turns are landing on lanes "
              "nobody chose. Check odometry.start_theta against the world "
              "file, and that TURN carries a heading.", file=out)
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", nargs="?", type=pathlib.Path,
                        help="compose log file; reads stdin when omitted")
    args = parser.parse_args(argv)

    lines = (args.log.read_text(errors="replace").splitlines()
             if args.log else sys.stdin.read().splitlines())
    return report(parse(lines))


if __name__ == "__main__":
    raise SystemExit(main())
