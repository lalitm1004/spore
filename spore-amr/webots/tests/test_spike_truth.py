"""Parsing the supervisor's ground-truth log.

The scorer is the thing that says whether a run was good, so it has to be
right about the two failure modes it exists to catch: a heading error, and a
robot halting on a lost line.
"""

from tools.spike_truth import parse

LOG = """
supervisor-1  | label: bot_01  node 113 pass-through charging/PT/097  (200.0, 400.0) cm  region 3   fix error 4 mm  wheels -0.2 deg  heading -0.3 deg
supervisor-1  | label: bot_01  node 114 pass-through charging/PT/098  (400.0, 400.0) cm  region 3   fix error 9 mm  wheels +0.4 deg  heading +1.1 deg
bot_01-1  | bot_01: turning to 90 deg for node 114
supervisor-1  | label: bot_02  node 116 pass-through charging/PT/100  (800.0, 400.0) cm  region 3   fix error 2003 mm  wheels -90.0 deg  heading -90.0 deg
bot_02-1  | bot_02: turn timed out
bot_02-1  | bot_02: line lost for 2.0s, halted
""".strip().splitlines()


def test_reads_are_scored_per_robot():
    data = parse(LOG)

    assert data["fixes"]["bot_01"] == [4.0, 9.0]
    assert data["headings"]["bot_01"] == [0.3, 1.1]


def test_a_repeated_label_at_the_same_node_counts_once():
    """The supervisor reprints a robot's label whenever its text changes, so
    the same read can appear more than once. Counting lines would inflate the
    sample; only a change of node is a new read."""
    repeated = LOG + [LOG[1], LOG[1]]

    assert parse(repeated)["fixes"]["bot_01"] == [4.0, 9.0]


def test_returning_to_a_node_is_a_new_read():
    """A robot really can come back -- the lidar reflex reverses it to the
    previous marker -- and that read is not a duplicate."""
    revisit = LOG + [LOG[0]]

    assert parse(revisit)["fixes"]["bot_01"] == [4.0, 9.0, 4.0]


def test_turns_timeouts_and_halts_are_counted():
    data = parse(LOG)

    assert data["turns"]["bot_01"] == 1
    assert data["timeouts"]["bot_02"] == 1
    assert data["halts"]["bot_02"] == 1


def test_a_ninety_degree_heading_error_survives_into_the_report():
    """The exact symptom of an unseeded frame. It must not be averaged away."""
    assert max(parse(LOG)["headings"]["bot_02"]) == 90.0
