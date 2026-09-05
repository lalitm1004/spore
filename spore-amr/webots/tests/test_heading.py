"""The firmware's heading frame.

A `TURN` carries an *absolute* bearing off the map, and the turn controller's
only feedback is the odometry heading. So the two have to share a frame. They
did not: odometry started at theta=0 whatever way the robot was actually
placed, and on the warehouse window every charging bay faces +/-90 degrees.
Measured against the supervisor, all ten robots were out by exactly their
spawn bearing, which put every turn on the wrong lane.
"""

import math

import pytest

from robot.companion import answer_junction
from robot.config import ControllerConfig
from robot.navigator import Navigator
from robot.network import Decision
from robot.odometry import Odometry, Pose
from tools.track.graph import lattice

SENSORS = {"offsets": [0.02, 0.0, -0.02], "white_ref": 1000, "black_ref": 200}


class FixedGoal:
    """Stands in for the network layer: it wants the robot at one node.

    A destination, not a direction -- nothing on the wire says left or right,
    so working out that node 5 is to the left is the robot's own job. That is
    the property these tests are about, and it does not depend on which
    transport carries the answer.
    """

    def __init__(self, goal):
        self.goal = goal
        self.asked = []

    def ask(self, query):
        self.asked.append(query)
        return Decision(query_id=query.query_id, target_node_id=self.goal)


class Event:
    def __init__(self, **fields):
        self.name = "MARKER"
        self.fields = fields


# --------------------------------------------------- seeding the boot frame --

def test_start_heading_defaults_to_zero():
    config = ControllerConfig.from_dict(
        {"name": "bot_01", "sensors": SENSORS, "control": {}})

    assert config.odometry.start_theta == 0.0


def test_start_heading_is_read_from_the_generated_document():
    config = ControllerConfig.from_dict({
        "name": "bot_01", "sensors": SENSORS, "control": {},
        "odometry": {"start_theta": math.pi / 2},
    })

    assert config.odometry.start_theta == pytest.approx(math.pi / 2)


def test_odometry_can_start_on_a_heading_and_integrates_from_it():
    """A robot parked in a bay is not facing along +x just because it booted.
    Driving straight from a seeded heading must move along that heading."""
    odometry = Odometry(wheel_radius=0.02, track_width=0.0994)
    odometry.reset(Pose(theta=math.pi / 2))

    odometry.update(0.0, 0.0)          # establish the encoder reference
    odometry.update(1.0, 1.0)          # both wheels forward: straight ahead

    assert odometry.pose.theta == pytest.approx(math.pi / 2)
    assert odometry.pose.x == pytest.approx(0.0, abs=1e-9)
    assert odometry.pose.y == pytest.approx(0.02)   # 1 rad * 0.02 m


# ------------------------------------------- correcting the frame at a node --

def graph_and_navigator():
    # A 3x3 lattice at 2 m spacing; ids run row-major, so 4 is the centre.
    graph = lattice(rows=3, columns=3, spacing=2.0)
    return graph, Navigator(graph)


def test_turn_command_carries_the_exact_arrival_heading():
    """The companion knows the lane the robot came in on -- previous node to
    this one -- so it can hand the firmware a heading that owes nothing to
    odometry. Without it the firmware turns in a drifted frame."""
    graph, navigator = graph_and_navigator()
    navigator.arrived(3)                       # came from the node west of centre
    router = FixedGoal(goal=5)

    commands = answer_junction(navigator, router,
                               Event(node=4, heading=-1.2))  # badly drifted

    assert len(commands) == 1
    assert commands[0].fields["heading"] == pytest.approx(
        graph.bearing(3, 4))


def test_arrival_heading_is_absent_on_the_very_first_marker():
    """Nothing to derive it from yet, so the firmware must keep its own frame
    -- which is why the spawn heading has to be seeded at boot."""
    _, navigator = graph_and_navigator()
    router = FixedGoal(goal=5)

    commands = answer_junction(navigator, router, Event(node=4, heading=0.0))

    assert "heading" not in commands[0].fields


def test_the_turn_bearing_is_still_the_map_bearing():
    """The heading correction must not disturb the target it turns to."""
    graph, navigator = graph_and_navigator()
    navigator.arrived(3)
    router = FixedGoal(goal=5)

    commands = answer_junction(navigator, router, Event(node=4, heading=-1.2))

    assert commands[0].fields["bearing"] == pytest.approx(graph.bearing(4, 5))
