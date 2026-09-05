"""The robot half of the wire: what the companion sends, and what it makes of
what comes back.

`tests/test_network.py` used to live here and tested a newline-JSON codec. The
wire is protobuf now, so the codec is gone and with it that whole class of bug.
What did not go away is the *conversion* — `robot/uplink.py` is the one place on
this side that turns a `Query` into a message and a reply into a `Decision`, and
a mistake there is silent: a robot drives somewhere, just not where it was told.

The network layer's own `tests/test_proto_contract.py` guards the schemas. This
guards our reading of them.
"""

import pathlib
import sys

import pytest

from robot.network import PROCEED, REROUTE, WAIT, YIELD, Query
from robot.uplink import Uplink

# The network layer is a sibling project, mounted at /network-layer in the
# container and reached this way from a checkout. The proto is the contract
# between the two halves, so testing our side of it means importing theirs --
# a copy would be a copy that drifts.
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / "network-layer"))

pytest.importorskip("grpc", reason="the uplink needs grpcio")
pytest.importorskip("proto.robot_pb2", reason="needs the network layer alongside")

from proto import robot_pb2  # noqa: E402


class FakeStream:
    """Stands in for the reply half of a Session."""

    def __init__(self, *replies):
        self._replies = list(replies)

    def __iter__(self):
        return iter(self._replies)


def uplink_with(*replies) -> Uplink:
    link = Uplink("unused:0", bot_id=3)
    link._replies = FakeStream(*replies)
    return link


def sent(link):
    """The messages the uplink queued, in order."""
    out = []
    while not link._outbound.empty():
        message = link._outbound.get_nowait()
        if message is not None:
            out.append(message)
    return out


# ---- what goes up ------------------------------------------------------------

def test_a_report_carries_position_and_asks_nothing():
    """The half that never existed. Every one of these updates where the fleet
    thinks this robot is; none of them is a question."""
    link = uplink_with()
    link.report(node_id=113, region_id=3, battery=64.5)

    (message,) = sent(link)
    assert message.latest_node_id == 113
    assert message.region_id == 3
    assert message.telemetry.battery.percentage == pytest.approx(64.5)
    assert not message.available, "a report with exits is a question, not telemetry"


def test_a_question_carries_the_exits_the_robot_resolved():
    """`available` is what makes it a question, and the nodes in it are the
    robot's own answer to what is reachable from here."""
    link = uplink_with(robot_pb2.NetworkToRobot(target_node_id=114, query_id=1))
    link.ask(Query(query_id=1, node_id=113, region_id=3,
                   heading_rad=1.5, available=(114, 97)))

    (message,) = sent(link)
    assert list(message.available) == [114, 97]
    assert message.heading_rad == pytest.approx(1.5)
    assert message.query_id == 1


def test_a_reported_obstacle_carries_the_node_it_was_seen_at():
    """The field whose absence made every obstruction test synthetic."""
    link = uplink_with()
    link.report(node_id=455, region_id=3, obstacle_node=455)

    (message,) = sent(link)
    assert message.fault.warning.obstacle.current_node_id == 455


def test_a_clean_report_carries_no_fault_at_all():
    link = uplink_with()
    link.report(node_id=455, region_id=3)

    (message,) = sent(link)
    assert not message.HasField("fault")


# ---- what comes back ---------------------------------------------------------

@pytest.mark.parametrize(("wire", "expected"), [
    (robot_pb2.KIND_PROCEED, PROCEED),
    (robot_pb2.KIND_REROUTE, REROUTE),
    (robot_pb2.KIND_WAIT, WAIT),
    (robot_pb2.KIND_YIELD, YIELD),
    # Absent means take the lane: a robot reading only the node still behaves
    # correctly for every moving kind, which is what makes the field additive.
    (robot_pb2.KIND_UNSPECIFIED, PROCEED),
])
def test_every_kind_survives_the_wire(wire, expected):
    link = uplink_with(robot_pb2.NetworkToRobot(
        target_node_id=114, kind=wire, query_id=1))

    decision = link.ask(Query(query_id=1, node_id=113, available=(114,)))
    assert decision.kind == expected


def test_a_wait_carries_its_hold_and_names_no_lane():
    """The answer the original protocol had no way to give. A robot told
    nothing is indistinguishable from one whose network layer has died."""
    link = uplink_with(robot_pb2.NetworkToRobot(
        kind=robot_pb2.KIND_WAIT, hold_ms=800, because="holding against [1]",
        query_id=1))

    decision = link.ask(Query(query_id=1, node_id=113, available=(114,)))
    assert decision.is_wait
    assert decision.hold_ms == 800
    assert decision.because == "holding against [1]"
    assert decision.target_node_id == 0


def test_a_late_answer_to_the_previous_junction_is_discarded():
    """Two junctions can share a destination, so the id is the only thing that
    tells a fresh answer from a stale one — and a stale WAIT would stop a robot
    that had nothing wrong with it."""
    link = uplink_with(
        robot_pb2.NetworkToRobot(target_node_id=999, query_id=1),   # last node's
        robot_pb2.NetworkToRobot(target_node_id=114, query_id=2),
    )

    decision = link.ask(Query(query_id=2, node_id=113, available=(114,)))
    assert decision.target_node_id == 114


def test_no_network_layer_is_answered_with_none_rather_than_an_exception():
    """A robot with nobody to ask is a situation, not a crash. The caller
    decides what to do about it; this layer will not guess."""
    link = Uplink("127.0.0.1:1", timeout_s=0.2)
    assert link.ask(Query(query_id=1, node_id=113, available=(114,))) is None


# ---- the companion's use of it -----------------------------------------------

class Recorder:
    """A network layer that writes down what it was told."""

    def __init__(self):
        self.reports = []

    def report(self, node_id, region_id, **kwargs):
        self.reports.append((node_id, region_id, kwargs.get("obstacle_node")))

    def ask(self, query):
        return None


class Event:
    def __init__(self, name, **fields):
        self.name = name
        self.fields = fields


def navigator_at(node_id):
    from robot.navigator import Navigator
    from tools.track.graph import lattice

    navigator = Navigator(lattice(rows=3, columns=3, spacing=2.0))
    navigator.arrived(node_id)
    return navigator


def test_an_obstacle_is_reported_at_the_node_the_robot_reversed_to():
    """The reflex backs the robot to its last marker, so that is where it is
    when it reports -- and it is the node whose lane out is unusable."""
    from robot.companion import report_obstacle

    network = Recorder()
    navigator = navigator_at(4)
    report_obstacle(navigator, network, Event("OBSTACLE", state="HOLDING"))

    region = navigator.graph.nodes[4].region_id
    assert network.reports == [(4, region, 4)]


def test_a_cleared_obstacle_gives_the_lane_back():
    """`EVT OBSTACLE` fires on every change, clearing included. A report with
    no obstacle in it is how the node stops costing anything."""
    from robot.companion import report_obstacle

    network = Recorder()
    navigator = navigator_at(4)
    report_obstacle(navigator, network, Event("OBSTACLE", state="CLEAR"))

    region = navigator.graph.nodes[4].region_id
    # Zero, not None: present-and-zero is how the lane is given back. A report
    # that mentions no obstacle says nothing about obstacles, and almost every
    # report is one of those.
    assert network.reports == [(4, region, 0)]


def test_an_obstacle_before_the_first_marker_is_not_reported():
    """Nowhere to pin it to. A blockage at an unknown node would block a node
    chosen at random, which is worse than not knowing about it."""
    from robot.companion import report_obstacle
    from robot.navigator import Navigator
    from tools.track.graph import lattice

    network = Recorder()
    report_obstacle(Navigator(lattice(rows=3, columns=3, spacing=2.0)), network,
                    Event("OBSTACLE", state="HOLDING"))

    assert network.reports == []
