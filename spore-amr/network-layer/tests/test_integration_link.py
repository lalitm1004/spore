"""The seam: a real companion talking to a real network-layer bot.

This is the join the whole system rests on and it had no test at all. The two
halves were developed against hand-written twins of the same wire -- the robot
side had its `Query`/`Decision`, the network side had its own copy -- and
nothing ever checked one against the other. That is exactly the shape of gap
that lets a fleet run for its whole life with every bot believing it stands at
node 0.

WHAT runs here
    A real `bot.Bot`, serving `RobotNetwork.Session` on a real gRPC port. A real
    `Navigator` on a real lattice map. A real `Uplink`. Real `answer_junction`.

WHAT does not
    The camera, the firmware and the physics. Events are handed in as the
    firmware would emit them, so this is the whole path from `EVT MARKER` to a
    `CMD TURN` and back into the bot's own idea of where its robot is.

WHY it earns its place
    `test_uplink.py` checks our side of the conversion against a fake. The
    network layer's `test_proto_contract.py` checks the proto against the
    schemas. Neither notices if the two halves disagree about what the exchange
    *means* -- that a report updates position, that a question is answered, that
    a WAIT is honoured rather than driven through. Only running both does.
"""

import pathlib
import sys
from concurrent import futures

import grpc
import pytest

# The robot half is a sibling project. This test lives here rather than there
# because it needs both, and only one of the two can import the other: the
# webots project targets Python 3.10 (its container is Ubuntu 22.04) while the
# planner needs 3.11+ for `enum.StrEnum`. Its modules are 3.10 code and import
# cleanly on 3.12, so this direction works and the other does not.
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / "webots"))

from planning.decide import Decision, DecisionKind  # noqa: E402
from planning.robot_service import RobotNetworkServicer  # noqa: E402
from proto import robot_pb2_grpc  # noqa: E402
from robot.companion import answer_junction, report_obstacle  # noqa: E402
from robot.navigator import Navigator  # noqa: E402
from robot.uplink import Uplink  # noqa: E402
from tools.track.graph import lattice  # noqa: E402


class Event:
    """A firmware event, shaped as `robot/protocol.py` decodes one."""

    def __init__(self, name, **fields):
        self.name = name
        self.fields = fields


class Brain:
    """Stands in for the bot's run loop, not for the wire.

    Everything between it and the robot is real: the servicer, the stream, the
    proto, the uplink. What is faked is only the planner's *answer*, so a test
    can say "reply WAIT" and check the robot honours it.
    """

    def __init__(self, answer=None):
        self.answer = answer
        self.reported = []          # every RobotState the run loop would see
        self.obstructions = []      # every (seen_at, level)
        self.asked = []             # every Query the planner was handed

    def route(self, query):
        self.asked.append(query)
        if self.answer is not None:
            return self.answer(query)
        target = min(query.available) if query.available else 0
        return Decision(query_id=query.query_id, target_node_id=target)

    def report(self, state):
        self.reported.append(state)

    def obstruct(self, seen_at, level):
        self.obstructions.append((seen_at, level))


class State:
    """`bot.RobotState` without importing the bot module's threads."""

    def __init__(self, **fields):
        self.__dict__.update(fields)


@pytest.fixture
def link():
    """A served stream and a companion dialling it, on a real port."""
    brain = Brain()
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=4))
    servicer = RobotNetworkServicer(
        router=brain.route, report=brain.report, obstruct=brain.obstruct,
        state_factory=State, bot_id=0)
    robot_pb2_grpc.add_RobotNetworkServicer_to_server(servicer, server)
    port = server.add_insecure_port("127.0.0.1:0")
    server.start()

    uplink = Uplink(f"127.0.0.1:{port}", timeout_s=5.0)
    navigator = Navigator(lattice(rows=3, columns=3, spacing=2.0))
    try:
        yield brain, navigator, uplink, servicer
    finally:
        uplink.close()
        server.stop(grace=1)


# ---- the reporting half ------------------------------------------------------

def test_a_junction_question_tells_the_bot_where_its_robot_is(link):
    """The headline. Asking is also reporting, and that is the fix for a fleet
    whose bots never learned their own position."""
    brain, navigator, uplink, _ = link
    navigator.arrived(3)

    answer_junction(navigator, uplink, Event("MARKER", node=4, heading=0.0))

    assert brain.reported, "the bot heard nothing about where its robot is"
    assert brain.reported[-1].latest_node_id == 4


def test_an_obstacle_reaches_the_bot_with_the_node_it_was_seen_at(link):
    """The field whose absence made every obstruction test synthetic."""
    brain, navigator, uplink, _ = link
    navigator.arrived(4)

    report_obstacle(navigator, uplink, Event("OBSTACLE", state="HOLDING"))
    uplink.ask_pending = None
    # A report carries no answer, so nothing comes back to wait on. Ask a
    # question to push it through the stream and be sure it landed.
    answer_junction(navigator, uplink, Event("MARKER", node=4, heading=0.0))

    assert (4, 1.0) in brain.obstructions


def test_a_clear_report_gives_the_lane_back(link):
    brain, navigator, uplink, _ = link
    navigator.arrived(4)

    report_obstacle(navigator, uplink, Event("OBSTACLE", state="CLEAR"))
    answer_junction(navigator, uplink, Event("MARKER", node=4, heading=0.0))

    assert (0, 0.0) in brain.obstructions


# ---- the asking half ---------------------------------------------------------

def test_the_robot_is_told_a_node_and_turns_to_it(link):
    """End to end: a marker goes in, a bearing comes out, and the bearing is the
    map's -- derived by the robot from the node it was named, never sent."""
    brain, navigator, uplink, _ = link
    navigator.arrived(3)

    commands = answer_junction(navigator, uplink, Event("MARKER", node=4, heading=0.0))

    assert len(commands) == 1 and commands[0].name == "TURN"
    target = commands[0].fields["node"]
    assert target in navigator.graph.neighbours(4)
    # Rounded to five places on the way out; the point is that it is the map's
    # bearing and not something the network layer sent.
    assert commands[0].fields["bearing"] == pytest.approx(
        navigator.graph.bearing(4, target), abs=1e-4)
    assert brain.asked[-1].available, "the bot was asked without being told the exits"


def test_the_exits_the_bot_sees_are_the_ones_the_robot_can_reach(link):
    """The robot resolves them against the heading it arrived on, and the lane
    it came in on is not among them. Our map and its map can disagree; when they
    do, the robot is right about the floor it is standing on."""
    brain, navigator, uplink, _ = link
    navigator.arrived(3)          # arrived at 4 from the west

    answer_junction(navigator, uplink, Event("MARKER", node=4, heading=0.0))

    offered = set(brain.asked[-1].available)
    assert offered <= set(navigator.graph.neighbours(4))
    assert 3 not in offered, "it was offered the lane it just drove down"


def test_a_wait_is_held_rather_than_driven_through(link):
    """The answer the original protocol could not give. A robot told to hold
    must hold -- otherwise WAIT is indistinguishable from silence, which is the
    failure the whole kind field exists to prevent."""
    brain, navigator, uplink, _ = link
    brain.answer = lambda q: Decision(
        query_id=q.query_id, kind=DecisionKind.WAIT, hold_ms=800,
        because="holding against [1]")
    navigator.arrived(3)

    commands = answer_junction(navigator, uplink, Event("MARKER", node=4, heading=0.0))

    assert [c.name for c in commands] == ["HOLD"]
    assert commands[0].fields["ms"] == 800


def test_a_planner_that_raises_still_answers(link):
    """Never silence. Whatever went wrong upstream, the robot is standing at a
    node waiting, and it only asks again on reaching the next one."""
    def explode(query):
        raise RuntimeError("the planner fell over")

    brain, navigator, uplink, _ = link
    brain.answer = explode
    navigator.arrived(3)

    commands = answer_junction(navigator, uplink, Event("MARKER", node=4, heading=0.0))

    assert [c.name for c in commands] == ["HOLD"], \
        "a planner error must become a hold, not a silence"


def test_one_stream_serves_a_whole_shift(link):
    """A connection per junction would be pure overhead on hardware that has
    none to spare, so the companion opens one and keeps it."""
    brain, navigator, uplink, servicer = link

    for node in (4, 5, 4, 3, 4):
        navigator.arrived(node)
        answer_junction(navigator, uplink, Event("MARKER", node=node, heading=0.0))

    assert servicer.reports >= 5
    assert len({q.query_id for q in brain.asked}) == len(brain.asked), \
        "two junctions shared a query id, so an answer could be misapplied"


def test_the_bot_is_told_the_region_the_qr_said(link):
    """`region_id` off the floor is what drives migration, and it was being
    dropped: the companion read it and never passed it on."""
    brain, navigator, uplink, _ = link
    navigator.arrived(3)

    answer_junction(navigator, uplink, Event("MARKER", node=4, heading=0.0))

    assert brain.reported[-1].region_id == navigator.graph.nodes[4].region_id
