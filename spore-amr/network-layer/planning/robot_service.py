"""The robot link: one long-lived stream between a robot and its own bot.

WHAT
    `RobotNetworkServicer` — serves `spore.network.v1.RobotNetwork/Session`
    (`proto/robot.proto`). Every message a robot sends becomes a `RobotState`
    the run loop reads; the ones that ask a question also get an answer.

WHERE
    Registered on the bot's own gRPC server (`bot.Bot._start_grpc_server`). The
    client is `spore-amr/webots/robot/companion.py`, acting for its robot, and
    it dials **this robot's bot** rather than a service for the fleet — see
    `docs/boundary.md`.

WHY — two conversations, one wire
    A robot says two different kinds of thing and they were never both carried.

    It **reports**: where it is, how charged it is, what it is carrying, what is
    wrong. That is the canonical location interface, and for the whole life of
    this fleet it did not exist. `latest_node_id` had exactly one writer, fed by
    a queue nothing in production ever pushed to, so every bot believed it stood
    at node 0 for ever — and because 0 is a legal-looking node, everything
    downstream failed silently rather than loudly. See `docs/location.md`.

    It **asks**: I am at this node, these are the ways out, which one? That is a
    blocking question. The robot stops and waits, and if it hears nothing it
    stays there for the rest of its shift, because it only asks again on
    reaching the next node and it will not reach one.

    Both travel as `RobotToNetwork`, and `available` is what tells them apart: a
    report carrying legal exits is a question and is answered; one without them
    is telemetry and is not. Every report updates position either way, which is
    the point — the fleet learns where its robots are from the same messages
    that ask it where to send them.

HOW — the rules that shape the code below
    **Never answer with silence.** A message we cannot make sense of, a planner
    that raised, a map that disagrees with the robot's: all of them get an
    answer. A wrong lane is recoverable at the next node; no answer is not.

    **Answer on the asking thread, not the reading one.** gRPC gives each stream
    its own thread, so a robot's question is planned and replied to without
    touching the run loop's tick. Position, by contrast, is *handed* to the run
    loop rather than applied here, so there is one writer of the bot's own state
    and it is the same one there has always been.

    **A stream ending is not an error.** A companion restarts, a shift ends, a
    container is killed. The bot keeps the last position it was told and carries
    on; the next stream picks up where this one stopped.
"""
from __future__ import annotations

import logging
from typing import TYPE_CHECKING, Callable, Iterator

import grpc

from planning.decide import Decision, DecisionKind, Query
from proto import robot_pb2, robot_pb2_grpc

if TYPE_CHECKING:
    from bot import RobotState

log = logging.getLogger(__name__)

#: Our `DecisionKind` to the wire's enum. Kept explicit rather than derived from
#: the names: the two are allowed to diverge, and a mapping that breaks loudly
#: when they do is better than one that silently sends UNSPECIFIED.
_KIND = {
    DecisionKind.PROCEED: robot_pb2.KIND_PROCEED,
    DecisionKind.REROUTE: robot_pb2.KIND_REROUTE,
    DecisionKind.WAIT: robot_pb2.KIND_WAIT,
    DecisionKind.YIELD: robot_pb2.KIND_YIELD,
}

#: Wire error enum to the flat string the roster carries. `PeerRecord.fault` is
#: a string and stays one -- the leader shows it and nothing parses it -- so
#: this is where the typed value becomes the human one.
_ERROR = {
    robot_pb2.ERROR_TYPE_MOTOR_ERROR: "MOTOR_ERROR",
    robot_pb2.ERROR_TYPE_CAMERA_ERROR: "CAMERA_ERROR",
    robot_pb2.ERROR_TYPE_LIDAR_ERROR: "LIDAR_ERROR",
    robot_pb2.ERROR_TYPE_LOCATION_UNKNOWN: "LOCATION_UNKNOWN",
    robot_pb2.ERROR_TYPE_MISC_ERROR: "MISC_ERROR",
}

_MISSION = {
    "park": "PARK",
    "charge": "CHARGE",
    "hold": "HOLD",
    "idle": "IDLE",
    "cargo": "CARGO",
}

_CARGO = {
    robot_pb2.CARGO_STATE_PICKUP: "PICKUP",
    robot_pb2.CARGO_STATE_DROPOFF: "DROPOFF",
    robot_pb2.CARGO_STATE_EN_ROUTE: "EN_ROUTE",
}


def query_of(message: robot_pb2.RobotToNetwork) -> Query:
    """The asking half of a report, in the planner's own shape."""
    return Query(
        query_id=message.query_id,
        node_id=message.latest_node_id,
        region_id=message.region_id,
        heading_rad=message.heading_rad,
        available=tuple(message.available),
    )


def reply_of(decision: Decision) -> robot_pb2.NetworkToRobot:
    """The planner's answer, on the wire."""
    return robot_pb2.NetworkToRobot(
        target_node_id=decision.target_node_id,
        kind=_KIND.get(decision.kind, robot_pb2.KIND_UNSPECIFIED),
        hold_ms=decision.hold_ms,
        because=decision.because,
        query_id=decision.query_id,
    )


def state_of(message: robot_pb2.RobotToNetwork, state_factory) -> RobotState:
    """The reporting half, as the run loop's own snapshot.

    Mission and cargo come out of a `oneof`, so "which case is set" *is* the
    value; there is no separate discriminator to keep in agreement with it.
    """
    mission = _MISSION.get(message.mission.WhichOneof("kind") or "", "")
    cargo = ""
    if message.mission.WhichOneof("kind") == "cargo":
        cargo = _CARGO.get(message.mission.cargo.state, "")

    fault = ""
    if message.fault.HasField("error"):
        fault = _ERROR.get(message.fault.error.type, "MISC_ERROR")
    elif message.fault.warning.WhichOneof("kind") == "low_battery":
        fault = f"LOW_BATTERY:{message.fault.warning.low_battery.percentage:.0f}"

    return state_factory(
        latest_node_id=message.latest_node_id,
        region_id=message.region_id,
        battery=message.telemetry.battery.percentage,
        # A robot that is answering at a node is standing at it. Anything else
        # it is doing between nodes is not something this wire reports, and
        # guessing MOVING here would make every arrival look like motion.
        state="IDLE",
        mission=mission or "IDLE",
        fault=fault,
        job_id=message.mission.cargo.cargo_id if cargo else "",
        cargo_state=cargo,
    )


def obstruction_in(message: robot_pb2.RobotToNetwork) -> int | None:
    """Whether this report is about something in the lane, and where from.

    Returns the node the robot was standing at when it saw the obstacle, `0` if
    it is saying the lane is clear again, or `None` if the report says nothing
    about obstacles at all -- which is almost all of them.

    This is the field whose absence made the whole obstruction path synthetic:
    the shared schema has always carried `current_node_id` on an OBSTACLE
    warning, and the network layer's flat fault string dropped it on the way in,
    so nothing could ever build an obstruction from what a robot actually saw.

    Note the three-way answer. "No obstacle mentioned" and "the obstacle has
    gone" are different things, and conflating them would clear every blockage
    on the next ordinary marker report -- which is every report.
    """
    if message.fault.warning.WhichOneof("kind") != "obstacle":
        return None
    return message.fault.warning.obstacle.current_node_id


class RobotNetworkServicer(robot_pb2_grpc.RobotNetworkServicer):
    """Serves one robot's stream.

    `router` plans an answer; `report` hands a snapshot to the run loop;
    `obstruct` records a blockage. Three callables rather than the whole bot, so
    this module is testable without one.
    """

    def __init__(
        self,
        router: Callable[[Query], Decision],
        report: Callable[[RobotState], None],
        obstruct: Callable[[int, float], None],
        state_factory,
        bot_id: int = 0,
    ) -> None:
        self._router = router
        self._report = report
        self._obstruct = obstruct
        self._state_factory = state_factory
        self._bot_id = bot_id
        #: Counters, for tests and for anyone wondering whether a robot is
        #: talking at all.
        self.reports = 0
        self.answered = 0

    def Session(self, request_iterator, context: grpc.ServicerContext) -> Iterator:
        try:
            for message in request_iterator:
                reply = self._handle(message)
                if reply is not None:
                    yield reply
                    self.answered += 1
        except grpc.RpcError:
            # The companion went away mid-stream. Not an error: a shift ended,
            # or a container was killed. We keep the last position it gave us.
            log.debug("bot-%d: robot stream ended", self._bot_id)

    def _handle(self, message: robot_pb2.RobotToNetwork) -> robot_pb2.NetworkToRobot | None:
        """One message in, at most one answer out."""
        try:
            self._report(state_of(message, self._state_factory))
            self.reports += 1
        except Exception:
            # A report we could not apply must not cost the robot its answer.
            log.exception("bot-%d: could not apply a robot report", self._bot_id)

        seen_at = obstruction_in(message)
        if seen_at is not None:
            self._obstruct(seen_at, 1.0 if seen_at else 0.0)

        if not message.available:
            return None  # telemetry: position noted, nothing asked

        query = query_of(message)
        try:
            return reply_of(self._router(query))
        except Exception:
            # Whatever went wrong upstream, the robot is still standing at a
            # node waiting. Hold it briefly and let it ask again rather than
            # leaving it there for the rest of the shift.
            log.exception("bot-%d: planning failed for node %d",
                          self._bot_id, query.node_id)
            return reply_of(Decision(
                query_id=query.query_id,
                kind=DecisionKind.WAIT,
                hold_ms=1000,
                because="planner error",
            ))
