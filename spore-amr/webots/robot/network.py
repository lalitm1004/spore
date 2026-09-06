"""The robot <-> network wire: what a robot asks, and what it is told.

At every QR node the companion resolves which nodes it can physically reach and
asks this question; the network layer answers with one of them. The robot never
picks a direction for itself -- that is the local autonomy the architecture says
the network layer owns.

The network layer is `spore-amr/network-layer`, one bot process per robot, and
these travel as `spore.network.v1.RobotToNetwork` / `NetworkToRobot`
(`network-layer/proto/robot.proto`). This module is only the shapes the robot
half reasons in: pure, no I/O, no protobuf. `robot/uplink.py` is the only thing
that knows there is a wire.

**Left and right never cross it.** `available` is node ids and an answer names a
node; the robot derives the bearing from the map it already holds, which is
exact because lanes are straight. A direction on the wire would be a second,
weaker description of geometry both ends have -- and the firmware bears it out,
reading `bearing` and `heading` off a TURN and never a turn name.

Message shapes follow spore-amr/shared/schemas/, plus the four fields this
fleet adds for asking rather than reporting (see the proto).
"""

from dataclasses import dataclass
from typing import Tuple

# What an answer can be. The original protocol had only "take this lane", which
# left no way to say "stay where you are" -- and a robot told nothing is
# indistinguishable from a robot whose network layer has died, because it only
# asks again on reaching the next node. WAIT closes that hole: hold for
# `hold_ms`, then ask the same question again.
PROCEED, REROUTE, WAIT, YIELD = "PROCEED", "REROUTE", "WAIT", "YIELD"
KINDS = (PROCEED, REROUTE, WAIT, YIELD)

# Cargo, as the shared schema names it. A job is a place to collect from, a
# place to deliver to, and a robot saying which of those it has done: the
# network layer sends CARGO/PICKUP with the collection node, and only moves the
# goal to the delivery node once the robot reports EN_ROUTE. Nothing else
# advances a job, so a robot that never reports arrives at the collection point
# and stops there for the rest of the shift.
IDLE, CARGO = "IDLE", "CARGO"
PICKUP, EN_ROUTE, DROPOFF = "PICKUP", "EN_ROUTE", "DROPOFF"


@dataclass(frozen=True)
class Query:
    """What a robot sends on arriving at a node.

    `available` is the nodes it can reach from here, resolved by the robot from
    the shared warehouse map against the heading it arrived on -- so it, not the
    network layer, decides what is physically possible.
    """

    query_id: int
    node_id: int
    node_type: str = "PT"
    region_id: int = 0
    heading_rad: float = 0.0
    available: Tuple[int, ...] = ()


@dataclass(frozen=True)
class Decision:
    """The answer: which node to head for, and whether to head anywhere at all.

    `query_id` comes back so a fresh answer is distinguishable from a late
    answer to the previous question. Two junctions can share a destination -- a
    reroute to the same place -- and that is exactly when confusing them would
    matter. It matters more with `hold_ms` in play: a WAIT meant for the
    previous node, applied at this one, stops a robot that had nothing wrong
    with it.
    """

    query_id: int
    target_node_id: int = 0
    kind: str = PROCEED
    hold_ms: int = 0
    because: str = ""
    # The job this robot is on, when the network layer has given it one. Empty
    # mission means it said nothing about cargo, which is not the same as
    # saying the robot is idle -- a WAIT carries no mission and must not clear
    # the job the robot is halfway through.
    mission: str = ""
    cargo_id: str = ""
    cargo_state: str = ""

    @property
    def is_wait(self) -> bool:
        return self.kind == WAIT
