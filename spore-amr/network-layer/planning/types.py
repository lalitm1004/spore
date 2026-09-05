"""Inputs and outputs of the planner.

These are the planner's own view of the world, not the wire format. The heartbeat
carries a great deal that routing does not care about -- job state, priority keys,
who is blocked on whom -- and the adapter at the transport boundary is expected to
project it down to `PeerView`. Keeping the planner off the wire types means it can
be tested without a codec, and that a heartbeat change only moves one adapter.

All times are integer milliseconds on the *local* monotonic clock. Peer timestamps
arrive on the peer's clock and are converted on the way in (see `PeerView.
clock_offset_ms`), so nothing downstream has to think about whose clock it holds.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, field
from enum import StrEnum

from planning.cost import EnergyState
from planning.geometry import Heading

INF_MS = 1 << 62
"""Stands in for "forever" in interval arithmetic."""


# -- inputs ------------------------------------------------------------------


@dataclass(frozen=True, slots=True)
class Reservation:
    """One entry of a peer's `res[]`: a timed claim on a single node.

    `t_in`/`t_out` are on the *claiming peer's* clock. `dir` is the heading the peer
    takes on leaving the node, and is None for the last hop of its path, where it
    has not committed to a direction yet.
    """

    node_id: int
    t_in: int
    t_out: int
    dir: Heading | None = None
    claimed_at: int = 0

    def __post_init__(self) -> None:
        if self.t_out < self.t_in:
            raise ValueError(
                f"reservation on node {self.node_id} ends before it starts: "
                f"{self.t_in} > {self.t_out}"
            )


@dataclass(frozen=True, slots=True)
class PeerView:
    """What this robot knows about one peer, projected from its heartbeat."""

    bot_id: int
    reservations: tuple[Reservation, ...] = ()

    node_id: int | None = None
    """Last QR node the peer scanned."""

    edge: tuple[int, int] | None = None
    """Node id pair the peer is currently between, if it is mid-edge."""

    progress: float = 0.0
    """How far along `edge` the peer is, 0..1."""

    speed_cm_s: float = 0.0

    clock_offset_ms: int = 0
    """Added to the peer's timestamps to put them on the local clock (from the
    beacon). Zero when clocks are already aligned."""

    desynced: bool = False
    """Peer's clock is not trusted; its claims get widened by `skew_bound_ms`."""


@dataclass(frozen=True, slots=True)
class SelfState:
    """This robot's own state at planning time."""

    node_id: int
    heading: Heading | None = None
    """Current heading. None means unknown, which is charged as a full reversal --
    conservative, because under-charging here would under-size the reservations."""

    moving: bool = False
    energy: EnergyState = EnergyState.OK
    urgent: bool = False
    battery_percent: float = 100.0


class GoalKind(StrEnum):
    NODE = "NODE"
    """Go to one specific node."""

    CHARGE = "CHARGE"
    """Go to whichever charger is cheapest to reach, traffic included."""

    PARK = "PARK"
    """Go to whichever parking bay is cheapest to reach."""


@dataclass(frozen=True, slots=True)
class Goal:
    kind: GoalKind
    node_id: int | None = None

    def __post_init__(self) -> None:
        if self.kind is GoalKind.NODE and self.node_id is None:
            raise ValueError("a NODE goal needs a node_id")

    @staticmethod
    def node(node_id: int) -> Goal:
        return Goal(kind=GoalKind.NODE, node_id=node_id)

    @staticmethod
    def charge() -> Goal:
        return Goal(kind=GoalKind.CHARGE)

    @staticmethod
    def park() -> Goal:
        return Goal(kind=GoalKind.PARK)


@dataclass(frozen=True, slots=True)
class Obstruction:
    """A reported blockage on a node, repeated as soft state in the heartbeat.

    `level` runs 0..1. Above `Config.obstruction_block_level` the node is treated as
    impassable; below it the node is merely expensive, so a robot will still route
    through a lightly obstructed lane if the alternative is much worse.
    """

    node_id: int
    level: float = 1.0
    asserter: int | None = None


@dataclass(frozen=True, slots=True)
class RegionGossip:
    """Second-hand summary of a region, used past the reservation horizon.

    `edge_load` is keyed by node id because reservations are node-addressed; if the
    wire format keys it differently, the heartbeat adapter translates.
    """

    region_id: int
    bots: int = 0
    chargers_free: int = 0
    edge_load: Mapping[int, float] = field(default_factory=dict)
    load: float = 0.0
    """Region-wide load 0..1, used when per-node detail is not available."""

    t_observed: int = 0


@dataclass(frozen=True, slots=True)
class Config:
    """Tuning. Every duration is milliseconds."""

    k_commit: int = 8
    """How many leading hops carry timing and become reservations. Owned by the
    protocol; the planner only needs to know how much of its path to time."""

    skew_bound_ms: int = 300
    """How far a desynced peer's clock might be out. Its claims widen by this."""

    safety_ms: int = 250
    """Clearance added to every peer claim, covering control latency and speed
    estimation error."""

    follow_gap_ms: int = 0
    """Extra separation required behind a peer, beyond the traversal overlap that
    already keeps robots a node apart."""

    horizon_ms: int = 180_000
    """Plans further out than this are not worth timing."""

    max_wait_ms: int = 30_000
    """Longest single wait the search will consider at one node."""

    max_expansions: int = 120_000
    """Search budget, so a pathological request cannot blow the tick."""

    yield_wait_threshold_ms: int = 4_000
    """Planned wait above which the planner offers a yield suggestion."""

    improvement_margin: float = 0.05
    """A new path must beat the current one by this fraction to displace it."""

    stable_ticks: int = 3
    """...and must do so this many ticks running."""

    yield_search_hops: int = 8
    """How far to look for somewhere to stand aside before giving up. Beyond
    this the drive to the bay costs more than the wait it saves."""

    arrived_hold_ms: int = 2_000
    """How long a robot sitting on its goal waits before asking again. It has
    nowhere to be until a new job arrives."""

    blocked_hold_ms: int = 1_000
    """How long to hold when there is no route at all. Short, because the thing
    in the way is usually another robot that is about to move."""

    max_hold_ms: int = 4_000
    """Longest single WAIT we will issue. Capped below the robot's socket
    timeout so a held robot always comes back to ask again rather than deciding
    the network layer has died."""

    obstruction_block_level: float = 0.7
    congestion_weight: float = 0.35
    """Scales the soft traffic penalty, as a fraction of one straight hop."""

    congestion_decay_hops: float = 6.0
    """How quickly a peer's soft influence falls off with distance."""

    def __post_init__(self) -> None:
        if self.k_commit < 1:
            raise ValueError("k_commit must be at least 1")
        if not 0.0 <= self.improvement_margin < 1.0:
            raise ValueError("improvement_margin must be in [0, 1)")


DEFAULT_CONFIG = Config()


def from_env() -> Config:
    """Build the planner's tuning from `config.py`.

    The fleet has one config file and it is env-driven (PROTOCOL.md §9); this
    dataclass exists so the planner stays testable without environment
    variables, not as a second place to tune things.
    """
    import config as fleet

    return Config(
        k_commit=fleet.K_COMMIT,
        safety_ms=int(fleet.PLAN_SAFETY * 1000),
        horizon_ms=int(fleet.PLAN_HORIZON * 1000),
        max_wait_ms=int(fleet.MAX_WAIT * 1000),
        max_expansions=fleet.MAX_EXPANSIONS,
        yield_wait_threshold_ms=int(fleet.T_YIELD_THRESHOLD * 1000),
        yield_search_hops=fleet.YIELD_SEARCH_HOPS,
        arrived_hold_ms=int(fleet.T_ARRIVED_HOLD * 1000),
        blocked_hold_ms=int(fleet.T_BLOCKED_HOLD * 1000),
        max_hold_ms=int(fleet.T_MAX_HOLD * 1000),
        stable_ticks=fleet.PLAN_STABLE_TICKS,
        congestion_weight=fleet.CONGESTION_WEIGHT,
    )


# -- outputs -----------------------------------------------------------------


@dataclass(frozen=True, slots=True)
class Hop:
    """One node on the planned path, with the window the robot will hold it for.

    Maps one-to-one onto a `res[]` entry: the reservation layer publishes
    `rid=node_id, dir, t_in, t_out` and stamps `claimed_at` itself. Timing is
    meaningful only for the first `Path.committed` hops.
    """

    node_id: int
    dir: Heading | None
    t_in: int
    t_out: int
    wait: int = 0


class PlanStatus(StrEnum):
    OK = "OK"
    ALREADY_THERE = "ALREADY_THERE"
    START_BLOCKED = "START_BLOCKED"
    """A peer holds the robot's own node right now -- a conflict that already
    exists, which the planner cannot route out of."""

    UNREACHABLE = "UNREACHABLE"
    NO_GOAL_AVAILABLE = "NO_GOAL_AVAILABLE"
    """A class goal with no candidates: no charger, no parking bay."""

    SEARCH_EXHAUSTED = "SEARCH_EXHAUSTED"


@dataclass(frozen=True, slots=True)
class Path:
    hops: tuple[Hop, ...]
    committed: int
    goal_node_id: int
    cost: float
    duration_ms: int
    energy_j: float

    @property
    def node_ids(self) -> tuple[int, ...]:
        return tuple(hop.node_id for hop in self.hops)

    def index_of(self, node_id: int) -> int | None:
        """Where along this path a node sits, or None if it is not on it.

        First occurrence wins. A path can revisit a node -- backing out of a
        dead-end bay is the common case -- and for locating a robot that is
        following the path forwards, the earliest match is the right one.
        """
        for i, hop in enumerate(self.hops):
            if hop.node_id == node_id:
                return i
        return None

    def __len__(self) -> int:
        return len(self.hops)


@dataclass(frozen=True, slots=True)
class YieldSuggestion:
    """Somewhere to step aside. Advisory: the decision to yield is not the
    planner's to make."""

    node_id: int
    kind: str
    """`YI`, `JUNCTION`, or the bay type borrowed as a fallback."""

    hops_away: int
    reason: str


@dataclass(frozen=True, slots=True)
class Diagnostics:
    expansions: int = 0
    blocking_peers: tuple[int, ...] = ()
    """Peers whose claims forced a wait or a detour."""

    corridor_entered: tuple[int, ...] = ()
    """Nodes of the corridor the first hop commits to, for the layer that owns
    deadlock resolution. The planner reports it and does not act on it."""

    corridor_opposing_peers: tuple[int, ...] = ()
    """Peers inside that corridor travelling the other way."""

    goal_candidates: int = 0
    goal_rationale: str = ""
    replan_reason: str = ""


@dataclass(frozen=True, slots=True)
class Result:
    status: PlanStatus
    path: Path | None
    changed: bool
    yield_to: YieldSuggestion | None = None
    diagnostics: Diagnostics = field(default_factory=Diagnostics)

    @property
    def ok(self) -> bool:
        return self.status in (PlanStatus.OK, PlanStatus.ALREADY_THERE)


@dataclass(frozen=True, slots=True)
class Request:
    now: int
    self_state: SelfState
    goal: Goal
    peers: tuple[PeerView, ...] = ()
    obstructions: tuple[Obstruction, ...] = ()
    gossip: tuple[RegionGossip, ...] = ()
    current: Path | None = None
    stable_for: int = 0
    """How many consecutive ticks an alternative has already looked better. The
    caller carries this between calls; hysteresis returns the updated count."""
