"""What to tell a robot that is standing at a node and waiting.

WHAT
    `Query` / `Decision` — this side's view of the robot link, and `decide`,
    which turns one into the other.

WHERE
    Called by `planning.server` for every question a robot asks. The wire form
    is newline-delimited JSON defined by `spore-amr/webots/robot/network.py`;
    these are the typed mirrors, so nothing below this module parses JSON.

WHY
    The robot is blind by design. It arrives at a QR node, works out which turns
    physically exist from its own copy of the map, and asks which one to take.
    It then *blocks* — up to its socket timeout — and if it hears nothing it
    sits there forever, because it only asks again on reaching the next node.

    Two rules follow from that, and both matter more than they look:

    **Never answer with silence.** Even when our map and its map disagree, or
    the node we wanted is not among the turns offered, we answer with a legal
    turn. A wrong turn is recoverable on the next node; no answer is not.

    **Waiting has to be sayable.** The original protocol had no way to express
    it, so a robot told to wait was indistinguishable from a robot whose network
    layer had died. `Decision.kind` adds it: `WAIT` with `hold_ms`, after which
    the robot asks again.

HOW — proceed, wait, yield, or reroute
    The search already prices waiting against going round: that is what its wait
    actions are for, and with charge in the cost function a robot low on battery
    correctly waits where a fresh one detours. Yielding is the third option and
    the only one that means leaving the route entirely, so it needs an explicit
    rule rather than falling out of the search:

      1. the plan waits longer than `yield_wait_threshold_ms`, and
      2. we *lose* the yield-priority comparison against whoever is blocking us
         — free 0 < heading-to-pickup 1 < carrying cargo 2, ties on lower bot id
         — and
      3. somewhere to stand aside is within reach.

    If we *win* that comparison we wait and keep our claim: the other robot is
    the one that should move. Both sides compute the same verdict from the same
    two numbers, so they cannot both decide to give way, and they cannot both
    decide to stay.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from enum import StrEnum

from planning.geometry import NodeType
from planning.graph import Graph
from planning.topology import Topology
from planning.traffic import Observation, TrafficView
from planning.types import Config, PlanStatus

TURNS = ("left", "straight", "right")


@dataclass(frozen=True, slots=True)
class Query:
    """What a robot asks on arriving at a node.

    `available` is the turns that lead somewhere, resolved by the robot from the
    shared map against the heading it arrived on — so it, not us, decides what
    is physically possible.
    """

    query_id: int
    node_id: int
    node_type: str = "PT"
    region_id: int = 0
    x_cm: float = 0.0
    y_cm: float = 0.0
    heading_rad: float = 0.0
    available: dict[str, int] = field(default_factory=dict)

    @classmethod
    def from_json(cls, text: str) -> Query:
        d = json.loads(text)
        node = d.get("node", {})
        position = d.get("robot_position", {})
        return cls(
            query_id=int(d["query_id"]),
            node_id=int(node["id"]),
            node_type=str(node.get("node_type", "PT")),
            region_id=int(node.get("region_id", 0)),
            x_cm=float(position.get("x", 0.0)),
            y_cm=float(position.get("y", 0.0)),
            heading_rad=float(d.get("heading_rad", 0.0)),
            available={k: int(v) for k, v in (d.get("available") or {}).items()},
        )


class DecisionKind(StrEnum):
    PROCEED = "PROCEED"
    """Take this lane; it is the route we were already on."""

    REROUTE = "REROUTE"
    """Take this lane, but the route changed since we last answered."""

    WAIT = "WAIT"
    """Stay put for `hold_ms`, then ask again."""

    YIELD = "YIELD"
    """Leave the route and stand aside at the node named."""


@dataclass(frozen=True, slots=True)
class Decision:
    """The answer. `turn` is empty only for WAIT, which names no lane."""

    query_id: int
    kind: DecisionKind = DecisionKind.PROCEED
    turn: str = ""
    target_node_id: int = 0
    hold_ms: int = 0
    because: str = ""

    def to_json(self) -> str:
        # `kind`, `hold_ms` and `because` are additive: a robot reading only
        # `turn` and `target_node_id` still behaves correctly for every moving
        # kind. Only WAIT needs the new fields understood.
        return json.dumps(
            {
                "schema_version": "v0.1.0",
                "query_id": self.query_id,
                "kind": str(self.kind),
                "turn": self.turn,
                "target_node_id": self.target_node_id,
                "hold_ms": self.hold_ms,
                "because": self.because,
            },
            separators=(",", ":"),
        )


def turn_for(query: Query, node_id: int) -> str | None:
    """Which of the offered turns leads to `node_id`."""
    for turn, destination in sorted(query.available.items()):
        if destination == node_id:
            return turn
    return None


def fallback_turn(query: Query) -> tuple[str, int] | None:
    """Any legal turn, preferring straight on.

    Used only when the node we wanted is not among the ones offered — our map
    and the robot's disagree. Going somewhere recoverable beats standing still.
    """
    for preferred in ("straight", "left", "right"):
        if preferred in query.available:
            return preferred, query.available[preferred]
    for turn in sorted(query.available):
        return turn, query.available[turn]
    return None


def outranked_by(mine: int, my_bot_id: int, theirs: int, their_bot_id: int) -> bool:
    """Whether the other robot has right of way over us.

    The same ordering both sides run: higher yield priority wins, ties break on
    the lower bot id. Computed from numbers every bot already has, so neither
    has to ask the other.
    """
    return (-theirs, their_bot_id) < (-mine, my_bot_id)


def choose_yield_spot(
    graph: Graph,
    topology: Topology,
    at_node: int,
    *,
    radius: int,
    avoid: frozenset[int] = frozenset(),
) -> int | None:
    """Somewhere to stand aside: a yield bay, else a junction, else any bay.

    The cascade exists because real yield bays are scarce — 15 on the whole
    floor, covering two regions of seven — so insisting on one would mean never
    yielding across most of the warehouse. A junction is the honest second
    choice: it is somewhere a robot can actually get past us, and there are
    plenty. A parking or charging spur is the last resort, and worth logging,
    because sitting in a charger may block someone who needs it.
    """
    for predicate in (
        lambda n: graph.node_type[n] is NodeType.YI,
        topology.is_junction,
        lambda n: graph.node_type[n] in (NodeType.PK, NodeType.CH),
    ):
        # `predicate` is bound as a default: `nearest` happens to call this
        # synchronously, but a late-bound loop variable is a bug waiting for
        # someone to make the call lazy.
        found = graph.nearest(
            at_node,
            lambda n, want=predicate: want(n) and n not in avoid and n != at_node,
            radius,
        )
        if found is not None:
            return found[0]
    return None


def decide(
    graph: Graph,
    topology: Topology,
    query: Query,
    *,
    plan,
    traffic: TrafficView,
    observations: tuple[Observation, ...],
    my_bot_id: int,
    my_rank: int,
    config: Config,
    last_target: int | None = None,
) -> Decision:
    """Turn a plan into the one thing the robot can act on.

    `plan` is a `planning.types.Result`. Everything here is about expressing it
    inside a protocol that offers three turns and a wait.
    """
    if plan is None or plan.status is PlanStatus.ALREADY_THERE:
        return Decision(
            query_id=query.query_id,
            kind=DecisionKind.WAIT,
            hold_ms=config.arrived_hold_ms,
            because="at the goal",
        )

    if plan.status is not PlanStatus.OK or plan.path is None or len(plan.path.hops) < 2:
        # No route: blocked, unreachable, or the search gave up. Hold briefly and
        # ask again rather than guessing — but never stay silent.
        return Decision(
            query_id=query.query_id,
            kind=DecisionKind.WAIT,
            hold_ms=config.blocked_hold_ms,
            because=f"no route ({plan.status.value.lower()})",
        )

    here, ahead = plan.path.hops[0], plan.path.hops[1]
    wait_ms = here.wait

    if wait_ms >= config.yield_wait_threshold_ms:
        # Whoever is stopping us is on the node *ahead*, not the one underfoot,
        # and over the window we wanted it rather than the later one the plan
        # settled for — by then the blocker has gone and nobody looks at fault.
        blockers = traffic.blockers(graph.index(ahead.node_id), here.t_in, ahead.t_out)
        ranks = {o.bot_id: o.rank for o in observations}
        losing = any(
            outranked_by(my_rank, my_bot_id, ranks.get(b, 0), b) for b in blockers
        )
        if losing:
            spot = choose_yield_spot(
                graph,
                topology,
                graph.index(here.node_id),
                radius=config.yield_search_hops,
            )
            if spot is not None:
                spot_id = graph.id_of(spot)
                step = _first_step_towards(graph, graph.index(here.node_id), spot)
                turn = turn_for(query, graph.id_of(step)) if step is not None else None
                if turn is not None:
                    return Decision(
                        query_id=query.query_id,
                        kind=DecisionKind.YIELD,
                        turn=turn,
                        target_node_id=graph.id_of(step),
                        because=f"giving way to {sorted(blockers)} at node {spot_id}",
                    )
        # We win the comparison, or there is nowhere to stand aside. Hold the
        # node and let the other robot move.
        return Decision(
            query_id=query.query_id,
            kind=DecisionKind.WAIT,
            hold_ms=min(wait_ms, config.max_hold_ms),
            because=f"holding against {sorted(blockers)}" if blockers else "waiting",
        )

    if wait_ms > 0:
        return Decision(
            query_id=query.query_id,
            kind=DecisionKind.WAIT,
            hold_ms=wait_ms,
            because="short wait for the lane ahead",
        )

    turn = turn_for(query, ahead.node_id)
    if turn is None:
        # Our map and the robot's disagree about what leaves this node. Answer
        # with something legal: a wrong turn is recoverable next node, silence
        # is not.
        alternative = fallback_turn(query)
        if alternative is None:
            return Decision(
                query_id=query.query_id,
                kind=DecisionKind.WAIT,
                hold_ms=config.blocked_hold_ms,
                because="the robot offered no turns",
            )
        turn, target = alternative
        return Decision(
            query_id=query.query_id,
            kind=DecisionKind.REROUTE,
            turn=turn,
            target_node_id=target,
            because=f"node {ahead.node_id} was not among the turns offered",
        )

    changed = last_target is not None and last_target != ahead.node_id
    return Decision(
        query_id=query.query_id,
        kind=DecisionKind.REROUTE if changed else DecisionKind.PROCEED,
        turn=turn,
        target_node_id=ahead.node_id,
        because="rerouted" if changed else "",
    )


def _first_step_towards(graph: Graph, start: int, goal: int) -> int | None:
    """The neighbour of `start` that begins the shortest way to `goal`."""
    if start == goal:
        return None
    hops = graph.hops_from(goal)
    best = min(
        (n for n, _ in graph.neighbours(start)),
        key=lambda n: (hops[n], n),
        default=None,
    )
    return best
