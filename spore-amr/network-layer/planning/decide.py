"""What to tell a robot that is standing at a node and waiting.

WHAT
    `Query` / `Decision` — this side's view of the robot link, and `decide`,
    which turns one into the other.

WHERE
    Called by `planning.robot_service` for every question a robot asks. The wire
    form is `proto/robot.proto`; these are the planner's own shapes, so nothing
    below this module touches protobuf. The mapping between them lives in one
    place, and it is the only place that knows there is a wire at all.

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

from dataclasses import dataclass
from enum import StrEnum

from planning.geometry import NodeType
from planning.graph import Graph
from planning.topology import Topology
from planning.traffic import Observation, TrafficView
from planning.types import Config, PlanStatus

@dataclass(frozen=True, slots=True)
class Query:
    """What a robot asks on arriving at a node.

    `available` is the nodes it can legally reach from here, resolved by the
    robot from its own map against the heading it arrived on — so it, not us,
    decides what is physically possible. Nodes rather than turn names, because
    left and right never cross the wire: we name a node and the robot derives
    the bearing from the map it also holds, which is exact where a turn name is
    a second, weaker description of geometry both ends already have.
    """

    query_id: int
    node_id: int
    node_type: str = "PT"
    region_id: int = 0
    heading_rad: float = 0.0
    available: tuple[int, ...] = ()


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
    """The answer: a node to head for, or a reason to stay put.

    `target_node_id` is 0 only for WAIT, which names no lane. There is no turn
    here and there is none on the wire — see `Query.available`.
    """

    query_id: int
    kind: DecisionKind = DecisionKind.PROCEED
    target_node_id: int = 0
    hold_ms: int = 0
    because: str = ""


def offers(query: Query, node_id: int) -> bool:
    """Whether the robot said it can reach `node_id` from where it stands."""
    return node_id in query.available


def fallback_node(query: Query) -> int | None:
    """Any node the robot did offer.

    Used only when the one we wanted is not among them — our map and the
    robot's disagree about what leaves this node. Going somewhere recoverable
    beats standing still, and the lowest id is as good a choice as any: there
    is nothing to prefer between lanes we did not plan for. Sorted so two bots
    reading the same disagreement answer it the same way.
    """
    return min(query.available) if query.available else None


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

    # A head-on, which nothing else in this system can resolve. The planner
    # already names peers coming the other way down the corridor we are about
    # to enter (`Diagnostics.corridor_opposing_peers`) and deliberately does
    # not act on it -- `_opposing_peers` says so, and leaves it "to the layer
    # that owns the priority ordering". This is that layer.
    #
    # It has to be handled here rather than in the search because the search
    # cannot: a claim reserves a node and says nothing about the lane leading to
    # it, so two robots one lane apart, each holding its own node, are both
    # correctly reserved and can still drive into the lane between them from
    # opposite ends. Measured on an eight-robot run, two pairs ended nose to
    # nose -- 0.66 m and 0.69 m apart, both 180 degrees facing -- with no claim
    # violated by either. On a single painted line there is no passing, so
    # neither could recover.
    #
    # Refusing to enter is not the fix: both robots would refuse, and a
    # symmetric refusal is a livelock rather than a deadlock. So exactly one of
    # them has to give way, and both have to reach the same answer without
    # asking each other. `outranked_by` is that answer and it already exists --
    # higher yield priority wins, so a robot carrying cargo is never asked to
    # give way to an empty one, and the lower bot id breaks the tie. Both sides
    # compute it from the roster, so they agree by construction.
    opposing = getattr(plan.diagnostics, "corridor_opposing_peers", ())
    if opposing:
        ranks = {o.bot_id: o.rank for o in observations}
        if any(outranked_by(my_rank, my_bot_id, ranks.get(b, 0), b) for b in opposing):
            # Stand aside anywhere but inside the corridor we are refusing to
            # enter, and not on a node the oncoming robot has claimed.
            occupied = frozenset(
                graph.index(claim.node_id)
                for peer in traffic.peers
                for claim in peer.reservations
                if graph.has_id(claim.node_id)
            )
            spot = choose_yield_spot(
                graph, topology, graph.index(here.node_id),
                radius=config.yield_search_hops, avoid=occupied,
            )
            if spot is not None:
                return Decision(
                    query_id=query.query_id,
                    kind=DecisionKind.YIELD,
                    target_node_id=graph.id_of(spot),
                    because="head-on: yielding to bot {}".format(min(opposing)),
                )
            # Nowhere to pull over. Holding still is the whole of the fix here:
            # the robot with right of way keeps coming and takes the lane, and
            # we ask again once it has. Better than entering a corridor we
            # cannot back out of.
            return Decision(
                query_id=query.query_id,
                kind=DecisionKind.WAIT,
                hold_ms=config.blocked_hold_ms,
                because="head-on: holding for bot {}".format(min(opposing)),
            )

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
            # Never stand aside *through* the robot we are standing aside for.
            # Two separate things have to be checked, because `nearest` filters
            # the candidates and not the road to them:
            #
            #   the spot   -- `avoid`, so we do not pick somewhere to wait that
            #                 a blocker is already sitting on;
            #   the step   -- `is_free`, because the first hop is the one we
            #                 are about to actually drive, and it is the hop
            #                 that can land on the blocker.
            #
            # Without the second check a yield could send us straight into the
            # node we were yielding to avoid, which is the one outcome worse
            # than not yielding at all.
            occupied = frozenset(
                graph.index(claim.node_id)
                for peer in traffic.peers
                if peer.bot_id in set(blockers)
                for claim in peer.reservations
                if graph.has_id(claim.node_id)
            )
            spot = choose_yield_spot(
                graph,
                topology,
                graph.index(here.node_id),
                radius=config.yield_search_hops,
                avoid=occupied,
            )
            if spot is not None:
                spot_id = graph.id_of(spot)
                step = _first_step_towards(graph, graph.index(here.node_id), spot)
                step_id = graph.id_of(step) if step is not None else None
                if (step_id is not None and offers(query, step_id)
                        and traffic.is_free(step, here.t_in, ahead.t_out)):
                    return Decision(
                        query_id=query.query_id,
                        kind=DecisionKind.YIELD,
                        target_node_id=step_id,
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

    if not offers(query, ahead.node_id):
        # Our map and the robot's disagree about what leaves this node. Answer
        # with something legal: a wrong lane is recoverable at the next node,
        # silence is not.
        target = fallback_node(query)
        if target is None:
            return Decision(
                query_id=query.query_id,
                kind=DecisionKind.WAIT,
                hold_ms=config.blocked_hold_ms,
                because="the robot offered nowhere to go",
            )
        return Decision(
            query_id=query.query_id,
            kind=DecisionKind.REROUTE,
            target_node_id=target,
            because=f"node {ahead.node_id} was not among the nodes offered",
        )

    changed = last_target is not None and last_target != ahead.node_id
    return Decision(
        query_id=query.query_id,
        kind=DecisionKind.REROUTE if changed else DecisionKind.PROCEED,
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
