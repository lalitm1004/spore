"""The planner: a world snapshot in, a proposed path out.

Everything else in this package is a piece of one call. `plan` runs them in order:

    reservations -> goal candidates -> congestion -> search -> windows -> hysteresis

The step worth reading closely is the last but one, where the search's arrival times
become the windows the reservation layer will publish. A robot holds a node from the
moment it *starts moving into* it until the moment it is *fully inside the next one*
-- so consecutive claims overlap by exactly one traversal. That overlap is the whole
reason node-addressed reservations are sufficient: two robots swapping across an
edge both claim both of its endpoints at once, and the clash surfaces as an ordinary
interval overlap. Publish the windows any tighter and that guarantee is lost.
"""

from __future__ import annotations

from spore_planner.planner import congestion as congestion_module
from spore_planner.planner import goals as goals_module
from spore_planner.planner import hysteresis, yielding
from spore_planner.planner.cost import CostModel
from spore_planner.planner.intervals import ReservationTable
from spore_planner.planner.kinematics import DEFAULT_KINEMATICS, Kinematics
from spore_planner.planner.sipp import SearchResult, search
from spore_planner.planner.types import (
    DEFAULT_CONFIG,
    Config,
    Diagnostics,
    Hop,
    Path,
    PlanStatus,
    Request,
    Result,
)
from spore_planner.warehouse.graph import Graph
from spore_planner.warehouse.map import Heading
from spore_planner.warehouse.topology import Topology


class Planner:
    """Plans for one robot on one map.

    Holds no state between calls -- every answer is a function of the request it was
    given -- so a `Planner` is safe to reuse, and several may share a `Graph` to
    share its cached distance tables. That is what the simulator does.
    """

    __slots__ = ("bot_id", "config", "graph", "kinematics", "topology")

    def __init__(
        self,
        graph: Graph,
        *,
        bot_id: int | None = None,
        config: Config = DEFAULT_CONFIG,
        kinematics: Kinematics | None = None,
        topology: Topology | None = None,
    ) -> None:
        self.graph = graph
        self.bot_id = bot_id
        self.config = config
        self.kinematics = kinematics if kinematics is not None else DEFAULT_KINEMATICS
        validate = getattr(self.kinematics, "validate_for_spacing", None)
        if validate is not None:
            validate(graph.node_spacing)
        self.topology = topology if topology is not None else Topology(graph)

    def plan(self, request: Request) -> Result:
        graph = self.graph
        config = self.config
        state = request.self_state

        if not graph.has_id(state.node_id):
            return _failure(
                PlanStatus.UNREACHABLE,
                Diagnostics(replan_reason=f"node {state.node_id} is not on this map"),
            )
        start = graph.index(state.node_id)

        cost_model = CostModel.for_state(
            graph.node_spacing,
            state.energy,
            urgent=state.urgent,
            kinematics=self.kinematics,
        )
        hop_cost = cost_model.min_hop_cost()

        table = ReservationTable(
            graph,
            request.peers,
            now=request.now,
            config=config,
            exclude_bot_id=self.bot_id,
        )

        resolved = goals_module.resolve(
            graph,
            request.goal,
            hop_cost=hop_cost,
            peers=request.peers,
            gossip=request.gossip,
            exclude_bot_id=self.bot_id,
        )
        if not resolved.candidates:
            return _failure(
                PlanStatus.NO_GOAL_AVAILABLE,
                Diagnostics(goal_rationale=resolved.rationale),
            )

        field = congestion_module.build(
            graph,
            config=config,
            hop_cost=hop_cost,
            peers=request.peers,
            gossip=request.gossip,
            obstructions=request.obstructions,
            exclude_bot_id=self.bot_id,
        )
        penalty = _combine(field, resolved.penalties)

        found = search(
            graph,
            table,
            start=start,
            goals=resolved.candidates,
            cost_model=cost_model,
            config=config,
            start_heading=state.heading,
            moving=state.moving,
            blocked_nodes=field.blocked,
            node_penalty=penalty,
        )

        diagnostics = self._diagnose(found, table, request, resolved)

        if found.status is PlanStatus.ALREADY_THERE:
            path = Path(
                hops=(Hop(node_id=state.node_id, dir=None, t_in=request.now, t_out=request.now),),
                committed=1,
                goal_node_id=state.node_id,
                cost=0.0,
                duration_ms=0,
                energy_j=0.0,
            )
            return Result(
                status=found.status, path=path, changed=True, diagnostics=diagnostics
            )

        if found.status is not PlanStatus.OK:
            return _failure(found.status, diagnostics)

        path = self._to_path(found, cost_model)

        decision = hysteresis.decide(
            graph,
            table,
            current=request.current,
            candidate=path,
            at_node_id=state.node_id,
            goal=request.goal,
            blocked=field.blocked,
            config=config,
            stable_for=request.stable_for,
        )
        diagnostics = _with_reason(diagnostics, decision.reason)

        return Result(
            status=PlanStatus.OK,
            path=path if decision.replace else request.current,
            changed=decision.replace,
            yield_to=self._maybe_yield(found, table),
            diagnostics=diagnostics,
        )

    # -- assembling the answer ----------------------------------------------

    def _to_path(self, found: SearchResult, cost_model: CostModel) -> Path:
        """Turn search timings into the windows the reservation layer publishes."""
        graph = self.graph
        nodes = found.nodes
        last = len(nodes) - 1
        settle_ms = cost_model.arrive().duration_ms

        hops: list[Hop] = []
        for i, node in enumerate(nodes):
            # Held from the moment the robot begins moving in -- which is the start
            # of the previous traversal, not the moment it arrives.
            t_in = found.arrivals[0] if i == 0 else found.departures[i - 1]
            if i < last:
                # ...and not released until the robot is fully inside the next node.
                t_out = found.arrivals[i + 1]
                direction: Heading | None = Heading(found.headings[i])
                wait = found.waits[i]
            else:
                t_out = found.arrivals[i] + settle_ms
                direction = None
                wait = 0
            hops.append(
                Hop(
                    node_id=graph.id_of(node),
                    dir=direction,
                    t_in=t_in,
                    t_out=t_out,
                    wait=wait,
                )
            )

        return Path(
            hops=tuple(hops),
            committed=min(len(hops), self.config.k_commit),
            goal_node_id=graph.id_of(nodes[-1]),
            cost=found.cost,
            duration_ms=found.duration_ms,
            energy_j=found.energy_j,
        )

    def _diagnose(
        self,
        found: SearchResult,
        table: ReservationTable,
        request: Request,
        resolved: goals_module.ResolvedGoal,
    ) -> Diagnostics:
        graph = self.graph
        blocking: set[int] = set()
        for i, wait in enumerate(found.waits):
            if wait > 0 and i + 1 < len(found.nodes):
                blocking.update(
                    table.blockers(
                        found.nodes[i + 1], found.arrivals[i], found.arrivals[i + 1]
                    )
                )

        corridor_nodes: tuple[int, ...] = ()
        opposing: tuple[int, ...] = ()
        if len(found.nodes) >= 2:
            corridor = self.topology.corridor_entered_by(found.nodes[0], found.nodes[1])
            if corridor is not None:
                corridor_nodes = tuple(graph.id_of(n) for n in corridor.nodes)
                opposing = self._opposing_peers(
                    corridor_nodes, Heading(found.headings[0]), request
                )

        return Diagnostics(
            expansions=found.expansions,
            blocking_peers=tuple(sorted(blocking)),
            corridor_entered=corridor_nodes,
            corridor_opposing_peers=opposing,
            goal_candidates=resolved.count,
            goal_rationale=resolved.rationale,
        )

    def _opposing_peers(
        self, corridor_node_ids: tuple[int, ...], heading: Heading, request: Request
    ) -> tuple[int, ...]:
        """Peers inside the corridor we are entering, travelling against us.

        Reported, never acted on. Two robots entering opposite ends of a corridor
        longer than `K_COMMIT` will not see each other's claims, and neither one can
        route out of it once committed -- resolving that belongs to the layer that
        owns the priority ordering.
        """
        inside = set(corridor_node_ids)
        against = heading.opposite
        found: set[int] = set()
        for peer in request.peers:
            if self.bot_id is not None and peer.bot_id == self.bot_id:
                continue
            for reservation in peer.reservations:
                if reservation.node_id in inside and reservation.dir is against:
                    found.add(peer.bot_id)
                    break
        return tuple(sorted(found))

    def _maybe_yield(self, found: SearchResult, table: ReservationTable):
        """Offer somewhere to pull over if the plan involves a long stand-still."""
        if not found.waits:
            return None
        longest = max(found.waits)
        if longest < self.config.yield_wait_threshold_ms:
            return None
        index = found.waits.index(longest)
        blockers = set(
            table.blockers(found.nodes[index + 1], found.arrivals[index], found.arrivals[index + 1])
        )
        # Steer the suggestion away from anything the blocking peers have claimed --
        # pulling over into their path helps nobody.
        avoid = frozenset(
            node
            for node in table.blocked_nodes
            if any(claim.bot_id in blockers for claim in table.claims(node))
        )
        return yielding.suggest(
            self.graph,
            self.topology,
            at_node=found.nodes[index],
            avoid=avoid,
            reason=f"planned wait of {longest} ms for {sorted(blockers)}",
        )


def _combine(field: congestion_module.CongestionField, extra: dict[int, float]):
    if not extra:
        return field
    merged = dict(field.penalties)
    for node, amount in extra.items():
        merged[node] = merged.get(node, 0.0) + amount
    return congestion_module.CongestionField(penalties=merged, blocked=field.blocked)


def _failure(status: PlanStatus, diagnostics: Diagnostics) -> Result:
    return Result(status=status, path=None, changed=False, diagnostics=diagnostics)


def _with_reason(diagnostics: Diagnostics, reason: str) -> Diagnostics:
    return Diagnostics(
        expansions=diagnostics.expansions,
        blocking_peers=diagnostics.blocking_peers,
        corridor_entered=diagnostics.corridor_entered,
        corridor_opposing_peers=diagnostics.corridor_opposing_peers,
        goal_candidates=diagnostics.goal_candidates,
        goal_rationale=diagnostics.goal_rationale,
        replan_reason=reason,
    )
