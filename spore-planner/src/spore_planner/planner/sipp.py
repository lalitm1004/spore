"""Safe Interval Path Planning over the warehouse graph.

Classic space-time A* discretises time into ticks and searches `(node, timestep)`.
That is a poor fit here. The reservation wire format is already interval-based, so
ticks would have to be expanded back into intervals anyway, and any quantisation is
a choice between rounding outward -- losing throughput in corridors that are already
single-file -- and rounding inward, which is unsafe.

SIPP (Phillips and Likhachev, 2011) searches `(node, safe interval)` instead, in
continuous time. The state space collapses to the number of *distinct occupancy
windows*, which with fewer than twenty peers is tiny, and waiting falls out of the
formulation for free: choosing a later departure inside the same interval is a wait,
and it is priced by the cost model like any other manoeuvre.

Two additions to the textbook formulation:

*Heading is part of the state.* Robots rotate in place, so what a hop costs depends
on the direction of arrival. Without heading in the state the search cannot tell a
straight run from a zig-zag, and on a battery-bound robot that difference matters.

*Two candidate manoeuvres per edge.* A robot that neither turns nor waits keeps its
speed and flows through; one that does either has to stop and get going again. Both
are generated and the search picks, which is exactly the wait-versus-detour decision
the energy term exists to express.
"""

from __future__ import annotations

from collections.abc import Callable, Iterable
from dataclasses import dataclass
from heapq import heappop, heappush

from spore_planner.planner.cost import CostModel
from spore_planner.planner.intervals import ReservationTable
from spore_planner.planner.types import Config, PlanStatus
from spore_planner.warehouse.graph import UNREACHABLE, Graph
from spore_planner.warehouse.map import Heading, quarter_turns

NO_HEADING = -1
"""Sentinel for an unknown heading, kept an int so search keys stay comparable."""


@dataclass(frozen=True, slots=True)
class SearchResult:
    """Raw search output, in dense node indices and local milliseconds."""

    status: PlanStatus
    nodes: tuple[int, ...] = ()
    """Dense indices, start first."""

    arrivals: tuple[int, ...] = ()
    """`arrivals[i]` is when the robot is fully inside `nodes[i]`."""

    departures: tuple[int, ...] = ()
    """`departures[i]` is when it starts moving from `nodes[i]` to `nodes[i+1]`;
    one shorter than `nodes`."""

    waits: tuple[int, ...] = ()
    """Deliberate wait at each node; one shorter than `nodes`."""

    headings: tuple[int, ...] = ()
    """Heading taken *out* of each node; one shorter than `nodes`."""

    cost: float = 0.0
    energy_j: float = 0.0
    duration_ms: int = 0
    expansions: int = 0


def search(
    graph: Graph,
    table: ReservationTable,
    *,
    start: int,
    goals: Iterable[int],
    cost_model: CostModel,
    config: Config,
    start_heading: Heading | None = None,
    moving: bool = False,
    blocked_nodes: frozenset[int] = frozenset(),
    node_penalty: Callable[[int], float] | None = None,
) -> SearchResult:
    """Cheapest conflict-free path from `start` to the nearest of `goals`.

    `blocked_nodes` are impassable outright (a severe obstruction). `node_penalty`
    adds soft cost for entering a node -- the congestion field. Because a penalty
    only ever adds, the heuristic stays admissible.
    """
    goal_set = frozenset(goals)
    if not goal_set:
        return SearchResult(status=PlanStatus.NO_GOAL_AVAILABLE)
    if start in goal_set:
        return SearchResult(
            status=PlanStatus.ALREADY_THERE,
            nodes=(start,),
            arrivals=(table.now,),
        )

    start_interval = table.interval_containing(start, table.now)
    if start_interval is None:
        # A peer already holds the node this robot is standing on. That is a
        # conflict that exists now; routing cannot undo it.
        return SearchResult(status=PlanStatus.START_BLOCKED)

    hops_to_goal = graph.hops_from(goal_set)
    if hops_to_goal[start] == UNREACHABLE:
        return SearchResult(status=PlanStatus.UNREACHABLE)

    kin = cost_model.kinematics
    traverse_ms = kin.cruise_ms(graph.node_spacing)
    stop_and_go_ms = kin.stop_and_go_ms()
    half_stop_ms = kin.half_stop_ms()
    min_hop_cost = cost_model.min_hop_cost()
    arrive_cost = cost_model.arrive()
    max_wait = config.max_wait_ms
    deadline = table.now + config.horizon_ms

    # The one hop shape used on every straight pass, hoisted out of the loop.
    flow_cost = cost_model.hop(quarter_turns=0, wait_ms=0, moving=True)

    def heuristic(node: int) -> float:
        hops = hops_to_goal[node]
        return float("inf") if hops == UNREACHABLE else hops * min_hop_cost

    states = _States(
        node=start,
        heading=NO_HEADING if start_heading is None else int(start_heading),
        interval=_interval_index(table, start, table.now),
        arrival=table.now,
        moving=moving,
    )
    open_heap: list[tuple[float, float, int, int, int, int, int]] = []
    heappush(
        open_heap,
        (heuristic(start), 0.0, table.now, start, states.heading[0], states.interval[0], 0),
    )

    expansions = 0
    best_goal = -1
    best_goal_cost = 0.0

    while open_heap:
        _, g, arrival, node, heading, interval_index, state = heappop(open_heap)

        if node in goal_set:
            best_goal = state
            best_goal_cost = g
            break

        expansions += 1
        if expansions > config.max_expansions:
            return SearchResult(status=PlanStatus.SEARCH_EXHAUSTED, expansions=expansions)

        key = (node, heading, interval_index)
        if states.dominated(key, g, arrival):
            continue
        states.remember(key, g, arrival)

        interval_end = table.safe_intervals(node)[interval_index][1]
        was_moving = states.moving[state]

        for neighbour, step_heading in graph.neighbours(node):
            if neighbour in blocked_nodes:
                continue
            if hops_to_goal[neighbour] == UNREACHABLE:
                continue

            q = 2 if heading == NO_HEADING else quarter_turns(Heading(heading), step_heading)
            extra = node_penalty(neighbour) if node_penalty is not None else 0.0

            # Candidate 1: keep rolling. Only available going straight on, already
            # moving, and only if the node ahead happens to be free right then.
            if q == 0 and was_moving:
                depart = arrival
                landed = depart + traverse_ms
                if landed <= interval_end and landed <= deadline:
                    index = _interval_index_covering(table, neighbour, depart, landed)
                    if index is not None:
                        states.relax(
                            open_heap,
                            heuristic,
                            g=g + flow_cost.cost + extra,
                            energy=states.energy[state] + flow_cost.energy_j,
                            arrival=landed,
                            node=neighbour,
                            heading=int(step_heading),
                            interval_index=index,
                            parent=state,
                            wait=0,
                        )

            # Candidate 2: stop, turn if needed, wait if it helps, then go. The
            # earliest departure is fixed; each safe interval ahead offers at most
            # one useful wait, namely the shortest that reaches it.
            base_penalty = stop_and_go_ms if was_moving else half_stop_ms
            earliest = arrival + base_penalty + kin.turn_ms(q)
            for index, (window_start, window_end) in enumerate(
                table.safe_intervals(neighbour)
            ):
                depart = window_start if window_start > earliest else earliest
                landed = depart + traverse_ms
                if landed > window_end:
                    continue
                if landed > interval_end or landed > deadline:
                    break
                wait = depart - earliest
                if wait > max_wait:
                    continue
                hop = cost_model.hop(quarter_turns=q, wait_ms=wait, moving=was_moving)
                states.relax(
                    open_heap,
                    heuristic,
                    g=g + hop.cost + extra,
                    energy=states.energy[state] + hop.energy_j,
                    arrival=landed,
                    node=neighbour,
                    heading=int(step_heading),
                    interval_index=index,
                    parent=state,
                    wait=wait,
                )

    if best_goal < 0:
        return SearchResult(status=PlanStatus.UNREACHABLE, expansions=expansions)

    nodes: list[int] = []
    arrivals: list[int] = []
    waits: list[int] = []
    cursor = best_goal
    while cursor >= 0:
        nodes.append(states.node[cursor])
        arrivals.append(states.arrival[cursor])
        waits.append(states.wait[cursor])
        cursor = states.parent[cursor]
    nodes.reverse()
    arrivals.reverse()
    waits.reverse()

    # `waits[i]` was recorded on the state the robot arrived *at*; the wait actually
    # happens at the node it left, so shift it back by one.
    hop_waits = tuple(waits[1:])
    headings = tuple(
        int(graph.heading_to(nodes[i], nodes[i + 1])) for i in range(len(nodes) - 1)
    )
    departures = tuple(arrivals[i + 1] - traverse_ms for i in range(len(nodes) - 1))

    total = best_goal_cost + arrive_cost.cost
    return SearchResult(
        status=PlanStatus.OK,
        nodes=tuple(nodes),
        arrivals=tuple(arrivals),
        departures=departures,
        waits=hop_waits,
        headings=headings,
        cost=total,
        energy_j=states.energy[best_goal] + arrive_cost.energy_j,
        duration_ms=arrivals[-1] + arrive_cost.duration_ms - table.now,
        expansions=expansions,
    )


class _States:
    """Search states, held as parallel lists indexed by state id.

    Lists rather than objects because the heap then carries only plain tuples of
    numbers, which compare without any custom ordering and keep the inner loop free
    of attribute lookups. `seen` holds, per `(node, heading, interval)`, the
    non-dominated `(cost, arrival)` pairs already reached: a new state is worth
    exploring only if nothing both cheaper *and* earlier already got there.
    """

    __slots__ = (
        "arrival",
        "energy",
        "heading",
        "interval",
        "moving",
        "node",
        "parent",
        "seen",
        "wait",
    )

    def __init__(
        self, *, node: int, heading: int, interval: int, arrival: int, moving: bool
    ) -> None:
        self.node: list[int] = [node]
        self.heading: list[int] = [heading]
        self.interval: list[int] = [interval]
        self.arrival: list[int] = [arrival]
        self.parent: list[int] = [-1]
        self.wait: list[int] = [0]
        self.moving: list[bool] = [moving]
        self.energy: list[float] = [0.0]
        self.seen: dict[tuple[int, int, int], list[tuple[float, int]]] = {}

    def dominated(self, key: tuple[int, int, int], cost: float, arrival: int) -> bool:
        entries = self.seen.get(key)
        return entries is not None and any(
            seen_cost <= cost and seen_arrival <= arrival
            for seen_cost, seen_arrival in entries
        )

    def remember(self, key: tuple[int, int, int], cost: float, arrival: int) -> None:
        """Record a reached state, dropping anything it dominates."""
        entries = self.seen.get(key, ())
        kept = [
            (seen_cost, seen_arrival)
            for seen_cost, seen_arrival in entries
            if not (cost <= seen_cost and arrival <= seen_arrival)
        ]
        kept.append((cost, arrival))
        self.seen[key] = kept

    def relax(
        self,
        open_heap: list,
        heuristic,
        *,
        g: float,
        energy: float,
        arrival: int,
        node: int,
        heading: int,
        interval_index: int,
        parent: int,
        wait: int,
    ) -> None:
        """Queue a successor, unless something better already reached it."""
        key = (node, heading, interval_index)
        if self.dominated(key, g, arrival):
            return
        h = heuristic(node)
        if h == float("inf"):
            return
        state = len(self.node)
        self.node.append(node)
        self.heading.append(heading)
        self.interval.append(interval_index)
        self.arrival.append(arrival)
        self.parent.append(parent)
        self.wait.append(wait)
        # Every successor is reached by crossing an edge, which ends at speed.
        self.moving.append(True)
        self.energy.append(energy)
        heappush(open_heap, (g + h, g, arrival, node, heading, interval_index, state))


def _interval_index(table: ReservationTable, node: int, t: int) -> int:
    for index, (start, end) in enumerate(table.safe_intervals(node)):
        if start <= t <= end:
            return index
    raise ValueError(f"no safe interval on node {node} at t={t}")


def _interval_index_covering(
    table: ReservationTable, node: int, start: int, end: int
) -> int | None:
    """Index of the safe interval wholly containing `[start, end]`, if any."""
    for index, (window_start, window_end) in enumerate(table.safe_intervals(node)):
        if window_start <= start and end <= window_end:
            return index
        if window_start > start:
            break
    return None
