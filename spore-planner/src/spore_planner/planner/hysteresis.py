"""Deciding whether a freshly planned path is worth switching to.

The planner runs on every heartbeat tick. Traffic shifts constantly, and two routes
around a busy aisle are often within a few percent of each other, so a planner that
simply took the cheapest answer each tick would flip between them -- churning
reservations, and making the robot's intent unreadable to the peers trying to plan
around it.

So a new path has to earn its place. It is taken immediately if the current one is
no longer valid, and otherwise only if it is clearly better for several ticks
running. `stable_for` is the count of consecutive ticks an alternative has been
winning; the caller carries it between calls.
"""

from __future__ import annotations

from dataclasses import dataclass

from spore_planner.planner.intervals import ReservationTable
from spore_planner.planner.types import Config, Goal, GoalKind, Path
from spore_planner.warehouse.graph import Graph


@dataclass(frozen=True, slots=True)
class Decision:
    replace: bool
    stable_for: int
    reason: str


def decide(
    graph: Graph,
    table: ReservationTable,
    *,
    current: Path | None,
    candidate: Path | None,
    at_node_id: int,
    goal: Goal | None = None,
    blocked: frozenset[int] = frozenset(),
    config: Config,
    stable_for: int = 0,
) -> Decision:
    """Whether to adopt `candidate` in place of `current`."""
    if candidate is None:
        return Decision(replace=False, stable_for=0, reason="no candidate path")
    if current is None or not current.hops:
        return Decision(replace=True, stable_for=0, reason="no current path")

    invalid = _invalidation(
        graph,
        table,
        current=current,
        at_node_id=at_node_id,
        goal=goal,
        blocked=blocked,
    )
    if invalid is not None:
        return Decision(replace=True, stable_for=0, reason=invalid)

    threshold = current.cost * (1.0 - config.improvement_margin)
    if candidate.cost >= threshold:
        return Decision(
            replace=False, stable_for=0, reason="current path is still competitive"
        )

    streak = stable_for + 1
    if streak >= config.stable_ticks:
        return Decision(
            replace=True,
            stable_for=0,
            reason=(
                f"cheaper by {(1 - candidate.cost / current.cost):.1%} "
                f"for {streak} ticks"
            ),
        )
    return Decision(
        replace=False,
        stable_for=streak,
        reason=f"better, but only for {streak} of {config.stable_ticks} ticks",
    )


def _invalidation(
    graph: Graph,
    table: ReservationTable,
    *,
    current: Path,
    at_node_id: int,
    goal: Goal | None,
    blocked: frozenset[int],
) -> str | None:
    """Why the current path can no longer be followed, if it cannot."""
    # A class goal names a kind of node, not a node, so heading for a different
    # charger than this tick's search preferred is not a reason to tear up the path.
    if goal is not None and goal.kind is GoalKind.NODE and current.goal_node_id != goal.node_id:
        return "goal changed"

    position = current.index_of(at_node_id)
    if position is None:
        return "robot is no longer on its planned path"

    # Validate the committed window *ahead of where the robot now is*, not the one
    # it started with. A robot part way down a path has already driven through the
    # leading hops; re-checking those says nothing, while the hops it is about to
    # claim are exactly the ones a peer may have taken in the meantime.
    for hop in current.hops[position : position + current.committed]:
        if not graph.has_id(hop.node_id):
            return f"node {hop.node_id} is not on this map"
        node = graph.index(hop.node_id)
        if node in blocked:
            return f"node {hop.node_id} is obstructed"
        if not table.is_free(node, hop.t_in, hop.t_out):
            blockers = table.blockers(node, hop.t_in, hop.t_out)
            return f"node {hop.node_id} was claimed by {list(blockers)}"
    return None
