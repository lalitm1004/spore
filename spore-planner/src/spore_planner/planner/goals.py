"""Turning a class goal into a set of candidate nodes.

`CHARGE` and `PARK` do not name a node -- they name a kind of node, and picking
which one is a routing decision, not a mission one. The nearest charger by hop count
is often the wrong answer: it may be behind a queue, already spoken for, or in a
region the gossip says has nothing free.

Rather than score candidates separately and re-plan for each, every candidate is
seeded into a single multi-goal search and the preference is expressed as a penalty
on *entering* the candidate. Because the search pays a node's penalty exactly once,
when it arrives, that lands the bias precisely on the goal -- and one search returns
the best trade-off of travel and desirability, instead of N searches returning N
travel costs to be reconciled afterwards.

Every charger and parking bay on the real map is a dead-end spur, so reaching one
always costs a turn in and a reversal out. The search charges that on its own; there
is nothing to add here beyond the preference.
"""

from __future__ import annotations

from collections.abc import Iterable
from dataclasses import dataclass

from spore_planner.planner.types import Goal, GoalKind, PeerView, RegionGossip
from spore_planner.warehouse.graph import Graph
from spore_planner.warehouse.map import NodeType

GOAL_NODE_TYPE: dict[GoalKind, NodeType] = {
    GoalKind.CHARGE: NodeType.CH,
    GoalKind.PARK: NodeType.PK,
}

CLAIMED_PENALTY = 3.0
"""Cost, in straight hops, of choosing a bay a peer is already heading for. High
enough to pick another when one exists, low enough to still queue if none does."""

REGION_FULL_PENALTY = 2.0
"""Cost, in straight hops, of a bay whose region reports nothing free."""


@dataclass(frozen=True, slots=True)
class ResolvedGoal:
    candidates: frozenset[int]
    penalties: dict[int, float]
    rationale: str

    @property
    def count(self) -> int:
        return len(self.candidates)


def resolve(
    graph: Graph,
    goal: Goal,
    *,
    hop_cost: float,
    peers: Iterable[PeerView] = (),
    gossip: Iterable[RegionGossip] = (),
    exclude_bot_id: int | None = None,
) -> ResolvedGoal:
    """Candidate goal nodes, with a preference penalty on each."""
    if goal.kind is GoalKind.NODE:
        assert goal.node_id is not None
        if not graph.has_id(goal.node_id):
            return ResolvedGoal(frozenset(), {}, f"node {goal.node_id} is not on this map")
        return ResolvedGoal(
            frozenset({graph.index(goal.node_id)}), {}, f"explicit node {goal.node_id}"
        )

    node_type = GOAL_NODE_TYPE[goal.kind]
    candidates = set(graph.nodes_of_type(node_type))
    if not candidates:
        return ResolvedGoal(frozenset(), {}, f"no {node_type} nodes on this map")

    penalties: dict[int, float] = {}

    # A peer whose committed path ends on one of these bays is going there. Treating
    # that as merely expensive rather than forbidden means a robot still queues for
    # the last charger when every other one is taken too.
    claimed: set[int] = set()
    for peer in sorted(peers, key=lambda p: p.bot_id):
        if exclude_bot_id is not None and peer.bot_id == exclude_bot_id:
            continue
        if not peer.reservations:
            continue
        destination = peer.reservations[-1].node_id
        if graph.has_id(destination):
            node = graph.index(destination)
            if node in candidates:
                claimed.add(node)
    for node in claimed:
        penalties[node] = penalties.get(node, 0.0) + CLAIMED_PENALTY * hop_cost

    # Gossip only speaks about chargers, so it shapes CHARGE and leaves PARK alone.
    if goal.kind is GoalKind.CHARGE:
        for report in gossip:
            if report.chargers_free > 0:
                continue
            for node in candidates:
                if graph.region_id[node] == report.region_id:
                    penalties[node] = penalties.get(node, 0.0) + REGION_FULL_PENALTY * hop_cost

    rationale = (
        f"{len(candidates)} {node_type} candidates, "
        f"{len(claimed)} already targeted by peers"
    )
    return ResolvedGoal(frozenset(candidates), penalties, rationale)
