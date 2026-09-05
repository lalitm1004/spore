"""Soft traffic cost, for the part of the route reservations do not cover.

A peer's `res[]` reaches only `K_COMMIT` hops ahead. Past that the planner has no
hard constraints at all, and a route planned purely on hard constraints will happily
commit its tail to an aisle that is visibly filling up. This module supplies the
gentler signal for that stretch: where the robots are, what the gossip says about
each region, how tightly the region is laned, and what has been reported blocked.

None of it forbids anything -- it only makes some nodes dearer than others, which is
why the A* heuristic stays admissible: a penalty can only ever add to a hop's cost,
never subtract, so a lower bound computed without penalties remains a lower bound.

The one exception is a severe obstruction, which is a hard block: past a certain
level the node is not expensive, it is impassable.

Penalties are denominated as fractions of one straight hop, so `congestion_weight`
reads as "a fully congested node is worth this much of an extra hop".
"""

from __future__ import annotations

from collections.abc import Iterable
from dataclasses import dataclass, field

from spore_planner.planner.types import Config, Obstruction, PeerView, RegionGossip
from spore_planner.warehouse.graph import Graph
from spore_planner.warehouse.map import Density

DENSITY_FACTOR: dict[Density, float] = {
    Density.DENSE: 1.0,
    Density.MEDIUM: 0.6,
    Density.SPARSE: 0.3,
}
"""How much a region's lane packing amplifies congestion. On the real map only the
grid field is `dense`; the ring highway and crossdock are `medium`."""


@dataclass(frozen=True, slots=True)
class CongestionField:
    """Per-node soft cost, plus the nodes that are outright impassable."""

    penalties: dict[int, float] = field(default_factory=dict)
    blocked: frozenset[int] = frozenset()

    def __call__(self, node: int) -> float:
        return self.penalties.get(node, 0.0)

    @property
    def busiest(self) -> tuple[int, ...]:
        """Nodes carrying a penalty, dearest first -- for diagnostics."""
        return tuple(
            node
            for node, _ in sorted(self.penalties.items(), key=lambda kv: (-kv[1], kv[0]))
        )


def build(
    graph: Graph,
    *,
    config: Config,
    hop_cost: float,
    peers: Iterable[PeerView] = (),
    gossip: Iterable[RegionGossip] = (),
    obstructions: Iterable[Obstruction] = (),
    exclude_bot_id: int | None = None,
) -> CongestionField:
    """Assemble the soft field for one plan call.

    `hop_cost` is the cost of one straight hop, used to scale every penalty into the
    same units the search works in.
    """
    penalties: dict[int, float] = {}
    blocked: set[int] = set()

    def add(node: int, amount: float) -> None:
        if amount > 0.0:
            penalties[node] = penalties.get(node, 0.0) + amount

    # -- where the robots actually are ---------------------------------------
    sources: list[int] = []
    for peer in sorted(peers, key=lambda p: p.bot_id):
        if exclude_bot_id is not None and peer.bot_id == exclude_bot_id:
            continue
        for node_id in _positions_of(peer):
            if graph.has_id(node_id):
                sources.append(graph.index(node_id))

    decay = max(1.0, config.congestion_decay_hops)
    if sources:
        for node, hops in graph.bfs_within(sources, int(decay)).items():
            nearness = 1.0 - hops / decay
            add(node, config.congestion_weight * hop_cost * nearness * _density(graph, node))

    # -- second-hand summaries of whole regions ------------------------------
    region_load = {report.region_id: report.load for report in gossip}
    per_node: dict[int, float] = {}
    for report in gossip:
        for node_id, load in report.edge_load.items():
            if graph.has_id(node_id):
                per_node[graph.index(node_id)] = max(
                    per_node.get(graph.index(node_id), 0.0), load
                )

    if region_load:
        for node in range(graph.n):
            load = region_load.get(graph.region_id[node], 0.0)
            if load > 0.0:
                add(node, config.congestion_weight * hop_cost * load * _density(graph, node))
    for node, load in per_node.items():
        add(node, config.congestion_weight * hop_cost * load)

    # -- reported blockages --------------------------------------------------
    for obstruction in obstructions:
        if not graph.has_id(obstruction.node_id):
            continue
        node = graph.index(obstruction.node_id)
        if obstruction.level >= config.obstruction_block_level:
            blocked.add(node)
        else:
            # Below the threshold the lane is passable but unattractive. Scaled well
            # above the congestion weight so a robot only pushes through a reported
            # obstruction when the alternative is genuinely much worse.
            add(node, hop_cost * obstruction.level * 2.0)

    return CongestionField(penalties=penalties, blocked=frozenset(blocked))


def _positions_of(peer: PeerView) -> tuple[int, ...]:
    """Node ids a peer is at or between."""
    if peer.edge is not None:
        return peer.edge
    if peer.node_id is not None:
        return (peer.node_id,)
    return ()


def _density(graph: Graph, node: int) -> float:
    return DENSITY_FACTOR[graph.density[node]]
