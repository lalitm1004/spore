"""Where to step aside, when the plan says the robot is going to be waiting.

The map has 15 yield bays, and they sit in only two of its fourteen regions --
receiving/inspection, and pick/pack/sort. The ring highway, the grid field and the
crossdock lane, which carry the heaviest traffic, have none at all. A suggester that
only ever offered a `YI` node would therefore return nothing across most of the
floor, exactly where congestion is worst.

So the cascade widens as it fails:

1. a real yield bay, if one is close;
2. otherwise the nearest junction -- anywhere a peer can actually get past, which
   needs no dedicated bay and exists all over the map;
3. otherwise a parking or charging spur, borrowed as a pull-over. Flagged as such,
   because a robot loitering in a charger bay may block a peer that needs it.

This is advice. Whether to take it -- and who between two robots should be the one
to give way -- is the business of the layer that owns the priority ordering.
"""

from __future__ import annotations

from spore_planner.planner.types import YieldSuggestion
from spore_planner.warehouse.graph import Graph
from spore_planner.warehouse.map import NodeType
from spore_planner.warehouse.topology import Topology

SEARCH_RADIUS_HOPS = 8
"""How far to look before giving up. Beyond this the detour to reach the bay costs
more than the wait it saves."""

_PULL_OVER_TYPES = (NodeType.PK, NodeType.CH)


def suggest(
    graph: Graph,
    topology: Topology,
    *,
    at_node: int,
    avoid: frozenset[int] = frozenset(),
    radius: int = SEARCH_RADIUS_HOPS,
    reason: str = "",
) -> YieldSuggestion | None:
    """Nearest place to get out of the way, or None if there is nowhere near."""

    def usable(node: int) -> bool:
        return node not in avoid

    yield_bay = graph.nearest(
        at_node, lambda n: graph.node_type[n] is NodeType.YI and usable(n), radius
    )
    if yield_bay is not None:
        node, hops = yield_bay
        return YieldSuggestion(
            node_id=graph.id_of(node), kind="YI", hops_away=hops, reason=reason
        )

    junction = graph.nearest(
        at_node, lambda n: topology.is_junction(n) and usable(n), radius
    )
    if junction is not None:
        node, hops = junction
        return YieldSuggestion(
            node_id=graph.id_of(node),
            kind="JUNCTION",
            hops_away=hops,
            reason=reason or "no yield bay in range; a junction is passable",
        )

    pull_over = graph.nearest(
        at_node, lambda n: graph.node_type[n] in _PULL_OVER_TYPES and usable(n), radius
    )
    if pull_over is not None:
        node, hops = pull_over
        kind = graph.node_type[node].value
        return YieldSuggestion(
            node_id=graph.id_of(node),
            kind=kind,
            hops_away=hops,
            reason=(
                f"no yield bay or junction in range; borrowing a {kind} bay, which "
                "may block a robot that needs it"
            ),
        )
    return None
