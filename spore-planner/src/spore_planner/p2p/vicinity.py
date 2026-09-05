"""Who is close enough to be worth talking to.

Reservations only matter between robots that could actually reach the same node,
so there is no reason to announce to the whole region -- let alone the fleet.

The bound is exact rather than a guess. A bot claims at most `k_commit` hops ahead,
so its claimed nodes all lie within `k_commit` hops of it. Two bots' claim sets can
only intersect if they are within `2 * k_commit` hops of each other; further apart,
no claim either makes can possibly touch the other's. That distance is the vicinity.

On this floor plan that keeps the peer plane small: with twenty robots a bot has a
mean of 1.5 neighbours within ten hops and 4 at the 95th percentile, and even at a
hundred robots the mean is under eight. The leader's roster already carries every
region-mate's `latest_node_id` and dialable `address`, so working out who to talk to
costs one bounded breadth-first search and no extra protocol.
"""

from __future__ import annotations

from collections.abc import Iterable, Mapping

from spore_planner.warehouse.graph import Graph


def radius_for(k_commit: int) -> int:
    """Hop distance beyond which two bots' claims cannot possibly overlap."""
    return 2 * k_commit


def neighbours(
    graph: Graph,
    *,
    at_node_id: int,
    peers: Mapping[int, int],
    radius: int,
) -> tuple[int, ...]:
    """Bot ids within `radius` hops, nearest first.

    `peers` maps bot id to the node it was last seen at -- exactly what the
    roster carries. Ties break on bot id so every bot derives the same list.
    """
    if not graph.has_id(at_node_id):
        return ()
    reach = graph.bfs_within([graph.index(at_node_id)], radius)
    found: list[tuple[int, int]] = []
    for bot_id, node_id in peers.items():
        if not graph.has_id(node_id):
            continue
        hops = reach.get(graph.index(node_id))
        if hops is not None:
            found.append((hops, bot_id))
    return tuple(bot_id for _, bot_id in sorted(found))


def in_claim_range(
    graph: Graph,
    *,
    claimed_node_ids: Iterable[int],
    peers: Mapping[int, int],
    reach_hops: int,
) -> tuple[int, ...]:
    """Bot ids whose claims could touch mine, given what I have actually claimed.

    Tighter than `neighbours` and still exact. A peer's claims all lie within
    `reach_hops` of where it stands, so they can only meet mine if one of the nodes
    *I* hold is within that distance of it. Measuring from my real claim set rather
    than from a ball around me is a much smaller question to ask -- eight specific
    nodes instead of everything within eight hops in every direction -- and on this
    map it roughly halves the number of bots worth announcing to.
    """
    sources = [graph.index(n) for n in claimed_node_ids if graph.has_id(n)]
    if not sources:
        return ()
    reach = graph.bfs_within(sources, reach_hops)
    found: list[tuple[int, int]] = []
    for bot_id, node_id in peers.items():
        if not graph.has_id(node_id):
            continue
        hops = reach.get(graph.index(node_id))
        if hops is not None:
            found.append((hops, bot_id))
    return tuple(bot_id for _, bot_id in sorted(found))


def within(
    graph: Graph, *, at_node_id: int, node_ids: Iterable[int], radius: int
) -> frozenset[int]:
    """Which of `node_ids` lie within `radius` hops -- used to filter claims."""
    if not graph.has_id(at_node_id):
        return frozenset()
    reach = graph.bfs_within([graph.index(at_node_id)], radius)
    return frozenset(
        node_id
        for node_id in node_ids
        if graph.has_id(node_id) and graph.index(node_id) in reach
    )
