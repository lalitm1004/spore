"""Who is close enough to be worth telling.

WHAT
    `in_claim_range` — of the bots in the roster, which ones could possibly want
    a node we are holding.

WHERE
    Called by `reservations.sender` once per announce, with positions taken from
    the roster `peers.table.PeerTable` already keeps.

WHY
    Announcing to the whole region would work and would be wasteful. Reservations
    only matter between bots whose claims can actually meet, and on this floor
    plan that is two or three bots out of twenty rather than nineteen.

HOW
    The test is exact rather than a radius guess. A bot's claims never stretch
    more than `reach_hops` from where it stands, so it can only contest a node we
    hold if it is within that distance *of that node*. Measuring from the nodes we
    actually hold — a handful of specific nodes — is a much smaller question than
    asking who is within reach in every direction.

    Distances come from `warehouse.map.WarehouseMap.distance`, which is already
    BFS-cached per source node. With no map file that class degrades to `NullMap`,
    where every distance is 0 and therefore every bot is a neighbour: chattier,
    still correct, and the same way job dispatch degrades without geography.
"""
from __future__ import annotations

from collections.abc import Iterable, Mapping


def in_claim_range(
    warehouse_map,
    *,
    claimed_node_ids: Iterable[int],
    peers: Mapping[int, int],
    reach_hops: int,
) -> tuple[int, ...]:
    """Bot ids whose claims could touch ours, nearest first.

    `peers` maps bot id to the node it was last seen at — exactly what the roster
    carries. Ties break on bot id so the list is stable.
    """
    held = list(claimed_node_ids)
    if not held:
        return ()

    found: list[tuple[float, int]] = []
    for bot_id, node_id in peers.items():
        nearest = min((warehouse_map.distance(node, node_id) for node in held), default=float("inf"))
        if nearest <= reach_hops:
            found.append((nearest, bot_id))
    return tuple(bot_id for _, bot_id in sorted(found))
