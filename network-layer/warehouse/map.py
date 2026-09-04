"""The static warehouse map — the navigable graph of QR nodes.

WHAT
    `WarehouseMap` loads `warehouse-layout.json` (schema:
    `spore-amr/shared/schemas/warehouse-map.schema.json`) and answers the
    two questions the network layer has about geography:
      * which region is node N in?              (`region_of`)
      * how far is node A from node B / region R? (`distance`, `region_distance`)

WHERE
    Loaded once at boot by `bot.Bot` from `config.WAREHOUSE_MAP`. Used by
    `bus.jobs.Dispatcher` to rank free bots by distance to a pickup node and
    to choose the nearest region to forward a job to.

WHY
    Job dispatch needs "nearest". Positions alone mislead in a warehouse —
    two nodes 1 m apart across a storage block can be 40 m of driving. Hop
    count over the real edges is the honest metric, and the graph is small
    (≈900 nodes) so all-pairs BFS on demand with a cache is fine.

HOW
    * Adjacency from `edges` (undirected). `distance(a, b)` = BFS hops,
      cached per source node. `math.inf` if unreachable or unknown.
    * `region_distance(node, region)` = min hops from `node` to any node
      in `region`.
    * If the file is missing we degrade to `NullMap`: every distance is 0
      and `region_of` is unknown (None). Dispatch then falls back to battery
      and id ordering, and forwarding tries regions in id order. The fleet
      keeps working; it just can't rank by geography.
"""
from __future__ import annotations

import json
import logging
import math
from collections import deque
from pathlib import Path

log = logging.getLogger(__name__)


class NullMap:
    """Stand-in when no map file is available. See module docstring."""

    def region_of(self, node_id: int) -> int | None:
        return None

    def distance(self, a: int, b: int) -> float:
        return 0.0

    def region_distance(self, node_id: int, region_id: int) -> float:
        return 0.0

    def nodes_in(self, region_id: int) -> list[int]:
        return []


class WarehouseMap:
    def __init__(self, data: dict) -> None:
        self._region_of: dict[int, int] = {n["id"]: n["region_id"] for n in data["nodes"]}
        self._adj: dict[int, list[int]] = {n["id"]: [] for n in data["nodes"]}
        for e in data["edges"]:
            self._adj[e["a"]].append(e["b"])
            self._adj[e["b"]].append(e["a"])
        self._by_region: dict[int, list[int]] = {}
        for node, region in self._region_of.items():
            self._by_region.setdefault(region, []).append(node)
        self._bfs_cache: dict[int, dict[int, int]] = {}

    @classmethod
    def load(cls, path: str | Path) -> "WarehouseMap | NullMap":
        p = Path(path)
        if not p.is_file():
            log.warning("warehouse map %s not found; geography-blind dispatch", p)
            return NullMap()
        with p.open() as f:
            m = cls(json.load(f))
        log.info("warehouse map loaded: %d nodes, %d regions", len(m._region_of), len(m._by_region))
        return m

    # ---- Queries ------------------------------------------------------------

    def region_of(self, node_id: int) -> int | None:
        return self._region_of.get(node_id)

    def nodes_in(self, region_id: int) -> list[int]:
        return list(self._by_region.get(region_id, []))

    def _hops_from(self, src: int) -> dict[int, int]:
        cached = self._bfs_cache.get(src)
        if cached is not None:
            return cached
        dist = {src: 0}
        q = deque([src])
        while q:
            u = q.popleft()
            for v in self._adj.get(u, ()):
                if v not in dist:
                    dist[v] = dist[u] + 1
                    q.append(v)
        self._bfs_cache[src] = dist
        return dist

    def distance(self, a: int, b: int) -> float:
        """Driving distance in QR hops (edges are all `node_spacing` long)."""
        if a not in self._adj or b not in self._adj:
            return math.inf
        return float(self._hops_from(a).get(b, math.inf))

    def region_distance(self, node_id: int, region_id: int) -> float:
        """Hops from `node_id` to the nearest node of `region_id`."""
        if node_id not in self._adj:
            return math.inf
        hops = self._hops_from(node_id)
        best = math.inf
        for n in self._by_region.get(region_id, ()):
            d = hops.get(n)
            if d is not None and d < best:
                best = d
        return float(best)
