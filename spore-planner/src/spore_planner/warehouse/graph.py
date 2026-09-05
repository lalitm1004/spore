"""Navigable graph built from a `WarehouseMap`.

Two things matter for the planner's hot loop:

*Dense indices.* The map's node ids happen to be contiguous today, but nothing in
the schema promises that. Internally every node is a dense index `0..n-1`; external
ids are translated only at the API boundary, so the search never pays a dict lookup
per expansion and never breaks if the generator renumbers.

*A cached exact heuristic.* `hops_from` runs a BFS and memoises the result. Because
the graph is undirected, a BFS from the goal gives the exact hop distance from every
node to that goal -- a perfect heuristic for A*, which matters a great deal on a
network this sparse (881 nodes, average degree 2.16): the search follows the true
corridor instead of flooding the floor. Goals repeat heavily in practice (chargers,
parking bays, transfer points), so the cache is warm after the first plan to each.
"""

from __future__ import annotations

from array import array
from collections import deque
from collections.abc import Callable, Iterable

from spore_planner.warehouse.map import (
    Density,
    Heading,
    NodeType,
    Position,
    WarehouseMap,
    heading_between,
)

UNREACHABLE = 0xFFFF
"""Sentinel hop-distance for nodes with no path to the sources."""


class Graph:
    """Adjacency, per-node attributes and cached hop distances for one map."""

    __slots__ = (
        "_adj",
        "_by_type",
        "_dist_cache",
        "_index",
        "density",
        "ids",
        "map",
        "n",
        "node_spacing",
        "node_type",
        "position",
        "region_id",
    )

    def __init__(self, wmap: WarehouseMap) -> None:
        nodes = sorted(wmap.nodes, key=lambda node: node.id)
        if len(nodes) > UNREACHABLE:
            raise ValueError(
                f"graph has {len(nodes)} nodes, which exceeds the {UNREACHABLE} the "
                "uint16 distance cache can address"
            )

        self.map = wmap
        self.n = len(nodes)
        self.node_spacing = wmap.node_spacing
        self.ids: tuple[int, ...] = tuple(node.id for node in nodes)
        self._index: dict[int, int] = {node.id: i for i, node in enumerate(nodes)}
        self.node_type: tuple[NodeType, ...] = tuple(node.node_type for node in nodes)
        self.region_id: tuple[int, ...] = tuple(node.region_id for node in nodes)
        self.position: tuple[Position, ...] = tuple(node.position for node in nodes)

        density_by_region = {region.id: region.density for region in wmap.regions}
        self.density: tuple[Density, ...] = tuple(
            density_by_region[node.region_id] for node in nodes
        )

        # Adjacency as a tuple of (neighbour, heading) tuples, indexed by dense id.
        # Sorted so expansion order -- and therefore every tie-break downstream -- is
        # deterministic across processes and runs.
        buckets: list[list[tuple[int, Heading]]] = [[] for _ in range(self.n)]
        for edge in wmap.edges:
            u, v = self._index[edge.a], self._index[edge.b]
            buckets[u].append((v, heading_between(self.position[u], self.position[v])))
            buckets[v].append((u, heading_between(self.position[v], self.position[u])))
        self._adj: tuple[tuple[tuple[int, Heading], ...], ...] = tuple(
            tuple(sorted(bucket, key=lambda item: (int(item[1]), item[0]))) for bucket in buckets
        )

        by_type: dict[NodeType, list[int]] = {node_type: [] for node_type in NodeType}
        for i, node_type in enumerate(self.node_type):
            by_type[node_type].append(i)
        self._by_type: dict[NodeType, tuple[int, ...]] = {
            node_type: tuple(members) for node_type, members in by_type.items()
        }

        self._dist_cache: dict[frozenset[int], array] = {}

    # -- index translation ---------------------------------------------------

    def index(self, node_id: int) -> int:
        """Dense index for an external node id."""
        try:
            return self._index[node_id]
        except KeyError:
            raise LookupError(f"node id {node_id} is not in this map") from None

    def id_of(self, i: int) -> int:
        """External node id for a dense index."""
        return self.ids[i]

    def has_id(self, node_id: int) -> bool:
        return node_id in self._index

    # -- topology accessors --------------------------------------------------

    def neighbours(self, i: int) -> tuple[tuple[int, Heading], ...]:
        """`(neighbour, heading_to_reach_it)` pairs, in deterministic order."""
        return self._adj[i]

    def degree(self, i: int) -> int:
        return len(self._adj[i])

    def nodes_of_type(self, node_type: NodeType) -> tuple[int, ...]:
        """Dense indices of every node of `node_type`, ascending."""
        return self._by_type[node_type]

    def heading_to(self, i: int, j: int) -> Heading:
        """Heading to step from `i` to an adjacent `j`."""
        for neighbour, heading in self._adj[i]:
            if neighbour == j:
                return heading
        raise LookupError(f"nodes {self.id_of(i)} and {self.id_of(j)} are not adjacent")

    def are_adjacent(self, i: int, j: int) -> bool:
        return any(neighbour == j for neighbour, _ in self._adj[i])

    # -- distances -----------------------------------------------------------

    def hops_from(self, sources: int | Iterable[int]) -> array:
        """Exact hop distance from every node to the nearest of `sources`.

        Multi-source when given several, which is how a class goal ("any charger")
        gets an admissible heuristic from a single BFS. Results are memoised on the
        graph, so planners sharing a `Graph` share the work.
        """
        key = frozenset((sources,) if isinstance(sources, int) else sources)
        cached = self._dist_cache.get(key)
        if cached is not None:
            return cached

        if not key:
            raise ValueError("hops_from requires at least one source")
        for source in key:
            if not 0 <= source < self.n:
                raise IndexError(f"source index {source} out of range for {self.n} nodes")

        dist = array("H", bytes(2 * self.n))
        for i in range(self.n):
            dist[i] = UNREACHABLE
        queue = deque(sorted(key))
        for source in key:
            dist[source] = 0
        adj = self._adj
        while queue:
            u = queue.popleft()
            d = dist[u] + 1
            for v, _ in adj[u]:
                if dist[v] == UNREACHABLE:
                    dist[v] = d
                    queue.append(v)

        self._dist_cache[key] = dist
        return dist

    def clear_distance_cache(self) -> None:
        self._dist_cache.clear()

    def bfs_within(self, sources: Iterable[int], max_depth: int) -> dict[int, int]:
        """Hop distance to the nearest source, out to `max_depth` only.

        Deliberately *not* cached, unlike `hops_from`. Callers pass sets that change
        every tick -- where the peers are standing, say -- and memoising those would
        grow without bound. Bounding the depth keeps each call cheap instead.
        """
        if max_depth < 0:
            raise ValueError("max_depth must be non-negative")
        depth: dict[int, int] = {}
        frontier = []
        for source in sorted(set(sources)):
            if 0 <= source < self.n and source not in depth:
                depth[source] = 0
                frontier.append(source)
        for d in range(1, max_depth + 1):
            if not frontier:
                break
            nxt: list[int] = []
            for u in frontier:
                for v, _ in self._adj[u]:
                    if v not in depth:
                        depth[v] = d
                        nxt.append(v)
            frontier = nxt
        return depth

    def nearest(
        self, source: int, predicate: Callable[[int], bool], max_depth: int
    ) -> tuple[int, int] | None:
        """Closest node to `source` satisfying `predicate`, as `(node, hops)`.

        Ties break on the lower dense index, so the answer is stable. Returns None
        if nothing matches within `max_depth`.
        """
        if predicate(source):
            return (source, 0)
        seen = {source}
        frontier = [source]
        for d in range(1, max_depth + 1):
            nxt: list[int] = []
            for u in frontier:
                for v, _ in self._adj[u]:
                    if v not in seen:
                        seen.add(v)
                        nxt.append(v)
            matches = [v for v in sorted(nxt) if predicate(v)]
            if matches:
                return (matches[0], d)
            frontier = nxt
            if not frontier:
                break
        return None

    def __repr__(self) -> str:
        return f"<Graph {self.n} nodes, {len(self.map.edges)} edges>"
