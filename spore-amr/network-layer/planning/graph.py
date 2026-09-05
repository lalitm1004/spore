"""The search graph: adjacency with headings, node kinds, and hop distances.

WHAT
    `Graph` wraps the one loaded `WarehouseMap` and adds what the search needs
    on top of it: the heading of every edge, node kinds as enums, per-node
    region density, and a multi-source distance table for the A* heuristic.

WHERE
    Built once at boot from `bot.Bot.map`. Every other module in `planning/`
    takes a `Graph`; none of them touches the map document.

WHY
    Two things the search cannot do without. **Headings**, because robots rotate
    in place, so the cost of a hop depends on the direction it is entered from.
    And an **exact distance heuristic**: on a network this sparse -- 881 nodes
    carrying 952 edges -- the true remaining hop count keeps A* inside the real
    corridor instead of flooding the floor, taking a median 26-hop route in
    about 84 expansions.

    It deliberately holds no copy of the map. `WarehouseMap` owns the node and
    edge data and this reads through to it, because a second copy on a robot
    that has to fit in a Pi's memory is a second copy too many.

HOW
    Dense indices `0..n-1` throughout, borrowed from `WarehouseMap` so both
    agree, with ids translated only at the boundary. Node ids are not
    contiguous -- a window of the real map runs 37..252 with gaps -- so the
    translation is load-bearing, not a formality.

    `hops_from` memoises BFS per goal *set*, so a class goal ("any charger")
    gets an admissible heuristic from a single multi-source sweep. The cache is
    an `array("H")` per entry, ~1.7 KB, bounded by an LRU.
"""

from __future__ import annotations

from array import array
from collections import OrderedDict, deque
from collections.abc import Callable, Iterable

from planning.geometry import Density, Heading, NodeType, Position, heading_between
from warehouse.map import UNREACHABLE, WarehouseMap


class Graph:
    """Adjacency, per-node attributes and cached hop distances for one map."""

    def __init__(self, wmap: WarehouseMap, hops_cache_size: int = 32) -> None:
        self.map = wmap
        self.n = wmap.n
        self.ids = wmap.ids
        self.node_spacing = wmap.node_spacing

        self.position: tuple[Position, ...] = tuple(
            Position(*wmap.position_of(node_id)) for node_id in wmap.ids
        )
        self.node_type: tuple[NodeType, ...] = tuple(
            NodeType(wmap.type_of(node_id)) for node_id in wmap.ids
        )
        self.region_id: tuple[int, ...] = tuple(
            wmap.region_of(node_id) or 0 for node_id in wmap.ids
        )
        self.density: tuple[Density, ...] = tuple(
            Density(wmap.density_of(node_id)) for node_id in wmap.ids
        )

        # Adjacency with the heading of each step, sorted so expansion order --
        # and every tie-break downstream -- is identical on every bot.
        self._adj: tuple[tuple[tuple[int, Heading], ...], ...] = tuple(
            tuple(
                sorted(
                    (
                        (v, heading_between(self.position[u], self.position[v]))
                        for v in wmap.adjacency()[u]
                    ),
                    key=lambda item: (int(item[1]), item[0]),
                )
            )
            for u in range(self.n)
        )

        by_type: dict[NodeType, list[int]] = {t: [] for t in NodeType}
        for i, node_type in enumerate(self.node_type):
            by_type[node_type].append(i)
        self._by_type = {t: tuple(members) for t, members in by_type.items()}

        self._dist_cache: OrderedDict[frozenset[int], array] = OrderedDict()
        self._dist_cache_size = max(1, hops_cache_size)

    # ---- Identity -----------------------------------------------------------

    def index(self, node_id: int) -> int:
        try:
            return self.map.index(node_id)
        except KeyError:
            raise LookupError(f"node id {node_id} is not in this map") from None

    def id_of(self, i: int) -> int:
        return self.ids[i]

    def has_id(self, node_id: int) -> bool:
        return self.map.has(node_id)

    # ---- Topology -----------------------------------------------------------

    def neighbours(self, i: int) -> tuple[tuple[int, Heading], ...]:
        """`(neighbour, heading_to_reach_it)` pairs, in deterministic order."""
        return self._adj[i]

    def degree(self, i: int) -> int:
        return len(self._adj[i])

    def nodes_of_type(self, node_type: NodeType) -> tuple[int, ...]:
        return self._by_type[node_type]

    def heading_to(self, i: int, j: int) -> Heading:
        for neighbour, heading in self._adj[i]:
            if neighbour == j:
                return heading
        raise LookupError(f"nodes {self.id_of(i)} and {self.id_of(j)} are not adjacent")

    def are_adjacent(self, i: int, j: int) -> bool:
        return any(neighbour == j for neighbour, _ in self._adj[i])

    # ---- Distances ----------------------------------------------------------

    def hops_from(self, sources: int | Iterable[int]) -> array:
        """Exact hop distance from every node to the nearest of `sources`.

        Multi-source when given several, which is how a class goal ("any
        charger") gets an admissible heuristic from one sweep. Bounded LRU:
        goals repeat heavily in practice, but not without limit.
        """
        key = frozenset((sources,) if isinstance(sources, int) else sources)
        cached = self._dist_cache.get(key)
        if cached is not None:
            self._dist_cache.move_to_end(key)
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
        if len(self._dist_cache) > self._dist_cache_size:
            self._dist_cache.popitem(last=False)
        return dist

    def clear_distance_cache(self) -> None:
        self._dist_cache.clear()

    def bfs_within(self, sources: Iterable[int], max_depth: int) -> dict[int, int]:
        """Hop distance to the nearest source, out to `max_depth` only.

        Deliberately not cached, unlike `hops_from`: callers pass sets that
        change every tick -- where the peers are standing -- and memoising those
        would grow without bound. Bounding the depth keeps each call cheap.
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

        Ties break on the lower dense index, so the answer is stable.
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
        return f"<Graph {self.n} nodes>"
