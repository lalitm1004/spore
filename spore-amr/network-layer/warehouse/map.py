"""The static warehouse map — the navigable graph of QR nodes.

WHAT
    `WarehouseMap` loads `warehouse-layout.json` (schema:
    `spore-amr/shared/schemas/warehouse-map.schema.json`) and is the single
    in-memory copy of the floor. It answers the geography questions the fleet
    asks:
      * which region is node N in?                (`region_of`)
      * how far is node A from node B / region R? (`distance`, `region_distance`)
      * what is adjacent to N, and where is it?   (`neighbours`, `position_of`)

WHERE
    Loaded once at boot by `bot.Bot` from `config.WAREHOUSE_MAP`. `bus.jobs`
    ranks free bots by distance to a pickup; `planning.graph` builds its search
    structures on top of this object rather than re-reading the file, so the
    node and edge data exists once per process.

WHY
    Job dispatch needs "nearest". Positions alone mislead in a warehouse -- two
    nodes a metre apart across a storage block can be forty metres of driving --
    so hop count over the real edges is the honest metric.

    If the file is missing we degrade to `NullMap`: every distance 0, no nodes,
    unknown regions. Dispatch falls back to battery and id ordering and the
    fleet keeps working; it just cannot rank by geography.

HOW — dense indices
    Node ids are not contiguous. The real layout numbers 0..880, but a window
    of it (the Webots track) carries real ids 37..252 with gaps. Everything
    internal therefore works on a dense index `0..n-1`, with ids translated at
    the boundary, so a sparse id space costs nothing and cannot silently index
    the wrong row.

HOW — the distance cache
    `_hops_from` memoises a BFS per source. It used to hold `dict[int, dict[int,
    int]]`, which measured ~36 KB per source and never evicted: querying every
    node on the real map would reach ~33 MB, on hardware that does not have it
    to spare. It is now one `array("H")` per source -- 2 bytes per node, ~1.7 KB
    -- behind an LRU bounded by `HOPS_CACHE_SIZE`.
"""
from __future__ import annotations

import json
import logging
import math
from array import array
from collections import OrderedDict, deque
from pathlib import Path

log = logging.getLogger(__name__)

UNREACHABLE = 0xFFFF
"""Sentinel hop count. Also the largest map this cache representation supports."""


class MapError(ValueError):
    """A map document is malformed, or violates an invariant the fleet relies on.

    Raised at load rather than tolerated, because every alternative is worse: a
    dangling edge becomes a route to nowhere, a non-axis-aligned pair becomes a
    turn the robot cannot make, and both surface hours later as a robot sitting
    still for no visible reason.
    """


class NullMap:
    """Stand-in when no map file is available. See module docstring."""

    n = 0
    node_spacing = 0
    ids: tuple[int, ...] = ()
    dimensions: tuple[int, int] = (0, 0)
    units = "cm"
    edge_count = 0

    def region_ids(self) -> tuple[int, ...]:
        return ()

    def region_of(self, node_id: int) -> int | None:
        return None

    def distance(self, a: int, b: int) -> float:
        return 0.0

    def region_distance(self, node_id: int, region_id: int) -> float:
        return 0.0

    def nodes_in(self, region_id: int) -> list[int]:
        return []

    def has(self, node_id: int) -> bool:
        return False

    def neighbours(self, node_id: int) -> tuple[int, ...]:
        return ()

    def position_of(self, node_id: int):
        return None

    def type_of(self, node_id: int) -> str:
        return "PT"

    def density_of(self, node_id: int) -> str:
        return "sparse"


class WarehouseMap:
    def __init__(self, data: dict, hops_cache_size: int = 64) -> None:
        _validate(data)
        nodes = sorted(data["nodes"], key=lambda n: int(n["id"]))
        self.ids: tuple[int, ...] = tuple(int(n["id"]) for n in nodes)
        self.n = len(nodes)
        if self.n > UNREACHABLE:
            raise ValueError(
                f"map has {self.n} nodes; the uint16 distance cache addresses "
                f"at most {UNREACHABLE}"
            )
        self._index: dict[int, int] = {node_id: i for i, node_id in enumerate(self.ids)}

        self.node_spacing = int(data.get("node_spacing", 0))
        self._region_of: tuple[int, ...] = tuple(int(n["region_id"]) for n in nodes)
        self._node_type: tuple[str, ...] = tuple(str(n.get("node_type", "PT")) for n in nodes)
        self._position: tuple[tuple[float, float], ...] = tuple(
            (float(n["position"]["x"]), float(n["position"]["y"])) for n in nodes
        )
        self._density: dict[int, str] = {
            int(r["id"]): str(r.get("density", "sparse")) for r in data.get("regions", ())
        }

        self._by_region: dict[int, list[int]] = {}
        for i, region in enumerate(self._region_of):
            self._by_region.setdefault(region, []).append(self.ids[i])

        adjacency: list[list[int]] = [[] for _ in range(self.n)]
        for edge in data["edges"]:
            a, b = self._index[int(edge["a"])], self._index[int(edge["b"])]
            adjacency[a].append(b)
            adjacency[b].append(a)
        # Sorted so expansion order -- and every tie-break downstream -- is the
        # same on every bot, which is what lets peers predict each other.
        self._adj: tuple[tuple[int, ...], ...] = tuple(tuple(sorted(a)) for a in adjacency)

        self.dimensions: tuple[int, int] = (
            int(data["dimensions"]["width"]),
            int(data["dimensions"]["height"]),
        )
        self.units = str(data["units"])
        self.edge_count = len(data["edges"])

        self._hops_cache: OrderedDict[int, array] = OrderedDict()
        self._hops_cache_size = max(1, hops_cache_size)

    @classmethod
    def load(cls, path: str | Path, hops_cache_size: int = 64) -> WarehouseMap | NullMap:
        p = Path(path)
        if not p.is_file():
            log.warning("warehouse map %s not found; geography-blind dispatch", p)
            return NullMap()
        try:
            with p.open() as f:
                document = json.load(f)
        except json.JSONDecodeError as exc:
            # A truncated or hand-edited map is a hard stop, not a degradation:
            # NullMap is for "no map configured", not "the map is wrong".
            raise MapError(f"{p}: not valid JSON: {exc}") from exc
        m = cls(document, hops_cache_size=hops_cache_size)
        log.info("warehouse map loaded: %d nodes, %d regions", m.n, len(m._by_region))
        return m

    # ---- Identity -----------------------------------------------------------

    def has(self, node_id: int) -> bool:
        return node_id in self._index

    def index(self, node_id: int) -> int:
        """Dense index for an external node id."""
        return self._index[node_id]

    def id_of(self, i: int) -> int:
        return self.ids[i]

    # ---- Attributes ---------------------------------------------------------

    def region_of(self, node_id: int) -> int | None:
        i = self._index.get(node_id)
        return None if i is None else self._region_of[i]

    def type_of(self, node_id: int) -> str:
        i = self._index.get(node_id)
        return "PT" if i is None else self._node_type[i]

    def position_of(self, node_id: int) -> tuple[float, float] | None:
        i = self._index.get(node_id)
        return None if i is None else self._position[i]

    def density_of(self, node_id: int) -> str:
        region = self.region_of(node_id)
        return self._density.get(region, "sparse") if region is not None else "sparse"

    def region_ids(self) -> tuple[int, ...]:
        return tuple(sorted(self._by_region))

    def nodes_in(self, region_id: int) -> list[int]:
        return list(self._by_region.get(region_id, []))

    def neighbours(self, node_id: int) -> tuple[int, ...]:
        i = self._index.get(node_id)
        return () if i is None else tuple(self.ids[j] for j in self._adj[i])

    def adjacency(self) -> tuple[tuple[int, ...], ...]:
        """Dense adjacency, for callers that already work in dense indices."""
        return self._adj

    # ---- Distances ----------------------------------------------------------

    def hops_from(self, node_id: int) -> array:
        """Hop distance from `node_id` to every node, indexed densely.

        Cached, bounded, and `UNREACHABLE` where there is no path.
        """
        i = self._index.get(node_id)
        if i is None:
            raise KeyError(node_id)
        return self._hops_from(i)

    def _hops_from(self, src: int) -> array:
        cached = self._hops_cache.get(src)
        if cached is not None:
            self._hops_cache.move_to_end(src)
            return cached

        dist = array("H", bytes(2 * self.n))
        for i in range(self.n):
            dist[i] = UNREACHABLE
        dist[src] = 0
        queue = deque((src,))
        adj = self._adj
        while queue:
            u = queue.popleft()
            d = dist[u] + 1
            for v in adj[u]:
                if dist[v] == UNREACHABLE:
                    dist[v] = d
                    queue.append(v)

        self._hops_cache[src] = dist
        if len(self._hops_cache) > self._hops_cache_size:
            self._hops_cache.popitem(last=False)
        return dist

    def distance(self, a: int, b: int) -> float:
        """Driving distance in QR hops (edges are all `node_spacing` long)."""
        ia, ib = self._index.get(a), self._index.get(b)
        if ia is None or ib is None:
            return math.inf
        d = self._hops_from(ia)[ib]
        return math.inf if d == UNREACHABLE else float(d)

    def region_distance(self, node_id: int, region_id: int) -> float:
        """Hops from `node_id` to the nearest node of `region_id`."""
        i = self._index.get(node_id)
        if i is None:
            return math.inf
        hops = self._hops_from(i)
        best = UNREACHABLE
        for other in self._by_region.get(region_id, ()):
            d = hops[self._index[other]]
            if d < best:
                best = d
        return math.inf if best == UNREACHABLE else float(best)


# ---- Validation --------------------------------------------------------------

_NODE_TYPES = {"PT", "TR", "CH", "PK", "YI"}
_DENSITIES = {"dense", "medium", "sparse"}


def _validate(doc: dict) -> None:
    """Check the document against the schema's promises before trusting it.

    The JSON Schema at `spore-amr/shared/schemas/warehouse-map.schema.json` is
    the ground truth; this re-checks the parts the fleet actually depends on,
    plus the two geometric invariants the schema only states in prose: every
    edge is axis-aligned and exactly `node_spacing` long. Those two are what
    make 90-degree turns the only turns, so a map that breaks them breaks the
    robot rather than the loader.
    """
    if not isinstance(doc, dict):
        raise MapError(f"map must be an object, got {type(doc).__name__}")
    for key in ("schema_version", "units", "node_spacing", "dimensions", "regions", "nodes", "edges"):
        if key not in doc:
            raise MapError(f"map is missing required field {key!r}")
    if doc["units"] != "cm":
        raise MapError(f"units must be 'cm', got {doc['units']!r}")

    spacing = _positive_int(doc["node_spacing"], "node_spacing")
    dims = doc["dimensions"]
    if not isinstance(dims, dict):
        raise MapError("dimensions must be an object")
    width = _positive_int(dims.get("width"), "dimensions.width")
    height = _positive_int(dims.get("height"), "dimensions.height")

    regions = doc["regions"]
    if not isinstance(regions, list) or not regions:
        raise MapError("regions must be a non-empty array")
    region_ids: set[int] = set()
    for i, region in enumerate(regions):
        rid = _non_negative(region.get("id"), f"regions[{i}].id")
        if rid in region_ids:
            raise MapError(f"duplicate region id {rid}")
        region_ids.add(rid)
        if region.get("density") not in _DENSITIES:
            raise MapError(f"regions[{i}].density is invalid: {region.get('density')!r}")
        if not isinstance(region.get("name"), str) or not region["name"]:
            raise MapError(f"regions[{i}].name must be a non-empty string")

    nodes = doc["nodes"]
    if not isinstance(nodes, list) or not nodes:
        raise MapError("nodes must be a non-empty array")
    positions: dict[int, tuple[float, float]] = {}
    for i, node in enumerate(nodes):
        nid = _non_negative(node.get("id"), f"nodes[{i}].id")
        if nid in positions:
            raise MapError(f"duplicate node id {nid}")
        if node.get("region_id") not in region_ids:
            raise MapError(f"node {nid} references unknown region_id {node.get('region_id')}")
        if node.get("node_type") not in _NODE_TYPES:
            raise MapError(f"node {nid} has invalid node_type {node.get('node_type')!r}")
        if not isinstance(node.get("name"), str) or not node["name"]:
            raise MapError(f"node {nid} name must be a non-empty string")
        pos = node.get("position")
        if not isinstance(pos, dict) or not isinstance(pos.get("x"), (int, float)) or not isinstance(pos.get("y"), (int, float)):
            raise MapError(f"node {nid} position must be an object with numeric x and y")
        x, y = float(pos["x"]), float(pos["y"])
        if not (0 <= x <= width) or not (0 <= y <= height):
            raise MapError(
                f"node {nid} position ({x}, {y}) lies outside dimensions {width}x{height}"
            )
        positions[nid] = (x, y)

    edges = doc["edges"]
    if not isinstance(edges, list) or not edges:
        raise MapError("edges must be a non-empty array")
    seen: set[tuple[int, int]] = set()
    for i, edge in enumerate(edges):
        a = _non_negative(edge.get("a"), f"edges[{i}].a")
        b = _non_negative(edge.get("b"), f"edges[{i}].b")
        for end in (a, b):
            if end not in positions:
                raise MapError(f"edges[{i}] references unknown node id {end}")
        if a == b:
            raise MapError(f"edges[{i}] is a self-loop on node {a}")
        key = (a, b) if a < b else (b, a)
        if key in seen:
            raise MapError(f"duplicate edge between nodes {key[0]} and {key[1]}")
        seen.add(key)

        length = _positive_int(edge.get("length"), f"edges[{i}].length")
        if length != spacing:
            raise MapError(
                f"edge {a}-{b} has length {length}, but every edge must equal "
                f"node_spacing ({spacing})"
            )
        (ax, ay), (bx, by) = positions[a], positions[b]
        dx, dy = abs(bx - ax), abs(by - ay)
        if (dx and dy) or (dx + dy) != float(spacing):
            raise MapError(
                f"edge {a}-{b} is not an axis-aligned step of {spacing} cm: dx={dx}, dy={dy}"
            )


def _positive_int(value, field: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool):
        raise MapError(f"{field} must be an integer, got {value!r}")
    if value <= 0:
        raise MapError(f"{field} must be positive, got {value}")
    return value


def _non_negative(value, field: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool):
        raise MapError(f"{field} must be an integer, got {value!r}")
    if value < 0:
        raise MapError(f"{field} must be non-negative, got {value}")
    return value
