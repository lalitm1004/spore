"""Load and validate a warehouse map document.

The JSON Schema at `spore-amr/shared/schemas/warehouse-map.schema.json` is the
ground truth for the wire format. This module re-checks the parts of it that the
planner actually depends on, plus the geometric invariants the schema only states
in prose: every edge is axis-aligned and exactly `node_spacing` long.

Validating here rather than trusting the producer keeps the failure at load time,
where the message can name the offending node, instead of surfacing as a wrong
path hours later.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from spore_planner.warehouse.map import (
    Density,
    Dimensions,
    Edge,
    Node,
    NodeType,
    Position,
    Region,
    WarehouseMap,
)


class MapError(ValueError):
    """A warehouse map document is malformed or violates a planner invariant."""


def load_map_file(path: str | Path) -> WarehouseMap:
    """Read and validate a warehouse map from a JSON file."""
    text = Path(path).read_text()
    try:
        doc = json.loads(text)
    except json.JSONDecodeError as exc:
        raise MapError(f"{path}: not valid JSON: {exc}") from exc
    return parse_map(doc)


def load_map(text: str) -> WarehouseMap:
    """Validate a warehouse map from a JSON string."""
    try:
        doc = json.loads(text)
    except json.JSONDecodeError as exc:
        raise MapError(f"not valid JSON: {exc}") from exc
    return parse_map(doc)


def parse_map(doc: Any) -> WarehouseMap:
    """Validate an already-decoded warehouse map document."""
    if not isinstance(doc, dict):
        raise MapError(f"map must be an object, got {type(doc).__name__}")

    for key in ("schema_version", "units", "node_spacing", "dimensions", "regions", "nodes", "edges"):
        if key not in doc:
            raise MapError(f"map is missing required field {key!r}")

    units = doc["units"]
    if units != "cm":
        raise MapError(f"units must be 'cm', got {units!r}")

    spacing = _positive_int(doc["node_spacing"], "node_spacing")
    dimensions = _parse_dimensions(doc["dimensions"])
    regions = _parse_regions(doc["regions"])
    region_ids = {r.id for r in regions}
    nodes = _parse_nodes(doc["nodes"], region_ids, dimensions)
    edges = _parse_edges(doc["edges"], nodes, spacing)

    return WarehouseMap(
        schema_version=str(doc["schema_version"]),
        units=units,
        node_spacing=spacing,
        dimensions=dimensions,
        regions=regions,
        nodes=nodes,
        edges=edges,
    )


def _positive_int(value: Any, field: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool):
        raise MapError(f"{field} must be an integer, got {value!r}")
    if value <= 0:
        raise MapError(f"{field} must be positive, got {value}")
    return value


def _non_negative_id(value: Any, field: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool):
        raise MapError(f"{field} must be an integer, got {value!r}")
    if value < 0:
        raise MapError(f"{field} must be non-negative, got {value}")
    return value


def _parse_dimensions(raw: Any) -> Dimensions:
    if not isinstance(raw, dict):
        raise MapError("dimensions must be an object")
    return Dimensions(
        width=_positive_int(raw.get("width"), "dimensions.width"),
        height=_positive_int(raw.get("height"), "dimensions.height"),
    )


def _parse_regions(raw: Any) -> tuple[Region, ...]:
    if not isinstance(raw, list) or not raw:
        raise MapError("regions must be a non-empty array")
    regions: list[Region] = []
    seen: set[int] = set()
    for i, item in enumerate(raw):
        if not isinstance(item, dict):
            raise MapError(f"regions[{i}] must be an object")
        rid = _non_negative_id(item.get("id"), f"regions[{i}].id")
        if rid in seen:
            raise MapError(f"duplicate region id {rid}")
        seen.add(rid)
        try:
            density = Density(item.get("density"))
        except ValueError as exc:
            raise MapError(f"regions[{i}].density is invalid: {item.get('density')!r}") from exc
        name = item.get("name")
        if not isinstance(name, str) or not name:
            raise MapError(f"regions[{i}].name must be a non-empty string")
        regions.append(
            Region(id=rid, name=name, density=density, description=str(item.get("description", "")))
        )
    return tuple(regions)


def _parse_nodes(raw: Any, region_ids: set[int], dims: Dimensions) -> tuple[Node, ...]:
    if not isinstance(raw, list) or not raw:
        raise MapError("nodes must be a non-empty array")
    nodes: list[Node] = []
    seen: set[int] = set()
    for i, item in enumerate(raw):
        if not isinstance(item, dict):
            raise MapError(f"nodes[{i}] must be an object")
        nid = _non_negative_id(item.get("id"), f"nodes[{i}].id")
        if nid in seen:
            raise MapError(f"duplicate node id {nid}")
        seen.add(nid)

        region_id = _non_negative_id(item.get("region_id"), f"nodes[{i}].region_id")
        if region_id not in region_ids:
            raise MapError(f"node {nid} references unknown region_id {region_id}")

        try:
            node_type = NodeType(item.get("node_type"))
        except ValueError as exc:
            raise MapError(f"node {nid} has invalid node_type {item.get('node_type')!r}") from exc

        name = item.get("name")
        if not isinstance(name, str) or not name:
            raise MapError(f"node {nid} name must be a non-empty string")

        pos = item.get("position")
        if not isinstance(pos, dict) or not isinstance(pos.get("x"), (int, float)) or not isinstance(pos.get("y"), (int, float)):
            raise MapError(f"node {nid} position must be an object with numeric x and y")
        x, y = float(pos["x"]), float(pos["y"])
        if not (0 <= x <= dims.width) or not (0 <= y <= dims.height):
            raise MapError(
                f"node {nid} position ({x}, {y}) lies outside dimensions "
                f"{dims.width}x{dims.height}"
            )

        nodes.append(
            Node(id=nid, name=name, region_id=region_id, node_type=node_type, position=Position(x, y))
        )
    return tuple(nodes)


def _parse_edges(raw: Any, nodes: tuple[Node, ...], spacing: int) -> tuple[Edge, ...]:
    if not isinstance(raw, list) or not raw:
        raise MapError("edges must be a non-empty array")
    by_id = {n.id: n for n in nodes}
    edges: list[Edge] = []
    seen: set[tuple[int, int]] = set()
    for i, item in enumerate(raw):
        if not isinstance(item, dict):
            raise MapError(f"edges[{i}] must be an object")
        a = _non_negative_id(item.get("a"), f"edges[{i}].a")
        b = _non_negative_id(item.get("b"), f"edges[{i}].b")
        if a not in by_id:
            raise MapError(f"edges[{i}] references unknown node id {a}")
        if b not in by_id:
            raise MapError(f"edges[{i}] references unknown node id {b}")
        if a == b:
            raise MapError(f"edges[{i}] is a self-loop on node {a}")

        key = (a, b) if a < b else (b, a)
        if key in seen:
            raise MapError(f"duplicate edge between nodes {key[0]} and {key[1]}")
        seen.add(key)

        length = _positive_int(item.get("length"), f"edges[{i}].length")
        if length != spacing:
            raise MapError(
                f"edge {a}-{b} has length {length}, but every edge must equal "
                f"node_spacing ({spacing})"
            )

        pa, pb = by_id[a].position, by_id[b].position
        dx, dy = abs(pb.x - pa.x), abs(pb.y - pa.y)
        if (dx and dy) or (dx + dy) != float(spacing):
            raise MapError(
                f"edge {a}-{b} is not an axis-aligned step of {spacing} cm: "
                f"dx={dx}, dy={dy}"
            )

        edges.append(Edge(a=a, b=b, length=length))
    return tuple(edges)
