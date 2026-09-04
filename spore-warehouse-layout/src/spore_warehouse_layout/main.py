#!/usr/bin/env python3
"""Generate a static AMR warehouse map (nodes + edges) as JSON.

120 m x 70 m fulfilment-centre layout, one QR node every 200 cm on every path.
Zones run in material-flow order, inbound west to outbound east, around a central
lattice storage field. All movement is axis-aligned (90 degree turns only).

Yield nodes are pull-over bays: one-node spurs perpendicular to a busy lane, so a
robot can step off the path and let another pass. They sit only where traffic
actually converges — receiving/inspection and pick/pack/sort.

Outputs warehouse.json and warehouse_map.svg. Standard library only.
"""

from __future__ import annotations

import json
from collections.abc import Iterator
from itertools import pairwise
from pathlib import Path
from typing import Literal, TypedDict

Pos = tuple[int, int]
NodeType = Literal["PT", "TR", "CH", "PK", "YI"]


class Position(TypedDict):
    x: float
    y: float


class Dimensions(TypedDict):
    width: int
    height: int


class Region(TypedDict):
    id: int
    name: str
    density: str
    description: str


class NodeMeta(TypedDict):
    region_id: int
    node_type: NodeType


class Node(TypedDict):
    id: int
    name: str
    region_id: int
    node_type: NodeType
    position: Position


class Edge(TypedDict):
    a: int
    b: int
    length: int


class Doc(TypedDict):
    schema_version: str
    units: str
    node_spacing: int
    dimensions: Dimensions
    regions: list[Region]
    nodes: list[Node]
    edges: list[Edge]


SPACING: int = 200
WIDTH: int = 12000
HEIGHT: int = 7000

# Storage field: travel lanes form a lattice, storage blocks between them are solid.
FX0, FX1 = 3400, 9000
FY0, FY1 = 1400, 5600
VP, HP = 800, 1400  # lane pitch -> 3 x 6 cell storage blocks

# Ring highway
HW_W, HW_E = 800, 11200
HW_S, HW_N = 600, 6400

X_IN_DOCK = 400
X_STOW = 3200
X_PICK = 9200
X_PACK = 9800
X_SORT = 10600
X_OUT_DOCK = 11600

Y_CHARGE, Y_CHARGE_S, Y_CHARGE_N = 1000, 800, 1200
Y_CROSSDOCK = 6000
Y_PARK_N, Y_PARK_S = 6200, 5800

BULK_X: tuple[int, int, int] = (800, 2000, 3200)  # bulk storage: wider lane pitch
BULK_Y: tuple[int, int] = (1400, 2400)
BUF_Y: tuple[int, int] = (2600, 3200)  # put-away buffer, directly under receiving

(
    R_RECEIVING,
    R_PARKING,
    R_CHARGING,
    R_PICK_PACK_SORT,
    R_STORAGE_A,
    R_STORAGE_B,
    R_STORAGE_C,
) = range(1, 8)

REGIONS: list[Region] = [
    {
        "id": R_RECEIVING,
        "name": "receiving_and_buffer",
        "density": "sparse",
        "description": "Inbound doors, receiving & inspection, put-away buffer and stow "
        "stations feeding the field.",
    },
    {
        "id": R_PARKING,
        "name": "parking",
        "density": "sparse",
        "description": "Idle-robot bays along the north strip, the cross-dock express "
        "lane, and the north half of the ring highway.",
    },
    {
        "id": R_CHARGING,
        "name": "charging",
        "density": "sparse",
        "description": "Charger bays along the south edge and the south half of the "
        "ring highway.",
    },
    {
        "id": R_PICK_PACK_SORT,
        "name": "pick_pack_sort",
        "density": "sparse",
        "description": "Pick, pack/VAS and sort/staging columns, plus the outbound "
        "shipping doors.",
    },
    {
        "id": R_STORAGE_A,
        "name": "bulk_and_storage_cols_1_2",
        "density": "dense",
        "description": "Bulk/oversize storage plus the first two columns of the grid "
        "field (nearest stow).",
    },
    {
        "id": R_STORAGE_B,
        "name": "storage_cols_3_4_5",
        "density": "dense",
        "description": "The middle three columns of the grid field.",
    },
    {
        "id": R_STORAGE_C,
        "name": "storage_cols_6_7",
        "density": "dense",
        "description": "The last two columns of the grid field (nearest pick).",
    },
]

# Storage-block column -> region, west to east. Column c sits between
# VLANES[c] and VLANES[c + 1].
BLOCK_REGION: tuple[int, ...] = (
    R_STORAGE_A, R_STORAGE_A,
    R_STORAGE_B, R_STORAGE_B, R_STORAGE_B,
    R_STORAGE_C, R_STORAGE_C,
)
# Vertical lattice lane -> region. A lane shared by two column groups belongs
# to the group on its west side.
LANE_REGION: tuple[int, ...] = (
    R_STORAGE_A, R_STORAGE_A, R_STORAGE_A,
    R_STORAGE_B, R_STORAGE_B, R_STORAGE_B,
    R_STORAGE_C, R_STORAGE_C,
)

# When paths overlap, the most specific node type and the lowest region id win.
TYPE_PRIORITY: dict[NodeType, int] = {"PT": 0, "YI": 1, "PK": 2, "CH": 3, "TR": 4}

VLANES: list[int] = list(range(FX0, FX1 + 1, VP))
HLANES: list[int] = list(range(FY0, FY1 + 1, HP))

SPINE_TIES: tuple[int, int, int] = (3800, 6200, 8600)  # offset from bay lanes
IN_DOORS: tuple[int, int, int] = (4200, 4800, 5400)
OUT_DOORS: tuple[int, int, int, int, int] = (1400, 2400, 3400, 4400, 5400)
RECEIVE_H: tuple[int, int, int] = (3400, 4200, 5600)
RECEIVE_V: tuple[int, int, int] = (1200, 2000, 2800)

CHARGERS: list[int] = [x for x in range(1200, 10801, 600) if x not in SPINE_TIES]
PARK_BAYS: list[int] = [x for x in range(1200, 10801, 400) if x not in SPINE_TIES]

# Pull-over bays: (anchor node on the lane, bay node one cell to the side).
YIELD_BAYS: tuple[tuple[Pos, Pos], ...] = (
    # receiving & inspection — spurs off the horizontal lanes and the middle column
    ((1600, 3400), (1600, 3600)),
    ((2400, 3400), (2400, 3600)),
    ((1600, 4200), (1600, 4400)),
    ((2400, 4200), (2400, 4400)),
    ((2000, 4800), (2200, 4800)),
    ((2000, 5200), (2200, 5200)),
    # pick / pack / sort — spurs off each column, where outbound traffic converges
    ((X_PICK, 2000), (9400, 2000)),
    ((X_PICK, 3400), (9400, 3400)),
    ((X_PICK, 4800), (9400, 4800)),
    ((X_PACK, 2000), (10000, 2000)),
    ((X_PACK, 3400), (10000, 3400)),
    ((X_PACK, 4800), (10000, 4800)),
    ((X_SORT, 2000), (10800, 2000)),
    ((X_SORT, 3400), (10800, 3400)),
    ((X_SORT, 4800), (10800, 4800)),
)


def hline(y: int, x0: int, x1: int) -> list[Pos]:
    return [(x, y) for x in range(x0, x1 + SPACING, SPACING)]


def vline(x: int, y0: int, y1: int) -> list[Pos]:
    return [(x, y) for y in range(y0, y1 + SPACING, SPACING)]


class MapBuilder:
    def __init__(self) -> None:
        self.nodes: dict[Pos, NodeMeta] = {}
        self.edges: set[tuple[Pos, Pos]] = set()

    def path(
        self, points: list[Pos], region_id: int, node_type: NodeType = "PT"
    ) -> None:
        for p in points:
            self._add(p, region_id, node_type)
        for a, b in pairwise(points):
            lo, hi = sorted((a, b))
            self.edges.add((lo, hi))

    def _add(
        self, p: Pos, region_id: int, node_type: NodeType, force_region: bool = False
    ) -> None:
        cur = self.nodes.get(p)
        if cur is None:
            self.nodes[p] = {"region_id": region_id, "node_type": node_type}
            return
        cur["region_id"] = region_id if force_region else min(cur["region_id"], region_id)
        if TYPE_PRIORITY[node_type] > TYPE_PRIORITY[cur["node_type"]]:
            cur["node_type"] = node_type

    def mark(
        self, points: list[Pos], node_type: NodeType, region_id: int | None = None
    ) -> None:
        """Retag existing points. region_id defaults to the point's current region
        (inherit); pass one explicitly to force it, e.g. when a node's lattice lane
        and its functional group disagree."""
        for p in points:
            if region_id is not None:
                self._add(p, region_id, node_type, force_region=True)
            else:
                self._add(p, self.nodes[p]["region_id"], node_type)

    def bay(self, anchor: Pos, tip: Pos, region_id: int, node_type: NodeType) -> None:
        """One-cell spur off a lane: somewhere to stand that is not on the path."""
        self.path([anchor, tip], region_id)
        self.mark([tip], node_type)


def build() -> MapBuilder:
    b = MapBuilder()

    # Ring highway: south half (incl. lower halves of the vertical edges) joins
    # charging; north half joins parking. Split at the vertical edges' midpoint,
    # rounded down to the 200cm grid — (HW_S + HW_N) // 2 lands on an odd multiple
    # of 100 and would otherwise create off-grid phantom nodes.
    HW_MID = (HW_S + HW_N) // 2 // SPACING * SPACING
    b.path(hline(HW_S, HW_W, HW_E), R_CHARGING)
    b.path(hline(HW_N, HW_W, HW_E), R_PARKING)
    b.path(vline(HW_W, HW_S, HW_MID), R_CHARGING)
    b.path(vline(HW_W, HW_MID, HW_N), R_PARKING)
    b.path(vline(HW_E, HW_S, HW_MID), R_CHARGING)
    b.path(vline(HW_E, HW_MID, HW_N), R_PARKING)

    # Storage field: lane lattice, solid blocks between the lanes. Each lane and
    # each row segment is tagged by which storage-column group it belongs to.
    for i, x in enumerate(VLANES):
        b.path(vline(x, FY0, FY1), LANE_REGION[i])
    field_splits = (FX0, VLANES[2], VLANES[5], FX1)
    for y in HLANES:
        for x0, x1, region in zip(
            field_splits, field_splits[1:], (R_STORAGE_A, R_STORAGE_B, R_STORAGE_C)
        ):
            b.path(hline(y, x0, x1), region)
    for c, x in enumerate(VLANES[:-1]):
        for y in HLANES[:-1]:
            b.mark([(x, y + 600)], "TR", region_id=BLOCK_REGION[c])

    # Inbound: dock doors -> receiving & inspection floor.
    for y in IN_DOORS:
        b.path(hline(y, X_IN_DOCK, HW_W), R_RECEIVING)
        b.mark([(X_IN_DOCK, y)], "TR")
    for y in RECEIVE_H:
        b.path(hline(y, HW_W, X_STOW), R_RECEIVING)
    for x in RECEIVE_V:
        b.path(vline(x, BUF_Y[0], RECEIVE_H[-1]), R_RECEIVING)
        b.mark([(x, 3400), (x, 4200)], "TR")

    # Put-away buffer: inspected goods wait here for a free stow station.
    for y in BUF_Y:
        b.path(hline(y, HW_W, X_STOW), R_RECEIVING)
    for x in RECEIVE_V:
        b.mark([(x, 2800), (x, 3000)], "TR")

    # Bulk / oversize storage: wider lanes, larger blocks than the grid field.
    for x in BULK_X:
        b.path(vline(x, BULK_Y[0], BULK_Y[1]), R_STORAGE_A)
    for y in BULK_Y:
        b.path(hline(y, BULK_X[0], BULK_X[-1]), R_STORAGE_A)
    b.path(vline(BULK_X[1], BULK_Y[1], BUF_Y[0]), R_STORAGE_A)
    for x in BULK_X[:-1]:
        b.mark([(x + 400, BULK_Y[0]), (x + 400, BULK_Y[1])], "TR")

    # Stow column feeding the west face of the field.
    b.path(vline(X_STOW, FY0, FY1), R_RECEIVING)
    for y in HLANES:
        b.path(hline(y, X_STOW, FX0), R_RECEIVING)
        b.mark([(X_STOW, y)], "TR")

    # Outbound: field east face -> pick -> pack/VAS -> sort/stage -> shipping doors.
    for x in (X_PICK, X_PACK, X_SORT):
        b.path(vline(x, FY0, FY1), R_PICK_PACK_SORT)
        b.mark([(x, y) for y in HLANES], "TR")
    for y in HLANES:
        b.path(hline(y, FX1, X_PICK), R_PICK_PACK_SORT)
        b.path(hline(y, X_PICK, HW_E), R_PICK_PACK_SORT)
    for y in OUT_DOORS:
        b.path(hline(y, HW_E, X_OUT_DOCK), R_PICK_PACK_SORT)
        b.mark([(X_OUT_DOCK, y)], "TR")

    # Charging bank: spine across the full south edge, chargers down both sides.
    b.path(hline(Y_CHARGE, HW_W, HW_E), R_CHARGING)
    for x in CHARGERS:
        b.bay((x, Y_CHARGE), (x, Y_CHARGE_S), R_CHARGING, "CH")
        b.bay((x, Y_CHARGE), (x, Y_CHARGE_N), R_CHARGING, "CH")

    # North strip: cross-dock express lane with parking bays down both sides.
    b.path(hline(Y_CROSSDOCK, HW_W, HW_E), R_PARKING)
    for x in PARK_BAYS:
        b.bay((x, Y_CROSSDOCK), (x, Y_PARK_N), R_PARKING, "PK")
        b.bay((x, Y_CROSSDOCK), (x, Y_PARK_S), R_PARKING, "PK")

    # Tie the south and north bands into the ring and the field edges.
    for x in SPINE_TIES:
        b.path(vline(x, HW_S, FY0), R_CHARGING)
        b.path(vline(x, FY1, HW_N), R_PARKING)

    # Pull-over bays where traffic converges.
    for anchor, tip in YIELD_BAYS:
        b.bay(anchor, tip, b.nodes[anchor]["region_id"], "YI")

    return b


def serialize(b: MapBuilder) -> tuple[Doc, dict[Pos, int]]:
    ordered = sorted(b.nodes.items(), key=lambda kv: (kv[0][1], kv[0][0]))
    ids: dict[Pos, int] = {pos: i for i, (pos, _) in enumerate(ordered)}
    region_name: dict[int, str] = {r["id"]: r["name"] for r in REGIONS}
    seq: dict[tuple[int, NodeType], int] = {}
    nodes: list[Node] = []
    for pos, meta in ordered:
        key = (meta["region_id"], meta["node_type"])
        seq[key] = seq.get(key, 0) + 1
        nodes.append(
            {
                "id": ids[pos],
                "name": f"{region_name[meta['region_id']]}/{meta['node_type']}/{seq[key]:03d}",
                "region_id": meta["region_id"],
                "node_type": meta["node_type"],
                "position": {"x": float(pos[0]), "y": float(pos[1])},
            }
        )
    edges: list[Edge] = sorted(
        ({"a": ids[a], "b": ids[b_], "length": SPACING} for a, b_ in b.edges),
        key=lambda e: (e["a"], e["b"]),
    )
    return {
        "schema_version": "v0.1.0",
        "units": "cm",
        "node_spacing": SPACING,
        "dimensions": {"width": WIDTH, "height": HEIGHT},
        "regions": REGIONS,
        "nodes": nodes,
        "edges": edges,
    }, ids


TYPE_COLORS: dict[NodeType, str] = {
    "PT": "#9aa4ad",
    "TR": "#1857b8",
    "CH": "#0f9d58",
    "PK": "#9a6b00",
    "YI": "#d23f31",
}
REGION_BOXES: list[tuple[str, int, int, int, int, str]] = [
    ("#faf3e6", HW_W, RECEIVE_H[0], X_STOW, RECEIVE_H[-1], "RECEIVING / INSPECTION / BUFFER"),
    ("#faf3e6", HW_W, BUF_Y[0], X_STOW, BUF_Y[1], ""),
    ("#efe8f7", BULK_X[0], BULK_Y[0], BULK_X[-1], BULK_Y[1], "BULK + STORAGE COLS 1-2"),
    ("#e8eef8", VLANES[0], FY0, VLANES[2], FY1, ""),
    ("#dbe6f6", VLANES[2], FY0, VLANES[5], FY1, "STORAGE COLS 3-4-5"),
    ("#e8eef8", VLANES[5], FY0, VLANES[7], FY1, "STORAGE COLS 6-7"),
    ("#eef4ea", X_PICK, FY0, X_SORT, FY1, "PICK / PACK / SORT"),
    ("#e9f6ee", HW_W, HW_S, HW_E, FY0, "CHARGING"),
    ("#eceff4", HW_W, FY1, HW_E, HW_N, "PARKING"),
]


def storage_blocks() -> Iterator[tuple[int, int, int, int]]:
    """Solid block between four lanes, inset half a cell so it never covers a lane."""
    half = SPACING // 2
    for x in VLANES[:-1]:
        for y in HLANES[:-1]:
            yield x + half, y + half, VP - SPACING, HP - SPACING


def bulk_blocks() -> Iterator[tuple[int, int, int, int]]:
    half = SPACING // 2
    for x0, x1 in pairwise(BULK_X):
        yield (
            x0 + half,
            BULK_Y[0] + half,
            x1 - x0 - SPACING,
            BULK_Y[1] - BULK_Y[0] - SPACING,
        )


def to_svg(doc: Doc, scale: float = 0.11, pad: int = 90) -> str:
    w, h = WIDTH * scale + pad * 2, HEIGHT * scale + pad * 2

    def px(x: float) -> float:
        return pad + x * scale

    def py(y: float) -> float:
        return pad + (HEIGHT - y) * scale

    pos: dict[int, tuple[float, float]] = {
        n["id"]: (n["position"]["x"], n["position"]["y"]) for n in doc["nodes"]
    }

    out: list[str] = [
        (
            f'<svg xmlns="http://www.w3.org/2000/svg" width="{w:.0f}" height="{h:.0f}" '
            f'viewBox="0 0 {w:.0f} {h:.0f}" font-family="monospace">'
        ),
        f'<rect width="{w:.0f}" height="{h:.0f}" fill="#ffffff"/>',
    ]

    for tint, x0, y0, x1, y1, _ in REGION_BOXES:
        out.append(
            f'<rect x="{px(x0):.1f}" y="{py(y1):.1f}" width="{(x1 - x0) * scale:.1f}" '
            f'height="{(y1 - y0) * scale:.1f}" fill="{tint}" stroke="#c9d0d6" '
            f'stroke-dasharray="4 3"/>'
        )

    for bx, by, bw, bh in list(storage_blocks()) + list(bulk_blocks()):
        out.append(
            f'<rect x="{px(bx):.1f}" y="{py(by + bh):.1f}" width="{bw * scale:.1f}" '
            f'height="{bh * scale:.1f}" fill="#c3ccd8" stroke="#9aa6b4" stroke-width="0.5"/>'
        )

    out.append(
        f'<rect x="{px(0):.1f}" y="{py(HEIGHT):.1f}" width="{WIDTH * scale:.1f}" '
        f'height="{HEIGHT * scale:.1f}" fill="none" stroke="#191e21" stroke-width="2"/>'
    )

    for e in doc["edges"]:
        (x1, y1), (x2, y2) = pos[e["a"]], pos[e["b"]]
        out.append(
            f'<line x1="{px(x1):.1f}" y1="{py(y1):.1f}" x2="{px(x2):.1f}" '
            f'y2="{py(y2):.1f}" stroke="#b6bec6" stroke-width="1.1"/>'
        )

    for n in doc["nodes"]:
        t = n["node_type"]
        x, y = n["position"]["x"], n["position"]["y"]
        r = 1.5 if t == "PT" else 3.0
        out.append(
            f'<circle cx="{px(x):.1f}" cy="{py(y):.1f}" r="{r}" fill="{TYPE_COLORS[t]}"/>'
        )

    for _, x0, y0, x1, y1, label in REGION_BOXES:
        tx, ty = px(x0) + 8, py(y1) + 17
        out.append(
            f'<rect x="{tx - 4:.1f}" y="{ty - 12:.1f}" width="{len(label) * 6.9:.1f}" '
            f'height="16" fill="#ffffff" fill-opacity="0.9"/>'
        )
        esc = label.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")
        out.append(
            f'<text x="{tx:.1f}" y="{ty:.1f}" font-size="11.5" fill="#2b3439">{esc}</text>'
        )

    legend: list[tuple[NodeType, str]] = [
        ("PT", "pass-through"),
        ("TR", "transfer"),
        ("CH", "charging"),
        ("PK", "parking bay"),
        ("YI", "yield / pull-over bay"),
    ]
    for i, (t, label) in enumerate(legend):
        lx, ly = pad + i * 175, pad + HEIGHT * scale + 40
        out.append(f'<circle cx="{lx}" cy="{ly - 4}" r="4" fill="{TYPE_COLORS[t]}"/>')
        out.append(
            f'<text x="{lx + 12}" y="{ly}" font-size="12" fill="#191e21">{t} {label}</text>'
        )
    out.append(
        f'<text x="{pad}" y="{pad - 30}" font-size="15" fill="#191e21">'
        f"AMR warehouse map — {WIDTH / 100:.0f} m x {HEIGHT / 100:.0f} m, "
        f"{len(doc['nodes'])} nodes, {len(doc['edges'])} edges, "
        f"{SPACING} cm spacing — flow runs west to east</text>"
    )
    out.append("</svg>")
    return "\n".join(out)


def to_ascii(doc: Doc, step: int = 1) -> str:
    glyph: dict[NodeType, str] = {"PT": "·", "TR": "T", "CH": "C", "PK": "P", "YI": "+"}
    cell = SPACING * step
    cols, rows = WIDTH // cell + 1, HEIGHT // cell + 1
    best: list[list[NodeType | None]] = [[None] * cols for _ in range(rows)]
    for n in doc["nodes"]:
        c, r = int(n["position"]["x"]) // cell, int(n["position"]["y"]) // cell
        t = n["node_type"]
        cur = best[r][c]
        if cur is None or TYPE_PRIORITY[t] > TYPE_PRIORITY[cur]:
            best[r][c] = t
    lines = ["+" + "-" * cols + "+"]
    for r in range(rows - 1, -1, -1):
        lines.append(
            "|" + "".join(" " if t is None else glyph[t] for t in best[r]) + "|"
        )
    lines.append("+" + "-" * cols + "+")
    return "\n".join(lines)


def run() -> None:
    out = Path.cwd() / "output"
    out.mkdir(parents=True, exist_ok=True)
    b = build()
    doc, _ = serialize(b)

    (out / "warehouse.json").write_text(json.dumps(doc, indent=2) + "\n")
    (out / "warehouse_map.svg").write_text(to_svg(doc) + "\n")

    counts: dict[NodeType, int] = {}
    for n in doc["nodes"]:
        counts[n["node_type"]] = counts.get(n["node_type"], 0) + 1
    print(to_ascii(doc))
    print(f"\n{WIDTH / 100:.0f} m x {HEIGHT / 100:.0f} m, {SPACING} cm spacing")
    print(
        f"{len(doc['nodes'])} nodes, {len(doc['edges'])} edges, "
        f"{len(VLANES)}x{len(HLANES)} field lanes, "
        f"{sum(1 for _ in storage_blocks())} storage blocks"
    )
    print("by type: " + ", ".join(f"{k}={v}" for k, v in sorted(counts.items())))
    for r in REGIONS:
        n = sum(1 for x in doc["nodes"] if x["region_id"] == r["id"])
        print(f"  {r['id']:>2} {r['name']:<22} {n:>4} nodes  ({r['density']})")


if __name__ == "__main__":
    run()
