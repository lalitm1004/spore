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

import json
from pathlib import Path

SPACING = 200
WIDTH = 12000
HEIGHT = 7000

# Storage field: travel lanes form a lattice, storage blocks between them are solid.
FX0, FX1 = 3400, 9000
FY0, FY1 = 1400, 5600
VP, HP = 800, 1400              # lane pitch -> 3 x 6 cell storage blocks

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

BULK_X = (800, 2000, 3200)      # bulk storage: wider lane pitch than the field
BULK_Y = (1400, 2400)
BUF_Y = (2600, 3200)            # put-away buffer, directly under receiving

(R_HIGHWAY, R_FIELD, R_IN_DOCK, R_RECEIVING, R_BUFFER, R_BULK, R_STOW, R_PICK,
 R_PACK, R_SORT, R_OUT_DOCK, R_CROSSDOCK, R_CHARGING, R_PARKING) = range(1, 15)

REGIONS = [
    {"id": R_HIGHWAY, "name": "ring_highway", "density": "medium",
     "description": "Perimeter highway loop; every zone hangs off it."},
    {"id": R_FIELD, "name": "grid_field", "density": "dense",
     "description": "Lattice of travel lanes around 21 solid storage blocks."},
    {"id": R_IN_DOCK, "name": "inbound_dock", "density": "sparse",
     "description": "3 receiving doors on the west wall."},
    {"id": R_RECEIVING, "name": "receiving_inspection", "density": "sparse",
     "description": "Unload, verify and inspect goods before they enter stock."},
    {"id": R_BUFFER, "name": "inbound_buffer", "density": "sparse",
     "description": "Put-away buffer: inspected goods wait for a free stow station."},
    {"id": R_BULK, "name": "bulk_storage", "density": "sparse",
     "description": "Oversize and bulk stock on wide lanes, for items too large for "
                    "the grid field."},
    {"id": R_STOW, "name": "stow", "density": "sparse",
     "description": "Stow stations feeding accepted goods into the field."},
    {"id": R_PICK, "name": "pick", "density": "sparse",
     "description": "Pick stations on the east face of the field."},
    {"id": R_PACK, "name": "pack_vas", "density": "sparse",
     "description": "Pack and value-added services: kitting, repack, labelling."},
    {"id": R_SORT, "name": "sort_staging", "density": "sparse",
     "description": "Sort and stage completed orders by outbound route."},
    {"id": R_OUT_DOCK, "name": "outbound_dock", "density": "sparse",
     "description": "5 shipping doors on the east wall."},
    {"id": R_CROSSDOCK, "name": "crossdock", "density": "medium",
     "description": "Express lane carrying goods inbound to outbound without storing them."},
    {"id": R_CHARGING, "name": "charging", "density": "sparse",
     "description": "Charger bank spanning the full south edge, bays down both sides "
                    "of the spine."},
    {"id": R_PARKING, "name": "parking", "density": "sparse",
     "description": "Idle-robot bays filling the north strip, both sides of the "
                    "cross-dock lane."},
]

# When paths overlap, the most specific node type and the lowest region id win.
TYPE_PRIORITY = {"PT": 0, "YI": 1, "PK": 2, "CH": 3, "TR": 4}

VLANES = list(range(FX0, FX1 + 1, VP))
HLANES = list(range(FY0, FY1 + 1, HP))

SPINE_TIES = (3800, 6200, 8600)   # offset from bay lanes so traffic misses the bays
IN_DOORS = (4200, 4800, 5400)
OUT_DOORS = (1400, 2400, 3400, 4400, 5400)
RECEIVE_H = (3400, 4200, 5600)
RECEIVE_V = (1200, 2000, 2800)

CHARGERS = [x for x in range(1200, 10801, 600) if x not in SPINE_TIES]
PARK_BAYS = [x for x in range(1200, 10801, 400) if x not in SPINE_TIES]

# Pull-over bays: (anchor node on the lane, bay node one cell to the side).
YIELD_BAYS = (
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


def hline(y, x0, x1):
    return [(x, y) for x in range(x0, x1 + SPACING, SPACING)]


def vline(x, y0, y1):
    return [(x, y) for y in range(y0, y1 + SPACING, SPACING)]


class MapBuilder:
    def __init__(self):
        self.nodes = {}
        self.edges = set()

    def path(self, points, region_id, node_type="PT"):
        for p in points:
            self._add(p, region_id, node_type)
        for a, b in zip(points, points[1:]):
            self.edges.add(tuple(sorted((a, b))))

    def _add(self, p, region_id, node_type):
        cur = self.nodes.get(p)
        if cur is None:
            self.nodes[p] = {"region_id": region_id, "node_type": node_type}
            return
        cur["region_id"] = min(cur["region_id"], region_id)
        if TYPE_PRIORITY[node_type] > TYPE_PRIORITY[cur["node_type"]]:
            cur["node_type"] = node_type

    def mark(self, points, node_type):
        for p in points:
            self._add(p, self.nodes[p]["region_id"], node_type)

    def bay(self, anchor, tip, region_id, node_type):
        """One-cell spur off a lane: somewhere to stand that is not on the path."""
        self.path([anchor, tip], region_id)
        self.mark([tip], node_type)


def build():
    b = MapBuilder()

    # Ring highway.
    b.path(hline(HW_S, HW_W, HW_E), R_HIGHWAY)
    b.path(hline(HW_N, HW_W, HW_E), R_HIGHWAY)
    b.path(vline(HW_W, HW_S, HW_N), R_HIGHWAY)
    b.path(vline(HW_E, HW_S, HW_N), R_HIGHWAY)

    # Storage field: lane lattice, solid blocks between the lanes.
    for x in VLANES:
        b.path(vline(x, FY0, FY1), R_FIELD)
    for y in HLANES:
        b.path(hline(y, FX0, FX1), R_FIELD)
    for x in VLANES[:-1]:
        for y in HLANES[:-1]:
            b.mark([(x, y + 600)], "TR")

    # Inbound: dock doors -> receiving & inspection floor.
    for y in IN_DOORS:
        b.path(hline(y, X_IN_DOCK, HW_W), R_IN_DOCK)
        b.mark([(X_IN_DOCK, y)], "TR")
    for y in RECEIVE_H:
        b.path(hline(y, HW_W, X_STOW), R_RECEIVING)
    for x in RECEIVE_V:
        b.path(vline(x, BUF_Y[0], RECEIVE_H[-1]), R_RECEIVING)
        b.mark([(x, 3400), (x, 4200)], "TR")

    # Put-away buffer: inspected goods wait here for a free stow station.
    for y in BUF_Y:
        b.path(hline(y, HW_W, X_STOW), R_BUFFER)
    for x in RECEIVE_V:
        b.mark([(x, 2800), (x, 3000)], "TR")

    # Bulk / oversize storage: wider lanes, larger blocks than the grid field.
    for x in BULK_X:
        b.path(vline(x, BULK_Y[0], BULK_Y[1]), R_BULK)
    for y in BULK_Y:
        b.path(hline(y, BULK_X[0], BULK_X[-1]), R_BULK)
    b.path(vline(BULK_X[1], BULK_Y[1], BUF_Y[0]), R_BULK)
    for x in BULK_X[:-1]:
        b.mark([(x + 400, BULK_Y[0]), (x + 400, BULK_Y[1])], "TR")

    # Stow column feeding the west face of the field.
    b.path(vline(X_STOW, FY0, FY1), R_STOW)
    for y in HLANES:
        b.path(hline(y, X_STOW, FX0), R_STOW)
        b.mark([(X_STOW, y)], "TR")

    # Outbound: field east face -> pick -> pack/VAS -> sort/stage -> shipping doors.
    for x, region in ((X_PICK, R_PICK), (X_PACK, R_PACK), (X_SORT, R_SORT)):
        b.path(vline(x, FY0, FY1), region)
        b.mark([(x, y) for y in HLANES], "TR")
    for y in HLANES:
        b.path(hline(y, FX1, X_PICK), R_PICK)
        b.path(hline(y, X_PICK, HW_E), R_SORT)
    for y in OUT_DOORS:
        b.path(hline(y, HW_E, X_OUT_DOCK), R_OUT_DOCK)
        b.mark([(X_OUT_DOCK, y)], "TR")

    # Charging bank: spine across the full south edge, chargers down both sides.
    b.path(hline(Y_CHARGE, HW_W, HW_E), R_CHARGING)
    for x in CHARGERS:
        b.bay((x, Y_CHARGE), (x, Y_CHARGE_S), R_CHARGING, "CH")
        b.bay((x, Y_CHARGE), (x, Y_CHARGE_N), R_CHARGING, "CH")

    # North strip: cross-dock express lane with parking bays down both sides.
    b.path(hline(Y_CROSSDOCK, HW_W, HW_E), R_CROSSDOCK)
    for x in PARK_BAYS:
        b.bay((x, Y_CROSSDOCK), (x, Y_PARK_N), R_PARKING, "PK")
        b.bay((x, Y_CROSSDOCK), (x, Y_PARK_S), R_PARKING, "PK")

    # Tie the south and north bands into the ring and the field edges.
    for x in SPINE_TIES:
        b.path(vline(x, HW_S, FY0), R_CHARGING)
        b.path(vline(x, FY1, HW_N), R_CROSSDOCK)

    # Pull-over bays where traffic converges.
    for anchor, tip in YIELD_BAYS:
        b.bay(anchor, tip, b.nodes[anchor]["region_id"], "YI")

    return b


def serialize(b):
    ordered = sorted(b.nodes.items(), key=lambda kv: (kv[0][1], kv[0][0]))
    ids = {pos: i for i, (pos, _) in enumerate(ordered)}
    region_name = {r["id"]: r["name"] for r in REGIONS}
    seq = {}
    nodes = []
    for pos, meta in ordered:
        key = (meta["region_id"], meta["node_type"])
        seq[key] = seq.get(key, 0) + 1
        nodes.append({
            "id": ids[pos],
            "name": f"{region_name[meta['region_id']]}/{meta['node_type']}/{seq[key]:03d}",
            "region_id": meta["region_id"],
            "node_type": meta["node_type"],
            "position": {"x": float(pos[0]), "y": float(pos[1])},
        })
    edges = sorted(
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


TYPE_COLORS = {"PT": "#9aa4ad", "TR": "#1857b8", "CH": "#0f9d58", "PK": "#9a6b00", "YI": "#d23f31"}
REGION_BOXES = [
    ("#faf3e6", HW_W, RECEIVE_H[0], X_STOW, RECEIVE_H[-1], "RECEIVING & INSPECTION"),
    ("#fdf3e2", HW_W, BUF_Y[0], X_STOW, BUF_Y[1], "PUT-AWAY BUFFER"),
    ("#efe8f7", BULK_X[0], BULK_Y[0], BULK_X[-1], BULK_Y[1], "BULK / OVERSIZE STORAGE"),
    ("#e8eef8", FX0, FY0, FX1, FY1, "GRID FIELD - 21 storage blocks"),
    ("#eef4ea", X_PICK, FY0, X_SORT, FY1, "PICK / PACK / SORT"),
    ("#e9f6ee", HW_W, HW_S, HW_E, FY0, "CHARGING"),
    ("#eceff4", HW_W, FY1, HW_E, HW_N, "PARKING"),
]


def storage_blocks():
    """Solid block between four lanes, inset half a cell so it never covers a lane."""
    half = SPACING // 2
    for x in VLANES[:-1]:
        for y in HLANES[:-1]:
            yield x + half, y + half, VP - SPACING, HP - SPACING


def bulk_blocks():
    half = SPACING // 2
    for x0, x1 in zip(BULK_X, BULK_X[1:]):
        yield x0 + half, BULK_Y[0] + half, x1 - x0 - SPACING, BULK_Y[1] - BULK_Y[0] - SPACING


def to_svg(doc, scale=0.11, pad=90):
    w, h = WIDTH * scale + pad * 2, HEIGHT * scale + pad * 2
    px = lambda x: pad + x * scale
    py = lambda y: pad + (HEIGHT - y) * scale
    pos = {n["id"]: (n["position"]["x"], n["position"]["y"]) for n in doc["nodes"]}

    out = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{w:.0f}" height="{h:.0f}" '
           f'viewBox="0 0 {w:.0f} {h:.0f}" font-family="monospace">',
           f'<rect width="{w:.0f}" height="{h:.0f}" fill="#ffffff"/>']

    for tint, x0, y0, x1, y1, _ in REGION_BOXES:
        out.append(f'<rect x="{px(x0):.1f}" y="{py(y1):.1f}" width="{(x1-x0)*scale:.1f}" '
                   f'height="{(y1-y0)*scale:.1f}" fill="{tint}" stroke="#c9d0d6" '
                   f'stroke-dasharray="4 3"/>')

    for bx, by, bw, bh in list(storage_blocks()) + list(bulk_blocks()):
        out.append(f'<rect x="{px(bx):.1f}" y="{py(by+bh):.1f}" width="{bw*scale:.1f}" '
                   f'height="{bh*scale:.1f}" fill="#c3ccd8" stroke="#9aa6b4" stroke-width="0.5"/>')

    out.append(f'<rect x="{px(0):.1f}" y="{py(HEIGHT):.1f}" width="{WIDTH*scale:.1f}" '
               f'height="{HEIGHT*scale:.1f}" fill="none" stroke="#191e21" stroke-width="2"/>')

    for e in doc["edges"]:
        (x1, y1), (x2, y2) = pos[e["a"]], pos[e["b"]]
        out.append(f'<line x1="{px(x1):.1f}" y1="{py(y1):.1f}" x2="{px(x2):.1f}" '
                   f'y2="{py(y2):.1f}" stroke="#b6bec6" stroke-width="1.1"/>')

    for n in doc["nodes"]:
        t = n["node_type"]
        x, y = n["position"]["x"], n["position"]["y"]
        r = 1.5 if t == "PT" else 3.0
        out.append(f'<circle cx="{px(x):.1f}" cy="{py(y):.1f}" r="{r}" fill="{TYPE_COLORS[t]}"/>')

    for _, x0, y0, x1, y1, label in REGION_BOXES:
        tx, ty = px(x0) + 8, py(y1) + 17
        out.append(f'<rect x="{tx-4:.1f}" y="{ty-12:.1f}" width="{len(label)*6.9:.1f}" '
                   f'height="16" fill="#ffffff" fill-opacity="0.9"/>')
        esc = label.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")
        out.append(f'<text x="{tx:.1f}" y="{ty:.1f}" font-size="11.5" fill="#2b3439">{esc}</text>')

    legend = [("PT", "pass-through"), ("TR", "transfer"), ("CH", "charging"),
              ("PK", "parking bay"), ("YI", "yield / pull-over bay")]
    for i, (t, label) in enumerate(legend):
        lx, ly = pad + i * 175, pad + HEIGHT * scale + 40
        out.append(f'<circle cx="{lx}" cy="{ly-4}" r="4" fill="{TYPE_COLORS[t]}"/>')
        out.append(f'<text x="{lx+12}" y="{ly}" font-size="12" fill="#191e21">{t} {label}</text>')
    out.append(f'<text x="{pad}" y="{pad-30}" font-size="15" fill="#191e21">'
               f'AMR warehouse map — {WIDTH/100:.0f} m x {HEIGHT/100:.0f} m, '
               f'{len(doc["nodes"])} nodes, {len(doc["edges"])} edges, '
               f'{SPACING} cm spacing — flow runs west to east</text>')
    out.append("</svg>")
    return "\n".join(out)


def to_ascii(doc, step=1):
    glyph = {"PT": "·", "TR": "T", "CH": "C", "PK": "P", "YI": "+"}
    cell = SPACING * step
    cols, rows = WIDTH // cell + 1, HEIGHT // cell + 1
    best = [[None] * cols for _ in range(rows)]
    for n in doc["nodes"]:
        c, r = int(n["position"]["x"]) // cell, int(n["position"]["y"]) // cell
        t = n["node_type"]
        if best[r][c] is None or TYPE_PRIORITY[t] > TYPE_PRIORITY[best[r][c]]:
            best[r][c] = t
    lines = ["+" + "-" * cols + "+"]
    for r in range(rows - 1, -1, -1):
        lines.append("|" + "".join(" " if t is None else glyph[t] for t in best[r]) + "|")
    lines.append("+" + "-" * cols + "+")
    return "\n".join(lines)


def main():
    here = Path(__file__).parent
    b = build()
    doc, _ = serialize(b)

    (here / "warehouse.json").write_text(json.dumps(doc, indent=2) + "\n")
    (here / "warehouse_map.svg").write_text(to_svg(doc) + "\n")

    counts = {}
    for n in doc["nodes"]:
        counts[n["node_type"]] = counts.get(n["node_type"], 0) + 1
    print(to_ascii(doc))
    print(f"\n{WIDTH/100:.0f} m x {HEIGHT/100:.0f} m, {SPACING} cm spacing")
    print(f"{len(doc['nodes'])} nodes, {len(doc['edges'])} edges, "
          f"{len(VLANES)}x{len(HLANES)} field lanes, "
          f"{sum(1 for _ in storage_blocks())} storage blocks")
    print("by type: " + ", ".join(f"{k}={v}" for k, v in sorted(counts.items())))
    for r in REGIONS:
        n = sum(1 for x in doc["nodes"] if x["region_id"] == r["id"])
        print(f"  {r['id']:>2} {r['name']:<22} {n:>4} nodes  ({r['density']})")


if __name__ == "__main__":
    main()
