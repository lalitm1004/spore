"""Export a lane graph as `warehouse.json`.

Deliberately the same shape as `spore-warehouse-layout/output/warehouse.json`:
schema_version, units, node_spacing, dimensions, regions, nodes, edges, with
positions in centimetres. A robot cannot tell the simulated map from the real
one, so the code that reads it here is the code that reads it there.

This is what makes the shared QR schema sufficient. The marker says only which
node the robot is on; everything else -- which lanes leave it, which way each
one runs -- comes from this file, which every robot holds.

Pure: builds a dict, writes no files.
"""

from typing import Dict

SCHEMA_VERSION = "v0.1.0"

REGION_NAMES = {
    1: "southwest",
    2: "southeast",
    3: "northwest",
    4: "northeast",
}


def to_document(graph, node_spacing_cm: int, plane_m) -> Dict:
    """The whole map, ready to be written as JSON."""
    # `plane_m` may be a single float (square) or a (width, height) pair.
    try:
        plane_w, plane_h = float(plane_m[0]), float(plane_m[1])
    except TypeError:
        plane_w = plane_h = float(plane_m)

    regions = sorted({node.region_id for node in graph.nodes.values()})

    return {
        "schema_version": SCHEMA_VERSION,
        "units": "cm",
        "node_spacing": int(node_spacing_cm),
        "dimensions": {
            "width": int(round(plane_w * 100)),
            "height": int(round(plane_h * 100)),
        },
        "regions": [
            {
                "id": region,
                "name": REGION_NAMES.get(region, "region_{}".format(region)),
                "density": "sparse",
                "description": "Generated quadrant of the simulated lattice.",
            }
            for region in regions
        ],
        "nodes": [
            {
                "id": node.node_id,
                "name": node.name,
                "region_id": node.region_id,
                "node_type": node.kind,
                # Centimetres from the plane's corner, so nothing is negative
                # -- the same frame the QR payloads use.
                "position": {
                    "x": round((node.x + plane_w / 2.0) * 100, 1),
                    "y": round((node.y + plane_h / 2.0) * 100, 1),
                },
            }
            for node in sorted(graph.nodes.values(), key=lambda n: n.node_id)
        ],
        "edges": [
            {
                "a": edge.a,
                "b": edge.b,
                "length": int(round(graph.length(edge.a, edge.b) * 100)),
            }
            for edge in graph.edges
        ],
    }


def load_window(path, origin_cm, size_m):
    """A rectangular window of a real `warehouse.json`, as a Graph.

    The full 120 x 70 m warehouse cannot be simulated at line-following
    resolution: one texture would be 61440 x 35840 px against a GPU limit
    around 16384, and a QR tile for each of its 881 nodes would want roughly
    3.7 GB of texture memory. A window is the honest way to run the real
    layout -- real node ids, names, types, regions and edges, just fewer of
    them.

    Positions are rebased so the window's centre is the world origin, which is
    what Webots and the robots' odometry both work in. Node ids are untouched,
    so a marker read here names the same node it would in the full warehouse.
    """
    import json
    import pathlib

    from tools.track.graph import Edge, Graph, Node

    document = json.loads(pathlib.Path(path).read_text())
    x0, y0 = float(origin_cm[0]), float(origin_cm[1])
    width_cm, height_cm = size_m[0] * 100.0, size_m[1] * 100.0

    inside = {}
    for entry in document["nodes"]:
        x, y = float(entry["position"]["x"]), float(entry["position"]["y"])
        if not (x0 <= x < x0 + width_cm and y0 <= y < y0 + height_cm):
            continue
        inside[int(entry["id"])] = Node(
            node_id=int(entry["id"]),
            x=(x - x0) / 100.0 - size_m[0] / 2.0,
            y=(y - y0) / 100.0 - size_m[1] / 2.0,
            kind=entry.get("node_type", "PT"),
            name=entry.get("name", ""),
            region_id=int(entry.get("region_id", 0)),
        )

    # Only edges with both ends inside: half an edge leaving the window would
    # be a lane that runs off the floor.
    edges = [Edge(int(e["a"]), int(e["b"])) for e in document["edges"]
             if int(e["a"]) in inside and int(e["b"]) in inside]

    # Drop nodes the window left stranded -- a node with no lane is a marker
    # tile a robot can never reach.
    reachable = {n for e in edges for n in (e.a, e.b)}
    nodes = [n for n in inside.values() if n.node_id in reachable]
    return Graph(nodes, [e for e in edges])
