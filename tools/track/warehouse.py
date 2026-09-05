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


def to_document(graph, node_spacing_cm: int, plane_m: float) -> Dict:
    """The whole map, ready to be written as JSON."""
    regions = sorted({node.region_id for node in graph.nodes.values()})

    return {
        "schema_version": SCHEMA_VERSION,
        "units": "cm",
        "node_spacing": int(node_spacing_cm),
        "dimensions": {
            "width": int(round(plane_m * 100)),
            "height": int(round(plane_m * 100)),
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
                    "x": round((node.x + plane_m / 2.0) * 100, 1),
                    "y": round((node.y + plane_m / 2.0) * 100, 1),
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
