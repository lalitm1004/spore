# spore-warehouse-layout

Generates a static AMR (Autonomous Mobile Robot) warehouse map as a JSON graph
of nodes and edges, plus an SVG visualization. The map is deterministic,
standard-library only, and meant to be consumed by the Spore simulation stack.

## Layout

A 120 m × 70 m fulfilment-centre floor plan with a QR code node every 200 cm on
every path. Zones run in material-flow order, inbound (west) to outbound (east),
around a central lattice storage field. All movement is axis-aligned (90-degree
turns only).

## Install & run

```sh
uv sync
uv run spore-warehouse-layout
```

Outputs are written to `./output/`:

- `warehouse.json` — the node/edge graph (schema `v0.1.0`)
- `warehouse_map.svg` — visual rendering of the layout

## Output format

`warehouse.json` contains:

| Field            | Description                                              |
| ---------------- | -------------------------------------------------------- |
| `schema_version` | Version of the output schema                             |
| `units`          | Distance units (`cm`)                                    |
| `node_spacing`   | Grid spacing between nodes along a path (`200` cm)       |
| `dimensions`     | Bounding box of the floor plan                            |
| `regions`        | The 14 functional zones, each with `id`, `name`, `density` |
| `nodes`          | Graph nodes with `id`, `name`, `region_id`, `node_type`, `position` |
| `edges`          | Axis-aligned connections between node ids, each `200` cm long |

## Node types

| Type | Name                 | Count | Description                                        |
| ---- | -------------------- | ----- | -------------------------------------------------- |
| `PT` | pass-through         | 721   | Plain path nodes along travel lanes                |
| `TR` | transfer             | 61    | Points where goods change mode (dock, stow, pick)  |
| `PK` | parking bay          | 50    | Idle-robot bays along the north strip              |
| `CH` | charging             | 34    | Charger bays along the south spine                 |
| `YI` | yield / pull-over bay | 15    | One-node spurs where a robot can step off a busy lane |

**Totals:** 881 nodes, 952 edges.

## Regions

| id | name                 | density | description                                        |
| -- | -------------------- | ------- | -------------------------------------------------- |
| 1  | ring_highway         | medium  | Perimeter highway loop; every zone hangs off it.   |
| 2  | grid_field           | dense   | Lattice of travel lanes around 21 solid storage blocks. |
| 3  | inbound_dock         | sparse  | 3 receiving doors on the west wall.                |
| 4  | receiving_inspection | sparse  | Unload, verify and inspect goods.                  |
| 5  | inbound_buffer       | sparse  | Put-away buffer for inspected goods.               |
| 6  | bulk_storage         | sparse  | Oversize/bulk stock on wide lanes.                 |
| 7  | stow                 | sparse  | Stow stations feeding the field.                   |
| 8  | pick                 | sparse  | Pick stations on the east face of the field.       |
| 9  | pack_vas             | sparse  | Pack and value-added services.                     |
| 10 | sort_staging         | sparse  | Sort and stage orders by outbound route.           |
| 11 | outbound_dock        | sparse  | 5 shipping doors on the east wall.                 |
| 12 | crossdock            | medium  | Express inbound-to-outbound lane.                  |
| 13 | charging             | sparse  | Charger bank along the south edge.                 |
| 14 | parking              | sparse  | Idle-robot bays along the north strip.             |
