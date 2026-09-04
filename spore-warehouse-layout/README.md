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
| `regions`        | The 7 functional zones, each with `id`, `name`, `density`  |
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

Regions are grouped by function, not by construction step — e.g. the ring highway
(the perimeter loop every zone hangs off) has no nodes of its own: its south half
folds into `charging` and its north half into `parking`, since those zones already
run the full width of the building.

| id | name                        | density | description                                                                    |
| -- | --------------------------- | ------- | ------------------------------------------------------------------------------ |
| 1  | receiving_and_buffer        | sparse  | Inbound doors, receiving & inspection, put-away buffer and stow stations.      |
| 2  | parking                     | sparse  | Idle-robot bays, the cross-dock express lane, and the north half of the ring highway. |
| 3  | charging                    | sparse  | Charger bays along the south edge and the south half of the ring highway.      |
| 4  | pick_pack_sort               | sparse  | Pick, pack/VAS and sort/staging columns, plus the outbound shipping doors.     |
| 5  | bulk_and_storage_cols_1_2   | dense   | Bulk/oversize storage plus storage columns 1–2 (nearest stow).                 |
| 6  | storage_cols_3_4_5          | dense   | The middle three storage columns.                                              |
| 7  | storage_cols_6_7            | dense   | The last two storage columns (nearest pick).                                   |
