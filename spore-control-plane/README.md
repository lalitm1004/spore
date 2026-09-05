# spore-control-plane

Central control plane for the Spore AMR fleet. It provides a web interface to
create cargo orders (*"collect goods at node-X, deliver them to node-Y"*) and
dispatches each order, over gRPC, to the leader of the warehouse region where
the order starts.

This project is deliberately **separate** from the fleet: it owns its own wire
schema (`proto/controlplane.proto`) and never imports or depends on the
network layer's code. The network layer implements this project's proto.

Read [`DOCUMENTATION.md`](DOCUMENTATION.md) for the goal, the decisions, and
why things are shaped the way they are.

## Layout

```
proto/controlplane.proto        the contract the network layer implements
src/spore_control_plane/
  config.py                     env-driven configuration
  map.py                        warehouse-layout.json -> node/region lookup
  discovery.py                  DiscoverLeaders client + region->leader cache
  submitter.py                  DispatchOrder client (leader-first, any-bot fallback)
  client.py                     gRPC channel pool + wire identity metadata
  app.py                        FastAPI web app (form + POST /orders)
  proto/                        generated stubs (controlplane_pb2*.py)
templates/order.html            the order form
shared/warehouse-layout.json    copied from spore-amr/shared (validation + region lookup)
```

## Quick start

```bash
uv sync
BOT_ADDRESSES=amr-region-14-bot-0:50051,amr-region-2-bot-0:50051 uv run spore-control-plane
# open http://localhost:8000
```

Run without any bots configured (the app still starts; dispatch will simply
report that nobody accepted):

```bash
uv run spore-control-plane
```

## Configuration

All via environment (see `config.py` for defaults):

| Var | Meaning |
|---|---|
| `BOT_ADDRESSES` | comma-separated `host:port` of bots to talk to. The control plane already knows these because it boots the fleet. |
| `CONTROL_BOT_ID` / `CONTROL_REGION_ID` / `CONTROL_ROLE` | the reserved identity sent in gRPC metadata (the network layer admits `ControlPlaneService` by this identity). |
| `WAREHOUSE_MAP` | path to `warehouse-layout.json` (used for node validation + region lookup). |
| `GRPC_TIMEOUT` | per-RPC timeout in seconds. |
| `LEADER_CACHE_TTL` | how long a region->leader entry is trusted before re-discovery. |
| `HTTP_HOST` / `HTTP_PORT` | uvicorn bind (default `0.0.0.0:8000`). |

## Docker

```bash
docker build -t spore-control-plane:dev .
docker run --rm -p 8000:8000 \
  -e BOT_ADDRESSES=amr-region-14-bot-0:50051,amr-region-2-bot-0:50051 \
  spore-control-plane:dev
```

## Regenerating the proto stubs

```bash
uv run python -m grpc_tools.protoc -I proto \
  --python_out=src/spore_control_plane/proto \
  --grpc_python_out=src/spore_control_plane/proto \
  proto/controlplane.proto
sed -i 's/^import controlplane_pb2/from spore_control_plane.proto import controlplane_pb2/' \
  src/spore_control_plane/proto/controlplane_pb2_grpc.py
```

## What the network layer must implement

Copy `proto/controlplane.proto`, regenerate the stubs, and:

1. Serve `ControlPlaneService` on **every** bot.
2. `DispatchOrder` → map `Order` to the fleet's internal job and call the
   existing dispatcher (a non-leader forwards to its leader; a leader forwards
   to the pickup region's leader — this routing already exists).
3. `DiscoverLeaders` → return the bot's known leaders plus its own
   region/leader.
4. Add a virtual-network policy entry that admits `controlplane.ControlPlaneService`
   to the control plane's reserved identity (see `CONTROL_BOT_ID` etc.).

## Tests

```bash
uv run pytest
```

Tests use a mock `ControlPlaneService` gRPC server and FastAPI's test client,
so the web → client → dispatch path is verifiable without the network layer.
