# spore-control-plane

Central control plane for the Spore AMR fleet. It provides a web interface to
create cargo orders (*"collect goods at node-X, deliver them to node-Y"*) and
hands each order to the fleet over gRPC. The fleet does the routing — the
control plane knows no regions and no leaders, because it can't.

This project is deliberately **separate** from the fleet: it owns its own wire
schema (`proto/controlplane.proto`) and never imports or depends on the
network layer's code. The network layer implements this project's proto.

Read [`DOCUMENTATION.md`](DOCUMENTATION.md) for the goal, the decisions, what
the project actually does, and the exact edits the network layer needs to make
to integrate with it.

## Layout

```
proto/controlplane.proto        the contract the network layer implements
src/spore_control_plane/
  config.py                     env-driven configuration
  map.py                        warehouse-layout.json -> node-id validation only
  submitter.py                  DispatchOrder client + retry loop
  client.py                     gRPC channel pool + wire identity metadata
  app.py                        FastAPI web app (form + POST /orders)
  proto/                        generated stubs (controlplane_pb2*.py)
templates/order.html            the order form
shared/warehouse-layout.json    copied from spore-amr/shared (node validation only)
```

## Quick start

```bash
uv sync
BOT_ADDRESSES=amr-region-14-bot-0:50051,amr-region-2-bot-0:50051 uv run spore-control-plane
# open http://localhost:8000
```

`BOT_ADDRESSES` is a comma-separated list of `host:port` of bots the control
plane may talk to. It doesn't need any specific bot — any one of them will
forward the order to the right region/leader. The example addresses match the
container names the network layer's `up.py` creates on a local Docker fleet
(`amr-region-<region>-bot-<id>` on gRPC port `50051`); in Kubernetes they'd be
the pod DNS names the controller already knows.

You can run without `BOT_ADDRESSES` and the app still starts — the web UI comes
up, but every order fails with "not dispatched" because there's nobody to ask.

## Configuration

All via environment (see `config.py` for defaults):

| Var | Meaning |
|---|---|
| `BOT_ADDRESSES` | comma-separated `host:port` of bots to talk to (the control plane already knows these because it boots the fleet). |
| `CONTROL_BOT_ID` / `CONTROL_REGION_ID` / `CONTROL_ROLE` | the reserved identity sent in gRPC metadata (the network layer admits `ControlPlaneService` by this identity). |
| `WAREHOUSE_MAP` | path to `warehouse-layout.json` (used only to validate node ids). |
| `GRPC_TIMEOUT` | per-RPC timeout in seconds. |
| `DISPATCH_ATTEMPTS` | how many passes over `BOT_ADDRESSES` an order dispatch makes before giving up; default `5`. |
| `DISPATCH_BACKOFF` | seconds to wait between dispatch passes; default `1.0`. |
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

## The network layer's side

Implemented: `spore-amr/network-layer/bus/control_plane.py` serves
`ControlPlaneService.DispatchOrder` on every bot, translating an `Order` to the
fleet's own `Job` and handing it to the existing dispatcher, which already
routes follower → leader → pickup region's leader. The proto is vendored into
`network-layer/proto/` with a test that fails if the copy drifts from this one.
Proven on real containers by scenarios B13–B19 in
`network-layer/docs/scenarios.md`, and placed on the live simulator fleet with
`spore-amr/webots/fleet.sh order <pickup> <dropoff>`.

## Tests

```bash
uv run pytest
```

Tests use a mock `ControlPlaneService` gRPC server and FastAPI's test client,
so the web → dispatch path (including fallback across bots and retry) is
verifiable without the network layer.
