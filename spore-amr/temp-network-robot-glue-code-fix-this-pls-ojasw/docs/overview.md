# temp-network-interface — Overview

`temp-network-interface` is the AMR **network-layer interface**: the gRPC link between a
warehouse robot and the network layer, following the shared JSON schemas. It is
one of three sibling projects under the `spore` repository:

```
spore/
  spore-amr/                 the Webots robot implementation (firmware + companion)
  spore-warehouse-layout/    generates the static warehouse map (nodes + edges)
  temp-network-interface/             THIS PROJECT — the robot <-> network interface
```

This document is the complete reference: what the project is, what it does, and
what it affects.

---

## 1. What it is

A standalone `uv` project (Python ≥ 3.13) that implements the **robot ↔ network
interface** defined by two JSON schemas in `spore-amr/shared/schemas/`. Those
schemas are the ground-truth contract for the whole system; this project is the
concrete transport and the running code that both sides speak.

It is deliberately standalone *for now*. The robot-side half (`client.py` and the
pure core) is written to be pasted into the Webots companion
(`spore-amr/webots/robot/companion.py`) later, so it is pure, host-testable, and
imports `grpc` only behind a thin adapter boundary.

### The interface

Two directions, one schema each:

| direction | schema | meaning |
|---|---|---|
| robot → network | `robot-to-network.schema.json` | a robot reports its status upward |
| network → robot | `network-to-robot.schema.json` | the network commands a robot |

There is **no existing gRPC for this link anywhere in the repo** — the schemas
are the only prior definition. (`spore-amr/webots/proto/firmware.proto` is a
drafted gRPC schema for a *different* link, the ESP32↔Pi firmware↔companion
serial line, and is unreferenced.)

### Dependencies

- **runtime**: `grpcio`, `jsonschema`
- **dev**: `grpcio-tools` (stub generation), `pytest`

---

## 2. What it does

### 2.1 The transport

`proto/network.proto` defines a single bidirectional-streaming RPC:

```proto
service RobotNetwork {
  rpc Session(stream RobotToNetwork) returns (stream NetworkToRobot);
}
```

Both schemas are expressed as typed messages, field for field. The stream types
name the direction, so there is no envelope and no `schema` discriminator to get
wrong: what a robot may send and what it may receive are different types.

Two things proto3 cannot say, handled in the file rather than left as traps:

- **`required`** does not exist, and every `Id`/`Timestamp` has `minimum: 0`, so
  zero is legal and an absent field would be indistinguishable from a present
  zero. Required scalars are declared `optional`, which buys explicit presence
  at no cost on the wire. This makes required-ness *weaker* at the protobuf
  level — an empty message is structurally valid — so the JSON Schema
  validation in `transport.py` is what still enforces it.
- **`oneOf`** maps to `oneof` exactly, for `Mission` and `Warning`. `Fault` is
  deliberately not a oneof: its schema requires nothing and forbids nothing, so
  a robot may report a warning and an error at once.

The schemas remain the ground-truth contract, and a network layer written in any
language can validate against the same files. There are two definitions of this
interface now where the envelope had one; `tests/test_proto_matches_schemas.py`
is what catches them drifting.

### 2.2 The domain model (`messages.py`)

Typed dataclasses mirror the schemas field-for-field. String-valued fields are
constrained to the schema's enums at construction, so an invalid value fails
immediately rather than surfacing as a schema error at the wire.

| message | fields |
|---|---|
| `Battery` | `percentage` |
| `Telemetry` | `battery` |
| `Cargo` | `cargo_id` (UUID), `state` (`PICKUP`/`DROPOFF`/`EN_ROUTE`) |
| `Mission` | `type` (`PARK`/`CHARGE`/`HOLD`/`IDLE`/`CARGO`), optional `cargo` |
| `Warning` | `type` (`LOW_BATTERY`/`OBSTACLE`), optional `percentage` / `current_node_id` |
| `Error` | `type` (`MOTOR_ERROR`/`CAMERA_ERROR`/`LIDAR_ERROR`/`LOCATION_UNKNOWN`/`MISC_ERROR`) |
| `Fault` | optional `warning`, optional `error` |
| `RobotToNetwork` | `bot_id`, `region_id`, `latest_node_id`, `mission`, `telemetry`, `timestamp`, optional `fault` |
| `NetworkToRobot` | `target_node_id`, `timestamp`, optional `set_mission` |
| `RobotState` | `target_node_id`, `mission`, `timestamp` — see §2.5 |

### 2.3 Validation at the boundary (`schemas.py`, `transport.py`)

`schemas.py` compiles the two JSON Schemas into cached draft-2020-12 validators.
`transport.py` marshals domain messages into protobuf and validates every
payload as it crosses the wire — a payload that fails validation never leaves
the process.

**The schema document is the pivot.** Encoding goes domain → document →
validate → protobuf, and decoding goes protobuf → document → validate → domain.
Nothing converts a domain object to a protobuf directly, so validation stays at
the boundary, `messages.py` needs no protobuf awareness, and a required field
missing from the wire surfaces as a schema error naming the field rather than as
a silent zero.

### 2.4 The robot side (`client.py`)

`NetworkClient` is what the companion will use. It opens one long-lived
bidirectional stream, decoupled by queues so the control loop never blocks on
the network:

- `send(RobotToNetwork)` — push a status report. Non-blocking; drops the message
  if the outbound buffer is full (status is a heartbeat, a newer one replaces
  it). Returns `False` when disconnected.
- `recv(timeout)` — the next `NetworkToRobot`, or `None` on timeout/end.
- A stalled or absent network degrades to no-ops, never failure.

### 2.5 The robot's state (`RobotState`)

This is the core idea: **the firmware never knows about other robots.** The
entirety of a robot's knowledge is its own goal, which is exactly what the
`network-to-robot` schema conveys (`target_node_id` + `set_mission`).

`NetworkClient.state` is a `RobotState` — the latest command projected into a
plain `(target_node_id, mission, timestamp)` — updated as each command arrives
and **preserved** until the next one. This is what the companion reads in its
loop to drive the firmware. The fleet is the network layer's concern and never
leaks to the robot.

### 2.6 The network side (`server.py`, `state.py`, `relay.py`, `policy.py`)

`NetworkService` is the network layer's gRPC server, one bidirectional stream
per robot:

- Incoming `RobotToNetwork` messages are persisted into the durable `Fleet`.
- The `Policy` maps fleet state onto `TargetedCommand(bot_id, command)` — so a
  decision triggered by one robot can command another.
- Commands are persisted and `Relay`ed to the destination robot's stream.

`Fleet` is the authoritative global state (latest status + outstanding commands
per robot). It is **thread-safe** — one connection per robot runs concurrently,
and a command for robot A can be produced by robot B's message.

### 2.7 Persistence (`store.py`)

State and commands are durable. `Journal` is an append-only JSONL file, one
record per line with a flush-per-line (a crash truncates at most the last
record, which replay skips). Records:

```
{"type": "status",  "bot_id": 5, "status":  {...}}   # a robot reported
{"type": "command", "bot_id": 5, "command": {...}}   # a command was issued
```

`Fleet.load(journal)` replays the journal on startup to reconstruct the global
state, then keeps appending to it.

### 2.8 Reconciliation (`state.fulfilled`)

A command stays outstanding until the robot's next status shows it was carried
out — arrival at `target_node_id`, or a matching mission (and, for cargo, the
matching cargo and state). This is what keeps the world state honest: at any
moment the fleet can say what each robot should be doing and whether it is done.

### 2.9 CLI (`__main__.py`)

```
temp-network-interface serve  [--address] [--journal PATH] [--policy hold|noop]
temp-network-interface probe  [--target] [--bot-id] [--region-id] [--node-id]
```

`--journal` makes the network layer's global state durable across restarts.

---

## 3. What it affects

### 3.1 The Webots implementation (`spore-amr`)

This is the primary consumer. The paste-in integration is:

1. On each `MARKER` event the firmware reports, the companion builds a
   `RobotToNetwork` (`bot_id`, `region_id`, `latest_node_id`, current `mission`,
   `telemetry`) and calls `client.send(...)`.
2. The companion reads `client.state` (its `target_node_id` + `mission`) in its
   loop and drives the firmware from it. This is the seam `robot/turn.py` and
   the drafted `proto/firmware.proto` already anticipate.
3. `TEMP_NETWORK_INTERFACE_SCHEMA_DIR` is pointed at `spore-amr/shared/schemas/` so the
   canonical schemas, not the local copies, are validated against.

Adding it pulls `grpcio` and `jsonschema` into the Webots project's dependency
set.

### 3.2 The shared schemas

The project consumes `robot-to-network.schema.json` and
`network-to-robot.schema.json` (canonical copies live under `schemas/` for now).
Any change to those schemas changes what this project accepts/emits — they are
the one contract the two halves must not diverge from.

### 3.3 Deployment topology

When deployed, the network layer runs as a service (or, in the per-robot
co-location model, alongside each robot), and each robot's companion runs a
`NetworkClient` against it. The `Fleet`/`Journal` give the network layer a
durable, global view of the world; the `RobotState` gives each robot only its
own view.

### 3.4 Concurrency model

One gRPC stream per robot runs concurrently. `Fleet`, `Journal`, and `Relay` are
thread-safe. Command delivery is **at-least-once**: a command is persisted,
delivered if the robot is connected, and re-delivered if the robot reconnects
before reconciling it (the stub commands are idempotent, so a duplicate is
harmless).

---

## 4. Module reference

| module | role | imports grpc? |
|---|---|---|
| `messages.py` | typed domain messages + `RobotState` | no |
| `schemas.py` | schema loading + validation | no |
| `state.py` | `Fleet` (global state) + `TargetedCommand` + `fulfilled` | no |
| `store.py` | `Journal` (append-only JSONL durability) | no |
| `relay.py` | routes commands to the right robot's stream | no |
| `policy.py` | `Policy` protocol + `HoldPolicy` / `NoopPolicy` stubs | no |
| `transport.py` | domain ↔ protobuf marshalling, validates at the wire | **yes** |
| `client.py` | `NetworkClient` (robot side) | **yes** |
| `server.py` | `NetworkService` + `serve()` (network side) | **yes** |
| `__main__.py` | CLI (`serve`, `probe`) | lazy |
| `network_pb2*.py` | generated protobuf stubs | **yes** |

The split is the same boundary the Webots implementation draws around the
Webots API: pure core, thin adapter.

## 5. Commands

```
uv sync                                  # install
uv run pytest -q                         # 37 tests, no simulator needed
uv run python tools/gen_proto.py         # regenerate gRPC stubs after a proto edit
uv run temp-network-interface serve --journal state/fleet.jsonl
uv run temp-network-interface probe --target localhost:50051 --bot-id 1
```

## 6. Boundaries & invariants

- **Schemas are authoritative.** The wire is typed protobuf; the documents
  are validated against the shared schemas at the wire.
- **The network layer knows the fleet; the firmware knows only itself.** A robot
  holds its own `RobotState` and nothing else.
- **Pure core, thin grpc adapter.** Only `transport.py`, `client.py`,
  `server.py` import `grpc`.
- **The client never blocks control.** Status is expendable; a dead network
  degrades to no-ops.
- **State and commands are durable** via the journal, reconstructed on startup.

## 7. Seams & open questions

- `Policy.on_status(fleet, status) -> list[TargetedCommand]` is where real task
  allocation slots in; `HoldPolicy`/`NoopPolicy` are stubs.
- Command delivery is at-least-once; exactly-once needs correlation IDs, which
  the schema deliberately does not carry.
- `timestamp` is an integer in the schemas but the Webots world runs at a
  simulation clock, not wall time — the mapping is undecided.
- The network↔network peer protocol (robot-to-robot coordination) is **not**
  covered by these schemas and does not exist yet.
