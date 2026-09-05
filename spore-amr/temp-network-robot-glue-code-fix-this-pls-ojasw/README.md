# temp-network-interface

The AMR network-layer interface: the gRPC link between a warehouse robot and
the network layer, following the shared JSON schemas. This is a standalone
project for now; its robot-side client is written to be pasted into the Webots
companion (`spore-amr/webots/robot/companion.py`) later, so it is pure,
host-testable, and imports `grpc` only behind the transport boundary.

> Full reference — what it is, what it does, what it affects, plus module and
> message references — lives in [`docs/overview.md`](docs/overview.md).

## The interface

Two directions, one schema each:

| direction | schema | carries |
|---|---|---|
| robot → network | `robot-to-network.schema.json` | `bot_id`, `region_id`, `latest_node_id`, `mission`, `telemetry` (battery), optional `fault`, `timestamp` |
| network → robot | `network-to-robot.schema.json` | `target_node_id`, optional `set_mission`, `timestamp` |

The schemas are the ground-truth contract (canonical copies of
`spore-amr/shared/schemas/`). The gRPC service is a deliberate thin envelope —
`proto/network.proto` ships each payload as validated JSON — so the schemas can
evolve without touching the transport, and a network layer written in any
language validates against the same files.

```
robot (companion) ──gRPC bidi stream──► network layer
   NetworkClient                          NetworkService
   sends RobotToNetwork                   persists status into Fleet (journal)
   receives NetworkToRobot                persists commands, relays to the robot
```

## Layout

```
proto/network.proto         the gRPC service + opaque Message envelope
schemas/*.schema.json       the two shared contracts (copies)
src/temp_network_interface/
  messages.py               typed domain messages (pure)
  schemas.py                schema loading + validation (pure)
  state.py                  Fleet: global world state + reconciliation (pure)
  store.py                  Journal: append-only JSONL durability (pure)
  relay.py                  routes commands to the right robot's stream (pure)
  policy.py                 pluggable Policy (stub: HoldPolicy, NoopPolicy)
  transport.py              domain <-> envelope marshalling, validates at the wire
  client.py                 NetworkClient (robot side, for the companion)
  server.py                 NetworkService + serve()
  __main__.py               CLI: serve / probe
tools/gen_proto.py          regenerate the gRPC stubs
tests/                      35 host tests, including live loopback round trips
```

## Install & run

```sh
uv sync

# run the network service (--journal makes global state durable across restarts)
uv run temp-network-interface serve --address '[::]:50051' --journal state/fleet.jsonl

# in another shell, send one status and print the reply
uv run temp-network-interface probe --target localhost:50051 --bot-id 1
```

## Tests

```sh
uv run pytest -q
```

## Design notes

- **Schemas are authoritative.** Every payload is validated against its schema
  at the boundary where it crosses the wire (`transport.py`); a payload that
  fails validation never leaves the process, and an unknown schema name is
  refused rather than guessed at.
- **Pure core, thin grpc adapter.** Only `transport.py`, `client.py` and
  `server.py` import `grpc`. Everything else — messages, world state, durability,
  relay — is pure and host-testable, the same boundary the Webots implementation
  draws around the Webots API.
- **The client never blocks control.** Status is queued and dropped if the
  buffer is full (telemetry is expendable), and an absent network degrades to
  no-ops rather than failure.
- **State and commands are durable.** `Fleet` is the authoritative global state;
  every status and command is appended to a JSONL `Journal` (flush-per-line, so
  a crash truncates at most the last record), and `Fleet.load` replays it on
  startup. Give `serve --journal` a path to persist across restarts.
- **Commands are relayed, not just echoed.** A `Policy` returns
  `TargetedCommand(bot_id, command)`, so a decision triggered by one robot can
  command another. The `Relay` routes by `bot_id`; a robot that is offline keeps
  its command outstanding and receives it on reconnect.
- **Reconciliation keeps world state honest.** A command stays outstanding until
  the robot's next status shows it carried out (arrival at `target_node_id`, or
  a matching mission), so the fleet can always say what is done and what is not.
- **The policy is the seam for real task allocation.** `Policy.on_status(fleet,
  status) -> list[TargetedCommand]` is where a real allocator slots in; the
  current `HoldPolicy` is a deterministic stub, the analogue of
  `robot/network.py`'s `RandomRouter`.

## Pasting into the Webots implementation

The robot side is `client.py` + the pure core. When this moves into the
companion, the integration is:

1. On each `MARKER` event the firmware reports, build a `RobotToNetwork`
   (`bot_id`, `region_id`, `latest_node_id`, current `mission`, `telemetry`)
   and `client.send(...)`.
2. Read `client.state` in the companion's loop — `target_node_id` and `mission`
   are the robot's entire state, preserved from the latest `NetworkToRobot` and
   updated as new commands arrive. That state is what drives the firmware
   (`robot/turn.py` and the drafted `proto/firmware.proto` already anticipate
   it). The firmware never sees the fleet; it only sees its own goal.
3. Point `TEMP_NETWORK_INTERFACE_SCHEMA_DIR` at `spore-amr/shared/schemas/` so the
   canonical schemas, not the local copies, are validated against.

## Regenerating the stubs

```sh
uv run python tools/gen_proto.py
```

The generator patches the `_grpc.py` import so both generated files live
cleanly inside the `temp_network_interface` package.
