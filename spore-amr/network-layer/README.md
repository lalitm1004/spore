# network-layer

The communication layer of the AMR fleet: one process per robot that finds
its region-mates, elects a leader, keeps a live roster, follows the robot
across regions, gets cargo jobs to the nearest free bot, lets neighbouring bots
reserve nodes directly between themselves, and tells its robot which way to turn
at every node it reaches. Pure gRPC over one flat
network; no broker, no registry, no central server.

**Read [`PROTOCOL.md`](PROTOCOL.md) for the design.** This file is about
running and extending the code. **[`TODO.md`](TODO.md)** tracks which use
cases are covered and what is still open.
**[`docs/scenarios.md`](docs/scenarios.md)** is what the fleet does, situation
by situation, with the container test that proves each answer.
**[`docs/location.md`](docs/location.md)** is how a bot knows where it is, and
what silently stops working when it does not.
**[`docs/boundary.md`](docs/boundary.md)** is why there is one process per robot
and no central service — read it before proposing otherwise.

## Layout

```
bot.py               the bot process: run loop, roles, wire payloads, robot bridge
config.py            every env var and timing constant
bus/
  heartbeat.py       follower ↔ leader heartbeats, roster, redirect, departure
  leader_exchange.py leader ↔ leader heartbeats, bootstrap discovery, split-brain
  migration.py       Migrator state machine (bot side) + destination join
  jobs.py            job ledger, dispatcher, forwarding, observation
  policy.py          the "virtual network": who may call which service
  rpc.py             persistent gRPC channels
  admin.py           AdminService: inspect a bot / inject robot state (ADMIN_ENABLED only)
election/
  bully.py           bully election + abdication
  priority.py        priority formula (health, battery buckets, hysteresis)
  server.py          ElectionService handlers
planning/
  geometry.py        node kinds, headings, and which way is which
  graph.py           adjacency with headings, node kinds, hop distances
  topology.py        corridors, junctions, dead-end bays
  kinematics.py      how long a move takes and what it costs in charge
  cost.py            time + energy, weighted by battery state
  sipp.py            safe-interval search over (node, heading, interval)
  traffic.py         what other robots are doing, in three tiers
  routes.py          alternative routes, kept as diffs
  decide.py          Query in, Decision out -- proceed, wait, yield, reroute
  robot_service.py   the robot link: RobotNetwork.Session, one stream per robot
reservations/
  claims.py          what a claim is, and who gives way when two collide
  ledger.py          one bot's record of who holds what
  vicinity.py        who is close enough to be worth telling
  sender.py          the announce step, run from the bot's loop
  server.py          ReservationService handlers
peers/table.py       roster, other-region leaders, migration ledgers
warehouse/map.py     warehouse-layout.json → node→region, hop distances
proto/fleet.proto    the wire schema (+ generated fleet_pb2*.py)
up.py / down.py      local Docker orchestration
tests/               pytest: test_unit.py, test_protocol.py, test_jobs.py, containers/ (the scenario tier)
```

Every module starts with a *what / where / why / how* docstring.

## Quick start

```bash
uv sync                                   # deps (Python 3.12, grpcio, docker SDK, pytest)
uv run pytest tests -q                    # ~70 tests, ~15 s, no Docker needed

uv run up.py --bots 3 --region 14         # build image, launch 3 bots in parking
docker logs -f amr-region-14-bot-0        # watch them elect a leader
uv run up.py --bots 2 --region 2 --no-build
uv run down.py                            # remove all fleet containers + network
```

Run one bot natively:

```bash
BOT_ID=0 REGION_ID=14 OWN_ADDRESS=127.0.0.1:50051 PEER_LEADERS= uv run bot.py
```

## Configuration

All via environment (see `config.py` for defaults and why):

| Var | Meaning |
|---|---|
| `BOT_ID`, `REGION_ID` | identity; `BOT_ID` must be unique fleet-wide and < 100 |
| `OWN_ADDRESS` | `host:port` peers should dial for us |
| `PEER_LEADERS` | comma-separated addresses of other bots, for bootstrap discovery |
| `GRPC_PORT`, `GRPC_HOST` | listen port / bind interface |
| `T_HB` and friends | protocol timing (`PROTOCOL.md` §9) |
| `WAREHOUSE_MAP` | path to `warehouse-layout.json` (bind-mounted by `up.py`) |
| `JOB_MIN_BATTERY`, `JOB_MAX_HOPS`, `T_JOB_RETRY` | job dispatch |
| `NODE_TRAIL_LEN` | how many recent QR nodes a bot reports |
| `T_LEADER_TENURE` | leadership rotates to a free follower after this (0 = never) |
| `ROBOT_PATIENCE` | how long a robot waits at a junction before driving on by itself; every WAIT stays under it (`PROTOCOL.md` §16) |
| `K_COMMIT`, `PLAN_HORIZON`, `MAX_WAIT`, `MAX_EXPANSIONS` | how far ahead the planner commits, plans, waits and searches |
| `T_YIELD_THRESHOLD`, `YIELD_SEARCH_HOPS` | when a robot stands aside rather than waits, and how far it looks for somewhere to do it |
| `T_STALL` | commanded but not moving for this long is a stall; escalates replan → yield → NEEDS_ATTENTION |
| `ROUTE_ALTERNATES`, `HOPS_CACHE_SIZE`, `BATTERY_CRITICAL` | routes held per job, bounded distance cache, and where charge starts outweighing speed |
| `T_ANNOUNCE`, `RESERVATION_TTL`, `RESERVATION_REACH_HOPS` | reservations: how often a bot tells its neighbours what it holds, how long their claims stay believable, and how far a claim reaches (`PROTOCOL.md` §15) |
| `ADMIN_ENABLED` | serve `AdminService` (`GetState`, read-only); keep off in production |

## Plugging in a real robot

The bot talks to the robot through two small interfaces in `bot.py`:

- **`RobotSource.poll() -> RobotState | None`** — called every tick. Return
  the newest snapshot (shaped like `robot-to-network.schema.json`) or `None`.
- **`RobotSink.send(RobotCommand)`** — the bot's commands to the robot
  (shaped like `network-to-robot.schema.json`: `target_node_id` + a
  `mission` object).

`RobotState` contract the network layer relies on:

| Field | Expectation |
|---|---|
| `region_id`, `latest_node_id` | from the last QR scan; `region_id` drives migration |
| `state` | `IDLE`, `MOVING`, `FAULTED`, `COMMS_LONG`, … — `FAULTED`/`COMMS_LONG` make the bot ineligible to lead |
| `mission` | `PARK`, `CHARGE`, `HOLD`, `IDLE`, `CARGO` |
| `cargo_state` | when `mission == CARGO`: `PICKUP` (heading there), `EN_ROUTE` (cargo on board), `DROPOFF` (at destination). The bot infers **delivered** when the mission leaves `CARGO` after `DROPOFF` |
| `fault` | flatten the schema's fault object to a string: the error/warning type, e.g. `"MOTOR_ERROR"`, `"LOW_BATTERY:12"`; empty when healthy |

The defaults (`QueueRobotSource` / `QueueRobotSink`) are queues — push
states in, pop commands out — which is also how the tests simulate a robot.

`Bot.control_plane` is a callable that receives `NEEDS_ATTENTION` job
escalations (cargo stuck on a broken bot). Replace it with your client; the
default logs at ERROR and keeps events in `Bot.alerts`.

## Regenerating the proto

```bash
# -I. so the generated stubs import `from proto import ...` rather than a
# bare `fleet_pb2`, which only resolves if proto/ happens to be on the path.
# controlplane.proto is the control plane's, vendored here because the Docker
# build context is this directory alone. Copy it across before regenerating;
# tests/test_control_plane.py fails if the two differ.
cp ../../spore-control-plane/proto/controlplane.proto proto/
uv run python -m grpc_tools.protoc -I. --python_out=. --grpc_python_out=. \
    proto/fleet.proto proto/robot.proto proto/controlplane.proto
sed -i 's/^import fleet_pb2/from proto import fleet_pb2/' proto/fleet_pb2_grpc.py
```

(The `sed` fixes the generated import to be package-relative.)

## Testing notes

- Tests use `127.0.0.1` and ports 21xxx on purpose — `localhost` resolves to
  `::1` first and ports in the ephemeral range (32768–60999) can be stolen
  by an earlier test's outgoing connection. gRPC reports a failed bind by
  returning `0`, not raising; both bind sites check for that.
- `tests/conftest.py` shortens `T_MIGRATION_TIMEOUT` so failure paths run in
  seconds. The `fleet` fixture tears down every thread and server.
- `up.py` publishes each container's gRPC port on an ephemeral host port, and
  the Docker tests dial that rather than the container IP. Docker Desktop on
  macOS does not route to container IPs at all, so without it the whole Docker
  tier is unrunnable there. Bots still reach each other by container name on the
  bridge and never use the published port.
- `tests/containers/` runs the chaos scenarios (kill, pause, partition,
  migration, jobs, reservations) on real containers and a private bridge network
  per test.
  It needs a Docker daemon (skipped otherwise), builds the image once per
  session (`AMR_DOCKER_NO_BUILD=1` to reuse), takes ~45 s, and drives bots
  through `AdminService` — enabled by `up.py`'s `ADMIN_ENABLED=1`, off by
  default. Run only the fast tiers with `-m "not docker"`.
