# Spore AMR — agent context

Read this before touching anything. It is short on purpose; each subproject has
its own deeper document and this one exists to stop you making the four or five
mistakes that have already cost this project real time.

---

## What this is

A warehouse robot fleet, simulated end to end. Four tiers, each a separate
project with its own contract:

```
spore-control-plane/          an operator places a cargo order: node A -> node B
        │  gRPC  controlplane.ControlPlaneService/DispatchOrder
        │        (the control plane knows NO regions and NO leaders)
        ▼
spore-amr/network-layer/      one bot process PER ROBOT. elects region leaders,
        │                     keeps a roster, dispatches jobs, reserves nodes
        │                     bot-to-bot, and answers "which way at this node"
        │  gRPC  spore.network.v1.RobotNetwork/Session
        │        up:   latest_node_id + available exits
        │        down: target_node_id  (never a turn)
        ▼
spore-amr/webots/robot/companion.py     "the Pi": holds the map, turns a named
        │                               node into a bearing
        │  newline ASCII over a socat pty
        ▼
spore-amr/webots/robot/main.py          "the ESP32": IR arrays, motors, camera,
                                        62.5 Hz control loop. No network code.
```

`spore-amr/shared/schemas/` is the contract between tiers and is authoritative.
`spore-amr/temp-network-robot-glue-code-fix-this-pls-ojasw/` is **superseded** —
its architecture was rejected in favour of one bot per robot. Do not build on
it; read `spore-amr/network-layer/docs/boundary.md` for why.

| project | read this |
|---|---|
| `spore-amr/webots/` | `CLAUDE.md`, then `docs/architecture.md` |
| `spore-amr/network-layer/` | `PROTOCOL.md`, then `docs/boundary.md` |
| `spore-control-plane/` | `DOCUMENTATION.md` |

---

## Rules that are not negotiable

1. **Left and right never cross the network wire.** `RobotToNetwork` carries
   `latest_node_id`; `NetworkToRobot` carries `target_node_id`. The network
   names a *place*; the robot holds the map too and works out the bearing
   itself. A `turn` field is not an extension, it is a different system.

2. **No robot gets a privileged sensor.** Ground truth lives only in
   `robot/supervisor.py`, a separate process. No GPS, no compass. A sensor that
   exists is one the control code eventually uses, and then the localisation
   claim is worthless.

3. **The lidar is a reflex, never a planning input.** It may slow, stop and
   reverse the robot. Nothing it sees may reach a router.

4. **`fleet.yaml` is the single source of truth for the simulator.** The world,
   every texture, every per-robot config and `compose.fleet.yml` are generated
   from it by `tools/gen_fleet.py`. Edit the manifest, never the outputs.

5. **The schemas are authoritative, and the proto is a second rendering of
   them.** `spore-amr/network-layer/proto/robot.proto` adds exactly seven fields
   the schemas do not have, and `tests/test_proto_contract.py` asserts *exactly*
   those and no others. If you add a field, that test is the conversation.

---

## Running it

Everything below is from `spore-amr/webots/`.

```bash
./fleet.sh up            # regenerate if fleet.yaml changed, clear out/, start
./fleet.sh orders 8      # seed fake demand: 8 orders, pick station -> yard
./fleet.sh robots        # per robot: distance, state, is it actually moving
./fleet.sh score         # against the supervisor's ground truth -- THE metric
./fleet.sh fleet         # leaders, jobs, claims: the coordination layer
./fleet.sh replay3d      # a 3D replay you can orbit; open out/replay3d.html
./fleet.sh down
```

**A fleet with no orders looks broken and is not.** With nothing to do, every
robot is answered "wait" at every junction and sits still. `./fleet.sh orders N`
is the fake demand a real order system would supply. Seed it, or you are
debugging a fleet that has correctly decided to do nothing.

**Cut the map down before you iterate.** The full warehouse is 881 nodes and
924 MB of marker texture; it runs, but slowly. A window keeps the job cycle
intact if it contains all three kinds:

```bash
uv run python -m tools.gen_fleet --window 700,300,30,60   # 263 nodes, ~0.75x realtime
```

Charging bays (`CH`) sit at the bottom of the map, pick stations (`PK`) at the
top, yard nodes (`YI`) in the middle — so a window can cut **width** but not
height, or there is no job to run.

```bash
uv run pytest -q         # 240 tests, no Webots, under a second
```

---

## What actually limits you

Measured, not guessed:

```
webots-sim   ~155% CPU        <- everything waits on this
each robot     ~1.2% CPU
total          ~280% of 1600% available on a 16-core box
```

**The simulator is one process that plateaus around 1.5 cores.** You are not
CPU-bound, RAM-bound or GPU-bound. More cores will not help; a faster
single-core clock helps roughly linearly. To go faster you either make each
tick cheaper (the 512×512 QR camera is ~707× everything else the sim renders
per robot) or run several smaller worlds in parallel.

Optimising the robot controllers is wasted effort — they are at 1.2%.

---

## Traps that have already cost time

Each of these read as something else entirely, which is why they are here.

**A mounted dependency with no `PYTHONPATH` is an inert mount.** `robot/uplink.py`
does `from proto import robot_pb2_grpc`, and `/project/proto` holds only
`firmware.proto` with no `__init__.py` — so Python bound `proto` to that as a
namespace package and the import failed. `connect()` turned the failure into
`False`, `ask()` into `None`, and the companion printed "no answer from the
network layer" at every node. The whole fleet then ran on its junction timeout,
which drives *straight on*; charging bays come in facing pairs on a shared
junction, so two robots drove into the opposite bay, off the end of a
degree-1 spur, and halted on a lost line. **It read as bad driving.** Nothing
said "misconfigured".

**Speed is coupled to turn accuracy, and it is not the PID.** Leaving a turn the
robot carries a few degrees of heading error, and lateral drift is `v·sin(e)`
against a 20 mm line: 1.19 s to recover at 0.12 m/s, 0.40 s at 0.36 m/s. Raising
cruise to 18 rad/s with no ramp cost two robots of eight. The fix is
`accel_rad_s2` — leave every turn from rest — not a lower cruise. Steering
saturated on 0.9% of ticks, so the loop had authority to spare the whole time.

**Off the ground plane reads as a perfect line.** Every IR sensor returns
`black_ref`, so the weighted mean is exactly 0.0 at maximum confidence: dead
centre, total certainty. `lost` is built for *too little* line and has no notion
of too much. Any sensor reading that cannot fail is not a sensor reading.

**Ground truth is the only thing that catches a lie.** A robot beached on its
belly with the wheels spinning reported 65 m of travel; the supervisor had it
0.000 m from where it started. Its own telemetry was internally consistent and
completely wrong. Score with `./fleet.sh score` before believing anything.

**Check `git status` before you commit.** `config/warehouse.json`,
`worlds/track.wbt`, `textures/` and `config/bot_*.yaml` are generated and *are*
committed at full-warehouse scale. If you generated a window to iterate, those
files in your tree are the small map — committing them silently replaces the
warehouse. Commit source, regenerate the full map deliberately.

**Stale `config/bot_*.yaml` outlive a shrinking fleet.** Going from 10 robots to
8 leaves `bot_09.yaml` and `bot_10.yaml`, and `fleet.sh` derives container names
from that glob. Delete them.

---

## How to work here

- **Measure before you fix.** Nearly every wrong turn in this project's history
  was a plausible theory acted on without evidence. `./fleet.sh score`,
  `tools/spike_*.py` and the supervisor exist for this.
- **One change at a time.** Two changes and a failure tells you nothing.
- **If three fixes fail, the architecture is the problem, not the fourth fix.**
- **Write down what you measured, next to the number it produced.** Most
  constants in `fleet.yaml` carry the measurement that chose them. Keep that up
  — a bare number invites someone to "clean it up" later.
