# Webots AMR simulator — agent context

A containerised Webots simulation of line-following warehouse robots, under
`spore-amr/webots/`. It is the **robot half** of the system: perception, motion
control, and the physical layer. The real network layer — task allocation,
multi-robot coordination — is somebody else's code, and is represented here by
a stand-in that hands out random turns.

Ten robots run on a window of the real `warehouse.json`, spawning at charging
bays, stopping at every QR marker to ask their own network-layer process which
way to turn.

Read this before changing anything. Most of what follows is a decision that
cost measurement to reach, and several are things that look wrong until you
know why they are that way.

---

## Where the boundaries are

```
spore-warehouse-layout/output/
  warehouse.json ─┐   the real 120x70 m layout, 881 nodes
  warehouse_map.svg   the drawing of it
                  │
                  ▼  a 32 x 16 m window, generated at build time
     config/warehouse.json   83 real nodes, real ids, 10 charging bays
     textures/track-<hash>.png   the floor
     textures/markers/*.png      one QR tile per node
                  │
   ┌──────────────┴───────────────────┐   shared schemas, do not diverge:
   │ robot/main.py    "the ESP32"     │   spore-amr/shared/schemas/
   │  IR arrays, motors, cameras      │     qr-code.schema.json
   │  62.5 Hz control loop            │     robot-to-network.schema.json
   └──────────────┬───────────────────┘     network-to-robot.schema.json
                  │ newline ASCII over a socat pty  (robot/protocol.py)
   ┌──────────────┴───────────────────┐
   │ robot/companion.py  "the Pi"     │   holds the map, resolves which turns
   │  sets speed, never wheel values  │   exist, asks the network layer
   └──────────────┬───────────────────┘
                  │ newline JSON over a unix socket  (robot/navigator.py)
   ┌──────────────┴───────────────────┐
   │ robot/netlayer.py                │   one process per robot, not one
   │  RandomRouter: a legal turn      │   shared service
   └──────────────────────────────────┘
```

**The firmware has no network code.** Grep it: the only matches for
`socket|network` are comments. It reports "I am at node 7" and receives "turn
to 90 degrees"; it never loads a map and never opens a socket. If the link
stalls it keeps following the line on its last setpoint, and the junction wait
is bounded by `junction_timeout_s`.

**Only `robot/main.py` and `robot/supervisor.py` import the Webots API.**
Everything else is pure and host-testable, which is why 167 tests run in about
a second with no simulator. A new module that imports `controller` is a module
nobody can test.

---

## Invariants — breaking these breaks the demo

1. **No robot gets a privileged sensor.** Ground truth lives in
   `robot/supervisor.py`, a separate Webots process. No GPS, no compass on the
   robot, deliberately: a sensor that exists is a sensor the control code
   eventually starts using, and then the localisation claim is worthless.

2. **The lidar is a reflex, never a planning input.** `robot/obstacle.py` may
   slow, stop and reverse. Nothing it sees may reach a router. The project's
   argument is that collision avoidance is a property of the protocol, not of
   sensing; the lidar covers the one case the protocol cannot — something on
   the floor that never announced itself because it is not a robot.

3. **`fleet.yaml` is the single source of truth.** The floor texture, every
   marker texture, `config/warehouse.json`, `worlds/track.wbt`,
   `compose.fleet.yml` and every per-robot config are generated from it by
   `tools/gen_fleet.py`. Edit the manifest, never the outputs.

4. **The QR payload follows the shared schema exactly.** All 83 payloads
   validate against `qr-code.schema.json`. It carries no out-edges and no lane
   bearing and does not need to: both come from `config/warehouse.json`, which
   every robot holds. Adding fields costs QR modules, and modules are camera
   pixels.

---

## Traps

Things that have already gone wrong here. Each cost real time to find.

**A Webots `Solid` with a `boundingObject` and no `Physics` does not
collide.** It renders, it looks right, and it is ignored. The robot balanced on
two wheels for weeks. Collision geometry for non-physics parts belongs in the
parent's `boundingObject Group`.

**`--no-rendering` keeps camera sensors.** Verified with the flag confirmed in
the live command line. Sensor rendering is a separate offscreen pass. This
takes simulator CPU from 907% to 5.65% — always run `RENDERING=off`, including
for the browser view, because w3d streaming renders client-side anyway.

**The viewer caches the floor by URL.** The texture is content-addressed
(`track-<hash>.png`) for exactly this reason. A regenerated floor kept arriving
in the browser as the previous one — world right, file on disk right, picture
stale — and it looked for two rounds like the generator was broken. If the
floor ever looks wrong, check what the browser is *fetching* before checking
what the generator wrote.

**A camera raises rather than returning empty on the step it is enabled.**
`wb_camera_get_image` gives NULL and the Python binding turns that into
`ValueError: NULL pointer access`, so `if not image:` never runs. Skip the
enabling step.

**Webots' `Camera` and `DistanceSensor` do not share a native axis.** The
`rotation 0 1 0 1.5708` that points a DistanceSensor down does *not* point a
Camera down. Verify by saving a frame, not by reading the docs.

**A synchronized robot whose controller exits freezes Webots for every other
robot.** The firmware halts and keeps stepping instead of leaving its loop. One
robot losing the line must not stop the fleet.

**The rendered colour is not the authored colour.** A `(255,122,0)` tile
renders as `(242,173,56)` under this world's lighting. The border classifier
compares chromaticity, not RGB, so lighting changes need no retuning — but the
threshold (0.30) was measured. Re-measure with `tools/spike_drive.py` after any
lighting change.

**`x or default` is wrong when `0.0` is a legal value.** Written twice here, in
`robot/marker.py` and `robot/obstacle.py`: a crossing beginning at distance 0.0
was treated as no crossing. Use `is not None`.

**Reversing off a forward-facing sensor is non-minimum-phase.** Correcting
toward the error rotates the sensor further off the line. Measured: the line
was absent for 91% of a retreat. Hence the second IR array at x=-70 mm, and its
inverted steering sign.

**`rsvg-convert` parses a bare negative offset as a flag.** `--left -2000`
fails with exit 2; `--left=-2000` works. It failed silently into the fallback
renderer for a while, which looked like a completely different bug.

**Fitting a coordinate mapping on ranges does not tell you the orientation.**
Min-to-min and max-to-max match whether or not an axis is flipped. The
warehouse SVG's y axis *is* inverted relative to world y, and the landmark that
settles it is that the CHARGING label sits at svg y=723 while its nodes are at
world y=800 cm. Correct mapping: `x_px = 0.11*x_cm + 90`,
`y_px = 860 - 0.11*y_cm`.

---

## The optics budget

The tightest constraint in the project. Change one of these and check the
others still hold — `tests/test_marker.py::test_survives_the_camera_resolution`
is the gate.

```
camera 512 px over a 92.7 mm footprint       ->  0.181 mm per pixel
QR code 60 mm, 49 modules (JSON, ec=L)       ->  1.22 mm per module
                                             ->  6.8 px per module  (floor is 4)
```

Error correction is **L, not M**: L needs 49 modules where M needs 57, and
OpenCV's decoder measurably prefers the smaller code — 5 of 5 node payloads at
L against 4 of 5 at M on identical renders. The recovery given up is bought
back in time, since a marker is in view for roughly 22 frames.

Marker tiles are separate textured planes, never pixels in the floor. Their
texture is 1024x1024 by construction (100 mm at 10.24 px/mm): Webots silently
rescales a non-power-of-two texture, and rescaling a QR resamples the very
module edges the decoder reads.

---

## Geometry

Millimetres, robot frame. The origin is at wheel-axle height, so the floor is
20 mm below it.

| device | x (fwd) | y (left +) | above floor |
|---|---|---|---|
| `color` (1x1 Camera) | +125 | 0 | 15 |
| `qr` (512x512 Camera) | +95 | 0 | 80 (mast) |
| `ir0..2` forward array | +70 | +20 / 0 / -20 | 15 |
| `lidar` | +62 | 0 | 32 |
| ball transfer, front | +45 | 0 | contact |
| wheels | 0 | ±45 | — |
| ball transfer, rear | -45 | 0 | contact |
| `irb0..2` rear array | -70 | +20 / 0 / -20 | 15 |

Both optics sit on a forward boom past the chassis front edge at +60. A camera
at the chassis edge looking straight down sees its own body fill the rear half
of every frame.

`track_width` is **0.0994 m**, not the nominal 0.090. Calibrated against ground
truth: contact behaves like the wheels' outer edge, not their centre. The
nominal value cost 27 degrees of heading drift per lap; the calibrated one
gives about 0.1 degrees per marker segment.

---

## Crossing a marker

A 100 mm tile on a 20 mm lane covers the line, so the robot crosses it blind.
Distances from the moment the colour sensor first sees orange:

```
d = 0 mm       colour sensor meets the tile's near edge
d = 55 mm      IR array enters the tile -- the line is gone
d = 59-101 mm  code inside the camera's footprint, ~22 frames to decode
d = 155 mm     IR array clears the tile -- the line is back
```

No alignment manoeuvre and no stop for the read itself. The robot arrives
square because it followed the line to get there.

While blind the firmware holds the **averaged** steering, not the last
instantaneous value. The PD output oscillates around what a curve needs, so a
sample near zero sends the robot straight for 250 mm.

**A crossing is not a line loss.** The whole sequence — `OVER` and
`RECOVERING` — is suppressed from `EVT LINE_LOST`, exempt from the lost-line
timeout, and exempt from the lost-line search. Each was a separate bug: the
companion throttled 6.0 -> 3.6 -> 2.16 -> 1.5 rad/s reading every marker as
"going too fast"; the timeout stopped the robot on a working track; and the
search — a hard turn at the steering limit — spun it 180 degrees.

**A crossing that cannot finish must be ended.** `RECOVERING` only exits when
the line is found. If it never is, it suppresses the lost-line timeout for
ever, the robot falls into the full-lock search, stops travelling, and so never
spends the distance budget that would end the crossing. One robot span on the
spot for 96 seconds. `MarkerCrossing.reset()` exists for this and for turns.

---

## Running it

Always from the repo root — the volume mount is `./:/project`, so `./` is your
shell's directory, not the compose file's.

```bash
uv run python -m tools.gen_fleet          # after ANY fleet.yaml edit
RENDERING=off MISSION_DURATION=900 \
  docker compose -f compose.yml -f compose.fleet.yml up -d
#   viewer: http://localhost:1234/index.html
docker compose -f compose.yml -f compose.fleet.yml down --remove-orphans
```

`--remove-orphans` matters: changing the robot count leaves containers running
against a world that no longer has them, and a synchronized robot with no
controller freezes the simulation.

`docker/robot-entrypoint.sh` is `COPY`'d into the image, so editing it changes
nothing until `docker build -f docker/controller.Dockerfile -t
sih2026/controller:dev .`. Python is volume-mounted and needs no rebuild.

```bash
uv run pytest -q                                    # 167 tests, no Webots
docker compose ... logs bot_01 | grep "node "       # marker reads
docker compose ... logs | grep "netlayer: node"     # routing decisions
docker compose ... logs supervisor | grep label:    # localisation error
```

`tools/spike_*.py` are single-purpose diagnostics that each found a real bug —
device inventory, camera frames under `--no-rendering`, what the ground sensors
actually see, where marker solids really are, turn accuracy against ground
truth. Reach for them before theorising.

---

## State of the work

Measured on the current warehouse window, ten robots:

| | |
|---|---|
| track | 32 x 16 m window of the real layout, 83 nodes, 10 charging bays |
| markers | 83, all validating against the shared QR schema |
| line tracking | 0.79 mm mean |
| localisation fix error | 6-11 mm |
| wheel heading drift | under 1.7 degrees per lap |
| obstacle reflex | trips at 178 mm, retreats to the previous marker |

**Known rough edges:** ten robots on random routes jam, which is the honest
result rather than a bug — see `docs/demo.md`. Charging bays are degree-1
spurs that pair onto one junction, so robots leaving them contend immediately.

**Not built:** the real network layer (this has a random stand-in);
`robot/turn.py`'s accuracy has never been measured against ground truth;
`proto/firmware.proto` is a drafted gRPC schema no code references.

Further reading: `docs/architecture.md` for the module map and the reasoning,
`docs/demo.md` for running it, `docs/markers.md` for the marker subsystem.
