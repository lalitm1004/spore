# Webots AMR simulator — agent context

A containerised Webots simulation of line-following warehouse robots, under
`spore-amr/webots/`. It is the **robot half** of the system: perception, motion
control, and the physical layer. The real network layer — task allocation,
multi-robot coordination — is somebody else's code, and is represented here by
a stand-in that names a random neighbouring node.

Ten robots run on a window of the real `warehouse.json`, parked on charging
bays and released one at a time, stopping at every QR marker to ask their own
network-layer process which way to turn.

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
   │ robot/companion.py  "the Pi"     │   holds the map, reports which node
   │  sets speed, never wheel values  │   it is at, turns the answer into a
   └──────────────┬───────────────────┘   bearing
                  │ newline JSON over a unix socket  (robot/navigator.py)
                  │   up:   latest_node_id   "I am at node 116"
                  │   down: target_node_id   "go to node 70"
   ┌──────────────┴───────────────────┐
   │ robot/netlayer.py                │   one process per robot, not one
   │  RandomRouter: holds the map,    │   shared service
   │  picks a neighbour               │
   └──────────────────────────────────┘
```

**Left and right never cross the wire.** `RobotToNetwork` carries
`latest_node_id`; `NetworkToRobot` carries `target_node_id`. Both schemas set
`additionalProperties: false`, so a `turn` field or a menu of available turns
is not an extension — it is a message the real network layer must reject. The
network layer routes and therefore holds the map; the robot holds the map too
and works out for itself that node 70 is a right turn, which it can do exactly,
because lanes are straight and it knows where both nodes are.

**The firmware has no network code.** Grep it: the only matches for
`socket|network` are comments. It reports "I am at node 7" and receives "turn
to 90 degrees"; it never loads a map and never opens a socket. If the link
stalls it keeps following the line on its last setpoint, and the junction wait
is bounded by `junction_timeout_s`.

**Only `robot/main.py` and `robot/supervisor.py` import the Webots API.**
Everything else is pure and host-testable, which is why 196 tests run in about
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

4. **The wire format is the shared schema, not a superset.** The robot once
   sent the legal turns and was answered with `left`/`straight`/`right`. Both
   fields are absent from `shared/schemas/`, and putting the route choice on
   the robot's side of the wire had a cost beyond tidiness: the turn menu was
   filtered to ±45° of left, straight and right, so the way back out of a
   degree-1 charging bay — 180° — matched nothing, the menu came back empty,
   and a robot routed into a bay sat in it for the rest of the run. Node-based
   routing has no such case: a bay's one neighbour is its junction.

5. **The QR payload follows the shared schema exactly.** All 83 payloads
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

**`running` in `robot/main.py` means "the run is over", not "the wheels are
stopped".** The bottom of the loop reads `not running` as the companion having
called time: it closes the telemetry log and prints the run summary. Holding a
robot at the start by clearing that same flag therefore ended its run on tick
one — summary at zero steps — and the first row written after release raised
`ValueError: I/O operation on closed file`. The sequential start carries its
own `held` flag for exactly this reason. Two states that both stop the motors
are not the same state.

**A stale `out/*.status.json` is scored as this run's first marker.** It is how
a robot tells the supervisor what it last read, and the supervisor computes
localisation error the moment a new `(node_id, t)` appears. A file left behind
by the previous run is a new one to a fresh supervisor, so it scores a node the
robot read minutes ago against a robot standing at its charging bay: measured,
18 m of "localisation error" and 90 degrees of "heading error" that never
happened. It shows up as exactly one bad sample per robot, always the first,
with the medians healthy — and it is indistinguishable at a glance from the
heading-frame bug below, which is real and looks identical. `./fleet.sh up`
clears `out/` for this reason. If a score has one wild first read per robot and
sane medians, suspect the artefacts before the robots.

**A `TURN` is an absolute bearing, so the firmware's heading frame has to be
right.** `robot/turn.py` says its estimate "was corrected by the marker the
robot is standing on" — it never was. The marker supplies position only, and
`Odometry` started at theta=0 however the robot was actually parked. On the
warehouse window every charging bay faces ±90°, so all ten robots executed
every turn 90° out: onto a perpendicular lane, or back the way they came.
It reads from outside as bad line-following, because what you see is a robot
leaving the node wrong and then hunting for a line that is not there.
`odometry.start_theta` seeds the frame from the same pose the world file uses,
and the companion now sends the exact arrival heading with each `TURN`.
Score any change here with `tools/spike_truth.py`; a heading error above a few
degrees means turns are landing on lanes nobody chose.

**A turn must happen about the node, and the read does not happen there.**
The colour trigger fires 175 mm before the node centre and the code decodes
roughly 80 mm later, so when the robot stops to be routed its own origin is
still about 95 mm short of the junction. Rotating there rotates about a point
*beside* the lane it is turning onto: 95 mm off, against a 20 mm line under an
array spanning ±40 mm, so the line is not merely hard to find, it is not under
the robot at all. What follows looks like bad line-following — the robot drives
parallel to the lane it wanted, spends the 0.15 m turn-recovery budget, then
the lost-line search spins it at full lock onto whichever lane it happens to
meet. Watched from above it drives *around* the node. `MarkerCrossing.lever_arm`
already returns exactly the distance still to travel (it is what makes the
position fix accurate to a millimetre); the firmware now rolls that last stretch
on blind before starting the turn. A turn commanded but not yet begun is
`pending_bearing`.

**A one-tick line loss is noise, and the throttle must be able to go back up.**
`EVT LINE_LOST` was edge-triggered with no debounce, and the companion answers
every one by multiplying speed by 0.6 with nothing to raise it again. Three
ticks after a crossing ends the array is not always back over the lane, so a
single 16 ms dropout cost a permanent speed step: measured, robots sat at the
1.5 rad/s floor — a quarter of cruise — for the rest of the run. Hence
`lost_line_debounce_s` and `speed_recover_after_s`. A ratchet in a control
loop is a bug even when every individual step is correct.

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

`./fleet.sh` is the front door and owns the flags that are easy to get wrong —
it regenerates when `fleet.yaml` has changed, clears the previous run's
artefacts, always passes `--remove-orphans`, and anchors itself to this
directory so the `./:/project` mount cannot pick up the wrong tree.

```bash
./fleet.sh up          # regenerate if needed, clear out/, bring the fleet up
./fleet.sh score       # score the run against the supervisor's ground truth
./fleet.sh goals       # what the network layer has told each robot to do
./fleet.sh status      # what is running, and how far through the run
./fleet.sh logs bot_01 # follow one robot; no argument follows them all
./fleet.sh down        # tear it down
#   viewer: http://localhost:1234/index.html
```

`MODE`, `RENDERING` and `MISSION_DURATION` override its defaults. The
underlying commands, when you want them by hand:

```bash
uv run python -m tools.gen_fleet          # after ANY fleet.yaml edit
MODE=fast RENDERING=off MISSION_DURATION=900 \
  docker compose -f compose.yml -f compose.fleet.yml up -d
docker compose -f compose.yml -f compose.fleet.yml down --remove-orphans
```

`--remove-orphans` matters: changing the robot count leaves containers running
against a world that no longer has them, and a synchronized robot with no
controller freezes the simulation.

`MODE=fast` runs the world as fast as the host allows; `realtime` is the
default and paces it to the wall clock. Webots has no numeric multiplier, so
`fast` is the whole switch — and at ten robots it is worth about **1.12x**,
measured on 16 cores with `RENDERING=off`. The simulator process sits at
270-460% CPU while every controller stays under 30% of one core, so the
simulator is the constraint and `defaults.resources.cpus` buys headroom against
a QR-decode stall rather than speed. Real speedup means fewer robots or fewer
enabled sensors. Simulation time and `MISSION_DURATION` are unaffected either
way.

`docker/robot-entrypoint.sh` is `COPY`'d into the image, so editing it changes
nothing until `docker build -f docker/controller.Dockerfile -t
sih2026/controller:dev .`. Python is volume-mounted and needs no rebuild.

```bash
uv run pytest -q                                    # 196 tests, no Webots
docker compose ... logs bot_01 | grep "node "       # marker reads
docker compose ... logs | grep "netlayer: bot"      # routing decisions
docker compose ... logs supervisor | grep label:    # localisation error
docker compose ... logs | uv run python -m tools.spike_truth   # score the run
```

`tools/spike_*.py` are single-purpose diagnostics that each found a real bug —
device inventory, camera frames under `--no-rendering`, what the ground sensors
actually see, where marker solids really are, turn accuracy against ground
truth. Reach for them before theorising.

`tools/spike_truth.py` is the one to run after any change to steering, turning
or odometry: it folds a run's supervisor output into fix error, heading error,
turns, timeouts and halts per robot. Heading is the number that matters — the
turn still completes when it is wrong, just onto the wrong lane, so nothing
else in the system reports it.

---

## State of the work

Measured on the current warehouse window, ten robots:

| | |
|---|---|
| track | 32 x 16 m window of the real layout, 83 nodes, 10 charging bays |
| markers | 83, all validating against the shared QR schema |
| line tracking | 0.025-0.21 mm mean per robot |
| localisation fix error | 5 mm median, 12 mm worst |
| heading error against truth | 1.1 degrees median, 1.8 degrees worst |
| turns | 44 of 44 executed, no timeouts, no halts |
| line lost | 1.2 s of a 420 s run, fleet mean (0.3%) |
| obstacle reflex | trips at 178 mm, retreats to the previous marker |

Measured with `tools/spike_truth.py` over one 420 s run of ten robots. Before
the heading frame was seeded and turns were made about the node rather than
95 mm short of it, the same run scored 90 degrees of heading error, fix errors
of 2-4.5 m on first reads, turns landing on lanes nobody chose, and two robots
with the line lost for 78% and 90% of the run.

**Known rough edges:** ten robots on random routes jam, which is the honest
result rather than a bug — see `docs/demo.md`. Charging bays are degree-1
spurs that pair onto one junction; the sequential start keeps a pair from
meeting there, but does nothing for the contention everywhere else.

**Not built:** the real network layer (this has a random stand-in, which now
holds the map and picks a neighbour, so swapping it out means changing what
listens on the socket and nothing else); `proto/firmware.proto` is a drafted
gRPC schema no code references.

Further reading: `docs/architecture.md` for the module map and the reasoning,
`docs/demo.md` for running it, `docs/markers.md` for the marker subsystem.
