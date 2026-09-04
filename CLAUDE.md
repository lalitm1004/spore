# Webots AMR simulator — agent context

A containerised Webots simulation of line-following warehouse robots, sitting
under `spore-amr/webots/`. It is the **robot half** of the system: perception,
motion control, and the physical layer. The network layer — task allocation,
routing, multi-robot coordination — is somebody else's code and is only
represented here by a stand-in.

Read this before changing anything. Most of what follows is a decision that
cost measurement to reach, and several are things that look wrong until you
know why they are that way.

---

## Where the boundaries are

```
warehouse.json  ──►  every robot holds the map (881 nodes, 952 edges, cm)
                      ▲
                      │ node id
   floor QR marker ───┘                    shared schema, do not diverge:
                      │                    spore-amr/shared/schemas/
   ┌──────────────────┴───────────────┐      qr-code.schema.json
   │ robot/main.py    "the ESP32"     │      robot-to-network.schema.json
   │  IR arrays, motors, cameras      │      network-to-robot.schema.json
   │  62.5 Hz control loop            │
   └──────────────┬───────────────────┘
                  │ newline ASCII over a socat pty  (robot/protocol.py)
   ┌──────────────┴───────────────────┐
   │ robot/companion.py  "the Pi"     │
   │  sets speed, never wheel values  │
   └──────────────────────────────────┘
```

**The firmware never depends on the companion.** If the link stalls or the
companion dies, the robot keeps following the line on its last setpoint. Serial
writes drop on `EAGAIN` rather than blocking — telemetry is expendable, control
timing is not. Do not add a blocking call to that path.

**Only `robot/main.py` and `robot/supervisor.py` import the Webots API.**
Everything else is pure and host-testable, which is why 136 tests run in under
a second with no simulator. Keep it that way: a new module that imports
`controller` is a module nobody can test.

---

## Invariants — breaking these breaks the demo

1. **No robot gets a privileged sensor.** Ground truth lives in
   `robot/supervisor.py`, which is a separate Webots process. There is no GPS
   or compass on the robot, deliberately: a sensor that exists is a sensor the
   control code eventually starts using, and then the localisation claim is
   worthless.

2. **The lidar is a reflex, never a planning input.** `robot/obstacle.py` may
   stop and reverse. Nothing it sees may reach a router. The project's argument
   is that collision avoidance is a property of the protocol, not of sensing;
   the lidar exists only for the case the protocol cannot cover — something on
   the floor that never announced itself because it is not a robot.

3. **`fleet.yaml` is the single source of truth.** `worlds/track.wbt`,
   `compose.fleet.yml`, `config/*.yaml` and `textures/` are all generated from
   it by `tools/gen_fleet.py`. Edit the manifest, never the outputs. They are
   tracked in git on purpose — the same convention `warehouse.json` follows.

4. **The QR payload follows the shared schema exactly.** It carries no
   out-edges and no lane bearing, and does not need to: both are derivable from
   `warehouse.json`, which every robot holds. Adding fields costs QR modules,
   and modules are camera pixels (see the optics budget below).

---

## Traps

Things that have already gone wrong here. Each cost real time to find.

**A Webots `Solid` with a `boundingObject` and no `Physics` does not
collide.** It renders, it looks right, and it is ignored. The robot balanced on
two wheels for weeks this way. Collision geometry for non-physics parts belongs
in the parent's `boundingObject Group`.

**`--no-rendering` keeps camera sensors.** Verified with the flag confirmed in
the live command line. Sensor rendering is a separate offscreen pass. This
takes simulator CPU from 907% to 5.65% and is what makes a fleet affordable —
always run with `RENDERING=off`, including for the browser view, because w3d
streaming renders client-side anyway.

**A camera raises rather than returning empty on the step it is enabled.**
`wb_camera_get_image` gives NULL and the Python binding turns that into
`ValueError: NULL pointer access`, so `if not image:` never runs. Skip the
enabling step.

**Webots' `Camera` and `DistanceSensor` do not share a native axis.** The
`rotation 0 1 0 1.5708` that points a DistanceSensor down does *not* point a
Camera down. Verify by saving a frame, not by reading the docs.

**The rendered colour is not the authored colour.** A `(255,122,0)` tile
renders as `(242,173,56)` under this world's lighting. The border classifier
compares chromaticity, not RGB, so lighting changes do not require retuning —
but the threshold (0.30) was measured, not guessed. Re-measure with
`tools/spike_drive.py` after any lighting change.

**`x or default` is wrong when `0.0` is a legal value.** This bug was written
twice, in `robot/marker.py` and `robot/obstacle.py`: a crossing that begins at
distance 0.0 was treated as "no crossing". Use `is not None`.

**Reversing off a forward-facing sensor is non-minimum-phase.** Correcting
toward the error rotates the sensor further off the line. Measured: the line
was absent for 91% of a retreat. This is why there is a second IR array at
x=−70 mm, and why its steering sign is inverted against the forward loop.

---

## The optics budget

The tightest constraint in the project. Change any one of these and check the
others still work — `tests/test_marker.py::test_survives_the_camera_resolution`
is the gate.

```
camera 512 px over a 92.7 mm footprint       →  0.181 mm per pixel
QR code 60 mm, 49 modules (JSON, ec=L)       →  1.22 mm per module
                                             →  6.8 px per module   (floor is 4)
```

Error correction is **L, not M**: L needs 49 modules where M needs 57, and
OpenCV's decoder measurably prefers the smaller code — 5 of 5 node payloads
decoded at L against 4 of 5 at M on identical renders. The recovery given up is
bought back in time, since a marker is in view for roughly 22 frames.

Marker tiles are separate textured planes, never pixels in the track texture.
At the track's 512 px/m a QR module would be one texel and Webots would mipmap
the finder patterns into mush.

---

## Geometry

Millimetres, robot frame. Origin is at wheel-axle height, so the floor is 20 mm
below it.

| device | x (fwd) | y (left +) | above floor |
|---|---|---|---|
| `color` (1×1 Camera) | +125 | 0 | 15 |
| `qr` (512×512 Camera) | +95 | 0 | 80 (mast) |
| `ir0..2` forward array | +70 | +20 / 0 / −20 | 15 |
| `lidar` | +62 | 0 | 32 |
| ball transfer, front | +45 | 0 | contact |
| wheels | 0 | ±45 | — |
| ball transfer, rear | −45 | 0 | contact |
| `irb0..2` rear array | −70 | +20 / 0 / −20 | 15 |

Both optics sit on a forward boom past the chassis front edge at +60. A camera
at the chassis edge looking straight down sees its own body fill the rear half
of every frame.

`track_width` is **0.0994 m**, not the nominal 0.090. Calibrated against ground
truth: contact behaves like the wheels' outer edge, not their centre. The
nominal value cost 27° of heading drift per lap; the calibrated one gives about
0.1° per marker segment.

---

## Crossing a marker

A 100 mm tile on a 20 mm lane covers the line, so the robot crosses it blind.
Distances from the moment the colour sensor first sees orange:

```
d = 0 mm       colour sensor meets the tile's near edge
d = 55 mm      IR array enters the tile — the line is gone
d = 59-101 mm  code inside the camera's footprint, ~22 frames to decode
d = 155 mm     IR array clears the tile — the line is back
```

No alignment manoeuvre and no stop. The robot arrives square because it
followed the line to get there, and markers are laid along the lane tangent —
about 2 mm of cross-track error against 16 mm of slack.

While blind the firmware holds the **averaged** steering, not the last
instantaneous value. The PD output oscillates around what a curve needs, so a
sample near zero sends the robot straight for 250 mm — 31 mm off a 1 m-radius
arc, outside a 20 mm lane.

**A crossing is not a line loss.** The whole sequence, `OVER` and `RECOVERING`
both, is suppressed from `EVT LINE_LOST`, exempt from the lost-line timeout,
and exempt from the lost-line search. Each of those was a separate bug: the
companion throttled the robot 6.0 → 3.6 → 2.16 → 1.5 rad/s reading every marker
as "going too fast"; the timeout stopped the robot on a working track; and the
search — a hard turn at the steering limit — spun it 180°.

---

## Running it

Always from the repo root, because the volume mount is `./:/project`.

```bash
uv run python -m tools.gen_fleet          # after ANY fleet.yaml edit
RENDERING=off MISSION_DURATION=200 \
  docker compose -f compose.yml -f compose.fleet.yml up -d
#   browser view: http://localhost:1234/index.html
docker compose -f compose.yml -f compose.fleet.yml down
```

Python code is volume-mounted, so `restart bot_01 supervisor` picks up edits
with no rebuild. Only Dockerfile changes need `--build`.

**Both `bot_01` and `supervisor` must be connected or the world freezes** —
`bot_01` is `synchronization TRUE`, so Webots blocks until its controller
attaches. A one-off `docker compose run` also recreates `sim` with different
env vars unless you pass `--no-deps`, which silently turns rendering back on.

```bash
uv run pytest -q                                   # 136 tests, no Webots needed
docker compose ... logs bot_01     | grep "node "  # marker reads
docker compose ... logs supervisor | grep label:   # readout + localisation error
```

`tools/spike_*.py` are single-purpose diagnostics that each found a real bug —
device inventory, camera frames under `--no-rendering`, what the ground sensors
actually see, where marker solids really are, and turn accuracy against ground
truth. Reach for them before theorising.

---

## State of the work

**Verified by measurement**, on a full 8.3 m lap:

| | |
|---|---|
| markers decoded | 5 of 5, in order, shared schema |
| localisation fix error | 6–11 mm |
| line tracking | 1.11 mm mean, 8.06 mm max |
| wheel heading drift | under 1.7° per lap |
| obstacle reflex | trips at 178 mm, retreats to the previous marker |

**Built but unexercised:** `robot/turn.py` (in-place turning, unit-tested, 90°
accuracy against ground truth never measured — the spike container removed
itself before its log could be read) and `robot/network.py` (`RandomRouter`,
tested, but every marker currently has one exit so there is nothing to choose).

**Not built:** the track is a single oval with no branches, so `PT`/`TR`/`CH`/
`PK`/`YI` are five names for identical behaviour. A graph track is the blocker
for everything else — turning, routing, and the emulated network layer all sit
on top of it. `proto/firmware.proto` is a drafted gRPC schema for the
ESP32↔Pi link that no code references yet.

Full write-up of the marker subsystem, including the measurements behind every
number above: `docs/markers.md`.
