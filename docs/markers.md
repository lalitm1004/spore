# Floor markers — what was added, and how to run it

**This has been run.** All five markers decode on a full lap of the oval, and
the numbers below are measured, not predicted. Everything was verified with
`RENDERING=off`, so it holds at the CPU budget you actually want to run at.

    bot_01: node 20 TR at (3110, 1208) mm  ->  90deg>30
    bot_01: node 30 YI at (3237, 2676) mm  ->  180deg>40
    bot_01: node 40 CH at (1672, 3000) mm  ->  180deg>50
    bot_01: node 50 PK at (504, 2086) mm  ->  270deg>10

| measured over one 8.32 m lap | |
|---|---|
| markers decoded | 5 of 5, in order |
| line tracking | 1.49 mm mean, 6.94 mm max |
| line lost | never, including across every tile |
| position fix error | 1, 1, 8, 1 mm |
| heading error | 5.6, 4.8, 3.2, 4.5 deg |

---

## What a marker is

A QR code inside an orange border, on its own textured plane, laid along the
lane at a known point.

```
  100 mm tile ──────────────────┐
  ┌──────────────────────────┐  │  orange border, 15 mm  → the colour sensor's trigger
  │ ████████░░░████████      │  │  white quiet space,  5 mm
  │ ██  ██░░█░░░██  ██       │  │  QR code, 60 mm, 33 modules
  │ ████████░█░█████████     │  │  → 1.82 mm per module
  │  ░░░█░░██░░░█░░░         │  │  → 5.0 camera pixels per module
  └──────────────────────────┘  │
```

**Why not pixels in the track texture.** At the track's 512 px/m a QR module
would be one texel wide, and Webots would mipmap the finder patterns into
mush — the same failure class as the non-power-of-two rescale that resampled
the line edges. Its own tile decouples marker resolution from track resolution
entirely, at the cost of one small PNG per node.

**Why not literal ISO QR at full size.** A Version-1 QR is 21×21 modules plus a
mandatory 4-module quiet zone, and three corners are finder patterns rather
than data. Keeping the code at 60 mm inside a 100 mm tile is what buys the
5 px/module budget.

## The payload

```
20.TR.3110.1208.38.90:30
│  │  │    │    │  └─ out-edges: bearing:neighbour, slash-separated
│  │  │    │    └──── the tile's own bearing in the facility frame
│  │  │    └───────── y, absolute mm from the facility origin
│  │  └────────────── x, absolute mm
│  └───────────────── node kind
└──────────────────── node id
```

Kinds are `PT` pass-through, `TR` transfer, `CH` charging, `PK` parking,
`YI` yield. 24 characters, QR version 2, alphanumeric mode, 33 modules — the
bearing field costs no extra modules.

Three properties worth keeping:

- **Absolute coordinates, so the graph is emergent.** A robot reading a stream
  of these has observed the layout without ever being handed a map. There are
  no shared constants to agree on either — an id-plus-arithmetic scheme would
  need every robot to know the grid pitch, which is a small piece of shared
  configuration, which is a control plane wearing a very small hat.
- **Degree decides behaviour, not a separate kind.** One out-edge is a
  waypoint; three is a junction. Node *type* (what you can do here) and node
  *degree* (where you can go) are orthogonal facts, kept orthogonal.
- **The bearing makes it a heading reference, not just a position one.** The
  tile is laid along the lane, so its bearing plus the code's rotation within
  the camera frame is an absolute heading. Without it, a fix inherits whatever
  drift the wheels have accumulated — 27 degrees a lap, measured.

Separators are drawn from QR's alphanumeric set (`. / :`) so the code stays in
alphanumeric mode. Byte mode would cost roughly 45% more modules, and modules
are camera pixels.

## The crossing

A 100 mm tile laid on a 20 mm lane covers the line. The robot crosses it blind.
All distances are measured from the moment the colour sensor first sees orange:

```
  d = 0 mm       colour sensor (x=125) meets the tile's near edge
  d = 55 mm      IR array (x=70) enters the tile — the line is gone
  d = 59-101 mm  code inside the camera's footprint — ~22 frames to decode
  d = 155 mm     IR array clears the tile — the line is back
```

**No alignment manoeuvre and no stop.** The robot has been following the line
to get here and markers are laid along the lane tangent, so it arrives square
by construction: about 2 mm of cross-track error against 16 mm of slack between
the 60 mm code and the 92.7 mm footprint. Alignment is a *property of having
followed the line*, not a step.

While blind the firmware holds its last good steering and the lost-line timer
does **not** run — the line is absent by design, not by failure.

**The camera moved twice.** From x=30 to x=60 on the arithmetic — at x=30 the
code came into view only after the robot had regained the line — and then to
x=95 once a captured frame showed the chassis occluding the rear half of the
view. `test_camera_position_is_far_enough_forward` pins the first;
`out/qr_view.png` from `tools/spike_drive.py` is how the second was found.

## Sensor positions

Millimetres, in the robot frame. The origin is at wheel-axle height, so the
floor is 20 mm below it.

| device | x (fwd) | y (left +) | above floor | added |
|---|---|---|---|---|
| `ir0` `ir1` `ir2` | +70 | +20 / 0 / −20 | 15 | — |
| `color` | +125 | 0 | 15 | new |
| `qr` | +95 | 0 | 80 (mast) | new |
| `status` LED | −20 | 0 | 46 | new |
| left/right wheel sensor | 0 | ±45 | — | now used |
| wheels | 0 | ±45 | — | — |
| caster | −45 | 0 | 10 | — |

The QR camera is on a mast because at the IR array's 15 mm height a camera sees
only a 16 mm circle of floor. From 80 mm with a 1.05 rad FOV it sees a 92.7 mm
square at 0.362 mm/px. Both optics sit on a forward boom, ahead of the chassis
front edge at x=60 — see bug 2 below.

Webots has no colour-sensor device — `color` is a `Camera` with `width 1
height 1`, which reports a single RGB triple. That is what a TCS34725 gives you
over I2C on the real board.

## Files

| file | what it is |
|---|---|
| `tools/track/marker.py` | payload encode/decode, tile rendering — pure |
| `tools/make_markers.py` | CLI, one PNG per node |
| `robot/odometry.py` | wheel angles → distance — pure |
| `robot/marker.py` | colour trigger + crossing state machine — pure |
| `robot/qr.py` | OpenCV decode from a BGRA frame |
| `robot/supervisor.py` | ground truth + the on-screen readout |
| `robot/main.py` | `Optics` class, dead reckoning, lever-arm fix, `EVT MARKER` |
| `tools/spike_optics.py` | does `--no-rendering` keep camera frames |
| `tools/spike_drive.py` | what the ground sensors actually see, saves frames |
| `tools/spike_marker.py` | where the marker Solids really are |

`protos/LineBot.proto` gained `hasOptics`, both cameras, and the status LED.
`tools/gen_fleet.py` now emits marker Solids, `DEF` names, a supervisor node,
and per-robot optics config — and renders the marker textures, so a world can
never reference a PNG nobody generated.

## Running it

```bash
uv run python -m tools.gen_fleet          # markers, world, compose, configs
docker compose -f compose.yml -f compose.fleet.yml up --build
# then open http://localhost:1234/index.html
```

The controller image now installs `numpy` and `opencv-python-headless`, so the
first build is slower. `--build` matters on the first run.

### Seeing what was decoded

Three independent readouts. Only the first needs a browser:

1. **The status LED on each robot** — scene geometry, so it definitely renders.
   Amber while searching, then green `PT`, blue `TR`, cyan `CH`, violet `PK`,
   red `YI`.
2. **Supervisor overlay text** — `robot/supervisor.py` writes each robot's last
   decoded marker as a label in the 3D view, with its localisation error
   against ground truth. Whether `setLabel` reaches a w3d browser client is the
   one thing still unverified.
3. **The same text on stdout** — every label change is echoed, so
   `docker compose logs supervisor | grep label:` gives the readout without a
   browser at all:
   ```
   label: bot_01  node 30 yield  (3237, 2676) mm  next 40   fix error 1 mm  heading +4.8 deg
   ```

### Checking without the browser

```bash
docker compose logs bot_01 | grep "node "
#   bot_01: node 20 TR at (3110, 1208) mm  ->  90deg>30

cat out/bot_01.status.json
tail -5 out/bot_01.csv          # distance, crossing, node_id columns
```

## What running it found

Five bugs that only the simulator could surface. Listed because each cost real
time and the reasoning generalises.

1. **`--no-rendering` keeps camera sensors.** `tools/spike_optics.py` confirms
   frames arrive with `--no-rendering` verifiably in the command line. Camera
   sensors render offscreen, independently of the main view, so the QR camera
   and the 907% → 5.65% CPU saving are compatible. This was the biggest open
   risk and it resolved in your favour.

2. **The chassis occluded half the camera.** At `cameraX 0.06` — the chassis
   front edge — a downward camera saw its own blue body fill the rear half of
   every frame, leaving 46 mm of floor for a 60 mm code. Looking straight down,
   *any* backward ray hits the chassis, so no amount of extra height helps; the
   optics had to move onto a forward boom. `cameraX` is now 0.095 and
   `colorSensorX` 0.125.

3. **The rendered border is not the authored border.** A (255, 122, 0) tile
   renders as (242, 173, 56) under the world's lighting — chromaticity distance
   0.206 against a 0.12 threshold, so the trigger never fired. The threshold is
   now 0.30, chosen from measurement: the nearest confusable surface is the
   white floor at 0.481, so it sits between them with margin on both sides.

4. **Webots raises on a camera frame that does not exist yet.** There is no
   image until the step *after* `enable()`, and the Python binding raises
   `ValueError: NULL pointer access` rather than returning empty — so a
   `if not image` guard never runs. The reader now skips the enabling step.

5. **Odometry heading drifts 27 degrees a lap, and a lever-arm fix inherits
   it.** See below.

## The result worth putting on a slide

A marker read has to be converted from "the marker is at (x, y)" to "*I* am at
(x, y)", which means rotating a ~115 mm lever arm by the robot's heading. Doing
that with odometry heading gave a fix error that grew every lap, because the
odometry over-reports rotation by about 7.6%:

| | heading from odometry | heading from the marker |
|---|---|---|
| position fix error | 5 → 16 → 39 → 45 mm, growing | 1, 1, 8, 1 mm, bounded |
| heading error | 3 → 13 → 19 → 27 deg, growing | 5.6, 4.8, 3.2, 4.5 deg, bounded |

The fix is that the tile is *laid along the lane*, so its bearing — now carried
in the payload — plus the code's rotation within the frame is an absolute
heading that owes nothing to the wheels. On a straight it matches truth to a
fraction of a degree (marker 40: 180 deg authored, 179.98 deg true). On a curve
it carries a few degrees of bias, because the code is read about 60 mm before
the tile's centre and the lane turns in between.

That residual bias is the honest caveat, and it is the right kind: **bounded,
where odometry drift is not.** This is the whole argument for absolute fixes,
and it is now a measurement rather than a claim.

Recalibrate `Optics.CAMERA_ROTATION_ZERO` after any change to how the camera is
mounted:

```bash
docker compose ... /project/robot/supervisor.py --calibrate
```

## Still to check by eye

Only one thing needs a browser: **do the supervisor labels appear in the w3d
view?** Everything else is verified headlessly. If they do not, the readout is
still available three other ways — the status LED on each robot, the
`label:` lines in `docker compose logs supervisor`, and
`out/bot_01.status.json`.

## Known gaps

- **The oval has no branches.** `PT`/`TR`/`CH`/`PK`/`YI` only mean something on
  a graph — a robot "turns or goes straight" only where there is a choice, and
  `YI` has to be a physical spur off the lane, not a painted spot. The track
  generator knows exactly one shape. Turning it into a graph of segments with
  branch geometry is a bigger change than any of the above, and everything in
  layers 3–6 depends on it.
- **`out_edges` are currently decorative.** They are encoded, decoded and
  displayed, but nothing routes on them until there is somewhere to turn.
- **Confirmation, not query.** The design settled on the robot already knowing
  its route and using the marker to confirm position — match, keep moving;
  mismatch, stop and report. That decision is not yet implemented, because it
  needs the network layer to have sent a route in the first place.
- **`robot/main.py`** is still named for the era before the firmware/companion
  split. `robot/firmware.py` would read better.
