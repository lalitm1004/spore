# Architecture

Companion to `CLAUDE.md`, which carries the invariants and traps. This one
covers what each module is for and why the shape is what it is.

---

## 1. Two processes per robot

Each robot is two OS processes, joined by a `socat` pty pair so both sides open
a real serial device path exactly as they would on hardware.

| | `robot/main.py` — "the ESP32" | `robot/companion.py` — "the Pi" |
|---|---|---|
| owns | IR arrays, motors, cameras, lidar | nothing physical |
| Webots access | yes, the extern controller | **none** |
| rate | 62.5 Hz control loop | event-driven |
| sends | `EVT LINE_LOST / MARKER / OBSTACLE / STATUS` | `CMD SET_SPEED / STOP / START` |
| if the link dies | keeps following the line | irrelevant to control |

The split is a policy boundary, not just a process boundary. The companion sets
setpoints and can never command a wheel velocity. When it sees `LINE_LOST` it
concludes the robot is going too fast and lowers the target speed — a decision
with no business inside a control loop.

Wire format is newline-delimited ASCII (`robot/protocol.py`), readable with a
serial monitor. Everything lives behind `encode`/`decode`, so a binary framing
can replace it without either side's logic changing.

## 2. Pure core, thin adapter

Only `robot/main.py` and `robot/supervisor.py` import the Webots API.
Everything else is pure: no I/O, no simulator, host-testable. That is what made
the firmware/companion split cheap — the control modules moved across
untouched — and it is why the test suite runs in under a second.

```
robot/
  main.py          firmware: the only place Webots meets control    641 lines
  companion.py     the policy half                                   67
  supervisor.py    ground truth + the on-screen readout             193

  hal.py           MCU-like front end: 10-bit ADC, own sample clock   63
  line_estimator.py  IR counts -> cross-track position               46
  pid.py           PID with output limiting                          61
  drive.py         steering -> wheel speeds, turn authority first    36
  odometry.py      wheel angles -> pose and path length              82

  marker.py        colour trigger + crossing state machine          189
  qr.py            OpenCV decode of a BGRA frame                    121
  obstacle.py      the lidar reflex                                 183
  turn.py          in-place turning to an absolute heading           93
  network.py       stand-in router                                   87

  protocol.py      the firmware <-> companion wire format            68
  events.py        what the firmware reports upward, and when        48
  policy.py        the companion's decision making                   48
  config.py        per-robot configuration                          139
  telemetry.py     CSV recording and run summaries                   92

tools/
  gen_fleet.py     fleet.yaml -> world, compose, configs, textures  384
  manifest.py      fleet.yaml loading and validation                158
  track/centerline.py   analytic centreline (currently Oval only)    96
  track/raster.py       centreline -> ground texture                 55
  track/marker.py       QR payload + tile rendering                 210
  spike_*.py       six single-purpose diagnostics
```

## 3. Static generation from one manifest

`fleet.yaml` generates `worlds/track.wbt`, `compose.fleet.yml`,
`config/<robot>.yaml` and every texture. A robot's name in the world cannot
drift from the `--robot-name` its container connects with, because both come
from the same source.

Chosen over dynamic spawning because per-robot heterogeneity is the reason
separate containers exist at all — different gains, different memory limits, a
deliberately degraded robot.

The generated files are tracked in git. That is the same choice this repo
already makes for `warehouse.json` and `warehouse_map.svg`, and it means a
fresh clone can load the world without running anything first.

## 4. Localisation

Three sources, in increasing order of authority:

**Wheel odometry** integrates shaft angles into a pose (`robot/odometry.py`).
Good to about 0.1° of heading per marker segment after calibration, and it is
the only continuous source.

**The IR array** gives cross-track error against the lane, which is what the PD
loop steers on. It says nothing about position along the lane.

**A marker read** gives an absolute position fix. The QR carries the node's
position, and the robot converts "the marker is at (x, y)" into "*I* am at
(x, y)" by removing the lever arm between its own origin and the tile centre —
about 115 mm, rotated by its heading.

Heading is **not** corrected from markers. The shared QR schema carries no lane
bearing, and the obvious substitute — the chord between consecutive markers —
is only the lane's direction when the lane between them is straight. On this
track's arc it gave 85° where the lane ran at 133°, enough to turn the robot
around. In the real warehouse, edges are straight 2 m spans and the chord would
be exact; the oval is the outlier. It costs nothing anyway now that
`track_width` is calibrated.

## 5. The obstacle reflex

```
CLEAR ──►  STOPPING ──►  PAUSED ──►  BACKING ──►  HOLDING
        0.8 s ramp     1 s settle   0.6 s ramp   until it leaves
        down from      before       into
        cruise         reversing    reverse
```

Deliberately unhurried. Going straight from cruise into reverse pitches the
chassis hard enough to throw the camera boom around, and on real hardware is
how a gearbox dies.

Two things make the retreat work, and both were arrived at the hard way:

**It stops by counting orange bands, not by odometry.** Reversing over a tile
the colour sensor crosses the far band, the code, then the near band — and that
second band is where the robot stood before it drove on. Counting cannot drift
and does not care that the lane curves.

**It steers off the rear IR array.** Reversing off the forward array is
non-minimum-phase; the sensor sits ahead of the direction of travel, so
correcting toward the error rotates it further off. Measured before the rear
array existed: the line was absent for 91% of a retreat. After: 0%, with
1.55 mm mean cross-track.

Resuming needs the *obstacle* to move, not the robot. Resuming on "range is now
clear" livelocks, because reversing is itself what produced the clearance —
observed in sim as a BACKING/CLEAR cycle every four seconds. The robot is
stationary while holding, so only a further improvement in range can mean the
obstacle itself left.

## 6. What the simulator is honest about

The two things hardest to fake are not faked.

**Perception is real.** The sim rasterises the floor, warps it under each
robot's pose, and hands a genuine 512×512 frame to the same
`cv2.QRCodeDetector` call that would run on a Pi. A blurred or undersized tag
genuinely fails to decode rather than being modelled as failing — which is how
the error-correction level was chosen.

**The two-process split is real.** Twelve robots would be twenty-four OS
processes talking over real pty pairs, each independently killable.

What is simulated is rigid-body physics, which is not what this project is
about. The MCU-like HAL (`robot/hal.py`) deliberately quantises to 10-bit ADC
counts on its own sampling clock with optional transport latency, so
quantisation surprises surface in simulation rather than on hardware.

## 7. Open work, in dependency order

1. **A graph track.** `tools/track/centerline.py` knows exactly one shape,
   `Oval` — a single closed loop with no branches. Every node has one exit, so
   the five node types are five names for identical behaviour. Turning the
   generator into a graph of segments is the blocker for everything below it.
   Note that `signed_distance_to` becomes unsigned in the process: a graph has
   no inside.

2. **Turning.** `robot/turn.py` exists and is unit-tested. Its accuracy against
   ground truth has never been measured — `tools/spike_turn.py` and
   `spike_turn_truth.py` are written for exactly that and were never
   successfully run to completion.

3. **The emulated network layer.** `robot/network.py` has a seeded
   `RandomRouter` behind a `Router` interface, tested, with nothing to route
   on. The junction path — stop, ask, wait, turn, resume — is designed but not
   built.

4. **gRPC.** `proto/firmware.proto` is drafted for the ESP32↔Pi link. No code
   references it; the ASCII protocol is still what runs.
