# Architecture

Companion to `CLAUDE.md`, which carries the invariants and the traps. This one
is the handover: what every module is for, why the shape is what it is, what
has been measured, and what is still open.

---

## 1. What this is, and what it is not

This is the **robot half** of an AMR fleet: perception, motion control, the
physical layer, and a simulator faithful enough to develop them against. The
distributed coordination the project is really about — task allocation,
reservations, deadlock resolution — is somebody else's code. It is represented
here by `robot/network.py`'s `RandomRouter`, which picks a neighbour uniformly
and knows nothing about tasks, congestion or other robots.

That stand-in is not a placeholder to be embarrassed about. It is the interface
frozen early, so the firmware and the companion were written against their
final contract rather than ported to it later, and a fleet of random routers is
a real baseline — random assignment is the floor any allocation algorithm has
to beat.

## 2. Three processes per robot

Each robot is three OS processes in one container.

| | `robot/main.py` | `robot/companion.py` | `robot/netlayer.py` |
|---|---|---|---|
| stands for | the ESP32 / Arduino | the Pi | the network layer |
| owns | IR arrays, motors, cameras, lidar | the map, the policy | routing |
| Webots access | yes, the extern controller | none | none |
| rate | 62.5 Hz control loop | event-driven | request/response |
| talks to | companion, over a socat pty | firmware and netlayer | companion |

The firmware and companion are joined by a `socat` pty pair, so both sides open
a real serial device path exactly as they would on hardware. The companion and
network layer are joined by a unix socket carrying newline-delimited JSON —
a real serialisation boundary, so replacing the stand-in with the real
TypeScript layer means changing what listens on the socket and nothing else.

**The split is a policy boundary, not just a process boundary.** The companion
sets setpoints and can never command a wheel velocity. When it sees
`LINE_LOST` it concludes the robot is going too fast and lowers the target
speed, and when the line has been clean for `speed_recover_after_s` it gives
the step back — a decision with no business inside a control loop. Both
directions matter: while the throttle only went down, one transient early in a
run left a robot at a quarter of cruise for the rest of it. The firmware has no
network code at all: it reports "I am at node 7" and receives "turn to 90
degrees".

That exchange only works because both halves share a heading frame. `TURN`
carries an *absolute* bearing off the map, and the firmware's only feedback is
its odometry, so the companion sends the heading the robot arrived on — the
bearing of the lane it came down, which is exact — and the firmware seeds the
frame at boot from `odometry.start_theta`. Neither half owns the frame alone,
which is why a mismatch was invisible in both.

**The wire names nodes, never directions.** `RobotToNetwork` carries
`latest_node_id` and `NetworkToRobot` carries `target_node_id`; both schemas
set `additionalProperties: false`, so there is no room for a menu of turns
going up or a `left`/`right` coming down. The network layer routes and holds
the map for it; the robot holds the map as well and derives the bearing to the
node it was named. That is exact — lanes are straight — so a direction on the
wire would be a second, weaker description of geometry both ends already have.

**One network layer per robot, not one shared service.** A single service
answering every robot would be a control plane wearing a hat. Per-robot also
makes killing one robot's coordinator a real thing to demonstrate: its firmware
never had a network dependency to lose, so it keeps following its lane and
stops at the next junction with nothing to tell it where to go.

## 3. Pure core, thin adapter

Only `robot/main.py` and `robot/supervisor.py` import the Webots API.
Everything else is pure: no I/O, no simulator, host-testable. That is what made
the firmware/companion split cheap — the control modules moved across
untouched — and it is why 167 tests run in about a second.

```
robot/
  main.py          firmware: the only place Webots meets control    ~700 lines
  companion.py     the policy half, and the junction handshake       ~130
  netlayer.py      the stand-in network layer, as a process          ~110
  supervisor.py    ground truth and the on-screen readout            ~190

  hal.py           MCU-like front end: 10-bit ADC, own sample clock    63
  line_estimator.py  IR counts -> cross-track position                 46
  pid.py           PID with output limiting                            61
  drive.py         steering -> wheel speeds, turn authority first      36
  odometry.py      wheel angles -> pose and path length                82

  marker.py        colour trigger + crossing state machine            190
  qr.py            OpenCV decode of a BGRA frame                       120
  obstacle.py      the lidar reflex                                   200
  turn.py          in-place turning to an absolute heading             93
  navigator.py     map lookup, and the socket to the network layer    200
  network.py       Query/Decision wire types, RandomRouter            140

  protocol.py      the firmware <-> companion wire format              68
  events.py        what the firmware reports upward, and when          48
  policy.py        the companion's speed policy                        48
  config.py        per-robot configuration                            160
  telemetry.py     CSV recording and run summaries                     92

tools/
  gen_fleet.py     fleet.yaml -> world, compose, configs, textures    ~560
  manifest.py      fleet.yaml loading and validation                  ~280
  make_markers.py  one QR tile per node
  svg2png.py       SVG -> PNG with a backend fallback chain           ~150
  track/graph.py   the lane graph and turn resolution                 ~270
  track/warehouse.py  warehouse.json export, and window loading       ~120
  track/svgfloor.py   the warehouse's own map as the floor            ~290
  track/raster.py     lane rendering for generated lattices            85
  track/marker.py     QR payload and tile rendering                   ~210
  track/centerline.py the original analytic oval, still used          ~120
  spike_*.py       six single-purpose diagnostics
```

## 4. One manifest generates everything

`fleet.yaml` generates the floor texture, every marker texture,
`config/warehouse.json`, `worlds/track.wbt`, `compose.fleet.yml` and every
`config/bot_XX.yaml`. A robot's name in the world cannot drift from the
`--robot-name` its container connects with, because both come from one source.

Three track types are supported, and `TrackConfig.from_dict` dispatches on
which block is present:

```yaml
track:
  warehouse:                  # a window of the real layout  (current)
    source: ../spore/spore-warehouse-layout/output/warehouse.json
    origin_cm: [8200, 600]
    size_m: [32, 16]
    pixels_per_metre: 256

track:
  graph: {rows: 4, columns: 4, spacing: 2.0}    # a generated lattice

track:
  shape: oval                 # the original single loop
  plane_size: [4.0, 4.0]
  track_size: [3.0, 2.0]
```

Robots are placed either explicitly or from the track:

```yaml
robots:
  count: 10
  spawn: charging             # poses computed from the CH nodes
```

Generated files are tracked in git. That is this repo's existing convention for
`warehouse.json` and `warehouse_map.svg`, and it means a fresh clone can load
the world without running anything first.

## 5. The warehouse window

The whole layout runs: **881 nodes, 952 edges, 34 charging bays** over 114 x
60 m. It did not always, and the arithmetic that used to forbid it is worth
keeping, because it was wrong in an instructive way.

The old sums:

- One floor texture at line-following resolution would be **32768 x 16384 px**
  — 2.1 GB — against a texture limit Webots hits well before that.
- A 1024x1024 QR tile for each of 881 nodes is another **3.7 GB**.

Both numbers were real and both were self-inflicted. The floor was that big
only because it carried the guide line, and a 20 mm line needs several pixels
across to be followable; the lanes are geometry now, so the floor is a picture
and 64 px/m is plenty. The tiles were that big only because nobody had measured
what the camera can actually resolve — 5.52 px/mm, against tiles drawn at
10.24.

What it costs now:

| | |
|---|---|
| floor, 7680 x 4224 at 64 px/m | 130 MB |
| 881 marker tiles at 512 x 512 | 924 MB |
| 1833 lane and node pieces, as geometry | 0 |
| **total** | **1.05 GB** |

`tools/track/warehouse.load_window` still takes a rectangle, because the window
is also what gives the floor a margin: it is the node span plus a metre, and
the plane is that plus `margin_m` again on every side. A node on the plane's
edge is a robot that can drive off the world — see the trap in `CLAUDE.md`.
Positions are rebased so the window's centre is the world origin. Edges with
only one end inside are dropped, and nodes left with no lane go with them: a
marker tile no robot can reach is worse than no tile.

## 6. The floor is the warehouse's own drawing

`warehouse_map.svg` is what the layout tool produces alongside the JSON, so the
simulated warehouse looks like the warehouse rather than like something this
project drew from the same data. `tools/track/svgfloor.py` crops the window out
of it with `rsvg-convert`, then draws the 20 mm guide line on top at true width
— the map's lanes are hairlines, which is right for a diagram and useless for
an IR array.

Three things about that were only settled by measurement:

**The coordinate mapping is `x_px = 0.11*x_cm + 90`, `y_px = 860 - 0.11*y_cm`.**
Fitted against all 952 lane endpoints. The y axis is *inverted*, which a fit on
ranges cannot tell you — min-to-min and max-to-max match either way. The
landmark that settles it: the CHARGING label sits at svg y=723 while its nodes
are at world y=800 cm.

**The map's node dots are removed.** They are drawn at 1.5-4 svg px, right for
a 1500 px diagram and **273-727 mm** once that drawing is a floor — three to
seven times the 100 mm marker tile and bigger than the 120 mm robot. They
swallowed the QR tiles. Nothing is lost: every node carries a real tile, which
is the physical thing a robot reads.

**The texture is content-addressed.** `track-<hash>.png`, because the streaming
viewer caches the floor by URL and a regenerated floor kept arriving in the
browser as the previous one.

`tools/svg2png.py` is the general utility behind this, trying backends in order
of fidelity — `rsvg-convert`, `qlmanage`, `cairosvg`, then a built-in reader
that draws rects, lines and circles. The fallback chain exists because
`cairosvg` wants a system libcairo macOS does not ship, and a demo that needs a
`brew install` is a demo that fails on a teammate's laptop.

## 7. Localisation

Three sources, in increasing order of authority:

**Wheel odometry** integrates shaft angles into a pose. Good to about 0.1
degrees of heading per marker segment after calibration, and the only
continuous source.

**The IR array** gives cross-track error against the lane, which the PD loop
steers on. It says nothing about position along the lane.

**A marker read** gives an absolute position fix. The QR carries the node's
position; the robot converts "the marker is at (x, y)" into "*I* am at (x, y)"
by removing the ~115 mm lever arm between its origin and the tile centre,
rotated by its heading.

Heading is **not** corrected from markers. The shared QR schema carries no lane
bearing, and the obvious substitute — the chord between consecutive markers —
is only the lane's direction when the lane between them is straight. On the
original oval it gave 85 degrees where the lane ran at 133, enough to turn the
robot around. It costs nothing now that `track_width` is calibrated.

## 8. The junction handshake

```
firmware                companion                    netlayer
   |                        |                            |
   | EVT MARKER node=7 -->  |                            |
   | (rolls onto the node)  | {"latest_node_id":7,...}-> |
   |                        |                            | (holds the map,
   |                        |                <-- {"target_node_id":9}  picks 9)
   |                        | bearing(7 -> 9)            |
   |  <-- CMD TURN bearing  |                            |
   | (rotates, reacquires)  |                            |
```

The robot sends **where it is**, and is told **where to go next**. It does not
send the turns that exist and it is not answered with a direction: both fields
are absent from `shared/schemas/`, which sets `additionalProperties: false` on
each message. The network layer holds the map because routing is what it is
for; the robot holds it too, so `bearing(7 -> 9)` is a local calculation and
exact, because lanes are straight.

That narrowness earned itself. While the robot resolved the turns, the menu it
offered was filtered to within 45 degrees of left, straight and right — and the
way back out of a degree-1 charging bay is 180 degrees, which matched none of
them. The menu came back empty, the companion reported "nowhere to go from
here", and a robot routed into a bay sat in it for the whole run: measured, 90%
of one. Routing by node has no such case. A bay has one neighbour, it is the
junction, and there is nothing to filter.

`RandomRouter` still avoids sending a robot straight back the way it came, but
that is now a routing preference on the routing side rather than a geometric
filter on the robot's, and it is a preference rather than a rule — at a dead
end the way back is the only answer there is.

`query_id` is echoed back so a fresh answer is distinguishable from a late
answer to the previous junction — two junctions can share a target, which is
exactly when confusing them would matter.

The firmware's wait is bounded by `junction_timeout_s` (6 s), after which it
carries straight on. A robot that is never answered must not hold a lane for
the rest of the run.

## 9. The obstacle reflex

```
CLEAR ──► STOPPING ──► PAUSED ──► BACKING ──► HOLDING
       0.8 s ramp    1 s settle  0.6 s ramp  until it leaves,
       down from     before      into        or 8 s, whichever
       cruise        reversing   reverse     comes first
```

Deliberately unhurried. Going straight from cruise into reverse pitches the
chassis hard enough to throw the camera boom around, and on real hardware is
how a gearbox dies.

Four things make the retreat work, all arrived at the hard way:

**It stops by counting orange bands, not by odometry.** Reversing over a tile
the colour sensor crosses the far band, the code, then the near band — and that
second band is where the robot stood before it drove on. Counting cannot drift
and does not care that the lane curves.

**It steers off the rear IR array.** Reversing off the forward array is
non-minimum-phase; the sensor sits ahead of the direction of travel, so
correcting toward the error rotates it further off. Measured before the rear
array existed: the line was absent for 91% of a retreat. After: 0%, with
1.55 mm mean cross-track.

**It gives up after 0.45 m.** At 2.0 m — a full lane span — a robot at the grid
edge reversed straight off the floor. Clearing a blocked lane needs
centimetres.

**Resuming needs the obstacle to move, not the robot.** Resuming on "range is
now clear" livelocks, because reversing is itself what produced the clearance —
observed as a BACKING/CLEAR cycle every four seconds. The robot is stationary
while holding, so only a *further* improvement in range can mean the obstacle
left. But holding also times out after 8 s, because two robots waiting on each
other never move: one spent 69% of a run parked behind another.

## 10. What the simulator is honest about

**Perception is real.** The sim rasterises the floor, warps it under each
robot's pose, and hands a genuine 512x512 frame to the same
`cv2.QRCodeDetector` call that would run on a Pi. A blurred or undersized tag
genuinely fails to decode rather than being modelled as failing — which is how
the error-correction level was chosen.

**The process split is real.** Ten robots are thirty processes over real pty
pairs and real unix sockets, each independently killable.

What is simulated is rigid-body physics, which is not what this project is
about. The MCU-like HAL (`robot/hal.py`) deliberately quantises to 10-bit ADC
counts on its own sampling clock with optional transport latency, so
quantisation surprises surface in simulation rather than on hardware.

## 11. Schema conformance

| schema | status |
|---|---|
| `qr-code.schema.json` | **conforms exactly** — all 83 payloads validate |
| `warehouse.json` shape | **matches** — same keys, `units: cm`, `node_spacing: 200` |
| `robot-to-network.schema.json` | **not used** — see below |
| `network-to-robot.schema.json` | **not used** — see below |

The junction handshake invented its own `Query`/`Decision` types rather than
using the two message schemas, and that gap is worth closing deliberately:

- **Their down-message is already sufficient.** `NetworkToRobot
  {target_node_id, timestamp}` is enough — the robot knows its current node and
  derives the bearing from the map, so the `turn` field is redundant. It needs
  `query_id` added, or `timestamp` used for the same purpose.
- **Settled: the junction query *is* `RobotToNetwork`.** It used to be a
  blocking question carrying a list of legal exits, which the schema has no
  field for. Rather than add one, the query was narrowed to what the schema
  already says — `latest_node_id` — and the answer to `target_node_id`. A real
  system will still want periodic telemetry on the same message; that is a
  cadence question now, not a shape one.

## 12. Open work, in dependency order

1. **Align the junction messages** to `robot-to-network` / `network-to-robot`,
   after agreeing the above with whoever owns the schemas.
2. **Measure turn accuracy.** `robot/turn.py` is unit-tested but has never been
   checked against ground truth. `tools/spike_turn.py` and
   `spike_turn_truth.py` exist for exactly this and have never completed a run
   — the first attempt used `--rm` and the container deleted its own logs.
3. **The real network layer.** `robot/netlayer.py` is a socket and a `route()`
   call; replacing the router is the whole change.
4. **gRPC.** `proto/firmware.proto` is drafted for the firmware link. No code
   references it; the ASCII protocol is what runs.

## 13. Things that are true and surprising

Collected because each one cost an hour and none is guessable from the code.

- Ten robots on random routes **jam**, and that is the result rather than a
  bug. It is the argument for coordination in one measurement.
- `optics.enabled` was derived from the manifest's marker list, which a graph
  track leaves empty — so every graph world silently ran with its cameras off.
- Turning read as a line loss, so the companion throttled after every turn.
- A robot stopped on a tile never finishes its crossing: the colour sensor
  stays over orange, so the state machine keeps re-entering `OVER` and
  resetting its distance counter. It then drives away *blind* on pre-turn
  steering, which after a 90 degree turn walks it off the lane.
- `RECOVERING` suppresses the lost-line timeout, and if the line is never found
  it suppresses it for ever — the robot falls into the full-lock search, stops
  travelling, and so never spends the distance budget that would end the
  crossing. One robot span on the spot for 96 seconds.
- Spawn placement matters more than it sounds. Robots on arbitrary lanes put
  pairs on a collision course before they had moved a metre.
