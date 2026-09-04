# Containerized Line-Following Robot Fleet in Webots

**Date:** 2026-09-04
**Status:** Approved (design)

## 1. Purpose and Scope

Build a line-following robot in Webots as the foundation for a larger multi-robot
SIH 2026 project. The line follower itself is milestone 1; the architecture must
survive growth to many robots, richer sensing, and eventually ROS 2.

The defining constraint is **one container per robot**, so each robot's compute
can be independently limited (RAM, CPU) and independently crashed, restarted, and
profiled. This forces extern controllers and rules out embedding control logic in
the simulator.

**In scope for milestone 1:**

- A custom differential-drive robot PROTO with a downward-facing IR sensor array.
- A parametrically generated track.
- PID line following with tunable gains and per-robot telemetry.
- A fleet of N robots, each in its own container, defined by a single manifest.
- Browser-based viewing of the simulation; no host GUI dependency.

**Out of scope for milestone 1:** junction/T-intersection logic, line-gap recovery,
inter-robot collision avoidance, ROS 2 nodes, real hardware.

## 2. Architecture

### 2.1 Process topology

```
                 docker compose network
  ┌──────────────────────────────────────┐
  │ sim   (cyberbotics/webots:R2025a)    │
  │   xvfb-run webots --stream --port=1234
  │   world contains N Robot nodes, each │
  │   with controller "<extern>"         │
  └───────────────┬──────────────────────┘
                  │ TCP :1234  (multiplexed)
     ┌────────────┼────────────┬──────────────┐
     │            │            │              │
  browser      bot_01       bot_02   …     bot_NN
  (w3d view)   container    container      container
               mem/cpu      mem/cpu        mem/cpu
               limited      limited        limited
```

Webots serves robot windows, web streaming, and **all** extern controller
connections on a single TCP port (1234 by default). The sim container therefore
exposes exactly one port. Each controller container connects with:

```
webots-controller --protocol=tcp --ip-address=sim --port=1234 \
                  --robot-name=bot_01 /robot/main.py
```

### 2.2 Startup ordering

Robot nodes use `synchronization TRUE`. Webots then waits for each extern
controller to connect before stepping that robot, so container start order does
not matter and no health-check orchestration is required.

The tradeoff, accepted deliberately: if one controller container dies, the
simulation stalls rather than continuing without that robot. This is the correct
default for reproducible experiments. A `synchronization FALSE` escape hatch is
exposed per robot in the manifest for fault-injection experiments where the
simulation should continue without a crashed brain.

### 2.3 What per-container limits do and do not bound

`mem_limit` / `cpus` on a controller container bound **that robot's brain**: the
Python process, its buffers, and any perception added later. They do **not** bound
that robot's share of the simulator. Physics, IR raycasting, and collision for all
robots run inside the single `sim` process, because the robots share one world and
must be able to interact within it.

This is a property of single-world simulation, not a limitation of this design.
The consequence to plan around: **the sim container is the scaling wall, not the
controllers.** Sim cost grows with robot count and with rays per robot
(5 IR sensors x N robots). Measure it with `webots --log-performance` before
assuming a fleet size is feasible.

## 3. Fleet Definition: Static Generation

A single manifest is the source of truth. A generator emits both the world file
and the compose file from it, so robot names in the world can never drift from
`--robot-name` in compose.

`fleet.yaml`:

```yaml
track:
  size: [4.0, 4.0]        # metres
  line_width: 0.02
  shape: oval             # consumed by tools/make_track.py

defaults:
  controller:
    base_speed: 4.0       # rad/s at the wheels
    pid: {kp: 0.9, ki: 0.0, kd: 0.04}
    lost_line_timeout_s: 2.0
  resources:
    memory: 256m
    cpus: "0.5"

robots:
  - name: bot_01
    pose: {x: 0.0, y: -1.4, theta: 0.0}
  - name: bot_02
    pose: {x: 0.3, y: -1.4, theta: 0.0}
    controller: {pid: {kp: 1.4}}     # deep-merged over defaults
    resources: {memory: 128m}
```

`tools/gen_fleet.py` reads `fleet.yaml` and writes:

- `worlds/track.wbt` — N `LineBot` nodes with matching `name` and `controller "<extern>"`.
- `compose.fleet.yml` — one service per robot, with its `--robot-name`, resource
  limits, and config path.
- `config/<name>.yaml` — the deep-merged per-robot controller config.

Generated files are committed, so a misbehaving robot can be diagnosed by reading
a real file. Regeneration is a single command; CI asserts the generated files are
in sync with the manifest.

Per-robot heterogeneity is the reason for this approach: varying gains, start
poses, memory budgets, or deliberately degrading one robot's sensors is the whole
point of giving robots separate containers.

## 4. Robot Model: `LineBot.proto`

Differential drive, Z-up world (Webots ENU convention since R2022a).

- Chassis: `Box 0.12 x 0.09 x 0.03`, mass ~0.4 kg.
- Two `HingeJoint`s with `RotationalMotor`s named `left wheel motor` /
  `right wheel motor`, wheel radius 0.02 m, axle separation 0.09 m.
- One passive caster sphere with low friction `contactMaterial`.
- Five `DistanceSensor` nodes, `type "infra-red"`, named `ir0`..`ir4`, mounted at
  the front, pointing along -Z, ~0.02 m above the floor, ~0.012 m apart laterally.

IR sensors are the correct choice here because Webots IR rays collide with `Solid`
nodes themselves rather than bounding objects, and their response is
colour-sensitive: light surfaces read higher than dark ones. The black line on a
white floor is therefore directly readable without a camera.

Exposed PROTO fields: `translation`, `rotation`, `name`, `controller`,
`synchronization`, `sensorSpacing`, `sensorHeight`. Everything else is fixed so
robots stay comparable.

## 5. Track Generation

`tools/make_track.py` renders a track PNG (Pillow) from the `track` block of the
manifest and writes `worlds/textures/track.png`.

**The ground must be a `Plane`, not the stock `Floor` PROTO.** Webots' IR
line-following support is documented as requiring the ground texture to be placed
in a `Plane`; the `Floor` PROTO builds its surface from an `IndexedFaceSet`. The
world therefore uses a plain `Solid` with `Plane` geometry and a `PBRAppearance`
whose `baseColorMap` is the generated PNG. A `Plane` maps its texture 1:1 across
`size` when viewed from above, so no `TextureTransform` is needed, and its normal
is the local z-axis, which matches the Z-up world. As a `boundingObject` a `Plane`
is infinite, which is exactly right for a floor.

Contrast is read on the **red channel**, not luminance: the IR reflection factor is
`f = 0.2 + 0.8 * red_level * (1 - 0.5*roughness) * (1 - 0.5*occlusion)`, where
`red_level` combines `baseColor`/`diffuseColor` with the texture's pixel value. A
black line on a white floor spans the full range (`f` = 0.2 vs 1.0). Keep the
appearance's `roughness` and `occlusion` maps unset so they do not modulate the
reading.

Generating rather than hand-drawing the track means track difficulty becomes a
parameter, and later milestones can emit tracks with junctions and gaps without
editing world files by hand.

## 6. Control Software

### 6.1 Pure core, thin adapter

The Webots API is confined to one file. Everything else is plain Python that runs
on the host with no simulator.

```
robot/
  main.py            # ONLY file importing `controller`; owns the step loop
  line_estimator.py  # raw IR readings -> normalised line position   (pure)
  pid.py             # PID with clamping and anti-windup             (pure)
  drive.py           # steering signal -> (left, right) wheel speeds (pure)
  config.py          # load + validate per-robot YAML                (pure)
  telemetry.py       # row buffering and CSV writing                 (pure I/O)
```

This split is the part of the design that matters long-term. Moving to ROS 2, to a
different simulator, or to real hardware replaces `main.py` and keeps the
algorithm and its tests intact.

### 6.2 Line estimation

Each raw reading is normalised against per-robot calibration references:

```
r_i = clamp((raw_i - black_ref) / (white_ref - black_ref), 0, 1)
w_i = 1 - r_i                       # weight: high on the dark line
line_pos = sum(w_i * x_i) / sum(w_i)    # x_i = sensor lateral offset, metres
```

`line_pos` is 0 when centred, negative when the line is left of centre. If
`sum(w_i)` falls below `min_confidence`, the line is considered lost.

`white_ref` and `black_ref` live in the per-robot config. `tools/calibrate.py`
drives a robot across the line and prints measured values to paste back into
`fleet.yaml`.

### 6.3 Control law

PID on `error = line_pos` toward a setpoint of 0, with output clamping and
integral anti-windup. Steering `u` maps to wheel speeds:

```
v_left  = clamp(base_speed - u, -max_speed, max_speed)
v_right = clamp(base_speed + u, -max_speed, max_speed)
```

Lost-line behaviour for milestone 1 stays deliberately simple: hold the last error
sign and rotate to reacquire; after `lost_line_timeout_s`, stop and log. Junction
and gap handling are explicitly deferred.

### 6.4 Telemetry

Each controller writes `out/<robot_name>.csv` (a mounted volume), one row per
control step: `t, ir0..ir4, r0..r4, line_pos, error, p, i, d, u, v_left, v_right,
lost`. On exit it writes `out/<robot_name>.summary.json` with run duration, mean
and max `|error|`, time spent lost, and control steps completed.

The summary file is what CI and tuning experiments assert on; the CSV is for
plotting. Per-robot files rather than one shared file, so N containers never
contend for a single writer.

## 7. Images

Two images, both in `docker/`:

- `sim.Dockerfile` — `cyberbotics/webots:R2025a-ubuntu22.04`, adds the world,
  PROTO, and textures. Entrypoint runs `xvfb-run webots --stream --batch --stdout
  --stderr --mode=realtime worlds/track.wbt`. The image ships Xvfb, and `w3d`
  streaming renders in the browser, so no GPU is required in the container.
- `controller.Dockerfile` — same Webots base initially, for `webots-controller`
  and the Python bindings. Robot code is bind-mounted for a fast edit loop.

Basing controller containers on the full Webots image looks wasteful but is not:
N containers share one image's layers, so disk is paid once, and RSS — the thing
actually being limited — is just the Python process. Slimming to a multi-stage
build that copies `webots-controller` and `lib/controller/` onto `python:3.13-slim`
is a later, measurable optimization, deliberately not done first so that linker
problems are not debugged before a robot works.

Pinned to `R2025a-ubuntu22.04`; the `latest` tag is stale at R2023b.

## 8. Testing

**Host unit tests (no Webots, no Docker, fast):** `pid.py` including clamping and
anti-windup, `line_estimator.py` including the lost-line boundary, `drive.py`
saturation, config merge and validation, track geometry, and fleet generation
(generated world and compose agree on every robot name).

**Containerised smoke test:** a one-robot fleet run headless (`--mode=fast
--no-rendering --batch`) for a fixed number of simulated seconds; assert from
`summary.json` that mean `|error|` is below threshold and time-lost is zero. This
is the regression gate and is CI-runnable.

Whether `--no-rendering` is compatible with `w3d` streaming is unverified. It
should be, since w3d renders client-side, and it would save sim CPU. It will be
tested during implementation; if it breaks the stream, it is used only for the
headless smoke test and dropped from the interactive path.

## 9. Repository Layout

```
SIH2026/
  fleet.yaml
  compose.yml                 # sim service
  compose.fleet.yml           # generated: one service per robot
  docker/{sim,controller}.Dockerfile
  protos/LineBot.proto
  worlds/track.wbt            # generated
  worlds/textures/track.png   # generated
  config/<robot>.yaml         # generated
  robot/                      # control software (section 6)
  tools/{make_track,gen_fleet,calibrate}.py
  tests/
  out/                        # telemetry, gitignored
  docs/superpowers/specs/
```

## 10. Build Order

1. Track generator + host tests.
2. `LineBot.proto` + a hand-written single-robot world; verify in the browser.
3. Pure control core (`pid`, `line_estimator`, `drive`, `config`) under TDD.
4. `main.py` adapter + `controller.Dockerfile`; one robot following the line.
5. Telemetry and summary output.
6. `gen_fleet.py`; scale to N robots with per-robot limits.
7. Headless smoke test wired to CI.

Each step is independently demonstrable, and steps 1-3 need no container at all.
