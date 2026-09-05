# Running the fleet demo

Ten robots on the whole 114 x 60 m warehouse layout — 881 nodes — parked on
charging bays and released one at a time, each with its own network layer
handing it random turns at every junction. Every command runs from the repo root — the volume
mount is `./:/project`, so `./` is your shell's directory, not the compose
file's.

---

## Start

```bash
./fleet.sh up
```

That regenerates the world if `fleet.yaml` has changed, clears the previous
run's artefacts, and brings everything up with the flags below already set.
`./fleet.sh help` lists the rest; `MODE`, `RENDERING` and `MISSION_DURATION`
override its defaults, e.g. `MISSION_DURATION=600 ./fleet.sh up`.

By hand, if you would rather:

```bash
uv run python -m tools.gen_fleet
MODE=fast RENDERING=off MISSION_DURATION=600 \
  docker compose -f compose.yml -f compose.fleet.yml up -d
```

Then open **http://localhost:1234/index.html**, or over a tailnet,
`http://<this-machine>:1234/index.html`. Docker publishes on all interfaces
and no firewall change is needed.

`RENDERING=off` even for the browser view: w3d streaming renders client-side,
so server-side rendering is pure waste. It is the difference between 907% and
5.65% simulator CPU.

`MODE=fast` runs the world as fast as the machine allows; `MODE=realtime`
(the default) paces it to the wall clock. Webots has no numeric multiplier --
`fast` is the whole switch.

Do not expect much from it at ten robots. Measured on a 16-core box with
`RENDERING=off`: **1.12x real time**, with the simulator process at 270-460%
CPU and every controller under 30% of one core. The bottleneck is the simulator
itself -- physics plus one render pass per enabled sensor per tick, for ten
robots -- so raising `defaults.resources.cpus` buys headroom against a stall
during a QR decode, not speed. Getting materially past real time means fewer
robots or fewer sensors, not a flag.

The clock in the viewer is simulation time, and `MISSION_DURATION` is in
simulation seconds, so neither depends on how fast the host runs.

## Stop

```bash
./fleet.sh down
```

which is `docker compose -f compose.yml -f compose.fleet.yml down
--remove-orphans`.

`--remove-orphans` matters when you change the robot count: dropping from ten
to two leaves eight containers running against a world that no longer has
them, and a robot whose controller never attaches freezes the simulation.

## Restart after a code change

Python is volume-mounted, so no rebuild:

```bash
docker compose -f compose.yml -f compose.fleet.yml restart
```

Two exceptions that **do** need `docker build`:

```bash
docker build -f docker/controller.Dockerfile -t sih2026/controller:dev .
```

- `docker/robot-entrypoint.sh` — it is `COPY`'d into the image, so editing it
  changes nothing until you rebuild. This has cost time before.
- Either Dockerfile.

## Changing the fleet

Edit `fleet.yaml`, then **always** regenerate:

```bash
uv run python -m tools.gen_fleet
```

One command rewrites the floor texture, the marker textures,
`config/warehouse.json`, `worlds/track.wbt`, `compose.fleet.yml` and every
per-robot config. They cannot drift apart because they all come from the one
manifest.

To change the fleet size, edit the count -- poses are computed from the track's
charging bays, so a layout change cannot leave stale coordinates behind:

```yaml
robots:
  count: 10
  spawn: charging
  start_interval_s: 4.0
```

To move the window, edit `track.warehouse.origin_cm` and `size_m`. The window
need not be a power of two any more -- the lanes are geometry and nothing
senses the floor -- but a marker tile must be, or Webots
rescales the texture, which resamples the exact lane edges the IR array reads.
This finds the charging-densest window of a given size:

```bash
uv run python -c "
import json
d = json.load(open('../spore/spore-warehouse-layout/output/warehouse.json'))
ch = [n for n in d['nodes'] if n['node_type'] == 'CH']
w, h = 3200, 1600   # cm
best = max(
    ((sum(1 for n in ch if x <= n['position']['x'] < x+w
                        and y <= n['position']['y'] < y+h), x, y)
     for x in range(0, 12000-w+1, 200) for y in range(0, 7000-h+1, 200)))
print('origin_cm: [%d, %d]  -> %d charging bays' % (best[1], best[2], best[0]))
"
```

---

## What to watch

**In the browser.** The floor is the warehouse's own `warehouse_map.svg` --
region blocks in their own colours, the lane network as the layout tool draws
it -- with an orange-bordered QR tile at each of the 881 nodes and the 20 mm
guide line drawn on top at true width. Each robot carries a status LED that changes
colour with the kind of node it last read: amber while searching, then green
for pass-through, blue transfer, cyan charging, violet parking, red yield.

**In the logs.** This is where the routing is legible:

```bash
docker compose -f compose.yml -f compose.fleet.yml logs -f bot_01
```

```
bot_01: node 113 PT charging/PT/097 at (1000.0, 400.0) cm  region 3
netlayer: bot 1 at node 113 -> node 114
bot_01: turning to 90 deg for node 114
```

Three processes speaking in turn: the firmware decodes a QR and stops on the
node; the companion reports **which node** the robot is at over a unix socket;
this robot's network layer picks a neighbour and answers with **which node** to
head for; the companion turns that into a bearing and the firmware rotates to
it. Nothing on the wire says left or right — both ends hold the map, so the
direction is derived rather than transmitted, and it is exact because lanes are
straight.

Fleet-wide:

```bash
docker compose -f compose.yml -f compose.fleet.yml logs 2>&1 | grep -c "netlayer: bot"
docker compose -f compose.yml -f compose.fleet.yml logs 2>&1 | grep "turning to"
```

**In the telemetry.** `out/bot_XX.csv`, one row per 16 ms control step, with
the crossing state, the obstacle state, lidar range, and the node last read.

---

## The thing worth pointing at

**Ten robots on random routes will jam, and that is the result, not a bug.**

On the real layout this is sharper than on a lattice: charging bays are
degree-1 spurs and they pair onto a single junction, so two robots leaving
facing bays would contend for the same node immediately. The fleet is released
one robot every `start_interval_s`, ordered so bay-mates are a full pass
through the junctions apart -- 20 s at the default interval, against the 16.7 s
a robot needs to cover its 2 m spur -- so a pair never meets at its own
junction. That buys the start; it buys nothing after it. The underlying
contention is real everywhere else on the map and is exactly what coordination
is for.

Robots have no idea what any other robot intends. Their only defence is the
forward lidar reflex, which stops and reverses to the previous marker. Two
robots meeting head-on both retreat and both re-approach; a robot parked behind
another waits for it to move, and it never does. Measured before a timeout was
added: one robot spent **69% of a run** parked behind another.

That is the argument for the real network layer in one picture. Coordination is
not a nicety on top of a working fleet — without it, ten robots spend most of
their time waiting for each other. The random router is the floor
any allocation algorithm has to beat, and it is deliberately the dumbest thing
that still exercises the whole path.

The reflex now gives up holding after 8 seconds and retries. That does not
resolve the conflict — resolving it is the network layer's job and the reflex
has no business trying — it only stops a jam being permanent.

---

## Demonstrating that it is really distributed

Each robot runs its **own** network layer in its own process. A single shared
service answering every robot would be a control plane wearing a hat.

```bash
# kill one robot's coordinator; the others do not notice
docker compose -f compose.yml -f compose.fleet.yml exec bot_03 pkill -f netlayer.py
```

`bot_03` keeps following its lane and stops at the next junction with nothing
to tell it where to go — its firmware never had a network dependency to lose.
Every other robot carries on.

```bash
# kill a whole robot; the world keeps running
docker compose -f compose.yml -f compose.fleet.yml stop bot_05
```

Its lane is now blocked, and the robots that meet it will back off to their
previous markers — which is the reflex doing exactly what it is for.

---

## If something looks wrong

| symptom | cause |
|---|---|
| simulation frozen, no robots moving | a robot's controller is not attached. Every `LineBot` is `synchronization TRUE`, so Webots blocks until all of them connect. Usually an orphan container from a previous fleet size — `down --remove-orphans` |
| a robot never reads a marker | `optics.enabled` false in its `config/*.yaml`. Regenerate |
| `no map: junctions will not be answered` | `config/warehouse.json` missing — run `gen_fleet` |
| entrypoint edits having no effect | it is baked into the image; rebuild |
| floor blank in the browser | the texture is served from the project root but referenced relative to the world file |
| floor shows a *previous* track | it should not any more -- the texture is content-addressed (`track-<hash>.png`) precisely so the viewer cannot serve a cached one. If it happens, check what the browser is fetching before checking what the generator wrote |
| node markers look like giant coloured blobs | the map's own node dots, which are 273-727 mm once the drawing is a floor. They are stripped in `svgfloor.py`; if they return, that strip is not reaching `rsvg-convert` |

Single-purpose diagnostics live in `tools/spike_*.py` — device inventory,
camera frames under `--no-rendering`, what the ground sensors actually see,
where marker solids really are. Each was written to answer one question that
guessing had failed to settle, and each found a real bug.
