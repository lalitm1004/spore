# Running the fleet demo

Ten robots on a 4×4 lattice, each with its own network layer handing it random
turns at every junction. Every command runs from the repo root — the volume
mount is `./:/project`, so `./` is your shell's directory, not the compose
file's.

---

## Start

```bash
uv run python -m tools.gen_fleet
RENDERING=off MISSION_DURATION=600 \
  docker compose -f compose.yml -f compose.fleet.yml up -d
```

Then open **http://localhost:1234/index.html**, or over a tailnet,
`http://<this-machine>:1234/index.html`. Docker publishes on all interfaces
and no firewall change is needed.

`RENDERING=off` even for the browser view: w3d streaming renders client-side,
so server-side rendering is pure waste. It is the difference between 907% and
5.65% simulator CPU.

Ten robots run slower than real time — roughly 0.6× on a laptop. The clock in
the viewer is simulation time.

## Stop

```bash
docker compose -f compose.yml -f compose.fleet.yml down --remove-orphans
```

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

To change the fleet size, edit the `robots:` list. Poses must sit on a lane —
lane midpoints are the safe choice, and this prints them:

```bash
uv run python -c "
from tools.track.graph import lattice
g = lattice(4, 4, 2.0)
for a, b in sorted((e.a, e.b) for e in g.edges):
    na, nb = g.nodes[a], g.nodes[b]
    print('  - name: bot_XX')
    print('    pose: {x: %.3f, y: %.3f, theta: %.4f}   # lane %d-%d'
          % ((na.x + nb.x) / 2, (na.y + nb.y) / 2, g.bearing(a, b), a, b))
"
```

---

## What to watch

**In the browser.** The floor is a 4×4 lattice of lanes with an orange-bordered
QR tile at each of the 16 nodes. Each robot carries a status LED that changes
colour with the kind of node it last read: amber while searching, then green
for pass-through, blue transfer, cyan charging, violet parking, red yield.

**In the logs.** This is where the routing is legible:

```bash
docker compose -f compose.yml -f compose.fleet.yml logs -f bot_01
```

```
bot_01: node 1 PT aisle/PT/001 at (300.0, 100.0) cm  region 1
netlayer: node 1 ['left', 'straight'] -> straight (node 2)
bot_01: turning to 0 deg for node 2
```

Three processes speaking in turn: the firmware decodes a QR and stops; the
companion looks the node up in `warehouse.json`, works out which turns exist,
and asks this robot's network layer over a unix socket; the network layer picks
one at random and answers. Note it offers only the turns that lead somewhere —
at node 3, a corner, `['left']` is the whole list.

Fleet-wide:

```bash
docker compose -f compose.yml -f compose.fleet.yml logs 2>&1 | grep -c "netlayer: node"
docker compose -f compose.yml -f compose.fleet.yml logs 2>&1 | grep "turning to"
```

**In the telemetry.** `out/bot_XX.csv`, one row per 16 ms control step, with
the crossing state, the obstacle state, lidar range, and the node last read.

---

## The thing worth pointing at

**Ten robots on random routes will jam, and that is the result, not a bug.**

Robots have no idea what any other robot intends. Their only defence is the
forward lidar reflex, which stops and reverses to the previous marker. Two
robots meeting head-on both retreat and both re-approach; a robot parked behind
another waits for it to move, and it never does. Measured before a timeout was
added: one robot spent **69% of a run** parked behind another.

That is the argument for the real network layer in one picture. Coordination is
not a nicety on top of a working fleet — without it, ten robots on 48 m of lane
spend most of their time waiting for each other. The random router is the floor
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
| floor blank in the browser | the texture is served from the project root but referenced relative to the world file. `../textures/track.png` is the one spelling both resolve |

Single-purpose diagnostics live in `tools/spike_*.py` — device inventory,
camera frames under `--no-rendering`, what the ground sensors actually see,
where marker solids really are. Each was written to answer one question that
guessing had failed to settle, and each found a real bug.
