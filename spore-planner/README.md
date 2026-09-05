# spore-planner

Traffic-aware path planning for Spore AMRs.

Spore has no central controller: every robot runs its own planner, and the only
shared knowledge is a 200 ms heartbeat that carries each peer's claimed next
`K_COMMIT` hops as timed node reservations. This package turns that snapshot into a
path that routes *around* other robots' declared traffic instead of colliding with
it and backing off.

It is a pure function from a world snapshot to a proposed path.

## Scope

**In scope**

- Space-time planning against peer node reservations
- Soft congestion avoidance beyond the reservation horizon
- A combined time + energy cost model, weighted by the robot's energy state
- Destination resolution for class goals (`CHARGE` -> best `CH`, `PARK` -> best `PK`)
- Advisory yield-node suggestions
- Path hysteresis, so near-equal routes do not thrash

**Out of scope** — owned by other layers, consumed here as given

- Claiming, renewing and releasing reservations
- Conflict resolution and the priority ordering
- The decision to yield, and deadlock breaking
- Heartbeat transport, the mission and motion state machines
- Emitting `NetworkToRobot`

## Install and run

```sh
uv sync
uv run pytest
```

## Two invariants worth knowing

**Node reservations must overlap across a traversal.** Reservations address nodes
only, so edge and head-on-swap conflicts have to be inferred. That works precisely
because a robot holds *both* endpoint nodes for the whole duration of a traversal —
it does not release A until it is fully inside B, so `t_out(A) >= t_in(B)`. Two
robots swapping across an edge then both claim both nodes at once, which is caught
as an ordinary interval overlap. The reservation layer must publish windows this way
or the guarantee does not hold.

**Every bay is a dead end.** On the current floor plan all 34 `CH`, all 50 `PK` and
all 15 `YI` nodes have exactly one neighbour, so every charge, park and yield
manoeuvre ends in a 180 degree reversal, and the contention point is the anchor node
on the lane rather than the bay itself. The kinematics model charges for this;
goal-resolution costs would be materially wrong without it.

## How it plans

Continuous-time **Safe Interval Path Planning** over `(node, heading, interval)`,
with two candidate manoeuvres per edge: flow straight through at speed, or stop,
turn if needed, wait if it helps, and go. Waiting is therefore priced like any other
manoeuvre, which is what lets the energy term decide between waiting and detouring.

Heading is part of the search state because robots rotate in place — without it the
search cannot tell a straight run from a zig-zag.

Beyond the reservation horizon there are no hard constraints, so a soft congestion
field takes over: where the peers are, what the gossip says, how tightly the region
is laned, and what has been reported obstructed. It only ever adds cost, which is why
the A* heuristic stays admissible.

That heuristic is the exact remaining hop count, from a BFS off the goal cached on
the graph. On a network this sparse it keeps the search inside the true corridor
instead of flooding the floor: a median 26-hop route takes about 84 expansions.

**Measured on the real 881-node map with 19 peers:** `plan()` runs in 0.9 ms at p50
and 3.5 ms at p99 — about 57x headroom inside the 200 ms heartbeat tick.

## Known hazard: corridor deadlock

The floor plan is a corridor network — 881 nodes carry 952 edges, and 609 nodes have
degree exactly 2. The longest corridor is **17 hops**, and 39 corridors are longer
than 5 hops.

If `K_COMMIT` is shorter than a corridor, two robots can each reserve into opposite
ends of it with no interval conflict visible, then meet head-on with nowhere to pass.
Space-time planning "resolves" this by waiting mid-corridor, which achieves nothing.

Preventing it is conflict resolution and belongs to the layer that owns the priority
ordering — either by making `K_COMMIT` span the longest corridor, or by adding
corridor-level admission control. This package reports the corridor being entered and
any opposing peer in `Diagnostics` so that layer has what it needs; it does not act
on it.

## The peer plane

Reservations are exchanged **directly between bots**, not through the leader.
`PROTOCOL.md` §7 requires that "reservations, collision avoidance, energy shutoff
work without the leader", so carrying claims in the leader-mediated heartbeat would
have made collision avoidance depend on the one thing it must survive.

The leader loses nothing and gains a job: its roster already carries every
region-mate's `latest_node_id` and dialable `address`, which is exactly what a bot
needs to decide who to announce to, and its `yield_priority` is exactly the right of
way rule. It supplies **discovery and ranking, not permission** — once a bot has a
neighbour's address it talks to it directly, and leader death never interrupts that.

Three properties make an announce-only protocol correct rather than merely hopeful:

- **Announce, don't negotiate.** Both sides of a contest see both claims and apply
  the same total order (`yield_priority`, then `bot_id`), so they agree with no round
  trip. Asking permission needs a reply per claim and deadlocks when two bots ask at
  once.
- **Relative windows.** "I hold node 412 from +200 ms to +2400 ms", stamped by the
  receiver. No shared clock, no beacon, no drift correction.
- **A fresh claim is provisional.** It becomes effective one announce period later,
  by which time any rival claim has arrived and the loser has withdrawn. Without
  this, two bots claiming the same node in the same breath would both act before
  either could see the conflict.

A bot also waits for the *withdrawal to arrive* rather than acting the moment it
computes that it should win — "I outrank them" is not the same as "the node is
free", and the difference is a collision with a robot that has not heard yet.

Announcements go only to bots whose claims could actually meet ours, by an exact
test rather than a radius guess: a peer's claims lie within `K_COMMIT` hops of where
it stands, so it needs to hear from us only when one of the nodes *we hold* is that
close to it. At a fleet of twenty that is ~2.6 recipients per announce against 19
for a broadcast.

`proto/reservation.proto` is the wire schema, ready to fold into `fleet.proto`; the
virtual network needs one line (`caller_region == bot.region_id`, the same rule
`RegionService` already uses).

## What the simulation shows

```sh
uv run spore-planner-sim --robots 20 --ticks 2500
```

Twenty independent planners on the real map, each seeing only what a heartbeat would
carry. Across three seeds of 500 simulated seconds:

| | |
|---|---|
| **node conflicts** | **0** |
| **head-on swaps** | **0** |
| starved robots | 0 |
| corridor standoffs (counted) | 1–6 |
| contests lost (counted) | 139–344 |
| announcements | ~2.6 per bot per tick |

**Nothing in the simulation has a god's-eye view of who holds what.** Each robot
keeps its own ledger and learns only what its neighbours announce. That is what
makes zero conflicts worth having: it is agreement reached through announcements
between independent bots, not an allocator that could not have let a conflict
through.

Lost contests and standoffs are counted rather than treated as failures — deciding
who *should* have had priority in a corridor is the layer above this one. The
numbers are here to give that work something real to aim at.
