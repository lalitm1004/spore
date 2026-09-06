# TODO — use cases covered, holes open

Checked = implemented **and** covered by a test in `tests/` (in-process gRPC
unless marked *Docker* = real containers on a real network). Section numbers
refer to `PROTOCOL.md`.

## Bootstrap & discovery (§4.1–4.3)

- [x] Everyone boots as leader; same-region conflict → lower priority yields (both RPC and sender paths)
- [x] Late joiner discovers the running leader through `PEER_LEADERS`
- [x] Unhealthy bot never claims leadership at boot; probes peers until told who leads
- [x] Follower with no leader finds one via another follower's `redirect_to`
- [x] Three bots all in parking converge on one leader over real Docker DNS *(Docker)*
- [x] Two regions (parking + grid) each elect, and their leaders meet *(Docker)*

## Heartbeats & roster (§3.1)

- [x] Heartbeat returns full roster + other leaders + `leader_bot_id` + ledger replica
- [x] Non-leader answers with `redirect_to`; a bot never follows itself; redirect ping-pong stays at 1 Hz
- [x] Graceful `Departure` removes the bot immediately; silent bot evicted after `T_DEAD`
- [x] Cross-region heartbeat refused by the virtual network
- [x] Location trail (last 3 distinct nodes) carried and relayed; leaders exchange every bot's trail
- [x] `mission`, `fault`, `job_id`, `cargo_state`, `yield_priority` on every roster record
- [x] Roster swap is atomic on followers

## Election & priorities (§5)

- [x] Bully chain: challenger → outranked → higher peer runs its own election → Coordinator to all
- [x] Highest priority wins; FAULTED bot cannot win or tell others to stand down; departing bot declines
- [x] Leader dies → followers elect → converge (live threads, and *Docker* `kill` + restart)
- [x] Hung leader (`pause`) → followers elect; on resume the split brain heals by priority *(Docker)*
- [x] Partitioned follower self-elects alone, yields on reconnect *(Docker)*
- [x] Winner notifies a challenger it had never seen (challenger registration)
- [x] Stale Coordinator (from a bot we outrank) ignored by a healthy leader
- [x] Election priority: health dominates, battery buckets, id tiebreak, leader hysteresis
- [x] Tenure: leader rotates to the best free follower after `T_LEADER_TENURE`; not while it has a job
- [x] Succession prefers a free healthy follower over a busy higher-priority one
- [x] Job priority (charge first, leader last) and yield priority (free < to-pickup < carrying)
- [x] Unhealthy leader abdicates

## Migration (§4.6, §4.7, §8)

- [x] Full handshake through the `Migrator`; `MIGRATING_OUT` in a ledger, survives heartbeats
- [x] `MIGRATING` reported while in flight; migration to an empty region → solo leader
- [x] Waits for a settled leader; retries with backoff (reconcile loop)
- [x] Leader migrates by abdicating first — to a free follower
- [x] QR region change drives migration, in-process and over real containers *(Docker)*
- [x] Join without handoff refused; pending handoff expires

## Virtual network (§12)

- [x] Missing metadata → `UNAUTHENTICATED`; per-service admission table incl. Job/Bot/Admin services

## Jobs (§14)

- [x] Orders enter over `ControlPlaneService.DispatchOrder`, served by every bot.
  The control plane owns that proto and knows no regions or leaders; routing
  stays inside the fleet where it is the only place it can be right

- [x] Submit → best free follower (charge, then distance); robot commanded to pickup; **bot-0 assignable**
- [x] Busy / low-battery / faulted bots not free; leader is last resort and self-observes its own job
- [x] Routed from a follower to its leader; routed to the pickup node's region leader
- [x] Nobody free → forwarded to nearest region, which becomes owner; nobody anywhere → queued, retried, stale copy dropped on later forward
- [x] Full lifecycle PICKUP → EN_ROUTE → DROPOFF → DELIVERED → crossed off → bot free (in-process and *Docker*)
- [x] Pre-pickup failure → re-queued + reassigned; failed bot drops the job; post-pickup → `NEEDS_ATTENTION` + control plane
- [x] Lost heartbeat with a job → failure; migrating bot → not a failure; dropped job detected; stale failure report ignored
- [x] Delivery observed in another region → `JobEvent` → owner crosses off; **retried until an owner acks**
- [x] Ledger replicated to followers; new leader inherits it; only leaders assign; duplicate submit idempotent

## Orchestration (§13)

- [x] `up.py` on the Docker SDK: labels, per-network isolation, `PEER_LEADERS`, unique ids, map bind-mount, `ADMIN_ENABLED`
- [x] `down.py` removes only fleet containers, then the network
- [x] `AdminService`: `GetState` only, read-only (gated by `ADMIN_ENABLED`). The injection RPCs are gone -- a robot reports over its own stream

## Reservations (§15)

- [x] Ordering: cargo beats free, lower id breaks the tie, and both sides reach the same verdict
- [x] Windows travel as offsets and are stamped on arrival — no shared clock anywhere
- [x] A fresh claim is provisional for one `T_ANNOUNCE`; carrying on holding a node does not restart it
- [x] A claim is refused when a better-ranked neighbour holds it, granted over one we outrank, and never half-granted
- [x] The winner waits for the loser's retraction rather than acting on the verdict
- [x] Two bots claiming at once settle with no round trip; exactly one gives way
- [x] An empty announcement is a withdrawal; a quiet neighbour lapses after `RESERVATION_TTL`
- [x] A late announcement is ignored (`seq`), and forgetting a peer forgets its sequence
- [x] Only bots within `RESERVATION_REACH_HOPS` of a node we hold are told; no map → tell everyone
- [x] Cross-region and unauthenticated announcements refused by the virtual network
- [x] The run-loop step claims the node underfoot and announces it; no QR scan → no claim
- [x] A claim crosses a real network into a peer's ledger *(Docker)*
- [x] Claims keep flowing when the leader is killed — the §7 property *(Docker)*
- [x] A paused bot's claims lapse and stop blocking *(Docker)*
- [x] Two bots injected onto one node settle on a single holder *(Docker)*
- [ ] Claiming a **route** rather than the node underfoot. Now possible: the bot
  knows where the robot is and where it told it to go
- [x] `may_enter` gates actual movement: a robot is answered `WAIT` and holds,
  and the firmware honours the hold rather than driving on when it times out

## Pathfinding (§16)

- [x] Safe-interval search over (node, heading, interval), with turns and charge priced
- [x] Three traffic tiers with strict precedence: a prediction never contradicts a declaration
- [x] Prediction runs only to the first junction, where the peer regains a choice
- [x] Yield when the wait is long, we lose the priority comparison, and there is somewhere to go
- [x] When we *win* the comparison we hold instead — both sides reach the same verdict alone
- [x] Yield cascade: YI bay, else junction, else a PK/CH spur (logged)
- [x] Alternative routes kept as diffs; promoted without a full replan when the primary dies
- [x] Distance caches bounded — an unbounded one reached ~33 MB on the real map
- [x] Cross-region routes planned optimistically; replanned on arrival
- [ ] The first decision after migrating is uninformed — accepted, not solved
- [~] Route alternates are built but deliberately unwired — see F1 below

## Routing (§16) — the robot link

- [x] The companion's question is answered over `RobotNetwork.Session`, and the same stream carries the position that answer is planned from
- [x] `Decision` carries a kind, so WAIT is sayable rather than implied by silence
- [x] Every query gets an answer — bad JSON, a planner error and a map disagreement all reply
- [x] Jobs set a navigation goal; the robot is driven a node at a time, not sent a destination
- [x] Run in the Webots fleet: ten robots, ten bots, real QR reads reaching the
      roster and real claims between them. Not yet *timed* — the host is arm64
      and the Webots image is amd64, so the numbers would be emulation's

The stand-in router and the placeholder glue project are both gone: each robot
container runs a real `bot.py`, and `spore-amr/webots/robot/network.py` is now
only the shared message shapes. There is one network layer.

---

## Scenario coverage (docs/scenarios.md)

55 scenarios on real containers, ids matching test names. A guard test
(`tests/test_scenarios_doc.py`) fails if a documented scenario has no test or a
test has no row, so the contract cannot quietly stop being true.

- [x] **A1-A10** decisions — never silence, the four kinds, malformed input, no map, one connection a shift
- [x] **B1-B12** job distribution — nearest, charge over distance, exclusions, leader last, idempotence, forwarding, queueing, full lifecycle
- [x] **C1-C8** planning — a job becomes turns, the goal moves at pickup, claims and trails respected, latency, unreachable
- [x] **D1-D8** exceptions — the three stall rungs, faults either side of pickup, companion reconnect, dead bot, dead leader
- [x] **E1-E6** collisions — two and three on a node, following, claims crossing, and the no-overlap invariant on its own
- [x] **F2, F3, F4, F6** redirections — obstruction avoided and cleared, migration replan, a peer's claim changing the answer
- [x] **G1, G2, G7** yielding — carrying beats free, the id tiebreak, exactly one side gives way
- [~] **F1** promoting a cached alternate — **not applicable as the fleet works today.** The cache saves a search only when losing a route is expensive; the robot asks at every node and a plan costs under a millisecond, so `bot._route` replans afresh each time and an alternative in hand would save nothing. `planning/routes.py` is kept, unwired, for a model that commits to longer routes
- [ ] **F5** REROUTE on a genuine route change — A4 covers the negative (unchanged means PROCEED); the positive needs a route that actually moves
- [ ] **G3-G6** the yield-spot cascade on containers — unit-tested; pinning which spot is chosen would be asserting the map rather than the rule
- [ ] **E3** two bots driving head-on down a whole corridor — the contest itself is covered by E4, G1 and G7
- [x] Obstructions come from what a robot reported: `Fault.warning.obstacle`
      carries the node it was seen at, and the bot blocks the lane it last sent
      that robot down. `InjectObstruction` is gone

---

## Cleanliness pass (September 2026)

- [x] Every timeout, sleep and hold reads from `config.py`; a test fails on a bare number outside it
- [x] One implementation per helper within a project (BFS, claim window, `wait_until`, replay loaders); cross-project twins name each other
- [x] `PROTOCOL.md` §11 embeds each proto verbatim, guarded; the hand-written mirrors are pinned to the wire
- [x] The container tier is a package, one file per scenario letter, parallel by default, and sweeps what an interrupted run left behind
- [x] Orders can be placed on the live simulator fleet (`fleet.sh order`, or `fleet.sh orders N`
      for random fake demand), and a simulator scenario proves one is taken and driven
- [ ] `PeerTable.region_locations()` is collected on every leader beat and read by nobody -- a design question, not a bug
