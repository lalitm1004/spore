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
- [x] `AdminService`: `GetState`, `InjectRobotState` (gated by `ADMIN_ENABLED`)

---