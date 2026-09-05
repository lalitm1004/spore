# AMR Fleet — Network Layer Protocol Spec

Covers the full gRPC communication design: topology, message schemas, RPC contracts, lifecycle flows, election algorithm, migration handshake, edge cases, and resolved decisions.

---

## 0. Code map — where each part of this spec lives

| Concern | Module | Notes |
|---|---|---|
| Identity, timing constants | `config.py` | Everything read from env; derived timeouts in multiples of `T_HB` |
| The bot process, run loop, role transitions, wire payloads | `bot.py` | The "brain": robot state in → decisions → heartbeats out |
| Follower ↔ leader heartbeats, redirect, departure, migration approval | `bus/heartbeat.py` | `HeartbeatSender` (follower thread) + `RegionServicer` (leader handlers) |
| Leader ↔ leader heartbeats, conflict detection, handoff | `bus/leader_exchange.py` | Bootstrap discovery and split-brain resolution happen here |
| Migration state machine + destination join | `bus/migration.py` | `Migrator` (bot side, retries, reconcile) + `MigrationJoinServicer` |
| Jobs: ledger, dispatch, forwarding, observation | `bus/jobs.py` | `Dispatcher` (leader side) + `JobServicer` / `BotServicer`; §14 |
| Warehouse graph: node → region, hop distance | `warehouse/map.py` | Loaded from `warehouse-layout.json`; §14 |
| Pathfinding: the search, traffic, routes, and the robot link | `planning/` | `sipp.py` + `cost.py` + `traffic.py` (the search), `decide.py` + `server.py` (the robot); §16 |
| Reservations: claims, who gives way, who to tell | `reservations/` | `claims.py` + `ledger.py` + `vicinity.py` (rules), `sender.py` + `server.py` (wire); §15 |
| Virtual network (who may call what) | `bus/policy.py` | gRPC server interceptor; §12 |
| Admin: introspection + robot-state injection | `bus/admin.py` | `ADMIN_ENABLED` only; used by the Docker harness; §13 |
| Persistent gRPC channels | `bus/rpc.py` | One channel per peer, low reconnect backoff |
| Bully election + abdication | `election/bully.py` | §5 |
| Priority formula + hysteresis | `election/priority.py` | §5.6 |
| ElectionService handlers | `election/server.py` | Thin adapter |
| Roster, other-region leaders, migration ledgers | `peers/table.py` | `PeerTable`, `Ledger` |
| Wire schema | `proto/fleet.proto` | §11; regenerate with `grpc_tools.protoc` |
| Local orchestration | `up.py`, `down.py` | §13 |
| Tests | `tests/` | `test_unit.py` (rules), `test_protocol.py` + `test_jobs.py` (live in-process fleets), `test_docker.py` (real containers) |

Every module's docstring explains *what* it is, *where* it sits, *why* it exists and *how* it works; this document is the cross-cutting view.

---

## 1. Topology

Star topology per region. All normal-operation traffic flows through the region leader. Bots never talk directly to each other except during leader election.

```
REGION 14 (parking)                   REGION 2 (grid_field)
┌──────────────────────────┐          ┌──────────────────────────┐
│  bot-1 ──┐               │          │               ┌── bot-5  │
│  bot-2 ──┼── leader-14 ◄─┼──────────┼──► leader-2 ──┼── bot-6  │
│  bot-3 ──┘               │          │               └── bot-7  │
└──────────────────────────┘          └──────────────────────────┘
```

Three communication paths, all gRPC unary RPCs over protobuf:

| Path | Direction | When |
|---|---|---|
| Bot → own leader | Every `T_HB` interval | Normal operation |
| Leader ↔ leader | Every `T_LEADER_HB` interval | Always (cross-region awareness) |
| Bot → peer bot | Only during election | Only when leader is dead |

No NATS, no message broker, no pub/sub. Single protocol, single library.

---

## 2. What a bot knows at birth

Injected as environment variables by the orchestrator:

| Env var | Example | Purpose |
|---|---|---|
| `BOT_ID` | `2` | This bot's unique identity (integer) |
| `REGION_ID` | `14` | Initial region (from first QR scan or orchestrator assignment) |
| `PEER_LEADERS` | `bot-0:50051,bot-1:50051,bot-3:50051` | Addresses of all other bots for bootstrap discovery |

Everything else — the region roster, who the real leader is, other regions' leaders — is learned dynamically through the leader exchange and heartbeat ack payloads.

### Location from QR codes

Bots determine their physical location by scanning QR codes embedded in the warehouse floor. Each QR code encodes a `Node`:

```json
{
  "id": 42,
  "name": "PK-A-03",
  "region_id": 14,
  "node_type": "PK",
  "position": { "x": 150.0, "y": 30.0 }
}
```

Node types: `PT` (pass-through), `TR` (transfer/pickup/dropoff), `CH` (charging), `PK` (parking), `YI` (yield).

The `region_id` from the QR code is the ground truth for which region a bot belongs to. When a bot scans a QR code with a different `region_id` than its current one, it triggers automatic migration.

**This is the fleet's location interface, and everything else is downstream of
it.** A scan reaches the network layer as a `RobotToNetwork` on the robot link
(§16.1), becomes `latest_node_id` and extends `node_trail`, and goes out in
every heartbeat — so the leader's roster, and through it every bot in the
region, is built on what the robots actually read off the floor.

That path is worth reading once in full, because when it is broken nothing
raises: node `0` is a legal-looking node, so a fleet with no position keeps
running and keeps being wrong. Reservations quietly hold nothing, "nearest free
bot" quietly becomes priority order, migration quietly never fires.
[`docs/location.md`](docs/location.md) has the whole path and the table of what
each failure looks like.

---

## 3. Services and RPCs

Eight gRPC services, split by communication pair. The four below are the
membership protocol; `JobService` and `BotService` (jobs) are in §14,
`ReservationService` (bot ↔ bot, no leader) in §15, and `AdminService`
(tooling) in §13. The complete schema is §11.

### 3.1 RegionService — bot ↔ own leader

The primary communication channel. Bots call the leader; the leader responds with state.

```protobuf
service RegionService {
  rpc Heartbeat(HeartbeatRequest)          returns (HeartbeatAck);
  rpc Departure(DepartureRequest)          returns (DepartureAck);
  rpc MigrationRequest(MigrationReq)       returns (MigrationReqAck);
}
```

#### Heartbeat / HeartbeatAck

The workhorse. Bot sends its state every `T_HB`. Leader responds with the full region picture.

**Request (bot → leader):**

| Field | Type | Notes |
|---|---|---|
| `bot_id` | int32 | Who's sending |
| `region_id` | int32 | Which region |
| `latest_node_id` | int32 | Last QR node scanned (ground-truth location), as reported by the robot on the link in §16.1 — see §2 |
| `node_trail` | repeated int32 | The last `NODE_TRAIL_LEN` (3) distinct nodes, newest first; `node_trail[0] == latest_node_id`. Two points give a heading, so every bot can see which way its region-mates are moving |
| `state` | string | FSM state: IDLE, MOVING, FAULTED, COMMS_LONG, etc. |
| `battery` | float | Current battery percentage (0–100) |
| `priority` | int32 | Election priority (higher wins) |
| `address` | string | This bot's dialable `host:port` |
| `mission` | string | Current mission type: PARK, CHARGE, HOLD, IDLE, CARGO |
| `fault` | string | Fault info if any (warning/error type), empty if healthy |
| `timestamp` | int64 | Unix epoch millis |

**Response (leader → bot):**

| Field | Type | Notes |
|---|---|---|
| `region_peers` | repeated PeerRecord | Every bot the leader knows about in this region |
| `other_leaders` | repeated LeaderRecord | Leaders of all other known regions |
| `redirect_to` | string | If this bot is NOT the leader, the current leader's address. Empty if this bot IS the leader. |
| `leader_bot_id` | int32 | The leader's id — the responder itself when it leads, or the bot it is redirecting to. Lets a bot that finds its leader by probing learn *who* leads, not just where. |

Where:

```
PeerRecord {
  bot_id:    int32
  address:   string   // dialable host:port (for election)
  priority:  int32
  state:     string
  battery:   float
  latest_node_id: int32
  node_trail: repeated int32   // newest first, ≤ NODE_TRAIL_LEN
}

LeaderRecord {
  region_id: int32
  bot_id:    int32
  address:   string
}
```

After a single heartbeat/ack cycle, every bot holds the full region roster (with addresses for election) and every other region's leader identity.

**Redirect behaviour:** If a bot sends a Heartbeat to a bot that is NOT the current leader, the receiver responds with `redirect_to` set to the current leader's address (if known). The sender retargets its running heartbeat loop (no restart — a redirect ping-pong must stay at 1 Hz, never RPC speed). **A bot never follows itself:** a redirect to its own address means the chain has come back around and *it* is the leader, so it promotes itself. Without this rule two bots that follow each other spin forever (seen on real containers after a paused leader resumed).

#### Departure / DepartureAck

Graceful shutdown. Bot calls this before exiting so the leader removes it from the roster immediately rather than waiting for missed-heartbeat timeout.

**Request:** `bot_id`, `timestamp`.
**Response:** empty ack.

If the bot dies without sending Departure (hard kill, power loss), the leader detects absence via missed heartbeats after `T_DEAD`.

#### MigrationRequest / MigrationReqAck

Bot tells its own leader "I want to move to region X" (triggered by scanning a QR code with a new `region_id`).

**Request:** `bot_id`, `destination_region_id`, `timestamp`.
**Response:** `approved` (bool), `destination_leader` (LeaderRecord — address of the destination region's leader).

The leader may deny (region too small, bot has active job). If approved, the leader marks the bot as `MIGRATING_OUT`.

---

### 3.2 LeaderExchangeService — leader ↔ leader

Cross-region awareness. Leaders heartbeat each other directly.

```protobuf
service LeaderExchangeService {
  rpc LeaderHeartbeat(LeaderHBRequest)        returns (LeaderHBAck);
  rpc MigrationHandoff(MigrationHandoffReq)   returns (MigrationHandoffAck);
}
```

#### LeaderHeartbeat / LeaderHBAck

Leaders periodically exchange region summaries.

**Request:**

| Field | Type | Notes |
|---|---|---|
| `region_id` | int32 | Sender's region |
| `leader_bot_id` | int32 | Sender's identity |
| `address` | string | Sender's dialable address |
| `bot_count` | int32 | How many bots in the region |
| `avg_battery` | float | Region average battery |
| `active_jobs` | int32 | Currently running jobs |
| `priority` | int32 | Sender's election priority — used to resolve same-region leader conflicts (§5.4) |
| `locations` | repeated BotLocation | `{bot_id, node_trail}` for every bot in the sender's region, leader included. Every leader therefore holds a fleet-wide picture of recent movement (kept in `PeerTable.region_locations`) — the input a dispatcher needs to find the nearest free bot in another region |
| `timestamp` | int64 | Unix epoch millis |

**Response:** mirrors the same fields (mutual exchange in one round trip).

**Same-region conflict detection:** If a leader receives a LeaderHeartbeat from another bot claiming the **same `region_id`**, this means two leaders exist for one region (bootstrap or split-brain). The lower-priority bot abdicates — see §5.3.

#### MigrationHandoff / MigrationHandoffAck

Source leader tells destination leader "expect this bot."

**Request:** `bot_id`, `source_region_id`, `bot_priority`, `bot_address`.
**Response:** `accepted` (bool).

If accepted, the destination leader creates a `PENDING_INCOMING` entry. If denied (at capacity), the source leader tells the bot migration was rejected.

---

### 3.3 ElectionService — bot ↔ peer bot

Point-to-point, used only when the leader is dead and bots fall back to their cached roster.

```protobuf
service ElectionService {
  rpc Elect(ElectRequest)              returns (ElectResponse);
  rpc Coordinator(CoordinatorRequest)  returns (CoordinatorResponse);
}
```

#### Elect / ElectResponse

Bully election challenge. A candidate calls this on every peer it believes might outrank it.

**Request:** `bot_id`, `priority`, `timestamp`, `address`.
**Response:** `ack` (bool) — `true` means "I outrank you, stand down."

**Challenger registration.** The receiver adds the challenger (with the
`address` it sent) to its roster *before* deciding. A challenger is by
definition alive and in-region; if the receiver goes on to win, its
`Coordinator` must reach the challenger even when the challenger was missing
from the receiver's last ack (e.g. the leader died right after the
challenger joined). Without this rule a region can elect a leader that never
tells the bot which triggered the election.

#### Coordinator / CoordinatorResponse

Victory announcement OR abdication handoff. The winner calls this on every peer to declare itself leader.

**Request:** `bot_id`, `priority`, `address` (the new leader's address).
**Response:** empty ack.

Also used for **leader abdication**: a leader stepping down calls Coordinator on the highest-priority peer to hand off leadership.

---

### 3.4 MigrationJoinService — bot → destination leader

Used only during migration, when a bot contacts a foreign region's leader.

```protobuf
service MigrationJoinService {
  rpc MigrationJoin(MigrationJoinReq)   returns (MigrationJoinAck);
}
```

#### MigrationJoin / MigrationJoinAck

The migrating bot introduces itself to the destination leader.

**Request:** `bot_id`, `source_region_id`, `priority`, `address`, `battery`, `state`, `latest_node_id`.
**Response:** `accepted` (bool), `region_peers` (new region's roster), `other_leaders` (updated leader list).

---

## 4. Lifecycle Flows

### 4.1 Cold start — "everyone starts as leader"

All bots boot simultaneously (e.g., from parking). There is no designated first leader.

```
1. All bots boot, scan parking QR → region_id: 14
2. All bots start ALL gRPC servers:
   - RegionService (accept heartbeats)
   - ElectionService (accept election RPCs)
   - LeaderExchangeService (accept leader heartbeats)
   - MigrationJoinService (accept migration joins)
3. All **healthy** bots self-declare as leader of region 14. A bot that boots
   FAULTED / COMMS_LONG never claims leadership: it starts as a leaderless
   follower and heartbeats `PEER_LEADERS` one by one until a leader answers
   with a roster or a follower answers with `redirect_to` (§3.1)
4. All bots begin sending LeaderHeartbeats to addresses in PEER_LEADERS
5. Each bot receives LeaderHeartbeats from other bots claiming region 14
6. Same-region conflict detected → bully election triggers
7. Highest-priority bot wins the election
8. Winner: continues serving all servers, begins accepting follower heartbeats
9. Losers: stop serving RegionService, start heartbeating the winner on T_HB
10. If bots in different regions exist, their leaders continue
    LeaderExchangeService heartbeats with each other
```

**Why this works:** By having every bot start as a leader, we reuse the LeaderExchangeService mesh for bootstrap discovery. No single bot is a hard dependency. Bots can start in any order.

### 4.2 Solo bot in a new region

A bot physically moves to region 2 (grid field) where nobody else is.

```
1. Bot scans QR code → region_id changes from 14 to 2
2. Bot initiates migration (see §4.6)
3. After migration completes, bot is alone in region 2
4. Bot self-declares as region-2 leader (no conflict, no election needed)
5. Bot starts LeaderExchangeService heartbeats to other known leaders
6. If another bot later migrates to region 2, it joins as a follower
```

### 4.3 Normal join (fleet running, leader alive)

A new bot starts after the fleet is already running.

```
1. Bot boots, reads env (BOT_ID, REGION_ID, PEER_LEADERS)
2. Bot starts all gRPC servers, self-declares as leader
3. Bot sends LeaderHeartbeats to PEER_LEADERS addresses
4. Bot receives a LeaderHeartbeat from an existing leader of the same region
5. Same-region conflict → election runs
6. Existing leader (likely higher priority) wins
7. New bot becomes follower, starts heartbeating the leader
8. Leader's next HeartbeatAck to all bots includes the new bot
```

### 4.4 Graceful departure

```
1. Bot receives SIGTERM
2. Bot calls Departure on its leader
3. Leader removes bot from roster immediately
4. Leader's next HeartbeatAck to all bots excludes the departed bot
5. Bot exits
```

### 4.5 Hard death (container killed, power loss)

```
1. Bot stops heartbeating (no Departure call)
2. Leader notices: no heartbeat for T_DEAD
3. Leader removes bot from roster
4. Other bots learn on their next HeartbeatAck cycle
```

### 4.6 Migration — full handshake

Triggered automatically when a bot scans a QR code with a different `region_id`.

```
1. bot-5 (region 14) scans QR → region_id: 2
2. bot-5 calls MigrationRequest on region-14's leader
   - Leader checks: can bot-5 leave? (no active job, region not too small)
   - Leader responds: approved=true, destination_leader={region-2 leader info}
   - Leader marks bot-5 as MIGRATING_OUT in roster

3. Region-14's leader calls MigrationHandoff on region-2's leader
   - "Expect bot-5, here's its info"
   - Region-2's leader responds accepted=true
   - Region-2's leader marks bot-5 as PENDING_INCOMING

4. bot-5 calls MigrationJoin on region-2's leader
   - Region-2's leader recognizes bot-5 (from the handoff)
   - Responds with region-2's roster and leader list
   - Region-2's leader adds bot-5 to its roster

5. bot-5 confirms: sends one successful Heartbeat to region-2's leader
   and receives a HeartbeatAck

6. bot-5 calls Departure on region-14's leader
   - Region-14's leader removes bot-5 (MIGRATING_OUT → gone)

7. bot-5 is now a region-2 member
```

**Critical rule:** bot-5 keeps heartbeating region-14's leader throughout steps 2–5. It only stops at step 6 after confirming connectivity with region 2. If anything fails, bot-5 reverts to region-14 membership.

**Migration is a state, not an event.** The bot side is a state machine (`bus/migration.py: Migrator`):

```
IDLE ─┬─▶ REQUESTING ─▶ JOINING ─▶ DEPARTING ─▶ IDLE      (success)
      │        │            │
      └────────┴────────────┴──▶ FAILED ─(backoff)─▶ IDLE
```

- **Reconciliation, not triggering.** The run loop only *records* the region
  of the last QR scan as `desired_region_id`. Every tick the `Migrator`
  compares it with the region the bot is a *member* of and starts an attempt
  if they differ and nothing is in flight. A parked bot that stops emitting
  updates still converges; a failed attempt is simply retried.
- **Backoff.** Failed attempts wait 1 s, 2 s, 4 s … (capped at
  `T_MIGRATION_BACKOFF_MAX`) before the next.
- **Asynchronous handoff.** The source leader records the bot in its
  `migrating_out` ledger and replies to `MigrationRequest` *immediately*;
  the leader→leader `MigrationHandoff` (step 3) runs on a background thread so
  no gRPC worker ever blocks on a cross-region call. The bot's `MigrationJoin`
  may therefore arrive before the handoff has landed: it is refused by the
  destination's virtual network (`PERMISSION_DENIED`) and the bot retries
  every 0.5 s until `T_MIGRATION_TIMEOUT`.
- **Settled leader first.** An attempt only starts once `leader_settled()` is
  true (§5.7). Nobody leaves a region mid-election.
- **Visible in flight.** While migrating the bot reports state `MIGRATING`
  in its own heartbeats, and the source leader shows it as `MIGRATING_OUT` in
  every ack — the latter from the ledger, never from the roster record, which
  the bot's next heartbeat would overwrite.
- **Departure before identity switch.** `Departure` to the old leader is sent
  while the bot still carries its old `region_id`, otherwise the old leader's
  virtual network would refuse it (§12).

**If destination region has no leader (empty region):** bot-5 skips steps 3–5. After getting approval from its current leader, bot-5 migrates, self-declares as the new region's leader, and sends Departure to the old leader.

### 4.7 Leader migration — abdicate-then-migrate

When the leader itself scans a QR code with a different `region_id`:

```
1. Leader of region 14 scans QR → region_id: 2
2. Leader picks highest-priority peer from its roster
3. Leader calls Coordinator on that peer:
   "You are the new leader of region 14"
4. New leader starts accepting heartbeats
5. Old leader initiates MigrationRequest (now as a regular bot,
   talking to the new leader)
6. Normal migration handshake proceeds (§4.6 steps 2–7)
```

**If the leader is the only bot in the region:** No one to abdicate to. Leader simply migrates — sends no Coordinator (empty region needs no leader), proceeds directly to the destination region.

### 4.8 Leader death and succession

```
1. Leader dies
2. All bots stop receiving HeartbeatAcks
3. Each bot waits T_LEADER_DEAD (missed acks)
4. Each bot consults its cached region_peers list
5. Bully election begins (see §5 for full algorithm)
6. Highest-priority reachable bot wins
7. Winner calls Coordinator on every peer with its address
8. All bots start heartbeating the new leader
9. New leader has other_leaders cached — immediately begins
   leader-to-leader heartbeats with other regions
```

**Cascading failure:** If the top candidate is also unreachable, the next bot in priority order tries after `T_ELECT_TIMEOUT`. Worst case: `dead_candidates × T_ELECT_TIMEOUT` before a leader is established.

### 4.9 Migration failure — destination leader dies mid-handshake

```
1. bot-5 has MigrationRequest approved (step 2 complete)
2. Source leader called MigrationHandoff (step 3 complete)
3. Destination leader dies before bot-5 calls MigrationJoin

4. bot-5 tries MigrationJoin — connection refused
5. bot-5 is still heartbeating source leader (never stopped)
6. bot-5 reports migration failure to source leader
7. Source leader clears MIGRATING_OUT flag
8. bot-5 returns to normal membership

9. bot-5 can retry after destination region elects a new leader
   (learns new leader address from source leader's HeartbeatAck)
```

### 4.10 Migration + source leader death

```
1. bot-5 is mid-migration, source leader dies
2. Source region elects a new leader
3. New leader's roster (from cached peers) still has bot-5
   but doesn't know about MIGRATING_OUT flag (that was on the dead leader)

4a. If migration succeeds:
    - bot-5 completes MigrationJoin with destination
    - bot-5 sends explicit Departure to source region's NEW leader
    - New leader removes bot-5

4b. If migration fails:
    - bot-5 reverts to source region
    - bot-5 heartbeats new leader (discovered via PEER_LEADERS or redirect)
    - New leader adds bot-5 back to roster normally
```

---

## 5. Bully Election — Full Algorithm

### 5.1 Preconditions

- The leader has been unreachable for `T_LEADER_DEAD`, OR
- A same-region LeaderHeartbeat conflict was detected (bootstrap / split-brain)
- Bot's FSM state is NOT `FAULTED` or `COMMS_LONG` — such a bot is *ineligible*:
  it never starts an election, always answers `Elect` with `ack=false`, and
  always yields a same-region conflict (§5.4) regardless of priority
- Bot has NOT initiated Departure (same three consequences)
- Bot has a cached `region_peers` list (from last HeartbeatAck or LeaderExchange)

### 5.2 Procedure

```
state:
    self.priority       # this bot's election priority
    self.bot_id         # this bot's ID
    self.address        # this bot's gRPC address
    known_leader = None
    cached_roster = []  # from last HeartbeatAck
    election_in_progress = False
    departing = False

on leader unreachable for T_LEADER_DEAD:
    if self.state in {FAULTED, COMMS_LONG}:
        return
    if self.departing:
        return
    start_election()

on same-region LeaderHeartbeat conflict:
    if (other.priority, other.bot_id) > (self.priority, self.bot_id):
        # other bot outranks us — abdicate, become follower
        known_leader = other.bot_id
        become_follower(other.address)
        return
    else:
        # we outrank them — they should abdicate
        # do nothing; they will detect the same conflict and yield
        return

def start_election():
    known_leader = None
    election_in_progress = True

    # Call Elect on every peer with higher priority
    higher_peers = cached_roster.filter(
        p => (p.priority, p.bot_id) > (self.priority, self.bot_id)
    )

    any_outranks = False
    for peer in higher_peers:
        try:
            resp = call_elect(peer.address, self.priority, self.bot_id)
            if resp.ack:
                any_outranks = True
                break  # someone higher is alive, stand down
        except ConnectionError:
            continue  # unreachable, skip

    if any_outranks:
        # Wait for that higher peer to declare Coordinator
        wait T_ELECT_TIMEOUT
        if no Coordinator received:
            # They never declared — restart election
            start_election()
        return

    # Nobody outranked us — declare victory
    known_leader = self.bot_id
    election_in_progress = False
    for peer in cached_roster:
        call_coordinator(peer.address, self.bot_id, self.priority, self.address)

on receive Elect(candidate_priority, candidate_id, candidate_address):
    cached_roster.ensure(candidate_id, candidate_address, candidate_priority)
        # the candidate is alive and in-region: our Coordinator must reach it
    if self.departing or self.state in {FAULTED, COMMS_LONG}:
        return ElectResponse(ack=False)  # ineligible, can't lead

    if (self.priority, self.bot_id) > (candidate_priority, candidate_id):
        # We outrank the candidate — tell them to stand down
        # Also start our own election if not already running
        if not election_in_progress:
            start_election()
        return ElectResponse(ack=True)
    else:
        return ElectResponse(ack=False)  # candidate outranks us

on receive Coordinator(leader_id, leader_priority, leader_address):
    known_leader = leader_id
    election_in_progress = False
    become_follower(leader_address)
```

### 5.3 Priority comparison

**Primary sort:** `priority` field (higher wins).
**Tiebreak:** `bot_id` (higher wins — deterministic, stable).

Comparison: `(priority_a, bot_id_a) > (priority_b, bot_id_b)` using tuple ordering.

Priority is carried on every heartbeat so the leader's roster always reflects current values.

### 5.4 Split-brain resolution

If a network partition splits a region into two groups, each group independently elects a leader. When the partition heals:

1. Both leaders resume LeaderExchangeService heartbeats
2. Each leader receives a LeaderHeartbeat from the other with the **same `region_id`**
3. Exactly one yields — an **unhealthy** leader always, otherwise the one with
   lower `(priority, bot_id)`. Both sides run the same rule on their own copy,
   so no extra message is needed. The yielding leader abdicates:
   - Calls Coordinator on the higher-priority leader
   - Stops serving RegionService
   - Becomes a follower of the higher-priority leader
4. Followers of the abdicating leader detect missed HeartbeatAcks
5. Those followers learn the new leader address from the redirect mechanism or from the Coordinator broadcast

No epoch/term counter needed — priority comparison is sufficient since priorities are stable.

### 5.6 Priority — what the number means

Recomputed every run-loop tick (`election/priority.py`) and carried in every
heartbeat and leader heartbeat. Compared with plain `>`.

```
priority = healthy·10000 + battery_bucket·100 + bot_id
```

| Term | Range | Why |
|---|---|---|
| `healthy` | 0 / 1 | Dominates everything: an unhealthy bot loses to *any* healthy one |
| `battery_bucket` | 0–4 (20-point steps) | Prefer charged bots; buckets so 61% vs 59% does not reorder every reading |
| `bot_id` | < 100 | Deterministic tiebreak (`up.py` allocates ids sequentially) |

**Hysteresis (no flapping).** A *sitting leader* adds 5 points of slack
before bucketing: at 59% it still advertises bucket 3 until it drops below
55%, while a challenger at 59% advertises bucket 2. Leadership therefore only
moves after a real swing and never oscillates at a bucket boundary — and the
comparison stays a plain `>`, which the bootstrap collapse in §4.1 relies on.

**Starvation.** A leader whose battery drains loses a bucket and hands over
to a charged bot by the normal conflict/election rules, then goes to charge.

**Tenure.** After `T_LEADER_TENURE` (30 min) a leader with no job of its own
hands off to the best free successor, whatever its own priority — leading is
a shift, not a life sentence. It may win the next real election; that is fine.

**Succession** (who a leader hands off to on migration, fault, tenure or
shutdown): a healthy bot, preferably one with **no job** (a busy successor
would drive away and hand off again), then by election priority. Never an
unhealthy one.

**The other two priorities** (`election/priority.py`):

| Priority | Question | Rule |
|---|---|---|
| job | who gets the next job | charge bucket first, then raw %, then map distance; the leader last by a dominating penalty; busy/unhealthy bots never |
| yield | right of way at YI nodes | free `0` < heading to pickup `1` < carrying cargo `2`; computed by the leader, distributed in every `PeerRecord.yield_priority`, enforced by the robot layer |

### 5.7 Settled leader

`leader_settled()` is true when:

- **follower:** we have a leader address *and* received an ack within `T_DEAD`;
- **leader:** we have held the role for `T_SETTLE` (= 2 × `T_LEADER_HB`) with
  no election in progress — long enough for the bootstrap conflicts of §4.1
  to have resolved.

Migration (§4.6) and graceful departure (§4.4) both wait for it, so a region
is never left mid-election.

### 5.5a Stale Coordinator

A `Coordinator` can arrive late: while a leader was paused or partitioned the
others elected someone, and the message waited in its socket buffer. A
**healthy leader that outranks the named bot ignores it** — after a comeback
the conflict rule (§5.4), not an old election result, decides, and the leader
exchange makes the other side yield within one round. A Coordinator from a
bot that outranks us is a real result and is obeyed.

### 5.5 Election-during-election

If a bot is mid-election and receives an Elect RPC from a higher-priority peer:
- It abandons its own election attempt
- Responds with `ack=False` (acknowledging the higher peer should proceed)
- Waits for that peer's Coordinator announcement

---

## 6. Bot States During Communication Events

| Event | Bot state | Communication effect |
|---|---|---|
| Normal operation | IDLE / MOVING | Heartbeating leader on `T_HB` |
| Leader dies | (unchanged) | Stops receiving acks → election after `T_LEADER_DEAD` |
| Election in progress | (unchanged) | Calling Elect/Coordinator on cached peers |
| New leader elected | (unchanged) | Switches heartbeat target to new leader |
| Bot enters FAULTED | FAULTED | Continues heartbeating (leader sees state), cannot participate in election |
| Bot enters COMMS_LONG | COMMS_LONG | Heartbeats failing, cannot participate in election |
| Bot rejoining after partition | REJOINING | Retries heartbeat to last-known leader; if stale, follows `redirect_to` or falls back to PEER_LEADERS |
| Migrating | MIGRATING (phases REQUESTING → JOINING → DEPARTING) | Dual-homing: heartbeating old leader AND contacting new leader; old leader shows it as MIGRATING_OUT |
| Leader enters FAULTED | FAULTED | Abdicates: Coordinator call to highest-priority peer |
| Leader migrating | MIGRATING | Abdicates first, then migrates as regular bot |
| Bot departing | DEPARTING | Sends Departure, declines Elect RPCs |

---

## 7. Leader Responsibilities

### Must do

- Accept heartbeats from every bot in the region
- Maintain the authoritative roster (alive bots, their state, priority, address, location)
- Return full roster + other_leaders + redirect_to in every HeartbeatAck
- Heartbeat other regions' leaders on `T_LEADER_HB`
- Coordinate migration handoffs with other leaders
- Remove bots on Departure or missed-heartbeat timeout (`T_DEAD`)
- Detect same-region leader conflicts and resolve via priority comparison
- Abdicate before migrating to another region

### Must NOT do

> Why there is no central service at all, and what that costs:
> **[`docs/boundary.md`](docs/boundary.md)**.

- Hold the only copy of any state — every bot caches the roster independently
- Be required for safety-critical paths — reservations, collision avoidance, energy shutoff work without the leader
- Accumulate history — no log, no journal, only "who's alive right now"

---

## 8. Timeout & Migration Ownership

Three parties are involved in a migration. Each enforces its own timeout independently:

| Party | Timeout | Action on expiry |
|---|---|---|
| Bot (migrating) | `T_MIGRATION_TIMEOUT` | Abort migration, revert to source region |
| Source leader | `T_MIGRATION_TIMEOUT` | Clear `MIGRATING_OUT` flag, bot is a normal member again |
| Destination leader | `T_MIGRATION_TIMEOUT` | Drop `PENDING_INCOMING` entry |

This prevents orphaned state if any party crashes mid-handshake.

**Implementation.** Both leader-side records are `peers.table.Ledger`
entries (`migrating_out` on the source, `pending_incoming` on the
destination), expired by the leader's run loop every tick. The bot side is
the `Migrator`'s per-attempt deadline. `Departure` clears `migrating_out`
early; a successful `MigrationJoin` clears `pending_incoming` early.

---

## 9. Config Reference

| Name | Default | Notes |
|---|---|---|
| `T_HB` | 1s | Bot heartbeat interval to leader |
| `T_LEADER_HB` | 2s | Leader-to-leader heartbeat interval |
| `T_DEAD` | 3 × T_HB = 3s | Leader removes bot after this many missed heartbeats |
| `T_LEADER_DEAD` | 3 × T_HB = 3s | Bot triggers election after this many missed acks |
| `T_ELECT_TIMEOUT` | 2 × T_HB = 2s | Wait for higher-priority peer to declare Coordinator |
| `T_MIGRATION_TIMEOUT` | 10s | Max time for full migration handshake |
| `T_SETTLE` | 2 × T_LEADER_HB = 4s | A new leader is "settled" after this long with no conflict (§5.7) |
| `T_MIGRATION_BACKOFF_MAX` | 10s | Cap on exponential backoff between migration attempts |
| `NODE_TRAIL_LEN` | 3 | Recent distinct QR nodes a bot reports (newest first) |
| `T_LEADER_TENURE` | 1800s | Leader rotates to the best free successor after this (0 disables) |
| `T_JOB_EVENT_TTL` | 600s | Observer keeps re-sending an unowned JobEvent this long |
| `JOB_MIN_BATTERY` / `JOB_MAX_HOPS` / `T_JOB_RETRY` | 30% / 14 / 5s | Job dispatch (§14) |
| `T_ANNOUNCE` | = T_HB = 1s | How often a bot tells its neighbours what it holds; also how long a fresh claim stays provisional (§15) |
| `RESERVATION_TTL` | 3 × T_ANNOUNCE = 3s | A neighbour's claims lapse after this without a fresher announcement (§15) |
| `RESERVATION_REACH_HOPS` | 8 | How far ahead a bot may claim, and the test for who needs to hear from it (§15) |
| `T_JOIN_RETRY` | 0.5s | Migrating bot's wait between join attempts |
| `T_THREAD_JOIN` | 2s | Grace given to a sender thread being stopped |
| `T_DEPARTURE` | 2s | How long a departing bot waits for its leader to ack |
| `GRPC_WORKERS` | 32 | Server worker threads |
| `T_STALL` | 6 × T_HB = 6s | Commanded but not moving for this long is a stall (§16) |
| `ROBOT_PATIENCE` | 5s | How long a robot waits at a junction before driving on by itself. Its firmware owns this, not us; every WAIT stays under it, or a hold becomes a robot that leaves halfway through one (§16) |
| `ROUTE_ALTERNATES` | 3 | Alternative routes kept per job, stored as diffs (§16) |
| `HOPS_CACHE_SIZE` | 64 | Distance tables cached per source node; bounded on purpose (§16) |
| `BATTERY_CRITICAL` | 15% | Below this the planner weights charge over speed (§16) |
| `ADMIN_ENABLED` | 0 | Serve `AdminService` (introspection, robot-state injection); `up.py` sets 1 |
| `GRPC_PORT` | 50051 | Port each bot listens on |

---

## 10. Known Tradeoffs & Trust Boundaries

### Boundary flicker on auto-migrate

Migration triggers automatically when a bot scans a QR code with a different `region_id`. A bot briefly passing through a region boundary could trigger an unwanted migration. If this becomes a problem in practice, add a dwell threshold (N consecutive scans or T seconds in the new region before triggering).

### No cross-region authentication

Any bot can send a LeaderHeartbeat claiming to be a region's leader. A rogue bot could impersonate a leader. For production: add mTLS or shared secrets per region.

### HeartbeatAck scaling

At 100 bots × 1 HB/sec, the leader serializes ~10K PeerRecords/sec. The warehouse has 14 regions, so ~30 bots/region is realistic and well within limits. If regions grow beyond ~100 bots, consider delta-based acks.

### Stale PEER_LEADERS

The `PEER_LEADERS` env var is set at container launch. If bots are added later, existing bots don't know about them. The new bots discover existing ones via PEER_LEADERS (which includes the existing bots), and existing bots learn about new bots through the leader's HeartbeatAck roster. So convergence happens, just not via PEER_LEADERS directly.

---

## 11. Proto File — Complete

Verbatim copy of `proto/fleet.proto` (the source of truth; regenerate stubs
as described in `README.md`).

```protobuf
syntax = "proto3";
package fleet;

// --- Shared types ---

message PeerRecord {
  int32  bot_id         = 1;
  string address        = 2;
  int32  priority       = 3;
  string state          = 4;
  float  battery        = 5;
  int32  latest_node_id = 6;
  // Recent QR nodes, newest first (node_trail[0] == latest_node_id), at most
  // NODE_TRAIL_LEN entries, consecutive duplicates collapsed. Two or more
  // points give direction of travel, not just position.
  repeated int32 node_trail = 7;
  string job_id             = 8;
  string cargo_state        = 9;
  string mission            = 10;  // PARK | CHARGE | HOLD | IDLE | CARGO
  string fault              = 11;  // e.g. "MOTOR_ERROR", "LOW_BATTERY:12", "" if healthy
  // Right of way at yield nodes, computed by the leader from job state:
  // 0 free, 1 heading to a pickup, 2 carrying cargo. Lower yields to higher.
  int32  yield_priority     = 12;
}

// One bot's recent whereabouts — the compact form carried between leaders.
message BotLocation {
  int32 bot_id              = 1;
  repeated int32 node_trail = 2;
}

message LeaderRecord {
  int32  region_id      = 1;
  int32  bot_id         = 2;
  string address        = 3;
}

// --- RegionService: bot <-> own leader ---

service RegionService {
  rpc Heartbeat(HeartbeatRequest)          returns (HeartbeatAck);
  rpc Departure(DepartureRequest)          returns (DepartureAck);
  rpc MigrationRequest(MigrationReq)       returns (MigrationReqAck);
}

message HeartbeatRequest {
  int32  bot_id              = 1;
  int32  region_id           = 2;
  int32  latest_node_id      = 3;
  string state               = 4;
  float  battery             = 5;
  int32  priority            = 6;
  string address             = 7;
  string mission             = 8;
  string fault               = 9;
  int64  timestamp           = 10;
  repeated int32 node_trail  = 11;  // see PeerRecord.node_trail
  // The job this bot is executing, if any. Travels *with the bot* so that
  // whichever leader it is heartbeating (it migrates as it drives) knows
  // whose job it is and how far along.
  string job_id              = 12;
  string cargo_state         = 13;  // PICKUP | EN_ROUTE | DROPOFF | DELIVERED
}

message HeartbeatAck {
  repeated PeerRecord   region_peers   = 1;
  repeated LeaderRecord other_leaders  = 2;
  // Non-empty when the receiver is NOT the leader: the address it believes
  // the leader is at. The sender should switch its heartbeat target.
  string                redirect_to    = 3;
  // bot_id of the leader — the receiver itself when it is leader, or the
  // leader it is redirecting to. Lets a bot discovering its leader by probing
  // learn *who* leads, not just where.
  int32                 leader_bot_id  = 4;
  // The leader's job ledger, replicated to every follower so a successor
  // inherits it — jobs must not die with a leader.
  repeated Job          jobs           = 5;
}

// --- Jobs: cargo from one node to another ---

message Job {
  string job_id          = 1;   // == cargo_id (UUID from the order system) → idempotent
  int32  pickup_node     = 2;
  int32  dropoff_node    = 3;
  int32  owner_region    = 4;   // region whose leader ASSIGNED it and will cross it off
  string status          = 5;   // PENDING | ASSIGNED | PICKED_UP | DELIVERED | NEEDS_ATTENTION
  // `optional` gives real presence: bot_id 0 is a valid bot, so "unset"
  // must be distinguishable from "assigned to bot-0".
  optional int32 assignee = 6;
  int32  last_node       = 7;   // where the assignee was last seen (for failure reports)
  string reason          = 8;   // why it failed / needs attention
  int64  updated_at      = 9;
}

message JobAck {
  bool   accepted        = 1;
  optional int32 assignee = 2;   // present iff a bot took the job (may be bot-0)
  int32  owner_region    = 3;
  string note            = 4;
}

message ForwardJobReq {
  Job   job                    = 1;
  repeated int32 tried_regions = 2;
  int32 hops                   = 3;
}

message JobEventAck {
  // True iff the receiver owns the job and applied the event. An observer
  // keeps re-sending to every leader it knows until one answers owned=true.
  bool owned = 1;
}

// Accepts new jobs. Served by every bot; a non-leader or a leader of the
// wrong region forwards to the pickup node's region leader.
service JobService {
  rpc SubmitJob(Job) returns (JobAck);
}

// Served by every bot: a leader hands a job to one of its followers.
service BotService {
  rpc AssignJob(Job) returns (JobAck);
}

// --- Reservations: bot <-> bot, no leader in the path ---
// Claims travel directly between bots because PROTOCOL.md §7 requires that
// collision avoidance keep working when there is no leader. Putting them in the
// leader-mediated heartbeat would make the one depend on the other. The leader
// still helps: its roster says where neighbours are and what to dial, and its
// yield_priority is the right of way used to settle a clash. Discovery and
// ranking, not permission. See PROTOCOL.md §15.

// One node held for one window, as offsets from the moment of sending. Relative
// rather than absolute so the two bots never need their clocks to agree; the
// receiver stamps arrival. start_offset_ms may be negative for a claim already
// under way.
message ClaimWindow {
  int32 node_id           = 1;
  int32 start_offset_ms   = 2;
  int32 end_offset_ms     = 3;
}

message ReservationAnnounce {
  int32 bot_id            = 1;
  // Bumped on every change. Announcements are idempotent and may arrive out of
  // order; a receiver holding a newer seq from this bot ignores the message.
  int32 seq               = 2;
  // Right of way, straight from PeerRecord.yield_priority: 0 free, 1 heading to
  // a pickup, 2 carrying cargo. Higher wins a clash, ties break on lower bot_id.
  // With no leader to compute it, 0 is safe — both sides then fall back to the
  // id, which is still an order they agree on.
  int32 yield_priority    = 3;
  // How long these claims stay believable without a fresher announcement, so a
  // bot that crashes stops blocking a lane instead of wedging it forever.
  int32 ttl_ms            = 4;
  // Everything this bot currently holds. An EMPTY list is a withdrawal and frees
  // those nodes at once, rather than leaving them held until the ttl lapses.
  repeated ClaimWindow windows = 5;
}

message ReservationAck {}

// Served by every bot; callers must be in the same region, the same rule the
// virtual network applies to RegionService and ElectionService (§12).
service ReservationService {
  rpc Announce(ReservationAnnounce) returns (ReservationAck);
}

// --- Admin: introspection and robot-state injection ---
// For operators and the Docker test harness. Only served when ADMIN_ENABLED=1
// (the virtual network refuses it otherwise).
//
// Read-only. It used to carry InjectRobotState and InjectObstruction, which put
// a robot snapshot or a blockage straight into the bot around the QR read, the
// companion and the wire. A robot reports over spore.network.v1.RobotNetwork
// now, like a robot; see proto/robot.proto and §16.1.

service AdminService {
  rpc GetState(Empty)                     returns (BotState);
}

message Empty {}

message BotState {
  int32  bot_id             = 1;
  int32  region_id          = 2;
  string role               = 3;   // "leader" | "follower"
  int32  leader_bot_id      = 4;
  string leader_address     = 5;
  int32  priority           = 6;
  string state              = 7;
  string current_job_id     = 8;
  string cargo_state        = 9;
  repeated PeerRecord   roster        = 10;
  repeated LeaderRecord other_leaders = 11;
  repeated Job          jobs          = 12;
  int32  desired_region_id  = 13;
  bool   leader_settled     = 14;
  // What this bot holds and what its neighbours have told it they hold, in one
  // list and told apart by bot_id. Without this a container test can start two
  // bots but cannot see whether a claim ever crossed between them (§15).
  repeated HeldClaim    reservations  = 15;
}

// One entry of a bot's ledger, in absolute milliseconds on that bot's clock.
message HeldClaim {
  int32 bot_id   = 1;
  int32 node_id  = 2;
  int64 start_ms = 3;
  int64 end_ms   = 4;
}

message DepartureRequest {
  int32 bot_id               = 1;
  int64 timestamp            = 2;
}

message DepartureAck {}

message MigrationReq {
  int32 bot_id                   = 1;
  int32 destination_region_id    = 2;
  int64 timestamp                = 3;
}

message MigrationReqAck {
  bool         approved          = 1;
  LeaderRecord destination_leader = 2;
}

// --- LeaderExchangeService: leader <-> leader ---

service LeaderExchangeService {
  rpc LeaderHeartbeat(LeaderHBRequest)          returns (LeaderHBAck);
  rpc MigrationHandoff(MigrationHandoffReq)     returns (MigrationHandoffAck);
  // "Nobody here is free — can your region take this job?"
  rpc ForwardJob(ForwardJobReq)                 returns (JobAck);
  // A leader observed something about a job it does not own (the assignee
  // migrated into its region and then delivered / failed / vanished).
  rpc JobEvent(Job)                             returns (JobEventAck);
}

message LeaderHBRequest {
  int32  region_id           = 1;
  int32  leader_bot_id       = 2;
  string address             = 3;
  int32  bot_count           = 4;
  float  avg_battery         = 5;
  int32  active_jobs         = 6;
  int64  timestamp           = 7;
  int32  priority            = 8;
  // Where every bot in the sender's region (leader included) has been
  // lately. Gives each leader a fleet-wide picture of movement.
  repeated BotLocation locations = 9;
}

message LeaderHBAck {
  int32  region_id           = 1;
  int32  leader_bot_id       = 2;
  string address             = 3;
  int32  bot_count           = 4;
  float  avg_battery         = 5;
  int32  active_jobs         = 6;
  int64  timestamp           = 7;
  int32  priority            = 8;
  repeated BotLocation locations = 9;
}

message MigrationHandoffReq {
  int32  bot_id              = 1;
  int32  source_region_id    = 2;
  int32  bot_priority        = 3;
  string bot_address         = 4;
}

message MigrationHandoffAck {
  bool accepted              = 1;
}

// --- ElectionService: bot <-> peer bot ---

service ElectionService {
  rpc Elect(ElectRequest)                  returns (ElectResponse);
  rpc Coordinator(CoordinatorRequest)      returns (CoordinatorResponse);
}

message ElectRequest {
  int32 bot_id               = 1;
  int32 priority             = 2;
  int64 timestamp            = 3;
  // The challenger's dialable address. The receiver adds the challenger to
  // its roster so that, if it wins, its Coordinator reaches the challenger
  // even when the challenger was missing from its last ack.
  string address             = 4;
}

message ElectResponse {
  bool ack                   = 1;
}

message CoordinatorRequest {
  int32  bot_id              = 1;
  int32  priority            = 2;
  string address             = 3;
}

message CoordinatorResponse {}

// --- MigrationJoinService: bot -> destination leader ---

service MigrationJoinService {
  rpc MigrationJoin(MigrationJoinReq)      returns (MigrationJoinAck);
}

message MigrationJoinReq {
  int32  bot_id              = 1;
  int32  source_region_id    = 2;
  int32  priority            = 3;
  string address             = 4;
  float  battery             = 5;
  string state               = 6;
  int32  latest_node_id      = 7;
  repeated int32 node_trail  = 8;
}

message MigrationJoinAck {
  bool                   accepted       = 1;
  repeated PeerRecord    region_peers   = 2;
  repeated LeaderRecord  other_leaders  = 3;
}
```

---

### 11.1 The robot link — `proto/robot.proto`

A second file, and deliberately so: `fleet.proto` is bot to bot, this is bot to
*its own robot*. They have different callers, different lifetimes and different
authorities, and the robot one is a rendering of the shared JSON schemas rather
than a design of ours.

```protobuf
// The robot <-> network link.
//
// These messages are the shared JSON schemas expressed as protobuf, field for
// field:
//
//   schemas/robot-to-network.schema.json   -- a robot reporting status upward
//   schemas/network-to-robot.schema.json   -- the network commanding a robot
//
// The schemas remain the ground-truth contract; this file is a second
// rendering of them for a typed, binary wire. Both directions share Id,
// Timestamp, CargoId, CargoState, Cargo and Mission because both schemas
// define them identically, so they are declared once here.
//
// **Left and right never cross this wire.** `RobotToNetwork` carries
// `latest_node_id` and `NetworkToRobot` carries `target_node_id`. Neither has
// a field for a turn. The network layer routes and holds the map; the robot
// holds the map too and derives the bearing to the node it was named. That is
// exact -- lanes are straight -- so a direction on the wire would be a second,
// weaker description of geometry both ends already have. The firmware bears
// this out: it reads `bearing` and `heading` off a TURN command and has never
// read the turn name beside them.
//
// **Four fields are ours, and are not in the schemas.** `available`,
// `heading_rad` and `query_id` going up, `kind`, `hold_ms`, `because` and
// `query_id` coming down. They exist because this fleet answers a robot *at a
// junction* as well as telling it where to go, and a destination alone cannot
// say "hold 800 ms and ask again" or "stand aside at node 412". Silence is the
// one answer a blind robot cannot recover from -- it only asks again on
// reaching the next node, and it will not reach one -- so the ability to say
// wait is not decoration. See PROTOCOL.md §16.2.
//
// A contract test asserts exactly these and no others are additions, so the
// next field cannot be added to one side quietly.
//
// Two places where proto3 cannot say what JSON Schema says, both handled below
// rather than left as traps:
//
//   `required`  proto3 has no such keyword, and every Id and Timestamp has
//               `minimum: 0`, so zero is a legal value and an absent field is
//               indistinguishable from a present zero. Required scalars are
//               therefore declared `optional`, which buys explicit presence:
//               a receiver can tell "bot 0" from "nobody said". This costs
//               nothing on the wire.
//
//   `oneOf`     maps to `oneof`, which is exact. `Fault` is deliberately NOT a
//               oneof: its schema requires nothing and forbids nothing, so a
//               fault may carry a warning, an error, both, or neither.

syntax = "proto3";

package spore.network.v1;

service RobotNetwork {
  // One long-lived bidirectional stream per robot. The robot (the companion,
  // acting for its robot) is the client; the network layer is the server --
  // and the server is *this robot's own bot*, not a service for the fleet.
  // Why: spore-amr/network-layer/docs/boundary.md.
  //
  // The stream types name the direction, so there is no envelope and no
  // `schema` discriminator to get wrong: what a robot may send and what it may
  // receive are different types, and the compiler enforces it.
  //
  // A stream is ordered and reliable, so no sequence numbers: a command and a
  // status can never be reordered, and `timestamp` is a data value, not a
  // transport concern.
  rpc Session(stream RobotToNetwork) returns (stream NetworkToRobot);
}

// --- Shared types -----------------------------------------------------------

// Schema `Mission`: one of five, exactly one set.
//
// Four variants carry no data -- the schema gives them a `type` const and
// nothing else -- so they are empty messages. The variant is the field that is
// set; there is no separate `type` string to keep in agreement with it.
message Mission {
  oneof kind {
    Park park = 1;
    Charge charge = 2;
    Hold hold = 3;
    Idle idle = 4;
    Cargo cargo = 5;
  }
}

message Park {}
message Charge {}
message Hold {}
message Idle {}

// Schema `MissionCargo` flattened: its `type` const is the `Mission.cargo`
// case, leaving the cargo itself.
message Cargo {
  // Schema `CargoId`: a UUID, intentionally not an `Id`. Orders are minted by
  // an external system, so their identifiers cannot come from a central
  // sequential allocator. The UUID pattern stays the schema's to enforce;
  // protobuf has no such type.
  optional string cargo_id = 1;

  optional CargoState state = 2;
}

enum CargoState {
  // Not in the schema: proto3 requires a zero value, and it doubles as the
  // "field absent or unrecognised" sentinel that `required` would otherwise
  // have caught.
  CARGO_STATE_UNSPECIFIED = 0;
  CARGO_STATE_PICKUP = 1;
  CARGO_STATE_DROPOFF = 2;
  CARGO_STATE_EN_ROUTE = 3;
}

// --- robot -> network -------------------------------------------------------

message RobotToNetwork {
  // Schema `Id`: integer, minimum 0. Zero is a legal bot, region and node, so
  // these are `optional` to keep "absent" distinguishable from "zero".
  optional uint32 bot_id = 1;
  optional uint32 region_id = 2;

  // The last node whose marker this robot read. Its whole report of where it
  // is: the network layer is told a node, never a pose.
  optional uint32 latest_node_id = 3;

  optional Mission mission = 4;
  optional Telemetry telemetry = 5;

  // Optional in the schema too: absent means nothing is wrong.
  optional Fault fault = 6;

  // Schema `Timestamp`: integer, minimum 0.
  optional uint64 timestamp = 7;

  // --- Not in the schema: asking, as opposed to reporting ------------------

  // The nodes this robot can legally reach from where it stands, resolved by
  // the robot from its own map against the heading it arrived on. It decides
  // what is physically possible, not us: our map and its map can disagree, and
  // when they do the robot is right about the floor it is on.
  //
  // **Its presence is the question.** A report with `available` populated is a
  // robot stopped at a junction waiting to be told where to go, and is
  // answered. A report without it is telemetry -- position, battery, a fault --
  // and is not. That is what makes this one wire rather than two.
  repeated uint32 available = 8;

  // The heading the robot arrived on, in radians. Exact rather than odometric:
  // lanes are straight, so the bearing from the previous node to this one *is*
  // the direction of travel, with no drift in it.
  optional double heading_rad = 9;

  // Ties an answer to the question that asked it. Two junctions can share a
  // destination -- a reroute to the same place -- so the id is the only way to
  // tell a fresh answer from a late one. It matters more here than it looks: a
  // `WAIT` meant for the previous node, applied at this one, stops a robot that
  // had nothing wrong with it.
  optional uint64 query_id = 10;
}

message Telemetry {
  optional Battery battery = 1;
}

message Battery {
  // Schema: `number`, 0 to 100 -- a JSON number, not an integer, so double.
  optional double percentage = 1;
}

// Schema `Fault`: both members optional, neither required. Not a oneof --
// a robot may report a warning and an error at once, and the schema allows an
// empty fault.
message Fault {
  optional Warning warning = 1;
  optional Error error = 2;
}

// Schema `Warning`: one of two, exactly one set.
message Warning {
  oneof kind {
    LowBattery low_battery = 1;
    Obstacle obstacle = 2;
  }
}

message LowBattery {
  // 0 to 100. Distinct from `Battery.percentage`: this one is the level that
  // tripped the warning, at the moment it tripped.
  optional double percentage = 1;
}

message Obstacle {
  // Where the robot was when it saw the obstacle. A node, not a pose, for the
  // same reason as `latest_node_id`.
  optional uint32 current_node_id = 1;
}

// Schema `Error` is an object wrapping the enum rather than the bare enum. Kept
// as a message so it matches the schema and has somewhere to grow -- a detail
// string, a timestamp -- without changing `Fault`.
message Error {
  optional ErrorType type = 1;
}

enum ErrorType {
  ERROR_TYPE_UNSPECIFIED = 0;
  ERROR_TYPE_MOTOR_ERROR = 1;
  ERROR_TYPE_CAMERA_ERROR = 2;
  ERROR_TYPE_LIDAR_ERROR = 3;
  ERROR_TYPE_LOCATION_UNKNOWN = 4;
  ERROR_TYPE_MISC_ERROR = 5;
}

// --- network -> robot -------------------------------------------------------

message NetworkToRobot {
  // Where to go. Not how to get there, and not which way to turn: the robot
  // derives the bearing from the map it already holds.
  optional uint32 target_node_id = 1;

  // Optional in the schema: absent leaves the robot's current mission alone.
  optional Mission set_mission = 2;

  optional uint64 timestamp = 3;

  // --- Not in the schema: answering, as opposed to commanding --------------

  // What kind of answer this is. Absent means PROCEED, so a robot that reads
  // only `target_node_id` still behaves correctly for every moving kind; only
  // WAIT needs the field understood.
  optional Kind kind = 4;

  // How long to stay put before asking again. Only meaningful with WAIT, and
  // it must stay under the robot's own patience -- the firmware drives on when
  // a junction goes unanswered for long enough, so a hold longer than that
  // becomes a robot that leaves mid-hold.
  optional uint32 hold_ms = 5;

  // Why, in words, for the log. Never parsed. A robot that waited for a reason
  // nobody recorded is a robot nobody can debug.
  optional string because = 6;

  // Echoes `RobotToNetwork.query_id`. See there.
  optional uint64 query_id = 7;
}

enum Kind {
  // Proto3 needs a zero value and it doubles as the sensible default: a
  // decision that says nothing about its kind is an ordinary "take this lane".
  KIND_UNSPECIFIED = 0;
  // Take this lane; it is the route we were already on.
  KIND_PROCEED = 1;
  // Take this lane, but the route changed since we last answered.
  KIND_REROUTE = 2;
  // Stay put for `hold_ms`, then ask again.
  KIND_WAIT = 3;
  // Leave the route and stand aside at the node named.
  KIND_YIELD = 4;
}
```

---

## 12. Virtual Network — the Policy Layer

All bots share one flat physical network: a single Docker bridge locally, one
WiFi in the warehouse. Physical isolation is deliberately **not** used — a
migrating bot must talk to two leaders at once (§4.6), which isolated networks
would make awkward.

Region isolation is therefore enforced in code. Every outgoing RPC attaches
identity metadata, and every bot runs a gRPC server interceptor
(`bus/policy.py`) that checks it before any handler executes:

| Metadata key | Value |
|---|---|
| `bot-id` | caller's `bot_id` |
| `region-id` | caller's current `region_id` |
| `role` | `leader` or `follower` |

| Service | Who may call it |
|---|---|
| `RegionService`, `ElectionService`, `ReservationService` | callers in **the receiver's own region** |
| `LeaderExchangeService` | callers whose role is `leader`, from **any** region |
| `MigrationJoinService` | callers with a **pending handoff** on the receiver |
| `JobService` (`SubmitJob`) | **any** authenticated caller — the order system or any bot |
| `BotService` (`AssignJob`) | callers whose role is `leader` |
| `AdminService` | any authenticated caller, **only when `ADMIN_ENABLED=1`** |
| (no metadata) | nobody — `UNAUTHENTICATED` |

Disallowed calls fail with `PERMISSION_DENIED`. Senders already treat any
`RpcError` as "peer unreachable", so a policy denial degrades the same way a
dead peer would: missed acks → `T_LEADER_DEAD` → election.

**Consequences worth knowing**

- A follower heartbeating a leader that has since *migrated to another region*
  is denied (region mismatch), counts missed acks, and elects. That is the
  intended behaviour — the leader is gone from this region's point of view.
- During migration the bot must send `Departure` to its old leader **before**
  switching its own `region_id`, otherwise the Departure is denied. The code
  does this; the ordering matters.
- Because `role` is self-asserted, this is isolation, not authentication. A
  rogue bot on the WiFi can claim `role=leader`. See §10 — the fix is mTLS,
  and the interceptor is the natural place to add it.

---

## 13. Local Orchestration (Docker)

`up.py` / `down.py` drive the Docker Engine API via the `docker` SDK — no
shelling out.

- One bridge network (`amr-net` by default); containers resolve each other by
  name through Docker's embedded DNS, so `OWN_ADDRESS` is simply
  `<container-name>:50051`.
- Every container carries `amr.fleet=1` and `amr.region=<id>` labels. `down.py`
  finds fleet containers by label and will never touch anything else; `up.py`
  refuses to replace a same-named container that lacks the label.
- `PEER_LEADERS` for a new batch is *every fleet container already running*
  plus its batch-mates, so a second region launched later discovers the first.
- `BOT_ID` continues from the highest running ID unless `--start-id` is given,
  keeping IDs unique fleet-wide (they double as election priority).
- `spore-amr/shared/warehouse-layout.json` is bind-mounted read-only into
  every container at `/app/warehouse-layout.json` (`WAREHOUSE_MAP`); it lives
  outside the build context so it is never baked into the image.
- `down.py` uses `remove(force=True)` — a SIGKILL. Bots get no chance to send
  `Departure`; peers notice via `T_DEAD`. That is the *hard death* path (§4.5),
  which is what you usually want to exercise locally.

**Docker test harness** (`tests/test_docker.py`): every scenario the
in-process suite cannot honestly test runs on real containers and a real
bridge network — bootstrap, `kill` + restart, `pause` (hung leader: calls
time out rather than being refused) + split-brain heal, `network disconnect`
/ `connect`, two-region migration, a job end to end. Each test gets its own
network and name prefix; bots are driven and inspected through
`AdminService`. Skipped without a daemon; `AMR_DOCKER_NO_BUILD=1` reuses the
image. Two bugs the in-process suite could not see were found here: bot-0
being un-assignable (`0` used as "no bot"), and the self-follow loop above.

```
uv run up.py --bots 3 --region 14        # build + launch region 14
uv run up.py --bots 2 --region 2 --no-build
docker logs -f amr-region-14-bot-0
uv run down.py --region 2               # or omit --region for everything
```

---

## 14. Jobs — cargo from one node to another

A job moves cargo from a pickup QR node to a dropoff QR node. The schemas
give the vocabulary: `network-to-robot` commands a bot with `target_node_id`
+ `set_mission: CARGO {cargo_id, PICKUP | EN_ROUTE | DROPOFF}`, and
`robot-to-network` reports `mission`, `telemetry.battery` and `fault`
(warnings `LOW_BATTERY`, `OBSTACLE`; errors `MOTOR_ERROR`, `CAMERA_ERROR`,
`LIDAR_ERROR`, `LOCATION_UNKNOWN`, `MISC_ERROR`). Code: `bus/jobs.py`,
`warehouse/map.py`, the job half of `bot.py`.

### 14.1 The record

```
Job {
  job_id        = cargo_id (UUID minted by the order system → idempotent)
  pickup_node, dropoff_node
  owner_region  = the region whose leader ASSIGNED it (see 14.2)
  status        PENDING → ASSIGNED → PICKED_UP → DELIVERED (removed)
                   ▲         │            │
                   └──fail───┘            └──fail──▶ NEEDS_ATTENTION
  assignee, last_node, reason, updated_at
}
```

### 14.2 Ownership — the leader that assigns, owns

The **owner** is the leader that *successfully hands the job to a bot*, not
the one that first received it. If region A had nobody free and forwarded to
B, B's leader owns the job and crosses it off; A keeps nothing.

The owner's ledger rides in every `HeartbeatAck.jobs`, so its followers hold
a replica and a successor inherits it — jobs must not die with a leader (§7).

### 14.3 The job travels with the bot

A bot executing a job **drives across regions and migrates** as it goes
(§4.6). So the leader that watches it deliver, or die, is often not the
owner. The job therefore lives in the bot's own heartbeat (`job_id`,
`cargo_state`), and whichever leader is receiving those heartbeats knows
whose job it is. Progress and failure are **observed**, never reported by
extra messages from the bot.

### 14.4 Flow

```
1. SubmitJob(job) → any bot.  A follower forwards to its leader; a leader
   forwards to the leader of the pickup node's region (map: node → region,
   roster: region → leader). If that leader is unknown/unreachable the
   receiving leader takes it itself.                          PENDING

2. The leader picks the best FREE follower:
     free = healthy ∧ state IDLE ∧ mission ∈ {IDLE, PARK} ∧ no job ∧ no fault
            ∧ battery ≥ JOB_MIN_BATTERY ∧ not migrating
     best = fewest map hops (BFS over warehouse-layout.json edges) from the
            bot's latest node to pickup_node, then higher battery, then id
   AssignJob(job) → bot; the bot accepts iff it is free.       ASSIGNED
   The leader itself is the candidate of LAST resort: it takes the job only
   when no follower is free. Nobody observes a leader's heartbeats, so a
   leader working a job observes itself each tick. If the job leaves the
   region it abdicates on the way out (§4.7) and the successor inherits the
   ledger from its replica. Net effect: the leader is normally the bot that
   stays put — parked or charging in its zone — which is also the best bot
   to lead.

3. Nobody free → ForwardJob(job, tried) to the leader of the nearest
   untried region (map hops from pickup_node to that region's nearest node).
   That leader runs step 2; if it also has nobody it forwards on, up to
   JOB_MAX_HOPS. Whoever assigns becomes owner and answers back up the
   chain. Nobody anywhere → the pickup region's leader keeps it PENDING and
   retries every T_JOB_RETRY; if a later retry forwards it successfully the
   stale local copy is dropped.

4. The bot commands the robot and mirrors its progress:
     on assign               → target=pickup,  CARGO/PICKUP
     robot says CARGO/EN_ROUTE (it has the cargo)
                             → target=dropoff, CARGO/EN_ROUTE
     robot says CARGO/DROPOFF then leaves CARGO (set down)
                             → bot reports cargo_state DELIVERED until acked
   The observing leader turns heartbeat transitions into events:
     PICKUP → EN_ROUTE/DROPOFF   ⇒ PICKED_UP
     … → DELIVERED               ⇒ DELIVERED   (owner removes the job)

5. Failure is observed the same way. Trouble =
     state FAULTED / COMMS_LONG, or
     fault MOTOR/CAMERA/LIDAR/LOCATION_UNKNOWN/MISC/LOW_BATTERY, or
     mission CHARGE, or
     the job simply vanishing from the bot's heartbeats without DELIVERED
     (it abandoned it, or its bridge reset), or
     heartbeats lost (evicted after T_DEAD — unless it is in migrating_out,
     in which case it merely left for another region, cargo and all).
   A failure report names the bot it is about; once the owner has
   re-assigned the job, a late report about the previous assignee is ignored.
   If the assignee's last cargo_state was PICKUP (nothing collected yet):
     ⇒ PENDING — someone still has to pick it up; the owner re-dispatches on
       the next tick. The failed bot drops the job so it cannot resume it
       after recovering and race the replacement.
   If it was EN_ROUTE / DROPOFF (cargo is on the broken bot):
     ⇒ NEEDS_ATTENTION — a human has to move it. The owner raises it to the
       control plane (`Bot.control_plane`, default: ERROR log + `Bot.alerts`)
       with the last node. The broken bot keeps reporting the job so anyone
       reading the roster can see where the cargo is.

6. If the observing leader is not the owner it sends JobEvent to every
   leader it knows; only the owner acts (`owner_region` is a region, not a
   bot, so a successor leader acts too).
```

### 14.5 Surface

| Piece | Where | Notes |
|---|---|---|
| `JobService.SubmitJob` | every bot | any authenticated caller (§12) |
| `BotService.AssignJob` | every bot | leaders only (§12) |
| `LeaderExchangeService.ForwardJob` / `JobEvent` | leaders | leaders only |
| `HeartbeatRequest.job_id / cargo_state`, `PeerRecord.{job_id, cargo_state, mission, fault}` | heartbeats / acks | the job travels with the bot; region-mates see who is busy or broken |
| `HeartbeatAck.jobs` | acks | ledger replication |
| `RobotSink` | `bot.py` | outbound twin of `RobotSource`; emits `network-to-robot` commands |
| `warehouse/map.py` | boot | node → region, BFS hop distance; degrades to geography-blind if the file is missing |
| `JOB_MIN_BATTERY`, `JOB_MAX_HOPS`, `T_JOB_RETRY`, `WAREHOUSE_MAP` | `config.py` | |

### 14.6 Known gaps

- `JobEvent` delivery is retried: the observer keeps an event queued and
  re-sends it to every leader it knows every `T_JOB_RETRY` until one answers
  `owned=true`, giving up after `T_JOB_EVENT_TTL`. The remaining gap is an
  owner region that stays unknown for longer than that.
- Nothing tells a leader *where to be*. It leads from wherever it was when
  elected; the protocol does not care (WiFi), but the robot layer might
  (lanes). A `HOLD` at the region's nearest yield/parking node on becoming
  leader would be a one-line `RobotSink` command once the map carries node
  types — a robot-behaviour decision, not taken yet.
- Distance uses the assignee's *last scanned node*; a bot between nodes is
  scored from the node behind it.

---

## 15. Reservations — two robots, one node

Everything else in this document flows through a leader. This does not, and that
is the whole point: §7 says the leader must not "be required for safety-critical
paths — reservations, collision avoidance, energy shutoff work without the
leader". A claim carried inside a heartbeat would make collision avoidance depend
on the very thing §7 says it must survive.

So claims go bot to bot. The leader is not cut out — it gains a job. Its roster
already carries every region-mate's `latest_node_id` and dialable `address`,
which is exactly what a bot needs to work out who to talk to, and its
`yield_priority` (§5.6) is the right of way used to settle a clash. **Discovery
and ranking, not permission.** Once a bot has a neighbour's address it talks to
it directly, and a leader dying never interrupts that.

### 15.1 The four rules

**Announce, do not ask.** A bot says "I am taking node 412 for the next two
seconds". It does not wait for a yes. Asking would need a reply per claim, and
two bots asking each other at the same moment would both wait forever.

**Windows are relative.** `+200ms to +2400ms`, and the receiver stamps its own
clock on arrival. The two bots never need their clocks to agree, which is why
none of the usual synchronisation machinery appears anywhere in this section.

**A fresh claim is provisional.** It only counts one `T_ANNOUNCE` after it is
made. This looks like a delay for nothing and is the entire safety story: two
bots that claim the same node in the same breath have not heard each other yet,
and if either acted at once they would collide. Waiting one round guarantees the
clash surfaces before anybody moves. Carrying on holding a node you already hold
is *not* a fresh claim — otherwise re-announcing every tick would restart the
clock forever and no claim would ever come into force.

**A clash is settled by an ordering, not a conversation.** Both bots hold both
claims and both sort them the same way: higher `yield_priority` first, then lower
`bot_id`. Same facts, same rule, same answer, computed independently. The loser
withdraws; its next announcement carries the retraction.

One deliberate extra caution: the winner does **not** move the moment it works
out that it won. It waits until the loser's retraction actually arrives. "I
should win" is not "the other bot knows it lost", and acting on the first is how
you drive into a robot that has not got the message.

### 15.2 Who hears from whom

Not the whole region. A bot's claims never reach more than
`RESERVATION_REACH_HOPS` from where it stands, so a neighbour can only contest a
node we hold if it is within that distance *of that node*. Measuring from the
nodes actually held — a handful — rather than from a radius around the bot is
both exact and much smaller. In practice two or three bots out of twenty.

Positions come from the roster; distances from `warehouse/map.py`, already
BFS-cached per source. With no map file that class degrades to `NullMap`, every
distance reads 0, and a bot announces to everyone in its region: chattier, still
correct, and the same way job dispatch degrades without geography.

### 15.3 Lifecycle

| Event | What happens |
|---|---|
| Every `T_ANNOUNCE` | Expire lapsed neighbours, give way where we lost, claim the node underfoot, announce to whoever is in range |
| Claim refused | A better-ranked neighbour already holds part of it. Nothing is half-granted — a partial route is one the robot cannot finish |
| Contest lost | Withdraw, and drop the route that was costed against those windows |
| Neighbour goes quiet | Its claims lapse after `RESERVATION_TTL`, so a crashed bot stops blocking a lane |
| Empty announcement | A *withdrawal*, not a lapse: those nodes free up at once |
| Neighbour drifts out of range | Forgotten; it can no longer contest anything we hold |

### 15.4 What is not here yet

A bot currently claims only the node it is standing on. That is real information
— a neighbour planning through here needs to know somebody is sitting on it — but
it is the floor, not the ceiling. Claiming a *route* needs something that decides
where the robot is going next, and that decision does not live in this repository
yet. See `TODO.md`.

---

## 16. Pathfinding — telling a blind robot where to go

> What the fleet does in each concrete situation, with the container test that
> proves it, is **[`docs/scenarios.md`](docs/scenarios.md)**. This section is the
> design; that one is the behaviour.

The robot has no map of its own beyond adjacency and no idea where anything is.
It arrives at a QR node, works out which turns physically exist, and asks. It
then **blocks** — up to `ROBOT_PATIENCE`, its own firmware's limit — and if it
hears nothing it sits
there for the rest of its shift, because it only asks again on reaching the next
node, and it will not reach one.

Everything below follows from that.

### 16.1 The link

One long-lived gRPC stream per robot: `spore.network.v1.RobotNetwork/Session`,
defined in [`proto/robot.proto`](proto/robot.proto). Each bot serves it
(`planning/robot_service.py`) for **its own robot** — not a service for the
fleet, see [`docs/boundary.md`](docs/boundary.md) — and the companion dials
`NETWORK_ADDRESS`.

```
RobotToNetwork{bot_id, region_id, latest_node_id, mission, telemetry, fault,
               timestamp, available[], heading_rad, query_id}
NetworkToRobot{target_node_id, set_mission, timestamp,
               kind, hold_ms, because, query_id}
```

The messages are `spore-amr/shared/schemas/robot-to-network.schema.json` and
`network-to-robot.schema.json` rendered field for field, plus the four fields
this fleet adds for *asking* rather than reporting. `tests/test_proto_contract.py`
fails if either side gains a field the other has not declared.

**One message does both jobs.** A report carrying `available` is a robot stopped
at a junction waiting to be told where to go, and is answered. One without it is
telemetry, and is not. Both update position — which is the whole point, and is
covered in [`docs/location.md`](docs/location.md): the fleet learns where its
robots are from the same messages that ask it where to send them, so a robot
that is driving is a robot that is reporting.

Two things the robot gives us for free. `available` is resolved by the robot
from its own copy of the map against the heading it arrived on — so **it**
decides what is physically possible, not us. And `heading_rad` is exact: lanes
are straight, so the bearing between the last two nodes *is* the direction of
travel, with no odometry drift in it.

**Left and right never cross this wire.** `available` and `target_node_id` name
nodes; neither direction has a field for a turn. We route and hold the map, the
robot holds the map too, and it derives the bearing to the node it was named —
exact, where a turn name would be a second and weaker description of geometry
both ends already have. The firmware settles the argument: it reads `bearing`
and `heading` off a TURN command and has never read a turn name.

`kind` is one of:

| kind | means | carries |
|---|---|---|
| `PROCEED` | take this lane, same route as before | `turn`, `target_node_id` |
| `REROUTE` | take this lane, the route changed | `turn`, `target_node_id` |
| `WAIT` | stay put, then ask again | `hold_ms` |
| `YIELD` | leave the route, stand aside here | `turn`, `target_node_id` |

`kind` is additive: a robot reading only `turn` and `target_node_id` still
behaves correctly for the three moving kinds. **`WAIT` is the one that needed
the robot side too**, and it exists because the original protocol had no way to
express waiting — a robot told to wait was indistinguishable from a robot whose
network layer had died.

### 16.2 Never answer with silence

A malformed query, a planner that raised, a robot standing somewhere our map has
never heard of, a `Query` whose `available` excludes the node we wanted — every
one of them gets a Decision. A wrong turn is recoverable at the next node.
Silence is not recoverable at all.

The only case where we say nothing is a query so broken it has no `query_id` to
answer against, and a reply carrying the wrong one is discarded by the robot
anyway. That case is logged.

### 16.3 What we know about other robots — three tiers

| tier | source | strength |
|---|---|---|
| 1. Declared | peer `res[]` from the reservation ledger (§15) | hard |
| 2. Predicted | `node_trail` heading, extrapolated | hard, but yields to tier 1 |
| 3. Soft | positions, region density, obstructions | cost only |

A declared claim is a promise. A polled position is an observation, and where
that robot goes next is our inference — so **a prediction never contradicts a
declaration**. If a peer has told us it holds A and B, we do not additionally
block C because we guessed it was heading there. Prediction covers only peers
that have not announced: out of claim range, freshly arrived, or moving between
announcements.

Prediction is worth trusting because on this floor plan heading usually
*determines* the next hops. Over all 1,904 directed steps of the real map, 64%
have at least one next hop with no choice at all, 41% have two, and the longest
forced run is 16. Inside a corridor there is nowhere else to go — a fact about
the graph, not a guess about the driver. It stops at the first junction, where
the peer gets a real choice back.

### 16.4 Proceed, wait, yield, or reroute

The search already prices **waiting against going round**: that is what its wait
actions are for, and with charge in the cost function a robot low on battery
correctly waits where a fresh one detours.

**Yielding** is the third option and the only one that means leaving the route,
so it needs a rule rather than falling out of the search. We give way when all
three hold:

1. the plan waits longer than `T_YIELD_THRESHOLD`;
2. we **lose** the yield-priority comparison against whoever is blocking us —
   free `0` < heading-to-pickup `1` < carrying cargo `2` (§5.6), ties on the
   lower `bot_id`; and
3. somewhere to stand aside is within `YIELD_SEARCH_HOPS`.

If we *win* that comparison we wait and keep our claim: the other robot is the
one that should move. Both sides compute the same verdict from the same two
numbers, so they cannot both give way and cannot both stay.

Somewhere to stand aside is, in order: a `YI` bay, else a junction, else a `PK`
or `CH` spur. The cascade exists because real yield bays are scarce — 15 on the
whole floor, in two regions of seven — so insisting on one would mean never
yielding across most of the warehouse. A junction is the honest second choice:
somewhere a robot can actually get past us, and there are plenty. Borrowing a
charger is last and logged, because it may block someone who needs it.

### 16.5 Regions we cannot see

A follower's roster covers **its own region only**. Leaders exchange every bot's
trail (§3.2), but that never reaches followers and `HeartbeatAck` is not growing
to carry it (§10).

So a route is planned optimistically end to end and committed only within the
current region; nodes elsewhere carry tier-3 density cost and nothing else.
Migration completing triggers a replan, and the new region's roster arrives with
the first ack. **The first decision in a new region is uninformed.** That is
accepted rather than solved.

### 16.6 When a robot stops moving

Escalating, because the first suspicion should be the cheapest one. Each rung is
a whole `T_STALL`, so a robot pausing for traffic never trips it.

| after | conclusion | action |
|---|---|---|
| 1 × `T_STALL` | our route is stale | drop it and replan |
| 2 × `T_STALL` | something will not move for us | release our claims and stand aside |
| 3 × `T_STALL` | a person should look | `NEEDS_ATTENTION` to the control plane |

Peer death and collisions ride the machinery that already exists: a bot evicted
on `T_DEAD` has its claims dropped from every neighbour's ledger, and two bots
reporting the same node is a contest the yield rule settles.

### 16.7 What it costs

Planning is 0.9 ms at the median and 3.5 ms at the 99th percentile on the real
881-node map with nineteen peers — against a 1 s tick and a 5 s socket timeout.
The margin is what makes it safe to answer on the socket thread.

Two bounded caches keep it honest on hardware that has little memory: the
per-source distance tables (`HOPS_CACHE_SIZE`, 2 bytes per node per entry) and
the per-goal heuristic tables. An earlier unbounded version of the first reached
~33 MB on the real map.

Alternative routes are kept per job as **diffs** rather than copies — where each
leaves the primary and rejoins it. Four whole routes for a seventy-hop job is
mostly four copies of one list; measured, the diffs cost 76 stored nodes against
280.
