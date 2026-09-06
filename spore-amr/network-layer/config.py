"""Runtime configuration — every tunable in one place.

WHAT
    Identity (who am I, where am I, who else exists) and timing constants,
    all read from environment variables with the defaults from PROTOCOL.md §9.

WHERE
    Imported by every module. Read once at import time; treat as constants.

WHY
    The orchestrator (`up.py`, later a K8s pod spec) injects identity as env
    vars — that is the *only* thing a bot knows at birth (PROTOCOL.md §2).
    Timing lives here so a test or a deployment can shrink or stretch the
    whole protocol uniformly by setting T_HB.

HOW
    Derived timeouts are expressed in multiples of T_HB so the ratios in the
    spec (T_DEAD = 3 × T_HB, etc.) survive when T_HB is changed.
"""
import os

# ---- Identity --------------------------------------------------------------

#: This bot's fleet-unique integer id. Also the final tiebreak in elections.
BOT_ID = int(os.environ.get("BOT_ID", "0"))

#: Region we believe we are in at boot. Overridden by the first QR scan.
REGION_ID = int(os.environ.get("REGION_ID", "0"))

#: Addresses of every other bot the orchestrator knew about at launch.
#: Used for bootstrap discovery only — after that, rosters come from acks.
PEER_LEADERS = [
    addr.strip()
    for addr in os.environ.get("PEER_LEADERS", "").split(",")
    if addr.strip()
]

#: Port our gRPC server listens on, and the interface to bind.
GRPC_PORT = int(os.environ.get("GRPC_PORT", "50051"))
GRPC_HOST = os.environ.get("GRPC_HOST", "0.0.0.0")
#: Where the map sits inside the container image; `up.py` mounts it there.
CONTAINER_MAP_PATH = os.environ.get("CONTAINER_MAP_PATH", "/app/warehouse-layout.json")

#: Serve AdminService (introspection + robot-state injection). Off unless
#: explicitly enabled; `up.py` enables it for local fleets.
ADMIN_ENABLED = os.environ.get("ADMIN_ENABLED", "0") == "1"

#: The address *other* bots should dial to reach us. Advertised in every
#: heartbeat so peers can call us directly during elections.
OWN_ADDRESS = os.environ.get("OWN_ADDRESS", f"localhost:{GRPC_PORT}")

# ---- Timing (seconds) ------------------------------------------------------

#: Follower → leader heartbeat interval. The base unit for everything below.
T_HB = float(os.environ.get("T_HB", "1.0"))

#: Leader ↔ leader heartbeat interval.
T_LEADER_HB = float(os.environ.get("T_LEADER_HB", "2.0"))

#: Leader evicts a follower it has not heard from for this long.
T_DEAD = float(os.environ.get("T_DEAD", str(3 * T_HB)))

#: Follower declares its leader dead after this long without an ack.
T_LEADER_DEAD = float(os.environ.get("T_LEADER_DEAD", str(3 * T_HB)))

#: How long a stood-down candidate waits for the higher peer's Coordinator
#: before restarting the election.
T_ELECT_TIMEOUT = float(os.environ.get("T_ELECT_TIMEOUT", str(2 * T_HB)))

#: Upper bound for one migration attempt, enforced independently by the bot,
#: the source leader, and the destination leader (PROTOCOL.md §8).
T_MIGRATION_TIMEOUT = float(os.environ.get("T_MIGRATION_TIMEOUT", "10.0"))

#: A freshly self-declared leader is not "settled" until it has survived this
#: long without a same-region conflict — bootstrap conflicts resolve within one
#: leader-heartbeat round, so two rounds is comfortably safe. Migration and
#: voluntary departure wait for a settled leader (PROTOCOL.md §5.7).
T_SETTLE = float(os.environ.get("T_SETTLE", str(2 * T_LEADER_HB)))

#: Longest pause between migration retries (exponential backoff is capped here).
T_MIGRATION_BACKOFF_MAX = float(os.environ.get("T_MIGRATION_BACKOFF_MAX", "10.0"))

#: A leader hands off to the best free successor after leading this long, so
#: no single bot leads forever (PROTOCOL.md §5.6 "Tenure"). 0 disables.
T_LEADER_TENURE = float(os.environ.get("T_LEADER_TENURE", "1800.0"))

#: How long a bot must have been physically in a new region before it migrates
#: there. A robot crossing the floor changes region every time it reads a QR
#: node in one, and migration used to fire on the first such report -- so a
#: region a robot merely drives through costs a departure, a join, and if the
#: bot happened to be leading, an election at both ends.
#:
#: Measured on a five-robot run over a four-region window: 1,546 migrations and
#: "became leader of region 1" logged 6,155 times. The cost is not the churn
#: itself but what it does to job ownership -- a new leader inherits the ledger
#: by heartbeat replication, replication never settles if leadership changes
#: every few seconds, and the same job was assigned twice six minutes apart to
#: two different robots, which then converged on one node.
#:
#: A dwell fixes the common case for free: a robot passing through is gone
#: before this elapses, so it never migrates at all, while one that has genuinely
#: arrived migrates a few seconds later than it used to and nothing else changes.
T_REGION_DWELL = float(os.environ.get("T_REGION_DWELL", str(8 * T_HB)))

# ---- Jobs -----------------------------------------------------------------------

#: How long an observing leader keeps re-sending a JobEvent nobody has
#: claimed as owner before giving up (PROTOCOL.md §14.4 step 6).
T_JOB_EVENT_TTL = float(os.environ.get("T_JOB_EVENT_TTL", "600.0"))

#: A bot is only "free" for a job at or above this battery percentage.
JOB_MIN_BATTERY = float(os.environ.get("JOB_MIN_BATTERY", "30.0"))

#: How many leaders a job may be forwarded through before we give up and
#: leave it PENDING with its current owner (retried each tick).
JOB_MAX_HOPS = int(os.environ.get("JOB_MAX_HOPS", "14"))

#: Seconds between dispatch retries for a PENDING job nobody could take.
T_JOB_RETRY = float(os.environ.get("T_JOB_RETRY", "5.0"))

#: Path to warehouse-layout.json. Missing file → geography-blind dispatch.
WAREHOUSE_MAP = os.environ.get(
    "WAREHOUSE_MAP",
    # Relative to this file: network-layer/ -> spore-amr/ -> shared/. The extra
    # "spore-amr" this used to carry was correct when network-layer sat at the
    # repo root; the move into spore-amr/ left it one level too deep, and a
    # missing map degrades silently to NullMap rather than failing loudly.
    os.path.join(os.path.dirname(__file__), "..", "shared", "warehouse-layout.json"),
)

#: How long a migrating bot waits between attempts to join its destination.
#: Shorter than T_HB: a handshake that has landed should not wait a whole
#: heartbeat to be noticed.
T_JOIN_RETRY = float(os.environ.get("T_JOIN_RETRY", "0.5"))

#: How long a sender thread is given to finish when it is being stopped. Long
#: enough for an RPC in flight to return, short enough that shutdown is not
#: perceptibly slower than a kill.
T_THREAD_JOIN = float(os.environ.get("T_THREAD_JOIN", "2.0"))

#: How long a departing bot waits for its leader to acknowledge. Short on
#: purpose: we are already shutting down, and a leader that does not answer
#: will evict us on T_DEAD anyway.
T_DEPARTURE = float(os.environ.get("T_DEPARTURE", "2.0"))

#: gRPC server worker threads. Heartbeats are cheap, but a burst of migrations
#: must not starve them -- handoffs run on their own threads, not on workers.
GRPC_WORKERS = int(os.environ.get("GRPC_WORKERS", "32"))


# ---- Planning (PROTOCOL.md §16) -------------------------------------------------

#: How many hops ahead a robot plans in detail and claims. Also the reach used
#: to decide who needs to hear its claims (reservations/vicinity.py).
K_COMMIT = int(os.environ.get("K_COMMIT", "8"))

#: Clearance added to every peer claim, covering control latency and the error
#: in estimating how fast anyone is going. Widening is always outward: over-
#: reserving costs throughput, under-reserving costs a collision.
PLAN_SAFETY = float(os.environ.get("PLAN_SAFETY", "0.25"))

#: Plans further out than this are not worth timing — the world will have moved.
PLAN_HORIZON = float(os.environ.get("PLAN_HORIZON", "180.0"))

#: Longest single wait the search will consider at one node before deciding the
#: route is not worth having.
MAX_WAIT = float(os.environ.get("MAX_WAIT", "30.0"))

#: Search budget. Bounds the worst case so one pathological request cannot eat
#: the tick on a bot that has little CPU to spare.
MAX_EXPANSIONS = int(os.environ.get("MAX_EXPANSIONS", "120000"))

#: A wait longer than this makes the robot consider standing aside instead
#: (PROTOCOL.md §16). Below it, waiting is simply cheaper than moving.
T_YIELD_THRESHOLD = float(os.environ.get("T_YIELD_THRESHOLD", "4.0"))

#: How far to look for somewhere to stand aside. Real yield bays are scarce —
#: 15 on the whole floor — so the search falls back to junctions and then bays.
YIELD_SEARCH_HOPS = int(os.environ.get("YIELD_SEARCH_HOPS", "8"))

#: How long a robot sitting on its goal waits before asking again.
T_ARRIVED_HOLD = float(os.environ.get("T_ARRIVED_HOLD", "2.0"))

#: How long to hold when there is no route at all. Short: whatever is in the way
#: is usually another robot that is about to move.
T_BLOCKED_HOLD = float(os.environ.get("T_BLOCKED_HOLD", "1.0"))

#: Longest single WAIT we issue. Must stay below the robot's socket timeout, or
#: a held robot cannot tell us apart from a network layer that has died.
T_MAX_HOLD = float(os.environ.get("T_MAX_HOLD", "4.0"))

#: How many ticks a cheaper route must stay cheaper before we switch to it.
#: Without this a robot rebuilds its route every tick and its intent becomes
#: unreadable to the peers planning around it.
PLAN_STABLE_TICKS = int(os.environ.get("PLAN_STABLE_TICKS", "3"))

#: How much traffic beyond the reservation horizon pushes a route away, as a
#: fraction of one straight hop.
CONGESTION_WEIGHT = float(os.environ.get("CONGESTION_WEIGHT", "0.35"))
#: Clock skew a peer's claim is widened by on both sides (planning/intervals.py).
PLAN_SKEW_BOUND = float(os.environ.get("PLAN_SKEW_BOUND", "0.3"))
#: Extra gap held behind a robot ahead in the same lane.
PLAN_FOLLOW_GAP = float(os.environ.get("PLAN_FOLLOW_GAP", "0.0"))
#: How much cheaper an alternative must be before hysteresis lets it win.
PLAN_IMPROVEMENT_MARGIN = float(os.environ.get("PLAN_IMPROVEMENT_MARGIN", "0.05"))
#: Obstruction level at or above which a node is impassable, not just costly.
OBSTRUCTION_BLOCK_LEVEL = float(os.environ.get("OBSTRUCTION_BLOCK_LEVEL", "0.7"))
#: How many hops a congestion source's penalty decays over.
CONGESTION_DECAY_HOPS = float(os.environ.get("CONGESTION_DECAY_HOPS", "6.0"))

#: Below this the planner weights charge over speed outright (EnergyState
#: CRITICAL): it will wait rather than detour, and prefer routes near chargers.
#: Sits below JOB_MIN_BATTERY, which is where a bot stops taking new work.
BATTERY_CRITICAL = float(os.environ.get("BATTERY_CRITICAL", "15.0"))

#: Alternative routes kept per job, stored as diffs against the primary.
ROUTE_ALTERNATES = int(os.environ.get("ROUTE_ALTERNATES", "3"))

#: Distance tables cached per source node. Each is 2 bytes per node; the cache
#: is bounded because an unbounded one reached ~33 MB on the real map.
HOPS_CACHE_SIZE = int(os.environ.get("HOPS_CACHE_SIZE", "64"))

#: How long a robot will wait at a junction before giving up on us and driving
#: on by itself. Its firmware owns this number, not us -- it is the robot's
#: patience, and it exists so a robot is never stranded by a network layer that
#: died. Every WAIT we issue has to stay under it, or a hold becomes a robot
#: that leaves halfway through one.
#: Set by `webots/tools/gen_fleet.py` from `control.junction_timeout_s` in
#: fleet.yaml -- the same value the firmware reads -- so the invariant below
#: guards the number the robot actually uses rather than a copy of it. The
#: default here matches the firmware's default for a bot started by hand.
ROBOT_PATIENCE = float(os.environ.get("ROBOT_PATIENCE", "6.0"))

#: A robot commanded to move but still on the same node after this is stalled
#: (PROTOCOL.md §16): replan, then yield, then escalate.
T_STALL = float(os.environ.get("T_STALL", str(6 * T_HB)))


# ---- Reservations (PROTOCOL.md §15) ---------------------------------------------

#: How often a bot tells its neighbours what it is holding. Defaults to T_HB so
#: announcing rides the existing run loop rather than needing a thread of its
#: own. It is a separate name because reservations *want* to be faster than
#: membership: a fresh claim is provisional for one of these periods, so this is
#: also how long a bot waits after replanning before it may move. Lower it when
#: that wait starts costing throughput.
T_ANNOUNCE = float(os.environ.get("T_ANNOUNCE", str(T_HB)))

#: Channel behaviour (bus/rpc.py). A bot that comes back after a restart must be
#: reachable again promptly, not after gRPC's default two-minute backoff.
T_RECONNECT_MIN = float(os.environ.get("T_RECONNECT_MIN", "0.2"))
T_RECONNECT_MAX = float(os.environ.get("T_RECONNECT_MAX", str(T_HB * 2)))
T_KEEPALIVE = float(os.environ.get("T_KEEPALIVE", "10.0"))
T_KEEPALIVE_TIMEOUT = float(os.environ.get("T_KEEPALIVE_TIMEOUT", "3.0"))

#: How long a neighbour's claims stay believable without a fresher announcement.
#: Three periods, matching T_DEAD's three heartbeats: long enough to ride out a
#: lost message, short enough that a crashed bot stops blocking a lane.
RESERVATION_TTL = float(os.environ.get("RESERVATION_TTL", str(3 * T_ANNOUNCE)))

#: How far ahead of itself a bot may claim, in QR hops. Doubles as the test for
#: who needs to hear from us: a peer can only contest a node we hold if it is
#: within this distance of it (reservations/vicinity.py).
RESERVATION_REACH_HOPS = int(os.environ.get("RESERVATION_REACH_HOPS", "8"))


# ---- Location -----------------------------------------------------------------

#: How many recent QR nodes a bot reports, newest first. Two give a heading;
#: a third smooths over a single mis-scan. Carried in heartbeats, relayed in
#: acks, and summarised between leaders (PROTOCOL.md §3.1, §3.2).
NODE_TRAIL_LEN = int(os.environ.get("NODE_TRAIL_LEN", "3"))


# ---- Coherence ------------------------------------------------------------------

class ConfigError(ValueError):
    """Two settings that cannot both be right.

    Every one of these fails *silently* if left alone: the fleet starts, runs,
    and misbehaves in a way that looks like something else entirely. Better to
    refuse to boot and say which pair is wrong.
    """


def validate(node_spacing_cm: int | None = None) -> None:
    """Check the relationships between settings, not just their values.

    Called from `bot.Bot.start()`. Each check names the symptom rather than the
    rule, because the symptom is what someone will be staring at.

    `node_spacing_cm` comes from the loaded map, so the one check that needs to
    know how far apart two nodes are can live here with the others rather than
    somewhere on its own. A bot with no map passes it as None and skips that
    check, which is right: nothing can plan a route without a map anyway.
    """
    problems = []

    if T_MAX_HOLD >= ROBOT_PATIENCE:
        problems.append(
            f"T_MAX_HOLD ({T_MAX_HOLD}s) must be under ROBOT_PATIENCE "
            f"({ROBOT_PATIENCE}s): a robot held longer than it is willing to "
            "wait drives on mid-hold, and cannot tell us apart from a network "
            "layer that has died"
        )
    if RESERVATION_TTL <= T_ANNOUNCE:
        problems.append(
            f"RESERVATION_TTL ({RESERVATION_TTL}s) must exceed T_ANNOUNCE "
            f"({T_ANNOUNCE}s): claims would lapse before the announcement that "
            "renews them, so nothing would ever hold a node"
        )
    if BATTERY_CRITICAL > JOB_MIN_BATTERY:
        problems.append(
            f"BATTERY_CRITICAL ({BATTERY_CRITICAL}%) must not exceed "
            f"JOB_MIN_BATTERY ({JOB_MIN_BATTERY}%): a bot would be routed as if "
            "desperate for charge while still being handed new work"
        )
    if T_DEAD <= T_HB:
        problems.append(
            f"T_DEAD ({T_DEAD}s) must exceed T_HB ({T_HB}s): a leader would evict "
            "bots faster than they report, and the region would never hold a roster"
        )
    if T_STALL <= T_HB:
        problems.append(
            f"T_STALL ({T_STALL}s) must exceed T_HB ({T_HB}s): every robot pausing "
            "for traffic would be escalated as stuck"
        )

    if node_spacing_cm:
        # The window a stationary robot announces has to outlast the drive it is
        # meant to prevent. Two announce periods keeps a claim alive between
        # announcements, which is a different question, and for a long time it
        # was the only one asked -- at the shipped defaults `2 * T_ANNOUNCE` and
        # one hop were both exactly 2000 ms, so a claim expired on the same
        # millisecond a neighbour arrived and strict overlap read that as free.
        # `ReservationSender._hold_ms` takes the larger of the two; this refuses
        # to boot if the settings ever make that arithmetic wrong again.
        from planning.kinematics import DEFAULT_KINEMATICS

        from reservations.claims import claim_window_ms

        traversal_ms = DEFAULT_KINEMATICS.cruise_ms(node_spacing_cm)
        claim_ms = claim_window_ms(node_spacing_cm, DEFAULT_KINEMATICS)
        if claim_ms <= traversal_ms:
            problems.append(
                f"the claim window ({claim_ms:.0f}ms) must exceed one traversal "
                f"({traversal_ms:.0f}ms at {node_spacing_cm}cm spacing): a robot "
                "standing still would stop holding its node before a neighbour "
                "driving at it could arrive, and they would meet there"
            )

    if problems:
        raise ConfigError("; ".join(problems))
