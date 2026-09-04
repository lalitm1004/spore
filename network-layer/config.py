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
    os.path.join(os.path.dirname(__file__), "..", "spore-amr", "shared", "warehouse-layout.json"),
)

# ---- Location -----------------------------------------------------------------

#: How many recent QR nodes a bot reports, newest first. Two give a heading;
#: a third smooths over a single mis-scan. Carried in heartbeats, relayed in
#: acks, and summarised between leaders (PROTOCOL.md §3.1, §3.2).
NODE_TRAIL_LEN = int(os.environ.get("NODE_TRAIL_LEN", "3"))
