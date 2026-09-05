"""The fleet's priorities — three separate questions, three separate numbers.

    election priority   who should LEAD                      (`compute`)
    job priority        who should get the next JOB          (`job_priority`)
    yield priority      who has RIGHT OF WAY in the lanes    (`yield_priority`)
    succession          who a leader hands off TO            (`peers.table.PeerTable.best_successor`)

They are deliberately not one number: a fully charged bot is the best
candidate for a job *and* for leadership, but we want it doing the job and
the bot that stays put doing the leading — so the job ranking puts leaders
last while the election ranking ignores jobs entirely.

WHAT — election priority
    A single integer per bot, carried in every heartbeat and leader
    heartbeat, compared with plain `>` (ties broken by bot_id, which is folded
    into the number). Higher wins.

WHERE
    `bot.Bot._tick_priority()` recomputes it each run-loop tick and pushes it
    into the heartbeat payloads and the election state machine. Everything
    that compares priorities (`election.bully._outranks`, the same-region
    conflict rule in `bot.py`) just compares the integers.

WHY
    `priority = bot_id` was deterministic but meant the highest-numbered bot
    led forever, even at 3% battery. A leader that is about to die is the
    worst possible leader. We want:

      1. never an unhealthy leader (FAULTED / COMMS_LONG cannot win),
      2. prefer charged bots,
      3. deterministic ties,
      4. no flapping when two bots straddle a battery threshold.

HOW
    priority = healthy·10000 + battery_bucket·100 + bot_id

      healthy         1 or 0 — dominates everything else (goal 1).
      battery_bucket  0..4 in 20-point steps (goal 2). Buckets, not raw
                      percent, so a bot at 61% and one at 59% do not
                      reorder every reading.
      bot_id          final tiebreak (goal 3). bot_id must be < 100 for the
                      terms not to overlap; up.py allocates them sequentially.

    Flapping (goal 4): a *sitting leader* computes its bucket with LEADER_SLACK
    points of grace — at 59% it still advertises bucket 3 until it falls
    below 55%. A challenger at 59% advertises bucket 2. So leadership only
    changes hands after a real swing, never oscillates at a boundary, and
    the comparison itself stays a plain `>` (which the bootstrap ordering
    depends on — see PROTOCOL.md §5.6).

    Tenure (PROTOCOL.md §5.6): election priority alone would let a fully
    charged high-id bot lead forever. `bot.Bot._tick_tenure` rotates
    leadership to the best free successor after T_LEADER_TENURE — a rule in
    the run loop, not a term in the number, so the number stays comparable.

WHAT — job priority
    Rank for assignment among *free* bots. Charged bots first (buckets, then
    raw percent), and the leader last by a penalty that dominates everything
    (it should stay put and lead). Map distance breaks ties in the
    dispatcher, so within the same charge bucket the nearest bot wins.

WHAT — yield priority
    Right of way, for the robot layer to enforce at YI nodes: a free bot (0)
    yields to one heading to a pickup (1), which yields to one carrying
    cargo (2). The leader computes it from job state and distributes it in
    every roster record; the network layer never uses it itself.
"""
from __future__ import annotations

HEALTHY_WEIGHT = 10_000
# These four define what a priority *number means*, and every bot compares its
# number against every other bot's. They are therefore protocol, not a
# deployment knob, and deliberately not in config.py: a fleet where two bots
# booted with different bucket sizes would be comparing incomparable numbers,
# and the symptom -- an election that never settles -- would look nothing like
# the cause. Change them here, for everyone, or not at all (PROTOCOL.md §5.6).
BUCKET_WEIGHT = 100
BUCKET_SIZE = 20.0  # battery percent per bucket → buckets 0..4
MAX_BUCKET = 4
LEADER_SLACK = 5.0  # percent of grace a sitting leader gets before dropping a bucket

#: FSM states in which a bot may not lead or take part in elections (§5.1).
UNHEALTHY_STATES = frozenset({"FAULTED", "COMMS_LONG"})


def is_healthy(state: str) -> bool:
    return state not in UNHEALTHY_STATES


def battery_bucket(battery_pct: float, *, sitting_leader: bool = False) -> int:
    """20-point buckets, clamped to 0..MAX_BUCKET.

    A sitting leader gets LEADER_SLACK points added before bucketing, which is
    the hysteresis described in the module docstring.
    """
    pct = battery_pct + (LEADER_SLACK if sitting_leader else 0.0)
    return max(0, min(MAX_BUCKET, int(pct // BUCKET_SIZE)))


def compute(
    *, healthy: bool, battery_pct: float, bot_id: int, sitting_leader: bool = False
) -> int:
    """The election priority integer. See module docstring for the formula and why."""
    return (
        (HEALTHY_WEIGHT if healthy else 0)
        + battery_bucket(battery_pct, sitting_leader=sitting_leader) * BUCKET_WEIGHT
        + bot_id
    )


LEADER_JOB_PENALTY = 100_000  # dominates any battery term → leaders always rank last


def job_priority(*, healthy: bool, battery_pct: float, is_leader: bool, has_job: bool) -> int:
    """Rank for job assignment; higher = likelier. Negative = not a candidate."""
    if not healthy or has_job:
        return -1
    return (
        battery_bucket(battery_pct) * BUCKET_WEIGHT
        + int(min(100.0, max(0.0, battery_pct)))
        - (LEADER_JOB_PENALTY if is_leader else 0)
    )


YIELD_FREE, YIELD_TO_PICKUP, YIELD_CARRYING = 0, 1, 2


def yield_priority(*, has_job: bool, carrying: bool) -> int:
    """Right of way: lower yields to higher."""
    if carrying:
        return YIELD_CARRYING
    return YIELD_TO_PICKUP if has_job else YIELD_FREE
