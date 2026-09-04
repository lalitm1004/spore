"""Jobs — moving cargo from one QR node to another, and keeping track of it.

WHAT
    * `Job`         — the record (PROTOCOL.md §14.1); `JobLedger` holds them.
    * `Dispatcher`  — leader-side brain: accept a job, hand it to the best
                      free bot, forward it to a nearby region if nobody is
                      free, watch the assignee's heartbeats for progress and
                      trouble, re-queue or escalate.
    * `JobServicer` — `SubmitJob` (order system → any bot).
    * `BotServicer` — `AssignJob` (leader → one of its followers).
    * The bot-side half lives in `bot.Bot`: `handle_assign_job`, the
      `RobotSink` that emits `network-to-robot` commands, and the heartbeat
      fields `job_id` / `cargo_state` that carry the job *with the bot*.

WHERE
    Every bot serves all three services (any bot can become a leader). The
    leader-only paths check the role. `Dispatcher.observe()` is called from
    `bus.heartbeat.RegionServicer.Heartbeat` on every heartbeat a leader
    receives; `Dispatcher.tick()` from the run loop.

WHY  (the rules, as agreed)
    * The **owner** of a job is the leader that *successfully assigned* it —
      not the one that first received it. If region A had nobody free and
      forwarded to B, B's leader owns it and crosses it off.
    * A bot executing a job **drives across regions and migrates** as it
      goes, so the leader that sees it deliver or die is often not the
      owner. The job travels in the bot's heartbeat (`job_id`,
      `cargo_state`), and an observing leader sends a `JobEvent` to every
      other leader; the owner acts on it.
    * Failure **before pickup** (assignee FAULTED / errored / LOW_BATTERY /
      went to CHARGE / vanished while `cargo_state` was still PICKUP) →
      someone still has to collect it: the owner re-queues and re-dispatches.
    * Failure **after pickup** → the cargo is on a dead bot; only a human
      can move it. The owner marks NEEDS_ATTENTION and raises it to the
      control plane (`bot.control_plane`, a callable you can point anywhere).
    * The owner's ledger is replicated to its followers in every
      `HeartbeatAck`, so a new leader inherits it (§7: no single copy).

HOW  (state machine, PROTOCOL.md §14.3)
    PENDING → ASSIGNED → PICKED_UP → DELIVERED (removed from ledger)
       ▲          │           │
       └──fail────┘           └──fail──▶ NEEDS_ATTENTION (control plane)

    "free" = healthy ∧ state IDLE ∧ mission ∈ {IDLE, PARK} ∧ no job
             ∧ not migrating ∧ battery ≥ JOB_MIN_BATTERY
    "best" = highest job priority (election/priority.py: charge bucket
             first, leaders last), then fewest map hops to the pickup node,
             then lower bot_id. The leader is therefore the candidate of
             LAST resort: it only takes a job when no follower is free.
             (If the job leaves the region it abdicates on the way out,
             §4.7 — the successor inherits the ledger from its replica.)
             Nobody observes a leader's heartbeats, so a leader working a
             job observes itself each tick (`Bot._tick_self_job`).
    Forwarding order = regions by map hops from the pickup node, skipping
    ones already tried; each hop repeats the same logic; JOB_MAX_HOPS cap.
    A job nobody could take stays PENDING with the pickup region's leader
    and is retried every T_JOB_RETRY.
"""
from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass, field
from typing import TYPE_CHECKING

import grpc

from config import JOB_MIN_BATTERY, JOB_MAX_HOPS, T_JOB_RETRY, T_HB, T_JOB_EVENT_TTL
from election.priority import is_healthy, job_priority
from peers.table import Peer
from proto import fleet_pb2, fleet_pb2_grpc
from bus.rpc import pool

if TYPE_CHECKING:
    from bot import Bot

log = logging.getLogger(__name__)

PENDING, ASSIGNED, PICKED_UP, DELIVERED, NEEDS_ATTENTION = (
    "PENDING", "ASSIGNED", "PICKED_UP", "DELIVERED", "NEEDS_ATTENTION",
)
#: cargo_state values a bot reports (mirrors the schema's CargoState + our terminal marker)
CS_PICKUP, CS_EN_ROUTE, CS_DROPOFF, CS_DELIVERED = "PICKUP", "EN_ROUTE", "DROPOFF", "DELIVERED"
#: fault strings that mean "this bot cannot finish its job"
FATAL_FAULT_PREFIXES = ("MOTOR_ERROR", "CAMERA_ERROR", "LIDAR_ERROR", "LOCATION_UNKNOWN", "MISC_ERROR", "LOW_BATTERY")

#: "no assignee". Never 0 — bot-0 is a real bot (the first one up.py starts).
NO_BOT = -1


@dataclass
class Job:
    job_id: str
    pickup_node: int
    dropoff_node: int
    owner_region: int = 0
    status: str = PENDING
    assignee: int = NO_BOT
    last_node: int = 0
    reason: str = ""
    updated_at: float = field(default_factory=time.time)

    def to_proto(self) -> fleet_pb2.Job:
        p = fleet_pb2.Job(
            job_id=self.job_id, pickup_node=self.pickup_node, dropoff_node=self.dropoff_node,
            owner_region=self.owner_region, status=self.status,
            last_node=self.last_node, reason=self.reason, updated_at=int(self.updated_at * 1000),
        )
        if self.assignee != NO_BOT:
            p.assignee = self.assignee
        return p

    @classmethod
    def from_proto(cls, p: fleet_pb2.Job) -> "Job":
        return cls(
            job_id=p.job_id, pickup_node=p.pickup_node, dropoff_node=p.dropoff_node,
            owner_region=p.owner_region, status=p.status or PENDING,
            assignee=p.assignee if p.HasField("assignee") else NO_BOT,
            last_node=p.last_node, reason=p.reason,
            updated_at=(p.updated_at / 1000.0) if p.updated_at else time.time(),
        )

    def touch(self) -> None:
        self.updated_at = time.time()


class JobLedger:
    """Thread-safe job_id → Job. The owner's copy is authoritative; followers
    hold a replica delivered in every ack."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._jobs: dict[str, Job] = {}

    def upsert(self, job: Job) -> None:
        with self._lock:
            self._jobs[job.job_id] = job

    def get(self, job_id: str) -> Job | None:
        with self._lock:
            return self._jobs.get(job_id)

    def remove(self, job_id: str) -> Job | None:
        with self._lock:
            return self._jobs.pop(job_id, None)

    def all(self) -> list[Job]:
        with self._lock:
            return list(self._jobs.values())

    def replace(self, jobs: list[Job]) -> None:
        with self._lock:
            self._jobs = {j.job_id: j for j in jobs}

    def __contains__(self, job_id: str) -> bool:
        with self._lock:
            return job_id in self._jobs

    def __len__(self) -> int:
        with self._lock:
            return len(self._jobs)


class Dispatcher:
    """Leader-side job logic. See module docstring for the rules."""

    def __init__(self, bot: Bot) -> None:
        self._bot = bot
        # Guards every Job mutation and the pending-event map. Held only
        # around ledger/state changes — never across an RPC, or a slow
        # ForwardJob would stall heartbeat handling and trigger elections.
        self._lock = threading.RLock()
        #: (job_id, status) → (event, first_sent_at, next_send_at): events
        #: about jobs we do not own, re-sent until an owner acks them.
        self._pending_events: dict[tuple[str, str], tuple[Job, float, float]] = {}

    # ------------------------------------------------------------------ intake

    def submit(self, job: Job) -> fleet_pb2.JobAck:
        """`SubmitJob` arrived here. Route it to the pickup node's region
        leader (which may be us), who then tries to take it."""
        from election.bully import Role

        bot = self._bot
        ls = bot.leadership()
        if ls.role != Role.LEADER:
            if not ls.leader_address:
                return fleet_pb2.JobAck(accepted=False, note="no leader known yet; retry")
            return self._rpc_submit(ls.leader_address, job)

        target_region = bot.map.region_of(job.pickup_node) or bot.region_id
        if target_region != bot.region_id:
            dest = bot.peer_table.get_leader(target_region)
            if dest is not None:
                ack = self._rpc_submit(dest.address, job)
                if ack.accepted:
                    return ack
            # Unknown / unreachable pickup-region leader: we take it ourselves.
            log.info("bot-%d: pickup region %d has no reachable leader; taking job %s here",
                     bot.bot_id, target_region, job.job_id)
        return self.take(job, tried=[], hops=0, keep=True)

    def _rpc_submit(self, address: str, job: Job) -> fleet_pb2.JobAck:
        try:
            return pool.stub(address, fleet_pb2_grpc.JobServiceStub).SubmitJob(
                job.to_proto(), timeout=T_HB * 4, metadata=self._bot.rpc_metadata(), wait_for_ready=True)
        except grpc.RpcError as e:
            return fleet_pb2.JobAck(accepted=False, note=f"forward failed: {e.code()}")

    # ------------------------------------------------------------------ owning

    def take(self, job: Job, tried: list[int], hops: int, keep: bool) -> fleet_pb2.JobAck:
        """Try to become this job's owner: assign locally, else forward.

        `keep`: if nobody anywhere can take it, do we keep it PENDING in our
        ledger (the pickup region's leader does) or hand it back (a forwardee
        does, by answering accepted=False)?
        """
        bot = self._bot
        with self._lock:
            existing = bot.jobs.get(job.job_id)
            if existing is not None and existing.status != PENDING:
                return fleet_pb2.JobAck(accepted=True, assignee=existing.assignee,
                                        owner_region=existing.owner_region, note="duplicate: already tracked")

        assignee = self._assign_local(job)          # RPCs — outside the lock
        if assignee is not None:                    # `is not None`: bot-0 is a valid assignee
            with self._lock:
                job.owner_region = bot.region_id
                job.assignee = assignee
                job.status = ASSIGNED
                job.reason = ""
                job.touch()
                bot.jobs.upsert(job)
            log.info("bot-%d: job %s assigned to bot-%d (pickup %d → dropoff %d)",
                     bot.bot_id, job.job_id, assignee, job.pickup_node, job.dropoff_node)
            return fleet_pb2.JobAck(accepted=True, assignee=assignee, owner_region=bot.region_id)

        ack = self._forward(job, tried + [bot.region_id], hops)   # RPCs — outside the lock
        if ack.accepted:
            # If this was a job we had queued from an earlier attempt, it now
            # belongs to the region that took it — drop our stale copy.
            with self._lock:
                bot.jobs.remove(job.job_id)
            return ack

        if keep:
            with self._lock:
                job.owner_region = bot.region_id
                job.status = PENDING
                job.assignee = NO_BOT
                job.touch()
                bot.jobs.upsert(job)
            log.warning("bot-%d: nobody free anywhere for job %s; queued (retry every %.0fs)",
                        bot.bot_id, job.job_id, T_JOB_RETRY)
            return fleet_pb2.JobAck(accepted=True, owner_region=bot.region_id, note="queued")
        return fleet_pb2.JobAck(accepted=False, note="nobody free")

    # ------------------------------------------------------------------ local assignment

    def _is_free(self, p: Peer) -> bool:
        bot = self._bot
        return (
            is_healthy(p.state)
            and p.state == "IDLE"
            and p.mission in ("IDLE", "PARK", "")
            and not p.job_id
            and not p.fault
            and p.battery >= JOB_MIN_BATTERY
            and p.bot_id not in bot.migrating_out
        )

    def _candidates(self, job: Job) -> list[Peer]:
        """Free bots by job priority (charge first, leader last), then by map
        distance to the pickup, then id."""
        bot = self._bot
        m = bot.map
        free = [p for p in bot.peer_table.all_peers() if self._is_free(p)]
        if bot.is_free_for_job():
            free.append(bot.self_peer())

        def key(p: Peer):
            return (
                -job_priority(healthy=True, battery_pct=p.battery, is_leader=(p.bot_id == bot.bot_id), has_job=False),
                m.distance(p.latest_node_id, job.pickup_node),
                p.bot_id,
            )

        free.sort(key=key)
        return free

    def _assign_local(self, job: Job) -> int | None:
        """Offer the job to free bots, best first (ourselves last).
        Returns the bot_id that accepted, or None."""
        bot = self._bot
        for p in self._candidates(job):
            if p.bot_id == bot.bot_id:
                # Last resort: take it ourselves. No RPC — same process.
                if bot.handle_assign_job(job):
                    log.info("bot-%d: no follower free; leader taking job %s itself", bot.bot_id, job.job_id)
                    return bot.bot_id
                continue
            try:
                ack = pool.stub(p.address, fleet_pb2_grpc.BotServiceStub).AssignJob(
                    job.to_proto(), timeout=T_HB * 2, metadata=bot.rpc_metadata())
            except grpc.RpcError as e:
                log.warning("bot-%d: AssignJob to bot-%d failed: %s", bot.bot_id, p.bot_id, e.code())
                continue
            if ack.accepted:
                # Reflect it in the roster immediately so a second job in the
                # same tick does not pick the same bot.
                p.job_id = job.job_id
                p.cargo_state = CS_PICKUP
                return p.bot_id
        return None

    # ------------------------------------------------------------------ forwarding

    def _forward(self, job: Job, tried: list[int], hops: int) -> fleet_pb2.JobAck:
        bot = self._bot
        if hops >= JOB_MAX_HOPS:
            return fleet_pb2.JobAck(accepted=False, note="hop limit")
        leaders = [ld for ld in bot.peer_table.all_leaders() if ld.region_id not in tried]
        leaders.sort(key=lambda ld: (bot.map.region_distance(job.pickup_node, ld.region_id), ld.region_id))
        for ld in leaders:
            try:
                ack = pool.stub(ld.address, fleet_pb2_grpc.LeaderExchangeServiceStub).ForwardJob(
                    fleet_pb2.ForwardJobReq(job=job.to_proto(), tried_regions=tried, hops=hops + 1),
                    timeout=T_HB * 6, metadata=bot.rpc_metadata())
            except grpc.RpcError as e:
                log.debug("bot-%d: ForwardJob to region %d failed: %s", bot.bot_id, ld.region_id, e.code())
                continue
            if ack.accepted:
                log.info("bot-%d: job %s forwarded to region %d (assignee bot-%d)",
                         bot.bot_id, job.job_id, ack.owner_region, ack.assignee)
                return ack
            tried = tried + [ld.region_id]
        return fleet_pb2.JobAck(accepted=False, note="no region could take it")

    def handle_forward(self, req: fleet_pb2.ForwardJobReq) -> fleet_pb2.JobAck:
        """Another leader asks us to take a job. We never keep it if we can't
        assign — it stays with the region that asked."""
        from election.bully import Role

        if self._bot.role != Role.LEADER:
            return fleet_pb2.JobAck(accepted=False, note="not a leader")
        return self.take(Job.from_proto(req.job), list(req.tried_regions), req.hops, keep=False)

    # ------------------------------------------------------------------ periodic

    def tick(self) -> None:
        """Re-send unacked events (any role); retry PENDING jobs we own (leader)."""
        from election.bully import Role

        self._flush_events()
        bot = self._bot
        if bot.role != Role.LEADER:
            return
        now = time.time()
        with self._lock:
            due = [j for j in bot.jobs.all()
                   if j.owner_region == bot.region_id and j.status == PENDING and now - j.updated_at >= T_JOB_RETRY]
            for j in due:
                j.touch()
        for job in due:
            self.take(job, tried=[], hops=0, keep=True)

    # ------------------------------------------------------------------ observation

    def observe(self, prev: Peer | None, cur: Peer) -> None:
        """Called on every heartbeat a leader receives. Turns what the bot
        reports into job events. Nothing here sends a message to the bot."""
        if not cur.job_id:
            # The job vanished from the bot's reports. After DELIVERED that is
            # normal (it was acked and cleared). Otherwise the bot *dropped*
            # it — it abandoned before pickup, or its bridge reset — and the
            # owner must hear about it or the job stays ASSIGNED forever.
            if prev is not None and prev.job_id and prev.cargo_state != CS_DELIVERED:
                self._fail(prev, "bot dropped the job")
            return
        prev_cs = prev.cargo_state if prev else ""

        if cur.cargo_state == CS_DELIVERED:
            if prev_cs != CS_DELIVERED:
                self._report(cur, DELIVERED, "")
            return

        if cur.cargo_state in (CS_EN_ROUTE, CS_DROPOFF) and prev_cs in ("", CS_PICKUP):
            self._report(cur, PICKED_UP, "")

        trouble = self._trouble(cur)
        if trouble:
            self._fail(cur, trouble)

    @staticmethod
    def _trouble(p: Peer) -> str:
        if not is_healthy(p.state):
            return f"state {p.state}"
        if p.fault and p.fault.startswith(FATAL_FAULT_PREFIXES):
            return f"fault {p.fault}"
        if p.mission == "CHARGE":
            return "went to charge"
        return ""

    def on_peer_evicted(self, p: Peer) -> None:
        """A follower stopped heartbeating (T_DEAD). If it had a job, that job
        is in trouble."""
        if p.job_id:
            self._fail(p, "heartbeat lost")

    def _fail(self, p: Peer, reason: str) -> None:
        # Pre-pickup: the cargo is still on the shelf → re-queue.
        # Post-pickup: the cargo is on a dead bot → humans.
        if p.cargo_state in ("", CS_PICKUP):
            self._report(p, PENDING, f"pre-pickup failure: {reason}")
        else:
            self._report(p, NEEDS_ATTENTION, f"post-pickup failure: {reason}")

    def _report(self, p: Peer, status: str, reason: str) -> None:
        """Apply locally if we own the job, otherwise queue an event for the
        owner and start sending it."""
        bot = self._bot
        event = Job(job_id=p.job_id, pickup_node=0, dropoff_node=0, status=status,
                    assignee=p.bot_id, last_node=p.latest_node_id, reason=reason)
        with self._lock:
            owned = bot.jobs.get(p.job_id)
            if owned is not None and owned.owner_region == bot.region_id:
                self._apply(owned, event)
                return
            key = (event.job_id, event.status)
            if key not in self._pending_events:
                self._pending_events[key] = (event, time.time(), 0.0)
        log.info("bot-%d: observed job %s %s (bot-%d @ node %d) — notifying leaders",
                 bot.bot_id, p.job_id, status, p.bot_id, p.latest_node_id)
        self._flush_events()

    def _flush_events(self) -> None:
        """Send every due pending event to every leader we know; drop an
        event once some leader answers owned=true, or after T_JOB_EVENT_TTL.
        This is what makes cross-region job tracking survive an owner
        leader we have not met yet, or one that is briefly unreachable."""
        bot = self._bot
        now = time.time()
        with self._lock:
            due = [(k, ev, first) for k, (ev, first, nxt) in self._pending_events.items() if now >= nxt]
        for key, event, first in due:
            if now - first > T_JOB_EVENT_TTL:
                log.error("bot-%d: giving up on job event %s after %.0fs — no owner found",
                          bot.bot_id, key, T_JOB_EVENT_TTL)
                with self._lock:
                    self._pending_events.pop(key, None)
                continue
            acked = False
            for ld in bot.peer_table.all_leaders():
                try:
                    ack = pool.stub(ld.address, fleet_pb2_grpc.LeaderExchangeServiceStub).JobEvent(
                        event.to_proto(), timeout=T_HB * 2, metadata=bot.rpc_metadata())
                except grpc.RpcError as e:
                    log.debug("bot-%d: JobEvent to region %d failed: %s", bot.bot_id, ld.region_id, e.code())
                    continue
                if ack.owned:
                    acked = True
                    break
            with self._lock:
                if acked:
                    self._pending_events.pop(key, None)
                elif key in self._pending_events:
                    self._pending_events[key] = (event, first, now + T_JOB_RETRY)

    def pending_event_count(self) -> int:
        with self._lock:
            return len(self._pending_events)

    def handle_event(self, p: fleet_pb2.Job) -> bool:
        """A JobEvent from another leader. Returns True iff we own the job
        (and therefore applied the event)."""
        event = Job.from_proto(p)
        with self._lock:
            owned = self._bot.jobs.get(event.job_id)
            if owned is None or owned.owner_region != self._bot.region_id:
                return False
            self._apply(owned, event)
            return True

    def _apply(self, job: Job, event: Job) -> None:
        """Owner-side state transition. Caller holds `self._lock`."""
        bot = self._bot
        # A failure report is only about the bot it names. Once we have
        # re-assigned the job, a late report about the *previous* assignee
        # (e.g. its first job-less heartbeat) must not re-queue it again.
        if event.status in (PENDING, NEEDS_ATTENTION) and job.assignee not in (NO_BOT, event.assignee):
            log.debug("bot-%d: ignoring stale %s for job %s from bot-%d (now bot-%d)",
                      bot.bot_id, event.status, job.job_id, event.assignee, job.assignee)
            return
        if event.status == DELIVERED:
            bot.jobs.remove(job.job_id)
            log.info("bot-%d: job %s DELIVERED by bot-%d — crossed off", bot.bot_id, job.job_id, event.assignee)
        elif event.status == PICKED_UP:
            if job.status == ASSIGNED:
                job.status = PICKED_UP
                job.last_node = event.last_node
                job.touch()
        elif event.status == PENDING:
            if job.status in (ASSIGNED, PENDING):
                log.warning("bot-%d: job %s failed before pickup (%s); re-queuing",
                            bot.bot_id, job.job_id, event.reason)
                job.status = PENDING
                job.assignee = NO_BOT
                job.reason = event.reason
                job.last_node = event.last_node
                job.updated_at = 0.0  # retry on the very next tick
        elif event.status == NEEDS_ATTENTION:
            if job.status != NEEDS_ATTENTION:
                job.status = NEEDS_ATTENTION
                job.reason = event.reason
                job.last_node = event.last_node
                job.touch()
                bot.control_plane({
                    "type": "JOB_NEEDS_ATTENTION", "job_id": job.job_id, "assignee": event.assignee,
                    "last_node": event.last_node, "reason": event.reason,
                    "pickup_node": job.pickup_node, "dropoff_node": job.dropoff_node,
                })


# ---------------------------------------------------------------------------
# gRPC adapters
# ---------------------------------------------------------------------------

class JobServicer(fleet_pb2_grpc.JobServiceServicer):
    """`SubmitJob` — served by every bot; routes to the right leader."""

    def __init__(self, bot: Bot) -> None:
        self._bot = bot

    def SubmitJob(self, request: fleet_pb2.Job, context: grpc.ServicerContext):
        job = Job.from_proto(request)
        if not job.job_id:
            return fleet_pb2.JobAck(accepted=False, note="job_id required")
        return self._bot.dispatcher.submit(job)


class BotServicer(fleet_pb2_grpc.BotServiceServicer):
    """`AssignJob` — a leader hands us a job. Accept only if we are free."""

    def __init__(self, bot: Bot) -> None:
        self._bot = bot

    def AssignJob(self, request: fleet_pb2.Job, context: grpc.ServicerContext):
        job = Job.from_proto(request)
        accepted = self._bot.handle_assign_job(job)
        ack = fleet_pb2.JobAck(accepted=accepted, note="" if accepted else "busy")
        if accepted:
            ack.assignee = self._bot.bot_id
        return ack
