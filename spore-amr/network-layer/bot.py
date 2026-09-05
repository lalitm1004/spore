"""The bot process: one `Bot` per robot, tying every protocol piece together.

WHAT
    * The gRPC server hosting every service behind the virtual-network
      interceptor.
    * The **run loop** — the bot's brain. Every T_HB it:
        1. pulls the robot's physical state (QR position, battery, faults)
           from a `RobotSource` and mirrors it into the fields that go out
           in heartbeats;
        2. claims the node the robot is on and announces it to the bots near
           enough to contest it — bot to bot, no leader in the path (§15);
        3. recomputes election priority from health and battery;
        4. abdicates if it is an unhealthy leader;
        5. lets the `Migrator` reconcile physical region vs. membership;
        6. does leader housekeeping (evict dead followers, expire stale
           migration records).
    * Role transitions (`become_leader` / `become_follower`) and the single
      lock that makes them atomic.
    * Payload builders (`heartbeat_payload`, `roster_ack`, …) so the wire
      shape is defined in exactly one place.

WHERE
    Entry point: `uv run bot.py` (Dockerfile ENTRYPOINT). Identity comes from
    env vars via `config`. Everything else — `bus.heartbeat`,
    `bus.leader_exchange`, `bus.migration`, `election.bully` — holds a
    reference to this object and calls back into it.

WHY
    The protocol handlers are reactive (they run when an RPC arrives) and the
    senders are periodic, but *decisions* — "my region changed", "I am now
    FAULTED", "my battery dropped a bucket" — need a place that sees the
    robot's state and the network's state together. That is the run loop.

    Concurrency: `role`, `leader_address`, `leader_bot_id` are read by the
    run loop, two sender threads, and every gRPC worker. Single assignments
    are atomic under the GIL but *compound* checks ("if I'm a follower, use
    my leader address") are not, so all three live in one immutable
    `Leadership` snapshot swapped under an RLock. Readers take the snapshot
    once; writers go through the two `become_*` methods.

HOW — lifecycle (PROTOCOL.md §4.1)
    start()
      ├─ bind gRPC server
      ├─ healthy?  yes → become_leader()   (everyone starts as leader; the
      │            │                        leader exchange finds conflicts
      │            │                        and the lower priority yields)
      │            no  → follower with no leader; the heartbeat sender
      │                  probes PEER_LEADERS until someone tells it who leads
      └─ run loop until SIGTERM → graceful_shutdown()
"""
from __future__ import annotations

import logging
import queue
import signal
import threading
import time
from collections import deque
from concurrent import futures
from dataclasses import dataclass
from typing import Protocol

import grpc

import config
from election import priority as prio
from election.bully import BullyElection, Role, _outranks
from election.server import ElectionServicer
from bus.heartbeat import HeartbeatSender, RegionServicer
from bus.leader_exchange import LeaderExchangeSender, LeaderExchangeServicer
from bus.migration import Migrator, MigrationJoinServicer
from bus.policy import VirtualNetworkInterceptor, rpc_metadata
from bus.rpc import pool
from bus.jobs import (
    Dispatcher, JobLedger, Job, JobServicer, BotServicer,
    CS_PICKUP, CS_EN_ROUTE, CS_DROPOFF, CS_DELIVERED,
)
from bus.admin import AdminServicer
from bus.control_plane import ControlPlaneServicer
from peers.table import PeerTable, Peer, Leader, Ledger
from planning import decide as decide_module
from planning import traffic as traffic_module
from planning.decide import Decision, DecisionKind, Query
from planning.geometry import heading_from_radians
from planning.graph import Graph
from planning.planner import Planner
from planning.topology import Topology
from planning.types import Goal, Obstruction, Request, Reservation, SelfState
from planning.types import from_env as planning_config
from reservations import now_ms
from reservations.ledger import ReservationLedger
from reservations.sender import ReservationSender
from reservations.server import ReservationServicer
from planning.robot_service import RobotNetworkServicer
from proto import controlplane_pb2_grpc, fleet_pb2, fleet_pb2_grpc, robot_pb2_grpc
from warehouse.map import WarehouseMap

# Messages already carry "bot-N:"; one process is one bot, so no prefix here
# (a fixed prefix mislabels every bot as bot-0 when several run in one test).
logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
log = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Robot interface — the bridge between the physical robot and the network layer
# ---------------------------------------------------------------------------

@dataclass
class RobotState:
    """One snapshot of the robot, shaped like `robot-to-network.schema.json`.

    `region_id` is whatever the last scanned QR code said (the ground truth
    for where the robot physically is); 0 means "unknown / no scan yet".
    """
    latest_node_id: int = 0
    region_id: int = 0
    battery: float = 100.0
    state: str = "IDLE"
    mission: str = "PARK"
    fault: str = ""
    #: When mission == CARGO: which cargo and how far along (schema CargoState).
    job_id: str = ""
    cargo_state: str = ""


@dataclass
class RobotCommand:
    """One `network-to-robot` message: go to `target_node_id` with this mission.
    `mission` is the schema's Mission object, e.g.
    `{"type": "CARGO", "cargo": {"cargo_id": ..., "state": "PICKUP"}}`."""
    target_node_id: int
    mission: dict


class RobotSink(Protocol):
    """Where the bot *sends* commands to the robot — the outbound twin of
    `RobotSource`. Implement for real hardware."""

    def send(self, cmd: RobotCommand) -> None: ...


class QueueRobotSink:
    """Default `RobotSink`: commands pile up in a queue for the robot bridge
    (or a test) to drain."""

    def __init__(self) -> None:
        self._q: queue.Queue[RobotCommand] = queue.Queue()

    def send(self, cmd: RobotCommand) -> None:
        self._q.put(cmd)

    def pop(self) -> RobotCommand | None:
        try:
            return self._q.get_nowait()
        except queue.Empty:
            return None


class RobotSource(Protocol):
    """Where the run loop reads robot state from. Implement for real hardware."""

    def poll(self) -> RobotState | None:
        """Return the newest state, or None if nothing new. Must not block —
        called once per run-loop tick."""
        ...


class LatestRobotState:
    """Default `RobotSource`: a slot holding the newest report, and only that.

    A slot rather than a queue, and the difference matters. The run loop drains
    one item per tick (`T_HB`, a second), while a robot on a stream reports
    every marker and every status beat. A FIFO under that would grow without
    bound and hand the loop positions the robot left minutes ago -- a bot
    steadily more confident about somewhere the robot no longer is.

    Nothing downstream wants the ones in between. `latest_node_id` is a
    position, not an event log, and the trail it feeds only records *distinct*
    nodes anyway. So a later report simply replaces an unread earlier one.

    The one thing that buys is worth naming: a *level* survives collapsing and
    an *edge* does not. A robot reports "carrying, at the dropoff" on every tick
    until it is not, so the fleet sees it however often it reads. Something that
    were true for a single report would be missed, which is why nothing on this
    wire is shaped that way.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._latest: RobotState | None = None
        #: How many reports were replaced before the loop could read them.
        #: Not an error -- it is the design -- but a number worth being able
        #: to see if a fleet ever looks like it is lagging its robots.
        self.superseded = 0

    def push(self, state: RobotState) -> None:
        with self._lock:
            if self._latest is not None:
                self.superseded += 1
            self._latest = state

    def poll(self) -> RobotState | None:
        with self._lock:
            state, self._latest = self._latest, None
            return state


# ---------------------------------------------------------------------------
# Leadership snapshot
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class Leadership:
    """Immutable view of "who leads me". Swapped atomically by `become_*`.

    `since` is monotonic time the role began; `leader_settled()` uses it to
    tell a freshly self-declared bootstrap leader from an established one.
    """
    role: Role
    leader_id: int | None
    leader_address: str | None
    since: float


# ---------------------------------------------------------------------------
# Bot
# ---------------------------------------------------------------------------

class Bot:
    def __init__(self, robot_source: RobotSource | None = None, robot_sink: RobotSink | None = None) -> None:
        # ---- Identity (PROTOCOL.md §2) ----
        self.bot_id: int = config.BOT_ID
        self.region_id: int = config.REGION_ID       # region we are a *member* of
        self.desired_region_id: int | None = None    # region the robot is physically in
        self.address: str = config.OWN_ADDRESS
        self.peer_leaders: list[str] = list(config.PEER_LEADERS)
        self.priority: int = self.bot_id             # recomputed each tick (see _tick_priority)

        # ---- Robot state, mirrored from RobotSource each tick ----
        self.state: str = "IDLE"
        self.battery: float = 100.0
        self.latest_node_id: int = 0
        #: Recent QR nodes, newest first (see config.NODE_TRAIL_LEN).
        self.node_trail: deque[int] = deque(maxlen=config.NODE_TRAIL_LEN)
        self.mission: str = "PARK"
        self.fault: str = ""

        # ---- Network state ----
        self._lock = threading.RLock()
        self._leadership = Leadership(Role.LEADER, None, None, time.monotonic())
        self.last_ack_at: float = 0.0                # follower: last successful heartbeat
        self.peer_table = PeerTable()
        self.migrating_out = Ledger()                # leader: bot_id → dest region
        self.pending_incoming = Ledger()             # leader: bot_id → source region

        # ---- Jobs (PROTOCOL.md §14) ----
        self.map = WarehouseMap.load(config.WAREHOUSE_MAP)
        self.jobs = JobLedger()                      # leader: owned jobs; follower: replica from acks
        self.current_job: Job | None = None          # the job *this* bot is executing
        self.cargo_state: str = ""                   # PICKUP → EN_ROUTE → DROPOFF → DELIVERED
        self._delivered_reported = False
        self._last_self_peer: Peer | None = None     # leader self-observation (see _tick_self_job)
        #: Where NEEDS_ATTENTION escalations go. Replace with a real control
        #: plane client; the default logs at ERROR and keeps them in `alerts`.
        self.control_plane = self._default_control_plane
        self.alerts: list[dict] = []

        # ---- Collaborators ----
        self.election = BullyElection(
            bot_id=self.bot_id, priority=self.priority, address=self.address,
            on_leader_elected=self._on_leader_elected,
            on_become_leader=self.become_leader,
            metadata_fn=self.rpc_metadata,
            healthy_fn=self.is_healthy,
        )
        self.migrator = Migrator(self)
        self.dispatcher = Dispatcher(self)
        self._robot_source = robot_source or LatestRobotState()
        self._robot_sink = robot_sink or QueueRobotSink()
        # Reservations are the one channel that does not go through a leader
        # (PROTOCOL.md §7, §15), so every bot holds its own ledger and talks
        # straight to the neighbours it finds in the roster.
        self.ledger = ReservationLedger(
            self.bot_id,
            announce_period_ms=int(config.T_ANNOUNCE * 1000),
            ttl_ms=int(config.RESERVATION_TTL * 1000),
        )
        self._reservations = ReservationSender(self)

        # Pathfinding (PROTOCOL.md §16). The graph is built on the map already
        # loaded above rather than reading it again -- one copy of the floor.
        self.planning = planning_config()
        self.graph: Graph | None = None
        self.topology: Topology | None = None
        self.planner: Planner | None = None
        if getattr(self.map, "n", 0):
            self.graph = Graph(self.map, hops_cache_size=config.HOPS_CACHE_SIZE)
            self.topology = Topology(self.graph)
            self.planner = Planner(self.graph, bot_id=self.bot_id, config=self.planning)

        #: Where the robot is being sent, if anywhere. Set by the job layer;
        #: the route to it is worked out per query, not stored as a command.
        self.nav_goal: Goal | None = None
        self._last_target: int | None = None
        #: Blockages the planner routes around, keyed by node. Filled from what
        #: a robot actually reports -- `Fault.warning.obstacle` on the robot
        #: link -- rather than pushed in by an operator, which is how it worked
        #: until the wire carried the node.
        self.obstructions: dict[int, float] = {}
        self._nav_node: int = 0
        self._nav_since: float = time.monotonic()
        self._nav_strikes: int = 0
        self._hb_sender = HeartbeatSender(self)
        self._leader_exchange = LeaderExchangeSender(self)
        self._grpc_server: grpc.Server | None = None
        self._shutdown = threading.Event()

    # =====================================================================
    # Leadership: snapshot, properties, transitions
    # =====================================================================

    def leadership(self) -> Leadership:
        """Atomic view of role + leader. Take it once per decision."""
        with self._lock:
            return self._leadership

    # Properties keep call sites (and tests) simple while still routing every
    # write through the lock. Setting `role` directly is a test convenience;
    # production code uses become_leader / become_follower.
    @property
    def role(self) -> Role:
        return self._leadership.role

    @role.setter
    def role(self, value: Role) -> None:
        with self._lock:
            ls = self._leadership
            self._leadership = Leadership(value, ls.leader_id, ls.leader_address, time.monotonic())

    @property
    def leader_address(self) -> str | None:
        return self._leadership.leader_address

    @leader_address.setter
    def leader_address(self, value: str | None) -> None:
        with self._lock:
            ls = self._leadership
            self._leadership = Leadership(ls.role, ls.leader_id, value, ls.since)

    @property
    def leader_bot_id(self) -> int | None:
        return self._leadership.leader_id

    def become_leader(self) -> None:
        """We lead this region now: stop following, start the leader exchange."""
        with self._lock:
            log.info("bot-%d: became leader of region %d", self.bot_id, self.region_id)
            self._leadership = Leadership(Role.LEADER, None, None, time.monotonic())
        self._hb_sender.stop()
        self._leader_exchange.start()

    def become_follower(self, leader_id: int, leader_addr: str) -> None:
        """Follow `leader_id`: stop the leader exchange, start heartbeating."""
        if leader_id == self.bot_id or leader_addr == self.address:
            # "Follow yourself" is what a redirect chain says when it comes
            # back around to us: it means *we* are the leader. Never heartbeat
            # our own server — that is an infinite self-redirect loop.
            self.become_leader()
            return
        with self._lock:
            log.info("bot-%d: became follower, leader is bot-%d at %s", self.bot_id, leader_id, leader_addr)
            self._leadership = Leadership(Role.FOLLOWER, leader_id, leader_addr, time.monotonic())
        self._leader_exchange.stop()
        self._hb_sender.start()

    def retarget(self, leader_id: int, leader_addr: str) -> None:
        """Switch which leader we heartbeat WITHOUT restarting the sender —
        for redirects handled inside the sender's own loop, which then just
        continues at its normal 1 Hz pace. Restarting the thread there let a
        redirect ping-pong run at RPC speed."""
        if leader_id == self.bot_id or leader_addr == self.address:
            self.become_leader()
            return
        with self._lock:
            log.info("bot-%d: now following bot-%d at %s", self.bot_id, leader_id, leader_addr)
            self._leadership = Leadership(Role.FOLLOWER, leader_id, leader_addr, time.monotonic())

    def _on_leader_elected(self, leader_id: int, leader_addr: str, leader_priority: int | None = None) -> None:
        """Election callback: a Coordinator named `leader_id` (possibly us).

        A *stale* Coordinator is possible: while we were paused or partitioned
        the others elected someone, and the message was waiting in our socket
        when we came back. If we are a healthy leader that outranks the named
        bot, the conflict rule (§5.4) — not that old message — decides, so we
        ignore it; the leader exchange makes the other side yield.
        """
        if leader_id == self.bot_id:
            self.become_leader()
            return
        if (leader_priority is not None and self.role == Role.LEADER and self.is_healthy()
                and not self.election.departing
                and _outranks(self.priority, self.bot_id, leader_priority, leader_id)):
            log.info("bot-%d: ignoring stale Coordinator from bot-%d (we outrank it); conflict rule will settle it",
                     self.bot_id, leader_id)
            return
        self.become_follower(leader_id, leader_addr)

    def leader_settled(self) -> bool:
        """Is there an established leader for this region? (PROTOCOL.md §5.7)

        Follower: yes if we have heard from our leader within T_DEAD.
        Leader:   yes once we have held the role for T_SETTLE with no election
                  running — long enough for bootstrap conflicts to resolve.
        Migration and voluntary departure both wait for this.
        """
        ls = self.leadership()
        now = time.monotonic()
        if ls.role == Role.FOLLOWER:
            return ls.leader_address is not None and (now - self.last_ack_at) <= config.T_DEAD
        return (now - ls.since) >= config.T_SETTLE and not self.election.election_in_progress

    # =====================================================================
    # Health, state, priority
    # =====================================================================

    def is_healthy(self) -> bool:
        """May this bot lead? FAULTED / COMMS_LONG may not (PROTOCOL.md §5.1)."""
        return prio.is_healthy(self.state)

    def effective_state(self) -> str:
        """What we tell the region we are doing: the robot's FSM state, except
        that a migration in flight reports MIGRATING (PROTOCOL.md §6)."""
        return "MIGRATING" if self.migrator.in_flight else self.state

    def adopt_region(
        self, *, region_id: int, leader_id: int | None, leader_address: str | None,
        peers: list[Peer], leaders: list[Leader],
    ) -> None:
        """Switch membership to `region_id` with a fresh roster (end of a
        migration). If a leader address is given we follow it; otherwise the
        caller will `become_leader()` (solo region)."""
        with self._lock:
            self.region_id = region_id
            self.peer_table.replace(peers, leaders)
            self.migrating_out = Ledger()
            self.pending_incoming = Ledger()
            self.jobs = JobLedger()  # the old region's replica is not ours to carry
        if leader_address is not None and leader_id is not None:
            self.become_follower(leader_id, leader_address)
            self.last_ack_at = time.monotonic()  # the join ack counts as contact

    # =====================================================================
    # Wire payloads — the one place the message shapes are built
    # =====================================================================

    def rpc_metadata(self) -> list[tuple[str, str]]:
        """Identity attached to every outgoing call (checked by `bus.policy`)."""
        return rpc_metadata(self.bot_id, self.region_id, self.role)

    def heartbeat_payload(self) -> fleet_pb2.HeartbeatRequest:
        if self.cargo_state == CS_DELIVERED:
            self._delivered_reported = True  # cleared once this heartbeat is acked
        return fleet_pb2.HeartbeatRequest(
            bot_id=self.bot_id, region_id=self.region_id, latest_node_id=self.latest_node_id,
            state=self.effective_state(), battery=self.battery, priority=self.priority,
            address=self.address, mission=self.mission, fault=self.fault,
            timestamp=int(time.time() * 1000), node_trail=list(self.node_trail),
            job_id=self.current_job.job_id if self.current_job else "",
            cargo_state=self.cargo_state,
        )

    def leader_hb_payload(self, ack: bool = False):
        """LeaderHBRequest (or the mirrored LeaderHBAck): a summary of this
        region plus where each of its bots has been lately, so every leader
        holds a fleet-wide picture of movement (PROTOCOL.md §3.2)."""
        peers = self.peer_table.all_peers()
        batteries = [p.battery for p in peers] + [self.battery]
        locations = [fleet_pb2.BotLocation(bot_id=p.bot_id, node_trail=p.node_trail) for p in peers]
        locations.append(fleet_pb2.BotLocation(bot_id=self.bot_id, node_trail=list(self.node_trail)))
        cls = fleet_pb2.LeaderHBAck if ack else fleet_pb2.LeaderHBRequest
        return cls(
            region_id=self.region_id, leader_bot_id=self.bot_id, address=self.address,
            bot_count=len(peers) + 1, avg_battery=sum(batteries) / len(batteries),
            active_jobs=0, priority=self.priority, timestamp=int(time.time() * 1000),
            locations=locations,
        )

    def roster_ack(self) -> fleet_pb2.HeartbeatAck:
        """The full region picture a leader returns in every HeartbeatAck.

        Bots in `migrating_out` are shown as MIGRATING_OUT here regardless of
        what their own heartbeat said — that flag must not live on the Peer
        record, where the next heartbeat would overwrite it.
        """
        def yp(job_id: str, cargo_state: str) -> int:
            return prio.yield_priority(has_job=bool(job_id), carrying=cargo_state in (CS_EN_ROUTE, CS_DROPOFF))

        records = [
            fleet_pb2.PeerRecord(
                bot_id=p.bot_id, address=p.address, priority=p.priority,
                state="MIGRATING_OUT" if p.bot_id in self.migrating_out else p.state,
                battery=p.battery, latest_node_id=p.latest_node_id, node_trail=p.node_trail,
                mission=p.mission, fault=p.fault, job_id=p.job_id, cargo_state=p.cargo_state,
                yield_priority=yp(p.job_id, p.cargo_state),
            )
            for p in self.peer_table.all_peers()
        ]
        own_job = self.current_job.job_id if self.current_job else ""
        records.append(fleet_pb2.PeerRecord(   # include ourselves so followers can elect us
            bot_id=self.bot_id, address=self.address, priority=self.priority,
            state=self.effective_state(), battery=self.battery, latest_node_id=self.latest_node_id,
            node_trail=list(self.node_trail), mission=self.mission, fault=self.fault,
            job_id=own_job, cargo_state=self.cargo_state, yield_priority=yp(own_job, self.cargo_state),
        ))
        leaders = [
            fleet_pb2.LeaderRecord(region_id=ld.region_id, bot_id=ld.bot_id, address=ld.address)
            for ld in self.peer_table.all_leaders()
        ]
        return fleet_pb2.HeartbeatAck(
            region_peers=records, other_leaders=leaders, leader_bot_id=self.bot_id,
            jobs=[j.to_proto() for j in self.jobs.all()],  # replicate the ledger (§14.2)
        )

    # =====================================================================
    # Lifecycle
    # =====================================================================

    def start(self) -> None:
        log.info("bot-%d starting: region=%d, address=%s, peers=%s",
                 self.bot_id, self.region_id, self.address, self.peer_leaders)
        self._start_grpc_server()
        self._tick_priority()

        if self.is_healthy():
            # PROTOCOL.md §4.1: everyone starts as leader; conflicts collapse
            # the field to one within a leader-heartbeat round.
            self.become_leader()
        else:
            # Unhealthy bots never claim leadership. Sit as a leaderless
            # follower; the heartbeat sender probes PEER_LEADERS to find one.
            log.warning("bot-%d: unhealthy at boot (%s); waiting for a leader", self.bot_id, self.state)
            with self._lock:
                self._leadership = Leadership(Role.FOLLOWER, None, None, time.monotonic())
            self._hb_sender.start()

        # Refuse to boot on a configuration that cannot work, rather than
        # running and misbehaving in a way that looks like something else.
        config.validate(getattr(self.graph, "node_spacing", None))
        signal.signal(signal.SIGTERM, self._handle_signal)
        signal.signal(signal.SIGINT, self._handle_signal)
        self._run_loop()

    def _run_loop(self) -> None:
        """Tick every T_HB. Order matters: read the robot first so every
        later step sees this tick's truth."""
        while not self._shutdown.is_set():
            self._tick_robot_state()
            self._tick_navigation()
            self._tick_reservations()
            self._tick_self_job()
            self._tick_priority()
            self._tick_health()
            self._tick_tenure()
            self.migrator.tick()
            self._tick_leader_duties()
            self._shutdown.wait(config.T_HB)

    def _tick_robot_state(self) -> None:
        """Mirror the newest RobotState into our fields; note the physical region."""
        update = self._robot_source.poll()
        if update is None:
            return
        self.latest_node_id = update.latest_node_id
        # Only *movement* extends the trail: the same node reported twice is
        # the robot standing still, not a path.
        if update.latest_node_id and (not self.node_trail or self.node_trail[0] != update.latest_node_id):
            self.node_trail.appendleft(update.latest_node_id)
        self.battery = update.battery
        self.state = update.state
        self.mission = update.mission
        self.fault = update.fault
        if self.current_job is not None:
            self._advance_job(update)
        if update.region_id != 0:
            # Only *record* the desired region here. Acting on it is the
            # Migrator's job, on its own schedule, with retries (reconcile
            # loop) — never a one-shot reaction to this update.
            self.desired_region_id = update.region_id

    # =====================================================================
    # Pathfinding — answering the robot (PROTOCOL.md §16)
    # =====================================================================

    def _route(self, query: Query) -> Decision:
        """Answer one question from the robot standing at a node.

        Runs on the robot link's thread, not the run loop's, so a slow plan
        delays this robot rather than the whole bot. Everything it reads is
        either atomic or behind its own lock.

        Every path out of here returns a Decision. A robot that hears nothing
        stops for the rest of its shift, so "we do not know" has to be said out
        loud as a short WAIT rather than by falling silent.
        """
        if self.planner is None or self.graph is None or self.topology is None:
            return Decision(
                query_id=query.query_id, kind=DecisionKind.WAIT,
                hold_ms=int(config.T_BLOCKED_HOLD * 1000),
                because="no warehouse map loaded",
            )
        # Snapshot the goal rather than reading it twice. The run loop can clear
        # it between the check and the use -- abandoning a job before pickup does
        # exactly that -- and the planner would then be handed no goal at all.
        goal = self.nav_goal
        if goal is None:
            return Decision(
                query_id=query.query_id, kind=DecisionKind.WAIT,
                hold_ms=int(config.T_ARRIVED_HOLD * 1000),
                because="no job",
            )

        now = now_ms()
        observations = self._observations()
        traffic = traffic_module.build(
            self.graph, observations, now=now, config=self.planning,
            kinematics=self.planner.kinematics,
            exclude_bot_id=self.bot_id,
        )
        plan = self.planner.plan(Request(
            now=now,
            self_state=SelfState(
                node_id=query.node_id,
                heading=heading_from_radians(query.heading_rad),
                moving=self.state == "MOVING",
                energy=self._energy_state(),
            ),
            goal=goal,
            # The same peers the traffic view is built from, so the search
            # respects predicted occupancy and not just declared claims.
            peers=traffic.peers,
            # Tier 3. These have to travel on the request, because the search
            # is the only thing that prices them -- a blocked node is one the
            # planner declines to route through, and it can only decline what
            # it was told about.
            obstructions=tuple(
                Obstruction(node_id=node, level=level)
                for node, level in sorted(self.obstructions.items())
            ),
        ))
        decision = decide_module.decide(
            self.graph, self.topology, query,
            plan=plan, traffic=traffic, observations=observations,
            my_bot_id=self.bot_id, my_rank=self._yield_rank(),
            config=self.planning, last_target=self._last_target,
        )
        # Remember where we sent it, so the next answer can tell a route that
        # changed from one that did not. A WAIT names no node and must not
        # clear that memory: the robot is still headed where it was.
        if decision.target_node_id:
            self._last_target = decision.target_node_id
        return decision

    def report_robot_state(self, state: RobotState) -> None:
        """Take one report from the robot. **The only way position gets in.**

        Handed to the source rather than written straight onto the bot, so the
        run loop stays the single writer of `latest_node_id` and `node_trail`
        and a report arriving mid-tick cannot half-apply.
        """
        self._robot_source.push(state)

    def report_obstacle(self, seen_at: int, level: float) -> None:
        """A robot has something in its way, or has stopped having it.

        `seen_at` is where the *robot* was, which is what the shared schema
        carries and all a robot can honestly say -- it saw something ahead, and
        it knows which node it was standing on. What has to be blocked is the
        node it was heading for, and that is ours to know: we told it to go
        there one answer ago.

        Blocking `seen_at` instead would block the ground under the robot's own
        wheels, and the next question from it would come back
        `no route (start_blocked)` -- a robot walled in by its own report.

        A clear (`seen_at` of 0) drops everything this robot was routing
        around. It only knows about blockages it reported, so there is nothing
        else to drop, and leaving them would mean a lane lost for the shift
        because nothing ever says a second time that it is fine.
        """
        if level <= 0:
            if self.obstructions:
                log.info("bot-%d: lanes clear again (%d were blocked)",
                         self.bot_id, len(self.obstructions))
            self.obstructions.clear()
            self._last_target = None
            return

        blocked = self._last_target or seen_at
        log.info("bot-%d: obstacle reported at node %d, blocking node %d",
                 self.bot_id, seen_at, blocked)
        self.set_obstruction(blocked, level)

    def set_obstruction(self, node_id: int, level: float) -> None:
        """Mark a node blocked, or clear it with a level of zero."""
        if level <= 0:
            self.obstructions.pop(node_id, None)
        else:
            self.obstructions[node_id] = level
        # The route in hand was costed without this, so it is no longer the
        # cheapest thing we know -- drop it rather than drive it.
        self._last_target = None

    def _observations(self) -> tuple:
        """What we currently know about the other robots, in one shape.

        Declared claims come from our own ledger — announcements that actually
        reached us — and positions from the roster the leader distributes. A
        peer with no claims still contributes its trail, which is what tier 2
        predicts from.
        """
        claims: dict[int, list] = {}
        for claim in self.ledger.peer_claims():
            claims.setdefault(claim.bot_id, []).append(
                Reservation(node_id=claim.node_id, t_in=claim.start_ms, t_out=claim.end_ms)
            )
        return tuple(
            traffic_module.Observation(
                bot_id=peer.bot_id,
                node_id=peer.latest_node_id or None,
                trail=tuple(peer.node_trail),
                reservations=tuple(claims.get(peer.bot_id, ())),
                rank=prio.yield_priority(
                    has_job=bool(peer.job_id),
                    carrying=peer.cargo_state in (CS_EN_ROUTE, CS_DROPOFF),
                ),
            )
            for peer in self.peer_table.all_peers()
            if peer.bot_id != self.bot_id
        )

    def _yield_rank(self) -> int:
        """Our own right of way — the same number the roster advertises."""
        return prio.yield_priority(
            has_job=self.current_job is not None,
            carrying=self.cargo_state in (CS_EN_ROUTE, CS_DROPOFF),
        )

    def _energy_state(self):
        """Map battery onto the planner's four states.

        The robot reports a percentage; how much charge is "short" is a fleet
        policy, so it lives with the other job/battery thresholds in config
        rather than being invented inside the cost model.
        """
        from planning.cost import EnergyState

        if self.mission == "CHARGE":
            return EnergyState.RECOVERING
        if self.battery <= config.BATTERY_CRITICAL:
            return EnergyState.CRITICAL
        if self.battery <= config.JOB_MIN_BATTERY:
            return EnergyState.SHORT
        return EnergyState.OK

    def _tick_navigation(self) -> None:
        """Notice a robot that has been told to move and has not (PROTOCOL.md §16).

        Escalates rather than reacting all at once: the first suspicion is that
        our route is stale, the second that something is in the way that will
        not move for us, and only the third that a person needs to look. Each
        rung is a whole `T_STALL`, so a robot pausing for traffic never trips it.
        """
        if self.nav_goal is None or not self.latest_node_id:
            self._nav_node = self.latest_node_id
            self._nav_since = time.monotonic()
            self._nav_strikes = 0
            return

        if self.latest_node_id != self._nav_node:
            self._nav_node = self.latest_node_id
            self._nav_since = time.monotonic()
            self._nav_strikes = 0
            return

        if time.monotonic() - self._nav_since < config.T_STALL:
            return

        self._nav_since = time.monotonic()
        self._nav_strikes += 1
        if self._nav_strikes == 1:
            log.warning("bot-%d: stalled at node %d; dropping the route",
                        self.bot_id, self.latest_node_id)
            self._last_target = None
        elif self._nav_strikes == 2:
            log.warning("bot-%d: still stalled at node %d; will stand aside",
                        self.bot_id, self.latest_node_id)
            self.ledger.withdraw()   # stop holding a lane we are not using
        else:
            log.error("bot-%d: stuck at node %d for %.0fs",
                      self.bot_id, self.latest_node_id, self._nav_strikes * config.T_STALL)
            self.control_plane({
                "type": "NEEDS_ATTENTION",
                "bot_id": self.bot_id,
                "node_id": self.latest_node_id,
                "job_id": self.current_job.job_id if self.current_job else "",
                "reason": "stalled",
            })
            self._nav_strikes = 0

    def _tick_reservations(self) -> None:
        """Claim where we are and tell the neighbours (PROTOCOL.md §15).

        Straight after reading the robot, so the claim is about where it is now
        rather than where it was a tick ago.
        """
        self._reservations.tick()

    def _tick_priority(self) -> None:
        """Recompute election priority from health + battery (election/priority.py)."""
        self.priority = prio.compute(
            healthy=self.is_healthy(), battery_pct=self.battery, bot_id=self.bot_id,
            sitting_leader=(self.role == Role.LEADER),
        )
        self.election.priority = self.priority

    def _tick_health(self) -> None:
        """An unhealthy leader hands off to the best peer (PROTOCOL.md §7).
        If it is alone there is no one to hand off to; it stays until a
        healthy bot arrives and out-ranks it via the conflict rule."""
        if self.is_healthy() or self.role != Role.LEADER:
            return
        successor = self.peer_table.best_successor(exclude=self.bot_id)
        if successor is None:
            return
        log.info("bot-%d: %s as leader, abdicating to bot-%d", self.bot_id, self.state, successor.bot_id)
        if self.election.abdicate(successor):
            self.become_follower(successor.bot_id, successor.address)

    def _tick_tenure(self) -> None:
        """Rotate leadership after T_LEADER_TENURE (PROTOCOL.md §5.6 "Tenure").

        Election priority alone lets one charged high-id bot lead forever.
        After a full tenure a leader with no job of its own hands off to the
        best *free* healthy follower. It may well win leadership back at the
        next real election — that is fine; the point is that leading is a
        shift, not a life sentence.
        """
        if config.T_LEADER_TENURE <= 0 or self.current_job is not None:
            return
        ls = self.leadership()
        if ls.role != Role.LEADER or (time.monotonic() - ls.since) < config.T_LEADER_TENURE:
            return
        successor = self.peer_table.best_successor(exclude=self.bot_id)
        if successor is None or successor.job_id:
            return  # nobody free to take over; keep leading, check again next tick
        log.info("bot-%d: tenure of %.0fs served, rotating leadership to bot-%d",
                 self.bot_id, config.T_LEADER_TENURE, successor.bot_id)
        if self.election.abdicate(successor):
            self.become_follower(successor.bot_id, successor.address)

    def _tick_leader_duties(self) -> None:
        """Leader housekeeping: evict silent followers, expire stale migration
        records (this is the source-/destination-side T_MIGRATION_TIMEOUT),
        retry queued jobs."""
        if self.role != Role.LEADER:
            return
        for peer in self.peer_table.evict_dead():
            log.info("bot-%d: evicted dead peer bot-%d", self.bot_id, peer.bot_id)
            # A bot that went quiet *because it migrated* is not dead — its
            # new region's leader now sees its heartbeats (job included).
            if peer.bot_id not in self.migrating_out:
                self.dispatcher.on_peer_evicted(peer)
            self.migrating_out.pop(peer.bot_id)
        for bot_id in self.migrating_out.expire(config.T_MIGRATION_TIMEOUT):
            log.info("bot-%d: migration of bot-%d timed out; treating as member again", self.bot_id, bot_id)
        for bot_id in self.pending_incoming.expire(config.T_MIGRATION_TIMEOUT):
            log.info("bot-%d: expected bot-%d never joined; dropping handoff", self.bot_id, bot_id)
        self.dispatcher.tick()

    # =====================================================================
    # Jobs — the bot-side half (PROTOCOL.md §14)
    # =====================================================================

    def is_free_for_job(self) -> bool:
        return (
            self.is_healthy() and self.state == "IDLE" and self.mission in ("IDLE", "PARK", "")
            and self.current_job is None and not self.fault
            and self.battery >= config.JOB_MIN_BATTERY and not self.migrator.in_flight
        )

    def handle_assign_job(self, job: Job) -> bool:
        """A leader offers us a job. Accept iff free; then command the robot
        to the pickup node. From here on our heartbeats carry the job."""
        if not self.is_free_for_job():
            return False
        self.current_job = job
        self.cargo_state = CS_PICKUP
        self._delivered_reported = False
        # Set a goal, do not command a destination. The robot cannot drive to a
        # node seventy hops away -- it asks at every node it reaches, and
        # `_route` answers with the next turn (PROTOCOL.md §16).
        self.nav_goal = Goal.node(job.pickup_node)
        self._last_target = None
        self._robot_sink.send(RobotCommand(
            target_node_id=job.pickup_node,
            mission={"type": "CARGO", "cargo": {"cargo_id": job.job_id, "state": CS_PICKUP}},
        ))
        log.info("bot-%d: accepted job %s → pickup at node %d", self.bot_id, job.job_id, job.pickup_node)
        return True

    def _advance_job(self, update: RobotState) -> None:
        """Turn what the robot reports into job progress and the next command.

            robot: CARGO/PICKUP  → CARGO/EN_ROUTE (it has the cargo)
                   → we command: go to dropoff, CARGO/EN_ROUTE
            robot: CARGO/DROPOFF → mission leaves CARGO (it has set it down)
                   → we report DELIVERED in our heartbeat until acked
        """
        job = self.current_job

        # 1. Absorb the robot's progress FIRST. A robot can report "I have the
        #    cargo" and "I am broken" in the same update; deciding pre/post
        #    pickup from stale state would drop a job whose cargo is on board.
        if update.mission == "CARGO" and update.cargo_state:
            if update.cargo_state == CS_EN_ROUTE and self.cargo_state == CS_PICKUP:
                self.nav_goal = Goal.node(job.dropoff_node)
                self._last_target = None
                self._robot_sink.send(RobotCommand(
                    target_node_id=job.dropoff_node,
                    mission={"type": "CARGO", "cargo": {"cargo_id": job.job_id, "state": CS_EN_ROUTE}},
                ))
                log.info("bot-%d: job %s picked up → dropoff at node %d", self.bot_id, job.job_id, job.dropoff_node)
            self.cargo_state = update.cargo_state
        elif update.mission != "CARGO" and self.cargo_state == CS_DROPOFF:
            self.cargo_state = CS_DELIVERED
            log.info("bot-%d: job %s delivered", self.bot_id, job.job_id)

        # 2. Then decide whether we can still finish. Before pickup the owner
        #    re-queues the job to someone else, so we must drop it — otherwise
        #    we would resume it after recovering and two bots would go for
        #    the same cargo. After pickup the cargo is physically on us: keep
        #    the job so our heartbeats keep saying where it is.
        in_trouble = (
            not self.is_healthy() or update.mission == "CHARGE"
            or (update.fault and update.fault.startswith(("MOTOR", "CAMERA", "LIDAR", "LOCATION", "MISC", "LOW_BATTERY")))
        )
        if in_trouble and self.cargo_state in ("", CS_PICKUP):
            log.warning("bot-%d: abandoning job %s before pickup (%s / %s)",
                        self.bot_id, job.job_id, update.state, update.fault or update.mission)
            self.current_job = None
            self.cargo_state = ""
            self.nav_goal = None

    def self_peer(self) -> Peer:
        """Ourselves as a roster record — what a leader would see if we were
        heartbeating it. Used for leader self-observation and as the
        last-resort job candidate."""
        return Peer(
            bot_id=self.bot_id, address=self.address, priority=self.priority,
            state=self.effective_state(), battery=self.battery, latest_node_id=self.latest_node_id,
            node_trail=list(self.node_trail), mission=self.mission, fault=self.fault,
            job_id=self.current_job.job_id if self.current_job else "", cargo_state=self.cargo_state,
        )

    def _tick_self_job(self) -> None:
        """A leader has nobody watching its heartbeats, so when it is working
        a job it must play both roles: feed its own state through the same
        `observe()` that followers' heartbeats go through, and clear the job
        once DELIVERED has been applied (there is no ack to wait for)."""
        if self.role != Role.LEADER:
            self._last_self_peer = None
            return
        cur = self.self_peer()
        if cur.job_id or (self._last_self_peer and self._last_self_peer.job_id):
            self.dispatcher.observe(self._last_self_peer, cur)
        self._last_self_peer = cur
        if self.current_job is not None and self.cargo_state == CS_DELIVERED:
            # Owner path removed it from the ledger; non-owner path fanned out
            # the event. Either way it has been seen — we are free again.
            log.info("bot-%d: own job %s delivered; free again", self.bot_id, self.current_job.job_id)
            self.current_job = None
            self.cargo_state = ""
            self._delivered_reported = False

    def on_ack_jobs(self, jobs: list[Job]) -> None:
        """Called by the heartbeat sender with the leader's ledger replica.
        Once the heartbeat that said DELIVERED has been acked, the leader has
        seen it and we can forget the job."""
        self.jobs.replace(jobs)
        if self.current_job is not None and self._delivered_reported:
            log.info("bot-%d: job %s acknowledged; free again", self.bot_id, self.current_job.job_id)
            self.current_job = None
            self.cargo_state = ""
            self._delivered_reported = False

    def _default_control_plane(self, event: dict) -> None:
        self.alerts.append(event)
        log.error("bot-%d: CONTROL PLANE ← %s", self.bot_id, event)

    # ---- gRPC server ----------------------------------------------------------

    def _start_grpc_server(self) -> None:
        self._grpc_server = grpc.server(
            futures.ThreadPoolExecutor(max_workers=config.GRPC_WORKERS),
            interceptors=[VirtualNetworkInterceptor(self)],
        )
        fleet_pb2_grpc.add_ElectionServiceServicer_to_server(ElectionServicer(self.election, self.peer_table), self._grpc_server)
        fleet_pb2_grpc.add_RegionServiceServicer_to_server(RegionServicer(self), self._grpc_server)
        fleet_pb2_grpc.add_LeaderExchangeServiceServicer_to_server(LeaderExchangeServicer(self), self._grpc_server)
        fleet_pb2_grpc.add_MigrationJoinServiceServicer_to_server(MigrationJoinServicer(self), self._grpc_server)
        fleet_pb2_grpc.add_JobServiceServicer_to_server(JobServicer(self), self._grpc_server)
        fleet_pb2_grpc.add_BotServiceServicer_to_server(BotServicer(self), self._grpc_server)
        fleet_pb2_grpc.add_ReservationServiceServicer_to_server(ReservationServicer(self.ledger), self._grpc_server)
        fleet_pb2_grpc.add_AdminServiceServicer_to_server(AdminServicer(self), self._grpc_server)
        # Where cargo orders enter the fleet. Served by every bot because the
        # control plane knows no leaders -- see bus/control_plane.py.
        controlplane_pb2_grpc.add_ControlPlaneServiceServicer_to_server(
            ControlPlaneServicer(self), self._grpc_server)
        # The robot link. On this bot's own server, so a robot talks to its own
        # coordinator and nothing else -- see docs/boundary.md.
        self._robot_service = RobotNetworkServicer(
            router=self._route,
            report=self.report_robot_state,
            obstruct=self.report_obstacle,
            state_factory=RobotState,
            bot_id=self.bot_id,
        )
        robot_pb2_grpc.add_RobotNetworkServicer_to_server(self._robot_service, self._grpc_server)

        bind = f"{config.GRPC_HOST}:{config.GRPC_PORT}"
        # add_insecure_port returns 0 on bind failure rather than raising.
        if self._grpc_server.add_insecure_port(bind) == 0:
            raise RuntimeError(f"bot-{self.bot_id}: could not bind gRPC server to {bind}")
        self._grpc_server.start()
        log.info("gRPC server listening on %s", bind)

    # =====================================================================
    # Event handlers (called from sender / gRPC threads)
    # =====================================================================

    def on_leader_dead(self) -> None:
        """Follower missed T_LEADER_DEAD of acks → elect (PROTOCOL.md §4.8)."""
        if self.election.departing or not self.is_healthy():
            return
        log.info("bot-%d: leader is dead, starting election", self.bot_id)
        self.election.start_election(self.peer_table.all_peers())

    def on_same_region_leader_conflict(self, other_bot_id: int, other_addr: str, other_priority: int | None = None) -> None:
        """Another bot also claims to lead our region (bootstrap or healed
        partition, PROTOCOL.md §5.4). Exactly one of us yields:
          * an unhealthy bot always yields (it should never have led);
          * otherwise the lower (priority, bot_id) yields.
        The other side runs the same rule on its copy, so no message is needed.
        """
        if self.role != Role.LEADER:
            return
        if other_priority is None:
            other = self.peer_table.get(other_bot_id)
            other_priority = other.priority if other else other_bot_id

        if not self.is_healthy():
            log.info("bot-%d: unhealthy (%s), yielding region %d to bot-%d", self.bot_id, self.state, self.region_id, other_bot_id)
            self.become_follower(other_bot_id, other_addr)
        elif _outranks(other_priority, other_bot_id, self.priority, self.bot_id):
            log.info("bot-%d: yielding to higher-priority leader bot-%d for region %d", self.bot_id, other_bot_id, self.region_id)
            self.become_follower(other_bot_id, other_addr)
        else:
            log.info("bot-%d: we outrank conflicting leader bot-%d, they should yield", self.bot_id, other_bot_id)

    # =====================================================================
    # Shutdown
    # =====================================================================

    def graceful_shutdown(self) -> None:
        """SIGTERM path (PROTOCOL.md §4.4). Waits briefly for a settled leader
        so we never leave a region mid-election, then hands off or departs."""
        log.info("bot-%d: shutting down gracefully", self.bot_id)
        self.election.departing = True
        # A decision issued while we are leaving would send the robot somewhere
        # nobody is left to coordinate, so the robot stream goes down with the
        # rest of the server below rather than outliving us.

        deadline = time.monotonic() + config.T_SETTLE
        while not self.leader_settled() and time.monotonic() < deadline:
            time.sleep(0.2)

        ls = self.leadership()
        if ls.role == Role.LEADER:
            successor = self.peer_table.best_successor(exclude=self.bot_id)
            if successor:
                self.election.abdicate(successor)
        elif ls.leader_address:
            try:
                pool.stub(ls.leader_address, fleet_pb2_grpc.RegionServiceStub).Departure(
                    fleet_pb2.DepartureRequest(bot_id=self.bot_id, timestamp=int(time.time() * 1000)),
                    timeout=config.T_DEPARTURE, metadata=self.rpc_metadata(),
                )
            except grpc.RpcError:
                log.debug("bot-%d: couldn't send departure to leader", self.bot_id)

        self._hb_sender.stop()
        self._leader_exchange.stop()
        if self._grpc_server:
            self._grpc_server.stop(grace=2)
        pool.close_all()
        self._shutdown.set()

    def _handle_signal(self, signum, frame) -> None:
        self.graceful_shutdown()


def main() -> None:
    Bot().start()


if __name__ == "__main__":
    main()
