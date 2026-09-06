"""The harness for the container scenarios: fleets, fixtures, helpers.

Everything the in-process suite cannot honestly test lives in the
`test_docker_*.py` files beside this one, one per scenario letter of
`docs/scenarios.md`. This module is what they share: a bridge
network with Docker DNS, processes that can be killed, paused (hung, socket
open, never answers) or cut off the network, and a fleet that must converge
afterwards. Bots are driven and inspected through `AdminService`
(`bus/admin.py`), which `up.py` enables for local fleets.

Skipped automatically when no Docker daemon is reachable. The image is built
once per session. Each test gets its own network and container-name prefix,
so tests never see each other and never touch a developer's `amr-net` fleet.

Timing: T_HB is 1 s in the image, so "dead leader" takes ~3 s to notice and
an election a few seconds more; a paused container fails by *timeout* rather
than refusal, so allow ~10 s. Waits below are generous on purpose.
"""
from __future__ import annotations

import functools
import os
import sys
import time
import uuid
from pathlib import Path

from filelock import FileLock

import grpc
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import down
import up
from bus.policy import rpc_metadata
from tests._wait import wait_until as _wait_until
from proto import (
    controlplane_pb2, controlplane_pb2_grpc, fleet_pb2, fleet_pb2_grpc,
    robot_pb2, robot_pb2_grpc)

# Region ids from the consolidated 7-region map (was 14 before the upstream
# `consolidate warehouse regions` change): parking is now 2, and the old single
# grid_field is split across 5/6/7 -- 6 is the middle band.
PARK, GRID = 2, 6


ADMIN_MD = rpc_metadata(999, 0, "admin")
#: A robot is not a bot: it has no identity in the fleet and no region to be
#: in. The policy admits its own robot unconditionally, so this is only enough
#: for the interceptor to have something to read.


ROBOT_MD = rpc_metadata(0, 0, "robot")


def _docker_client():
    try:
        import docker
        c = docker.from_env()
        c.ping()
        return c
    except Exception:
        return None


@pytest.fixture(scope="session")
def client():
    c = _docker_client()
    if c is None:
        pytest.skip("no Docker daemon reachable")
    return c


@pytest.fixture(scope="session", autouse=True)
def _sweep_leaked_fleets(client):
    """Remove fleets a previous run left behind, before this one starts.

    Teardown is not guaranteed. Interrupt a run -- Ctrl-C, a killed xdist
    worker, a laptop lid -- and the containers outlive the process that made
    them. They are not idle: each is a bot heartbeating on a timer, and a dozen
    of them turn a one-minute suite into three while looking like nothing at
    all. Measured exactly that way, twice.

    Safe because it filters on our own label, which only `up.py` sets. A dev
    fleet from `up.py` carries it too and will be swept -- that is the intended
    trade: a test run starts from a known floor.
    """
    down.remove_fleet(client, quiet=True)
    yield


@pytest.fixture(scope="session")
def image(client, tmp_path_factory, worker_id):
    """Build once, however many workers are running.

    Session-scoped means *per worker* under xdist, so without this every worker
    would build the same tag at the same time. The layer cache makes that cheap
    rather than free, and two builders writing one tag is a race nobody needs.

    The lock lives in xdist's shared root temp dir, which is the one directory
    every worker agrees on.
    """
    if os.environ.get("AMR_DOCKER_NO_BUILD") == "1":
        return up.IMAGE

    if worker_id == "master":                      # not running under xdist
        up.build_image(client, quiet=True)
        return up.IMAGE

    done = tmp_path_factory.getbasetemp().parent / "amr-image-built"
    with FileLock(str(done) + ".lock"):
        if not done.exists():
            up.build_image(client, quiet=True)
            done.write_text(up.IMAGE)
    return up.IMAGE


# Containers answer slowly, so poll them at a gentler cadence than the
# in-process fleet. Same function; one bound argument.
wait_until = functools.partial(_wait_until, step=0.5)


class DockerFleet:
    """A private network + containers for one test, with admin access."""

    def __init__(self, client, image: str) -> None:
        self.client = client
        self.image = image
        tag = uuid.uuid4().hex[:6]
        self.network_name = f"amrtest-{tag}"
        self.prefix = f"amrtest{tag}"
        self.network = client.networks.create(self.network_name, driver="bridge", labels={up.LABEL_FLEET: "1"})
        self.containers: list = []

    # ---- lifecycle -------------------------------------------------------

    def launch(self, n: int, region: int, **extra_env) -> list:
        cs = up.launch(self.client, n, region, self.network_name, prefix=self.prefix,
                       extra_env={k: str(v) for k, v in extra_env.items()}, quiet=True)
        self.containers.extend(cs)
        return cs

    def close(self) -> None:
        for c in self.containers:
            try:
                c.remove(force=True)
            except Exception:
                pass
        try:
            self.network.remove()
        except Exception:
            pass

    # ---- access ----------------------------------------------------------

    def ip(self, c) -> str:
        c.reload()
        return c.attrs["NetworkSettings"]["Networks"][self.network_name]["IPAddress"]

    def endpoint(self, c) -> str:
        """Where the *test process* dials. Not the container IP: Docker Desktop
        on macOS does not route to those, so `up.launch` publishes a host port
        and this prefers it."""
        return up.host_endpoint(c)

    def admin(self, c):
        return fleet_pb2_grpc.AdminServiceStub(grpc.insecure_channel(self.endpoint(c)))

    def state(self, c) -> fleet_pb2.BotState:
        return self.admin(c).GetState(fleet_pb2.Empty(), timeout=2, metadata=ADMIN_MD)

    def place(self, c, latest_node_id: int, region_id: int = PARK, *,
              battery: float = 90.0, mission: str = "IDLE", fault: str = "",
              job_id: str = "", cargo_state: str = "", state: str = "IDLE",
              obstacle_node: int = 0) -> None:
        """Put a robot at a node by *telling the truth about where it is*.

        This used to be `inject`, and it was the last back door: an admin RPC
        pushing a whole `RobotState` straight into the bot, bypassing the QR
        read, the companion and the wire. It was also the reason the container
        suite could not see that production never fed position at all -- it
        supplied by hand the one thing nothing else supplied.

        Now it is a `RobotToNetwork` on the real stream, which is what a
        companion sends and all a companion can send. Two consequences worth
        knowing:

        `state` is accepted and ignored. The wire has no field for a robot's
        FSM state, deliberately -- a robot reporting at a node is standing at
        it, and anything it does between nodes is not something this link
        describes. It stays in the signature so scenarios read the way they did.

        A report is applied on the run loop's next tick, not on return. That was
        true of injection too; it is why every scenario here waits for what it
        asked for rather than assuming it.
        """
        message = robot_pb2.RobotToNetwork(
            latest_node_id=latest_node_id,
            region_id=region_id,
            telemetry=robot_pb2.Telemetry(
                battery=robot_pb2.Battery(percentage=battery)),
            mission=_mission(mission, job_id, cargo_state),
        )
        if fault:
            message.fault.error.type = _ERROR_TYPE.get(
                fault, robot_pb2.ERROR_TYPE_MISC_ERROR)
        if obstacle_node:
            message.fault.warning.obstacle.current_node_id = obstacle_node
        self.report(c, message)

    def submit_job(self, c, job_id: str, pickup: int, dropoff: int) -> fleet_pb2.JobAck:
        stub = fleet_pb2_grpc.JobServiceStub(grpc.insecure_channel(self.endpoint(c)))
        return stub.SubmitJob(fleet_pb2.Job(job_id=job_id, pickup_node=pickup, dropoff_node=dropoff),
                              timeout=10, metadata=rpc_metadata(999, 0, "orders"))

    def report(self, c, message, timeout: int = 10):
        """Play the companion: one turn of the real robot stream.

        `RobotNetwork.Session` is the canonical link -- the same service a
        companion dials -- so this exercises the whole path: a report in,
        position applied, and where it asked a question, plan and answer out.

        Reaching it from the host is the point of moving off a unix socket: a
        socket had to be spoken to from inside the container, and that meant
        `docker exec` and a fresh python interpreter for every single question.

        Returns the answer, or None when the message was telemetry and asked
        nothing.
        """
        stub = robot_pb2_grpc.RobotNetworkStub(grpc.insecure_channel(self.endpoint(c)))
        for reply in stub.Session(iter([message]), timeout=timeout, metadata=ROBOT_MD):
            return reply
        return None

    def converse(self, c, messages, timeout: int = 20) -> list:
        """A whole shift's worth of talking on **one** stream.

        The point of a long-lived stream is that it is long-lived: a connection
        per question would be pure overhead on hardware that has none to spare.
        Anything asserting about the link itself -- that it survives a message
        it cannot use, that it serves fifty questions, what one costs -- has to
        speak on a single stream or it is measuring connection setup.
        """
        stub = robot_pb2_grpc.RobotNetworkStub(grpc.insecure_channel(self.endpoint(c)))
        return list(stub.Session(iter(messages), timeout=timeout, metadata=ROBOT_MD))

    def ask(self, c, message, timeout: int = 10):
        """`report`, for a message that is a question and must be answered."""
        reply = self.report(c, message, timeout=timeout)
        assert reply is not None, "a robot that asked was answered with silence"
        return reply

    def reset(self, cs, nodes=None) -> None:
        """Put shared bots back to a known state between scenarios.

        A job in hand has to be **finished**, not dropped. Faulting a bot before
        pickup makes the fleet requeue the job by its own rule (§14.4), which is
        correct behaviour and exactly wrong here: the work comes straight back
        and lands on the next scenario's bot. After a few of those nothing is
        free and every routing scenario fails with "nobody took the job", which
        is what happened the first time these shared fleets carried real jobs.
        Delivering is the only exit that empties the queue.

        `_advance_job` reads the cargo state the robot reports and not the node
        it stands on, so this delivers wherever the bot happens to be. That is a
        liberty a real robot could not take, and it is the last one left here --
        the canonical driver removes it along with the rest of the injection.

        The fault trick stays for what it is actually good at: clearing a goal
        on a bot that never had a job, and clearing a fault a scenario injected.
        """
        nodes = nodes or _map_nodes(PARK, len(cs) * 4)[:: 4]
        for c, node in zip(cs, nodes, strict=False):
            held = self.state(c).current_job_id
            if not held:
                continue
            self.place(c, latest_node_id=node, region_id=PARK, battery=90.0,
                        state="IDLE", mission="CARGO", job_id=held,
                        cargo_state="DROPOFF")
            self.place(c, latest_node_id=node, region_id=PARK, battery=90.0,
                        state="IDLE", mission="IDLE")
            assert wait_until(lambda c=c: not self.state(c).current_job_id, 20,
                              what=f"job {held} to be crossed off"), \
                f"bot still holds {held}; the next scenario will find nobody free"
        for c, node in zip(cs, nodes, strict=False):
            self.place(c, latest_node_id=node, region_id=PARK, battery=90.0,
                        state="IDLE", mission="IDLE", fault="MOTOR_ERROR")
        for c, node in zip(cs, nodes, strict=False):
            self.place(c, latest_node_id=node, region_id=PARK, battery=90.0,
                        state="IDLE", mission="IDLE", fault="")

    def obstruct(self, c, at_node: int, level: float = 1.0) -> None:
        """Report something in the lane, the way a robot reports it.

        `at_node` is where the *robot* is, not what gets blocked. A robot can
        only honestly say "I am here and something is in front of me"; which
        node that makes unusable is the network layer's to work out, because it
        is the one that told the robot where to go. So a scenario asks first,
        and what gets blocked is the lane it was answered with.

        `level` survives as a parameter but only zero-or-not reaches the wire: a
        robot sees something or it does not, and has no way to say how much. The
        planner's graded levels are for a sensor that can.
        """
        message = robot_pb2.RobotToNetwork(
            latest_node_id=at_node,
            region_id=PARK,
            telemetry=robot_pb2.Telemetry(
                battery=robot_pb2.Battery(percentage=90.0)),
            mission=robot_pb2.Mission(idle=robot_pb2.Idle()),
        )
        # Present-and-zero is how a robot says the lane is clear again, which is
        # a different thing from a report that mentions no obstacle at all --
        # and almost every report mentions none.
        message.fault.warning.obstacle.current_node_id = at_node if level > 0 else 0
        self.report(c, message)

    def drive(self, c, nodes, region: int = PARK, pace: float | None = None,
              **fields) -> None:
        """Walk a robot along a route, one QR scan at a time.

        `pace` is how long each position is held, and it is not decoration. The
        bot reads its robot once per tick and keeps only the newest report, so a
        position pushed and replaced inside one tick is a position the fleet
        never saw -- and `node_trail`, which is *distinct successive* nodes, is
        the thing that notices: four markers 50 ms apart used to arrive as one.

        One tick is the floor, and the default. Anything asserting about claims
        needs `_hop_seconds()` instead: a claim covers a whole traversal, so two
        positions closer together than that overlap by construction.
        """
        wait = pace if pace is not None else _tick_seconds()
        for node in nodes:
            self.place(c, latest_node_id=node, region_id=region, **fields)
            time.sleep(wait)

    def decisions(self, c, nodes, region: int = PARK, **fields) -> list:
        """Drive a route and collect what the bot answered at each node.

        The shape most planning scenarios want: a journey, and the sequence of
        turns it produced.
        """
        out = []
        for i, node in enumerate(nodes):
            self.place(c, latest_node_id=node, region_id=region, **fields)
            out.append(self.ask(c, _query(node, _neighbours(node), query_id=i + 1, region=region)))
        return out

    def ledger_windows(self, cs) -> dict:
        """Every window every bot believes is held, keyed by node."""
        held = {}
        for c in cs:
            for r in self.state(c).reservations:
                held.setdefault(r.node_id, set()).add((r.bot_id, r.start_ms, r.end_ms))
        return held

    def assert_no_overlap(self, cs) -> None:
        """The standing invariant: no node was ever held by two bots at once.

        Checked wherever bots share a floor rather than in one test that might
        be the only thing exercising it.
        """
        for node, windows in self.ledger_windows(cs).items():
            entries = sorted(windows)
            for i, (bot_a, start_a, end_a) in enumerate(entries):
                for bot_b, start_b, end_b in entries[i + 1:]:
                    if bot_a != bot_b and start_a < end_b and end_a > start_b:
                        raise AssertionError(
                            f"node {node} held by bot {bot_a} [{start_a},{end_a}] "
                            f"and bot {bot_b} [{start_b},{end_b}] at once"
                        )

    # ---- assertions helpers ---------------------------------------------

    def leaders(self, cs) -> list:
        out = []
        for c in cs:
            try:
                if self.state(c).role == "leader":
                    out.append(c)
            except grpc.RpcError:
                pass
        return out

    def converged(self, cs, region: int | None = None) -> bool:
        """Exactly one leader; every other bot follows it; all in `region`."""
        states = []
        for c in cs:
            try:
                states.append(self.state(c))
            except grpc.RpcError:
                return False
        if region is not None and any(s.region_id != region for s in states):
            return False
        leaders = [s for s in states if s.role == "leader"]
        if len(leaders) != 1:
            return False
        lid = leaders[0].bot_id
        return all(s.leader_bot_id == lid for s in states if s.role == "follower")

    def roster_ids(self, c) -> set[int]:
        return {p.bot_id for p in self.state(c).roster}


@pytest.fixture
def fleet(client, image):
    """A fleet of this scenario's own. Use for anything that submits a job,
    kills a container, or otherwise leaves state behind."""
    f = DockerFleet(client, image)
    yield f
    f.close()


# ---- Compressed timings ------------------------------------------------------

#: One place the fast clock is defined, so no scenario can quietly invent its
#: own and then pass for a reason nobody intended. Everything that derives from
#: T_HB in config.py scales with it, so shortening one shortens the protocol.
#:
#: What this does NOT prove is behaviour at production timings -- a scenario
#: that only passes here belongs in the slow tier, not in a tuned version of
#: this one. `docs/scenarios.md` says which is which.


FAST_TIMINGS = {
    "T_HB": 0.3,
    "T_STALL": 1.0,
    "T_ANNOUNCE": 0.3,
    "RESERVATION_TTL": 0.9,
    "T_MIGRATION_TIMEOUT": 3.0,
    "T_JOB_RETRY": 1.0,
}

#: The shared fleets get one change: a stall clock long enough to ignore.
#:
#: A bot on a shared fleet is put where a scenario wants it by injection and
#: then stands there while the scenario asks its questions. The fleet is right
#: to call that a stall -- a real robot given a goal and not moving for three
#: seconds is stuck -- and rung 3 raises NEEDS_ATTENTION, which before pickup
#: means the job is dropped and requeued (§14.4). So a routing scenario that
#: pauses to wait for anything would be told `no job` by the bot it had just
#: given one to, which is a true fact about a fleet nobody is driving and has
#: nothing to do with what the scenario is asking.
#:
#: The scenarios that *are* about stalling (D1-D3) launch their own fleet on
#: FAST_TIMINGS, so the short clock is still exercised where it means something.
#: The canonical driver removes the need for this: a robot that really drives
#: never looks stalled.


SHARED_TIMINGS = dict(FAST_TIMINGS, T_STALL=30.0)


def _shared(client, image, count: int, region: int):
    """A fleet several scenarios share, built once and reset between them.

    Fifty-odd scenarios each launching their own containers would spend three
    minutes starting Docker before a single assertion ran. Scenarios that only
    read state or ask the robot link questions do not need isolation, so they
    share one fleet per shape.
    """
    f = DockerFleet(client, image)
    cs = f.launch(count, region, **SHARED_TIMINGS)
    assert wait_until(lambda: f.converged(cs, region), 30, what="the shared fleet to converge")
    yield f, cs
    f.close()


@pytest.fixture(scope="module")
def one_bot(client, image):
    yield from _shared(client, image, 1, PARK)


@pytest.fixture(scope="module")
def two_bots(client, image):
    yield from _shared(client, image, 2, PARK)


@pytest.fixture(scope="module")
def three_bots(client, image):
    yield from _shared(client, image, 3, PARK)


def _tick_seconds() -> float:
    """One run-loop tick, plus a little.

    The bot reads its robot once per `T_HB` and keeps only the newest report, so
    this is the shortest a position can be held and still be seen at all.
    """
    return float(SHARED_TIMINGS["T_HB"]) * 1.5


def _hop_seconds() -> float:
    """How long a robot really needs to move one node, plus a little.

    Derived from the same numbers the claim window is, so the two cannot drift:
    if a hop ever gets cheaper or a claim longer, this follows. A harness that
    moves robots faster than this is teleporting them, and every invariant about
    two robots and one node stops meaning anything.
    """
    from planning.kinematics import DEFAULT_KINEMATICS
    from warehouse.map import WarehouseMap
    import config

    spacing = WarehouseMap.load(config.WAREHOUSE_MAP).node_spacing
    return (DEFAULT_KINEMATICS.cruise_ms(spacing) + config.PLAN_SAFETY * 1000) / 1000 + 0.2


def _map_nodes(region: int, n: int) -> list[int]:
    from warehouse.map import WarehouseMap
    import config
    return WarehouseMap.load(config.WAREHOUSE_MAP).nodes_in(region)[:n]


# =============================================================================
# Reservations (PROTOCOL.md §15)
#
# The claim these make is one the in-process suite cannot: that bots agree over a
# real network, in separate processes, with no shared memory and no leader in the
# path. Ledgers are read through AdminService, which is the only way to see what a
# container believes.
def _nearby_pair(region: int, max_hops: int = 3) -> tuple[int, int]:
    """Two nodes in `region` close enough to contest each other's claims."""
    from warehouse.map import WarehouseMap
    import config
    m = WarehouseMap.load(config.WAREHOUSE_MAP)
    nodes = m.nodes_in(region)
    first = nodes[0]
    for other in nodes[1:]:
        if 0 < m.distance(first, other) <= max_hops:
            return first, other
    raise AssertionError(f"no two nodes within {max_hops} hops in region {region}")


def _claims_of(fleet, container, bot_id: int) -> list:
    return [r for r in fleet.state(container).reservations if r.bot_id == bot_id]


def _park(fleet, container, node: int) -> None:
    fleet.place(container, latest_node_id=node, region_id=PARK,
                 battery=90.0, state="IDLE", mission="IDLE")


# =============================================================================
# Pathfinding (PROTOCOL.md §16)
#
# These play the companion: they ask on the bot's own unix socket, inside the
# container, which is the same link a real robot uses. What they exercise is the
# whole path -- query in, plan against live traffic, decision out.
_ERROR_TYPE = {
    "MOTOR_ERROR": robot_pb2.ERROR_TYPE_MOTOR_ERROR,
    "CAMERA_ERROR": robot_pb2.ERROR_TYPE_CAMERA_ERROR,
    "LIDAR_ERROR": robot_pb2.ERROR_TYPE_LIDAR_ERROR,
    "LOCATION_UNKNOWN": robot_pb2.ERROR_TYPE_LOCATION_UNKNOWN,
    "MISC_ERROR": robot_pb2.ERROR_TYPE_MISC_ERROR,
}


_CARGO_STATE = {
    "PICKUP": robot_pb2.CARGO_STATE_PICKUP,
    "DROPOFF": robot_pb2.CARGO_STATE_DROPOFF,
    "EN_ROUTE": robot_pb2.CARGO_STATE_EN_ROUTE,
}


def _mission(mission: str, job_id: str, cargo_state: str):
    """A mission on the wire: which `oneof` case is set *is* the value."""
    if mission == "CARGO" or cargo_state:
        return robot_pb2.Mission(cargo=robot_pb2.Cargo(
            cargo_id=job_id,
            state=_CARGO_STATE.get(cargo_state, robot_pb2.CARGO_STATE_UNSPECIFIED)))
    return {
        "PARK": robot_pb2.Mission(park=robot_pb2.Park()),
        "CHARGE": robot_pb2.Mission(charge=robot_pb2.Charge()),
        "HOLD": robot_pb2.Mission(hold=robot_pb2.Hold()),
    }.get(mission, robot_pb2.Mission(idle=robot_pb2.Idle()))


_KIND_NAME = {
    # UNSPECIFIED reads as PROCEED on purpose: the wire lets a decision leave
    # the field unset, and a robot that sees nothing there takes the lane.
    robot_pb2.KIND_UNSPECIFIED: "PROCEED",
    robot_pb2.KIND_PROCEED: "PROCEED",
    robot_pb2.KIND_REROUTE: "REROUTE",
    robot_pb2.KIND_WAIT: "WAIT",
    robot_pb2.KIND_YIELD: "YIELD",
}


def _kind(reply) -> str:
    """The answer's kind, by name, so scenarios read as behaviour."""
    return _KIND_NAME[reply.kind]


def _query(node_id: int, available, query_id: int = 1, region: int = PARK,
           battery: float = 90.0):
    """A robot stopped at a node, asking which way to go.

    `available` is what makes it a question rather than telemetry. The nodes in
    it are the robot's own answer to what is physically reachable from here --
    it holds the map and knows the heading it arrived on, so that is its call
    and not ours.
    """
    return robot_pb2.RobotToNetwork(
        query_id=query_id,
        latest_node_id=node_id,
        region_id=region,
        heading_rad=0.0,
        available=list(available),
        mission=robot_pb2.Mission(idle=robot_pb2.Idle()),
        telemetry=robot_pb2.Telemetry(battery=robot_pb2.Battery(percentage=battery)),
    )


def _neighbours(node_id: int):
    """The nodes reachable from one node, as the robot would offer them.

    Node ids, not turn names: left and right never cross this wire. The network
    names a node and the robot derives the bearing from the map it also holds,
    which is exact -- lanes are straight.
    """
    from warehouse.map import WarehouseMap
    import config

    return tuple(WarehouseMap.load(config.WAREHOUSE_MAP).neighbours(node_id))


def _holder_of(fleet, cs, job_id: str):
    """Which bot took the job. The dispatcher decides, not the scenario."""
    holder = []
    def assigned() -> bool:
        for c in cs:
            if fleet.state(c).current_job_id == job_id:
                holder.append(c)
                return True
        return False
    wait_until(assigned, 20, what=f"job {job_id} to reach a bot")
    return holder[0] if holder else None


# -----------------------------------------------------------------------------
def _corridor(min_hops: int = 6):
    """A run of degree-2 nodes on the real map: somewhere with no way around."""
    from planning.graph import Graph
    from planning.topology import Topology
    from warehouse.map import WarehouseMap
    import config

    graph = Graph(WarehouseMap.load(config.WAREHOUSE_MAP))
    topo = Topology(graph)
    for corridor in sorted(topo.corridors, key=lambda c: -c.hops):
        if corridor.hops >= min_hops:
            return [graph.id_of(n) for n in corridor.nodes]
    raise AssertionError(f"no corridor of {min_hops}+ hops on this map")


def _move(fleet, c, node: int, region: int = PARK):
    """Move a bot without clobbering everything else about it.

    A report replaces the whole state, and proto3 defaults make every
    omitted field zero -- so injecting a node on its own reports a flat battery
    and an empty mission, and a bot that was mid-job promptly abandons it as if
    it had failed. Which is correct behaviour, and a trap: the scenario asks for
    a turn and is told `no job` by a bot it had just given one to.

    Echo back what the bot already believes, with the node changed.
    """
    st = fleet.state(c)
    # Only a cargo state that is still *in flight* is worth echoing. A leftover
    # DELIVERED from the previous scenario's job, replayed onto this one, tells
    # the fleet the new job is already finished -- it gets crossed off, the goal
    # goes with it, and the scenario is answered `no job` by a bot it had just
    # given work to. Found exactly that way: F6 passed alone and failed after F3.
    in_flight = st.cargo_state if st.cargo_state in ("PICKUP", "EN_ROUTE", "DROPOFF") else ""
    fleet.place(c, latest_node_id=node, region_id=region, battery=90.0,
                 state="IDLE", mission="IDLE",
                 job_id=st.current_job_id, cargo_state=in_flight)


def _routing_fleet(fleet, job_id: str, at: int, count: int = 2, region: int = PARK):
    """A fleet of this scenario's own, one bot holding a job and standing on `at`.

    Two things every routing scenario needs, and one it must not have.

    **A goal.** A bot with no job answers `WAIT "no job"` before the planner is
    ever consulted, so asking a jobless bot for a turn proves the socket replies
    and nothing more -- and an assertion written as
    `if kind in ("PROCEED", "REROUTE")` then never runs at all. That is not
    hypothetical: it is how F2 passed for weeks while the bot drove happily into
    nodes it had been told were impassable. Hence `_planned()` below, which
    every one of these scenarios calls on the reply before trusting it.

    **A clear floor.** The bots start on distinct nodes, none of them `at`. Two
    robots on one node is a state driving cannot produce; both claim it and the
    no-overlap invariant fails, correctly.

    **Not a shared fleet.** These scenarios were tried on one and it does not
    hold: a job left outstanding is requeued onto the next scenario's bot, a
    delivered cargo state replayed onto a fresh job closes it on arrival, and a
    departed bot's claim outlives the move now that a claim covers a traversal.
    Each of those is the fleet behaving correctly, and together they made a
    suite that passed in isolation and failed in company -- which is worse than
    one that takes an extra few seconds. The canonical driver removes the need
    for the isolation by removing the teleporting that causes it.
    """
    cs = fleet.launch(count, region, **SHARED_TIMINGS)
    assert wait_until(lambda: fleet.converged(cs, region), 30, what="converge")

    far = _map_nodes(region, 40)
    spare = [n for n in far if n not in (at, far[-2], far[-1])]
    for i, c in enumerate(cs):
        fleet.place(c, latest_node_id=spare[-(i + 1)], region_id=region,
                     battery=90.0, state="IDLE", mission="IDLE")
    leader = fleet.leaders(cs)[0]
    assert wait_until(lambda: all(p.mission == "IDLE" for p in fleet.state(leader).roster),
                      20, what="the roster to settle before dispatch")

    assert fleet.submit_job(leader, job_id, far[-2], far[-1]).accepted
    holder = _holder_of(fleet, cs, job_id)
    assert holder is not None, "nobody took the job, so no bot has a goal to plan toward"
    _move(fleet, holder, at, region)

    # A bot that has moved still holds its old node until the claim runs out,
    # and that window is now a full traversal rather than two announce periods.
    mine = fleet.state(holder).bot_id
    assert wait_until(
        lambda: not any(r.node_id == at and r.bot_id != mine
                        for r in fleet.state(holder).reservations),
        20, what=f"peer claims on node {at} to lapse")
    return cs, holder


def _planned(reply, where: str = ""):
    """Assert the planner actually ran, and hand the reply back.

    Guards the vacuous-pass hole described in `_routing_fleet`: a scenario
    that means to test routing must fail loudly if the bot never routed,
    rather than skipping its own assertion.
    """
    assert reply is not None, f"no answer at all {where}".strip()
    assert reply.because != "no job", \
        f"the bot had no goal, so the planner was never asked {where}".strip()
    return reply


# -----------------------------------------------------------------------------
def _near_and_far(region: int, pickup: int):
    """Two nodes in `region`, one genuinely closer to `pickup` than the other.

    By driving distance, never by position in the node list: ids follow where a
    node sits on the floor and say nothing about how far apart two of them are
    to drive between. Picking by index held on the old map and silently stopped
    holding on this one.
    """
    from warehouse.map import WarehouseMap
    import config

    warehouse = WarehouseMap.load(config.WAREHOUSE_MAP)
    ranked = sorted((n for n in warehouse.nodes_in(region) if n != pickup),
                    key=lambda n: warehouse.distance(n, pickup))
    return ranked[0], ranked[-1]


def _free_fleet(fleet, count: int, region: int = PARK, node=None):
    """A converged fleet whose bots are all idle and visible as idle.

    The roster is only as fresh as the last heartbeat, and the leader assigns
    from the roster -- so waiting for it is part of the setup, not flakiness.
    """
    cs = fleet.launch(count, region, **FAST_TIMINGS)
    assert wait_until(lambda: fleet.converged(cs, region), 30, what="converge")
    node = node or _map_nodes(region, 1)[0]
    for c in cs:
        fleet.place(c, latest_node_id=node, region_id=region, battery=90.0,
                     state="IDLE", mission="IDLE")
    leader = fleet.leaders(cs)[0]
    assert wait_until(lambda: all(p.mission == "IDLE" for p in fleet.state(leader).roster),
                      20, what="the roster to see everyone idle")
    return cs, leader


def _order_stub(fleet, c):
    return controlplane_pb2_grpc.ControlPlaneServiceStub(
        grpc.insecure_channel(fleet.endpoint(c)))


def _place_order(fleet, c, order_id: str, pickup: int, dropoff: int):
    return _order_stub(fleet, c).DispatchOrder(
        controlplane_pb2.Order(order_id=order_id, pickup_node=pickup,
                               dropoff_node=dropoff),
        timeout=15, metadata=rpc_metadata(999, 0, "orders"))


# -----------------------------------------------------------------------------
def _logs(c) -> str:
    return c.logs().decode("utf-8", "replace")


# -----------------------------------------------------------------------------
YIELD_TIMINGS = dict(FAST_TIMINGS, T_YIELD_THRESHOLD=0.05)
