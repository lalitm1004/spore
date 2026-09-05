"""Chaos and scenario tests on REAL containers and a REAL Docker network.

Everything the in-process suite cannot honestly test lives here: a bridge
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

import json
import os
import sys
import time
import uuid
from pathlib import Path

import grpc
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import up
from bus.policy import rpc_metadata
from proto import fleet_pb2, fleet_pb2_grpc

pytestmark = pytest.mark.docker

# Region ids from the consolidated 7-region map (was 14 before the upstream
# `consolidate warehouse regions` change): parking is now 2, and the old single
# grid_field is split across 5/6/7 -- 6 is the middle band.
PARK, GRID = 2, 6
ADMIN_MD = rpc_metadata(999, 0, "admin")


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


@pytest.fixture(scope="session")
def image(client):
    if os.environ.get("AMR_DOCKER_NO_BUILD") != "1":
        up.build_image(client, quiet=True)
    return up.IMAGE


def wait_until(pred, timeout: float, step: float = 0.5, what: str = "") -> bool:
    """Poll until true or the deadline passes.

    `what` names the thing being waited for and is reported on failure. Every
    call site already passed one; it just never reached the failure message,
    which made a timeout read as a bare `assert False` with nothing to go on.
    """
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            if pred():
                return True
        except grpc.RpcError:
            pass  # container not up / paused / partitioned — keep polling
        time.sleep(step)
    if what:
        print("timed out after {:.0f}s waiting for {}".format(timeout, what), flush=True)
    return False


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

    def inject(self, c, **fields) -> None:
        self.admin(c).InjectRobotState(fleet_pb2.RobotStateMsg(**fields), timeout=2, metadata=ADMIN_MD)

    def submit_job(self, c, job_id: str, pickup: int, dropoff: int) -> fleet_pb2.JobAck:
        stub = fleet_pb2_grpc.JobServiceStub(grpc.insecure_channel(self.endpoint(c)))
        return stub.SubmitJob(fleet_pb2.Job(job_id=job_id, pickup_node=pickup, dropoff_node=dropoff),
                              timeout=10, metadata=rpc_metadata(999, 0, "orders"))

    def ask(self, c, query: dict, timeout: int = 10) -> dict | None:
        """Play the companion: ask this bot for a turn on its own socket.

        Run inside the container, because a unix socket is not reachable from
        the host. This is the real link -- the same socket `planning/server.py`
        binds and a real companion dials -- so what it exercises is the whole
        path: query in, plan, decision out.
        """
        script = (
            "import socket,json,sys\n"
            "s=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM);s.settimeout(%d)\n"
            "s.connect('/tmp/spore-robot.sock')\n"
            "s.sendall((json.dumps(%r)+chr(10)).encode())\n"
            "b=b''\n"
            "while chr(10).encode() not in b: b+=s.recv(4096)\n"
            "sys.stdout.write(b.split(chr(10).encode())[0].decode())\n"
        ) % (timeout, query)
        result = c.exec_run(["python3", "-c", script])
        text = result.output.decode().strip()
        try:
            return json.loads(text)
        except json.JSONDecodeError:
            return None

    def reset(self, cs, nodes=None) -> None:
        """Put shared bots back to a known state between scenarios.

        Dropping a job uses the fleet's own rule rather than a back door: a bot
        that reports a hard fault before pickup abandons its job (§14.4). So a
        fault clears the job, and the second inject clears the fault.
        """
        nodes = nodes or _map_nodes(PARK, len(cs) * 4)[:: 4]
        for c, node in zip(cs, nodes, strict=False):
            self.inject(c, latest_node_id=node, region_id=PARK, battery=90.0,
                        state="IDLE", mission="IDLE", fault="MOTOR_ERROR")
        for c, node in zip(cs, nodes, strict=False):
            self.inject(c, latest_node_id=node, region_id=PARK, battery=90.0,
                        state="IDLE", mission="IDLE", fault="")

    def obstruct(self, c, node_id: int, level: float = 1.0) -> None:
        """Block a node for this bot, or clear it with level 0."""
        self.admin(c).InjectObstruction(
            fleet_pb2.ObstructionMsg(node_id=node_id, level=level),
            timeout=2, metadata=ADMIN_MD)

    def drive(self, c, nodes, region: int = PARK, **fields) -> None:
        """Walk a robot along a route, one QR scan at a time."""
        for node in nodes:
            self.inject(c, latest_node_id=node, region_id=region, **fields)
            time.sleep(0.05)

    def decisions(self, c, nodes, region: int = PARK, **fields) -> list:
        """Drive a route and collect what the bot answered at each node.

        The shape most planning scenarios want: a journey, and the sequence of
        turns it produced.
        """
        out = []
        for i, node in enumerate(nodes):
            self.inject(c, latest_node_id=node, region_id=region, **fields)
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


def _shared(client, image, count: int, region: int):
    """A fleet several scenarios share, built once and reset between them.

    Fifty-odd scenarios each launching their own containers would spend three
    minutes starting Docker before a single assertion ran. Scenarios that only
    read state or ask the robot link questions do not need isolation, so they
    share one fleet per shape.
    """
    f = DockerFleet(client, image)
    cs = f.launch(count, region, **FAST_TIMINGS)
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


def _map_nodes(region: int, n: int) -> list[int]:
    from warehouse.map import WarehouseMap
    import config
    return WarehouseMap.load(config.WAREHOUSE_MAP).nodes_in(region)[:n]


# =============================================================================

def test_bootstrap_converges_on_one_leader(fleet):
    cs = fleet.launch(3, PARK)
    assert wait_until(lambda: fleet.converged(cs, PARK), 30, what="converge")
    leader = fleet.leaders(cs)[0]
    assert wait_until(lambda: fleet.roster_ids(leader) == {0, 1, 2}, 15)
    assert fleet.state(leader).leader_settled or wait_until(lambda: fleet.state(leader).leader_settled, 10)


def test_kill_leader_then_restart_it(fleet):
    cs = fleet.launch(3, PARK)
    assert wait_until(lambda: fleet.converged(cs, PARK), 30)
    leader = fleet.leaders(cs)[0]
    survivors = [c for c in cs if c is not leader]

    leader.kill()
    assert wait_until(lambda: fleet.converged(survivors, PARK), 40, what="re-elect"), "survivors should elect"
    new_leader = fleet.leaders(survivors)[0]
    assert new_leader is not leader

    leader.start()   # comes back as a self-declared leader → conflict → resolves
    assert wait_until(lambda: fleet.converged(cs, PARK), 40, what="reconverge after restart")
    final = fleet.leaders(cs)[0]
    assert wait_until(lambda: fleet.roster_ids(final) == {0, 1, 2}, 20), "restarted bot rejoins the roster"


def test_paused_leader_is_treated_as_dead_then_split_brain_heals(fleet):
    """pause ≠ kill: the socket stays open and calls time out instead of
    being refused. Exercises the deadline path, not the connection path."""
    cs = fleet.launch(3, PARK)
    assert wait_until(lambda: fleet.converged(cs, PARK), 30)
    leader = fleet.leaders(cs)[0]
    others = [c for c in cs if c is not leader]

    leader.pause()
    assert wait_until(lambda: fleet.converged(others, PARK), 60, what="elect around a hung leader")

    leader.unpause()   # it still thinks it leads → two leaders → priority rule
    assert wait_until(lambda: fleet.converged(cs, PARK), 40, what="split-brain heal")


def test_partitioned_follower_self_elects_then_yields_on_reconnect(fleet):
    cs = fleet.launch(3, PARK)
    assert wait_until(lambda: fleet.converged(cs, PARK), 30)
    leader = fleet.leaders(cs)[0]
    victim = next(c for c in cs if c is not leader)

    fleet.network.disconnect(victim)
    # From the leader's side: the victim drops out of the roster.
    remaining = {int(s.split("BOT_ID=")[1]) for c in cs if c is not victim
                 for s in c.attrs["Config"]["Env"] if s.startswith("BOT_ID=")}
    assert wait_until(lambda: fleet.roster_ids(leader) == remaining, 30, what="victim evicted")

    fleet.network.connect(victim)
    # Alone it elected itself; back on the network the conflict rule collapses it.
    assert wait_until(lambda: fleet.converged(cs, PARK), 40, what="rejoin")
    assert wait_until(lambda: fleet.roster_ids(fleet.leaders(cs)[0]) == {0, 1, 2}, 20)


def test_two_regions_and_a_migration_over_real_containers(fleet):
    park = fleet.launch(2, PARK)
    grid = fleet.launch(1, GRID)              # sees the park bots via PEER_LEADERS
    assert wait_until(lambda: fleet.converged(park, PARK) and fleet.converged(grid, GRID), 40)
    park_leader = fleet.leaders(park)[0]
    grid_leader = grid[0]
    assert wait_until(lambda: {ld.region_id for ld in fleet.state(park_leader).other_leaders} == {GRID}, 20,
                      what="leaders meet")

    mover = next(c for c in park if c is not park_leader)
    mover_id = fleet.state(mover).bot_id
    grid_node = _map_nodes(GRID, 1)[0]
    fleet.inject(mover, latest_node_id=grid_node, region_id=GRID, battery=90.0, state="IDLE", mission="IDLE")

    assert wait_until(lambda: fleet.state(mover).region_id == GRID and fleet.state(mover).role == "follower", 40,
                      what="migrate")
    assert wait_until(lambda: mover_id in fleet.roster_ids(grid_leader), 15)
    assert wait_until(lambda: mover_id not in fleet.roster_ids(park_leader), 15)


def test_job_dispatched_and_completed_over_real_containers(fleet):
    cs = fleet.launch(3, PARK)
    assert wait_until(lambda: fleet.converged(cs, PARK), 30)
    leader = fleet.leaders(cs)[0]
    assert wait_until(lambda: fleet.roster_ids(leader) == {0, 1, 2}, 15)
    pickup, dropoff = _map_nodes(PARK, 6)[0], _map_nodes(PARK, 6)[5]

    ack = fleet.submit_job(leader, "job-docker-1", pickup, dropoff)
    assert ack.accepted and ack.HasField("assignee"), ack
    assignee = next(c for c in cs if fleet.state(c).bot_id == ack.assignee)
    assert fleet.state(assignee).current_job_id == "job-docker-1"

    # Drive the assignee's "robot" through the job.
    fleet.inject(assignee, latest_node_id=pickup, region_id=PARK, battery=90.0, state="MOVING",
                 mission="CARGO", job_id="job-docker-1", cargo_state="EN_ROUTE")
    assert wait_until(lambda: any(j.job_id == "job-docker-1" and j.status == "PICKED_UP" for j in fleet.state(leader).jobs), 15)
    fleet.inject(assignee, latest_node_id=dropoff, region_id=PARK, battery=88.0, state="MOVING",
                 mission="CARGO", job_id="job-docker-1", cargo_state="DROPOFF")
    fleet.inject(assignee, latest_node_id=dropoff, region_id=PARK, battery=88.0, state="IDLE", mission="IDLE")
    assert wait_until(lambda: not any(j.job_id == "job-docker-1" for j in fleet.state(leader).jobs), 15,
                      what="crossed off")
    assert wait_until(lambda: fleet.state(assignee).current_job_id == "", 10, what="assignee free again")


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
    fleet.inject(container, latest_node_id=node, region_id=PARK,
                 battery=90.0, state="IDLE", mission="IDLE")


@pytest.mark.docker
def test_E5_a_claim_crosses_a_real_network(fleet):
    """One bot claims the node it is standing on; the other hears about it."""
    a_node, b_node = _nearby_pair(PARK)
    cs = fleet.launch(2, PARK)
    assert wait_until(lambda: fleet.converged(cs, PARK), 30, what="converge")

    _park(fleet, cs[0], a_node)
    _park(fleet, cs[1], b_node)

    ids = [fleet.state(c).bot_id for c in cs]
    assert wait_until(lambda: _claims_of(fleet, cs[1], ids[0]), 20,
                      what="bot-0's claim to reach bot-1"), "no claim arrived"
    assert _claims_of(fleet, cs[1], ids[0])[0].node_id == a_node


@pytest.mark.docker
def test_D8_claims_keep_flowing_when_the_leader_dies(fleet):
    """The §7 promise: collision avoidance does not depend on a leader.

    Kill the leader, move a survivor, and check the other survivor still learns
    where it went. Announcements never went through the leader in the first
    place -- this is what proves it.
    """
    a_node, b_node = _nearby_pair(PARK)
    cs = fleet.launch(3, PARK)
    assert wait_until(lambda: fleet.converged(cs, PARK), 30, what="converge")

    leader = fleet.leaders(cs)[0]
    survivors = [c for c in cs if c.id != leader.id]
    _park(fleet, survivors[0], a_node)
    _park(fleet, survivors[1], b_node)
    def watcher_sees():
        return _claims_of(fleet, survivors[1], fleet.state(survivors[0]).bot_id)
    assert wait_until(lambda: bool(watcher_sees()), 20, what="claims before the kill")

    leader.kill()

    # Move the survivor somewhere new; the claim its neighbour holds must follow.
    _, elsewhere = _nearby_pair(PARK, max_hops=2)
    _park(fleet, survivors[0], elsewhere)
    assert wait_until(
        lambda: any(c.node_id == elsewhere for c in watcher_sees()), 30,
        what="a fresh claim to arrive with no leader alive",
    ), "announcements stopped when the leader died"


@pytest.mark.docker
def test_D7b_a_bot_that_goes_quiet_stops_blocking_its_neighbours(fleet):
    """Claims lapse, so a hung robot does not wedge a lane forever."""
    a_node, b_node = _nearby_pair(PARK)
    cs = fleet.launch(2, PARK, RESERVATION_TTL=2.0)
    assert wait_until(lambda: fleet.converged(cs, PARK), 30, what="converge")

    _park(fleet, cs[0], a_node)
    _park(fleet, cs[1], b_node)
    quiet_id = fleet.state(cs[0]).bot_id
    assert wait_until(lambda: bool(_claims_of(fleet, cs[1], quiet_id)), 20, what="claim to arrive")

    cs[0].pause()
    try:
        assert wait_until(lambda: not _claims_of(fleet, cs[1], quiet_id), 20,
                          what="the paused bot's claims to lapse")
    finally:
        cs[0].unpause()


@pytest.mark.docker
def test_two_bots_on_one_node_settle_on_a_single_holder(fleet):
    """Both want it, both apply the same ordering, and only one keeps it."""
    node, _ = _nearby_pair(PARK)
    cs = fleet.launch(2, PARK)
    assert wait_until(lambda: fleet.converged(cs, PARK), 30, what="converge")

    _park(fleet, cs[0], node)
    _park(fleet, cs[1], node)

    def one_holder() -> bool:
        holders = set()
        for c in cs:
            holders |= {r.bot_id for r in fleet.state(c).reservations if r.node_id == node}
        return len(holders) == 1

    assert wait_until(one_holder, 30, what="one of them to give way"), "both kept the node"



# =============================================================================
# Pathfinding (PROTOCOL.md §16)
#
# These play the companion: they ask on the bot's own unix socket, inside the
# container, which is the same link a real robot uses. What they exercise is the
# whole path -- query in, plan against live traffic, decision out.

def _query(node_id: int, available: dict, query_id: int = 1, region: int = PARK) -> dict:
    return {
        "query_id": query_id,
        "node": {"id": node_id, "node_type": "PT", "region_id": region},
        "robot_position": {"x": 0.0, "y": 0.0},
        "heading_rad": 0.0,
        "available": available,
    }


def _neighbours(node_id: int) -> dict:
    """Turns out of a node, named the way a robot would name them."""
    from warehouse.map import WarehouseMap
    import config

    warehouse = WarehouseMap.load(config.WAREHOUSE_MAP)
    # left / straight / right is the whole vocabulary: a robot never gets
    # offered the lane it arrived on, so a degree-4 node still has three
    # choices. `strict=False` is deliberate and this comment is why -- without
    # it, a reader would reasonably suspect a dropped turn.
    names = ("straight", "left", "right")
    return dict(zip(names, warehouse.neighbours(node_id), strict=False))


# -----------------------------------------------------------------------------
# A. Decisions — answering the robot (PROTOCOL.md §16, docs/scenarios.md A)
#
# The robot blocks at every node until it hears back, and if it never does it
# sits there for the rest of its shift. So the through-line of this block is
# that there is no input, and no internal failure, that makes a bot go quiet.

@pytest.mark.docker
def test_A1_a_bot_with_no_job_still_answers(one_bot):
    fleet, cs = one_bot
    node = _map_nodes(PARK, 1)[0]
    fleet.inject(cs[0], latest_node_id=node, region_id=PARK, battery=90.0,
                 state="IDLE", mission="IDLE")

    reply = fleet.ask(cs[0], _query(node, _neighbours(node)))
    assert reply is not None, "the bot said nothing at all"
    assert reply["kind"] == "WAIT"
    assert reply["hold_ms"] > 0, "a zero hold would have it ask in a tight loop"


@pytest.mark.docker
def test_A2_a_bot_given_a_job_is_routed_towards_it(fleet):
    """The whole point: a job becomes turns, one node at a time."""
    cs = fleet.launch(2, PARK, **FAST_TIMINGS)
    assert wait_until(lambda: fleet.converged(cs, PARK), 30, what="converge")
    nodes = _map_nodes(PARK, 8)
    start, pickup, dropoff = nodes[0], nodes[6], nodes[7]

    for c in cs:
        fleet.inject(c, latest_node_id=start, region_id=PARK, battery=90.0,
                     state="IDLE", mission="IDLE")
    assert wait_until(lambda: all(p.mission == "IDLE" for p in fleet.state(cs[0]).roster),
                      20, what="the roster to catch up with the injected state")
    assert fleet.submit_job(cs[0], "A2", pickup, dropoff).accepted

    holder = _holder_of(fleet, cs, "A2")
    assert holder is not None, "nobody took the job"
    reply = fleet.ask(holder, _query(start, _neighbours(start)))
    assert reply["kind"] in ("PROCEED", "REROUTE", "WAIT", "YIELD"), reply
    if reply["kind"] in ("PROCEED", "REROUTE"):
        assert reply["target_node_id"] in _neighbours(start).values(), \
            "it must name a lane the robot said exists"
    fleet.assert_no_overlap(cs)


@pytest.mark.docker
def test_A4_a_changed_route_is_announced_as_a_reroute(fleet):
    """PROCEED and REROUTE differ only in whether the robot's route changed --
    which is what makes a log readable when a bot doubles back."""
    cs = fleet.launch(2, PARK, **FAST_TIMINGS)
    assert wait_until(lambda: fleet.converged(cs, PARK), 30, what="converge")
    nodes = _map_nodes(PARK, 12)
    start = nodes[0]

    for c in cs:
        fleet.inject(c, latest_node_id=start, region_id=PARK, battery=90.0,
                     state="IDLE", mission="IDLE")
    assert wait_until(lambda: all(p.mission == "IDLE" for p in fleet.state(cs[0]).roster), 20)
    assert fleet.submit_job(cs[0], "A4-a", nodes[6], nodes[7]).accepted
    holder = _holder_of(fleet, cs, "A4-a")
    assert holder is not None

    first = fleet.ask(holder, _query(start, _neighbours(start)))
    # Same question again with the same route: nothing changed, so nothing is
    # announced as changed.
    second = fleet.ask(holder, _query(start, _neighbours(start), query_id=2))
    assert first is not None and second is not None
    if second["kind"] in ("PROCEED", "REROUTE"):
        assert second["kind"] == "PROCEED", \
            "an unchanged route must not be reported as a reroute"


@pytest.mark.docker
def test_A3_a_bot_standing_on_its_goal_is_told_to_wait(fleet):
    cs = fleet.launch(2, PARK, **FAST_TIMINGS)
    assert wait_until(lambda: fleet.converged(cs, PARK), 30, what="converge")
    nodes = _map_nodes(PARK, 8)
    goal = nodes[6]

    for c in cs:
        fleet.inject(c, latest_node_id=goal, region_id=PARK, battery=90.0,
                     state="IDLE", mission="IDLE")
    assert wait_until(lambda: all(p.mission == "IDLE" for p in fleet.state(cs[0]).roster),
                      20, what="the roster to catch up")
    ack = fleet.submit_job(cs[0], "A3", goal, nodes[7])
    assert ack.accepted

    holder = _holder_of(fleet, cs, "A3")
    assert holder is not None
    reply = fleet.ask(holder, _query(goal, _neighbours(goal)))
    assert reply["kind"] == "WAIT", reply
    assert reply["hold_ms"] > 0


@pytest.mark.docker
def test_A5_a_query_offering_turns_we_did_not_plan_is_still_answered(one_bot):
    """Our map and the robot's can disagree. A wrong turn is recoverable at the
    next node; silence is not recoverable at all."""
    fleet, cs = one_bot
    node = _map_nodes(PARK, 1)[0]
    reply = fleet.ask(cs[0], _query(node, {"left": 999999}))
    assert reply is not None, "a disagreement must not silence the bot"
    assert reply["kind"] in ("WAIT", "PROCEED", "REROUTE", "YIELD")


@pytest.mark.docker
def test_A6_a_malformed_query_does_not_break_the_link(one_bot):
    """One bad line must not cost the robot the rest of its shift."""
    _, cs = one_bot
    node = _map_nodes(PARK, 1)[0]
    script = (
        "import socket,json,sys\n"
        "s=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM);s.settimeout(10)\n"
        "s.connect('/tmp/spore-robot.sock')\n"
        "s.sendall(b'{not json\\n')\n"
        "s.sendall((json.dumps(%r)+chr(10)).encode())\n"
        "b=b''\n"
        "while chr(10).encode() not in b: b+=s.recv(4096)\n"
        "sys.stdout.write(b.split(chr(10).encode())[0].decode())\n"
    ) % _query(node, _neighbours(node), query_id=7)
    out = cs[0].exec_run(["python3", "-c", script]).output.decode().strip()
    assert out, "the connection died on the malformed line"
    assert json.loads(out)["query_id"] == 7


@pytest.mark.docker
def test_A7_a_node_this_map_has_never_heard_of_is_still_answered(one_bot):
    fleet, cs = one_bot
    reply = fleet.ask(cs[0], _query(999999, {"left": 999998}))
    assert reply is not None
    assert reply["kind"] in ("WAIT", "PROCEED", "REROUTE", "YIELD")


@pytest.mark.docker
def test_A8_a_bot_with_no_map_answers_and_still_leads(fleet):
    """Geography-blind is a degraded fleet, not a dead one."""
    cs = fleet.launch(1, PARK, WAREHOUSE_MAP="/nonexistent/warehouse.json", **FAST_TIMINGS)
    assert wait_until(lambda: fleet.converged(cs, PARK), 30, what="converge without a map")

    reply = fleet.ask(cs[0], _query(434, {"left": 435}))
    assert reply is not None
    assert reply["kind"] == "WAIT"
    assert "map" in reply["because"], reply


@pytest.mark.docker
def test_A9_the_query_id_comes_back_exactly(one_bot):
    """Two junctions can share a destination, so the id is the only way the
    robot can tell a fresh answer from a late one."""
    fleet, cs = one_bot
    node = _map_nodes(PARK, 1)[0]
    for query_id in (1, 7, 4242):
        assert fleet.ask(cs[0], _query(node, _neighbours(node), query_id=query_id))["query_id"] == query_id


@pytest.mark.docker
def test_A10_one_connection_serves_a_whole_shift(one_bot):
    """A socket per question would be pure overhead on hardware that has none
    to spare, so the companion connects once and keeps it."""
    _, cs = one_bot
    node = _map_nodes(PARK, 1)[0]
    script = (
        "import socket,json,sys\n"
        "s=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM);s.settimeout(15)\n"
        "s.connect('/tmp/spore-robot.sock')\n"
        "q=%r\n"
        "n=0\n"
        "for i in range(50):\n"
        "    q['query_id']=i+1\n"
        "    s.sendall((json.dumps(q)+chr(10)).encode())\n"
        "    b=b''\n"
        "    while chr(10).encode() not in b: b+=s.recv(4096)\n"
        "    n+=1\n"
        "sys.stdout.write(str(n))\n"
    ) % _query(node, _neighbours(node))
    out = cs[0].exec_run(["python3", "-c", script]).output.decode().strip()
    assert out.endswith("50"), out


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
# C. Planning (PROTOCOL.md §16, docs/scenarios.md C)
#
# A job is a destination; what the robot needs is a turn. These follow that
# translation end to end on real containers.

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


def _bot_with_a_goal(fleet, cs, job_id: str, at: int):
    """Give one bot a real job, stand it on `at`, and return its container.

    Every routing scenario has to do this first. A bot with no job answers
    `WAIT "no job"` before the planner is ever consulted, so asking a jobless
    bot for a turn proves the socket replies and nothing more -- and an
    assertion written as `if kind in ("PROCEED", "REROUTE")` then never runs at
    all. That is not hypothetical: it is how F2 passed for weeks while the bot
    drove happily into nodes it had been told were impassable.

    Hence `_planned()` below, which every one of these scenarios now calls on
    the reply before trusting it.

    The dispatcher chooses the holder, not the scenario, so the job goes out
    first and the winner is placed afterwards.
    """
    far = _map_nodes(PARK, 40)
    for c in cs:
        fleet.inject(c, latest_node_id=at, region_id=PARK, battery=90.0,
                     state="IDLE", mission="IDLE")
    assert wait_until(lambda: all(p.mission == "IDLE" for p in fleet.state(cs[0]).roster),
                      20, what="the roster to settle before dispatch")
    assert fleet.submit_job(cs[0], job_id, far[-2], far[-1]).accepted
    holder = _holder_of(fleet, cs, job_id)
    assert holder is not None, "nobody took the job, so no bot has a goal to plan toward"
    fleet.inject(holder, latest_node_id=at, region_id=PARK)
    # Park everyone else out of the way. They had to start beside the holder to
    # be candidates for the job, but leaving them there makes every scenario a
    # congestion scenario: their claims sit on the very lanes under test, and a
    # test about obstructions would be measuring peer traffic instead. The
    # scenarios that *want* a second bot in the way place it themselves.
    for c in cs:
        if c is not holder:
            fleet.inject(c, latest_node_id=far[0], region_id=PARK, battery=90.0,
                         state="IDLE", mission="IDLE")
    return holder


def _planned(reply, where: str = ""):
    """Assert the planner actually ran, and hand the reply back.

    Guards the vacuous-pass hole described in `_bot_with_a_goal`: a scenario
    that means to test routing must fail loudly if the bot never routed,
    rather than skipping its own assertion.
    """
    assert reply is not None, f"no answer at all {where}".strip()
    assert reply.get("because") != "no job", \
        f"the bot had no goal, so the planner was never asked {where}".strip()
    return reply


@pytest.mark.docker
def test_C1_a_job_becomes_a_sequence_of_turns(fleet):
    """Every node on the way is answered, and each answer names a real lane."""
    cs = fleet.launch(2, PARK, **FAST_TIMINGS)
    assert wait_until(lambda: fleet.converged(cs, PARK), 30, what="converge")
    nodes = _map_nodes(PARK, 12)

    for c in cs:
        fleet.inject(c, latest_node_id=nodes[0], region_id=PARK, battery=90.0,
                     state="IDLE", mission="IDLE")
    assert wait_until(lambda: all(p.mission == "IDLE" for p in fleet.state(cs[0]).roster), 20)
    assert fleet.submit_job(cs[0], "C1", nodes[6], nodes[7]).accepted
    holder = _holder_of(fleet, cs, "C1")
    assert holder is not None

    route = _corridor(4)[:5]
    answers = fleet.decisions(holder, route)
    assert all(a is not None for a in answers), "a node went unanswered"
    assert all(a["kind"] in ("PROCEED", "REROUTE", "WAIT", "YIELD") for a in answers)
    fleet.assert_no_overlap(cs)


@pytest.mark.docker
def test_C2_the_goal_moves_to_the_dropoff_once_the_cargo_is_aboard(fleet):
    """Nobody re-commands the robot: picking the cargo up is what changes where
    it is going."""
    cs = fleet.launch(2, PARK, **FAST_TIMINGS)
    assert wait_until(lambda: fleet.converged(cs, PARK), 30, what="converge")
    nodes = _map_nodes(PARK, 12)
    pickup, dropoff = nodes[6], nodes[10]

    for c in cs:
        fleet.inject(c, latest_node_id=nodes[0], region_id=PARK, battery=90.0,
                     state="IDLE", mission="IDLE")
    assert wait_until(lambda: all(p.mission == "IDLE" for p in fleet.state(cs[0]).roster), 20)
    assert fleet.submit_job(cs[0], "C2", pickup, dropoff).accepted
    holder = _holder_of(fleet, cs, "C2")
    assert holder is not None

    # Arrive and report the cargo aboard, exactly as a robot would.
    fleet.inject(holder, latest_node_id=pickup, region_id=PARK, battery=90.0,
                 state="IDLE", mission="CARGO", job_id="C2", cargo_state="EN_ROUTE")
    assert wait_until(lambda: fleet.state(holder).cargo_state == "EN_ROUTE", 20,
                      what="the pickup to register")

    reply = fleet.ask(holder, _query(pickup, _neighbours(pickup)))
    assert reply is not None
    assert reply["kind"] != "WAIT" or "goal" not in reply["because"], \
        "it should now be heading for the dropoff, not sitting on its goal"


@pytest.mark.docker
def test_C3_a_neighbours_claim_is_respected(two_bots):
    """Tier 1: a declared claim is a promise, and the route honours it."""
    fleet, cs = two_bots
    fleet.reset(cs)
    corridor = _corridor(6)
    ours, theirs = corridor[0], corridor[1]

    ours_c = _bot_with_a_goal(fleet, cs, "C3", ours)
    other_c = next(c for c in cs if c is not ours_c)
    fleet.inject(other_c, latest_node_id=theirs, region_id=PARK, battery=90.0,
                 state="IDLE", mission="IDLE")
    other_id = fleet.state(other_c).bot_id
    assert wait_until(lambda: any(r.bot_id == other_id and r.node_id == theirs
                                  for r in fleet.state(ours_c).reservations), 20,
                      what="the neighbour's claim to arrive")

    reply = _planned(fleet.ask(ours_c, _query(ours, _neighbours(ours))))
    if reply["kind"] in ("PROCEED", "REROUTE"):
        assert reply["target_node_id"] != theirs, "it drove into a node a peer holds"


@pytest.mark.docker
def test_C4_a_peers_trail_reaches_us_over_the_network(two_bots):
    """Tier 2's input. Prediction itself is unit-tested; what a container proves
    is that the trail a peer builds actually arrives in our roster, which is the
    only thing prediction has to work from."""
    fleet, cs = two_bots
    fleet.reset(cs)
    corridor = _corridor(6)

    fleet.drive(cs[1], corridor[:4], battery=90.0, state="MOVING", mission="IDLE")
    other_id = fleet.state(cs[1]).bot_id
    assert wait_until(
        lambda: any(len(p.node_trail) >= 2 for p in fleet.state(cs[0]).roster
                    if p.bot_id == other_id),
        20, what="a multi-node trail to reach the roster")

    trail = [list(p.node_trail) for p in fleet.state(cs[0]).roster if p.bot_id == other_id][0]
    assert trail[0] != trail[1], "consecutive duplicates should have been collapsed"


@pytest.mark.docker
def test_C6_a_flat_battery_waits_where_a_charged_one_would_go_round(two_bots):
    """The energy term exists to make exactly this trade, and it only shows up
    where going round actually costs something."""
    fleet, cs = two_bots
    fleet.reset(cs)
    corridor = _corridor(8)
    ours, ahead = corridor[3], corridor[4]

    fleet.inject(cs[1], latest_node_id=ahead, region_id=PARK, battery=90.0,
                 state="IDLE", mission="IDLE")
    fleet.inject(cs[0], latest_node_id=ours, region_id=PARK, battery=8.0,
                 state="IDLE", mission="IDLE")
    assert wait_until(lambda: len(fleet.state(cs[0]).reservations) > 1, 20,
                      what="claims to be exchanged")

    reply = fleet.ask(cs[0], _query(ours, _neighbours(ours)))
    assert reply is not None, "a flat battery is not a reason to go silent"


@pytest.mark.docker
def test_C7_a_decision_lands_well_inside_the_tick(two_bots):
    """Measured inside the container, so it is planning time rather than
    docker exec overhead."""
    fleet, cs = two_bots
    fleet.reset(cs)
    node = _map_nodes(PARK, 1)[0]
    script = (
        "import socket,json,time,sys\n"
        "s=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM);s.settimeout(15)\n"
        "s.connect('/tmp/spore-robot.sock')\n"
        "q=%r\n"
        "t=time.monotonic()\n"
        "for i in range(20):\n"
        "    q['query_id']=i+1\n"
        "    s.sendall((json.dumps(q)+chr(10)).encode())\n"
        "    b=b''\n"
        "    while chr(10).encode() not in b: b+=s.recv(4096)\n"
        "sys.stdout.write('%%.1f' %% ((time.monotonic()-t)*1000/20))\n"
    ) % _query(node, _neighbours(node))
    per_ask_ms = float(cs[0].exec_run(["python3", "-c", script]).output.decode().strip())
    print(f"\n  C7: {per_ask_ms:.1f} ms per decision")
    assert per_ask_ms < 200, f"{per_ask_ms:.1f} ms is too slow to answer at every node"


@pytest.mark.docker
def test_C8_an_unreachable_goal_is_said_out_loud(fleet):
    """"I cannot get there" has to be spoken. Silence looks identical to a dead
    network layer, and the robot treats it that way -- by never moving again."""
    cs = fleet.launch(1, PARK, **FAST_TIMINGS)
    assert wait_until(lambda: fleet.converged(cs, PARK), 30, what="converge")
    node = _map_nodes(PARK, 1)[0]
    fleet.inject(cs[0], latest_node_id=node, region_id=PARK, battery=90.0,
                 state="IDLE", mission="IDLE")

    reply = fleet.ask(cs[0], _query(node, _neighbours(node)))
    assert reply is not None
    assert reply["kind"] == "WAIT"
    assert reply["because"], "a wait with no reason is a wait nobody can debug"


# -----------------------------------------------------------------------------
# B. Job distribution (PROTOCOL.md §14, docs/scenarios.md B)
#
# These take their own fleets: a job left half-finished in a leader's ledger is
# exactly the kind of state that makes the next scenario lie.

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
        fleet.inject(c, latest_node_id=node, region_id=region, battery=90.0,
                     state="IDLE", mission="IDLE")
    leader = fleet.leaders(cs)[0]
    assert wait_until(lambda: all(p.mission == "IDLE" for p in fleet.state(leader).roster),
                      20, what="the roster to see everyone idle")
    return cs, leader


@pytest.mark.docker
def test_B1_the_nearest_free_follower_is_assigned(fleet):
    cs, leader = _free_fleet(fleet, 3)
    nodes = _map_nodes(PARK, 40)
    pickup = nodes[0]
    followers = [c for c in cs if c.id != leader.id]
    near, far = _near_and_far(PARK, pickup)
    fleet.inject(followers[0], latest_node_id=near, region_id=PARK, battery=90.0,
                 state="IDLE", mission="IDLE")
    fleet.inject(followers[1], latest_node_id=far, region_id=PARK, battery=90.0,
                 state="IDLE", mission="IDLE")
    assert wait_until(lambda: {p.latest_node_id for p in fleet.state(leader).roster} >= {near, far},
                      20, what="both positions to reach the roster")

    ack = fleet.submit_job(leader, "B1", pickup, nodes[5])
    assert ack.accepted
    assert ack.assignee == fleet.state(followers[0]).bot_id, \
        "the far bot was assigned over the near one"


@pytest.mark.docker
def test_B2_the_charge_bucket_beats_distance(fleet):
    """A nearer bot that will need charging mid-job is the wrong bot."""
    cs, leader = _free_fleet(fleet, 3)
    nodes = _map_nodes(PARK, 40)
    pickup = nodes[0]
    followers = [c for c in cs if c.id != leader.id]
    near, far = _near_and_far(PARK, pickup)
    fleet.inject(followers[0], latest_node_id=near, region_id=PARK, battery=35.0,
                 state="IDLE", mission="IDLE")
    fleet.inject(followers[1], latest_node_id=far, region_id=PARK, battery=95.0,
                 state="IDLE", mission="IDLE")
    assert wait_until(lambda: {round(p.battery) for p in fleet.state(leader).roster} >= {35, 95},
                      20, what="both batteries to reach the roster")

    ack = fleet.submit_job(leader, "B2", pickup, nodes[5])
    assert ack.accepted
    assert ack.assignee == fleet.state(followers[1]).bot_id, \
        "the nearly-flat bot was sent on a job"


@pytest.mark.docker
def test_B3_a_bot_already_carrying_a_job_is_not_given_another(fleet):
    _, leader = _free_fleet(fleet, 2)
    nodes = _map_nodes(PARK, 12)
    first = fleet.submit_job(leader, "B3-a", nodes[4], nodes[5])
    assert first.accepted

    second = fleet.submit_job(leader, "B3-b", nodes[6], nodes[7])
    if second.accepted:
        assert second.assignee != first.assignee, "one bot was given two jobs"


@pytest.mark.docker
def test_B4_a_flat_bot_is_never_assigned(fleet):
    cs, leader = _free_fleet(fleet, 2)
    nodes = _map_nodes(PARK, 12)
    follower = [c for c in cs if c.id != leader.id][0]
    flat = fleet.state(follower).bot_id
    fleet.inject(follower, latest_node_id=nodes[1], region_id=PARK, battery=5.0,
                 state="IDLE", mission="IDLE")
    assert wait_until(lambda: any(p.bot_id == flat and p.battery < 10
                                  for p in fleet.state(leader).roster), 20)

    ack = fleet.submit_job(leader, "B4", nodes[4], nodes[5])
    if ack.accepted:
        assert ack.assignee != flat, "a bot below JOB_MIN_BATTERY took a job"


@pytest.mark.docker
def test_B5_a_faulted_bot_is_never_assigned(fleet):
    cs, leader = _free_fleet(fleet, 2)
    nodes = _map_nodes(PARK, 12)
    follower = [c for c in cs if c.id != leader.id][0]
    broken = fleet.state(follower).bot_id
    fleet.inject(follower, latest_node_id=nodes[1], region_id=PARK, battery=90.0,
                 state="FAULTED", mission="IDLE", fault="MOTOR_ERROR")
    assert wait_until(lambda: any(p.bot_id == broken and p.fault
                                  for p in fleet.state(leader).roster), 20)

    ack = fleet.submit_job(leader, "B5", nodes[4], nodes[5])
    if ack.accepted:
        assert ack.assignee != broken, "a broken bot took a job"


@pytest.mark.docker
def test_B6_the_leader_takes_a_job_only_when_nobody_else_can(fleet):
    """Leading and carrying at once is allowed, but it is the last resort: a
    leader that drives away has to hand off first."""
    cs, leader = _free_fleet(fleet, 2)
    nodes = _map_nodes(PARK, 12)
    follower = [c for c in cs if c.id != leader.id][0]
    fleet.inject(follower, latest_node_id=nodes[1], region_id=PARK, battery=5.0,
                 state="IDLE", mission="IDLE")
    assert wait_until(lambda: any(p.battery < 10 for p in fleet.state(leader).roster), 20)

    ack = fleet.submit_job(leader, "B6", nodes[4], nodes[5])
    assert ack.accepted
    assert ack.assignee == fleet.state(leader).bot_id, "the leader should have taken it"


@pytest.mark.docker
def test_B7_bot_zero_is_assignable(fleet):
    """Regression: bot_id 0 is falsy, and an `if assignee:` once made the first
    bot up.py starts quietly unassignable."""
    cs, _ = _free_fleet(fleet, 1)
    nodes = _map_nodes(PARK, 12)
    assert fleet.state(cs[0]).bot_id == 0

    ack = fleet.submit_job(cs[0], "B7", nodes[4], nodes[5])
    assert ack.accepted
    assert ack.assignee == 0


@pytest.mark.docker
def test_B8_submitting_the_same_job_twice_assigns_it_once(fleet):
    """The order system may retry. `job_id` is the cargo id, so a repeat is the
    same job, not a second one."""
    _, leader = _free_fleet(fleet, 2)
    nodes = _map_nodes(PARK, 12)
    first = fleet.submit_job(leader, "B8", nodes[4], nodes[5])
    second = fleet.submit_job(leader, "B8", nodes[4], nodes[5])
    assert first.accepted and second.accepted
    assert first.assignee == second.assignee
    ledger = [j for j in fleet.state(leader).jobs if j.job_id == "B8"]
    assert len(ledger) == 1, "the same cargo was booked twice"


@pytest.mark.docker
def test_B9_a_job_handed_to_a_follower_reaches_its_leader(fleet):
    cs, leader = _free_fleet(fleet, 2)
    nodes = _map_nodes(PARK, 12)
    follower = [c for c in cs if c.id != leader.id][0]

    ack = fleet.submit_job(follower, "B9", nodes[4], nodes[5])
    assert ack.accepted, ack.note
    assert wait_until(lambda: any(j.job_id == "B9" for j in fleet.state(leader).jobs), 20,
                      what="the job to reach the leader's ledger")


@pytest.mark.docker
def test_B12_a_job_runs_from_pickup_to_delivered(fleet):
    """The whole lifecycle, driven the way a robot drives it."""
    cs, leader = _free_fleet(fleet, 2)
    nodes = _map_nodes(PARK, 12)
    pickup, dropoff = nodes[6], nodes[10]
    ack = fleet.submit_job(leader, "B12", pickup, dropoff)
    assert ack.accepted
    holder = _holder_of(fleet, cs, "B12")
    assert holder is not None

    fleet.inject(holder, latest_node_id=pickup, region_id=PARK, battery=90.0,
                 state="IDLE", mission="CARGO", job_id="B12", cargo_state="EN_ROUTE")
    assert wait_until(lambda: fleet.state(holder).cargo_state == "EN_ROUTE", 20, what="pickup")

    fleet.inject(holder, latest_node_id=dropoff, region_id=PARK, battery=90.0,
                 state="IDLE", mission="CARGO", job_id="B12", cargo_state="DROPOFF")
    assert wait_until(lambda: fleet.state(holder).cargo_state == "DROPOFF", 20, what="dropoff")

    # Mission leaves CARGO: the robot has set the cargo down.
    fleet.inject(holder, latest_node_id=dropoff, region_id=PARK, battery=90.0,
                 state="IDLE", mission="IDLE")
    assert wait_until(lambda: fleet.state(holder).current_job_id == "", 30,
                      what="the job to be crossed off")
    assert wait_until(
        lambda: not any(j.job_id == "B12" and j.status not in ("DELIVERED",)
                        for j in fleet.state(leader).jobs), 30,
        what="the ledger to close the job")


@pytest.mark.docker
def test_B10_a_job_for_another_region_is_forwarded_to_its_leader(fleet):
    """Ownership follows the pickup: the region that can actually reach the
    cargo is the region that assigns and crosses off."""
    _, park_leader = _free_fleet(fleet, 2, PARK)
    grid = fleet.launch(1, GRID, start_id=10, **FAST_TIMINGS)
    assert wait_until(lambda: fleet.converged(grid, GRID), 30, what="the second region")
    grid_node = _map_nodes(GRID, 4)[0]
    fleet.inject(grid[0], latest_node_id=grid_node, region_id=GRID, battery=90.0,
                 state="IDLE", mission="IDLE")
    assert wait_until(
        lambda: {ld.region_id for ld in fleet.state(park_leader).other_leaders} >= {GRID},
        30, what="the leaders to find each other")

    ack = fleet.submit_job(park_leader, "B10", grid_node, _map_nodes(GRID, 4)[1])
    assert ack.accepted, ack.note
    assert ack.owner_region == GRID, \
        f"a job for region {GRID} was kept by region {ack.owner_region}"


@pytest.mark.docker
def test_B11_a_job_nobody_can_take_is_queued_and_retried(fleet):
    """Refusing it would lose the cargo. It waits for a bot instead."""
    cs, leader = _free_fleet(fleet, 2)
    nodes = _map_nodes(PARK, 12)
    follower = [c for c in cs if c.id != leader.id][0]
    for c in cs:
        fleet.inject(c, latest_node_id=nodes[1], region_id=PARK, battery=5.0,
                     state="IDLE", mission="IDLE")
    assert wait_until(lambda: all(p.battery < 10 for p in fleet.state(leader).roster), 20,
                      what="everyone to look too flat to work")

    ack = fleet.submit_job(leader, "B11", nodes[4], nodes[5])
    assert ack.accepted, "a job nobody can take must still be accepted, not dropped"
    assert wait_until(lambda: any(j.job_id == "B11" for j in fleet.state(leader).jobs), 20,
                      what="the job to be queued")

    # Charge one up; the retry should find it.
    fleet.inject(follower, latest_node_id=nodes[1], region_id=PARK, battery=95.0,
                 state="IDLE", mission="IDLE")
    assert wait_until(lambda: _holder_of(fleet, cs, "B11") is not None, 40,
                      what="the queued job to be picked up once a bot was free")


# -----------------------------------------------------------------------------
# D. Exceptions (docs/scenarios.md D)
#
# Escalation is logged rather than exposed in BotState, because it is an
# operational event rather than fleet state -- so these read the container's own
# log for the rungs, and BotState for the consequences.

def _logs(c) -> str:
    return c.logs().decode("utf-8", "replace")


@pytest.mark.docker
def test_D1_D2_D3_a_stalled_bot_escalates_through_its_rungs(fleet):
    """Cheapest suspicion first: our route is stale, then something will not
    move for us, then a person should look.

    Each rung is a whole T_STALL, so a robot pausing for traffic never trips it.
    """
    cs, leader = _free_fleet(fleet, 2)
    nodes = _map_nodes(PARK, 12)
    assert fleet.submit_job(leader, "D1", nodes[6], nodes[10]).accepted
    holder = _holder_of(fleet, cs, "D1")
    assert holder is not None

    # Stop reporting movement: same node, over and over.
    stuck_at = nodes[0]
    for _ in range(20):
        fleet.inject(holder, latest_node_id=stuck_at, region_id=PARK, battery=90.0,
                     state="MOVING", mission="CARGO", job_id="D1", cargo_state="PICKUP")
        time.sleep(0.3)

    log = _logs(holder)
    assert "stalled at node" in log, "rung 1: the route should have been dropped"
    assert "will stand aside" in log, "rung 2: it should have released its claims"
    assert "stuck at node" in log, "rung 3: it should have escalated"


@pytest.mark.docker
def test_D4_a_fault_before_pickup_gives_the_job_back(fleet):
    """The cargo is still on the floor, so another bot can take it -- and this
    one must let go, or it will resume after recovering and two bots will go."""
    cs, leader = _free_fleet(fleet, 2)
    nodes = _map_nodes(PARK, 12)
    assert fleet.submit_job(leader, "D4", nodes[6], nodes[10]).accepted
    holder = _holder_of(fleet, cs, "D4")
    assert holder is not None

    fleet.inject(holder, latest_node_id=nodes[1], region_id=PARK, battery=90.0,
                 state="FAULTED", mission="CARGO", fault="MOTOR_ERROR",
                 job_id="D4", cargo_state="PICKUP")
    assert wait_until(lambda: fleet.state(holder).current_job_id == "", 20,
                      what="the broken bot to drop the job it had not collected")


@pytest.mark.docker
def test_D5_a_fault_after_pickup_keeps_the_job_and_raises_it(fleet):
    """The cargo is physically on this bot. Dropping the job would lose track of
    where it is; the fleet needs a person instead."""
    cs, leader = _free_fleet(fleet, 2)
    nodes = _map_nodes(PARK, 12)
    assert fleet.submit_job(leader, "D5", nodes[6], nodes[10]).accepted
    holder = _holder_of(fleet, cs, "D5")
    assert holder is not None

    fleet.inject(holder, latest_node_id=nodes[6], region_id=PARK, battery=90.0,
                 state="IDLE", mission="CARGO", job_id="D5", cargo_state="EN_ROUTE")
    assert wait_until(lambda: fleet.state(holder).cargo_state == "EN_ROUTE", 20)

    fleet.inject(holder, latest_node_id=nodes[6], region_id=PARK, battery=90.0,
                 state="FAULTED", mission="CARGO", fault="MOTOR_ERROR",
                 job_id="D5", cargo_state="EN_ROUTE")
    assert wait_until(lambda: fleet.state(holder).current_job_id == "D5", 10), \
        "a bot carrying cargo must keep the job so its heartbeats keep saying where it is"
    assert wait_until(
        lambda: any(j.job_id == "D5" and j.status == "NEEDS_ATTENTION"
                    for j in fleet.state(leader).jobs), 30,
        what="the stranded cargo to be escalated")


@pytest.mark.docker
def test_D6_the_link_survives_a_companion_that_goes_away(one_bot):
    """A companion dying is a shift ending, not an error. The next one connects
    to the same listener."""
    fleet, cs = one_bot
    node = _map_nodes(PARK, 1)[0]
    fleet.inject(cs[0], latest_node_id=node, region_id=PARK, battery=90.0,
                 state="IDLE", mission="IDLE")

    first = fleet.ask(cs[0], _query(node, _neighbours(node), query_id=1))
    assert first is not None
    # `ask` opens and closes a connection each time, so the second call is a
    # reconnection by definition.
    second = fleet.ask(cs[0], _query(node, _neighbours(node), query_id=2))
    assert second is not None, "the listener did not survive the first companion leaving"
    assert second["query_id"] == 2


@pytest.mark.docker
def test_D7_a_killed_bot_stops_blocking_the_lane_it_held(fleet):
    """Claims lapse. A bot that dies holding a corridor must not hold it for
    ever, or one crash closes a lane for the rest of the shift."""
    cs = fleet.launch(3, PARK, **FAST_TIMINGS)
    assert wait_until(lambda: fleet.converged(cs, PARK), 30, what="converge")
    corridor = _corridor(6)
    for c, node in zip(cs, corridor[:3], strict=False):
        fleet.inject(c, latest_node_id=node, region_id=PARK, battery=90.0,
                     state="IDLE", mission="IDLE")

    victim, watcher = cs[0], cs[1]
    victim_id = fleet.state(victim).bot_id
    assert wait_until(lambda: any(r.bot_id == victim_id for r in fleet.state(watcher).reservations),
                      20, what="the victim's claim to arrive")

    victim.kill()
    assert wait_until(
        lambda: not any(r.bot_id == victim_id for r in fleet.state(watcher).reservations),
        30, what="the dead bot's claims to lapse")


# -----------------------------------------------------------------------------
# E. Collisions (docs/scenarios.md E)

@pytest.mark.docker
def test_E1_two_bots_on_one_node_settle_on_a_single_holder(two_bots):
    """Both want it, both apply the same ordering, and only one keeps it."""
    fleet, cs = two_bots
    fleet.reset(cs)
    node = _map_nodes(PARK, 1)[0]
    for c in cs:
        fleet.inject(c, latest_node_id=node, region_id=PARK, battery=90.0,
                     state="IDLE", mission="IDLE")

    def one_holder() -> bool:
        holders = set()
        for c in cs:
            holders |= {r.bot_id for r in fleet.state(c).reservations if r.node_id == node}
        return len(holders) == 1

    assert wait_until(one_holder, 30, what="one of them to give way")


@pytest.mark.docker
def test_E2_three_bots_on_one_node_still_settle_on_one(three_bots):
    """The ordering is total, so a third claimant changes nothing about it."""
    fleet, cs = three_bots
    fleet.reset(cs)
    node = _map_nodes(PARK, 1)[0]
    for c in cs:
        fleet.inject(c, latest_node_id=node, region_id=PARK, battery=90.0,
                     state="IDLE", mission="IDLE")

    def one_holder() -> bool:
        holders = set()
        for c in cs:
            holders |= {r.bot_id for r in fleet.state(c).reservations if r.node_id == node}
        return len(holders) == 1

    assert wait_until(one_holder, 30, what="two of the three to give way")
    fleet.assert_no_overlap(cs)


@pytest.mark.docker
def test_E4_a_following_bot_does_not_close_up_on_the_one_ahead(two_bots):
    """The overlapping-claim rule keeps a node held until the robot is fully
    inside the next one, so a follower cannot arrive early."""
    fleet, cs = two_bots
    fleet.reset(cs)
    corridor = _corridor(6)
    leader_node, follower_node = corridor[2], corridor[1]
    # The follower is the one doing the routing, so the job has to land on it.
    follower_c = _bot_with_a_goal(fleet, cs, "E4", follower_node)
    ahead_c = next(c for c in cs if c is not follower_c)
    fleet.inject(ahead_c, latest_node_id=leader_node, region_id=PARK, battery=90.0,
                 state="IDLE", mission="IDLE")
    ahead_id = fleet.state(ahead_c).bot_id
    assert wait_until(lambda: any(r.bot_id == ahead_id for r in fleet.state(follower_c).reservations),
                      20, what="the leader's claim to reach the follower")

    reply = _planned(fleet.ask(follower_c, _query(follower_node, _neighbours(follower_node))))
    if reply["kind"] in ("PROCEED", "REROUTE"):
        assert reply["target_node_id"] != leader_node, "it drove into an occupied node"
    fleet.assert_no_overlap(cs)


# -----------------------------------------------------------------------------
# F. Redirections (docs/scenarios.md F)

@pytest.mark.docker
def test_F2_an_obstruction_is_routed_around(two_bots):
    """The planner has always supported obstructions; nothing fed it one until
    now. See ObstructionMsg in fleet.proto for what this shortcut skips.

    Obstructions ride on the planning `Request`, not on the traffic view -- the
    search is the only thing that prices them. This scenario is what proves
    that wire is connected, so it takes the lane it is given rather than
    tolerating a bot that never planned.
    """
    fleet, cs = two_bots
    fleet.reset(cs)
    node = _map_nodes(PARK, 1)[0]
    ours = _bot_with_a_goal(fleet, cs, "F2", node)

    before = _planned(fleet.ask(ours, _query(node, _neighbours(node))), "before the block")
    assert before["kind"] in ("PROCEED", "REROUTE"), \
        f"nothing was in the way, so there is a lane to block; got {before}"
    blocked = before["target_node_id"]

    fleet.obstruct(ours, blocked, level=1.0)
    after = _planned(fleet.ask(ours, _query(node, _neighbours(node), query_id=2)),
                     "after the block")
    assert after["kind"] in ("PROCEED", "REROUTE", "WAIT", "YIELD"), \
        "an obstruction is not a reason to go silent"
    if after["kind"] in ("PROCEED", "REROUTE"):
        assert after["target_node_id"] != blocked, "it drove into a node reported blocked"
    fleet.obstruct(ours, blocked, level=0.0)


@pytest.mark.docker
def test_F3_clearing_an_obstruction_opens_the_lane_again(two_bots):
    """A blockage that is gone must stop costing anything, or the fleet slowly
    forgets lanes it can use."""
    fleet, cs = two_bots
    fleet.reset(cs)
    node = _map_nodes(PARK, 1)[0]
    ours = _bot_with_a_goal(fleet, cs, "F3", node)
    lane = list(_neighbours(node).values())[0]

    fleet.obstruct(ours, lane, level=1.0)
    blocked = _planned(fleet.ask(ours, _query(node, _neighbours(node), query_id=1)),
                       "while blocked")
    fleet.obstruct(ours, lane, level=0.0)
    cleared = _planned(fleet.ask(ours, _query(node, _neighbours(node), query_id=2)),
                       "once cleared")

    if blocked["kind"] in ("PROCEED", "REROUTE"):
        assert blocked["target_node_id"] != lane
    # With nothing in the way the lane is allowed again; the point is that the
    # obstruction stopped applying, not which lane wins.
    assert cleared["kind"] in ("PROCEED", "REROUTE", "WAIT", "YIELD")


@pytest.mark.docker
def test_F4_a_bot_that_migrates_replans_on_arrival(fleet):
    """Regions are subnetworks: a route across one is planned optimistically and
    only becomes informed once its roster arrives."""
    park, park_leader = _free_fleet(fleet, 2, PARK)
    grid = fleet.launch(1, GRID, start_id=10, **FAST_TIMINGS)
    assert wait_until(lambda: fleet.converged(grid, GRID), 30, what="the second region")
    assert wait_until(
        lambda: {ld.region_id for ld in fleet.state(park_leader).other_leaders} >= {GRID},
        30, what="the leaders to meet")

    mover = [c for c in park if c.id != park_leader.id][0]
    grid_node = _map_nodes(GRID, 4)[0]
    fleet.inject(mover, latest_node_id=grid_node, region_id=GRID, battery=90.0,
                 state="IDLE", mission="IDLE")
    assert wait_until(lambda: fleet.state(mover).region_id == GRID, 40, what="the migration")

    reply = fleet.ask(mover, _query(grid_node, _neighbours(grid_node), region=GRID))
    assert reply is not None, "it went quiet in its new region"


@pytest.mark.docker
def test_F6_a_peers_claim_between_two_questions_changes_the_answer(two_bots):
    """Traffic is not static between one node and the next, and the answer has
    to move with it.

    A claim has to outlive the drive it is meant to prevent, and for a while
    this scenario could not be made to hold. The reason was not the TTL -- that
    governs when a *received* claim lapses -- but the window the sender
    announces, which was two announce periods and so expired before a neighbour
    two seconds away could arrive. `ReservationSender._hold_ms` now covers a
    traversal, so the shared fleet is enough and this needs no clock of its own.
    """
    fleet, cs = two_bots
    fleet.reset(cs)
    corridor = _corridor(6)
    ours = corridor[0]
    ours_c = _bot_with_a_goal(fleet, cs, "F6", ours)
    other_c = next(c for c in cs if c is not ours_c)

    first = _planned(fleet.ask(ours_c, _query(ours, _neighbours(ours))), "on the first ask")
    assert first["kind"] in ("PROCEED", "REROUTE"), \
        f"nothing was in the way, so there is a lane to contest; got {first}"
    contested = first["target_node_id"]

    fleet.inject(other_c, latest_node_id=contested, region_id=PARK, battery=90.0,
                 state="IDLE", mission="IDLE")
    other = fleet.state(other_c).bot_id
    assert wait_until(
        lambda: any(r.bot_id == other and r.node_id == contested
                    for r in fleet.state(ours_c).reservations),
        20, what="the peer's claim to arrive")

    second = _planned(fleet.ask(ours_c, _query(ours, _neighbours(ours), query_id=2)),
                      "on the second ask")
    if second["kind"] in ("PROCEED", "REROUTE"):
        assert second["target_node_id"] != contested, \
            "it kept driving at a node a peer had since claimed"


# -----------------------------------------------------------------------------
# G. Yielding (PROTOCOL.md §16.4, docs/scenarios.md G)
#
# A yield needs a wait long enough to be worth leaving the route for. A claim
# lives about two announce periods, so on the compressed clock the threshold is
# dropped to match -- otherwise no wait here would ever be long enough and the
# rule would never be reached.

YIELD_TIMINGS = dict(FAST_TIMINGS, T_YIELD_THRESHOLD=0.05)


@pytest.mark.docker
def test_G1_the_free_bot_gives_way_to_the_one_carrying_cargo(fleet):
    """Right of way is not about who got there first. A robot with cargo aboard
    is not asked to reverse out of a corridor for an idle one."""
    cs = fleet.launch(2, PARK, **YIELD_TIMINGS)
    assert wait_until(lambda: fleet.converged(cs, PARK), 30, what="converge")
    corridor = _corridor(8)
    ours, theirs = corridor[3], corridor[4]

    nodes = _map_nodes(PARK, 12)
    for c in cs:
        fleet.inject(c, latest_node_id=nodes[0], region_id=PARK, battery=90.0,
                     state="IDLE", mission="IDLE")
    leader = fleet.leaders(cs)[0]
    assert wait_until(lambda: all(p.mission == "IDLE" for p in fleet.state(leader).roster), 20)
    assert fleet.submit_job(leader, "G1", nodes[6], nodes[10]).accepted
    carrier = _holder_of(fleet, cs, "G1")
    assert carrier is not None
    free = [c for c in cs if c.id != carrier.id][0]

    fleet.inject(carrier, latest_node_id=theirs, region_id=PARK, battery=90.0,
                 state="IDLE", mission="CARGO", job_id="G1", cargo_state="EN_ROUTE")
    assert wait_until(lambda: fleet.state(carrier).cargo_state == "EN_ROUTE", 20)
    fleet.inject(free, latest_node_id=ours, region_id=PARK, battery=90.0,
                 state="IDLE", mission="IDLE")
    carrier_id = fleet.state(carrier).bot_id
    assert wait_until(lambda: any(r.bot_id == carrier_id for r in fleet.state(free).reservations),
                      20, what="the carrier's claim to reach the free bot")

    reply = fleet.ask(free, _query(ours, _neighbours(ours)))
    assert reply is not None
    assert reply["kind"] != "PROCEED" or reply["target_node_id"] != theirs, \
        "the free bot drove at a robot carrying cargo"


@pytest.mark.docker
def test_G2_the_lower_bot_id_holds_when_ranks_are_equal(fleet):
    """Both free, so the tiebreak decides -- and both compute it identically,
    which is what stops them both moving or both staying."""
    cs = fleet.launch(2, PARK, **YIELD_TIMINGS)
    assert wait_until(lambda: fleet.converged(cs, PARK), 30, what="converge")
    corridor = _corridor(8)
    by_id = sorted(cs, key=lambda c: fleet.state(c).bot_id)
    lower, higher = by_id[0], by_id[1]
    fleet.inject(lower, latest_node_id=corridor[3], region_id=PARK, battery=90.0,
                 state="IDLE", mission="IDLE")
    fleet.inject(higher, latest_node_id=corridor[4], region_id=PARK, battery=90.0,
                 state="IDLE", mission="IDLE")
    higher_id = fleet.state(higher).bot_id
    assert wait_until(lambda: any(r.bot_id == higher_id for r in fleet.state(lower).reservations),
                      20, what="claims to be exchanged")

    reply = fleet.ask(lower, _query(corridor[3], _neighbours(corridor[3])))
    assert reply is not None
    assert reply["kind"] != "YIELD", "the bot that wins the tiebreak should hold, not give way"


@pytest.mark.docker
def test_G7_exactly_one_side_of_a_contest_gives_way(fleet):
    """Never both, never neither. Both sides run the same rule on the same two
    numbers, so the verdicts have to agree without them talking about it."""
    cs = fleet.launch(2, PARK, **YIELD_TIMINGS)
    assert wait_until(lambda: fleet.converged(cs, PARK), 30, what="converge")
    corridor = _corridor(8)
    fleet.inject(cs[0], latest_node_id=corridor[3], region_id=PARK, battery=90.0,
                 state="IDLE", mission="IDLE")
    fleet.inject(cs[1], latest_node_id=corridor[4], region_id=PARK, battery=90.0,
                 state="IDLE", mission="IDLE")
    assert wait_until(lambda: len(fleet.state(cs[0]).reservations) > 1, 20,
                      what="claims to be exchanged")

    a = fleet.ask(cs[0], _query(corridor[3], _neighbours(corridor[3])))
    b = fleet.ask(cs[1], _query(corridor[4], _neighbours(corridor[4])))
    assert a is not None and b is not None
    yields = [r["kind"] == "YIELD" for r in (a, b)]
    assert not all(yields), "both gave way, so nobody moves"
    fleet.assert_no_overlap(cs)


@pytest.mark.docker
def test_E6_no_node_is_ever_held_by_two_bots_at_once(three_bots):
    """Guarantee 2, on its own rather than only as a side-check.

    Three bots crowded onto adjacent nodes of one corridor is the shape most
    likely to break it: short lanes, no way round, and every claim in every
    other bot's range.
    """
    fleet, cs = three_bots
    fleet.reset(cs)
    corridor = _corridor(6)

    for c, node in zip(cs, corridor[:3], strict=False):
        fleet.inject(c, latest_node_id=node, region_id=PARK, battery=90.0,
                     state="IDLE", mission="IDLE")
    assert wait_until(lambda: len(fleet.state(cs[0]).reservations) >= 2, 20,
                      what="claims to be exchanged")

    # Shuffle them along the corridor and re-check after every move: a single
    # snapshot could miss an overlap that existed only between two of them.
    for step in range(3):
        for c, node in zip(cs, corridor[step:step + 3], strict=False):
            fleet.inject(c, latest_node_id=node, region_id=PARK, battery=90.0,
                         state="MOVING", mission="IDLE")
        time.sleep(0.4)
        fleet.assert_no_overlap(cs)
