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

import up  # noqa: E402
from bus.policy import rpc_metadata  # noqa: E402
from proto import fleet_pb2, fleet_pb2_grpc  # noqa: E402

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
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            if pred():
                return True
        except grpc.RpcError:
            pass  # container not up / paused / partitioned — keep polling
        time.sleep(step)
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
    f = DockerFleet(client, image)
    yield f
    f.close()


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
def test_a_claim_crosses_a_real_network(fleet):
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
def test_claims_keep_flowing_when_the_leader_dies(fleet):
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
    watcher_sees = lambda: _claims_of(fleet, survivors[1], fleet.state(survivors[0]).bot_id)
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
def test_a_bot_that_goes_quiet_stops_blocking_its_neighbours(fleet):
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
    names = ("straight", "left", "right")
    return {n: node for n, node in zip(names, warehouse.neighbours(node_id))}


@pytest.mark.docker
def test_a_bot_with_no_job_still_answers(fleet):
    """Silence is the one answer that is never allowed.

    A robot that hears nothing sits there for the rest of its shift, because it
    only asks again on reaching the next node -- and it will not reach one.
    """
    cs = fleet.launch(1, PARK)
    assert wait_until(lambda: fleet.converged(cs, PARK), 30, what="converge")
    node = _map_nodes(PARK, 1)[0]

    reply = fleet.ask(cs[0], _query(node, _neighbours(node)))
    assert reply is not None, "the bot said nothing at all"
    assert reply["kind"] == "WAIT"
    assert reply["hold_ms"] > 0, "a hold of zero would have it ask in a tight loop"


@pytest.mark.docker
def test_a_bot_given_a_job_is_routed_towards_it(fleet):
    """The whole point: a job becomes turns, one node at a time."""
    cs = fleet.launch(2, PARK)
    assert wait_until(lambda: fleet.converged(cs, PARK), 30, what="converge")

    nodes = _map_nodes(PARK, 8)
    start, pickup, dropoff = nodes[0], nodes[6], nodes[7]
    for c in cs:
        fleet.inject(c, latest_node_id=start, region_id=PARK,
                     battery=90.0, state="IDLE", mission="IDLE")
    # The leader assigns from what its roster says, and the roster is only as
    # fresh as the last heartbeat.
    assert wait_until(
        lambda: all(p.mission == "IDLE" for p in fleet.state(cs[0]).roster), 20,
        what="the roster to catch up with the injected state")

    ack = fleet.submit_job(cs[0], "job-nav", pickup, dropoff)
    assert ack.accepted, ack.note

    # Which bot takes it is the dispatcher's call, not this test's.
    holder = None
    def assigned() -> bool:
        nonlocal holder
        for c in cs:
            if fleet.state(c).current_job_id == "job-nav":
                holder = c
                return True
        return False
    assert wait_until(assigned, 20, what="the job to reach a bot")

    reply = fleet.ask(holder, _query(start, _neighbours(start)))
    assert reply is not None
    assert reply["kind"] in ("PROCEED", "REROUTE", "WAIT", "YIELD"), reply
    if reply["kind"] in ("PROCEED", "REROUTE"):
        assert reply["turn"] in ("left", "straight", "right")
        assert reply["target_node_id"] in _neighbours(start).values(), \
            "it must name a lane the robot said exists"


@pytest.mark.docker
def test_a_query_offering_turns_we_did_not_plan_is_still_answered(fleet):
    """Our map and the robot's can disagree. A wrong turn is recoverable at the
    next node; silence is not recoverable at all."""
    cs = fleet.launch(1, PARK)
    assert wait_until(lambda: fleet.converged(cs, PARK), 30, what="converge")
    node = _map_nodes(PARK, 1)[0]

    reply = fleet.ask(cs[0], _query(node, {"left": 999999}))
    assert reply is not None, "a disagreement must not silence the bot"
    assert reply["kind"] in ("WAIT", "PROCEED", "REROUTE", "YIELD")


@pytest.mark.docker
def test_a_malformed_query_does_not_break_the_link(fleet):
    """One bad line must not cost the robot the rest of its shift."""
    cs = fleet.launch(1, PARK)
    assert wait_until(lambda: fleet.converged(cs, PARK), 30, what="converge")
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
