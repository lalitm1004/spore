"""B. Job distribution.

One of the `test_docker_*.py` files; see `docs/scenarios.md` for what each
scenario claims and `tests/containers/harness.py` for the fleet they run on.

B. Job distribution (PROTOCOL.md §14, docs/scenarios.md B)

These take their own fleets: a job left half-finished in a leader's ledger is
exactly the kind of state that makes the next scenario lie.
"""
from __future__ import annotations

import grpc
import pytest

from tests.containers.harness import (
    _free_fleet,
    _holder_of,
    _map_nodes,
    _near_and_far,
    _place_order,
    FAST_TIMINGS,
    GRID,
    PARK,
    controlplane_pb2,
    controlplane_pb2_grpc,
    rpc_metadata,
    wait_until,
)

pytestmark = pytest.mark.docker


@pytest.mark.docker
def test_B1_the_nearest_free_follower_is_assigned(fleet):
    cs, leader = _free_fleet(fleet, 3)
    nodes = _map_nodes(PARK, 40)
    pickup = nodes[0]
    followers = [c for c in cs if c.id != leader.id]
    near, far = _near_and_far(PARK, pickup)
    fleet.place(followers[0], latest_node_id=near, region_id=PARK, battery=90.0,
                 state="IDLE", mission="IDLE")
    fleet.place(followers[1], latest_node_id=far, region_id=PARK, battery=90.0,
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
    fleet.place(followers[0], latest_node_id=near, region_id=PARK, battery=35.0,
                 state="IDLE", mission="IDLE")
    fleet.place(followers[1], latest_node_id=far, region_id=PARK, battery=95.0,
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
    fleet.place(follower, latest_node_id=nodes[1], region_id=PARK, battery=5.0,
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
    fleet.place(follower, latest_node_id=nodes[1], region_id=PARK, battery=90.0,
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
    fleet.place(follower, latest_node_id=nodes[1], region_id=PARK, battery=5.0,
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
def test_B15_two_orders_at_once_go_to_two_different_bots(fleet):
    """The dispatcher picks the nearest free bot, and *free* is the word doing
    the work. Two orders arriving together must not both pick the same bot
    because neither had been marked busy yet -- the classic read-then-assign
    race, and the reason assignment happens under the ledger lock."""
    cs, leader = _free_fleet(fleet, 2)
    nodes = _map_nodes(PARK, 12)

    for i, (pickup, dropoff) in enumerate([(nodes[4], nodes[8]), (nodes[6], nodes[10])]):
        assert _place_order(fleet, leader, f"B15-{i}", pickup, dropoff).accepted

    holders = {fleet.state(c).current_job_id for c in cs}
    assert holders == {"B15-0", "B15-1"}, \
        f"two orders did not reach two bots: {holders}"


@pytest.mark.docker
def test_B16_more_orders_than_bots_are_queued_and_drain(fleet):
    """A burst is not a reason to lose cargo. Everything above the number of
    free bots is accepted and held, and comes out as bots finish."""
    cs, leader = _free_fleet(fleet, 2)
    nodes = _map_nodes(PARK, 12)

    for i in range(5):
        assert _place_order(fleet, leader, f"B16-{i}", nodes[4], nodes[8]).accepted, \
            f"order B16-{i} was refused; a burst must queue, not drop"

    ledger = {j.job_id for j in fleet.state(leader).jobs}
    assert {f"B16-{i}" for i in range(5)} <= ledger, \
        f"orders vanished between accepted and the ledger: {sorted(ledger)}"

    assigned = {fleet.state(c).current_job_id for c in cs} - {""}
    assert len(assigned) <= 2, "more jobs are being executed than there are bots"


@pytest.mark.docker
def test_B17_a_successor_inherits_the_job_ledger(fleet):
    """Jobs must not die with a leader. The ledger rides in every HeartbeatAck
    for exactly this: a follower that becomes leader already holds it, and cargo
    accepted before the failure is still cargo somebody owes."""
    cs, leader = _free_fleet(fleet, 3)
    nodes = _map_nodes(PARK, 12)
    assert _place_order(fleet, leader, "B17", nodes[4], nodes[8]).accepted
    survivors = [c for c in cs if c is not leader]
    assert wait_until(
        lambda: all(any(j.job_id == "B17" for j in fleet.state(c).jobs) for c in survivors),
        20, what="the ledger to replicate to the followers")

    leader.kill()
    assert wait_until(lambda: fleet.leaders(survivors), 30, what="a successor")

    successor = fleet.leaders(survivors)[0]
    assert any(j.job_id == "B17" for j in fleet.state(successor).jobs), \
        "the job died with the leader that accepted it"


@pytest.mark.docker
def test_B18_a_killed_assignee_gives_its_job_back(fleet):
    """A bot that stops answering has not delivered anything. Before pickup the
    cargo is still where it was, so the job has to come back and be given to
    somebody else -- otherwise an order is silently never done."""
    cs, leader = _free_fleet(fleet, 3)
    nodes = _map_nodes(PARK, 12)
    assert _place_order(fleet, leader, "B18", nodes[4], nodes[8]).accepted
    holder = _holder_of(fleet, cs, "B18")
    assert holder is not None and holder is not leader, "need a follower to kill"

    holder.kill()
    survivors = [c for c in cs if c is not holder]
    assert wait_until(
        lambda: any(fleet.state(c).current_job_id == "B18" for c in survivors),
        40, what="the job to be given to another bot")


@pytest.mark.docker
def test_B19_an_order_for_a_node_off_the_map_is_still_answered(fleet):
    """Bad orders come from outside and must not wedge anything. Whatever the
    fleet decides -- take it, queue it, refuse it -- it has to *say so*, because
    the control plane retries until something answers and a silent drop becomes
    an infinite retry loop."""
    cs, leader = _free_fleet(fleet, 2)

    ack = _place_order(fleet, leader, "B19", 10**8, 10**8 + 1)

    assert ack.note or ack.accepted, "an impossible order got no answer at all"
    assert wait_until(lambda: fleet.converged(cs, PARK), 15,
                      what="the fleet to still be healthy afterwards")


@pytest.mark.docker
def test_B13_an_order_from_the_control_plane_reaches_a_bot(fleet):
    """The other door into the job system, and the one an outside world uses.

    `spore-control-plane` speaks its own four-field `Order` -- an id and two
    nodes -- and dials it at *any* bot, because it knows no leaders and no
    regions. Everything after that is `bus.jobs`: a non-leader forwards, a
    leader resolves the pickup's region and forwards again.

    Submitted to a follower on purpose. A leader would prove the translation and
    not the routing, and routing is the part the control plane is trusting us
    with.
    """
    cs, leader = _free_fleet(fleet, 2)
    follower = next(c for c in cs if c is not leader)
    nodes = _map_nodes(PARK, 12)

    stub = controlplane_pb2_grpc.ControlPlaneServiceStub(
        grpc.insecure_channel(fleet.endpoint(follower)))
    ack = stub.DispatchOrder(
        controlplane_pb2.Order(order_id="B13", pickup_node=nodes[6],
                               dropoff_node=nodes[10]),
        timeout=15, metadata=rpc_metadata(999, 0, "orders"))

    assert ack.accepted, ack.note
    assert _holder_of(fleet, cs, "B13") is not None, \
        "the order was accepted and never reached a bot"


@pytest.mark.docker
def test_B14_the_same_order_twice_places_one_job(fleet):
    """`order_id` is the idempotency key, and the control plane's retry loop
    depends on it: it re-sends the *same* order after a timeout, across
    different bots. Without this the fleet would send two robots for one box."""
    cs, leader = _free_fleet(fleet, 2)
    nodes = _map_nodes(PARK, 12)
    order = controlplane_pb2.Order(order_id="B14", pickup_node=nodes[6],
                                   dropoff_node=nodes[10])

    for container in cs:                     # as the retry loop would: any bot
        stub = controlplane_pb2_grpc.ControlPlaneServiceStub(
            grpc.insecure_channel(fleet.endpoint(container)))
        assert stub.DispatchOrder(order, timeout=15,
                                  metadata=rpc_metadata(999, 0, "orders")).accepted

    holders = [c for c in cs if fleet.state(c).current_job_id == "B14"]
    assert len(holders) <= 1, "two bots took one order"
    assert len([j for j in fleet.state(leader).jobs if j.job_id == "B14"]) == 1


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

    fleet.place(holder, latest_node_id=pickup, region_id=PARK, battery=90.0,
                 state="IDLE", mission="CARGO", job_id="B12", cargo_state="EN_ROUTE")
    assert wait_until(lambda: fleet.state(holder).cargo_state == "EN_ROUTE", 20, what="pickup")

    fleet.place(holder, latest_node_id=dropoff, region_id=PARK, battery=90.0,
                 state="IDLE", mission="CARGO", job_id="B12", cargo_state="DROPOFF")
    assert wait_until(lambda: fleet.state(holder).cargo_state == "DROPOFF", 20, what="dropoff")

    # Mission leaves CARGO: the robot has set the cargo down.
    fleet.place(holder, latest_node_id=dropoff, region_id=PARK, battery=90.0,
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
    fleet.place(grid[0], latest_node_id=grid_node, region_id=GRID, battery=90.0,
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
        fleet.place(c, latest_node_id=nodes[1], region_id=PARK, battery=5.0,
                     state="IDLE", mission="IDLE")
    assert wait_until(lambda: all(p.battery < 10 for p in fleet.state(leader).roster), 20,
                      what="everyone to look too flat to work")

    ack = fleet.submit_job(leader, "B11", nodes[4], nodes[5])
    assert ack.accepted, "a job nobody can take must still be accepted, not dropped"
    assert wait_until(lambda: any(j.job_id == "B11" for j in fleet.state(leader).jobs), 20,
                      what="the job to be queued")

    # Charge one up; the retry should find it.
    fleet.place(follower, latest_node_id=nodes[1], region_id=PARK, battery=95.0,
                 state="IDLE", mission="IDLE")
    assert wait_until(lambda: _holder_of(fleet, cs, "B11") is not None, 40,
                      what="the queued job to be picked up once a bot was free")
