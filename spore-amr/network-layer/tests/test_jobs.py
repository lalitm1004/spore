"""Jobs (PROTOCOL.md §14): dispatch, forwarding, progress, failure, ownership.

Bots are real gRPC servers; the *robot* is simulated by pushing `RobotState`
into a bot's source and reading `RobotCommand`s from its sink. Leaders learn
things the only way they ever do — from heartbeats — so most tests drive a
follower's heartbeat by hand with `hb()` rather than waiting on the 1 Hz loop.
"""
from __future__ import annotations

import math
import time

import grpc
import pytest

from bot import RobotState
from bus.jobs import Job, NO_BOT, PENDING, ASSIGNED, PICKED_UP, NEEDS_ATTENTION, CS_PICKUP, CS_EN_ROUTE, CS_DROPOFF, CS_DELIVERED
from election.bully import Role
from peers.table import Peer, Leader
from proto import fleet_pb2, fleet_pb2_grpc
from bus.rpc import pool
from warehouse.map import WarehouseMap
import config

from tests.conftest import md, wait_until

# Region ids from the consolidated 7-region map (was 14 before the upstream
# `consolidate warehouse regions` change): parking is now 2, and the old single
# grid_field is split across 5/6/7 -- 6 is the middle band.
PARK, GRID = 2, 6


def _stub(b, cls):
    return pool.stub(b.address, cls)


def hb(b, *, node=0, state="IDLE", mission="IDLE", fault="", job_id="", cargo_state="", battery=100.0):
    """A heartbeat *as bot b would send it*, but with the fields under test."""
    return fleet_pb2.HeartbeatRequest(
        bot_id=b.bot_id, region_id=b.region_id, latest_node_id=node, state=state, battery=battery,
        priority=b.priority, address=b.address, mission=mission, fault=fault,
        job_id=job_id, cargo_state=cargo_state, timestamp=int(time.time() * 1000),
    )


def register(leader, follower, **fields):
    """Deliver one heartbeat from follower to leader over gRPC."""
    return _stub(leader, fleet_pb2_grpc.RegionServiceStub).Heartbeat(
        hb(follower, **fields), timeout=2, metadata=md(follower.bot_id, follower.region_id))


def submit(target, job: Job, submitter_id=99, region=PARK):
    return _stub(target, fleet_pb2_grpc.JobServiceStub).SubmitJob(
        job.to_proto(), timeout=5, metadata=md(submitter_id, region))


def nodes(m: WarehouseMap, region: int, n: int) -> list[int]:
    return m.nodes_in(region)[:n]


# ---------------------------------------------------------------- map

def test_map_geometry():
    m = WarehouseMap.load(config.WAREHOUSE_MAP)
    assert isinstance(m, WarehouseMap)
    a, b = nodes(m, PARK, 2)
    assert m.region_of(a) == PARK
    assert m.distance(a, a) == 0 and m.distance(a, b) == m.distance(b, a) >= 1
    assert m.region_distance(a, PARK) == 0
    assert 0 < m.region_distance(a, GRID) < math.inf
    assert m.distance(a, 10**9) == math.inf


# ---------------------------------------------------------------- assignment

def _region(fleet, leader_id, region, follower_ids, *, leader_busy=False):
    """`leader_busy` makes the leader ineligible as the last-resort candidate,
    for tests about what happens when *nobody* is free."""
    leader = fleet.bot(leader_id, region_id=region)
    leader.role = Role.LEADER
    if leader_busy:
        leader.state = "MOVING"
    followers = [fleet.bot(i, region_id=region) for i in follower_ids]
    return leader, followers


def test_submit_assigns_nearest_free_follower(fleet):
    leader, (near, far) = _region(fleet, 9, PARK, [1, 2])
    p = leader.map.nodes_in(PARK)
    pickup = p[0]
    # Choose the near and far bots by actual driving distance, not by position in
    # the node list: ids are assigned by position on the floor, which says nothing
    # about how far apart two nodes are to drive between. Picking by index happened
    # to hold on the old 14-region map and silently stopped holding on the new one.
    by_distance = sorted(p[1:], key=lambda n: leader.map.distance(n, pickup))
    near_node, far_node = by_distance[0], by_distance[-1]
    assert leader.map.distance(near_node, pickup) < leader.map.distance(far_node, pickup)
    register(leader, near, node=near_node, mission="PARK")
    register(leader, far, node=far_node, mission="PARK")

    ack = submit(leader, Job("job-A", pickup, p[5]))
    assert ack.accepted and ack.assignee == near.bot_id and ack.owner_region == PARK
    assert leader.jobs.get("job-A").status == ASSIGNED
    assert near.current_job.job_id == "job-A" and near.cargo_state == CS_PICKUP
    cmd = near._robot_sink.pop()
    assert cmd.target_node_id == pickup and cmd.mission == {"type": "CARGO", "cargo": {"cargo_id": "job-A", "state": "PICKUP"}}
    assert far.current_job is None


def test_busy_low_battery_or_faulted_bots_are_not_free(fleet):
    leader, (busy, low, sick, ok) = _region(fleet, 9, PARK, [1, 2, 3, 4])
    p = leader.map.nodes_in(PARK)
    register(leader, busy, node=p[1], job_id="other", cargo_state=CS_EN_ROUTE)
    register(leader, low, node=p[1], battery=config.JOB_MIN_BATTERY - 1)
    register(leader, sick, node=p[1], state="FAULTED")
    register(leader, ok, node=p[-1])                     # farthest, but the only free one
    ack = submit(leader, Job("job-B", p[0], p[5]))
    assert ack.accepted and ack.assignee == ok.bot_id


def test_submit_to_follower_is_routed_to_its_leader(fleet):
    leader, (f,) = _region(fleet, 9, PARK, [1])
    p = leader.map.nodes_in(PARK)
    f.become_follower(9, leader.address)
    register(leader, f, node=p[1])
    ack = submit(f, Job("job-C", p[0], p[5]))         # handed to a follower
    assert ack.accepted and ack.assignee == f.bot_id and ack.owner_region == PARK


def test_submit_for_another_regions_pickup_goes_to_that_leader(fleet):
    park, (_,) = _region(fleet, 9, PARK, [1])
    grid, (g,) = _region(fleet, 8, GRID, [5])
    park.peer_table.upsert_leader(Leader(GRID, 8, grid.address))
    gp = grid.map.nodes_in(GRID)
    register(grid, g, node=gp[1])
    ack = submit(park, Job("job-D", gp[0], gp[9]))     # pickup is in the grid
    assert ack.accepted and ack.owner_region == GRID and ack.assignee == g.bot_id
    assert "job-D" in grid.jobs and "job-D" not in park.jobs


# ---------------------------------------------------------------- forwarding

def test_nobody_free_forwards_to_nearest_region_which_becomes_owner(fleet):
    park, (busy,) = _region(fleet, 9, PARK, [1], leader_busy=True)
    grid, (g,) = _region(fleet, 8, GRID, [5])
    park.peer_table.upsert_leader(Leader(GRID, 8, grid.address))
    pp, gp = park.map.nodes_in(PARK), grid.map.nodes_in(GRID)
    register(park, busy, node=pp[1], job_id="x", cargo_state=CS_PICKUP)
    register(grid, g, node=gp[1])
    ack = submit(park, Job("job-E", pp[0], pp[5]))
    assert ack.accepted and ack.owner_region == GRID and ack.assignee == g.bot_id
    assert "job-E" in grid.jobs and "job-E" not in park.jobs
    assert g.current_job.job_id == "job-E"


def test_nobody_free_anywhere_queues_then_retries(fleet):
    leader, (f,) = _region(fleet, 9, PARK, [1], leader_busy=True)
    pp = leader.map.nodes_in(PARK)
    ack = submit(leader, Job("job-F", pp[0], pp[5]))   # no followers registered at all
    assert ack.accepted and not ack.HasField("assignee") and ack.note == "queued"
    job = leader.jobs.get("job-F")
    assert job.status == PENDING and job.owner_region == PARK

    register(leader, f, node=pp[1])                    # someone shows up
    job.updated_at = 0.0                               # make the retry due now
    leader.dispatcher.tick()
    assert leader.jobs.get("job-F").status == ASSIGNED and f.current_job.job_id == "job-F"


def test_forwarded_queued_job_is_removed_from_forwarder(fleet):
    park, (busy,) = _region(fleet, 9, PARK, [1], leader_busy=True)
    pp = park.map.nodes_in(PARK)
    register(park, busy, node=pp[1], job_id="x", cargo_state=CS_PICKUP)
    submit(park, Job("job-G", pp[0], pp[5]))
    assert park.jobs.get("job-G").status == PENDING     # queued: nobody, no other leaders

    grid, (g,) = _region(fleet, 8, GRID, [5])
    register(grid, g, node=grid.map.nodes_in(GRID)[1])
    park.peer_table.upsert_leader(Leader(GRID, 8, grid.address))
    park.jobs.get("job-G").updated_at = 0.0
    park.dispatcher.tick()
    assert "job-G" not in park.jobs and grid.jobs.get("job-G").status == ASSIGNED


# ---------------------------------------------------------------- lifecycle

def _assigned(fleet):
    leader, (f,) = _region(fleet, 9, PARK, [1])
    pp = leader.map.nodes_in(PARK)
    register(leader, f, node=pp[1])
    job = Job("job-H", pp[0], pp[5])
    assert submit(leader, job).assignee == f.bot_id
    f._robot_sink.pop()
    return leader, f, job


def _robot(f, **kw):
    f._robot_source.push(RobotState(region_id=f.region_id, **kw))
    f._tick_robot_state()


def test_full_lifecycle_pickup_dropoff_delivered_crossed_off(fleet):
    leader, f, job = _assigned(fleet)
    # Robot reaches pickup and loads: reports CARGO/EN_ROUTE.
    _robot(f, latest_node_id=job.pickup_node, mission="CARGO", job_id=job.job_id, cargo_state=CS_EN_ROUTE)
    assert f.cargo_state == CS_EN_ROUTE
    cmd = f._robot_sink.pop()
    assert cmd.target_node_id == job.dropoff_node and cmd.mission["cargo"]["state"] == CS_EN_ROUTE
    register(leader, f, node=job.pickup_node, mission="CARGO", job_id=job.job_id, cargo_state=CS_EN_ROUTE)
    assert leader.jobs.get(job.job_id).status == PICKED_UP

    # Arrives, drops: CARGO/DROPOFF, then mission returns to IDLE.
    _robot(f, latest_node_id=job.dropoff_node, mission="CARGO", job_id=job.job_id, cargo_state=CS_DROPOFF)
    _robot(f, latest_node_id=job.dropoff_node, mission="IDLE")
    assert f.cargo_state == CS_DELIVERED
    payload = f.heartbeat_payload()
    assert payload.job_id == job.job_id and payload.cargo_state == CS_DELIVERED
    ack = _stub(leader, fleet_pb2_grpc.RegionServiceStub).Heartbeat(payload, timeout=2, metadata=md(1, PARK))
    assert job.job_id not in leader.jobs, "owner crosses it off"
    f.on_ack_jobs([Job.from_proto(j) for j in ack.jobs])
    assert f.current_job is None and f.is_free_for_job()


def test_pre_pickup_failure_requeues_and_reassigns(fleet):
    leader, f, job = _assigned(fleet)
    other = fleet.bot(2, region_id=PARK)
    register(leader, other, node=leader.map.nodes_in(PARK)[2])

    # The assignee faults on the way to pickup.
    _robot(f, latest_node_id=5, state="FAULTED", mission="CARGO", job_id=job.job_id, cargo_state=CS_PICKUP)
    assert f.current_job is None, "a bot that fails before pickup drops the job"
    register(leader, f, node=5, state="FAULTED", job_id=job.job_id, cargo_state=CS_PICKUP)
    j = leader.jobs.get(job.job_id)
    assert j.status == PENDING and j.assignee == NO_BOT and "pre-pickup" in j.reason

    leader.dispatcher.tick()                            # updated_at was zeroed → retry now
    assert leader.jobs.get(job.job_id).assignee == other.bot_id and other.current_job.job_id == job.job_id


def test_post_pickup_failure_needs_attention_and_alerts_control_plane(fleet):
    leader, f, job = _assigned(fleet)
    register(leader, f, node=job.pickup_node, mission="CARGO", job_id=job.job_id, cargo_state=CS_EN_ROUTE)
    register(leader, f, node=7, state="FAULTED", fault="MOTOR_ERROR", job_id=job.job_id, cargo_state=CS_EN_ROUTE)
    j = leader.jobs.get(job.job_id)
    assert j.status == NEEDS_ATTENTION and j.last_node == 7 and "post-pickup" in j.reason
    assert leader.alerts and leader.alerts[-1]["job_id"] == job.job_id and leader.alerts[-1]["last_node"] == 7
    # …and the broken bot keeps the job so its heartbeats keep saying where the cargo is
    _robot(f, latest_node_id=7, state="FAULTED", fault="MOTOR_ERROR", mission="CARGO", job_id=job.job_id, cargo_state=CS_EN_ROUTE)
    assert f.current_job is not None


def test_lost_heartbeat_with_job_is_a_failure(fleet):
    leader, f, job = _assigned(fleet)
    leader.state = "MOVING"                             # keep the leader from taking it over on the same tick
    leader.peer_table.get(f.bot_id).last_seen = time.monotonic() - 100
    leader._tick_leader_duties()
    assert leader.jobs.get(job.job_id).status == PENDING and "heartbeat lost" in leader.jobs.get(job.job_id).reason


def test_dropped_job_is_detected_from_the_next_heartbeat(fleet):
    """A follower abandons its job before pickup: its next heartbeat just has
    no job_id. The leader must treat that as a pre-pickup failure."""
    leader, f, job = _assigned(fleet)
    leader.state = "MOVING"
    register(leader, f, node=3, mission="CARGO", job_id=job.job_id, cargo_state=CS_PICKUP)
    register(leader, f, node=3, mission="IDLE")           # job gone, no DELIVERED
    j = leader.jobs.get(job.job_id)
    assert j.status == PENDING and j.assignee == NO_BOT and "dropped" in j.reason


def test_bot_zero_can_be_assigned(fleet):
    """Regression: bot_id 0 is falsy; a `if assignee:` check silently made the
    first bot up.py starts un-assignable. Found on real containers."""
    leader, (zero,) = _region(fleet, 9, PARK, [0])
    p = leader.map.nodes_in(PARK)
    register(leader, zero, node=p[1])
    ack = submit(leader, Job("job-Z", p[0], p[5]))
    assert ack.accepted and ack.HasField("assignee") and ack.assignee == 0
    assert leader.jobs.get("job-Z").assignee == 0 and zero.current_job.job_id == "job-Z"


def test_stale_failure_report_cannot_requeue_a_reassigned_job(fleet):
    leader, f, job = _assigned(fleet)
    other = fleet.bot(2, region_id=PARK)
    register(leader, other, node=leader.map.nodes_in(PARK)[2])
    register(leader, f, node=3, state="FAULTED", job_id=job.job_id, cargo_state=CS_PICKUP)  # fails → re-queued
    leader.dispatcher.tick()                                                              # → other
    assert leader.jobs.get(job.job_id).assignee == other.bot_id
    register(leader, f, node=3, state="IDLE", mission="IDLE")                             # f's late job-less heartbeat
    assert leader.jobs.get(job.job_id).assignee == other.bot_id and leader.jobs.get(job.job_id).status == ASSIGNED


def test_lost_heartbeat_of_migrating_bot_is_not_a_failure(fleet):
    leader, f, job = _assigned(fleet)
    leader.migrating_out.mark(f.bot_id, GRID)           # it left for the grid (with the cargo)
    leader.peer_table.get(f.bot_id).last_seen = time.monotonic() - 100
    leader._tick_leader_duties()
    assert leader.jobs.get(job.job_id).status == ASSIGNED


def test_delivery_observed_in_another_region_reaches_owner(fleet):
    """The assignee drove into the grid and delivered there. The grid leader
    does not own the job; it tells the leaders, and the owner crosses it off."""
    park = fleet.bot(9, region_id=PARK); park.role = Role.LEADER
    grid = fleet.bot(8, region_id=GRID); grid.role = Role.LEADER
    grid.peer_table.upsert_leader(Leader(PARK, 9, park.address))
    mover = fleet.bot(3, region_id=GRID, serve=False)
    park.jobs.upsert(Job("job-I", 1, 2, owner_region=PARK, status=PICKED_UP, assignee=3))

    register(grid, mover, node=2, mission="IDLE", job_id="job-I", cargo_state=CS_DELIVERED)
    assert wait_until(lambda: "job-I" not in park.jobs, 3)
    assert "job-I" not in grid.jobs


def test_ledger_is_replicated_to_followers(fleet):
    leader, (f,) = _region(fleet, 9, PARK, [1])
    leader.jobs.upsert(Job("job-J", 1, 2, owner_region=PARK, status=ASSIGNED, assignee=7))
    f.become_follower(9, leader.address)
    assert wait_until(lambda: "job-J" in f.jobs, 5)
    assert f.jobs.get("job-J").assignee == 7


def test_new_leader_inherits_the_ledger(fleet):
    leader, (f,) = _region(fleet, 9, PARK, [1])
    leader.jobs.upsert(Job("job-K", 1, 2, owner_region=PARK, status=PENDING))
    f.become_follower(9, leader.address)
    assert wait_until(lambda: "job-K" in f.jobs, 5)
    f.become_leader()
    assert f.jobs.get("job-K").status == PENDING, "replica survives the promotion"


def test_leader_is_last_resort_candidate(fleet):
    leader, (f,) = _region(fleet, 9, PARK, [1])
    pp = leader.map.nodes_in(PARK)
    leader.latest_node_id = pp[0]                       # leader is *at* the pickup node…
    register(leader, f, node=pp[-1])                    # …follower is far away
    ack1 = submit(leader, Job("job-L1", pp[0], pp[5]))
    assert ack1.assignee == f.bot_id, "a free follower beats the leader even when the leader is closer"
    ack2 = submit(leader, Job("job-L2", pp[0], pp[6]))  # follower now busy
    assert ack2.accepted and ack2.assignee == leader.bot_id
    assert leader.current_job.job_id == "job-L2" and leader._robot_sink.pop().target_node_id == pp[0]
    assert leader.jobs.get("job-L2").status == ASSIGNED


def test_leader_completes_its_own_job_by_self_observation(fleet):
    leader, _ = _region(fleet, 9, PARK, [])
    pp = leader.map.nodes_in(PARK)
    assert submit(leader, Job("job-L3", pp[0], pp[5])).assignee == leader.bot_id
    leader._tick_self_job()
    _robot(leader, latest_node_id=pp[0], mission="CARGO", job_id="job-L3", cargo_state=CS_EN_ROUTE)
    leader._tick_self_job()
    assert leader.jobs.get("job-L3").status == PICKED_UP
    _robot(leader, latest_node_id=pp[5], mission="CARGO", job_id="job-L3", cargo_state=CS_DROPOFF)
    _robot(leader, latest_node_id=pp[5], mission="IDLE")
    leader._tick_self_job()
    assert "job-L3" not in leader.jobs and leader.current_job is None and leader.is_free_for_job()


def test_leader_own_job_failure_before_pickup_requeues_to_a_follower(fleet):
    leader, _ = _region(fleet, 9, PARK, [])
    pp = leader.map.nodes_in(PARK)
    assert submit(leader, Job("job-L4", pp[0], pp[5])).assignee == leader.bot_id
    leader._tick_self_job()
    _robot(leader, latest_node_id=pp[1], state="FAULTED", mission="CARGO", job_id="job-L4", cargo_state=CS_PICKUP)
    leader._tick_self_job()
    j = leader.jobs.get("job-L4")
    assert j.status == PENDING and leader.current_job is None
    f = fleet.bot(1, region_id=PARK)
    register(leader, f, node=pp[2])
    leader.state = "IDLE"
    leader.dispatcher.tick()
    assert leader.jobs.get("job-L4").assignee == f.bot_id


def test_charge_bucket_beats_distance_within_free_followers(fleet):
    leader, (near_low, far_full) = _region(fleet, 9, PARK, [1, 2])
    p = leader.map.nodes_in(PARK)
    register(leader, near_low, node=p[1], battery=55.0)         # bucket 2, right next to pickup
    register(leader, far_full, node=p[-1], battery=100.0)       # bucket 4, far away
    ack = submit(leader, Job("job-P", p[0], p[5]))
    assert ack.assignee == far_full.bot_id, "job priority (charge) ranks above distance"


def test_roster_carries_yield_priority(fleet):
    leader, (free, to_pickup, carrying) = _region(fleet, 9, PARK, [1, 2, 3])
    register(leader, free, node=1)
    register(leader, to_pickup, node=1, job_id="a", cargo_state=CS_PICKUP)
    ack = register(leader, carrying, node=1, job_id="b", cargo_state=CS_EN_ROUTE)
    yp = {r.bot_id: r.yield_priority for r in ack.region_peers}
    assert yp[1] == 0 and yp[2] == 1 and yp[3] == 2


def test_job_event_is_retried_until_an_owner_acks(fleet):
    """The observing leader does not know the owner's leader yet. The event
    must be queued and re-sent, and land once the owner becomes known."""
    park = fleet.bot(9, region_id=PARK); park.role = Role.LEADER
    grid = fleet.bot(8, region_id=GRID); grid.role = Role.LEADER
    mover = fleet.bot(3, region_id=GRID, serve=False)
    park.jobs.upsert(Job("job-Q", 1, 2, owner_region=PARK, status=PICKED_UP, assignee=3))

    register(grid, mover, node=2, mission="IDLE", job_id="job-Q", cargo_state=CS_DELIVERED)
    assert grid.dispatcher.pending_event_count() == 1 and "job-Q" in park.jobs

    grid.peer_table.upsert_leader(Leader(PARK, 9, park.address))   # now it knows
    grid.dispatcher._pending_events[("job-Q", "DELIVERED")] = (
        grid.dispatcher._pending_events[("job-Q", "DELIVERED")][0], time.time(), 0.0)  # make it due
    grid.dispatcher.tick()
    assert "job-Q" not in park.jobs and grid.dispatcher.pending_event_count() == 0


def test_job_event_from_a_non_owner_is_not_acked_as_owned(fleet):
    a = fleet.bot(9, region_id=PARK); a.role = Role.LEADER
    ack = _stub(a, fleet_pb2_grpc.LeaderExchangeServiceStub).JobEvent(
        Job("nope", 1, 2, status="DELIVERED", assignee=3).to_proto(), timeout=2, metadata=md(8, GRID, "leader"))
    assert ack.owned is False


def test_leader_migration_hands_off_to_a_free_follower_not_the_busy_one(fleet):
    """§5.6 succession: bot-5 outranks bot-4 but is carrying cargo."""
    dst = fleet.bot(8, region_id=GRID); dst.role = Role.LEADER
    leader, busy, free = fleet.bot(6, region_id=PARK), fleet.bot(5, region_id=PARK), fleet.bot(4, region_id=PARK)
    leader.become_leader()
    from tests.test_protocol import _follow
    _follow(busy, leader, settled=False); _follow(free, leader, settled=False)
    assert wait_until(lambda: len(leader.peer_table) == 2, 5)
    register(leader, busy, node=1, job_id="x", cargo_state=CS_EN_ROUTE)
    leader.peer_table.upsert_leader(Leader(GRID, 8, dst.address))
    leader._leadership = leader._leadership.__class__(Role.LEADER, None, None, time.monotonic() - 100)
    leader.desired_region_id = GRID
    leader.migrator.tick()
    assert wait_until(lambda: free.role == Role.LEADER, 6), "the free bot should be handed leadership"
    assert busy.role != Role.LEADER


def test_assignjob_requires_a_leader(fleet):
    f = fleet.bot(1)
    with pytest.raises(grpc.RpcError) as e:
        _stub(f, fleet_pb2_grpc.BotServiceStub).AssignJob(Job("j", 1, 2).to_proto(), timeout=2, metadata=md(9, PARK, "follower"))
    assert e.value.code() == grpc.StatusCode.PERMISSION_DENIED


def test_duplicate_submission_is_idempotent(fleet):
    leader, f, job = _assigned(fleet)
    ack = submit(leader, Job(job.job_id, job.pickup_node, job.dropoff_node))
    assert ack.accepted and ack.assignee == f.bot_id and "duplicate" in ack.note
