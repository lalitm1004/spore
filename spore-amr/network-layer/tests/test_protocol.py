"""In-process integration tests: real gRPC servers on 127.0.0.1, real sender
threads, real elections — everything except Docker.

Each test names the PROTOCOL.md section it exercises. Bots are built via the
`fleet` fixture (see conftest.py) which tears down every thread and server.
"""
from __future__ import annotations

import time

import grpc
import pytest

from bot import RobotState
from bus.migration import Phase
from election.bully import Role
from peers.table import Peer, Leader
from proto import fleet_pb2, fleet_pb2_grpc
from bus.rpc import pool

from tests.conftest import md, wait_until


def _stub(bot_or_addr, cls):
    addr = bot_or_addr if isinstance(bot_or_addr, str) else bot_or_addr.address
    return pool.stub(addr, cls)


def _follow(b, leader, *, settled=True):
    """Make `b` a follower of `leader`, optionally pretending a recent ack so
    `leader_settled()` is true without waiting for the heartbeat thread."""
    b.become_follower(leader.bot_id, leader.address)
    if settled:
        b.last_ack_at = time.monotonic()


# =============================================================================
# §4.1 / §5.4  Same-region conflict — bootstrap and split-brain resolution
# =============================================================================

def test_conflict_received_in_rpc_lower_priority_yields(fleet):
    low, high = fleet.bot(0), fleet.bot(1)
    _stub(low, fleet_pb2_grpc.LeaderExchangeServiceStub).LeaderHeartbeat(
        high.leader_hb_payload(), timeout=2, metadata=md(1, 14, "leader"))
    assert low.role == Role.FOLLOWER and low.leader_address == high.address
    assert high.role == Role.LEADER


def test_conflict_seen_on_own_sender_thread_does_not_crash(fleet):
    """become_follower() → stop() runs *on* the sender thread; joining itself
    would raise. Regression test for a real crash the RPC-path test missed."""
    low, high = fleet.bot(0), fleet.bot(1)
    fleet.link_all()
    low.become_leader()  # starts low's leader exchange → heartbeats high → sees conflict in the ack
    assert wait_until(lambda: low.role == Role.FOLLOWER, 5)
    assert low.leader_address == high.address and high.role == Role.LEADER


def test_unhealthy_leader_always_yields_even_to_lower_priority(fleet):
    """§5.1: a FAULTED bot must never lead, whatever its priority."""
    sick, healthy = fleet.bot(9, state="FAULTED"), fleet.bot(1)
    _stub(sick, fleet_pb2_grpc.LeaderExchangeServiceStub).LeaderHeartbeat(
        healthy.leader_hb_payload(), timeout=2, metadata=md(1, 14, "leader"))
    assert sick.role == Role.FOLLOWER and sick.leader_address == healthy.address


# =============================================================================
# §3.1  Heartbeat / ack / redirect / departure
# =============================================================================

def test_heartbeat_returns_roster_including_leader_and_leader_id(fleet):
    leader, f = fleet.bot(2), fleet.bot(0, serve=False)
    ack = _stub(leader, fleet_pb2_grpc.RegionServiceStub).Heartbeat(f.heartbeat_payload(), timeout=2, metadata=md(0, 14))
    assert {p.bot_id for p in ack.region_peers} == {0, 2}
    assert ack.redirect_to == "" and ack.leader_bot_id == 2


def test_non_leader_redirects_to_its_leader(fleet):
    leader, follower, probe = fleet.bot(2), fleet.bot(1), fleet.bot(5, serve=False)
    _follow(follower, leader)
    ack = _stub(follower, fleet_pb2_grpc.RegionServiceStub).Heartbeat(probe.heartbeat_payload(), timeout=2, metadata=md(5, 14))
    assert ack.redirect_to == leader.address and ack.leader_bot_id == 2


def test_departure_removes_roster_entry_and_migrating_out(fleet):
    leader, f = fleet.bot(2), fleet.bot(0, serve=False)
    stub = _stub(leader, fleet_pb2_grpc.RegionServiceStub)
    stub.Heartbeat(f.heartbeat_payload(), timeout=2, metadata=md(0, 14))
    leader.migrating_out.mark(0, 2)
    stub.Departure(fleet_pb2.DepartureRequest(bot_id=0), timeout=2, metadata=md(0, 14))
    assert leader.peer_table.get(0) is None and 0 not in leader.migrating_out


def test_follower_with_no_leader_discovers_one_by_probing(fleet):
    """§4.3 / §6 REJOINING: an unhealthy-at-boot or rejoining bot heartbeats
    PEER_LEADERS until someone tells it who leads — here via a follower's
    redirect, the harder path."""
    leader, follower, lost = fleet.bot(2), fleet.bot(1), fleet.bot(0)
    _follow(follower, leader)
    lost.peer_leaders = [follower.address]      # only knows about a follower
    lost.role = Role.FOLLOWER                   # no leader_address set
    lost._hb_sender.start()
    assert wait_until(lambda: lost.leader_address == leader.address, 5)
    assert lost.leader_bot_id == 2


# =============================================================================
# §5  Bully election
# =============================================================================

def _seed_rosters(*bots):
    for b in bots:
        for o in bots:
            if o is not b:
                b.peer_table.upsert(Peer(bot_id=o.bot_id, address=o.address, priority=o.priority))


def test_election_chain_highest_priority_wins(fleet):
    """bot-0 challenges → bot-1 outranks and elects → bot-2 outranks and
    wins → Coordinator to all. Regression: an outranking bot must *start its
    own election*, else the challenger waits forever."""
    b0, b1, b2 = fleet.bot(0), fleet.bot(1), fleet.bot(2)
    _seed_rosters(b0, b1, b2)
    b0.election.start_election(b0.peer_table.all_peers())
    assert wait_until(lambda: b2.role == Role.LEADER and b0.leader_bot_id == 2 and b1.leader_bot_id == 2, 6)


def test_faulted_bot_cannot_win_election(fleet):
    """§5.1: the highest-id bot is FAULTED; the next healthy one must win."""
    b0, b1, sick = fleet.bot(0), fleet.bot(1), fleet.bot(9, state="FAULTED")
    _seed_rosters(b0, b1, sick)
    resp = _stub(sick, fleet_pb2_grpc.ElectionServiceStub).Elect(
        fleet_pb2.ElectRequest(bot_id=0, priority=b0.priority), timeout=2, metadata=md(0, 14))
    assert resp.ack is False, "a FAULTED bot must not tell anyone to stand down"
    b0.election.start_election(b0.peer_table.all_peers())
    assert wait_until(lambda: b1.role == Role.LEADER and b0.leader_bot_id == 1, 6)
    assert sick.role != Role.LEADER


def test_departing_bot_declines_election(fleet):
    b = fleet.bot(5)
    b.election.departing = True
    resp = _stub(b, fleet_pb2_grpc.ElectionServiceStub).Elect(
        fleet_pb2.ElectRequest(bot_id=1, priority=1), timeout=2, metadata=md(1, 14))
    assert resp.ack is False


def test_leader_death_followers_elect_and_converge(fleet):
    """§4.8 end to end with live heartbeat threads: kill the leader's server,
    followers miss T_LEADER_DEAD of acks, elect, and converge on the survivor
    with the highest priority."""
    b0, b1, leader = fleet.bot(0), fleet.bot(1), fleet.bot(2)
    leader.become_leader()
    _follow(b0, leader, settled=False)
    _follow(b1, leader, settled=False)
    assert wait_until(lambda: len(b0.peer_table) == 3 and len(b1.peer_table) == 3, 5)

    fleet.stop_server(leader)
    leader._leader_exchange.stop()

    assert wait_until(lambda: b1.role == Role.LEADER, 12), "bot-1 should win after the leader dies"
    assert wait_until(lambda: b0.role == Role.FOLLOWER and b0.leader_bot_id == 1, 8)


def test_leader_ignores_stale_coordinator_from_a_bot_it_outranks(fleet):
    """Found on real containers: a paused leader comes back to a Coordinator
    that was waiting in its socket. Obeying it made two bots follow each
    other. A healthy leader that outranks the named bot must ignore it."""
    leader, lower = fleet.bot(2), fleet.bot(1)
    leader.become_leader()
    _stub(leader, fleet_pb2_grpc.ElectionServiceStub).Coordinator(
        fleet_pb2.CoordinatorRequest(bot_id=1, priority=lower.priority, address=lower.address),
        timeout=2, metadata=md(1, 14))
    assert leader.role == Role.LEADER
    # …but a Coordinator from a *higher* bot is a real result and is obeyed.
    higher = fleet.bot(5)
    _stub(leader, fleet_pb2_grpc.ElectionServiceStub).Coordinator(
        fleet_pb2.CoordinatorRequest(bot_id=5, priority=higher.priority, address=higher.address),
        timeout=2, metadata=md(5, 14))
    assert leader.role == Role.FOLLOWER and leader.leader_bot_id == 5


def test_a_bot_never_follows_itself(fleet):
    b = fleet.bot(3)
    b.become_follower(3, b.address)
    assert b.role == Role.LEADER
    b.retarget(3, b.address)
    assert b.role == Role.LEADER


def test_mutual_followers_resolve_to_one_leader_without_a_hot_loop(fleet):
    """A follows B and B follows A (what a stale Coordinator produced). Each
    redirects the other to itself; the self-redirect must promote, and the
    heartbeat pace must stay 1 Hz."""
    a, b = fleet.bot(1), fleet.bot(2)
    fleet.link_all()   # both may self-promote at once; the leader exchange then settles it
    a.become_follower(2, b.address)
    b.become_follower(1, a.address)
    assert wait_until(lambda: {a.role, b.role} == {Role.LEADER, Role.FOLLOWER}, 8)
    leader, follower = (a, b) if a.role == Role.LEADER else (b, a)
    assert wait_until(lambda: follower.leader_bot_id == leader.bot_id, 5)
    assert wait_until(lambda: leader.peer_table.get(follower.bot_id) is not None, 5)


def test_election_winner_notifies_a_challenger_it_had_never_seen(fleet):
    """Regression for a real convergence bug: bot-1's roster is empty (its last
    ack predates bot-0 joining). bot-0 challenges it; bot-1 wins and MUST
    still Coordinator bot-0 — the challenger registers itself via
    ElectRequest.address."""
    b0, b1 = fleet.bot(0), fleet.bot(1)
    b0.peer_table.upsert(Peer(bot_id=1, address=b1.address, priority=b1.priority))
    assert len(b1.peer_table) == 0
    b0.election.start_election(b0.peer_table.all_peers())
    assert wait_until(lambda: b1.role == Role.LEADER and b0.leader_bot_id == 1, 6)
    assert b1.peer_table.get(0) is not None, "challenger should now be in the winner's roster"


# =============================================================================
# §3.1 / §3.2  Location trails
# =============================================================================

def test_heartbeat_carries_trail_and_leader_relays_it(fleet):
    leader, f = fleet.bot(2), fleet.bot(0, serve=False)
    for node in (10, 10, 11, 12, 12, 13):          # standing still twice, then moving
        f._robot_source.push(RobotState(latest_node_id=node, region_id=14))
        f._tick_robot_state()
    assert list(f.node_trail) == [13, 12, 11], "newest first, capped at 3, no consecutive duplicates"
    ack = _stub(leader, fleet_pb2_grpc.RegionServiceStub).Heartbeat(f.heartbeat_payload(), timeout=2, metadata=md(0, 14))
    rec = next(p for p in ack.region_peers if p.bot_id == 0)
    assert list(rec.node_trail) == [13, 12, 11] and rec.latest_node_id == 13


def test_leader_exchange_shares_every_bots_locations(fleet):
    a, b = fleet.bot(9, region_id=14), fleet.bot(8, region_id=2)
    a.role = Role.LEADER
    b.role = Role.LEADER
    a.peer_table.upsert(Peer(bot_id=3, address="x", priority=3, node_trail=[5, 4]))
    a.node_trail.extend([7, 6])
    _stub(b, fleet_pb2_grpc.LeaderExchangeServiceStub).LeaderHeartbeat(
        a.leader_hb_payload(), timeout=2, metadata=md(9, 14, "leader"))
    locs = b.peer_table.region_locations()
    assert locs[14] == {3: [5, 4], 9: [7, 6]}


# =============================================================================
# §4.6 / §4.2 / §8  Migration as a state machine
# =============================================================================

def _two_regions(fleet, mover_id=3):
    """src leader (14) knows dst leader (2) and the mover; mover follows src."""
    src, dst, mover = fleet.bot(9, region_id=14), fleet.bot(8, region_id=2), fleet.bot(mover_id, region_id=14)
    src.role = Role.LEADER
    dst.role = Role.LEADER
    src.peer_table.upsert(Peer(bot_id=mover.bot_id, address=mover.address, priority=mover.priority))
    src.peer_table.upsert_leader(Leader(region_id=2, bot_id=8, address=dst.address))
    _follow(mover, src)
    return src, dst, mover


def test_migrating_out_survives_the_bots_own_heartbeats(fleet):
    """Regression for the 'flag overwritten within one T_HB' bug: the leader's
    roster must show MIGRATING_OUT even though the bot keeps reporting IDLE."""
    src, _, mover = _two_regions(fleet)
    src.migrating_out.mark(3, 2)
    stub = _stub(src, fleet_pb2_grpc.RegionServiceStub)
    for _ in range(3):
        ack = stub.Heartbeat(mover.heartbeat_payload(), timeout=2, metadata=md(3, 14))
    assert next(p.state for p in ack.region_peers if p.bot_id == 3) == "MIGRATING_OUT"


def test_migration_end_to_end_via_migrator(fleet):
    src, dst, mover = _two_regions(fleet)
    mover.desired_region_id = 2
    mover.migrator.tick()
    assert wait_until(lambda: mover.region_id == 2 and mover.migrator.phase == Phase.IDLE, 8)
    assert mover.leader_bot_id == 8 and mover.leader_address == dst.address
    assert dst.peer_table.get(3) is not None and 3 not in dst.pending_incoming
    assert src.peer_table.get(3) is None and 3 not in src.migrating_out


def test_migration_reports_migrating_state_while_in_flight(fleet):
    src, dst, mover = _two_regions(fleet)
    fleet.stop_server(dst)                       # join will hang until timeout
    mover.desired_region_id = 2
    mover.migrator.tick()
    assert wait_until(lambda: mover.migrator.in_flight, 2)
    assert mover.effective_state() == "MIGRATING"
    assert mover.heartbeat_payload().state == "MIGRATING"


def test_migration_to_empty_region_goes_solo(fleet):
    src, mover = fleet.bot(9), fleet.bot(3)
    src.role = Role.LEADER
    src.peer_table.upsert(Peer(bot_id=3, address=mover.address, priority=mover.priority))
    _follow(mover, src)
    mover.desired_region_id = 7                  # nobody knows a region-7 leader
    mover.migrator.tick()
    assert wait_until(lambda: mover.region_id == 7 and mover.role == Role.LEADER, 6)
    assert src.peer_table.get(3) is None


def test_migration_waits_for_settled_leader(fleet):
    """§5.7: a follower that has not heard from its leader must not migrate."""
    src, dst, mover = _two_regions(fleet)
    mover.last_ack_at = 0.0                      # never heard from the leader
    mover.desired_region_id = 2
    mover.migrator.tick()
    time.sleep(0.5)
    assert mover.migrator.phase == Phase.IDLE and mover.region_id == 14


def test_migration_retries_after_transient_destination_failure(fleet):
    """§4.6 reconcile loop: destination down → attempt fails → backoff →
    destination returns → next attempt succeeds. Nothing is re-triggered by
    the robot; the loop notices desired ≠ actual on its own."""
    src, dst, mover = _two_regions(fleet)
    fleet.stop_server(dst)
    mover.desired_region_id = 2
    mover.migrator.tick()
    assert wait_until(lambda: mover.migrator.phase == Phase.FAILED, 8), "first attempt should fail"
    assert mover.region_id == 14

    fleet.serve(dst)
    for _ in range(80):                          # keep ticking like the run loop would
        mover.migrator.tick()
        if mover.region_id == 2:
            break
        time.sleep(0.1)
    assert mover.region_id == 2 and dst.peer_table.get(3) is not None


def test_leader_migrates_by_abdicating_first(fleet):
    """§4.7: a leader that must move hands off to its best follower, then
    migrates as an ordinary bot."""
    dst = fleet.bot(8, region_id=2)
    dst.role = Role.LEADER
    leader, follower = fleet.bot(5, region_id=14), fleet.bot(4, region_id=14)
    leader.become_leader()
    _follow(follower, leader, settled=False)
    assert wait_until(lambda: follower.last_ack_at > 0, 5)
    leader.peer_table.upsert_leader(Leader(region_id=2, bot_id=8, address=dst.address))
    follower.peer_table.upsert_leader(Leader(region_id=2, bot_id=8, address=dst.address))
    leader._leadership = leader._leadership.__class__(Role.LEADER, None, None, time.monotonic() - 100)  # settled

    leader.desired_region_id = 2
    leader.migrator.tick()
    assert wait_until(lambda: follower.role == Role.LEADER, 6), "follower should be handed leadership"
    assert wait_until(lambda: leader.region_id == 2 and leader.leader_bot_id == 8, 8)


# =============================================================================
# The run loop (bot.py) — the pieces that turn robot state into decisions
# =============================================================================

def test_run_loop_qr_region_change_drives_migration(fleet):
    src, dst, mover = _two_regions(fleet)
    # A QR scan reaching the bot the way one really does: a report, through
    # the only ingress there is.
    mover.report_robot_state(RobotState(latest_node_id=77, region_id=2))
    mover._tick_robot_state()
    assert mover.latest_node_id == 77 and mover.desired_region_id == 2 and mover.region_id == 14
    mover.migrator.tick()
    assert wait_until(lambda: mover.region_id == 2, 8)


def test_run_loop_faulted_leader_abdicates(fleet):
    leader, follower = fleet.bot(5), fleet.bot(4)
    leader.become_leader()
    _follow(follower, leader, settled=False)
    assert wait_until(lambda: leader.peer_table.get(4) is not None, 5)

    leader._robot_source.push(RobotState(state="FAULTED", region_id=14))
    leader._tick_robot_state()
    leader._tick_priority()
    leader._tick_health()
    assert wait_until(lambda: follower.role == Role.LEADER, 5)
    assert leader.role == Role.FOLLOWER and leader.leader_bot_id == 4


def test_tenure_rotates_leadership_to_a_free_follower(fleet, monkeypatch):
    """§5.6 Tenure: after T_LEADER_TENURE a leader hands off to the best free
    follower even though it still outranks everyone on election priority."""
    import config
    monkeypatch.setattr(config, "T_LEADER_TENURE", 0.5)
    leader, busy, free = fleet.bot(9), fleet.bot(5), fleet.bot(4)
    leader.become_leader()
    _follow(busy, leader, settled=False)
    _follow(free, leader, settled=False)
    assert wait_until(lambda: len(leader.peer_table) == 2, 5)
    leader.peer_table.get(5).job_id = "busy"          # bot-5 outranks bot-4 but is busy
    leader._tick_tenure()
    assert leader.role == Role.LEADER, "tenure not served yet"
    time.sleep(0.6)
    leader._tick_tenure()
    assert wait_until(lambda: free.role == Role.LEADER and leader.leader_bot_id == 4, 5)


def test_tenure_does_not_rotate_while_leader_has_a_job(fleet, monkeypatch):
    import config
    from bus.jobs import Job
    monkeypatch.setattr(config, "T_LEADER_TENURE", 0.1)
    leader, free = fleet.bot(9), fleet.bot(4)
    leader.become_leader()
    _follow(free, leader, settled=False)
    assert wait_until(lambda: len(leader.peer_table) == 1, 5)
    leader.current_job = Job("j", 1, 2)
    time.sleep(0.2)
    leader._tick_tenure()
    assert leader.role == Role.LEADER


def test_run_loop_priority_tracks_battery_and_health(fleet):
    b = fleet.bot(3, serve=False)
    b._robot_source.push(RobotState(battery=100, region_id=14)); b._tick_robot_state(); b._tick_priority()
    full = b.priority
    b._robot_source.push(RobotState(battery=30, region_id=14)); b._tick_robot_state(); b._tick_priority()
    assert b.priority < full
    b._robot_source.push(RobotState(battery=100, state="FAULTED", region_id=14)); b._tick_robot_state(); b._tick_priority()
    assert b.priority < 10_000 and not b.is_healthy()


# =============================================================================
# §12  Virtual network
# =============================================================================

def test_virtual_network_missing_metadata_is_unauthenticated(fleet):
    leader = fleet.bot(2)
    with pytest.raises(grpc.RpcError) as e:
        _stub(leader, fleet_pb2_grpc.RegionServiceStub).Heartbeat(fleet_pb2.HeartbeatRequest(bot_id=0, region_id=14), timeout=2)
    assert e.value.code() == grpc.StatusCode.UNAUTHENTICATED


def test_virtual_network_cross_region_heartbeat_denied(fleet):
    leader = fleet.bot(2, region_id=14)
    with pytest.raises(grpc.RpcError) as e:
        _stub(leader, fleet_pb2_grpc.RegionServiceStub).Heartbeat(
            fleet_pb2.HeartbeatRequest(bot_id=9, region_id=2), timeout=2, metadata=md(9, 2))
    assert e.value.code() == grpc.StatusCode.PERMISSION_DENIED
    assert leader.peer_table.get(9) is None


def test_virtual_network_join_without_handoff_denied(fleet):
    dst = fleet.bot(8, region_id=2)
    with pytest.raises(grpc.RpcError) as e:
        _stub(dst, fleet_pb2_grpc.MigrationJoinServiceStub).MigrationJoin(
            fleet_pb2.MigrationJoinReq(bot_id=99, source_region_id=14), timeout=2, metadata=md(99, 14))
    assert e.value.code() == grpc.StatusCode.PERMISSION_DENIED


def test_virtual_network_follower_denied_on_leader_exchange(fleet):
    leader = fleet.bot(2)
    with pytest.raises(grpc.RpcError) as e:
        _stub(leader, fleet_pb2_grpc.LeaderExchangeServiceStub).LeaderHeartbeat(
            fleet_pb2.LeaderHBRequest(region_id=2, leader_bot_id=7), timeout=2, metadata=md(7, 2, "follower"))
    assert e.value.code() == grpc.StatusCode.PERMISSION_DENIED
    assert leader.peer_table.get_leader(2) is None
