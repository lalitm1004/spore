"""D. Exceptions.

One of the `test_docker_*.py` files; see `docs/scenarios.md` for what each
scenario claims and `tests/containers/harness.py` for the fleet they run on.

D. Exceptions (docs/scenarios.md D)

Escalation is logged rather than exposed in BotState, because it is an
operational event rather than fleet state -- so these read the container's own
log for the rungs, and BotState for the consequences.
"""
from __future__ import annotations

import time
import pytest

from tests.containers.harness import (
    _claims_of,
    _corridor,
    _free_fleet,
    _holder_of,
    _logs,
    _map_nodes,
    _nearby_pair,
    _neighbours,
    _park,
    _query,
    FAST_TIMINGS,
    PARK,
    wait_until,
)

pytestmark = pytest.mark.docker


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
    # Election under eight parallel fleets takes longer than one alone; this
    # passes 3/3 in isolation, so the budget is what flaked, not the claim.

    # Move the survivor somewhere new; the claim its neighbour holds must follow.
    _, elsewhere = _nearby_pair(PARK, max_hops=2)
    _park(fleet, survivors[0], elsewhere)
    assert wait_until(
        lambda: any(c.node_id == elsewhere for c in watcher_sees()), 60,
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
        fleet.place(holder, latest_node_id=stuck_at, region_id=PARK, battery=90.0,
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

    fleet.place(holder, latest_node_id=nodes[1], region_id=PARK, battery=90.0,
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

    fleet.place(holder, latest_node_id=nodes[6], region_id=PARK, battery=90.0,
                 state="IDLE", mission="CARGO", job_id="D5", cargo_state="EN_ROUTE")
    assert wait_until(lambda: fleet.state(holder).cargo_state == "EN_ROUTE", 20)

    fleet.place(holder, latest_node_id=nodes[6], region_id=PARK, battery=90.0,
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
    fleet.place(cs[0], latest_node_id=node, region_id=PARK, battery=90.0,
                 state="IDLE", mission="IDLE")

    first = fleet.ask(cs[0], _query(node, _neighbours(node), query_id=1))
    assert first is not None
    # `ask` opens and closes a connection each time, so the second call is a
    # reconnection by definition.
    second = fleet.ask(cs[0], _query(node, _neighbours(node), query_id=2))
    assert second is not None, "the listener did not survive the first companion leaving"
    assert second.query_id == 2


@pytest.mark.docker
def test_D7_a_killed_bot_stops_blocking_the_lane_it_held(fleet):
    """Claims lapse. A bot that dies holding a corridor must not hold it for
    ever, or one crash closes a lane for the rest of the shift."""
    cs = fleet.launch(3, PARK, **FAST_TIMINGS)
    assert wait_until(lambda: fleet.converged(cs, PARK), 30, what="converge")
    corridor = _corridor(6)
    for c, node in zip(cs, corridor[:3], strict=False):
        fleet.place(c, latest_node_id=node, region_id=PARK, battery=90.0,
                     state="IDLE", mission="IDLE")

    victim, watcher = cs[0], cs[1]
    victim_id = fleet.state(victim).bot_id
    assert wait_until(lambda: any(r.bot_id == victim_id for r in fleet.state(watcher).reservations),
                      20, what="the victim's claim to arrive")

    victim.kill()
    assert wait_until(
        lambda: not any(r.bot_id == victim_id for r in fleet.state(watcher).reservations),
        30, what="the dead bot's claims to lapse")
