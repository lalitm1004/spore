"""Membership, election, migration and reservations on real containers.

One of the `test_docker_*.py` files; see `docs/scenarios.md` for what each
scenario claims and `tests/containers/harness.py` for the fleet they run on.
"""
from __future__ import annotations

import pytest

from tests.containers.harness import (
    _map_nodes,
    _nearby_pair,
    _park,
    GRID,
    PARK,
    wait_until,
)

pytestmark = pytest.mark.docker


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
    fleet.place(mover, latest_node_id=grid_node, region_id=GRID, battery=90.0, state="IDLE", mission="IDLE")

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
    fleet.place(assignee, latest_node_id=pickup, region_id=PARK, battery=90.0, state="MOVING",
                 mission="CARGO", job_id="job-docker-1", cargo_state="EN_ROUTE")
    assert wait_until(lambda: any(j.job_id == "job-docker-1" and j.status == "PICKED_UP" for j in fleet.state(leader).jobs), 15)
    fleet.place(assignee, latest_node_id=dropoff, region_id=PARK, battery=88.0, state="MOVING",
                 mission="CARGO", job_id="job-docker-1", cargo_state="DROPOFF")
    # Wait for it to land before saying the cargo is down. A robot reports its
    # state for many ticks; two reports a millisecond apart is not something one
    # does, and the newest simply replaces the unread older one.
    assert wait_until(lambda: fleet.state(assignee).cargo_state == "DROPOFF", 15,
                      what="the dropoff to be reported")
    fleet.place(assignee, latest_node_id=dropoff, region_id=PARK, battery=88.0, state="IDLE", mission="IDLE")
    assert wait_until(lambda: not any(j.job_id == "job-docker-1" for j in fleet.state(leader).jobs), 15,
                      what="crossed off")
    assert wait_until(lambda: fleet.state(assignee).current_job_id == "", 10, what="assignee free again")


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
