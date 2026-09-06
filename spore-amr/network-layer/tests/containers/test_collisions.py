"""E. Collisions.

One of the `test_docker_*.py` files; see `docs/scenarios.md` for what each
scenario claims and `tests/containers/harness.py` for the fleet they run on.

E. Collisions (docs/scenarios.md E)
"""
from __future__ import annotations

import time
import pytest

from tests.containers.harness import (
    _claims_of,
    _corridor,
    _hop_seconds,
    _kind,
    _map_nodes,
    _nearby_pair,
    _neighbours,
    _park,
    _planned,
    _query,
    _routing_fleet,
    PARK,
    wait_until,
)

pytestmark = pytest.mark.docker


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


# -----------------------------------------------------------------------------
@pytest.mark.docker
def test_E1_two_bots_on_one_node_settle_on_a_single_holder(two_bots):
    """Both want it, both apply the same ordering, and only one keeps it."""
    fleet, cs = two_bots
    fleet.reset(cs)
    node = _map_nodes(PARK, 1)[0]
    for c in cs:
        fleet.place(c, latest_node_id=node, region_id=PARK, battery=90.0,
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
        fleet.place(c, latest_node_id=node, region_id=PARK, battery=90.0,
                     state="IDLE", mission="IDLE")

    def one_holder() -> bool:
        holders = set()
        for c in cs:
            holders |= {r.bot_id for r in fleet.state(c).reservations if r.node_id == node}
        return len(holders) == 1

    assert wait_until(one_holder, 30, what="two of the three to give way")
    fleet.assert_no_overlap(cs)


@pytest.mark.docker
def test_E4_a_following_bot_does_not_close_up_on_the_one_ahead(fleet):
    """The overlapping-claim rule keeps a node held until the robot is fully
    inside the next one, so a follower cannot arrive early."""
    corridor = _corridor(6)
    leader_node, follower_node = corridor[2], corridor[1]
    # The follower is the one doing the routing, so the job has to land on it.
    cs, follower_c = _routing_fleet(fleet, "E4", follower_node)
    ahead_c = next(c for c in cs if c is not follower_c)
    fleet.place(ahead_c, latest_node_id=leader_node, region_id=PARK, battery=90.0,
                 state="IDLE", mission="IDLE")
    ahead_id = fleet.state(ahead_c).bot_id
    assert wait_until(lambda: any(r.bot_id == ahead_id for r in fleet.state(follower_c).reservations),
                      20, what="the leader's claim to reach the follower")

    reply = _planned(fleet.ask(follower_c, _query(follower_node, _neighbours(follower_node))))
    if _kind(reply) in ("PROCEED", "REROUTE"):
        assert reply.target_node_id != leader_node, "it drove into an occupied node"
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
        fleet.place(c, latest_node_id=node, region_id=PARK, battery=90.0,
                     state="IDLE", mission="IDLE")
    assert wait_until(lambda: len(fleet.state(cs[0]).reservations) >= 2, 20,
                      what="claims to be exchanged")

    # Shuffle them along the corridor and re-check after every move: a single
    # snapshot could miss an overlap that existed only between two of them.
    #
    # One step per hop time, and that is not a tuned number. A robot holds the
    # node it is on until it is fully inside the next, so a claim covers a whole
    # traversal -- move a bot sooner than that and the previous occupant is
    # still legitimately holding the node it just left. The overlap is then
    # real, and caused entirely by the harness moving robots faster than robots
    # move. Nothing about the fleet is being tested at 0.4 s.
    for step in range(3):
        for c, node in zip(cs, corridor[step:step + 3], strict=False):
            fleet.place(c, latest_node_id=node, region_id=PARK, battery=90.0,
                         state="MOVING", mission="IDLE")
        time.sleep(_hop_seconds())
        fleet.assert_no_overlap(cs)
