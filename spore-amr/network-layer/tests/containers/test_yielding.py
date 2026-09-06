"""G. Yielding.

One of the `test_docker_*.py` files; see `docs/scenarios.md` for what each
scenario claims and `tests/containers/harness.py` for the fleet they run on.

G. Yielding (PROTOCOL.md §16.4, docs/scenarios.md G)

A yield needs a wait long enough to be worth leaving the route for. A claim
lives about two announce periods, so on the compressed clock the threshold is
dropped to match -- otherwise no wait here would ever be long enough and the
rule would never be reached.
"""
from __future__ import annotations

import pytest

from tests.containers.harness import (
    _corridor,
    _holder_of,
    _kind,
    _map_nodes,
    _neighbours,
    _query,
    PARK,
    YIELD_TIMINGS,
    wait_until,
)

pytestmark = pytest.mark.docker


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
        fleet.place(c, latest_node_id=nodes[0], region_id=PARK, battery=90.0,
                     state="IDLE", mission="IDLE")
    leader = fleet.leaders(cs)[0]
    assert wait_until(lambda: all(p.mission == "IDLE" for p in fleet.state(leader).roster), 20)
    assert fleet.submit_job(leader, "G1", nodes[6], nodes[10]).accepted
    carrier = _holder_of(fleet, cs, "G1")
    assert carrier is not None
    free = [c for c in cs if c.id != carrier.id][0]

    fleet.place(carrier, latest_node_id=theirs, region_id=PARK, battery=90.0,
                 state="IDLE", mission="CARGO", job_id="G1", cargo_state="EN_ROUTE")
    assert wait_until(lambda: fleet.state(carrier).cargo_state == "EN_ROUTE", 20)
    fleet.place(free, latest_node_id=ours, region_id=PARK, battery=90.0,
                 state="IDLE", mission="IDLE")
    carrier_id = fleet.state(carrier).bot_id
    assert wait_until(lambda: any(r.bot_id == carrier_id for r in fleet.state(free).reservations),
                      20, what="the carrier's claim to reach the free bot")

    reply = fleet.ask(free, _query(ours, _neighbours(ours)))
    assert reply is not None
    assert _kind(reply) != "PROCEED" or reply.target_node_id != theirs, \
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
    fleet.place(lower, latest_node_id=corridor[3], region_id=PARK, battery=90.0,
                 state="IDLE", mission="IDLE")
    fleet.place(higher, latest_node_id=corridor[4], region_id=PARK, battery=90.0,
                 state="IDLE", mission="IDLE")
    higher_id = fleet.state(higher).bot_id
    assert wait_until(lambda: any(r.bot_id == higher_id for r in fleet.state(lower).reservations),
                      20, what="claims to be exchanged")

    reply = fleet.ask(lower, _query(corridor[3], _neighbours(corridor[3])))
    assert reply is not None
    assert _kind(reply) != "YIELD", "the bot that wins the tiebreak should hold, not give way"


@pytest.mark.docker
def test_G7_exactly_one_side_of_a_contest_gives_way(fleet):
    """Never both, never neither. Both sides run the same rule on the same two
    numbers, so the verdicts have to agree without them talking about it."""
    cs = fleet.launch(2, PARK, **YIELD_TIMINGS)
    assert wait_until(lambda: fleet.converged(cs, PARK), 30, what="converge")
    corridor = _corridor(8)
    fleet.place(cs[0], latest_node_id=corridor[3], region_id=PARK, battery=90.0,
                 state="IDLE", mission="IDLE")
    fleet.place(cs[1], latest_node_id=corridor[4], region_id=PARK, battery=90.0,
                 state="IDLE", mission="IDLE")
    assert wait_until(lambda: len(fleet.state(cs[0]).reservations) > 1, 20,
                      what="claims to be exchanged")

    a = fleet.ask(cs[0], _query(corridor[3], _neighbours(corridor[3])))
    b = fleet.ask(cs[1], _query(corridor[4], _neighbours(corridor[4])))
    assert a is not None and b is not None
    yields = [_kind(r) == "YIELD" for r in (a, b)]
    assert not all(yields), "both gave way, so nobody moves"
    fleet.assert_no_overlap(cs)
