"""Pure unit tests — no sockets, no threads.

Covers the pieces that encode a *rule*: the priority formula and its
hysteresis, the migration ledgers' TTL, the roster's atomic swap and
eviction, and the virtual-network admission table.
"""
from __future__ import annotations

import time
from types import SimpleNamespace

import pytest

from election import priority as prio
from election.bully import _outranks
from peers.table import PeerTable, Peer, Leader, Ledger
from bus.policy import VirtualNetworkInterceptor


# ---- election/priority.py ------------------------------------------------------

def test_priority_health_dominates_everything():
    faulted_full = prio.compute(healthy=False, battery_pct=100, bot_id=99)
    healthy_empty = prio.compute(healthy=True, battery_pct=0, bot_id=0)
    assert healthy_empty > faulted_full


def test_priority_battery_bucket_then_id():
    a = prio.compute(healthy=True, battery_pct=85, bot_id=1)   # bucket 4
    b = prio.compute(healthy=True, battery_pct=65, bot_id=9)   # bucket 3
    assert a > b, "a higher bucket beats a higher id"
    c = prio.compute(healthy=True, battery_pct=70, bot_id=2)   # bucket 3, id 2
    assert b > c, "same bucket → higher id wins (deterministic tiebreak)"


def test_priority_buckets_are_20_points_and_clamped():
    assert prio.battery_bucket(0) == 0
    assert prio.battery_bucket(19.9) == 0
    assert prio.battery_bucket(20) == 1
    assert prio.battery_bucket(100) == prio.MAX_BUCKET
    assert prio.battery_bucket(250) == prio.MAX_BUCKET


def test_sitting_leader_hysteresis_prevents_flapping():
    """Leader at 59% still advertises bucket 3; a challenger at 59% is bucket 2.
    Leadership therefore does not change hands at the boundary."""
    leader = prio.compute(healthy=True, battery_pct=59, bot_id=1, sitting_leader=True)
    challenger = prio.compute(healthy=True, battery_pct=59, bot_id=2)
    assert leader > challenger
    # …but once it really drains below the slack it does hand over.
    leader_low = prio.compute(healthy=True, battery_pct=54, bot_id=1, sitting_leader=True)
    assert challenger > leader_low


def test_job_priority_charge_first_leader_last_busy_never():
    full = prio.job_priority(healthy=True, battery_pct=100, is_leader=False, has_job=False)
    half = prio.job_priority(healthy=True, battery_pct=55, is_leader=False, has_job=False)
    leader_full = prio.job_priority(healthy=True, battery_pct=100, is_leader=True, has_job=False)
    assert full > half > leader_full, "a half-charged follower still beats a full leader"
    assert prio.job_priority(healthy=True, battery_pct=100, is_leader=False, has_job=True) < 0
    assert prio.job_priority(healthy=False, battery_pct=100, is_leader=False, has_job=False) < 0


def test_yield_priority_free_yields_to_busy_yields_to_carrying():
    free = prio.yield_priority(has_job=False, carrying=False)
    to_pickup = prio.yield_priority(has_job=True, carrying=False)
    carrying = prio.yield_priority(has_job=True, carrying=True)
    assert free < to_pickup < carrying


def test_best_successor_prefers_free_then_priority_never_unhealthy():
    t = PeerTable()
    t.upsert(Peer(bot_id=1, address="a", priority=10, job_id="busy"))
    t.upsert(Peer(bot_id=2, address="b", priority=5))
    t.upsert(Peer(bot_id=3, address="c", priority=99, state="FAULTED"))
    assert t.best_successor().bot_id == 2, "free beats busy even with lower priority; faulted never"
    t.upsert(Peer(bot_id=4, address="d", priority=7))
    assert t.best_successor().bot_id == 4, "among free bots, election priority decides"
    t2 = PeerTable()
    t2.upsert(Peer(bot_id=1, address="a", priority=10, job_id="busy"))
    assert t2.best_successor().bot_id == 1, "falls back to a busy bot when nobody is free"


def test_outranks_is_tuple_ordering():
    assert _outranks(10, 1, 9, 99)
    assert _outranks(10, 2, 10, 1)
    assert not _outranks(10, 1, 10, 1)


# ---- peers/table.py -----------------------------------------------------------

def test_ledger_mark_get_pop_contains():
    led = Ledger()
    led.mark(3, payload=2)
    assert 3 in led and led.get(3) == 2
    assert led.pop(3) == 2
    assert 3 not in led and led.pop(3) is None


def test_ledger_expire_by_ttl():
    led = Ledger()
    led.mark(1)
    led._entries[1] = (None, time.monotonic() - 100)  # age it
    led.mark(2)
    assert led.expire(ttl=10) == [1]
    assert 2 in led


def test_peer_table_replace_is_wholesale():
    t = PeerTable()
    t.upsert(Peer(bot_id=1, address="a", priority=1))
    t.replace([Peer(bot_id=2, address="b", priority=2)], [Leader(region_id=3, bot_id=9, address="c")])
    assert [p.bot_id for p in t.all_peers()] == [2]
    assert t.get_leader(3).bot_id == 9


def test_peer_table_evicts_by_last_seen():
    t = PeerTable()
    t.upsert(Peer(bot_id=7, address="x", priority=1))
    t._peers[7].last_seen = time.monotonic() - 100
    assert [p.bot_id for p in t.evict_dead(ttl=1.0)] == [7]
    assert t.get(7) is None


def test_highest_priority_peer_uses_bully_ordering_and_exclude():
    t = PeerTable()
    t.upsert(Peer(bot_id=1, address="a", priority=5))
    t.upsert(Peer(bot_id=2, address="b", priority=5))
    t.upsert(Peer(bot_id=3, address="c", priority=1))
    assert t.highest_priority_peer().bot_id == 2
    assert t.highest_priority_peer(exclude=2).bot_id == 1


# ---- bus/policy.py -------------------------------------------------------------

@pytest.fixture
def policy():
    bot = SimpleNamespace(bot_id=0, region_id=14, pending_incoming=Ledger())
    bot.pending_incoming.mark(5)
    return VirtualNetworkInterceptor(bot)


@pytest.mark.parametrize("service,caller,region,role,ok", [
    ("fleet.RegionService",        1, 14, "follower", True),
    ("fleet.RegionService",        1,  2, "follower", False),
    ("fleet.ElectionService",      1, 14, "follower", True),
    ("fleet.ElectionService",      1,  2, "leader",   False),
    ("fleet.LeaderExchangeService", 9, 2, "leader",   True),
    ("fleet.LeaderExchangeService", 9, 14, "follower", False),
    ("fleet.MigrationJoinService", 5, 2, "follower",  True),   # has a handoff
    ("fleet.MigrationJoinService", 6, 2, "follower",  False),  # no handoff
    ("fleet.JobService",           1, 2, "follower",  True),   # anyone may hand in a job
    ("fleet.BotService",           1, 14, "leader",   True),
    ("fleet.BotService",           1, 14, "follower", False),  # only leaders assign
    ("fleet.Unknown",              1, 14, "leader",   False),
])
def test_virtual_network_admission_table(policy, service, caller, region, role, ok):
    assert policy._allowed(service, caller, region, role) is ok


# ---- Configuration coherence -------------------------------------------------
# Each of these pairs fails silently when wrong: the fleet boots, runs, and goes
# wrong in a way that looks like a different problem entirely.

def test_the_shipped_configuration_is_coherent():
    import config

    config.validate()  # must not raise


@pytest.mark.parametrize(("setting", "value", "symptom"), [
    ("T_MAX_HOLD", 99.0, "cannot tell us apart from a network layer that has died"),
    ("RESERVATION_TTL", 0.1, "nothing would ever hold a node"),
    ("BATTERY_CRITICAL", 99.0, "still being handed new work"),
    ("T_DEAD", 0.1, "never hold a roster"),
    ("T_STALL", 0.1, "escalated as stuck"),
])
def test_an_incoherent_setting_stops_the_bot_booting(monkeypatch, setting, value, symptom):
    import config

    monkeypatch.setattr(config, setting, value)
    with pytest.raises(config.ConfigError, match=symptom):
        config.validate()


def test_the_shipped_claim_window_outlasts_a_traversal(real_map):
    """The check that needs the map, at the spacing the fleet actually runs."""
    import config

    config.validate(real_map.node_spacing)  # must not raise


def test_a_claim_window_shorter_than_a_hop_stops_the_bot_booting(monkeypatch, real_map):
    """The narrowest miss in the whole config, and the most expensive.

    At the shipped defaults `2 * T_ANNOUNCE` and one hop are both exactly
    2000 ms, so a stationary robot's claim expired on the same millisecond a
    neighbour arrived -- and because overlap is strict, that read as *free*.
    Nothing about a 200 cm spacing and a 1 s heartbeat makes that coincidence a
    design; move either constant a little and a robot standing still becomes
    invisible to the robot driving at it.
    """
    import config

    monkeypatch.setattr(config, "T_ANNOUNCE", 0.01)
    monkeypatch.setattr(config, "PLAN_SAFETY", -1.0)
    with pytest.raises(config.ConfigError, match="they would meet there"):
        config.validate(real_map.node_spacing)
