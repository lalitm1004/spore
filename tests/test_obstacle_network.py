"""The obstacle reflex and the stand-in router.

Both are pure, so both are testable without Webots -- which matters most for
the reflex, since the failure it guards against is one you cannot safely
provoke on hardware.
"""

import math

import pytest

from robot.network import Junction, RandomRouter
from robot.obstacle import Obstacle, ObstacleConfig, ObstacleGuard, nearest


# ------------------------------------------------------------- scan parsing --

def test_nearest_ignores_max_range_returns():
    """A lidar reports max range for "nothing there"; that is not a hit."""
    assert nearest([1.0, 1.0, 1.0], max_range=1.0) == float("inf")
    assert nearest([1.0, 0.4, 1.0], max_range=1.0) == pytest.approx(0.4)


def test_nearest_ignores_infinities_and_nans():
    assert nearest([float("inf"), float("nan"), 0.5], max_range=1.0) == pytest.approx(0.5)


def test_nearest_of_an_empty_scan_is_clear():
    assert nearest([], max_range=1.0) == float("inf")


# ------------------------------------------------------------- the reflex ----

def test_clear_path_never_fires():
    guard = ObstacleGuard()
    for step in range(50):
        assert guard.update(0.9, step * 0.01, 0.0) is Obstacle.CLEAR
    assert not guard.blocked
    assert guard.trips == 0


def test_obstacle_reverses_until_it_has_clearance():
    guard = ObstacleGuard(ObstacleConfig(stop_m=0.18, clear_m=0.30))

    assert guard.update(0.15, 1.00, 0.0) is Obstacle.BACKING
    assert guard.speeds() < 0, "backing off must drive the wheels in reverse"

    assert guard.update(0.20, 0.98, 0.0) is Obstacle.BACKING   # better, not clear
    assert guard.update(0.31, 0.95, 0.0) is Obstacle.HOLDING   # clear
    assert guard.speeds() == 0.0
    assert guard.blocked


def test_backoff_gives_up_rather_than_reversing_blind():
    """There is no rear sensor, so reversing forever is not safe. If clearance
    is not improving, stop and hold."""
    guard = ObstacleGuard(ObstacleConfig(stop_m=0.18, clear_m=0.30, max_backoff_m=0.15))
    guard.update(0.10, 0.0, 0.0)
    assert guard.update(0.10, -0.10, 0.0) is Obstacle.BACKING
    assert guard.update(0.10, -0.16, 0.0) is Obstacle.HOLDING


def test_forward_overshoot_does_not_count_as_backing_off():
    """The robot coasts forward for a few steps after the wheels reverse.

    An earlier version measured odometry path length, which is monotonic, so
    that overshoot counted as progress and the reflex finished having driven
    partly into the obstacle. Displacement from the trip point cannot do that.
    """
    guard = ObstacleGuard(ObstacleConfig(stop_m=0.18, clear_m=0.30, max_backoff_m=0.15))
    guard.update(0.15, 0.0, 0.0)

    # Coasting forward: closer, and no clearance gained.
    assert guard.update(0.14, 0.02, 0.0) is Obstacle.BACKING
    assert guard.update(0.13, 0.03, 0.0) is Obstacle.BACKING
    # Now actually reversing, but still short of clear_m.
    assert guard.update(0.22, -0.05, 0.0) is Obstacle.BACKING


def test_holding_needs_more_clearance_than_stopping_did():
    """Hysteresis: resuming at the trip distance would chatter, and on real
    hardware that is how a gearbox dies."""
    guard = ObstacleGuard(ObstacleConfig(stop_m=0.18, clear_m=0.30, max_backoff_m=0.05))
    guard.update(0.15, 0.0, 0.0)
    guard.update(0.15, -0.06, 0.0)
    assert guard.state is Obstacle.HOLDING

    assert guard.update(0.25, -0.06, 0.0) is Obstacle.HOLDING  # past stop_m, not clear_m
    assert guard.update(0.35, -0.06, 0.0) is Obstacle.CLEAR


def test_thresholds_that_would_chatter_are_rejected():
    with pytest.raises(ValueError, match="chatter"):
        ObstacleConfig(stop_m=0.30, clear_m=0.20)


# -------------------------------------------------------------- the router ---

def junction(out_edges, heading_deg=0.0, query_id=1):
    return Junction(query_id=query_id, node=30, kind="PT", x_mm=1000, y_mm=1000,
                    out_edges=tuple(out_edges), heading_rad=math.radians(heading_deg))


def test_router_only_ever_returns_a_legal_edge():
    router = RandomRouter(seed=7)
    edges = [(0, 40), (90, 50), (270, 60)]
    for _ in range(200):
        route = router.route(junction(edges))
        assert (route.bearing_deg, route.to_node) in edges


def test_router_echoes_the_query_id():
    """Without it, a late answer to the previous junction is indistinguishable
    from the answer to this one."""
    route = RandomRouter(seed=1).route(junction([(0, 40)], query_id=99))
    assert route.query_id == 99


def test_router_is_reproducible_from_its_seed():
    edges = [(0, 40), (90, 50), (180, 60), (270, 70)]
    a = [RandomRouter(seed=3).route(junction(edges, query_id=i)).to_node for i in range(20)]
    b = [RandomRouter(seed=3).route(junction(edges, query_id=i)).to_node for i in range(20)]
    assert a == b


def test_router_prefers_not_to_double_back():
    """Arriving heading 0, the way back is bearing 180. With somewhere else to
    go, take it -- on a one-way lane, reversing would be a wrong-way entry."""
    router = RandomRouter(seed=0)
    edges = [(0, 40), (180, 20)]
    for _ in range(50):
        assert router.route(junction(edges, heading_deg=0)).to_node == 40


def test_router_will_double_back_at_a_dead_end():
    """Preference, not prohibition: if reversing is the only way out, take it."""
    route = RandomRouter(seed=0).route(junction([(180, 20)], heading_deg=0))
    assert route.to_node == 20


def test_router_declines_a_node_with_no_way_out():
    assert RandomRouter(seed=0).route(junction([])) is None
