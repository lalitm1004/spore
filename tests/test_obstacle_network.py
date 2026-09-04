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
        assert guard.update(0.9, step * 0.01, None) is Obstacle.CLEAR
    assert not guard.blocked
    assert guard.trips == 0


def test_reflex_retreats_all_the_way_to_the_last_node():
    """A node is a position the router can act on. Stopping 80 mm back leaves
    the robot mid-lane with nothing useful to say about where it is."""
    guard = ObstacleGuard(ObstacleConfig(stop_m=0.18, arrive_m=0.04))

    # Tripped 400 mm of path past the node.
    assert guard.update(0.15, 0.0, 0.40) is Obstacle.BACKING
    assert guard.speeds() < 0, "backing off must drive the wheels in reverse"

    # Clearance alone is not enough to stop any more.
    assert guard.update(0.50, 0.20, 0.20) is Obstacle.BACKING
    assert guard.update(0.60, 0.34, 0.06) is Obstacle.BACKING
    assert guard.update(0.60, 0.38, 0.02) is Obstacle.HOLDING
    assert guard.speeds() == 0.0


def test_without_a_node_it_settles_for_clearance():
    """Before the first marker there is nowhere to retreat to."""
    guard = ObstacleGuard(ObstacleConfig(stop_m=0.18, clear_m=0.30))
    assert guard.update(0.15, 0.0, None) is Obstacle.BACKING
    assert guard.update(0.20, 0.02, None) is Obstacle.BACKING
    assert guard.update(0.31, 0.05, None) is Obstacle.HOLDING


def test_backoff_gives_up_rather_than_reversing_blind():
    """There is no rear sensor, so reversing forever is not safe. If clearance
    is not improving, stop and hold."""
    guard = ObstacleGuard(ObstacleConfig(stop_m=0.18, clear_m=0.30, max_backoff_m=0.15))
    guard.update(0.10, 0.0, 9.0)                       # a node it can never reach
    assert guard.update(0.10, 0.10, 8.9) is Obstacle.BACKING
    assert guard.update(0.10, 0.16, 8.8) is Obstacle.HOLDING


def test_forward_overshoot_does_not_count_as_backing_off():
    """The robot coasts forward for a few steps after the wheels reverse.

    An earlier version measured odometry path length, which is monotonic, so
    that overshoot counted as progress and the reflex finished having driven
    partly into the obstacle. Displacement from the trip point cannot do that.
    """
    guard = ObstacleGuard(ObstacleConfig(stop_m=0.18, clear_m=0.30, max_backoff_m=0.15))
    guard.update(0.15, 0.0, None)

    # Coasting forward: closer, and no clearance gained.
    assert guard.update(0.14, 0.02, None) is Obstacle.BACKING
    assert guard.update(0.13, 0.03, None) is Obstacle.BACKING
    # Now actually reversing, but still short of clear_m.
    assert guard.update(0.22, 0.05, None) is Obstacle.BACKING


def test_backing_stops_at_clear_m_when_there_is_no_node():
    """`clear_m` is the "far enough to stop reversing" threshold, and it has to
    exceed `stop_m` or the robot would stop reversing while still close enough
    to trip again immediately."""
    guard = ObstacleGuard(ObstacleConfig(stop_m=0.18, clear_m=0.30))
    guard.update(0.15, 0.0, None)
    assert guard.update(0.25, 0.05, None) is Obstacle.BACKING  # past stop_m
    assert guard.update(0.31, 0.08, None) is Obstacle.HOLDING  # past clear_m


def test_parked_robot_does_not_resume_on_its_own_retreat():
    """Reversing is what produced the clearance, so clearance alone must not
    mean "all clear" -- or the robot drives forward, trips on the same
    obstacle, reverses, and repeats for ever. Observed in sim as a BACKING /
    CLEAR cycle every 4 seconds.
    """
    guard = ObstacleGuard(ObstacleConfig(stop_m=0.18, clear_m=0.30,
                                         departed_m=0.15, arrive_m=0.04))
    guard.update(0.15, 0.0, 0.30)                    # trips
    guard.update(0.30, 0.28, 0.02)                   # arrives at the node
    assert guard.state is Obstacle.HOLDING

    # Range is now well past clear_m, but only because the robot moved.
    for _ in range(20):
        assert guard.update(0.32, 0.30, 0.0) is Obstacle.HOLDING

    # The obstacle itself moving is a different matter.
    assert guard.update(0.60, 0.30, 0.0) is Obstacle.CLEAR


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
