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

def run(guard, steps):
    """Feed (range, sees_border) pairs at 16 ms, returning the state each step."""
    seen = []
    for index, (range_m, border) in enumerate(steps):
        seen.append(guard.update(range_m, index * 0.016, border,
                                 travelled=index * 0.001, cruise_speed=6.0))
    return seen


def test_clear_path_never_fires():
    guard = ObstacleGuard()
    assert set(run(guard, [(0.9, False)] * 50)) == {Obstacle.CLEAR}
    assert not guard.blocked
    assert guard.trips == 0


def test_it_slows_and_settles_before_reversing():
    """Cruise straight into reverse pitches the chassis and throws the camera
    boom around; on hardware it is how a gearbox dies."""
    guard = ObstacleGuard(ObstacleConfig(decel_s=0.1, pause_s=0.1, accel_s=0.1))

    assert guard.update(0.15, 0.00, False, 0.0, cruise_speed=6.0) is Obstacle.STOPPING
    assert guard.update(0.15, 0.05, False, 0.0) is Obstacle.STOPPING
    assert guard.update(0.15, 0.11, False, 0.0) is Obstacle.PAUSED
    assert guard.speeds(0.11) == 0.0, "must be stationary while settling"
    assert guard.update(0.15, 0.22, False, 0.0) is Obstacle.BACKING


def test_deceleration_ramps_down_from_cruise():
    guard = ObstacleGuard(ObstacleConfig(decel_s=1.0))
    guard.update(0.15, 0.0, False, 0.0, cruise_speed=6.0)

    assert guard.speeds(0.0) == pytest.approx(6.0)
    assert guard.speeds(0.5) == pytest.approx(3.0)
    assert guard.speeds(1.0) == pytest.approx(0.0)


def test_reverse_ramps_up_rather_than_stepping():
    guard = ObstacleGuard(ObstacleConfig(decel_s=0.0, pause_s=0.0,
                                         accel_s=1.0, backoff_speed=2.0))
    guard.update(0.15, 0.0, False, 0.0, cruise_speed=6.0)
    guard.update(0.15, 0.1, False, 0.0)
    guard.update(0.15, 0.2, False, 0.0)
    assert guard.state is Obstacle.BACKING

    assert guard.speeds(0.2) == pytest.approx(0.0)
    assert guard.speeds(0.7) == pytest.approx(-1.0)
    assert guard.speeds(1.2) == pytest.approx(-2.0)


def test_it_stops_on_the_second_orange_band():
    """Reversing over a tile the sensor crosses the far band, the code, then
    the near band. The near band is where the robot stood before it drove on.
    """
    guard = ObstacleGuard(ObstacleConfig(decel_s=0.0, pause_s=0.0, accel_s=0.0))
    guard.update(0.15, 0.0, False, 0.0, cruise_speed=6.0)   # STOPPING
    guard.update(0.15, 0.1, False, 0.0)                     # PAUSED
    guard.update(0.15, 0.2, False, 0.0)                     # BACKING
    assert guard.state is Obstacle.BACKING

    assert guard.update(0.4, 0.3, True, 0.1) is Obstacle.BACKING    # far band
    assert guard.update(0.4, 0.4, True, 0.2) is Obstacle.BACKING    # still on it
    assert guard.update(0.4, 0.5, False, 0.3) is Obstacle.BACKING   # the code
    assert guard.update(0.4, 0.6, True, 0.4) is Obstacle.HOLDING    # near band
    assert guard.borders_seen == 2


def test_a_band_already_underneath_is_not_counted():
    """If the reflex fires while the sensor is still over a tile, that band is
    one the robot has already crossed."""
    guard = ObstacleGuard(ObstacleConfig(decel_s=0.0, pause_s=0.0, accel_s=0.0))
    guard.update(0.15, 0.0, True, 0.0, cruise_speed=6.0)
    guard.update(0.15, 0.1, True, 0.0)
    guard.update(0.15, 0.2, True, 0.0)   # BACKING, already on orange
    assert guard.state is Obstacle.BACKING
    assert guard.borders_seen == 0

    assert guard.update(0.4, 0.3, False, 0.1) is Obstacle.BACKING
    assert guard.update(0.4, 0.4, True, 0.2) is Obstacle.BACKING   # band one
    assert guard.update(0.4, 0.5, False, 0.3) is Obstacle.BACKING
    assert guard.update(0.4, 0.6, True, 0.4) is Obstacle.HOLDING   # band two


def test_it_gives_up_rather_than_reversing_blind():
    """No rear sensor, so reversing for ever is not safe. If the marker never
    turns up, stop anyway."""
    guard = ObstacleGuard(ObstacleConfig(decel_s=0.0, pause_s=0.0, accel_s=0.0,
                                         max_backoff_m=0.5))
    guard.update(0.15, 0.0, False, 0.0, cruise_speed=6.0)
    guard.update(0.15, 0.1, False, 0.0)
    assert guard.update(0.15, 0.2, False, 0.1) is Obstacle.BACKING
    assert guard.update(0.15, 0.3, False, 0.6) is Obstacle.HOLDING


def test_parked_robot_does_not_resume_on_its_own_retreat():
    """Reversing is what produced the clearance, so clearance alone must not
    mean "all clear" -- or the robot drives forward, trips on the same
    obstacle, reverses, and repeats for ever. Seen in sim as a BACKING/CLEAR
    cycle every four seconds.
    """
    guard = ObstacleGuard(ObstacleConfig(decel_s=0.0, pause_s=0.0, accel_s=0.0,
                                         departed_m=0.15))
    guard.update(0.15, 0.0, False, 0.0, cruise_speed=6.0)  # STOPPING
    guard.update(0.15, 0.1, False, 0.0)                    # PAUSED
    guard.update(0.15, 0.2, False, 0.1)                    # BACKING
    guard.update(0.15, 0.3, True, 0.2)                     # far band
    guard.update(0.15, 0.4, False, 0.3)                    # the code
    guard.update(0.15, 0.5, True, 0.4)                     # near band -> parked
    assert guard.state is Obstacle.HOLDING

    # Range is well past clear_m, but only because the robot moved.
    for step in range(20):
        assert guard.update(0.32, 1.0 + step * 0.1, False, 0.5) is Obstacle.HOLDING

    # The obstacle itself moving is a different matter.
    assert guard.update(0.60, 5.0, False, 0.5) is Obstacle.CLEAR


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
