"""Crossing a marker blind: the colour trigger, the state machine, odometry.

The crossing is the one stretch where the robot has no lateral feedback, so
these tests care most about the boundaries -- when the line stops being
trustworthy, and when the code is actually under the camera.
"""

import math

import pytest

from robot.marker import (
    BorderDetector,
    Crossing,
    CrossingConfig,
    MarkerCrossing,
    chromaticity,
)
from robot.odometry import Odometry, Pose


# --------------------------------------------------------------- odometry ---

def test_first_update_only_establishes_a_reference():
    """A shaft angle means nothing alone; only its change carries distance."""
    odometry = Odometry(wheel_radius=0.02, track_width=0.09)
    assert odometry.update(5.0, 5.0) == Pose()


def test_straight_line_distance_matches_wheel_arc():
    odometry = Odometry(wheel_radius=0.02, track_width=0.09)
    odometry.update(0.0, 0.0)
    pose = odometry.update(1.0, 1.0)

    assert pose.distance == pytest.approx(0.02)
    assert pose.x == pytest.approx(0.02)
    assert pose.y == pytest.approx(0.0, abs=1e-9)
    assert pose.theta == pytest.approx(0.0, abs=1e-9)


def test_spinning_in_place_travels_no_distance_but_turns():
    odometry = Odometry(wheel_radius=0.02, track_width=0.09)
    odometry.update(0.0, 0.0)
    pose = odometry.update(-1.0, 1.0)

    assert pose.x == pytest.approx(0.0, abs=1e-9)
    assert pose.theta == pytest.approx(2 * 0.02 / 0.09)


def test_distance_is_monotonic_through_a_reverse():
    """Crossing keys off path length, which must not unwind when backing up."""
    odometry = Odometry(wheel_radius=0.02, track_width=0.09)
    odometry.update(0.0, 0.0)
    odometry.update(1.0, 1.0)
    pose = odometry.update(0.0, 0.0)

    assert pose.distance == pytest.approx(0.04)
    assert pose.x == pytest.approx(0.0, abs=1e-9)


def test_reset_keeps_the_encoder_reference():
    """An absolute fix discards drift, not the encoder's zero point."""
    odometry = Odometry(wheel_radius=0.02, track_width=0.09)
    odometry.update(0.0, 0.0)
    odometry.update(1.0, 1.0)
    odometry.reset()
    pose = odometry.update(2.0, 2.0)

    assert pose.distance == pytest.approx(0.02)


def test_rejects_impossible_geometry():
    with pytest.raises(ValueError):
        Odometry(wheel_radius=0.0, track_width=0.09)


# ------------------------------------------------------- border detection ---

def test_border_is_recognised_at_its_own_colour():
    detector = BorderDetector((255, 122, 0))
    assert detector.sees_border((255, 122, 0))


def test_border_survives_a_change_in_lighting():
    """Chromaticity divides intensity out, so a dim or bright patch still reads.

    This is the whole reason the classifier is not a plain RGB threshold: the
    scene's DirectionalLight is 2.5, and nobody should have to retune a sensor
    because someone adjusted a lamp.
    """
    detector = BorderDetector((255, 122, 0))
    for scale in (0.35, 0.6, 1.0):
        dimmed = tuple(c * scale for c in (255, 122, 0))
        assert detector.sees_border(dimmed), "failed at {}x brightness".format(scale)


def test_track_colours_are_not_mistaken_for_the_border():
    detector = BorderDetector((255, 122, 0))
    for colour in ((255, 255, 255), (0, 0, 0), (128, 128, 128), (30, 90, 220)):
        assert not detector.sees_border(colour), "{} read as border".format(colour)


def test_chromaticity_of_black_is_undefined():
    assert chromaticity((0, 0, 0)) is None
    assert BorderDetector((255, 122, 0)).distance((0, 0, 0)) == float("inf")


# ------------------------------------------------------ crossing sequence ---

def test_line_is_trusted_until_the_tile_reaches_the_ir_array():
    """The colour sensor leads the array, so the line is still good for that
    first stretch even though a marker has already been detected."""
    crossing = MarkerCrossing()
    config = crossing.config
    crossing.update(0.0, sees_border=True)

    assert crossing.state is Crossing.OVER
    assert crossing.line_is_trustworthy(config.blind_start - 0.005)
    assert not crossing.line_is_trustworthy(config.blind_start + 0.005)
    assert not crossing.line_is_trustworthy(config.blind_end - 0.005)
    assert crossing.line_is_trustworthy(config.blind_end + 0.005)


def test_crossing_ends_and_recovers_when_the_line_returns():
    crossing = MarkerCrossing()
    end = crossing.config.blind_end
    crossing.update(0.0, sees_border=True)
    assert crossing.update(end / 2, sees_border=False) is Crossing.OVER
    assert crossing.update(end + 0.01, sees_border=False) is Crossing.RECOVERING

    crossing.recovered()
    assert crossing.state is Crossing.CLEAR
    assert crossing.crossings == 1


def test_a_second_marker_during_recovery_starts_a_new_crossing():
    crossing = MarkerCrossing()
    end = crossing.config.blind_end
    crossing.update(0.0, sees_border=True)
    crossing.update(end + 0.01, sees_border=False)
    crossing.update(end + 0.02, sees_border=True)

    assert crossing.state is Crossing.OVER
    assert crossing.crossings == 2


def test_the_read_window_sits_inside_the_blind_stretch():
    """The code must be readable before the tile leaves the IR array, or a
    failed decode has no second chance."""
    crossing = MarkerCrossing()
    crossing.update(0.0, sees_border=True)

    window = [d / 10000.0 for d in range(0, 2000)
              if crossing.should_read(d / 10000.0, camera_x=0.095,
                                      footprint=0.0927, code_size=0.060)]
    assert window, "the code is never in view"

    start, end = window[0], window[-1]
    assert end < crossing.config.blind_end, (
        "read window ends at {:.3f} m, after the tile clears the array at "
        "{:.3f} m".format(end, crossing.config.blind_end))
    assert (end - start) > 0.02, "only {:.0f} mm of read window".format((end - start) * 1000)


def test_no_reading_when_no_marker_is_present():
    crossing = MarkerCrossing()
    crossing.update(0.5, sees_border=False)
    assert not crossing.should_read(0.5, 0.095, 0.0927, 0.060)


def test_camera_position_is_far_enough_forward():
    """A camera mounted too far back sees the code only after the crossing.

    This is the geometry check that caught cameraX=0.03: the read window ran
    past the end of the blind stretch, so the robot regained the line before
    it had read anything.
    """
    config = CrossingConfig()
    footprint, code = 0.0927, 0.060
    slack = (footprint - code) / 2.0

    for camera_x, expected in ((0.030, False), (0.095, True)):
        centre_at_read = config.color_sensor_x + config.tile_length / 2.0 - camera_x
        assert (centre_at_read + slack < config.blind_end) is expected


def test_odometry_and_crossing_agree_over_a_simulated_pass():
    """Drive a wheel-angle sequence through both and check the sequencing."""
    odometry = Odometry(wheel_radius=0.02, track_width=0.09)
    crossing = MarkerCrossing()

    step = 0.12 * 0.016 / 0.02      # 0.12 m/s for 16 ms, in shaft radians
    angle = 0.0
    odometry.update(angle, angle)

    blind_steps, read_steps = 0, 0
    for tick in range(300):
        angle += step
        pose = odometry.update(angle, angle)
        # The border is under the colour sensor for the first 15 mm of tile.
        on_border = 0.30 <= pose.distance <= 0.315
        crossing.update(pose.distance, sees_border=on_border)

        if not crossing.line_is_trustworthy(pose.distance):
            blind_steps += 1
        if crossing.should_read(pose.distance, 0.095, 0.0927, 0.060):
            read_steps += 1

    assert crossing.crossings == 1
    assert blind_steps > 0 and read_steps > 0
    # The tile is still 100 mm, so the blind stretch is still ~52 steps at
    # 1.92 mm per step -- it just starts 55 mm after the trigger now. The
    # read window is
    # 2 * (16.35 slack + 5 margin) = 42.7 mm, so ~22 steps -- 22 chances to
    # decode one code, which is why a single blurred frame is not a failure.
    assert 45 <= blind_steps <= 60, blind_steps
    assert 18 <= read_steps <= 28, read_steps
