import pytest

from robot.line_estimator import LineEstimator

# Sensors left to right; +y is left in the robot frame.
OFFSETS = (0.02, 0.0, -0.02)
WHITE = 1000.0
BLACK = 200.0


def estimator(**overrides):
    kwargs = dict(offsets=OFFSETS, white_ref=WHITE, black_ref=BLACK, min_confidence=0.15)
    kwargs.update(overrides)
    return LineEstimator(**kwargs)


def test_line_under_the_centre_sensor_reads_as_centred():
    reading = estimator().estimate([WHITE, BLACK, WHITE])

    assert reading.position == pytest.approx(0.0)


def test_line_under_the_left_sensor_reads_as_left_of_centre():
    reading = estimator().estimate([BLACK, WHITE, WHITE])

    assert reading.position == pytest.approx(0.02)


def test_line_under_the_right_sensor_reads_as_right_of_centre():
    reading = estimator().estimate([WHITE, WHITE, BLACK])

    assert reading.position == pytest.approx(-0.02)


def test_line_straddling_two_sensors_interpolates_between_them():
    half = (WHITE + BLACK) / 2

    reading = estimator().estimate([half, half, WHITE])

    assert reading.position == pytest.approx(0.01)


def test_an_all_white_array_reports_the_line_as_lost():
    reading = estimator().estimate([WHITE, WHITE, WHITE])

    assert reading.lost is True
    assert reading.position == pytest.approx(0.0)


def test_a_faint_reading_below_the_confidence_floor_counts_as_lost():
    barely_dark = WHITE - 0.05 * (WHITE - BLACK)

    reading = estimator().estimate([barely_dark, WHITE, WHITE])

    assert reading.confidence == pytest.approx(0.05)
    assert reading.lost is True


def test_a_confident_reading_is_not_lost():
    reading = estimator().estimate([WHITE, BLACK, WHITE])

    assert reading.lost is False


def test_readings_outside_the_calibrated_range_are_clamped():
    # Brighter than white and darker than black must not produce weights
    # outside [0, 1], which would drag the weighted mean off the array.
    reading = estimator().estimate([1200.0, 50.0, WHITE])

    assert reading.normalised == pytest.approx((1.0, 0.0, 1.0))
    assert reading.position == pytest.approx(0.0)


# ------------------------------------------------------- too much of a line --

def test_an_array_seeing_only_black_is_flagged_as_saturated():
    """Off the edge of the ground plane every sensor reads `black_ref`, and a
    weighted mean of the offsets is then exactly zero at maximum confidence:
    the estimator reports a perfectly centred line with total certainty. A
    robot drove 150 s off the map that way, reporting it was on the line the
    whole time. Nothing was too little; there was too much."""
    estimator = LineEstimator(offsets=(0.02, 0.0, -0.02), white_ref=1023,
                              black_ref=205, min_confidence=0.15)

    reading = estimator.estimate([205, 205, 205])

    assert reading.lost is False        # it really is confident
    assert reading.position == pytest.approx(0.0)
    assert reading.saturated is True    # and that is exactly the problem


def test_a_normal_line_is_not_saturated():
    estimator = LineEstimator(offsets=(0.02, 0.0, -0.02), white_ref=1023,
                              black_ref=205, min_confidence=0.15)

    assert estimator.estimate([957, 603, 956]).saturated is False


def test_an_array_seeing_no_line_at_all_is_not_saturated():
    """Saturation is the opposite failure from a lost line, and the two must
    not be confused: this one is already handled by `lost`."""
    estimator = LineEstimator(offsets=(0.02, 0.0, -0.02), white_ref=1023,
                              black_ref=205, min_confidence=0.15)

    reading = estimator.estimate([1023, 1023, 1023])

    assert reading.lost is True
    assert reading.saturated is False


def test_a_lane_crossing_is_saturated_too_and_that_is_correct():
    """All three sensors are legitimately dark crossing a perpendicular lane.
    The reading is identical to being off the world; only how long it lasts
    tells them apart, which is why the firmware measures distance rather than
    treating one sample as a fault."""
    estimator = LineEstimator(offsets=(0.02, 0.0, -0.02), white_ref=1023,
                              black_ref=205, min_confidence=0.15)

    assert estimator.estimate([210, 208, 209]).saturated is True
