import pytest

from robot.drive import differential_speeds


def test_zero_steering_drives_both_wheels_at_the_base_speed():
    assert differential_speeds(base=4.0, steering=0.0, max_speed=10.0) == pytest.approx((4.0, 4.0))


def test_positive_steering_turns_left_by_speeding_up_the_right_wheel():
    left, right = differential_speeds(base=4.0, steering=1.0, max_speed=10.0)

    assert (left, right) == pytest.approx((3.0, 5.0))


def test_saturation_preserves_the_speed_difference_that_steers():
    # Naive clamping would cap the right wheel at 5.0 and leave the left at 1.0,
    # silently halving the turn authority exactly when it is needed most.
    left, right = differential_speeds(base=4.0, steering=3.0, max_speed=5.0)

    assert right - left == pytest.approx(6.0)
    assert max(abs(left), abs(right)) <= 5.0


def test_a_difference_wider_than_the_wheels_allow_is_scaled_down():
    left, right = differential_speeds(base=0.0, steering=20.0, max_speed=5.0)

    assert (left, right) == pytest.approx((-5.0, 5.0))
