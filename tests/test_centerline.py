import pytest
import math

from tools.track.centerline import oval


def test_oval_perimeter_is_two_straights_plus_a_full_circle():
    # A 4.0 x 3.0 oval is a stadium: semicircles of radius 1.5 at each end,
    # joined by two straights of length 4.0 - 3.0 = 1.0.
    track = oval(width=4.0, height=3.0)

    expected = 2 * 1.0 + 2 * math.pi * 1.5

    assert track.length == pytest.approx(expected)


def test_point_at_maps_normalised_arclength_onto_the_stadium():
    # Centred on the origin. Arclength starts at the left end of the bottom
    # straight and runs counter-clockwise: straight, right arc, top straight,
    # left arc.
    track = oval(width=4.0, height=3.0)
    at = lambda distance: track.point_at(distance / track.length)

    quarter_arc = math.pi * 1.5 / 2

    assert at(0.0) == pytest.approx((-0.5, -1.5))
    assert at(1.0) == pytest.approx((0.5, -1.5))
    assert at(1.0 + quarter_arc) == pytest.approx((2.0, 0.0))
    assert at(1.0 + 2 * quarter_arc) == pytest.approx((0.5, 1.5))
    assert at(2.0 + 2 * quarter_arc) == pytest.approx((-0.5, 1.5))
    assert at(2.0 + 3 * quarter_arc) == pytest.approx((-2.0, 0.0))


def test_point_at_wraps_around_the_closed_loop():
    track = oval(width=4.0, height=3.0)

    assert track.point_at(1.0) == pytest.approx(track.point_at(0.0))
    assert track.point_at(1.25) == pytest.approx(track.point_at(0.25))


def test_signed_distance_is_zero_on_the_centerline():
    track = oval(width=4.0, height=3.0)

    for s in (0.0, 0.17, 0.5, 0.83):
        x, y = track.point_at(s)
        assert track.signed_distance_to(x, y) == pytest.approx(0.0, abs=1e-9)


def test_signed_distance_is_negative_inside_the_loop_and_positive_outside():
    track = oval(width=4.0, height=3.0)

    # Bottom straight sits at y = -1.5; move 0.2 toward the middle, then away.
    assert track.signed_distance_to(0.0, -1.3) == pytest.approx(-0.2)
    assert track.signed_distance_to(0.0, -1.7) == pytest.approx(0.2)

    # Beyond the right arc, whose apex is at (2.0, 0.0).
    assert track.signed_distance_to(2.5, 0.0) == pytest.approx(0.5)


def test_oval_rejects_a_height_greater_than_its_width():
    # The straight sections would have negative length.
    with pytest.raises(ValueError, match="width"):
        oval(width=2.0, height=3.0)


def test_heading_is_tangent_to_the_track():
    """Compare the analytic heading against a finite difference of point_at.

    Markers are laid along the lane, so a wrong heading rotates every one of
    them off the line the robot is following.
    """
    track = oval(width=3.0, height=2.0)
    step = 1e-6

    for s in (0.0, 0.1, 0.25, 0.4, 0.5, 0.6, 0.75, 0.9):
        x0, y0 = track.point_at(s - step)
        x1, y1 = track.point_at(s + step)
        expected = math.atan2(y1 - y0, x1 - x0)
        actual = track.heading_at(s)
        difference = (actual - expected + math.pi) % (2 * math.pi) - math.pi
        assert abs(difference) < 1e-3, "heading at s={} was {}, expected {}".format(
            s, actual, expected)


def test_heading_wraps_with_the_loop():
    track = oval(width=3.0, height=2.0)
    assert track.heading_at(1.25) == pytest.approx(track.heading_at(0.25))
