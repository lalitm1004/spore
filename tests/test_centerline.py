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
