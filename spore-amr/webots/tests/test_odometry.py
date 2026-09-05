"""Dead reckoning from the wheels.

`distance` is the one field with a subtlety worth a test: it is path length,
not displacement, and code that treats it as "how far along am I" breaks the
moment the robot reverses.
"""

import pytest

from robot.odometry import Odometry




def test_distance_counts_travel_not_displacement():
    """`distance` is path length: it grows when reversing too.

    That is what marker crossing wants -- how far have I gone since the
    trigger -- but it makes `distance` useless as "am I there yet" across a
    retreat. The firmware rolls the last of the boom length onto a node before
    turning, and if the obstacle reflex reverses it mid-roll, the odometer
    keeps climbing and would report the node reached while the robot is
    further from it than when it started. The turn is abandoned on a block for
    exactly this reason.
    """
    odometry = Odometry(wheel_radius=0.02, track_width=0.0994)
    odometry.update(0.0, 0.0)

    odometry.update(1.0, 1.0)               # forward
    forward = odometry.pose.distance
    odometry.update(0.0, 0.0)               # back to where it started

    assert forward > 0.0
    assert odometry.pose.distance == pytest.approx(2 * forward)
    assert odometry.pose.x == pytest.approx(0.0, abs=1e-9)
