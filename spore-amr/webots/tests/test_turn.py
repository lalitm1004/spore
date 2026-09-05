"""Turning to an absolute bearing.

`robot/turn.py` had no tests at all. These cover what it actually guarantees:
the short way round, a floor under the output so the last few degrees do not
stall, a cap at full authority, and a timeout so a robot never holds a lane for
ever.

Not covered, deliberately. The class docstring claims that requiring several
consecutive in-band steps stops a fast pass through the target being mistaken
for arriving at it. That is not true for a chassis heavy enough to coast: at
full authority the robot turns 92 deg/s and the tolerance band is 4 deg wide,
so it crosses in 43 ms -- 2.7 control steps, fewer than the three
`settle_steps` asks for. The original 120 mm chassis stopped the moment the
steering did and settled anyway; the 500 mm deck does not, which is where its
occasional 14 deg heading miss comes from.

Braking on the differenced heading rate was tried as the fix and made things
far worse -- 25 turn timeouts against 0, and median heading error 0.1 -> 87 deg
-- so the controller is left as it is and the residual is recorded in
fleet.yaml. A real fix needs the turn to decelerate into the band rather than
be told to stop at its edge, which is a redesign rather than a constant.
"""

import math

import pytest

from robot.turn import TurnConfig, TurnController, wrap


def test_wrap_takes_the_short_way_round():
    """Turning 350 deg left is turning 10 deg right, and the controller must
    never choose the long way."""
    assert wrap(math.radians(350)) == pytest.approx(math.radians(-10))
    assert wrap(math.radians(-350)) == pytest.approx(math.radians(10))
    # Exactly pi lands on -pi: the same bearing, and the reason `wrap` is
    # documented as half-open at the negative end.
    assert wrap(math.pi) == pytest.approx(-math.pi)


def test_a_settled_turn_completes():
    """A robot genuinely on target must finish, or the turn never ends and the
    firmware's junction timeout drives it on regardless."""
    controller = TurnController()
    controller.start(0.0, 0.0)

    now, done = 0.0, False
    for _ in range(6):
        now += 0.016
        _, done, _ = controller.update(math.radians(0.5), now)
        if done:
            break
    assert done, "a settled turn never completed"
    assert not controller.active


def test_it_drives_toward_a_distant_target():
    controller = TurnController()
    controller.start(math.radians(90), 0.0)
    steering, done, timed_out = controller.update(0.0, 0.016)
    assert not done and not timed_out
    assert steering > 0, "should turn left toward a target 90 deg to the left"


def test_output_keeps_a_floor_until_the_band_is_reached():
    """A pure P term dies exactly where static friction is most able to stop
    the robot short, so the magnitude has a floor outside the band."""
    config = TurnConfig()
    controller = TurnController(config)
    controller.start(0.0, 0.0)
    steering, _, _ = controller.update(math.radians(2.5), 0.016)   # just outside
    assert abs(steering) >= config.min_rate


def test_output_is_capped_at_full_authority():
    config = TurnConfig()
    controller = TurnController(config)
    controller.start(math.pi, 0.0)
    steering, _, _ = controller.update(0.0, 0.016)
    assert abs(steering) <= config.max_rate


def test_a_turn_that_never_arrives_times_out():
    """A robot must never hold a lane for ever because a turn will not settle."""
    config = TurnConfig()
    controller = TurnController(config)
    controller.start(math.radians(90), 0.0)
    _, done, timed_out = controller.update(0.0, config.timeout_s + 0.1)
    assert timed_out and not done
    assert not controller.active
