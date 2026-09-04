"""Map a steering signal onto wheel speeds. Pure: no Webots, no I/O."""

from typing import Tuple


def differential_speeds(base: float, steering: float, max_speed: float) -> Tuple[float, float]:
    """Wheel speeds for a differential drive.

    Positive `steering` turns left. When the request exceeds what the motors can
    deliver, the *difference* between the wheels is preserved in favour of the
    forward speed: turn authority is what keeps the robot on the line, and
    clamping each wheel independently would quietly discard it mid-corner.
    """
    left = base - steering
    right = base + steering

    difference = right - left
    if abs(difference) > 2 * max_speed:
        # Not even achievable at full opposing lock; scale the turn itself.
        scale = 2 * max_speed / abs(difference)
        left *= scale
        right *= scale
        difference = right - left

    # Shift both wheels back into range, keeping their difference intact.
    overflow = max(left, right) - max_speed
    if overflow > 0:
        left -= overflow
        right -= overflow

    underflow = -max_speed - min(left, right)
    if underflow > 0:
        left += underflow
        right += underflow

    return (left, right)
