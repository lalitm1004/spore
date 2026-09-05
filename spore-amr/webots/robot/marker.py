"""Crossing a floor marker: the colour trigger and the blind-crossing state.

A marker tile covers the line for its whole length, so the robot crosses it
without lateral feedback. The sequence, all distances measured from the moment
the colour sensor first sees the border:

      d = 0      colour sensor (x=85mm) meets the tile's near edge
      d = 15     IR array (x=70mm) enters the tile -- the line is gone
      d = 59-91  QR fully inside the camera's footprint; read it here
      d = 115    IR array leaves the tile; the line is back

No alignment manoeuvre and no stop. The robot has been following the line to
get here, and markers are laid along the lane tangent, so it arrives square by
construction -- roughly 2mm of cross-track error against 16mm of slack between
the 60mm code and the 92.7mm footprint.

Pure: no Webots, no I/O, no camera decoding.
"""

from dataclasses import dataclass
from enum import Enum
from typing import Optional, Tuple

RGB = Tuple[float, float, float]


class Crossing(Enum):
    """Where the robot is relative to a marker."""

    CLEAR = "CLEAR"          # following the line normally
    OVER = "OVER"            # on the tile, holding heading, reading the code
    RECOVERING = "RECOVERING"  # past the tile, expecting the line back


def chromaticity(rgb: RGB) -> Optional[RGB]:
    """Colour with intensity divided out.

    Lighting scales all three channels together, so chromaticity is what
    survives a bright patch or a shadow. Returns None for black, where the
    ratios are meaningless.
    """
    total = sum(rgb)
    if total <= 1e-6:
        return None
    return (rgb[0] / total, rgb[1] / total, rgb[2] / total)


class BorderDetector:
    """Classifies the 1x1 colour camera against the marker's border colour.

    Compares chromaticity rather than RGB so the threshold does not have to be
    retuned when the scene lighting changes -- which it does the moment anyone
    touches the DirectionalLight.

    The default tolerance is measured, not guessed. A (255, 122, 0) border
    renders as (242, 173, 56) under the world's lighting, which is 0.206 from
    the reference; the nearest thing it could be confused with is the white
    floor at 0.481. 0.30 sits between them with margin on both sides:

        rendered border  (242, 173, 56)   0.206   must match
        white floor      (242, 244, 245)  0.481   must not
        black line       ( 53,  57,  61)  0.512   must not
        QR grey          (159, 162, 165)  0.487   must not
        sky              (153, 178, 217)  0.561   must not

    Re-measure with tools/spike_drive.py after any lighting change.
    """

    def __init__(self, border_rgb: Tuple[int, int, int], tolerance: float = 0.30):
        reference = chromaticity(tuple(float(c) for c in border_rgb))
        if reference is None:
            raise ValueError("border colour cannot be black")
        self.reference = reference
        self.tolerance = tolerance

    def distance(self, rgb: RGB) -> float:
        """Chromaticity distance from the border colour; inf for black."""
        sample = chromaticity(rgb)
        if sample is None:
            return float("inf")
        return sum((a - b) ** 2 for a, b in zip(sample, self.reference)) ** 0.5

    def sees_border(self, rgb: RGB) -> bool:
        return self.distance(rgb) <= self.tolerance


@dataclass(frozen=True)
class CrossingConfig:
    """Tile geometry, in metres, in the robot's own frame."""

    tile_length: float = 0.100
    color_sensor_x: float = 0.125
    ir_array_x: float = 0.070

    @property
    def blind_start(self) -> float:
        """Travel at which the IR array enters the tile and the line vanishes."""
        return self.color_sensor_x - self.ir_array_x

    @property
    def blind_end(self) -> float:
        """Travel at which the IR array clears the tile."""
        return self.blind_start + self.tile_length


class MarkerCrossing:
    """Tracks one marker crossing, driven by odometry distance.

    Deliberately knows nothing about steering or the camera: it answers "should
    I trust the line right now" and "should the camera be on", and the firmware
    decides what to do about it.
    """

    def __init__(self, config: CrossingConfig = CrossingConfig(),
                 read_margin: float = 0.005):
        self.config = config
        self.read_margin = read_margin
        self.state = Crossing.CLEAR
        self.entered_at: Optional[float] = None
        self._border_was = False
        self.crossings = 0

    def update(self, distance: float, sees_border: bool) -> Crossing:
        """Advance the state machine. `distance` is total odometry path length."""
        if self.state is Crossing.CLEAR:
            if sees_border:
                self.state = Crossing.OVER
                self.entered_at = distance
                self.crossings += 1
            return self.state

        travelled = distance - self.entered_at if self.entered_at is not None else 0.0

        if self.state is Crossing.OVER:
            if travelled >= self.config.blind_end:
                self.state = Crossing.RECOVERING
            return self.state

        # RECOVERING: the tile is behind us. Seeing the border again means a
        # second marker, so re-enter rather than waiting for the line.
        if sees_border:
            self.state = Crossing.OVER
            self.entered_at = distance
            self.crossings += 1
        return self.state

    def line_is_trustworthy(self, distance: float) -> bool:
        """False while the tile is under the IR array."""
        if self.state is not Crossing.OVER or self.entered_at is None:
            return True
        travelled = distance - self.entered_at
        return not (self.config.blind_start <= travelled < self.config.blind_end)

    def should_read(self, distance: float, camera_x: float,
                    footprint: float, code_size: float) -> bool:
        """True while the code is inside the camera's footprint.

        At d=0 the tile's near edge sits under the colour sensor, so in the
        robot frame the tile centre is at `color_sensor_x + tile/2 - d`, and
        the code is wholly in view within half the camera's spare footprint.
        `read_margin` widens that slightly to absorb the quantisation of the
        trigger -- the border is sampled once per control step, so the entry
        point is known only to about 2 mm.
        """
        if self.state is not Crossing.OVER or self.entered_at is None:
            return False

        travelled = distance - self.entered_at
        centre_x = self.config.color_sensor_x + self.config.tile_length / 2.0 - travelled
        slack = max(0.0, (footprint - code_size) / 2.0)
        return abs(centre_x - camera_x) <= slack + self.read_margin

    def lever_arm(self, distance: float) -> Optional[float]:
        """Metres from the robot's origin forward to the marker's centre.

        At d=0 the tile's near edge is under the colour sensor, so the centre
        is `color_sensor_x + tile/2` ahead of the origin, closing as the robot
        advances. This is what turns "the marker is at (x, y)" into "I am at
        (x, y)", and without it a fix is wrong by the length of the boom.
        """
        if self.entered_at is None:
            return None
        travelled = distance - self.entered_at
        return self.config.color_sensor_x + self.config.tile_length / 2.0 - travelled

    def reset(self) -> None:
        """Forget the current crossing entirely.

        Called after a turn. A crossing measures travel since the tile's near
        edge, and a robot that stopped on the tile to be routed has not
        travelled at all -- so the crossing never advances, `line_is_trustworthy`
        stays false, and the robot drives away from the junction blind, holding
        the steering it had before it turned. That walks it off the lane.

        After a turn the robot is on a different lane facing a different way.
        The old crossing describes none of that.
        """
        self.state = Crossing.CLEAR
        self.entered_at = None
        self._border_was = False

    def recovered(self) -> None:
        """Called once the line estimator has the line again."""
        if self.state is Crossing.RECOVERING:
            self.state = Crossing.CLEAR
            self.entered_at = None
