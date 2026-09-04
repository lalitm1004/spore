"""Turn raw IR readings into a line position. Pure: no Webots, no I/O."""

from dataclasses import dataclass
from typing import Sequence, Tuple


@dataclass(frozen=True)
class LineReading:
    position: float
    confidence: float
    lost: bool
    normalised: Tuple[float, ...]


@dataclass(frozen=True)
class LineEstimator:
    """Weighted-mean line position over a lateral sensor array.

    `offsets` are the sensors' lateral positions in the robot frame, +y left, so
    a positive `position` means the line lies to the robot's left.
    """

    offsets: Tuple[float, ...]
    white_ref: float
    black_ref: float
    min_confidence: float

    def estimate(self, readings: Sequence[float]) -> LineReading:
        span = self.white_ref - self.black_ref
        normalised = tuple(
            min(1.0, max(0.0, (r - self.black_ref) / span)) for r in readings
        )

        weights = [1.0 - n for n in normalised]
        confidence = sum(weights)
        if confidence < self.min_confidence:
            # Too little of the line under the array to place it. The caller
            # decides how to recover; the estimator does not guess.
            return LineReading(
                position=0.0, confidence=confidence, lost=True, normalised=normalised
            )

        position = sum(w * o for w, o in zip(weights, self.offsets)) / confidence
        return LineReading(
            position=position, confidence=confidence, lost=False, normalised=normalised
        )
