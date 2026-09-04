"""An MCU-like front end between the simulator and the control core.

Webots hands out continuous floats sampled at the control rate. Real firmware
reads quantised ADC counts, on its own sampling clock, with transport delay.
Putting that difference in simulation means quantisation and aliasing surprises
show up here rather than on the robot. Pure: no Webots, no I/O.
"""

from dataclasses import dataclass
from typing import List, Sequence, Tuple


@dataclass(frozen=True)
class Adc:
    """A unipolar ADC covering 0..full_scale in 2**bits steps."""

    bits: int = 10
    full_scale: float = 1000.0

    @property
    def max_count(self) -> int:
        return (1 << self.bits) - 1

    def counts(self, value: float) -> int:
        fraction = value / self.full_scale
        fraction = min(1.0, max(0.0, fraction))
        return int(round(fraction * self.max_count))


class SampledSensors:
    """Sample-and-hold on its own clock, with optional transport latency.

    `update` is called every control step; it returns the counts the firmware
    can currently see, which is the freshest sample old enough to have arrived.
    """

    def __init__(self, adc: Adc, sample_period_s: float, latency_s: float = 0.0):
        self.adc = adc
        self.sample_period_s = sample_period_s
        self.latency_s = latency_s
        self._samples: List[Tuple[float, Tuple[int, ...]]] = []
        self._next_sample_at = None

    def update(self, t: float, raw: Sequence[float]) -> Tuple[int, ...]:
        if self._next_sample_at is None or t >= self._next_sample_at:
            self._samples.append((t, tuple(self.adc.counts(v) for v in raw)))
            self._next_sample_at = t + self.sample_period_s

        visible = None
        for timestamp, counts in self._samples:
            if timestamp <= t - self.latency_s + 1e-12:
                visible = counts
            else:
                break

        if visible is None:  # nothing has arrived yet; hold the oldest sample
            visible = self._samples[0][1]

        # Drop samples that can no longer become visible.
        while len(self._samples) > 1 and self._samples[1][0] <= t - self.latency_s + 1e-12:
            self._samples.pop(0)

        return visible
