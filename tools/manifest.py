"""Loading and validation for `fleet.yaml`, the single source of truth."""

from dataclasses import dataclass
from typing import Tuple

from tools.track.centerline import oval

SHAPES = {"oval": oval}


@dataclass(frozen=True)
class TrackConfig:
    """Ground plane and the track drawn on it.

    The plane and the track are sized independently so the plane can stay square
    (and therefore a power of two in pixels, which Webots does not rescale)
    while the track itself is free to be a non-square oval.
    """

    plane_size: Tuple[float, float]
    track_size: Tuple[float, float]
    shape: str = "oval"
    line_width: float = 0.02
    pixels_per_metre: int = 512

    @classmethod
    def from_dict(cls, data: dict) -> "TrackConfig":
        known = {f: data[f] for f in cls.__dataclass_fields__ if f in data}
        for key in ("plane_size", "track_size"):
            known[key] = tuple(float(v) for v in known[key])
        return cls(**known)

    def build_centerline(self):
        if self.shape not in SHAPES:
            raise ValueError(
                "unknown track shape {!r}; known shapes: {}".format(
                    self.shape, ", ".join(sorted(SHAPES))
                )
            )

        for axis, (track, plane) in enumerate(zip(self.track_size, self.plane_size)):
            if track + self.line_width > plane:
                raise ValueError(
                    "track {} does not fit on the plane along axis {}: "
                    "{} plus a {} line exceeds {}".format(
                        self.track_size, axis, track, self.line_width, plane
                    )
                )

        return SHAPES[self.shape](width=self.track_size[0], height=self.track_size[1])


def deep_merge(base: dict, override: dict) -> dict:
    """Recursively merge `override` onto `base`, mutating neither."""
    merged = dict(base)
    for key, value in override.items():
        existing = merged.get(key)
        if isinstance(existing, dict) and isinstance(value, dict):
            merged[key] = deep_merge(existing, value)
        else:
            merged[key] = value
    return merged
