"""Loading and validation for `fleet.yaml`, the single source of truth."""

from dataclasses import dataclass
from typing import Tuple

from tools.track.centerline import oval

SHAPES = {"oval": oval}


@dataclass(frozen=True)
class TrackConfig:
    size: Tuple[float, float]
    shape: str = "oval"
    margin: float = 0.5
    line_width: float = 0.02
    pixels_per_metre: int = 512

    @classmethod
    def from_dict(cls, data: dict) -> "TrackConfig":
        known = {f: data[f] for f in cls.__dataclass_fields__ if f in data}
        known["size"] = tuple(float(v) for v in known["size"])
        return cls(**known)

    def build_centerline(self):
        """The track shape, inset from the ground plane by `margin`."""
        if self.shape not in SHAPES:
            raise ValueError(
                "unknown track shape {!r}; known shapes: {}".format(
                    self.shape, ", ".join(sorted(SHAPES))
                )
            )

        width = self.size[0] - 2 * self.margin
        height = self.size[1] - 2 * self.margin
        if width <= 0 or height <= 0:
            raise ValueError(
                "margin {} leaves no room inside a {} x {} plane".format(
                    self.margin, *self.size
                )
            )

        return SHAPES[self.shape](width=width, height=height)


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
