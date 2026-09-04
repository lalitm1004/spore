"""Render a track centerline to a ground texture.

The generated PNG is applied to a `Plane` in the world. Webots maps a plane's
texture as a 2D image seen from above, so image x runs with world +x and image
rows run against world +y.
"""

from dataclasses import dataclass
from typing import Tuple

WHITE = (255, 255, 255)
BLACK = (0, 0, 0)


@dataclass(frozen=True)
class TrackImageSpec:
    """Maps world metres onto texture pixels."""

    size: Tuple[float, float]
    pixels_per_metre: int

    @property
    def width_px(self) -> int:
        return round(self.size[0] * self.pixels_per_metre)

    @property
    def height_px(self) -> int:
        return round(self.size[1] * self.pixels_per_metre)

    def world_to_pixel(self, x: float, y: float) -> Tuple[float, float]:
        width, height = self.size
        return (
            (x + width / 2) * self.pixels_per_metre,
            (height / 2 - y) * self.pixels_per_metre,
        )


def render_track(centerline, spec: TrackImageSpec, line_width: float) -> "Image.Image":
    """Draw `centerline` as a dark line on a light background.

    Contrast is read by the IR sensors on the red channel, so the line is pure
    black on pure white to span the full reflection range.
    """
    from PIL import Image, ImageDraw

    image = Image.new("RGB", (spec.width_px, spec.height_px), WHITE)
    draw = ImageDraw.Draw(image)

    # Sample finely enough that each chord is well under a pixel.
    steps = max(64, int(centerline.length * spec.pixels_per_metre * 2))
    points = [spec.world_to_pixel(*centerline.point_at(i / steps)) for i in range(steps)]
    points.append(points[0])  # close the loop

    draw.line(points, fill=BLACK, width=round(line_width * spec.pixels_per_metre), joint="curve")
    return image
