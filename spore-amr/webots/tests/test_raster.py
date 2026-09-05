import pytest

from tools.track.raster import TrackImageSpec


def test_image_dimensions_follow_size_and_resolution():
    spec = TrackImageSpec(size=(4.0, 3.0), pixels_per_metre=512)

    assert (spec.width_px, spec.height_px) == (2048, 1536)


def test_world_to_pixel_puts_the_origin_at_the_centre_and_y_up():
    # A Plane maps its texture as a 2D image seen from above, so +y in the
    # world is up in the image and pixel rows increase downward.
    spec = TrackImageSpec(size=(4.0, 3.0), pixels_per_metre=512)

    assert spec.world_to_pixel(0.0, 0.0) == (1024.0, 768.0)
    assert spec.world_to_pixel(-2.0, 1.5) == (0.0, 0.0)
    assert spec.world_to_pixel(2.0, -1.5) == (2048.0, 1536.0)


from tools.track.centerline import oval
from tools.track.raster import render_track


def sample(image, spec, x, y):
    px, py = spec.world_to_pixel(x, y)
    return image.getpixel((int(px), int(py)))


def test_line_is_drawn_dark_on_a_light_background():
    spec = TrackImageSpec(size=(4.0, 3.0), pixels_per_metre=200)
    track = oval(width=3.0, height=2.0)

    image = render_track(track, spec, line_width=0.05)

    assert sample(image, spec, 0.0, -1.0) == (0, 0, 0)      # on the bottom straight
    assert sample(image, spec, 0.0, 0.0) == (255, 255, 255)  # inside the loop
    assert sample(image, spec, -1.9, 1.4) == (255, 255, 255)  # outside the loop


def test_line_is_drawn_at_the_requested_width():
    spec = TrackImageSpec(size=(4.0, 3.0), pixels_per_metre=200)
    track = oval(width=3.0, height=2.0)

    image = render_track(track, spec, line_width=0.05)

    # The bottom straight is centred on y = -1.0, so the line spans +/- 0.025.
    assert sample(image, spec, 0.0, -1.0 + 0.02) == (0, 0, 0)
    assert sample(image, spec, 0.0, -1.0 - 0.02) == (0, 0, 0)
    assert sample(image, spec, 0.0, -1.0 + 0.04) == (255, 255, 255)
    assert sample(image, spec, 0.0, -1.0 - 0.04) == (255, 255, 255)
