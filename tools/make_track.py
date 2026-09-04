"""CLI: render the ground texture described by `fleet.yaml`."""

import argparse
import pathlib

import yaml

from tools.manifest import TrackConfig
from tools.track.raster import TrackImageSpec, render_track

DEFAULT_MANIFEST = pathlib.Path("fleet.yaml")
DEFAULT_OUTPUT = pathlib.Path("worlds/textures/track.png")


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=pathlib.Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--output", type=pathlib.Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args(argv)

    manifest = yaml.safe_load(args.manifest.read_text())
    config = TrackConfig.from_dict(manifest["track"])

    centerline = config.build_centerline()
    spec = TrackImageSpec(size=config.plane_size, pixels_per_metre=config.pixels_per_metre)
    image = render_track(centerline, spec, line_width=config.line_width)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    image.save(args.output)
    print(
        "{} ({}x{} px, {:.2f} m centerline)".format(
            args.output, spec.width_px, spec.height_px, centerline.length
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
