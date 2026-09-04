"""CLI: render one texture per floor marker described by `fleet.yaml`.

Each marker gets its own PNG so its resolution is independent of the track
texture's. Written beside the track texture, where both Webots (resolving the
world's relative url) and the streaming server (serving from the project root)
find the same file.
"""

import argparse
import pathlib

import yaml

from tools.manifest import MarkerConfig, TrackConfig
from tools.track.marker import render_marker

DEFAULT_MANIFEST = pathlib.Path("fleet.yaml")
DEFAULT_OUTPUT = pathlib.Path("textures/markers")


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=pathlib.Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--output", type=pathlib.Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args(argv)

    manifest = yaml.safe_load(args.manifest.read_text())
    track = TrackConfig.from_dict(manifest["track"])
    markers = MarkerConfig.from_dict(manifest.get("markers"))

    if not markers.nodes:
        print("no markers in {}".format(args.manifest))
        return 0

    centerline = track.build_centerline()
    origin_offset = (track.plane_size[0] / 2.0, track.plane_size[1] / 2.0)

    args.output.mkdir(parents=True, exist_ok=True)
    for node in markers.nodes:
        payload = node.payload(centerline, origin_offset)
        image = render_marker(payload, markers.spec)
        path = args.output / "node_{:03d}.png".format(node.node_id)
        image.save(path)
        print("{}  {}".format(path, payload))

    spec = markers.spec
    print("\n{} markers, {} mm tile at {} px, {} mm QR".format(
        len(markers.nodes), spec.tile_mm, spec.size_px, spec.qr_mm))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
