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
from tools.track.marker import encode_payload, render_marker

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
    args.output.mkdir(parents=True, exist_ok=True)
    offset = track.plane_size[0] / 2.0

    if track.is_graph:
        graph = track.build_graph()
        for node in sorted(graph.nodes.values(), key=lambda n: n.node_id):
            payload = encode_payload(
                node_id=node.node_id, kind=node.kind,
                x_cm=round((node.x + offset) * 100, 1),
                y_cm=round((node.y + offset) * 100, 1),
                name=node.name, region_id=node.region_id,
            )
            render_marker(payload, markers.spec).save(
                args.output / "node_{:03d}.png".format(node.node_id))
        print("{} node markers".format(len(graph.nodes)))
        spec = markers.spec
        print("{} mm tile at {} px, {} mm QR".format(
            spec.tile_mm, spec.size_px, spec.qr_mm))
        return 0

    if not markers.nodes:
        print("no markers in {}".format(args.manifest))
        return 0

    centerline = track.build_centerline()
    origin_offset = (offset, track.plane_size[1] / 2.0)

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
