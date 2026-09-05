"""Render an SVG to PNG, using whatever this machine has.

Written because the warehouse layout tool emits `warehouse_map.svg` and the
simulator needs a raster to paste on the floor. There is no one SVG renderer
that is present everywhere, so this tries them in order of fidelity and says
which one it used -- a demo that only runs where somebody happened to `brew
install` something is a demo that fails on a teammate's laptop.

    rsvg-convert   the good one: full SVG, correct text and fonts
    qlmanage       macOS Quick Look, always present, no size control
    cairosvg       pure Python, but wants a system libcairo macOS lacks
    builtin        this project's own reader, in tools/track/svgfloor.py

The last is deliberately not general. It draws rects, lines and circles and
ignores everything else, which is exactly what `warehouse_map.svg` contains --
43 rects, 952 lines, 886 circles, no paths, no transforms, no gradients. For
that one file it is exact; for an arbitrary SVG it is not a renderer.

    uv run python -m tools.svg2png in.svg out.png --width 4096
"""

import argparse
import pathlib
import shutil
import subprocess
import sys
import tempfile


class Unrendered(Exception):
    """No backend could render the file."""


def _rsvg(svg: pathlib.Path, png: pathlib.Path, width: int, height=None) -> bool:
    binary = shutil.which("rsvg-convert")
    if binary is None:
        return False
    command = [binary, "-w", str(width)]
    if height:
        command += ["-h", str(height)]
    command += [str(svg), "-o", str(png)]
    return subprocess.run(command, capture_output=True).returncode == 0


def _qlmanage(svg: pathlib.Path, png: pathlib.Path, width: int, height=None) -> bool:
    """macOS Quick Look. Always there, but it only takes one size hint and
    writes `<name>.png` into a directory of its choosing."""
    binary = shutil.which("qlmanage")
    if binary is None:
        return False
    with tempfile.TemporaryDirectory() as directory:
        result = subprocess.run(
            [binary, "-t", "-s", str(width), "-o", directory, str(svg)],
            capture_output=True)
        if result.returncode != 0:
            return False
        produced = list(pathlib.Path(directory).glob("*.png"))
        if not produced:
            return False
        png.write_bytes(produced[0].read_bytes())
    return True


def _cairosvg(svg: pathlib.Path, png: pathlib.Path, width: int, height=None) -> bool:
    try:
        import cairosvg
    except Exception:
        return False
    try:
        cairosvg.svg2png(url=str(svg), write_to=str(png), output_width=width,
                         output_height=height)
    except Exception:
        return False
    return True


def _builtin(svg: pathlib.Path, png: pathlib.Path, width: int, height=None) -> bool:
    """This project's own reader. Only knows rects, lines and circles."""
    try:
        from tools.track.svgfloor import render_svg
    except Exception:
        return False
    try:
        render_svg(svg, width=width, height=height).save(png)
    except Exception:
        return False
    return True


BACKENDS = (
    ("rsvg-convert", _rsvg),
    ("qlmanage", _qlmanage),
    ("cairosvg", _cairosvg),
    ("builtin", _builtin),
)


def convert(svg_path, png_path, width=2048, height=None, prefer=None):
    """Render `svg_path` to `png_path`. Returns the backend that did it."""
    svg = pathlib.Path(svg_path)
    png = pathlib.Path(png_path)
    if not svg.exists():
        raise FileNotFoundError(svg)
    png.parent.mkdir(parents=True, exist_ok=True)

    backends = BACKENDS
    if prefer:
        backends = tuple(b for b in BACKENDS if b[0] == prefer) or BACKENDS

    tried = []
    for name, backend in backends:
        if backend(svg, png, width, height):
            return name
        tried.append(name)

    raise Unrendered(
        "no backend rendered {}: tried {}. Install one with "
        "`brew install librsvg`".format(svg, ", ".join(tried)))


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("svg", type=pathlib.Path)
    parser.add_argument("png", type=pathlib.Path)
    parser.add_argument("--width", type=int, default=2048)
    parser.add_argument("--height", type=int, default=None)
    parser.add_argument("--prefer", choices=[n for n, _ in BACKENDS],
                        help="force one backend rather than trying in order")
    args = parser.parse_args(argv)

    try:
        used = convert(args.svg, args.png, args.width, args.height, args.prefer)
    except (Unrendered, FileNotFoundError) as error:
        print(error, file=sys.stderr)
        return 1

    from PIL import Image

    with Image.open(args.png) as image:
        print("{} -> {} ({}x{}, via {})".format(
            args.svg, args.png, image.width, image.height, used))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
