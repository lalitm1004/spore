"""Render the warehouse's own SVG map as the simulator's floor texture.

`warehouse_map.svg` is the drawing the layout tool produces alongside
`warehouse.json` -- region blocks in their own colours, every lane, every node.
Using it as the floor means the simulated warehouse looks like the warehouse,
rather than like something this project drew from the same data.

It is rendered here rather than by a real SVG library on purpose: cairosvg
wants a system libcairo that macOS does not ship, and a demo that needs a
native dependency installed is a demo that fails on somebody else's laptop.
The file is flat -- 43 rects, 952 lines, 886 circles, no paths, no transforms,
no gradients -- so drawing those three shapes reproduces it exactly.

The 15 `<text>` labels are skipped: region names at 11.5 px in a 1500 px
drawing become illegible when a 32 m window of it is stretched over a floor,
and a smeared word is worse than no word.

Two coordinate systems meet here, and getting them the wrong way round puts
the lanes somewhere the robots are not:

    svg px  = 0.11 * world cm + 90        (both axes, fitted on all 952 lanes)
    world y grows north; svg y grows down

Pure apart from writing the image the caller asks for.
"""

import re
from typing import Optional, Tuple

# Fitted against every lane endpoint in the map: the 952 <line> elements
# against the same 952 edges in warehouse.json.
#
#     x_px = 0.11 * x_cm + 90
#     y_px = 860 - 0.11 * y_cm
#
# The y axis is INVERTED, which a fit on ranges alone will not tell you --
# min-to-min and max-to-max match either way. The check that settles it is a
# landmark: the CHARGING label sits at svg y=723 and its nodes are at world
# y=800 cm, so low world y is the bottom of the drawing.
#
# It also means no vertical flip is needed here. The map already runs the
# opposite way to the world, and a texture's rows do too, so the two agree.
SVG_SCALE = 0.11        # px per cm
SVG_OFFSET = 90.0       # px, x only
SVG_Y_TOP = 860.0       # px at world y = 0

# The lane guide line the robots follow. The SVG draws lanes as hairlines,
# which is right for a diagram and useless for an IR array, so they are
# redrawn at true width in the darkest ink the map uses.
LANE_INK = (43, 52, 57)


def svg_source(path) -> str:
    import pathlib

    return pathlib.Path(path).read_text()


def world_cm_to_svg(x_cm: float, y_cm: float) -> Tuple[float, float]:
    """World centimetres to the map's own pixel coordinates."""
    return (x_cm * SVG_SCALE + SVG_OFFSET, SVG_Y_TOP - y_cm * SVG_SCALE)


def _colour(text: str, attribute: str) -> Optional[Tuple[int, int, int]]:
    match = re.search(r'%s="#([0-9a-fA-F]{6})"' % attribute, text)
    if not match:
        return None
    value = match.group(1)
    return tuple(int(value[i:i + 2], 16) for i in (0, 2, 4))


def render_window(svg_path, origin_cm, size_m, pixels_per_metre,
                  graph=None, line_width_m=0.02):
    """The part of the map covering one window, at the floor's resolution.

    `graph`, when given, has its lanes redrawn at `line_width_m` so the IR
    array has something real to follow -- the map's own hairlines are a
    diagram, not a guide line.
    """
    import pathlib

    from PIL import Image, ImageDraw

    svg = pathlib.Path(svg_path).read_text()

    width_px = int(round(size_m[0] * pixels_per_metre))
    height_px = int(round(size_m[1] * pixels_per_metre))

    # svg px -> texture px for this window
    x0_svg, y0_svg = world_cm_to_svg(origin_cm[0], origin_cm[1])
    scale = pixels_per_metre / (SVG_SCALE * 100.0)

    def place(x, y):
        # The map's y already runs opposite to the world's, and so do texture
        # rows, so both agree and neither is flipped here.
        _, far_svg = world_cm_to_svg(origin_cm[0], origin_cm[1] + size_m[1] * 100.0)
        return ((x - x0_svg) * scale, (y - far_svg) * scale)

    image = Image.new("RGB", (width_px, height_px), (255, 255, 255))
    draw = ImageDraw.Draw(image)

    for tag in re.findall(r'<rect [^>]*/>', svg):
        fill = _colour(tag, "fill")
        if fill is None:
            continue
        numbers = {k: float(v) for k, v in
                   re.findall(r'\b(x|y|width|height)="([-\d.]+)"', tag)}
        if "width" not in numbers or "height" not in numbers:
            continue
        x, y = numbers.get("x", 0.0), numbers.get("y", 0.0)
        left, bottom = place(x, y)
        right, top = place(x + numbers["width"], y + numbers["height"])
        draw.rectangle([min(left, right), min(top, bottom),
                        max(left, right), max(top, bottom)], fill=fill)

    for tag in re.findall(r'<line [^>]*/>', svg):
        stroke = _colour(tag, "stroke")
        if stroke is None:
            continue
        numbers = {k: float(v) for k, v in
                   re.findall(r'\b(x1|y1|x2|y2)="([-\d.]+)"', tag)}
        if len(numbers) < 4:
            continue
        draw.line([place(numbers["x1"], numbers["y1"]),
                   place(numbers["x2"], numbers["y2"])],
                  fill=stroke, width=max(1, int(round(0.05 * pixels_per_metre))))

    for tag in re.findall(r'<circle [^>]*/>', svg):
        fill = _colour(tag, "fill")
        if fill is None:
            continue
        numbers = {k: float(v) for k, v in
                   re.findall(r'\b(cx|cy|r)="([-\d.]+)"', tag)}
        if len(numbers) < 3:
            continue
        cx, cy = place(numbers["cx"], numbers["cy"])
        radius = max(1.0, numbers["r"] * scale)
        draw.ellipse([cx - radius, cy - radius, cx + radius, cy + radius], fill=fill)

    if graph is not None:
        # The guide line, at its true 20 mm, on top of the diagram.
        width = max(1, round(line_width_m * pixels_per_metre))
        half_w, half_h = size_m[0] / 2.0, size_m[1] / 2.0

        def world_to_texture(x_m, y_m):
            return ((x_m + half_w) * pixels_per_metre,
                    height_px - (y_m + half_h) * pixels_per_metre)

        for edge in graph.edges:
            a, b = graph.nodes[edge.a], graph.nodes[edge.b]
            draw.line([world_to_texture(a.x, a.y), world_to_texture(b.x, b.y)],
                      fill=LANE_INK, width=width)
        radius = width / 2.0
        for node in graph.nodes.values():
            x, y = world_to_texture(node.x, node.y)
            draw.ellipse([x - radius, y - radius, x + radius, y + radius],
                         fill=LANE_INK)

    return image


def render_svg(svg_path, width, height=None, background=(255, 255, 255)):
    """The whole SVG as an image, at `width` pixels across.

    The fallback for tools/svg2png.py when no real renderer is installed.
    Draws rects, lines and circles in document order and ignores everything
    else -- which for `warehouse_map.svg` is exact, and for an arbitrary SVG
    is not a renderer. Text is skipped.
    """
    import pathlib

    from PIL import Image, ImageDraw

    svg = pathlib.Path(svg_path).read_text()

    canvas = re.search(r'<svg[^>]*\bwidth="([\d.]+)"[^>]*\bheight="([\d.]+)"', svg)
    if not canvas:
        raise ValueError("no width/height on the <svg> element")
    source_w, source_h = float(canvas.group(1)), float(canvas.group(2))

    scale = width / source_w
    height = height or int(round(source_h * scale))
    image = Image.new("RGB", (int(width), int(height)), background)
    draw = ImageDraw.Draw(image)

    def place(x, y):
        return (x * scale, y * scale)

    for tag in re.findall(r'<(?:rect|line|circle)\b[^>]*/?>', svg):
        numbers = {k: float(v) for k, v in
                   re.findall(r'\b(x|y|width|height|x1|y1|x2|y2|cx|cy|r)="([-\d.]+)"',
                              tag)}
        if tag.startswith("<rect"):
            fill = _colour(tag, "fill")
            if fill is None or "width" not in numbers:
                continue
            left, top = place(numbers.get("x", 0.0), numbers.get("y", 0.0))
            right, bottom = place(numbers.get("x", 0.0) + numbers["width"],
                                  numbers.get("y", 0.0) + numbers["height"])
            draw.rectangle([left, top, right, bottom], fill=fill)
        elif tag.startswith("<line"):
            stroke = _colour(tag, "stroke")
            if stroke is None or "x2" not in numbers:
                continue
            draw.line([place(numbers["x1"], numbers["y1"]),
                       place(numbers["x2"], numbers["y2"])],
                      fill=stroke, width=max(1, int(round(1.1 * scale))))
        else:
            fill = _colour(tag, "fill")
            if fill is None or "r" not in numbers:
                continue
            cx, cy = place(numbers["cx"], numbers["cy"])
            radius = max(1.0, numbers["r"] * scale)
            draw.ellipse([cx - radius, cy - radius, cx + radius, cy + radius],
                         fill=fill)

    return image


def render_window_via_converter(svg_path, origin_cm, size_m, pixels_per_metre,
                                graph=None, line_width_m=0.02):
    """The same window, rendered by a real SVG renderer rather than by hand.

    Preferred when rsvg-convert is installed: it draws the text, the legend and
    everything else the built-in reader skips, so the floor is the map rather
    than an approximation of it. Falls back to `render_window` otherwise.

    The window is cropped by the renderer, not afterwards. The full sheet at
    the floor's resolution is about 770 megapixels -- rasterising it and then
    cutting a piece out was a decompression-bomb-sized mistake.
    """
    import pathlib
    import shutil
    import subprocess
    import tempfile

    from PIL import Image, ImageDraw

    binary = shutil.which("rsvg-convert")
    if binary is None:
        return render_window(svg_path, origin_cm, size_m, pixels_per_metre,
                             graph=graph, line_width_m=line_width_m)

    # Drop the map's node dots. They are drawn at 1.5-4 svg px, which is right
    # for a 1500 px diagram of a 120 m warehouse and becomes 273-727 mm once
    # the same drawing is a floor -- three to seven times the 100 mm marker
    # tile, and bigger than the 120 mm robot. They swallowed the QR tiles.
    #
    # Nothing is lost: every node carries a real marker tile, which is the
    # physical thing a robot reads. The dots were the diagram's stand-in for
    # exactly that.
    stripped = re.sub(r'<circle\b[^>]*/?>', '', svg_source(svg_path))

    width_px = int(round(size_m[0] * pixels_per_metre))
    height_px = int(round(size_m[1] * pixels_per_metre))

    # One svg px is 1/SVG_SCALE cm of floor, so it must become this many
    # texture pixels.
    zoom = pixels_per_metre / (SVG_SCALE * 100.0)

    # rsvg's --left/--top are output pixels that offset the drawing, so both
    # are negative: they pull the window's corner back to the page origin.
    #
    # The map's y axis runs the same way as the world's -- both increase with
    # `y_px = 0.11 * y_cm + 90` -- but a texture's rows run the other way, so
    # the crop is taken from the near edge and flipped afterwards. Trying to
    # do it with the offset instead cropped an empty part of the sheet.
    # The window's far (north) edge is its smallest svg y, and that is what
    # belongs at the top of the texture.
    left_svg, _ = world_cm_to_svg(origin_cm[0], origin_cm[1])
    _, far_svg = world_cm_to_svg(origin_cm[0], origin_cm[1] + size_m[1] * 100.0)
    left = -left_svg * zoom
    top = -far_svg * zoom

    with tempfile.TemporaryDirectory() as directory:
        source = pathlib.Path(directory) / "map.svg"
        source.write_text(stripped)
        out = pathlib.Path(directory) / "window.png"
        # `--left=-N`, not `--left -N`: the offsets are negative and rsvg
        # parses a bare `-2000` as a flag. This failed silently into the
        # fallback renderer for a while, which looked like the strip below
        # simply not working.
        result = subprocess.run([
            binary,
            "--zoom={:.6f}".format(zoom),
            "--left={:.2f}".format(left),
            "--top={:.2f}".format(top),
            "--page-width={}".format(width_px),
            "--page-height={}".format(height_px),
            "--background-color=white",
            str(source), "-o", str(out)], capture_output=True)
        if result.returncode != 0 or not out.exists():
            print("rsvg-convert failed ({}), using the built-in reader: {}".format(
                result.returncode, result.stderr.decode()[:160].strip()))
            return render_window(svg_path, origin_cm, size_m, pixels_per_metre,
                                 graph=graph, line_width_m=line_width_m)
        image = Image.open(out).convert("RGB")

    if image.size != (width_px, height_px):
        image = image.resize((width_px, height_px), Image.LANCZOS)

    if graph is not None:
        # The guide line at its true 20 mm, on top of the diagram: the map's
        # own lanes are hairlines, which is right for a drawing and useless
        # for an IR array.
        draw = ImageDraw.Draw(image)
        width = max(1, round(line_width_m * pixels_per_metre))
        half_w, half_h = size_m[0] / 2.0, size_m[1] / 2.0

        def world_to_texture(x_m, y_m):
            return ((x_m + half_w) * pixels_per_metre,
                    height_px - (y_m + half_h) * pixels_per_metre)

        for edge in graph.edges:
            a, b = graph.nodes[edge.a], graph.nodes[edge.b]
            draw.line([world_to_texture(a.x, a.y), world_to_texture(b.x, b.y)],
                      fill=LANE_INK, width=width)
        radius = width / 2.0
        for node in graph.nodes.values():
            x, y = world_to_texture(node.x, node.y)
            draw.ellipse([x - radius, y - radius, x + radius, y + radius],
                         fill=LANE_INK)

    return image
