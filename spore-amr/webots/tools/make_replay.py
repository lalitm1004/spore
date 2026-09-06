"""Turn a recorded run into something you can actually watch.

Webots can export an animation or a movie, and neither helps here. The
animation replays against the world, which means a browser loading 83 marker
tiles at 1024x1024 and an 8192x4096 floor -- 482 MB of texture, and the reason
the streaming viewer stalls at 97%. The movie needs live rendering, which is
what makes a run slow in the first place.

So this draws the run instead of replaying the world. `robot/supervisor.py
--replay` records each robot's true pose a few times a second, which costs
nothing under `--no-rendering`; this pairs those poses with the lane graph and
emits one self-contained HTML file. No textures, no server, no Webots: open it
in any browser, scrub the timeline, watch the fleet.

It is ground truth, not the robots' belief -- the poses come from the
supervisor, which is the only thing in the system that knows where a robot
really is. What you are watching is what actually happened.

Usage:
    uv run python -m tools.make_replay            # out/replay.csv -> out/replay.html
    uv run python -m tools.make_replay --open
"""

import argparse
import json
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))


from tools.replay_data import load_graph, load_poses  # noqa: E402


PAGE = """<!doctype html>
<meta charset="utf-8">
<title>Fleet replay</title>
<style>
  :root {
    --ink: #e8e6e3; --dim: #8b9199; --bg: #14171a; --panel: #1c2126;
    --lane: #2f363d; --line: #3d464f;
  }
  * { box-sizing: border-box; }
  body { margin: 0; background: var(--bg); color: var(--ink);
         font: 13px/1.5 ui-monospace, SFMono-Regular, Menlo, monospace; }
  header { padding: 10px 14px; background: var(--panel);
           border-bottom: 1px solid var(--lane); display: flex;
           align-items: center; gap: 14px; flex-wrap: wrap; }
  h1 { font-size: 13px; margin: 0; font-weight: 600; letter-spacing: .02em; }
  .clock { color: var(--dim); font-variant-numeric: tabular-nums; }
  button { background: #2b333b; color: var(--ink); border: 1px solid var(--line);
           border-radius: 5px; padding: 4px 12px; cursor: pointer; font: inherit; }
  button:hover { background: #353f49; }
  input[type=range] { flex: 1; min-width: 220px; accent-color: #6aa9ff; }
  canvas { display: block; width: 100%; height: calc(100vh - 52px); }
  .legend { color: var(--dim); }
  .legend b { color: var(--ink); font-weight: 600; }
</style>
<header>
  <h1>Fleet replay</h1>
  <button id="play">pause</button>
  <input type="range" id="scrub" min="0" value="0">
  <span class="clock" id="clock"></span>
  <span class="legend"><b>__NROBOTS__</b> robots &middot; <b>__NNODES__</b> nodes
    &middot; ground truth from the supervisor</span>
</header>
<canvas id="c"></canvas>
<script>
const NODES = __NODES__, EDGES = __EDGES__, TIMES = __TIMES__, FRAMES = __FRAMES__;
const NAMES = __NAMES__;
// Distinct hues rather than a gradient: you are tracking individuals, and
// neighbouring shades of one colour are exactly what you cannot tell apart.
const COLORS = ["#ff6b6b","#ffd93d","#6aa9ff","#51cf66","#ff9f43",
                "#c77dff","#4ecdc4","#f06595","#a0e548","#74c0fc"];

const canvas = document.getElementById("c"), ctx = canvas.getContext("2d");
let frame = 0, playing = true;

const xs = Object.values(NODES).map(n => n.x), ys = Object.values(NODES).map(n => n.y);
const bounds = {x0: Math.min(...xs) - 1, x1: Math.max(...xs) + 1,
                y0: Math.min(...ys) - 1, y1: Math.max(...ys) + 1};

function fit() {
  const dpr = window.devicePixelRatio || 1;
  canvas.width = canvas.clientWidth * dpr;
  canvas.height = canvas.clientHeight * dpr;
  const sx = canvas.width / (bounds.x1 - bounds.x0);
  const sy = canvas.height / (bounds.y1 - bounds.y0);
  const s = Math.min(sx, sy);
  // World +y is up; canvas +y is down. Flipping here rather than at every
  // draw call keeps the geometry below in world coordinates.
  return {s, ox: (canvas.width - s * (bounds.x1 - bounds.x0)) / 2,
          oy: (canvas.height + s * (bounds.y1 - bounds.y0)) / 2};
}
let view = fit();
addEventListener("resize", () => { view = fit(); draw(); });
const px = x => view.ox + (x - bounds.x0) * view.s;
const py = y => view.oy - (y - bounds.y0) * view.s;

function draw() {
  ctx.fillStyle = "#14171a";
  ctx.fillRect(0, 0, canvas.width, canvas.height);

  ctx.strokeStyle = "#2f363d";
  ctx.lineWidth = Math.max(1, view.s * 0.02);
  ctx.beginPath();
  for (const [a, b] of EDGES) {
    const A = NODES[a], B = NODES[b];
    if (!A || !B) continue;
    ctx.moveTo(px(A.x), py(A.y)); ctx.lineTo(px(B.x), py(B.y));
  }
  ctx.stroke();

  for (const [id, n] of Object.entries(NODES)) {
    ctx.fillStyle = n.kind === "CH" ? "#1ae5e5" : n.kind === "PK" ? "#b44dff"
                  : n.kind === "TR" ? "#2a80ff" : n.kind === "YI" ? "#ff2a1a"
                  : "#39424b";
    ctx.beginPath();
    ctx.arc(px(n.x), py(n.y), Math.max(1.5, view.s * 0.05), 0, 7);
    ctx.fill();
  }

  const poses = FRAMES[frame] || {};
  NAMES.forEach((name, i) => {
    const p = poses[name];
    if (!p) return;
    const [x, y, th] = p, r = Math.max(3, view.s * 0.16);
    ctx.save();
    ctx.translate(px(x), py(y));
    ctx.rotate(-th);                    // world CCW, canvas y flipped
    ctx.fillStyle = COLORS[i % COLORS.length];
    // A wedge, not a dot: heading is half of what a robot is doing, and a
    // circle hides which way it is about to leave a junction.
    ctx.beginPath();
    ctx.moveTo(r * 1.5, 0);
    ctx.lineTo(-r, r * 0.9);
    ctx.lineTo(-r, -r * 0.9);
    ctx.closePath();
    ctx.fill();
    ctx.restore();
  });

  document.getElementById("clock").textContent =
    "t = " + (TIMES[frame] || 0).toFixed(1) + " s  /  " +
    (TIMES[TIMES.length - 1] || 0).toFixed(1) + " s";
}

const scrub = document.getElementById("scrub");
scrub.max = Math.max(0, TIMES.length - 1);
scrub.addEventListener("input", () => {
  frame = +scrub.value; playing = false;
  document.getElementById("play").textContent = "play";
  draw();
});
document.getElementById("play").addEventListener("click", e => {
  playing = !playing;
  e.target.textContent = playing ? "pause" : "play";
});

// Played back at the rate it was recorded, so what you see takes as long as
// it took: a robot held at a junction should feel held.
const PERIOD = TIMES.length > 1 ? (TIMES[1] - TIMES[0]) * 1000 : 100;
setInterval(() => {
  if (!playing || !TIMES.length) return;
  frame = (frame + 1) % TIMES.length;
  scrub.value = frame;
  draw();
}, Math.max(16, PERIOD));

draw();
</script>
"""


def build(times, frames, nodes, edges, names):
    return (PAGE
            .replace("__NODES__", json.dumps(nodes, separators=(",", ":")))
            .replace("__EDGES__", json.dumps(edges, separators=(",", ":")))
            .replace("__TIMES__", json.dumps(times, separators=(",", ":")))
            .replace("__FRAMES__", json.dumps(frames, separators=(",", ":")))
            .replace("__NAMES__", json.dumps(names, separators=(",", ":")))
            .replace("__NROBOTS__", str(len(names)))
            .replace("__NNODES__", str(len(nodes))))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--replay", type=pathlib.Path,
                        default=ROOT / "out" / "replay.csv")
    parser.add_argument("--map", type=pathlib.Path,
                        default=ROOT / "config" / "warehouse.json")
    parser.add_argument("--out", type=pathlib.Path,
                        default=ROOT / "out" / "replay.html")
    parser.add_argument("--open", action="store_true",
                        help="open the result in a browser")
    args = parser.parse_args(argv)

    if not args.replay.exists():
        print("no recording at {} -- run the fleet first".format(args.replay))
        return 1

    times, frames = load_poses(args.replay, decimals=4)
    if not times:
        print("{} is empty".format(args.replay))
        return 1

    nodes, edges = load_graph(args.map)
    names = sorted({name for frame in frames for name in frame})
    args.out.write_text(build(times, frames, nodes, edges, names))

    print("{}  --  {} robots, {} frames, {:.0f} s of run, {:.1f} MB".format(
        args.out, len(names), len(times), times[-1] - times[0],
        args.out.stat().st_size / 1e6))
    if args.open:
        import webbrowser
        webbrowser.open(args.out.resolve().as_uri())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
