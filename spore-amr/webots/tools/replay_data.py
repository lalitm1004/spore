"""What a replay is drawn from: the recorded poses and the map they sit on.

Shared by `make_replay.py` (flat) and `make_replay3d.py` (orbitable). They had
one copy each of both loaders, identical to the character, including the
centimetre-to-metre-about-the-centre shift that makes the map share the poses'
frame -- so a fix to one would have silently missed the other.
"""
import csv
import json


def load_poses(path, stride=1, decimals=None):
    """Frames of {robot: [x, y, theta]}, in recorded order.

    Rows arrive grouped by sample time; anything out of order simply lands in
    the frame nearest its own timestamp, so a truncated recording still plays.
    `stride` thins the frames; `decimals` rounds them, for a smaller page.
    """
    frames = {}
    with path.open() as handle:
        for row in csv.DictReader(handle):
            t = round(float(row["t"]), 3)
            pose = [float(row["x"]), float(row["y"]), float(row["theta"])]
            if decimals is not None:
                pose = [round(v, decimals) for v in pose]
            frames.setdefault(t, {})[row["robot"]] = pose
    times = sorted(frames)[::stride]
    return times, [frames[t] for t in times]


def load_graph(path):
    """Nodes and lanes in world metres, sharing the poses' frame.

    `warehouse.json` is in centimetres with the origin at a corner; the
    supervisor reports metres about the centre, so the map is shifted to match
    rather than the poses -- the poses are the ground truth.
    """
    document = json.loads(path.read_text())
    width = float(document["dimensions"]["width"])
    height = float(document["dimensions"]["height"])
    nodes = {
        int(n["id"]): {
            "x": round(float(n["position"]["x"]) / 100.0 - width / 200.0, 4),
            "y": round(float(n["position"]["y"]) / 100.0 - height / 200.0, 4),
            "kind": n.get("node_type", "PT"),
        }
        for n in document["nodes"]
    }
    edges = [[int(e["a"]), int(e["b"])] for e in document["edges"]]
    return nodes, edges
