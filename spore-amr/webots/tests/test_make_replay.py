"""Turning a recording into a replay.

The parsing is what can silently go wrong: a run killed mid-sample, or rows
that do not arrive in a tidy order, should still produce something watchable
rather than a broken page.
"""

import json

from tools.make_replay import build, load_graph, load_poses

ROWS = """t,robot,x,y,theta
0.000,bot_01,-14.0000,-6.0000,1.5708
0.000,bot_02,-8.0000,-6.0000,1.5708
0.100,bot_01,-14.0000,-5.9000,1.5708
0.100,bot_02,-8.0000,-5.9000,1.5708
"""


def write(tmp_path, text=ROWS):
    path = tmp_path / "replay.csv"
    path.write_text(text)
    return path


def test_samples_are_grouped_into_frames(tmp_path):
    times, frames = load_poses(write(tmp_path))

    assert times == [0.0, 0.1]
    assert set(frames[0]) == {"bot_01", "bot_02"}
    assert frames[1]["bot_01"] == [-14.0, -5.9, 1.5708]


def test_a_recording_cut_mid_sample_still_replays(tmp_path):
    """The supervisor flushes per sample, so a killed run ends part-way
    through a frame. That frame is short, not broken."""
    cut = ROWS.rsplit("\n", 2)[0] + "\n"
    times, frames = load_poses(write(tmp_path, cut))

    assert times == [0.0, 0.1]
    assert set(frames[-1]) == {"bot_01"}


def test_rows_out_of_order_land_in_the_right_frame(tmp_path):
    shuffled = "t,robot,x,y,theta\n" + "\n".join(
        ROWS.strip().splitlines()[1:][::-1]) + "\n"
    times, frames = load_poses(write(tmp_path, shuffled))

    assert times == [0.0, 0.1]
    assert frames[0]["bot_01"] == [-14.0, -6.0, 1.5708]


def test_the_graph_is_converted_into_the_poses_own_frame(tmp_path):
    """Poses are world metres from the centre; the map is centimetres from a
    corner. Drawing them in different frames puts the robots off the lanes."""
    path = tmp_path / "warehouse.json"
    path.write_text(json.dumps({
        "dimensions": {"width": 1000, "height": 400},
        "nodes": [{"id": 1, "position": {"x": 0, "y": 0}, "node_type": "CH"}],
        "edges": [],
    }))

    nodes, _ = load_graph(path)

    assert nodes[1]["x"] == -5.0 and nodes[1]["y"] == -2.0
    assert nodes[1]["kind"] == "CH"


def test_the_page_is_self_contained(tmp_path):
    """No server, no Webots, no textures -- the whole point. A page that
    fetches anything is a page that will not open from disk."""
    times, frames = load_poses(write(tmp_path))
    nodes, edges = load_graph_stub()

    page = build(times, frames, nodes, edges, ["bot_01", "bot_02"])

    assert "__FRAMES__" not in page and "__NODES__" not in page
    for forbidden in ("fetch(", "src=", "href=", "XMLHttpRequest"):
        assert forbidden not in page


def load_graph_stub():
    return {1: {"x": 0.0, "y": 0.0, "kind": "PT"}}, []
