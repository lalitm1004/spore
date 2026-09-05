"""The slow tier: the real simulator, the real camera, the whole fleet.

WHAT IT PROVES THAT NOTHING ELSE CAN
    Every other tier hands the network layer a node id. This one makes a robot
    *read* one — a QR tile under a downward camera, decoded by `cv2`, on a floor
    the robot is following a line across. The step before a report is the only
    step no other test covers, and it is the step that turns a warehouse into a
    node id in the first place.

    It also runs the fleet at the scale it is meant for: ten robots, ten bots,
    one region leader, and all of them contending for the same corridors.

WHY IT IS A TIER OF ITS OWN
    A container fleet is up in seconds. This one pays for a simulator, a world
    of 881 nodes, ten synchronized controllers that must all attach before time
    advances, and physics. `MODE=fast` and `RENDERING=off` are the levers that
    make it minutes rather than hours (`--no-rendering` keeps camera sensors —
    they are a separate offscreen pass — and takes sim CPU from 907% to 5.65%).

    So it is `-m webots`, it is not in the default run, and it is not on the
    five-minute budget the container tier is held to.

HOW TO RUN IT
    webots/fleet.sh up  &&  uv run pytest tests -q -m webots

    against a fleet you brought up yourself, or

    WEBOTS_TIER=1 uv run pytest tests -q -m webots

    to let it start and stop one. It will not start a simulator without that,
    because a command-line `-m` replaces the default deselection rather than
    adding to it, and `-m "not docker"` is a reasonable thing to type.

    Or watch instead of asserting: `spore-amr/webots/fleet.sh view`.

WHAT IT DOES NOT PROVE
    Timings, on any machine whose architecture differs from the Webots image's.
    The published image is amd64; on an arm64 host every number here is
    emulation's rather than the fleet's, which is why nothing below asserts on
    wall-clock or throughput. It asserts that the *path* works.
"""
from __future__ import annotations

import json
import os
import pathlib
import re
import subprocess
import time

import pytest

pytestmark = pytest.mark.webots

FLEET = pathlib.Path(__file__).resolve().parents[2] / "webots"
#: Simulated seconds. Robots spawn in charging bays and need to reach a marker,
#: contend for the corridor out, and be answered — several times over.
MISSION_SECONDS = 900
#: Wall seconds to wait for the assertions below. Generous: this is a tier whose
#: whole point is that it is slow, and a host under emulation is slower again.
PATIENCE = 900


def fleet_sh(*args, timeout: int = 600) -> subprocess.CompletedProcess:
    """Everything goes through `fleet.sh`.

    It is the one place that knows which compose files to use -- the GPU overlay
    is included only where `/dev/dri` exists -- and which environment a run
    needs. A test that rebuilt that invocation would be a second copy of it, and
    the first thing to go stale.
    """
    return subprocess.run(
        ["./fleet.sh", *args], cwd=FLEET, capture_output=True, text=True,
        env={**os.environ, "MISSION_DURATION": str(MISSION_SECONDS),
             "RENDERING": "off", "MODE": "fast"},
        timeout=timeout)


def logs(lines: int = 4000) -> str:
    return fleet_sh("dump", str(lines), timeout=180).stdout


def fleet_state() -> dict:
    """What the fleet believes about where its robots are.

    `fleet.sh where` is the implementation; this only parses it. Writing the
    gRPC call again here would be a second copy of it, and the operator-facing
    one is where anyone debugging a run will look first -- so it is the one that
    has to stay right.
    """
    out = fleet_sh("where", "json", timeout=120).stdout
    line = next((ln for ln in out.splitlines() if ln.startswith("{")), "")
    return json.loads(line) if line else {}


def wait_for(predicate, what: str, timeout: int = PATIENCE, step: int = 15):
    """Poll until it is true, or say what we were waiting for."""
    deadline = time.monotonic() + timeout
    last = None
    while time.monotonic() < deadline:
        try:
            last = predicate()
        except Exception:                         # a container still starting
            last = None
        if last:
            return last
        time.sleep(step)
    pytest.fail(f"timed out after {timeout}s waiting for {what} (last: {last!r})")


@pytest.fixture(scope="module")
def fleet():
    """One real fleet for the whole module. Bringing it up is the expensive
    part, and nothing here needs a fresh one."""
    if subprocess.run(["docker", "info"], capture_output=True).returncode != 0:
        pytest.skip("no Docker daemon reachable")
    if not (FLEET / "compose.fleet.yml").exists():
        pytest.skip("no generated fleet; run `webots/fleet.sh gen`")

    # Starting a simulator is not something to do by accident. `addopts`
    # deselects this tier, but a command-line `-m` *replaces* that rather than
    # adding to it -- so `-m "not docker"`, which is a reasonable thing to type,
    # would otherwise bring up ten robots and a world of 881 nodes without
    # anyone asking for it.
    #
    # So: run against a fleet that is already up, and only start one when told
    # to in as many words.
    already_up = "webots-sim-1" in subprocess.run(
        ["docker", "ps", "--format", "{{.Names}}"],
        capture_output=True, text=True).stdout

    if already_up:
        yield                       # someone else's fleet; leave it running
        return

    if os.environ.get("WEBOTS_TIER") != "1":
        pytest.skip("set WEBOTS_TIER=1 to let this tier start a simulator, "
                    "or bring one up yourself with `webots/fleet.sh up`")

    up = fleet_sh("up")
    if up.returncode != 0:
        pytest.skip(f"the fleet would not start: {up.stderr[-400:]}")
    try:
        yield
    finally:
        fleet_sh("down", timeout=300)


def test_the_fleet_elects_a_leader(fleet):
    """Ten bots that have never met settle on one leader per region, which is
    §4.1 and §5 running for real rather than on a synthetic roster."""
    wait_for(lambda: "became leader" in logs(2000), "a leader to be elected")


def test_a_robot_reads_a_qr_code_and_the_fleet_learns_where_it_is(fleet):
    """**The thing no other tier can prove.**

    A tile under a camera, decoded by `cv2`, becomes a node id in a roster that
    every bot in the region holds. Between those two points is the whole
    location interface: the marker event, the companion, `RobotToNetwork`, the
    slot, the run loop, the heartbeat and the leader's ack.

    For the entire life of this fleet the answer here was zero.
    """
    def someone_knows_where_they_are():
        state = fleet_state()
        return [p for p in state.get("roster", []) if p["node"]]

    known = wait_for(someone_knows_where_they_are,
                     "a robot's QR read to reach the roster")
    assert known, "no bot in the region knows where any robot is"
    for peer in known:
        assert peer["trail"] and peer["trail"][0] == peer["node"], \
            "node_trail must lead with the node it is a trail from"


def test_robots_claim_the_nodes_they_are_standing_on(fleet):
    """Reservations against real positions. This is the point of the whole
    location interface: with `latest_node_id` at zero the claim path is inert,
    because a bot that does not know where it is withdraws rather than claims.
    """
    claims = wait_for(lambda: fleet_state().get("claims"),
                      "a bot to claim the node its robot is on")
    assert claims
    nodes = [node for _, node in claims]
    assert all(node > 0 for node in nodes), f"a claim on node 0: {claims}"
    assert len(nodes) == len(set(nodes)), \
        f"two bots claimed one node at once: {claims}"


def test_a_robot_is_answered_at_every_marker_it_reaches(fleet):
    """Guarantee 1, against a real camera. A robot that reaches a node and is
    told nothing stands there for the rest of the shift, because it only asks
    again on reaching the next node — and it will not reach one.
    """
    seen = wait_for(lambda: (lambda text: text if "MARKER" in text else None)(logs()),
                    "a robot to reach a marker")

    asked = len(re.findall(r"<- MARKER", seen))
    answered = len(re.findall(r"-> (TURN|HOLD)", seen))
    assert asked, "no robot reached a marker"
    assert answered, f"{asked} markers reached and not one answered"


def test_no_robot_is_left_with_nowhere_to_go(fleet):
    """Every charging and parking bay on this floor is a degree-1 spur, and the
    fleet spawns in them. A robot offered no exit never leaves the bay it
    started in — which is what happened, for every run there has ever been,
    until the arrival lane stopped being excluded at a dead end.
    """
    assert "nowhere to go from here" not in logs(), \
        "a robot was offered no exit; see Graph.exits_from"
