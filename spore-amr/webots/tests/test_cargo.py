"""The robot's half of a cargo job.

A job only advances because the robot says it did. The network layer moves the
goal from the collection node to the delivery node when the robot reports
CARGO/EN_ROUTE, and marks the job delivered when the robot stops reporting
CARGO at all. `robot/uplink.py` reported `Mission(idle=...)` unconditionally
for the whole life of the file, so every robot drove to its pickup node and was
answered "hold, you are at the goal" for the rest of the shift. Nothing logged
an error; the fleet just never completed a job.
"""

import pytest

from robot.companion import answer_junction
from robot.navigator import Navigator
from robot.network import Decision
from tools.track.graph import lattice


class Event:
    def __init__(self, **fields):
        self.name = "MARKER"
        self.fields = fields


def navigator_on_a_lattice():
    """A 3x3 lattice at 2 m spacing; ids run row-major, so 4 is the centre."""
    return Navigator(lattice(rows=3, columns=3, spacing=2.0))


class CargoNetwork:
    """A network layer that hands out one job, and remembers what it is told."""

    def __init__(self, pickup, dropoff, cargo_id="c-1"):
        self.mission = ""
        self.cargo_id = ""
        self.cargo_state = ""
        self.pickup, self.dropoff = pickup, dropoff
        self._cargo_id = cargo_id
        self.reports = []

    def ask(self, query):
        # Before pickup, send it to the pickup node; after, to the dropoff.
        if self.cargo_state in ("", "PICKUP"):
            return Decision(query_id=query.query_id, target_node_id=self.pickup,
                            mission="CARGO", cargo_id=self._cargo_id,
                            cargo_state="PICKUP")
        return Decision(query_id=query.query_id, target_node_id=self.dropoff,
                        mission="CARGO", cargo_id=self._cargo_id,
                        cargo_state="EN_ROUTE")

    def report(self, node_id, region_id, **kw):
        self.reports.append((node_id, self.mission, self.cargo_state))


def marker_at(node_id, heading=0.0):
    return Event(node=node_id, heading=heading, t=1.0)


def test_arriving_at_the_collection_node_picks_the_cargo_up():
    """The report that unsticks a job. Without it the network layer never moves
    the goal on and the robot is held at the pickup for ever."""
    navigator = navigator_on_a_lattice()
    navigator.arrived(3)
    network = CargoNetwork(pickup=4, dropoff=0)

    answer_junction(navigator, network, marker_at(4))

    assert network.cargo_state == "EN_ROUTE", "the robot did not report collecting"
    assert network.mission == "CARGO"
    assert (4, "CARGO", "EN_ROUTE") in network.reports


def test_arriving_at_the_delivery_node_finishes_the_job():
    """Delivery is two reports: DROPOFF while still carrying, then a mission
    that is no longer CARGO. The network layer reads the pair as done."""
    navigator = navigator_on_a_lattice()
    navigator.arrived(3)
    network = CargoNetwork(pickup=0, dropoff=4)
    network.mission, network.cargo_state = "CARGO", "EN_ROUTE"
    network.cargo_id = "c-1"

    answer_junction(navigator, network, marker_at(4))

    states = [r[2] for r in network.reports]
    assert "DROPOFF" in states, "never reported putting the cargo down"
    assert network.mission == "", "still claims to be carrying after delivering"
    assert network.cargo_state == ""


def test_a_node_that_is_not_the_goal_does_not_touch_the_cargo():
    navigator = navigator_on_a_lattice()
    navigator.arrived(3)
    network = CargoNetwork(pickup=5, dropoff=5)
    network.mission, network.cargo_state = "CARGO", "PICKUP"

    answer_junction(navigator, network, marker_at(4))

    assert network.cargo_state == "PICKUP", "collected cargo at the wrong node"


def test_an_answer_with_no_mission_leaves_the_job_alone():
    """Every WAIT and most PROCEEDs carry no mission. Reading that as "idle"
    would drop the job a robot is halfway through."""
    class Silent(CargoNetwork):
        def ask(self, query):
            return Decision(query_id=query.query_id, target_node_id=5)

    navigator = navigator_on_a_lattice()
    navigator.arrived(3)
    network = Silent(pickup=5, dropoff=5)
    network.mission, network.cargo_state, network.cargo_id = "CARGO", "PICKUP", "c-1"

    answer_junction(navigator, network, marker_at(4))

    assert network.mission == "CARGO"
    assert network.cargo_state == "PICKUP"
    assert network.cargo_id == "c-1"
