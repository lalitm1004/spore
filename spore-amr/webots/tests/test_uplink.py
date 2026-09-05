"""The companion's link to the network layer.

Their client is asynchronous by design -- status queued, commands arriving on a
reader thread -- and a junction needs an answer now. This is where those two
meet, so the cases that matter are the ones where an answer is late, absent, or
is the node the robot is already standing on.
"""

from robot.uplink import Uplink
from temp_network_interface.messages import NetworkToRobot, RobotState


class FakeClient:
    """Stands in for `NetworkClient`: same `send`/`recv`/`state` surface."""

    def __init__(self, deliveries=()):
        self.state = RobotState()
        self.sent = []
        self._deliveries = list(deliveries)

    def send(self, message):
        self.sent.append(message)
        return True

    def recv(self, timeout=None):
        if not self._deliveries:
            return None                       # a timeout, as the real one does
        command = self._deliveries.pop(0)
        if command is None:
            return None
        self.state = RobotState.from_command(command)
        return command


def goal(node, timestamp=1):
    return NetworkToRobot(target_node_id=node, timestamp=timestamp)


def test_a_status_is_sent_with_the_position_it_was_given():
    client = FakeClient([goal(40)])
    Uplink(client).report(bot_id=3, region_id=7, node_id=12, timestamp=420000)

    (sent,) = client.sent
    assert sent.bot_id == 3
    assert sent.region_id == 7
    assert sent.latest_node_id == 12
    assert sent.timestamp == 420000


def test_the_standing_destination_is_returned():
    client = FakeClient([goal(40)])

    assert Uplink(client).report(1, 1, 12, 0) == 40


def test_a_destination_already_held_needs_no_new_command():
    """The client preserves the last command, so a robot part-way along a route
    must not have to wait for the network to repeat itself at every marker."""
    client = FakeClient()
    client.state = RobotState.from_command(goal(40))

    assert Uplink(client).report(1, 1, 12, 0) == 40
    assert client.sent                      # still reported where it is


def test_arriving_waits_for_the_next_destination():
    """A goal equal to where we stand is an arrival. The network layer
    reconciles that from this very status and sends somewhere new -- returning
    "nowhere to go" here would stop the robot for good the first time it
    arrived anywhere."""
    client = FakeClient([goal(90)])
    client.state = RobotState.from_command(goal(12))   # already at node 12

    assert Uplink(client).report(1, 1, 12, 0) == 90


def test_no_destination_and_no_answer_is_nothing():
    """Bounded: the robot sits still and the firmware's junction timeout drives
    it on. It must never hold a lane because a service is slow."""
    assert Uplink(FakeClient()).report(1, 1, 12, 0) is None


def test_a_timed_out_wait_gives_up_rather_than_waiting_again():
    """`recv` returning None means the whole wait elapsed. Going round again
    would multiply the bound by however many times it happened."""
    client = FakeClient([None, goal(55)])

    assert Uplink(client, wait_s=0.01).report(1, 1, 12, 0) is None


def test_a_destination_behind_an_irrelevant_command_is_still_found():
    """Commands that are not a usable destination -- a repeat of the node we
    stand on -- cost a turn of the loop, not the answer."""
    client = FakeClient([goal(12), goal(55)])

    assert Uplink(client, wait_s=5.0).report(1, 1, 12, 0) == 55


def test_battery_is_reported_because_the_schema_requires_it():
    client = FakeClient([goal(40)])
    Uplink(client, battery_percent=62.5).report(1, 1, 12, 0)

    assert client.sent[0].telemetry.battery.percentage == 62.5
