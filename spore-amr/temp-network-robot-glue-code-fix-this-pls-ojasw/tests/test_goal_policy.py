"""Assigning destinations: the policy that gives a robot somewhere to be.

`HoldPolicy` keeps a robot where it is, which exercises the round trip and
nothing else. This is the first policy that actually moves a fleet: each robot
is given a node far away and keeps it until it arrives, at which point
reconciliation drops the command and it is given another.

The map lives here because routing is the network layer's job. The robot is
told a destination, not a direction, and walks to it itself.
"""

import random

from temp_network_interface import Fleet, NetworkToRobot
from temp_network_interface.goal_policy import GoalPolicy

from .test_messages import status


class Ring:
    """A ring of `size` nodes, so hop distance is unambiguous."""

    def __init__(self, size):
        self.size = size
        self.nodes = {i: object() for i in range(size)}

    def neighbours(self, node):
        return [(node - 1) % self.size, (node + 1) % self.size]

    def far_nodes(self, start, minimum_hops=1):
        return sorted(n for n in range(self.size)
                      if n != start
                      and min((n - start) % self.size,
                              (start - n) % self.size) >= minimum_hops)


def policy(size=20, minimum_hops=5, seed=0):
    return GoalPolicy(Ring(size), minimum_hops=minimum_hops,
                      random=random.Random(seed))


def test_a_robot_with_no_goal_is_given_one():
    fleet = Fleet()
    reported = status(bot_id=1, latest_node_id=0)
    fleet.record_status(reported)

    (targeted,) = policy().on_status(fleet, reported)

    assert targeted.bot_id == 1
    assert isinstance(targeted.command, NetworkToRobot)


def test_the_goal_is_actually_far_away():
    """"Far" is in hops along the lanes. A destination two nodes down the
    aisle is not a mission, it is a nudge."""
    fleet = Fleet()
    ring = Ring(20)
    goals = set()
    for bot in range(1, 30):
        reported = status(bot_id=bot, latest_node_id=0)
        fleet.record_status(reported)
        (targeted,) = GoalPolicy(ring, minimum_hops=5,
                                 random=random.Random(bot)).on_status(fleet, reported)
        goals.add(targeted.command.target_node_id)

    assert goals <= set(ring.far_nodes(0, minimum_hops=5))


def test_a_robot_already_working_towards_a_goal_is_left_alone():
    """Re-issuing on every status would reset the destination at every marker
    and the robot would never arrive anywhere."""
    fleet = Fleet()
    pilot = policy()
    first = status(bot_id=1, latest_node_id=0)
    fleet.record_status(first)
    (targeted,) = pilot.on_status(fleet, first)
    fleet.record_command(1, targeted.command)

    moved = status(bot_id=1, latest_node_id=1)      # one hop along, not there yet
    fleet.record_status(moved)

    assert pilot.on_status(fleet, moved) == []


def test_arriving_earns_a_new_goal():
    """Reconciliation drops the command on arrival, so the next status finds
    nothing outstanding and the robot is sent somewhere else."""
    fleet = Fleet()
    pilot = policy()
    first = status(bot_id=1, latest_node_id=0)
    fleet.record_status(first)
    (targeted,) = pilot.on_status(fleet, first)
    goal = targeted.command.target_node_id
    fleet.record_command(1, targeted.command)

    arrived = status(bot_id=1, latest_node_id=goal)
    fleet.record_status(arrived)                    # fulfils the command

    (again,) = pilot.on_status(fleet, arrived)
    assert again.command.target_node_id != goal


def test_it_echoes_the_timestamp_it_was_told():
    """Sim milliseconds, so a journal lines up with the run that produced it."""
    fleet = Fleet()
    reported = status(bot_id=1, latest_node_id=0, timestamp=420000)
    fleet.record_status(reported)

    (targeted,) = policy().on_status(fleet, reported)

    assert targeted.command.timestamp == 420000


def test_a_map_too_small_for_the_preference_still_sends_it_somewhere():
    """`minimum_hops` is a preference, not a promise. On the 83-node window
    nothing is more than 20 hops from a charging bay, and a fleet that sits
    still because the map is small is worse than one sent as far as it goes."""
    fleet = Fleet()
    reported = status(bot_id=1, latest_node_id=0)
    fleet.record_status(reported)

    (targeted,) = policy(size=6, minimum_hops=99).on_status(fleet, reported)

    assert targeted.command.target_node_id != 0


def test_it_says_nothing_when_there_is_genuinely_nowhere_to_go():
    """An island, or a one-node map. Silence beats naming somewhere
    unreachable."""
    fleet = Fleet()
    reported = status(bot_id=1, latest_node_id=0)
    fleet.record_status(reported)

    assert policy(size=1, minimum_hops=1).on_status(fleet, reported) == []


def test_it_is_reproducible_from_its_seed():
    fleet = Fleet()
    reported = status(bot_id=1, latest_node_id=0)
    fleet.record_status(reported)

    a = policy(seed=7).on_status(fleet, reported)[0].command.target_node_id
    b = policy(seed=7).on_status(fleet, reported)[0].command.target_node_id

    assert a == b
