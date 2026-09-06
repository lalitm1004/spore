"""Every command the companion can send is one the firmware handles.

WHY THIS EXISTS
    `CMD HOLD` was unhandled for a while, and nothing said so. The state it
    sets (`hold_until`, `held_marker`) was declared, the block that consumes
    it was written, and the `elif` that connects them was missing -- so holds
    arrived and matched nothing. A held robot then timed out on
    `junction_timeout_s`, carried on off the end of a degree-1 bay, lost the
    line, and span on the spot in the lost-line search for the rest of the
    run. It looked like a driving fault and was a missing branch.

    The link is a text protocol with no schema and no dispatch table, so a
    name that matches nothing is silence rather than an error. This is the
    check that would have failed instead.

HOW
    `robot/main.py` cannot be imported outside Webots -- it needs the
    `controller` module -- so both sides are read as source. Crude, and it is
    the same crudeness as the protocol: a name in a string on one side and a
    name in a string on the other.
"""
from __future__ import annotations

import re
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent

#: Named in the table in `docs/architecture.md`; keep the three in step.
DOCUMENTED_COMMANDS = {"TURN", "HOLD", "START", "STOP", "SET_SPEED", "SET_GAINS"}


def _sent_by_companion() -> set[str]:
    source = (ROOT / "robot" / "companion.py").read_text()
    return set(re.findall(r'Message\(kind="CMD", name="([A-Z_]+)"', source))


def _handled_by_firmware() -> set[str]:
    source = (ROOT / "robot" / "main.py").read_text()
    return set(re.findall(r'command\.name == "([A-Z_]+)"', source))


def test_the_firmware_handles_every_command_the_companion_sends():
    unhandled = _sent_by_companion() - _handled_by_firmware()

    assert not unhandled, (
        "the companion sends {} and main.py's dispatch does not match it, so "
        "it is silently dropped".format(sorted(unhandled)))


def test_hold_restarts_the_junction_wait_rather_than_letting_it_expire():
    """The point of handling HOLD, not just that a branch exists.

    Setting `awaiting_since` forward is what stops an answered-with-wait robot
    from falling through to `junction_timeout_s` and driving on regardless.
    """
    source = (ROOT / "robot" / "main.py").read_text()
    branch = source.split('command.name == "HOLD"', 1)[1].split("elif command.name")[0]

    assert "hold_until" in branch
    assert "awaiting_since = hold_until" in branch


def test_the_documented_command_table_is_the_one_the_firmware_implements():
    assert _handled_by_firmware() == DOCUMENTED_COMMANDS
    table = (ROOT / "docs" / "architecture.md").read_text()
    for name in DOCUMENTED_COMMANDS:
        assert "`CMD {}`".format(name) in table, "{} is undocumented".format(name)
