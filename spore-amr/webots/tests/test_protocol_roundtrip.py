"""The firmware link, over a real pty rather than in memory.

`test_protocol.py` round-trips the codec through a buffer. That covers the
grammar and misses the one thing this link does that a buffer does not: a read
returns whatever happens to have arrived, so a message can be split anywhere --
mid-field, mid-number, between the last character and the newline.

In the container this is a socat pty pair standing in for the Pi-to-Arduino
serial line, and every byte crosses it. `os.openpty` gives the same thing
without socat, so this runs anywhere the rest of the suite does.
"""

import os

import pytest

from robot.protocol import LineReader, Message, encode


@pytest.fixture
def pty():
    """A real pty pair: writes on one end appear on the other, in pieces."""
    primary, secondary = os.openpty()
    try:
        yield primary, secondary
    finally:
        os.close(primary)
        os.close(secondary)


def drain(reader, fd, expected):
    """Read until `expected` messages have come out, one chunk at a time."""
    events = []
    while len(events) < expected:
        events.extend(reader.feed(os.read(fd, 4096)))
    return events


def test_a_message_survives_a_real_serial_hop(pty):
    primary, secondary = pty
    os.write(secondary, encode(Message(kind="EVT", name="MARKER", fields={
        "t": 15.744, "node": 77, "kind": "PT", "region": 3, "heading": 1.57})))

    (event,) = drain(LineReader(), primary, 1)
    assert event.name == "MARKER"
    assert event.fields["node"] == 77
    assert event.fields["heading"] == pytest.approx(1.57)


def test_a_message_split_across_reads_is_reassembled(pty):
    """The failure a buffer cannot show. The writer is a separate process and
    the reader gets whatever the kernel had ready."""
    primary, secondary = pty
    payload = encode(Message(kind="CMD", name="TURN", fields={
        "bearing": -1.5708, "node": 114, "heading": 0.0}))

    for i in range(0, len(payload), 3):     # three bytes at a time
        os.write(secondary, payload[i:i + 3])

    (command,) = drain(LineReader(), primary, 1)
    assert command.name == "TURN"
    assert command.fields["node"] == 114
    assert command.fields["bearing"] == pytest.approx(-1.5708)


def test_several_messages_in_one_read_all_come_out(pty):
    """The other direction: the firmware talks faster than the companion reads,
    so a single read holds a marker, a status and a command at once."""
    primary, secondary = pty
    os.write(secondary, b"".join([
        encode(Message(kind="EVT", name="STATUS", fields={"t": 1.0, "lost": 0})),
        encode(Message(kind="EVT", name="MARKER", fields={"t": 1.1, "node": 5})),
        encode(Message(kind="EVT", name="OBSTACLE", fields={"t": 1.2, "state": "HOLDING"})),
    ]))

    events = drain(LineReader(), primary, 3)
    assert [e.name for e in events] == ["STATUS", "MARKER", "OBSTACLE"]


def test_a_partial_tail_waits_for_the_rest(pty):
    """Half a message must produce nothing, not a message with half its fields.
    A `MARKER` missing its node would be read as node 0 -- a real place."""
    primary, secondary = pty
    payload = encode(Message(kind="EVT", name="MARKER", fields={"t": 2.0, "node": 42}))
    os.write(secondary, payload[:-4])

    reader = LineReader()
    assert list(reader.feed(os.read(primary, 4096))) == []

    os.write(secondary, payload[-4:])
    (event,) = drain(reader, primary, 1)
    assert event.fields["node"] == 42
