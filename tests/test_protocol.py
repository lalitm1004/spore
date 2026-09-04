import pytest

from robot.protocol import Message, LineReader, decode, encode


def test_encode_writes_one_newline_terminated_line():
    line = encode(Message(kind="CMD", name="SET_SPEED", fields={"value": 8.0}))

    assert line == b"CMD SET_SPEED value=8.0\n"


def test_encode_omits_the_field_list_when_there_are_none():
    assert encode(Message(kind="CMD", name="STOP", fields={})) == b"CMD STOP\n"


def test_decode_reads_numeric_fields_back_as_floats():
    message = decode("EVT STATUS t=12.34 err=-0.0012 lost=0")

    assert message.kind == "EVT"
    assert message.name == "STATUS"
    assert message.fields == {"t": 12.34, "err": -0.0012, "lost": 0.0}


def test_non_numeric_field_values_survive_as_text():
    assert decode("CMD SET_MODE mode=follow").fields == {"mode": "follow"}


def test_round_trip_preserves_the_message():
    original = Message(kind="EVT", name="LINE_LOST", fields={"t": 1.5})

    assert decode(encode(original).decode().strip()) == original


@pytest.mark.parametrize("bad", ["", "EVT", "NOPE NAME", "EVT NAME bad_field"])
def test_malformed_lines_are_rejected(bad):
    with pytest.raises(ValueError):
        decode(bad)


def test_reader_assembles_a_message_split_across_reads():
    # A real serial link delivers arbitrary chunks, not whole lines.
    reader = LineReader()

    assert list(reader.feed(b"CMD SET_SP")) == []
    assert list(reader.feed(b"EED value=8.0\nCMD ST")) == [
        Message(kind="CMD", name="SET_SPEED", fields={"value": 8.0})
    ]
    assert list(reader.feed(b"OP\n")) == [Message(kind="CMD", name="STOP", fields={})]


def test_reader_yields_several_messages_from_one_chunk():
    reader = LineReader()

    messages = list(reader.feed(b"CMD STOP\nCMD START\n"))

    assert [m.name for m in messages] == ["STOP", "START"]
