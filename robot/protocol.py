"""Wire protocol for the firmware <-> companion link.

Newline-delimited ASCII, so the link can be read with a serial monitor and
logs are legible without a decoder:

    CMD SET_SPEED value=8.0
    EVT LINE_LOST t=12.34

Everything lives behind `encode`/`decode`, so a binary framing can replace the
format without either side's logic changing. Pure: no I/O, no Webots.
"""

from dataclasses import dataclass, field
from typing import Dict, Iterator, Union

KINDS = ("CMD", "EVT")

Value = Union[float, str]


@dataclass(frozen=True)
class Message:
    kind: str
    name: str
    fields: Dict[str, Value] = field(default_factory=dict)


def encode(message: Message) -> bytes:
    parts = [message.kind, message.name]
    parts.extend("{}={}".format(k, v) for k, v in message.fields.items())
    return (" ".join(parts) + "\n").encode("ascii")


def decode(line: str) -> Message:
    tokens = line.strip().split()
    if len(tokens) < 2:
        raise ValueError("malformed message: {!r}".format(line))

    kind, name = tokens[0], tokens[1]
    if kind not in KINDS:
        raise ValueError("unknown message kind {!r} in {!r}".format(kind, line))

    fields: Dict[str, Value] = {}
    for token in tokens[2:]:
        if "=" not in token:
            raise ValueError("malformed field {!r} in {!r}".format(token, line))
        key, _, raw = token.partition("=")
        try:
            fields[key] = float(raw)
        except ValueError:
            fields[key] = raw

    return Message(kind=kind, name=name, fields=fields)


class LineReader:
    """Reassembles messages from arbitrary byte chunks off the link."""

    def __init__(self):
        self._buffer = b""

    def feed(self, chunk: bytes) -> Iterator[Message]:
        self._buffer += chunk
        while b"\n" in self._buffer:
            line, _, self._buffer = self._buffer.partition(b"\n")
            text = line.decode("ascii", "replace").strip()
            if text:
                yield decode(text)
