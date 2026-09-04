"""Reading a marker's QR code out of a camera frame.

Kept separate from `robot/marker.py` so the crossing state machine stays pure
and host-testable while this half owns the one heavy dependency. The decoder is
OpenCV's, which is also what would run on the Pi -- the sim exercises the same
call, so a frame too blurred or too small genuinely fails here rather than
being modelled as failing.
"""

from dataclasses import dataclass
from typing import Optional

from tools.track.marker import decode_payload


@dataclass(frozen=True)
class Read:
    """One successful decode."""

    node_id: int
    kind: str
    x_mm: int
    y_mm: int
    bearing_deg: int
    out_edges: tuple
    image_rotation: float = 0.0   # radians, code's rotation within the frame

    @property
    def summary(self) -> str:
        edges = " ".join("{}deg>{}".format(b, n) for b, n in self.out_edges)
        return "node {} {} at ({}, {}) mm{}".format(
            self.node_id, self.kind, self.x_mm, self.y_mm,
            "  ->  " + edges if edges else "")


def to_gray(image: bytes, width: int, height: int):
    """Webots hands back BGRA, 4 bytes per pixel, row-major."""
    import numpy as np

    frame = np.frombuffer(image, dtype=np.uint8)
    expected = width * height * 4
    if frame.size != expected:
        raise ValueError("expected {} bytes, got {}".format(expected, frame.size))

    bgra = frame.reshape((height, width, 4))
    # Rec. 601 luma on the BGR channels, which is what a mono sensor would see.
    blue, green, red = bgra[:, :, 0], bgra[:, :, 1], bgra[:, :, 2]
    return (0.114 * blue + 0.587 * green + 0.299 * red).astype("uint8")


def _image_rotation(points) -> float:
    """Rotation of the code within the frame, from its detected corners.

    The marker is laid along the lane, so this angle is the robot's heading
    relative to the lane -- an absolute heading reference that owes nothing to
    the wheels, and therefore does not inherit their accumulated drift.
    """
    import math

    import numpy as np

    corners = np.array(points, dtype=float).reshape(-1, 2)
    if corners.shape[0] < 2:
        return 0.0
    edge = corners[1] - corners[0]
    return math.atan2(float(edge[1]), float(edge[0]))


class QrReader:
    """Decodes marker payloads from grayscale frames.

    Holds the detector across calls because constructing one is not free, and
    this runs inside a 16 ms control step.
    """

    def __init__(self):
        import cv2

        self._detector = cv2.QRCodeDetector()

    def read(self, gray) -> Optional[Read]:
        """Return a `Read`, or None if there is no legible code in the frame.

        A frame with no code, an unreadable code, or a code whose payload is
        not ours are all the same answer: nothing to act on. Only genuinely
        unexpected failures propagate.
        """
        try:
            payload, points, _ = self._detector.detectAndDecode(gray)
        except Exception:  # OpenCV raises rather than returning on some frames
            return None

        if not payload or points is None:
            return None

        try:
            fields = decode_payload(payload)
        except ValueError:
            # A legible code that is not one of ours -- worth ignoring quietly,
            # since a warehouse floor may carry other markings.
            return None

        return Read(
            node_id=fields["node_id"],
            kind=fields["kind"],
            x_mm=fields["x_mm"],
            y_mm=fields["y_mm"],
            bearing_deg=fields["bearing_deg"],
            out_edges=tuple(fields["out_edges"]),
            image_rotation=_image_rotation(points),
        )
