# Robot brains. Based on the Webots image for `webots-controller` and the
# Python bindings; N containers share these layers, and the limited resource
# is the Python process' RSS, not image size.
FROM cyberbotics/webots:R2025a-ubuntu22.04

# opencv-python-headless carries the QR decoder and needs no X libraries;
# the same cv2.QRCodeDetector call is what would run on the Pi. grpcio and
# protobuf are for the network-layer bot that runs beside each companion --
# the fleet's membership, jobs and routing all speak gRPC between bots.
#
# KNOWN BREAK, and the reason it has never been noticed: this is Ubuntu 22.04,
# so Python 3.10, and the network layer needs 3.11+ for `enum.StrEnum`. The
# entrypoint launches `bot.py` here and it raises ImportError immediately, which
# is why the Webots fleet has never actually run the network layer. Do not fix
# it by dropping StrEnum -- `Decision.to_json` uses `str(self.kind)`, which a
# plain `(str, Enum)` renders as "DecisionKind.PROCEED". The fix is to run the
# bot in its own image, which is what moving the robot link onto gRPC allows.
RUN apt-get update \
    && apt-get install -y --no-install-recommends python3-yaml python3-pip socat \
    && pip3 install --no-cache-dir "numpy>=1.24" "opencv-python-headless>=4.9" \
        "grpcio>=1.83.1" "protobuf>=7.36.1" \
    && rm -rf /var/lib/apt/lists/*

ENV WEBOTS_HOME=/usr/local/webots
ENV HOME=/tmp
WORKDIR /project

COPY docker/robot-entrypoint.sh /usr/local/bin/robot-entrypoint

ENTRYPOINT ["/usr/local/bin/robot-entrypoint"]
