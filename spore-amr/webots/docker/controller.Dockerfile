# Robot brains. Based on the Webots image for `webots-controller` and the
# Python bindings; N containers share these layers, and the limited resource
# is the Python process' RSS, not image size.
FROM cyberbotics/webots:R2025a-ubuntu22.04

# opencv-python-headless carries the QR decoder and needs no X libraries;
# the same cv2.QRCodeDetector call is what would run on the Pi.
#
# grpcio and jsonschema are the network layer's: the companion speaks gRPC to
# the fleet service and every payload is validated against the shared schemas
# at the wire. This image is Ubuntu 22.04, so Python 3.10 -- which is why
# temp-network-interface pins >=3.10 rather than the 3.13 it was written on.
RUN apt-get update \
    && apt-get install -y --no-install-recommends python3-yaml python3-pip socat \
    && pip3 install --no-cache-dir "numpy>=1.24" "opencv-python-headless>=4.9" \
        "grpcio>=1.60" "protobuf>=4.25" "jsonschema>=4.20" \
    && rm -rf /var/lib/apt/lists/*

ENV WEBOTS_HOME=/usr/local/webots
ENV HOME=/tmp
WORKDIR /project

COPY docker/robot-entrypoint.sh /usr/local/bin/robot-entrypoint

ENTRYPOINT ["/usr/local/bin/robot-entrypoint"]
