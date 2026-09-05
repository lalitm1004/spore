# Robot brains. Based on the Webots image for `webots-controller` and the
# Python bindings; N containers share these layers, and the limited resource
# is the Python process' RSS, not image size.
FROM cyberbotics/webots:R2025a-ubuntu22.04

# opencv-python-headless carries the QR decoder and needs no X libraries;
# the same cv2.QRCodeDetector call is what would run on the Pi.
RUN apt-get update \
    && apt-get install -y --no-install-recommends python3-yaml python3-pip socat \
    && pip3 install --no-cache-dir "numpy>=1.24" "opencv-python-headless>=4.9" \
    && rm -rf /var/lib/apt/lists/*

ENV WEBOTS_HOME=/usr/local/webots
ENV HOME=/tmp
WORKDIR /project

COPY docker/robot-entrypoint.sh /usr/local/bin/robot-entrypoint

ENTRYPOINT ["/usr/local/bin/robot-entrypoint"]
