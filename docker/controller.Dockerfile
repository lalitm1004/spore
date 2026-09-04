# Robot brains. Based on the Webots image for `webots-controller` and the
# Python bindings; N containers share these layers, and the limited resource
# is the Python process' RSS, not image size.
FROM cyberbotics/webots:R2025a-ubuntu22.04

RUN apt-get update \
    && apt-get install -y --no-install-recommends python3-yaml socat \
    && rm -rf /var/lib/apt/lists/*

ENV WEBOTS_HOME=/usr/local/webots
ENV HOME=/tmp
WORKDIR /project

COPY docker/robot-entrypoint.sh /usr/local/bin/robot-entrypoint

ENTRYPOINT ["/usr/local/bin/robot-entrypoint"]
