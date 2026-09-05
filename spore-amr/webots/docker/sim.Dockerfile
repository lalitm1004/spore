# The simulator. Runs headless under Xvfb and streams the 3D view to the
# browser in w3d mode, which renders client-side and so needs no GPU.
FROM cyberbotics/webots:R2025a-ubuntu22.04

ENV HOME=/tmp
WORKDIR /project

COPY docker/sim-entrypoint.sh /usr/local/bin/sim-entrypoint
EXPOSE 1234

ENTRYPOINT ["/usr/local/bin/sim-entrypoint"]
