"""Package entry point.

WHAT
    `main()` builds the FastAPI app from production singletons and serves it
    with uvicorn. Exposed as the `spore-control-plane` console script.
"""
from __future__ import annotations

import logging

import uvicorn

from spore_control_plane import config

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")


def main() -> None:
    from spore_control_plane.app import create_app

    app = create_app()
    uvicorn.run(app, host=config.HTTP_HOST, port=config.HTTP_PORT)


if __name__ == "__main__":
    main()
