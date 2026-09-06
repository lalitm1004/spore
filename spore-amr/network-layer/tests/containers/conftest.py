"""Fixtures for the container tier, scoped to this directory.

`tests/conftest.py` has an in-process `fleet` fixture; this tier has a Docker
one with the same name and the same role. Pytest scopes a conftest to its
directory, so this shadows the root one for these tests only -- which is why
the container scenarios live in their own package (`containers`, not `docker`, which would shadow the library `up.py` imports).
"""
from tests.containers.harness import (  # noqa: F401
    _sweep_leaked_fleets, client, fleet, image, one_bot, three_bots, two_bots)
