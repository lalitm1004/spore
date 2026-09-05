"""Loading and validating the shared robot<->network JSON schemas.

The schemas are the ground-truth contract for this interface. These are
canonical copies from `spore-amr/shared/schemas/`; when this package is pasted
into the webots implementation, point `SCHEMA_DIR` at that shared directory
instead of the local copy and the two stay in lockstep.

Pure: no grpc, and the schema files are read once and cached.
"""

import json
import os
from functools import lru_cache
from pathlib import Path

ROBOT_TO_NETWORK = "robot-to-network"
NETWORK_TO_ROBOT = "network-to-robot"

_DEFAULT_DIR = Path(__file__).resolve().parent.parent.parent / "schemas"
SCHEMA_DIR = Path(os.environ.get("TEMP_NETWORK_INTERFACE_SCHEMA_DIR", _DEFAULT_DIR))


@lru_cache(maxsize=None)
def _validator(name: str):
    """Compile a schema into a cached draft-2020-12 validator."""
    import jsonschema

    path = SCHEMA_DIR / "{}.schema.json".format(name)
    schema = json.loads(path.read_text())
    validator = jsonschema.Draft202012Validator
    validator.check_schema(schema)  # fail fast on a malformed schema file
    return validator(schema)


def validate_robot_to_network(document: dict) -> None:
    """Raise `jsonschema.ValidationError` if `document` is not a valid
    robot-to-network payload."""
    _validator(ROBOT_TO_NETWORK).validate(document)


def validate_network_to_robot(document: dict) -> None:
    """Raise `jsonschema.ValidationError` if `document` is not a valid
    network-to-robot payload."""
    _validator(NETWORK_TO_ROBOT).validate(document)
