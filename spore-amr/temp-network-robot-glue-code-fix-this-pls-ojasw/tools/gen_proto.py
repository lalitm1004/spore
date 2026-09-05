"""Regenerate the gRPC stubs from proto/network.proto.

Run from the project root:

    uv run python tools/gen_proto.py

The generated `_grpc.py` references its `_pb2.py` sibling with a bare top-level
import, which breaks when both live inside the `temp_network_interface` package; this
script patches that to a package-absolute import after codegen.
"""

from __future__ import annotations

import pathlib

from grpc_tools import protoc

ROOT = pathlib.Path(__file__).resolve().parent.parent
PROTO = ROOT / "proto" / "network.proto"
OUT = ROOT / "src" / "temp_network_interface"


def main() -> int:
    code = protoc.main([
        "grpc_tools.protoc",
        "-I{}".format(ROOT / "proto"),
        "--python_out={}".format(OUT),
        "--grpc_python_out={}".format(OUT),
        str(PROTO),
    ])
    if code != 0:
        return code

    grpc_file = OUT / "network_pb2_grpc.py"
    text = grpc_file.read_text()
    patched = text.replace(
        "import network_pb2 as network__pb2",
        "from temp_network_interface import network_pb2 as network__pb2",
    )
    if patched == text:
        raise SystemExit("expected the generated import was not found; "
                         "check grpcio-tools output")
    grpc_file.write_text(patched)
    print("regenerated {} (+ import patch)".format(grpc_file.name))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
