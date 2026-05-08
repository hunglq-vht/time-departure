#!/usr/bin/env bash
# Regenerate scrp/proto/ from proto/scrp.proto.
# Requires: pip install grpcio-tools mypy-protobuf
set -euo pipefail

python -m grpc_tools.protoc \
  -I proto \
  --python_out=scrp/proto \
  --grpc_python_out=scrp/proto \
  --mypy_out=scrp/proto \
  proto/scrp.proto

# grpc_tools generates a bare "import scrp_pb2" which breaks when the file
# lives inside a package. Fix it to the project's relative import.
sed -i 's/^import scrp_pb2 as scrp__pb2$/from scrp.proto import scrp_pb2 as scrp__pb2/' \
  scrp/proto/scrp_pb2_grpc.py
