"""Entry point: start the SCRP gRPC microservice."""
from __future__ import annotations

import argparse
import logging
import signal
import sys
from concurrent import futures

import grpc

from scrp.grpc_server import SCRPServicer
from scrp.proto import scrp_pb2_grpc

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)s %(name)s: %(message)s",
)
logger = logging.getLogger("scrp.server")

_DEFAULT_HOST = "0.0.0.0"
_DEFAULT_PORT = 50051
_DEFAULT_WORKERS = 10


def serve(host: str, port: int, max_workers: int) -> None:
    address = f"{host}:{port}"
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=max_workers))
    scrp_pb2_grpc.add_SCRPServiceServicer_to_server(SCRPServicer(), server)
    server.add_insecure_port(address)
    server.start()
    logger.info("SCRP gRPC server listening on %s", address)

    def _handle_sigterm(*_):
        logger.info("Received shutdown signal — stopping gracefully …")
        done_event = server.stop(grace=5)
        done_event.wait()
        logger.info("Server stopped.")
        sys.exit(0)

    signal.signal(signal.SIGTERM, _handle_sigterm)
    signal.signal(signal.SIGINT, _handle_sigterm)

    server.wait_for_termination()


def main() -> None:
    parser = argparse.ArgumentParser(description="SCRP gRPC microservice")
    parser.add_argument("--host", default=_DEFAULT_HOST)
    parser.add_argument("--port", type=int, default=_DEFAULT_PORT)
    parser.add_argument("--workers", type=int, default=_DEFAULT_WORKERS)
    args = parser.parse_args()
    serve(args.host, args.port, args.workers)


if __name__ == "__main__":
    main()
