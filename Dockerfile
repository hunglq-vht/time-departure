FROM python:3.11-slim

WORKDIR /app

# Install grpc-health-probe for liveness/readiness checks
RUN apt-get update && apt-get install -y --no-install-recommends wget \
    && GRPC_HEALTH_PROBE_VERSION=v0.4.24 \
    && wget -qO /usr/local/bin/grpc_health_probe \
       https://github.com/grpc-ecosystem/grpc-health-probe/releases/download/${GRPC_HEALTH_PROBE_VERSION}/grpc_health_probe-linux-amd64 \
    && chmod +x /usr/local/bin/grpc_health_probe \
    && apt-get purge -y --auto-remove wget \
    && rm -rf /var/lib/apt/lists/*

COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY proto/ proto/
COPY scrp/ scrp/
COPY server.py .

# Generate Python protobuf bindings
RUN python -m grpc_tools.protoc \
      -I proto \
      --python_out=scrp/proto \
      --grpc_python_out=scrp/proto \
      proto/scrp.proto \
    && touch scrp/proto/__init__.py

EXPOSE 50051

ENTRYPOINT ["python", "server.py"]
