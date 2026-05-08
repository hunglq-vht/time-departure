FROM python:3.11-slim

# Nexus PyPI mirror — override at build time:
#   docker build --build-arg NEXUS_PYPI_URL=https://nexus.corp/repository/pypi-proxy/simple \
#                --build-arg NEXUS_PYPI_HOST=nexus.corp ...
ARG NEXUS_PYPI_URL=https://nexus.example.com/repository/pypi-proxy/simple
ARG NEXUS_PYPI_HOST=nexus.example.com

WORKDIR /app

# Install grpc-health-probe for liveness/readiness checks (apt — allowed online)
RUN apt-get update && apt-get install -y --no-install-recommends wget \
    && GRPC_HEALTH_PROBE_VERSION=v0.4.24 \
    && wget -qO /usr/local/bin/grpc_health_probe \
       https://github.com/grpc-ecosystem/grpc-health-probe/releases/download/${GRPC_HEALTH_PROBE_VERSION}/grpc_health_probe-linux-amd64 \
    && chmod +x /usr/local/bin/grpc_health_probe \
    && apt-get purge -y --auto-remove wget \
    && rm -rf /var/lib/apt/lists/*

# Write pip configuration so every subsequent pip call uses Nexus
RUN pip config set global.index-url "${NEXUS_PYPI_URL}" \
    && pip config set global.trusted-host "${NEXUS_PYPI_HOST}"

COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY proto/ proto/
COPY scrp/ scrp/
COPY server.py .

# Generate Python protobuf bindings (uses grpc_tools installed above — no extra network call)
RUN python -m grpc_tools.protoc \
      -I proto \
      --python_out=scrp/proto \
      --grpc_python_out=scrp/proto \
      proto/scrp.proto \
    && touch scrp/proto/__init__.py

EXPOSE 50051

ENTRYPOINT ["python", "server.py"]
