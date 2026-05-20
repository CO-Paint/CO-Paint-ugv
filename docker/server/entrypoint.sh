#!/bin/bash
set -e

uvicorn src.server_node:app --host 0.0.0.0 --port 8000 &
API_PID=$!

python src/telemetry_logger.py &
LOGGER_PID=$!

trap 'kill "$API_PID" "$LOGGER_PID" 2>/dev/null || true' INT TERM EXIT

wait -n "$API_PID" "$LOGGER_PID"
