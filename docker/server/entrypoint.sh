#!/bin/sh
set -eu

uvicorn src.server_node:app --host 0.0.0.0 --port 8000 &
api_pid=$!

python src/telemetry_logger.py &
logger_pid=$!

stop_children() {
    kill "$api_pid" "$logger_pid" 2>/dev/null || true
    wait "$api_pid" "$logger_pid" 2>/dev/null || true
}

trap stop_children INT TERM EXIT

while true; do
    if ! kill -0 "$api_pid" 2>/dev/null; then
        wait "$api_pid"
        exit $?
    fi
    if ! kill -0 "$logger_pid" 2>/dev/null; then
        wait "$logger_pid"
        exit $?
    fi
    sleep 1
done
