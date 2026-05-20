#!/bin/bash
# check_network.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
ENV_FILE="${PROJECT_DIR}/.env"

if [ -f "$ENV_FILE" ]; then
    while IFS='=' read -r key value; do
        case "$key" in
            GCS_IP|MINI_PC_IP|UAV_EDGE_IP|WEB_UI_PORT|ROSBRIDGE_PORT)
                export "$key=$value"
                ;;
        esac
    done < <(grep -E '^[A-Z0-9_]+=' "$ENV_FILE")
fi

GCS_IP="${GCS_IP:-192.168.53.5}"
MINI_PC_IP="${MINI_PC_IP:-192.168.53.6}"
UAV_EDGE_IP="${UAV_EDGE_IP:-192.168.53.2}"
WEB_UI_PORT="${WEB_UI_PORT:-80}"
ROSBRIDGE_PORT="${ROSBRIDGE_PORT:-9090}"

declare -A DEVICES=(
    ["Desktop / GCS"]="$GCS_IP"
    ["Mini PC / UGV SLAM"]="$MINI_PC_IP"
    ["Raspberry Pi / UAV edge"]="$UAV_EDGE_IP"
)

check_ping() {
    local name="$1"
    local ip="$2"

    if ping -c 1 -W 1 "$ip" > /dev/null 2>&1; then
        echo "✅ [$name] $ip - ping OK"
        return 0
    fi

    echo "❌ [$name] $ip - ping failed"
    return 1
}

check_tcp() {
    local name="$1"
    local host="$2"
    local port="$3"

    if timeout 2 bash -c "</dev/tcp/${host}/${port}" > /dev/null 2>&1; then
        echo "✅ [$name] ${host}:${port} - TCP open"
        return 0
    fi

    echo "❌ [$name] ${host}:${port} - TCP closed or unreachable"
    return 1
}

check_http() {
    local name="$1"
    local url="$2"

    if ! command -v curl > /dev/null 2>&1; then
        echo "⚠️  [$name] curl not installed - skipped"
        return 0
    fi

    if curl --fail --silent --show-error --max-time 3 "$url" > /dev/null; then
        echo "✅ [$name] $url - HTTP OK"
        return 0
    fi

    echo "❌ [$name] $url - HTTP failed"
    return 1
}

check_websocket_proxy() {
    local url="http://${GCS_IP}:${WEB_UI_PORT}/rosbridge/"

    if ! command -v curl > /dev/null 2>&1; then
        echo "⚠️  [WebSocket proxy] curl not installed - skipped"
        return 0
    fi

    local response
    response="$(curl --max-time 3 --include --no-buffer --http1.1 \
        --header "Connection: Upgrade" \
        --header "Upgrade: websocket" \
        --header "Sec-WebSocket-Key: SGVsbG8sIHdvcmxkIQ==" \
        --header "Sec-WebSocket-Version: 13" \
        "$url" 2>&1 || true)"

    if grep -q "101 Switching Protocols" <<< "$response"; then
        echo "✅ [WebSocket proxy] ws://${GCS_IP}/rosbridge/ - upgrade OK"
        return 0
    fi

    echo "❌ [WebSocket proxy] ws://${GCS_IP}/rosbridge/ - upgrade failed"
    return 1
}

echo "==============================="
echo " CO-Paint Network Check"
echo " $(date '+%Y-%m-%d %H:%M:%S')"
echo " GCS IP: ${GCS_IP}"
echo "==============================="

FAIL=0

for name in "${!DEVICES[@]}"; do
    ip="${DEVICES[$name]}"
    check_ping "$name" "$ip" || FAIL=$((FAIL + 1))
done

echo "-------------------------------"
check_tcp "GCS Web UI" "$GCS_IP" "$WEB_UI_PORT" || FAIL=$((FAIL + 1))
check_tcp "GCS rosbridge direct" "$GCS_IP" "$ROSBRIDGE_PORT" || FAIL=$((FAIL + 1))
check_http "GCS Web UI" "http://${GCS_IP}:${WEB_UI_PORT}/" || FAIL=$((FAIL + 1))
check_http "GCS API proxy" "http://${GCS_IP}:${WEB_UI_PORT}/api/topic-test-logs?limit=1" || FAIL=$((FAIL + 1))
check_websocket_proxy || FAIL=$((FAIL + 1))

echo "==============================="
if [ $FAIL -eq 0 ]; then
    echo "✅ All network checks passed"
else
    echo "⚠️  ${FAIL} check(s) failed - Please check network, firewall, or services"
fi
echo "==============================="

exit $FAIL
