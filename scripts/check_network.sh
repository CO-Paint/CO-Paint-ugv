#!/bin/bash
# check_network.sh

declare -A DEVICES=(
    ["Router"]="192.168.53.1"
    ["Desktop"]="192.168.53.5"
    ["Mini PC"]="192.168.53.4"
    ["Raspberry Pi"]="192.168.53.2"
)

GCS_IP="192.168.53.5"

echo "==============================="
echo " Network Connection Check"
echo " $(date '+%Y-%m-%d %H:%M:%S')"
echo "==============================="

FAIL=0

for name in "${!DEVICES[@]}"; do
    ip="${DEVICES[$name]}"
    if ping -c 1 -W 1 "$ip" > /dev/null 2>&1; then
        echo "✅ [$name] $ip - Connected"
    else
        echo "❌ [$name] $ip - Not Connected"
        FAIL=$((FAIL + 1))
    fi
done

check_tcp() {
    local name="$1"
    local host="$2"
    local port="$3"

    if timeout 2 bash -c "</dev/tcp/${host}/${port}" > /dev/null 2>&1; then
        echo "✅ [$name] ${host}:${port} - Open"
    else
        echo "❌ [$name] ${host}:${port} - Closed"
        FAIL=$((FAIL + 1))
    fi
}

check_http() {
    local name="$1"
    local url="$2"

    if ! command -v curl > /dev/null 2>&1; then
        echo "❌ [$name] curl not installed"
        FAIL=$((FAIL + 1))
        return
    fi

    if curl --max-time 3 --silent --fail "$url" > /dev/null; then
        echo "✅ [$name] $url - OK"
    else
        echo "❌ [$name] $url - Failed"
        FAIL=$((FAIL + 1))
    fi
}

check_websocket() {
    local url="$1"

    if ! command -v curl > /dev/null 2>&1; then
        echo "❌ [rosbridge WebSocket] curl not installed"
        FAIL=$((FAIL + 1))
        return
    fi

    if curl --max-time 3 --silent --include --no-buffer --http1.1 \
        --header "Connection: Upgrade" \
        --header "Upgrade: websocket" \
        --header "Sec-WebSocket-Key: SGVsbG8sIHdvcmxkIQ==" \
        --header "Sec-WebSocket-Version: 13" \
        "$url" 2>/dev/null | grep -q "101 Switching Protocols"; then
        echo "✅ [rosbridge WebSocket] $url - OK"
    else
        echo "❌ [rosbridge WebSocket] $url - Failed"
        FAIL=$((FAIL + 1))
    fi
}

echo "-------------------------------"
check_tcp "Web UI TCP" "$GCS_IP" 80
check_tcp "API TCP" "$GCS_IP" 8000
check_tcp "rosbridge TCP" "$GCS_IP" 9090
check_http "Web UI HTTP" "http://${GCS_IP}/"
check_http "API proxy" "http://${GCS_IP}/api/topic-test-logs?limit=1"
check_websocket "http://${GCS_IP}/rosbridge/"

echo "==============================="
if [ $FAIL -eq 0 ]; then
    echo "✅ All devices connected"
else
    echo "⚠️  ${FAIL} device(s) failed - Please check connection"
fi
echo "==============================="

exit $FAIL  # Returns number of failures as exit code (0 = all success)
