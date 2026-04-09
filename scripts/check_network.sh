#!/bin/bash
# check_network.sh

declare -A DEVICES=(
    ["Desktop"]="192.168.0.20"
    ["Mini PC"]="192.168.0.21"
    ["Raspberry Pi"]="192.168.0.41"
)

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

echo "==============================="
if [ $FAIL -eq 0 ]; then
    echo "✅ All devices connected"
else
    echo "⚠️  ${FAIL} device(s) failed - Please check connection"
fi
echo "==============================="

exit $FAIL  # Returns number of failures as exit code (0 = all success)