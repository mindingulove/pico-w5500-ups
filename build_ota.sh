#!/bin/zsh
set -euo pipefail

PROJECT_DIR="${0:A:h}"
ARDUINO_CLI="/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli"
OUTPUT_DIR="$PROJECT_DIR/build/ota"
BUILD_DIR="/private/tmp/ups-battery-monitor-ota-build"

mkdir -p "$OUTPUT_DIR"
"$ARDUINO_CLI" compile \
  --fqbn rp2040:rp2040:rpipico \
  --board-options flash=2097152_1048576 \
  --build-path "$BUILD_DIR" \
  --output-dir "$OUTPUT_DIR" \
  "$PROJECT_DIR"

echo "BIN: $OUTPUT_DIR/ups_battery_monitor.ino.bin"
echo "UF2: $OUTPUT_DIR/ups_battery_monitor.ino.uf2"
