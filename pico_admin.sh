#!/bin/zsh
set -euo pipefail

PROJECT_DIR="${0:A:h}"
PICO_IP="${PICO_IP:-192.168.6.122}"
ACTION="${1:-}"
TOKEN="$(sed -n 's/^#define UPS_OTA_TOKEN "\([^"]*\)"/\1/p' "$PROJECT_DIR/ups_secrets.h")"

if [[ -z "$TOKEN" ]]; then
  echo "UPS_OTA_TOKEN is missing from ups_secrets.h" >&2
  exit 1
fi

case "$ACTION" in
  upload)
    FIRMWARE_BIN="${2:-$PROJECT_DIR/build/ota/ups_battery_monitor.ino.bin}"
    if [[ ! -f "$FIRMWARE_BIN" ]]; then
      echo "Firmware binary not found: $FIRMWARE_BIN" >&2
      exit 1
    fi
    FIRMWARE_CRC32="$(python3 -c '
import struct, sys, zlib

data = open(sys.argv[1], "rb").read()
if not 4096 <= len(data) <= 900 * 1024 or len(data) % 4:
    raise SystemExit("not a valid-size Arduino-Pico .bin")

boot2_crc = 0xffffffff
for byte in data[:252]:
    boot2_crc ^= byte << 24
    for _ in range(8):
        boot2_crc = (((boot2_crc << 1) ^ 0x04c11db7) & 0xffffffff
                     if boot2_crc & 0x80000000 else (boot2_crc << 1) & 0xffffffff)
expected_boot2_crc = int.from_bytes(data[252:256], "little")
stack, reset = struct.unpack_from("<II", data, 256)
reset_address = reset & ~1
valid = (boot2_crc == expected_boot2_crc and
         0x20000000 <= stack <= 0x20042000 and stack % 8 == 0 and
         reset & 1 and 0x10000100 <= reset_address < 0x10000000 + len(data))
if not valid:
    raise SystemExit("not a valid Arduino-Pico RP2040 .bin")

print(f"{zlib.crc32(data) & 0xffffffff:08x}")
' "$FIRMWARE_BIN")"
    echo "Uploading ${FIRMWARE_BIN:t} (CRC32 $FIRMWARE_CRC32) to $PICO_IP"
    curl --fail-with-body --connect-timeout 5 --max-time 180 \
      -H 'Content-Type: application/octet-stream' \
      -H "X-Firmware-CRC32: $FIRMWARE_CRC32" \
      --data-binary "@$FIRMWARE_BIN" \
      "http://$PICO_IP/api/firmware?token=$TOKEN"
    ;;
  rollback)
    curl --fail-with-body --connect-timeout 5 --max-time 15 \
      -X POST "http://$PICO_IP/api/firmware/rollback?token=$TOKEN"
    ;;
  restart)
    curl --fail-with-body --connect-timeout 5 --max-time 15 \
      -X POST "http://$PICO_IP/api/system/restart?token=$TOKEN"
    ;;
  log)
    curl --fail-with-body --connect-timeout 5 --max-time 15 \
      "http://$PICO_IP/api/log"
    ;;
  *)
    echo "Usage: $0 {upload [firmware.bin]|rollback|restart|log}" >&2
    exit 2
    ;;
esac

echo
