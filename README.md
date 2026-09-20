# Pico W5500 UPS Battery Monitor

Firmware for a Raspberry Pi Pico UPS controller using W5500 Ethernet and I2C peripherals.

## Hardware

| Device | Address | Purpose |
| --- | --- | --- |
| XL9535 | `0x20`-`0x27` | Relay outputs |
| ADS1115 | `0x48`-`0x4B` | Isolated PC817 output sensing; required for outage-time relay cutoff |
| AT24C256 | `0x50`-`0x57` | Persistent battery estimate and configuration |
| LTC2944 | `0x64`-`0x67` | Battery voltage, current and temperature |

The Pico uses GP12 for SDA and GP13 for SCL. Devices within each ISO1540 power
domain share that domain's ground; do not bridge the isolation barrier. SDA and
SCL must remain at 3.3 V logic levels.

When ADS1115 is missing, firmware uses successful XL9535 relay commands only as an estimated output state. API and MQTT status mark this with `sensed_estimated: true` and `sense_source: "relay_command"`. This is not a physical measurement and never authorizes relay cutoff during an outage.

Power the ADS1115 from its local isolated-side 3.3 V supply. Its GND and ADDR
pins connect to the PC817 output-side GND, selecting I2C address `0x48`. Connect
A0 to the Pi1 PC817 output and A1 to the Pi2 PC817 output; never connect either
Pi power rail or Pi-side ground directly across the isolation barrier.

The installed PC817 outputs are active-low and calibrated for the measured
GPIO17 interface levels: approximately `0.017 V` while running and `0.58 V`
after the shutdown signal is released. Firmware must first observe an ADS
channel at or below `0.10 V` for 10 seconds while the Pi is running. After the
shutdown request, it must observe the channel at or above `0.45 V` for 30
seconds before permitting that Pi's relay to turn off. Readings between the two
thresholds are indeterminate and fail closed. A channel that starts in the OFF
range cannot authorize cutoff because the preceding running state was not seen.
The API/MQTT `ads_v` field displays the inverted logical Pi voltage
(`3.30 V - PC817 output`); `ads_raw_v` retains the actual ADC measurement used
by the safety interlock.

For GPIO-based host-alive sensing, install `ups_gpio_alive.service` so GPIO17
(physical pin 11) is driven HIGH after boot. Install the executable
`ups_gpio_shutdown_final` as
`/usr/lib/systemd/system-shutdown/ups_gpio_shutdown_final`; it drives GPIO17
LOW only for the final `poweroff` or `halt` phase, after normal services have
stopped. The hook runs `sync` and fails closed unless every persistent
filesystem visible in `/proc/mounts` is unmounted or read-only. Do not use an
ordinary service `ExecStop` for the LOW transition because that runs too early
to prove shutdown is complete.

## Network

- Static IP: `192.168.6.122`
- HTTP status: `http://192.168.6.122/api/status`
- NUT server: `192.168.6.122:3493`, UPS name `pico-ups`
- MQTT broker: `192.168.6.245:1883`
- MQTT base topic: `ups/battery/main`

MQTT connection requires a working Ethernet link. `rc=-2` means the TCP connection to the broker failed.

## PVE-UPS / NUT

The Pico exposes a read-only NUT protocol endpoint for PVE-UPS on TCP port
`3493`. Configure the PVE-UPS source as:

- Type: `NUT server`
- Host: `192.168.6.122`
- Port: `3493`
- UPS name: `pico-ups`
- Username/password: leave blank

The endpoint implements the anonymous `LIST VAR` operation used by PVE-UPS and
also supports `LIST UPS`, `VER`, `PROTVER`, and `LOGOUT`. It does not accept NUT
write or instant commands. Keep TCP port 3493 restricted to the trusted
management LAN because the NUT protocol is unencrypted.

`ups.status` changes to `OB` only after battery discharge is confirmed for five
seconds. Once on battery, it remains latched until voltage is at least `13.60 V`
with non-discharging current for 30 seconds. This prevents falling server load
during shutdown from being misreported as restored utility power. Sensor loss
is reported as `NOCOMM`, never as `OL`.

With PVE-UPS configured to shut PBS down first and the Proxmox host last at
`10%` charge, the Pico arms a RAM-only external-shutdown handoff when its NUT
state is `OB` and SOC reaches `10%`. Arming never authorizes relay cutoff by
itself. The Pico waits at least 60 seconds and requires every still-powered
host's ADS/PC817 channel to complete the normal running-to-final-OFF proof.
Only then does it persist the low-battery lockout and remove relay power.

If utility returns while PVE-UPS is completing shutdown, the handoff remains
armed for five minutes. A completed final-OFF proof during that window causes
the safe relay cutoff, followed by the normal confirmed automatic restoration,
so halted hosts receive an OFF-to-ON start. If no complete proof arrives, the
RAM latch expires without changing a relay. If PVE-UPS fails, the existing 5%
local shutdown agents remain the fallback. If both methods fail, GPIO stays in
the running state and relay cutoff remains blocked, including at `10.8 V`.
The status API exposes `pve_ups_handoff_armed`,
`pve_ups_handoff_cutoff_ready`, and
`pve_ups_handoff_recovery_remaining_sec` for diagnosis.

The Pico publishes `battery.runtime` only while it has a meaningful live
discharge estimate. PVE-UPS can always use its on-battery and charge triggers;
the runtime trigger becomes available during a measurable discharge. Output
load percentage is deliberately omitted because the measured signed battery
power is not the same as UPS output load relative to a rated capacity.

## Home Assistant: MQTT And HTTP Sensors

Both telemetry paths are intentionally available at the same time:

- MQTT discovery creates source-specific entities such as
  `sensor.ups_battery_main_mqtt_soc`,
  `sensor.ups_battery_main_mqtt_power`, and
  `sensor.ups_battery_main_mqtt_runtime`. These entities are attached to the
  `Server UPS` MQTT device. Their concise entity names (`SOC`, `Voltage`,
  `Current`, `Power`, `Temperature`, `Energy`, and `Runtime`) keep dashboard
  headings and graph legends readable without changing their entity IDs.
- The Home Assistant package polls `GET /api/status` every 15 seconds through
  `sensor.ups_pico_raw_status` and keeps the existing HTTP-derived entities,
  including `sensor.ups_battery_main_soc`,
  `sensor.ups_battery_main_power`, and
  `sensor.ups_battery_main_runtime`. Their display names include `HTTP` so the
  two sources are unambiguous while existing dashboard entity IDs continue to
  work.

MQTT and HTTP values should normally match, allowing either path to be graphed
or used in automations. The MQTT runtime text reports `Charging`, `Idle`, or a
remaining-time value instead of exposing the firmware's internal `-1` sentinel.
The numeric MQTT runtime-minutes entity has no numeric state while charging or
idle. The Pico subscribes to Home Assistant's `homeassistant/status` birth
topic and republishes discovery when Home Assistant reconnects.

`power_w` is signed battery power flow, not a direct measurement of total server
load. Positive values indicate charging and negative values indicate discharge.
`sensor.ups_battery_main_estimated_load_w` remains a Home Assistant-configured
outage-load estimate used when a live discharge runtime cannot be calculated.

## Local Configuration

Create the private firmware configuration before building:

```sh
cp ups_secrets.example.h ups_secrets.h
```

Set `UPS_OTA_TOKEN` and `UPS_CALIBRATION_TOKEN` to different long random
values. Set `UPS_MQTT_USER` and `UPS_MQTT_PASSWORD` when the broker requires
authentication; leave both empty for anonymous MQTT. Keep the matching
calibration token in the Home Assistant `rest_command` URLs. For the included
Home Assistant package, copy `homeassistant_ups_package.example.yaml` to
`homeassistant_ups_package.yaml` and replace
`replace_with_calibration_token` locally. The real `ups_secrets.h`, Home
Assistant package, `.env`, generated firmware, and private `plans/` tree are
excluded from Git.

## Build

Use the provided script so every firmware has the required OTA flash layout:

```sh
./build_ota.sh
```

The layout is `2 MB flash: 1 MB sketch + 1 MB LittleFS`. Output files are:

```text
build/ota/ups_battery_monitor.ino.bin
build/ota/ups_battery_monitor.ino.uf2
```

Do not build OTA releases with `2 MB (no FS)`. OTA staging and rollback will not work without the 1 MB LittleFS partition.

## First Installation

The OTA-capable firmware must be installed once over USB:

1. Hold BOOTSEL while connecting the Pico.
2. Copy `build/ota/ups_battery_monitor.ino.uf2` to the `RPI-RP2` volume.
3. Confirm the serial log or `/api/status` reports `Firmware: pve-ups-handoff-2026-09-20`.
4. Confirm Ethernet link and `192.168.6.122` are reachable.

Future updates can use Ethernet.

### Upgrading From A No-LittleFS Build

Check the `hardware.littlefs` field returned by `/api/status`. If it is `false`, the installed firmware uses the old `2 MB (no FS)` layout and cannot stage or roll back an Ethernet update. An attempted upload will be rejected without changing the running firmware:

```text
HTTP 507
{"ok":false,"error":"backup_failed"}
```

Install the UF2 once through BOOTSEL as described above. After `/api/status` reports `littlefs:true`, subsequent releases can use `./pico_admin.sh upload`.

## Ethernet Firmware Update

Build and upload:

```sh
./build_ota.sh
./pico_admin.sh upload
```

To upload a specific binary:

```sh
./pico_admin.sh upload /path/to/firmware.bin
```

The HTTP updater accepts the Arduino `.bin` file. Never upload a `.uf2` file to the HTTP endpoint.

The update token is stored in the untracked `ups_secrets.h`. Keep it private
and use a unique random value. The update API uses plain HTTP and is intended
for a trusted LAN.

Direct API request:

```text
POST /api/firmware?token=<UPS_OTA_TOKEN>
Content-Type: application/octet-stream
Content-Length: <firmware size>
X-Firmware-CRC32: <8 hexadecimal digits>

<raw Arduino .bin data>
```

`pico_admin.sh` first performs the RP2040 image checks locally, then calculates
and sends the CRC32 automatically. This also prevents an accidental `.uf2`
upload when the board is still running an older, permissive updater. Direct API
clients must provide the header; it detects corruption or a truncated/mixed
upload but is not a cryptographic signature. Authentication still depends on
the update token and the trusted LAN.

## Update Safety And Rollback

Firmware updates use the Arduino-Pico OTA bootloader:

1. The running image is copied to LittleFS as `/previous.bin`.
2. The previous rollback image remains recoverable while its replacement is committed.
3. The HTTP body is first written to `/firmware.tmp`, so an interrupted upload
   cannot become an installable candidate.
4. The candidate length, full-file CRC32, RP2040 boot2 CRC, initial stack
   pointer, and reset vector are validated. This rejects `.uf2`, random data,
   corruption, and incomplete `.bin` uploads.
5. Only a valid image is atomically renamed to `/firmware.bin`.
6. The boot-attempt marker must be written successfully before the PicoOTA
   bootloader command is committed.
7. The candidate is staged in LittleFS, then the Pico restarts.
8. The bootloader installs the candidate. Interrupted staging or installation is retried after power returns.
9. The candidate must run the main loop for 60 seconds with Ethernet link up.
10. An 8-second watchdog resets a candidate that hangs after startup.
11. After three failed candidate boots, `/previous.bin` is automatically restored.

This keeps the useful design from `JAndrassy/ArduinoOTA`—receive into a storage
object, close it, then apply it—while retaining this firmware's HTTP endpoint,
LittleFS rollback image, boot-attempt marker, watchdog, and automatic rollback.
The generic library's blocking receive loop was not copied; this implementation
retains a 10-second no-data timeout and verifies every filesystem write.

The health marker is only forgotten after `/ota.pending` is actually removed. A transient filesystem deletion failure is retried and cannot silently turn a healthy candidate into a later false rollback.

Useful OTA failure responses:

| HTTP | Error | Meaning |
| --- | --- | --- |
| `403` | `bad_token` | Authentication token did not match |
| `411` | `invalid_content_length` | Missing, too small, or oversized firmware body |
| `400` | `missing_firmware_crc32` | Client omitted the required `X-Firmware-CRC32` header |
| `409` | `candidate_not_healthy` | Current candidate is still inside its 60-second health window |
| `507` | `littlefs_unavailable` | Required OTA filesystem is not mounted |
| `507` | `backup_failed` | Running firmware could not be retained in LittleFS |
| `507` | `candidate_no_space` | LittleFS cannot retain both rollback and candidate images |
| `507` | `candidate_open_failed` | Candidate temporary file could not be created |
| `507` | `candidate_commit_failed` | Validated temporary file could not be atomically renamed |
| `507` | `pending_marker_failed` | Rollback boot marker could not be persisted |
| `507` | `staging_failed` | PicoOTA bootloader command could not be committed |
| `400` | `invalid_or_incomplete_firmware` | Transfer, CRC32, or RP2040 image validation failed |

Manual rollback:

```sh
./pico_admin.sh rollback
```

Application-managed rollback cannot recover code that fails before the OTA boot guard itself can execute. The RP2040 ROM BOOTSEL loader remains the final recovery method: hold BOOTSEL and install the last known-good UF2 over USB.

## Pico Soft Restart

A soft restart reboots only the Pico firmware; it is not an eight-second relay power cycle. During normal operation startup commands both configured UPS output relays on. After a low-battery cutoff, however, a persistent EEPROM lockout overrides that policy and keeps both outputs off across Pico resets and complete battery loss.

HTTP:

```sh
./pico_admin.sh restart
```

Direct endpoint:

```text
POST /api/system/restart?token=<UPS_OTA_TOKEN>
```

MQTT:

```text
Topic:   ups/system/restart/set
Payload: restart
```

Home Assistant MQTT discovery creates an `UPS Pico Restart` button automatically.

## Graceful Load Shutdown

At low battery, firmware does not immediately cut relay power:

1. Critical SOC, discharge current and non-charging voltage must persist for 30 seconds.
2. The configured shutdown HTTP endpoint must return an HTTP `2xx` response.
3. Failed shutdown requests are retried every 15 seconds.
4. After an accepted request, firmware waits at least 60 seconds. This is only
   a minimum delay: relay cutoff remains blocked for as long as the host-alive
   GPIO stays HIGH or its OFF state has not been stable for 30 seconds.
5. With ADS1115 present, each Pi relay can be removed only after its active-low PC817 output was first confirmed low while running and is then confirmed high for 30 seconds after shutdown.
6. The `10.8 V` emergency condition does not bypass this ADS host-off interlock.

The PVE-UPS 10% handoff uses the same ADS interlock. It does not replace or
weaken the independent 5% shutdown-request fallback described above.

The shutdown remains armed when host load falls during operating-system shutdown. Reduced discharge current alone is not interpreted as utility recovery. Cancellation requires at least `13.60 V` and non-discharging current continuously for 30 seconds.

Immediately before an ADS-authorized relay cutoff, the firmware stores a low-battery lockout flag in the AT24C256. On any subsequent Pico boot, the XL9535 output latch is prepared with both relay outputs off before those pins become outputs, avoiding an on/off boot pulse. The lockout is exposed by `/api/status` as `low_battery_lockout`.

Default shutdown endpoints are configured in `loadDefaultConfig()` and must correspond to real services that perform an orderly host shutdown.

For the Pi 3.3 V rail to fall after a clean halt, Raspberry Pi 4 bootloader
configuration must use `POWER_OFF_ON_HALT=1` and `WAKE_ON_GPIO=0`. Change these
only after the ADS wiring and live high reading have been verified. Otherwise a
halted Pi may not restart as expected.

### Operation Without ADS1115

Without ADS1115, firmware never turns a relay off during the outage, including
at `10.8 V`. If SOC reaches `3%` or less while utility voltage is absent, firmware records a
RAM-only assumption that the shutdown agents have already halted the Pis. When
utility recovery (`>=13.60 V`, current `>=-0.05 A`, SOC `>=7%`) then remains
valid for two minutes, each enabled relay that is still on is cycled off for
eight seconds and restored once, allowing the halted Pis to boot.

The latch is intentionally not stored in EEPROM. If the battery becomes fully
flat, the Pico and relay board lose power and the relays physically drop out.
When power returns, normal boot policy commands both relays on, which already
provides the required off-to-on restart; losing the RAM latch prevents an
unnecessary second cycle. A pre-existing persistent low-battery lockout still
uses the normal confirmed-utility restoration path.

### Automatic Power Restoration

Automatic restoration starts only after the low-battery sequence has completed and the relays were cut. The firmware does not treat a single voltage rise as proof that utility power is stable. All of the following evidence must remain true continuously:

| Condition | Required value |
| --- | --- |
| Battery voltage | At least `13.60 V` |
| Battery current | At least `-0.05 A` (not materially discharging) |
| Battery SOC | At least `7%` |
| Confirmation time | Two minutes |

If any condition fails during confirmation, the timer resets. After successful confirmation, both relays are enabled, the EEPROM lockout is cleared, and the hosts boot again. If only one relay write succeeds, that safely restored output remains on while firmware restarts confirmation and retries the missing output; it is never cut again merely to make the retry symmetrical. An explicit manual power-on command also clears the persistent lockout after the requested relay write succeeds.

Expected serial sequence:

```text
[LOW_BATTERY] Stage -> done.
[RELAY] Boot lockout active; outputs remain off pending confirmed utility recovery.
[AUTO_RESTORE] Utility recovery detected; confirming for 120s (...)
[AUTO_RESTORE] Utility recovery confirmed; enabling output relays.
[RELAY] Pi1 -> on ... (ok)
[RELAY] Pi2 -> on ... (ok)
[AUTO_RESTORE] Output relays restored; low-battery stage -> idle.
```

The automatic restoration path applies only to a low-battery cutoff. A manual graceful shutdown intentionally remains off until a manual power-on command or Pico restart. The Pico retries a manual host shutdown request until it receives HTTP `2xx`, waits the same 60-second minimum grace period, and only then disables that output relay if ADS confirms the Pi is already off. Direct `/power/off` requests are also subject to the ADS interlock and are intentionally not exposed as Home Assistant buttons.

## Main HTTP Endpoints

| Method | Path | Purpose |
| --- | --- | --- |
| GET | `/status` | Legacy battery status |
| GET | `/api/status` | Full battery, relay and hardware status |
| GET | `/api/battery/status` | Battery capacity status |
| GET | `/api/log` | Recent firmware/serial log from a bounded RAM buffer |
| POST | `/api/pi/1/power/on` | Enable Pi1 output |
| POST | `/api/pi/1/power/off` | Disable Pi1 output only when ADS host-off proof permits it |
| POST | `/api/pi/1/shutdown` | Gracefully shut down Pi1, then disable its relay |
| POST | `/api/pi/2/shutdown` | Gracefully shut down Pi2, then disable its relay |
| POST | `/api/pi/all/shutdown` | Gracefully shut down both hosts, then disable both relays |
| POST | `/api/pi/1/power-cycle` | Cycle Pi1 only when ADS host-off proof permits relay-off |
| POST | `/api/pi/2/power-cycle` | Cycle Pi2 only when ADS host-off proof permits relay-off |
| POST | `/api/pi/all/power-cycle` | Cycle both only when each ADS host-off proof permits relay-off |
| POST | `/api/firmware` | Stage a `.bin` firmware update; token required |
| POST | `/api/firmware/rollback` | Restore retained firmware; token required |
| POST | `/api/system/restart` | Soft-restart the Pico; token required |

The same recent diagnostic output shown over USB serial can be retrieved over
Ethernet without writing logs to flash:

```bash
curl http://192.168.6.122/api/log
./pico_admin.sh log
```

The endpoint returns the most recent 8 KiB as `text/plain`. Older lines are
discarded automatically when the RAM ring buffer fills.

## Diagnostics

A healthy I2C bus is idle high:

```text
[I2C] Idle levels before scan: SDA=1 SCL=1
```

The complete expected scan is normally:

```text
0x20
0x48
0x50
0x64
```

ADS1115 is optional for telemetry and no-ADS recovery mode, but required for
any relay cutoff while the UPS remains on battery. XL9535, AT24C256 and LTC2944
should remain detectable.
