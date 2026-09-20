#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <LittleFS.h>
#include <PicoOTA.h>
#include "ups_secrets.h"
#define MDNS_USING_ETHERNET true
#include <MDNS_Generic.h>

#include <math.h>
#include <string.h>
#include <strings.h>

// Mirror every firmware log line to USB serial and to a bounded RAM ring buffer.
// The HTTP log endpoint reads this buffer; no diagnostic output is written to
// LittleFS or EEPROM, avoiding flash wear during long-running operation.
class BufferedSerialLog : public Print {
 public:
  static const size_t CAPACITY = 8192;

  void begin(unsigned long baud) {
    ::Serial.begin(baud);
  }

  operator bool() const {
    return (bool)::Serial;
  }

  size_t write(uint8_t value) override {
    ::Serial.write(value);
    buffer_[head_] = (char)value;
    head_ = (head_ + 1U) % CAPACITY;
    if (size_ < CAPACITY) size_++;
    return 1;
  }

  using Print::write;

  size_t size() const {
    return size_;
  }

  char at(size_t index) const {
    if (index >= size_) return '\0';
    size_t oldest = (head_ + CAPACITY - size_) % CAPACITY;
    return buffer_[(oldest + index) % CAPACITY];
  }

 private:
  char buffer_[CAPACITY] = {0};
  size_t head_ = 0;
  size_t size_ = 0;
};

BufferedSerialLog bufferedSerialLog;
#define Serial bufferedSerialLog

const char* FIRMWARE_VERSION = "pve-ups-handoff-2026-09-20";

// ============================================================================
// Pico + W5500 wiring
// ============================================================================
#define I2C_SDA          12
#define I2C_SCL          13

#define W5500_CS         17
#define W5500_SCK        18
#define W5500_MOSI       19
#define W5500_MISO       16
#define W5500_RST        20

IPAddress FALLBACK_IP (192, 168, 6, 122);
IPAddress FALLBACK_DNS(192, 168, 6, 1);
IPAddress FALLBACK_GW (192, 168, 6, 1);
IPAddress FALLBACK_SN (255, 255, 255, 0);

byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x05 };

const char*     HOSTNAME  = "pico-ups";
const uint16_t  HTTP_PORT = 80;
const uint16_t  NUT_PORT = 3493;
const char*     NUT_UPS_NAME = "pico-ups";
const char*     NUT_UPS_DESCRIPTION = "Pico W5500 UPS";

IPAddress MQTT_BROKER_IP(192, 168, 6, 245);
const uint16_t MQTT_PORT = 1883;
const char* MQTT_USER = UPS_MQTT_USER;
const char* MQTT_PASSWORD = UPS_MQTT_PASSWORD;
const char* MQTT_CLIENT_ID = "pico-ups";
const char* MQTT_BASE_TOPIC = "ups/battery/main";
const char* MQTT_STATE_TOPIC = "ups/battery/main/state";
const char* MQTT_AVAILABILITY_TOPIC = "ups/battery/main/availability";
const char* MQTT_CALIBRATE_SET_TOPIC = "ups/battery/main/calibrate/set";
const char* MQTT_CALIBRATE_STATUS_TOPIC = "ups/battery/main/calibrate/status";
const char* MQTT_PI1_POWER_SET_TOPIC = "ups/pi/1/power/set";
const char* MQTT_PI2_POWER_SET_TOPIC = "ups/pi/2/power/set";
const char* MQTT_ALL_POWER_SET_TOPIC = "ups/pi/all/power/set";
const char* MQTT_POWER_STATUS_TOPIC = "ups/pi/power/status";
const char* MQTT_SYSTEM_RESTART_TOPIC = "ups/system/restart/set";
const char* MQTT_HOME_ASSISTANT_STATUS_TOPIC = "homeassistant/status";
const uint32_t MQTT_RECONNECT_INTERVAL_MS = 10000UL;

const char* OTA_PREVIOUS_FILE = "/previous.bin";
const char* OTA_PREVIOUS_TEMP_FILE = "/previous.tmp";
const char* OTA_PREVIOUS_OLD_FILE = "/previous.old";
const char* OTA_CANDIDATE_FILE = "/firmware.bin";
const char* OTA_CANDIDATE_TEMP_FILE = "/firmware.tmp";
const char* OTA_PENDING_FILE = "/ota.pending";
const uint8_t OTA_MAX_BOOT_ATTEMPTS = 3;
const uint32_t OTA_HEALTH_CONFIRM_MS = 60000UL;
const uint32_t OTA_WATCHDOG_MS = 8000UL;
const uint32_t OTA_UPLOAD_IDLE_TIMEOUT_MS = 10000UL;
const size_t OTA_MIN_IMAGE_SIZE = 4096UL;
const size_t OTA_MAX_IMAGE_SIZE = 900UL * 1024UL;
const size_t OTA_FS_RESERVE_BYTES = 8192UL;
const size_t OTA_RP2040_HEADER_SIZE = 264UL;

extern uint8_t __flash_binary_end;

// ============================================================================
// I2C peripherals
// ============================================================================
uint8_t LTC2944_ADDR = 0x64;
uint8_t ADS1115_ADDR = 0x48;
uint8_t XL9535_ADDR  = 0x20;
uint8_t AT24C256_ADDR = 0x50;

const uint8_t XL9535_RELAY_PIN_PI1 = 0;
const uint8_t XL9535_RELAY_PIN_PI2 = 1;
const bool RELAY_ACTIVE_HIGH = true;

const float LTC_SHUNT_OHMS       = 0.002f;
const float BATTERY_CAPACITY_AH  = 7.0f;
const float BATTERY_NOMINAL_V    = 12.0f;
const float USABLE_CAPACITY_PCT  = 0.75f;
const float CHARGE_EFFICIENCY    = 0.85f;
const float MIN_RUNTIME_LOAD_W   = 0.5f;
const float CURRENT_DEADBAND_A   = 0.05f;
const float FULL_FLOAT_VOLTAGE   = 13.40f;
const float FULL_CURRENT_A       = 0.25f;
const float CRITICAL_VOLTAGE     = 10.80f;
const uint32_t FULL_CONFIRM_MS   = 20UL * 60UL * 1000UL;

const uint32_t SAVE_INTERVAL_MS      = 10UL * 60UL * 1000UL;
const uint32_t MIN_SAVE_INTERVAL_MS  = 60UL * 1000UL;
const float    SAVE_DELTA_WH         = 0.5f;
const float    SAVE_DELTA_PERCENT    = 1.0f;
const uint32_t TELEMETRY_INTERVAL_MS = 5000UL;
const uint32_t POWER_CYCLE_OFF_MS    = 8000UL;
const uint32_t POWER_CYCLE_FAILSAFE_MS = 30000UL;
const uint32_t LOW_BATTERY_CONFIRM_MS = 30000UL;
const uint32_t SHUTDOWN_RETRY_MS = 15000UL;
const uint32_t SHUTDOWN_GRACE_MS = 60UL * 1000UL;
// PC817 outputs are active-low. The installed GPIO17 interface measures about
// 0.017 V while the Pi is running and about 0.58 V after the shutdown service
// releases the optocoupler. Keep a wide dead band between the calibrated ON
// and OFF thresholds so ambiguous readings fail closed.
const float ADS_PI_ON_MAX_V = 0.10f;
const float ADS_PI_OFF_MIN_V = 0.45f;
const float PC817_LOGICAL_HIGH_V = 3.30f;
const uint32_t ADS_PI_RAIL_ON_CONFIRM_MS = 10UL * 1000UL;
const uint32_t ADS_PI_RAIL_OFF_CONFIRM_MS = 30UL * 1000UL;
const float NO_ADS_RESTART_ARM_SOC = 3.0f;
const float AUTO_RESTORE_MIN_VOLTAGE = 13.60f;
const float AUTO_RESTORE_MIN_SOC = 7.0f;
const uint32_t AUTO_RESTORE_CONFIRM_MS = 2UL * 60UL * 1000UL;
const uint32_t UTILITY_RECOVERY_CANCEL_CONFIRM_MS = 30UL * 1000UL;
// A local W5500 link failure does not prove that Pi1/PixVirt is hung. Keep the
// destructive relay action disabled; link transitions are still monitored.
const bool ETH_LINK_WATCHDOG_ENABLED = false;
const uint8_t ETH_LINK_WATCHDOG_RELAY_INDEX = 0; // 0 = Pi1 relay, 1 = Pi2 relay
const uint32_t ETH_LINK_DOWN_POWER_CYCLE_MS = 90000UL;
const uint32_t ETH_LINK_POWER_CYCLE_COOLDOWN_MS = 300000UL;
const uint32_t I2C_BOOT_SETTLE_MS = 1500UL;
const uint32_t I2C_REDISCOVERY_INTERVAL_MS = 30000UL;
const uint32_t I2C_CLOCK_HZ = 50000UL;
const uint32_t NUT_ON_BATTERY_CONFIRM_MS = 5000UL;
const uint32_t NUT_UTILITY_RECOVERY_CONFIRM_MS = 30000UL;
// PVE-UPS is configured to request orderly host shutdown at 10% SOC. The Pico
// cannot authenticate that external request directly, so it only arms an
// expectation during a confirmed outage. Relay cutoff still requires the
// existing per-host ADS/PC817 running-to-final-OFF proof and the normal grace.
const float PVE_UPS_HANDOFF_SOC = 10.0f;
const uint32_t PVE_UPS_HANDOFF_RECOVERY_HOLD_MS = 5UL * 60UL * 1000UL;

const char* CALIBRATION_TOKEN = UPS_CALIBRATION_TOKEN;

const uint32_t PERSIST_MAGIC   = 0x55505332;  // "UPS2"
const uint16_t PERSIST_VERSION = 1;
const uint16_t PERSIST_FLAG_LOW_BATTERY_LOCKOUT = 1U << 0;
const uint16_t EEPROM_PAGE_SIZE = 64;

struct PiConfig {
  bool relayState;
  bool enabled;
  char shutdownUrl[96];
};

struct PersistedState {
  uint32_t magic;
  uint16_t version;
  uint16_t reserved0;
  float remainingWh;
  float remainingMah;
  float usableWh;
  float batteryCapacityAh;
  float usableCapacityPct;
  float shutdownSoc;
  float adsThresholdVolts;
  uint32_t shutdownDelaySec;
  uint32_t powerCycleMs;
  uint8_t relayActiveHigh;
  uint8_t relayPins[2];
  uint8_t relayState[2];
  uint8_t piEnabled[2];
  char shutdownUrl[2][96];
  uint32_t crc32;
};

struct SensorState {
  bool ltcPresent = false;
  bool adsPresent = false;
  bool ioPresent = false;
  bool eepromPresent = false;
  float vbus = 0.0f;
  float currentA = 0.0f;
  float powerW = 0.0f;
  float temperatureC = NAN;
  float soc = 50.0f;
  float remainingWh = (BATTERY_CAPACITY_AH * BATTERY_NOMINAL_V * USABLE_CAPACITY_PCT) * 0.5f;
  float remainingMah = (BATTERY_CAPACITY_AH * 1000.0f * USABLE_CAPACITY_PCT) * 0.5f;
  float runtimeMin = -1.0f;
  bool valid = false;
};

struct PiStatus {
  bool relayOn = true;
  bool sensedOn = false;
  bool sensedEstimated = true;
  float adsVolts = 0.0f;
};

enum BatteryFlowState : uint8_t {
  BATTERY_IDLE,
  BATTERY_CHARGING,
  BATTERY_DISCHARGING
};

enum NutPowerSource : uint8_t {
  NUT_POWER_UNKNOWN,
  NUT_POWER_MAINS,
  NUT_POWER_BATTERY
};

enum LowBatteryStage : uint8_t {
  LOW_BATTERY_IDLE,
  LOW_BATTERY_WAITING,
  LOW_BATTERY_DONE
};

enum PowerAction : uint8_t {
  POWER_ACTION_NONE,
  POWER_ACTION_RESTORE_ON
};

enum GracefulShutdownStage : uint8_t {
  GRACEFUL_SHUTDOWN_IDLE,
  GRACEFUL_SHUTDOWN_REQUESTING,
  GRACEFUL_SHUTDOWN_WAITING
};

struct GracefulShutdownState {
  GracefulShutdownStage stage = GRACEFUL_SHUTDOWN_IDLE;
  unsigned long lastAttemptMs = 0;
  unsigned long acceptedMs = 0;
};

struct PendingPowerAction {
  PowerAction action = POWER_ACTION_NONE;
  unsigned long dueMs = 0;
};

SensorState sensor;
PiConfig piConfig[2];
PiStatus piStatus[2];
PendingPowerAction powerActions[2];
GracefulShutdownState gracefulShutdowns[2];
unsigned long powerCycleStartedMs[2] = {0, 0};

EthernetServer httpServer(HTTP_PORT);
EthernetServer nutServer(NUT_PORT);
EthernetUDP mdnsUdp;
MDNS mdns(mdnsUdp);
EthernetClient mqttNetworkClient;
PubSubClient mqttClient(mqttNetworkClient);

unsigned long lastTelemetryMs = 0;
unsigned long lastMqttReconnectAttemptMs = 0;
unsigned long lastI2cRediscoveryMs = 0;
unsigned long ethLinkDownStartedMs = 0;
unsigned long lastEthLinkPowerCycleMs = 0;
bool lastEthLinkUp = false;
unsigned long lastSaveMs = 0;
unsigned long lastAnySaveMs = 0;
float lastSavedWh = 0.0f;
BatteryFlowState lastSavedState = BATTERY_IDLE;
unsigned long fullCandidateStartMs = 0;
unsigned long criticalCandidateStartMs = 0;
bool forceSave = false;

LowBatteryStage lowBatteryStage = LOW_BATTERY_IDLE;
unsigned long lowBatteryStageStartedMs = 0;
unsigned long autoRestoreCandidateStartMs = 0;
unsigned long shutdownRecoveryCandidateStartMs = 0;
bool lowBatteryLockout = false;
bool shutdownApiAttempted[2] = { false, false };
unsigned long shutdownAcceptedMs[2] = { 0, 0 };
unsigned long lastShutdownRetryMs = 0;
bool adsMissingLogged = false;
bool ltcReadFailureLogged = false;
bool adsReadFailureLogged[2] = { false, false };
bool lastPiSensedState[2] = { false, false };
bool lastPiSensedStateKnown[2] = { false, false };
bool adsPiRailSeenOn[2] = { false, false };
bool adsPiRailOffConfirmed[2] = { false, false };
unsigned long adsPiRailOnCandidateStartMs[2] = { 0, 0 };
unsigned long adsPiRailOffCandidateStartMs[2] = { 0, 0 };
unsigned long lastRelayCutoffBlockedLogMs[2] = { 0, 0 };
unsigned long lastLowBatteryCutoffBlockedLogMs = 0;
bool pveUpsHandoffArmed = false;
unsigned long pveUpsHandoffArmedMs = 0;
unsigned long pveUpsHandoffRecoveryStartMs = 0;
bool noAdsRestartPending = false;
bool noAdsRecoveryCycleStarted = false;
bool noAdsRecoveryCycleRequested[2] = { false, false };
unsigned long noAdsRecoveryCandidateStartMs = 0;
bool otaPendingBoot = false;
bool littleFsAvailable = false;
unsigned long otaBootStartedMs = 0;
bool restartRequested = false;
unsigned long restartDueMs = 0;
NutPowerSource nutPowerSource = NUT_POWER_UNKNOWN;
unsigned long nutOnBatteryCandidateStartMs = 0;
unsigned long nutUtilityRecoveryCandidateStartMs = 0;

// ============================================================================
// Utility
// ============================================================================
static float usableEnergyWh() {
  return BATTERY_CAPACITY_AH * BATTERY_NOMINAL_V * USABLE_CAPACITY_PCT;
}

static uint32_t crc32Update(uint32_t crc, uint8_t data) {
  crc ^= data;
  for (uint8_t i = 0; i < 8; i++) {
    crc = (crc & 1U) ? (crc >> 1U) ^ 0xEDB88320UL : (crc >> 1U);
  }
  return crc;
}

static uint32_t crc32Bytes(const uint8_t *data, size_t len) {
  uint32_t crc = 0xFFFFFFFFUL;
  for (size_t i = 0; i < len; i++) crc = crc32Update(crc, data[i]);
  return ~crc;
}

static uint32_t readLittleEndian32(const uint8_t *data) {
  return (uint32_t)data[0] |
         ((uint32_t)data[1] << 8U) |
         ((uint32_t)data[2] << 16U) |
         ((uint32_t)data[3] << 24U);
}

// RP2040 ROM validates the first-stage bootloader with CRC-32/MPEG-2 over
// bytes 0..251; bytes 252..255 contain the little-endian expected CRC.
static uint32_t rp2040Boot2Crc(const uint8_t *data) {
  uint32_t crc = 0xFFFFFFFFUL;
  for (size_t i = 0; i < 252; i++) {
    crc ^= (uint32_t)data[i] << 24U;
    for (uint8_t bit = 0; bit < 8; bit++) {
      crc = (crc & 0x80000000UL) ? (crc << 1U) ^ 0x04C11DB7UL : crc << 1U;
    }
  }
  return crc;
}

static bool otaValidateRp2040ImageHeader(const uint8_t *header, size_t imageSize) {
  if (imageSize < OTA_RP2040_HEADER_SIZE || (imageSize & 3U) != 0) return false;

  uint32_t expectedBoot2Crc = readLittleEndian32(header + 252);
  if (rp2040Boot2Crc(header) != expectedBoot2Crc) return false;

  // The vector table follows boot2 at offset 256 in an Arduino-Pico .bin.
  uint32_t initialStack = readLittleEndian32(header + 256);
  uint32_t resetVector = readLittleEndian32(header + 260);
  uint32_t resetAddress = resetVector & ~1UL;
  bool stackValid = initialStack >= 0x20000000UL && initialStack <= 0x20042000UL &&
                    (initialStack & 7U) == 0;
  bool resetValid = (resetVector & 1U) != 0 &&
                    resetAddress >= XIP_BASE + 256UL &&
                    resetAddress < XIP_BASE + imageSize;
  return stackValid && resetValid;
}

static void recoverI2cBusPins() {
  pinMode(I2C_SDA, INPUT_PULLUP);
  pinMode(I2C_SCL, INPUT_PULLUP);
  delay(20);

  Serial.printf("[I2C] Idle levels before recovery: SDA=%u SCL=%u\n",
                digitalRead(I2C_SDA) ? 1 : 0,
                digitalRead(I2C_SCL) ? 1 : 0);

  if (digitalRead(I2C_SDA) == LOW) {
    Serial.println("[I2C] SDA held low; pulsing SCL.");
    pinMode(I2C_SCL, OUTPUT);
    for (uint8_t i = 0; i < 9; i++) {
      digitalWrite(I2C_SCL, HIGH);
      delayMicroseconds(8);
      digitalWrite(I2C_SCL, LOW);
      delayMicroseconds(8);
    }
    digitalWrite(I2C_SCL, HIGH);
    delayMicroseconds(8);
  }

  pinMode(I2C_SDA, INPUT_PULLUP);
  pinMode(I2C_SCL, INPUT_PULLUP);
  delay(20);
  Serial.printf("[I2C] Idle levels after recovery: SDA=%u SCL=%u\n",
                digitalRead(I2C_SDA) ? 1 : 0,
                digitalRead(I2C_SCL) ? 1 : 0);
}

static bool i2cDevicePresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

static void i2cScan() {
  Serial.printf("[I2C] Idle levels before scan: SDA=%u SCL=%u\n",
                digitalRead(I2C_SDA) ? 1 : 0,
                digitalRead(I2C_SCL) ? 1 : 0);
  Serial.println("I2C scan:");
  uint8_t count = 0;
  for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
    if (i2cDevicePresent(addr)) {
      Serial.printf("  0x%02X\n", addr);
      count++;
    }
  }
  if (count == 0) Serial.println("  (no devices detected)");
}

static void logModulePresence(const char *name, uint8_t addr, bool present) {
  Serial.printf("[DISCOVERY] %s at 0x%02X: %s\n", name, addr, present ? "found" : "missing");
}

static bool findDeviceInRange(const char *name, uint8_t startAddr, uint8_t endAddr, uint8_t &foundAddr) {
  Serial.printf("[DISCOVERY] Searching %s from 0x%02X to 0x%02X\n", name, startAddr, endAddr);
  for (uint8_t addr = startAddr; addr <= endAddr; addr++) {
    if (i2cDevicePresent(addr)) {
      foundAddr = addr;
      Serial.printf("[DISCOVERY] %s matched at 0x%02X\n", name, addr);
      return true;
    }
  }
  Serial.printf("[DISCOVERY] %s not found in range 0x%02X-0x%02X\n", name, startAddr, endAddr);
  return false;
}

static BatteryFlowState batteryFlowState(float currentA) {
  if (currentA < -CURRENT_DEADBAND_A) return BATTERY_DISCHARGING;
  if (currentA >  CURRENT_DEADBAND_A) return BATTERY_CHARGING;
  return BATTERY_IDLE;
}

static void updateNutPowerSource() {
  unsigned long now = millis();

  // A committed low-battery shutdown is still an outage until the existing
  // relay-restore state machine has confirmed utility recovery and re-enabled
  // the outputs. This also preserves the outage across a Pico reboot.
  if (lowBatteryStage != LOW_BATTERY_IDLE || lowBatteryLockout) {
    nutPowerSource = NUT_POWER_BATTERY;
    nutOnBatteryCandidateStartMs = 0;
    nutUtilityRecoveryCandidateStartMs = 0;
    return;
  }

  if (!sensor.valid) {
    nutOnBatteryCandidateStartMs = 0;
    nutUtilityRecoveryCandidateStartMs = 0;
    return;
  }

  const bool discharging = sensor.currentA < -CURRENT_DEADBAND_A;
  const bool charging = sensor.currentA > CURRENT_DEADBAND_A;
  const bool utilityRecovery = sensor.vbus >= AUTO_RESTORE_MIN_VOLTAGE &&
                               sensor.currentA >= -CURRENT_DEADBAND_A;

  if (nutPowerSource == NUT_POWER_BATTERY) {
    nutOnBatteryCandidateStartMs = 0;
    if (!utilityRecovery) {
      nutUtilityRecoveryCandidateStartMs = 0;
      return;
    }

    if (nutUtilityRecoveryCandidateStartMs == 0) {
      nutUtilityRecoveryCandidateStartMs = now;
      return;
    }
    if (now - nutUtilityRecoveryCandidateStartMs < NUT_UTILITY_RECOVERY_CONFIRM_MS) return;

    nutPowerSource = NUT_POWER_MAINS;
    nutUtilityRecoveryCandidateStartMs = 0;
    Serial.println("[NUT] Utility recovery confirmed; status -> OL.");
    return;
  }

  nutUtilityRecoveryCandidateStartMs = 0;
  if (discharging) {
    if (nutOnBatteryCandidateStartMs == 0) {
      nutOnBatteryCandidateStartMs = now;
      return;
    }
    if (now - nutOnBatteryCandidateStartMs < NUT_ON_BATTERY_CONFIRM_MS) return;

    nutPowerSource = NUT_POWER_BATTERY;
    nutOnBatteryCandidateStartMs = 0;
    Serial.println("[NUT] Battery discharge confirmed; status -> OB.");
    return;
  }

  nutOnBatteryCandidateStartMs = 0;
  // Positive charge current is direct evidence that the charger is present.
  // Near-zero current is accepted as mains only at charger-level voltage.
  if (charging || utilityRecovery) {
    const bool changed = nutPowerSource != NUT_POWER_MAINS;
    nutPowerSource = NUT_POWER_MAINS;
    if (changed) Serial.println("[NUT] Charger/utility detected; status -> OL.");
  }
}

static float runtimeMinutes(float remainingWh, float loadW) {
  loadW = fabsf(loadW);
  if (loadW < MIN_RUNTIME_LOAD_W) return -1.0f;
  return remainingWh / loadW * 60.0f;
}

static bool otaWritePendingAttempts(uint8_t attempts) {
  File f = LittleFS.open(OTA_PENDING_FILE, "w");
  if (!f) return false;
  bool ok = f.write(&attempts, 1) == 1;
  f.close();
  return ok;
}

static bool otaScheduleFile(const char *path) {
  if (!LittleFS.exists(path)) return false;
  picoOTA.begin();
  if (!picoOTA.addFile(path)) return false;
  return picoOTA.commit();
}

static void otaBootGuardBegin() {
  littleFsAvailable = LittleFS.begin();
  if (!littleFsAvailable) {
    Serial.println("[OTA] LittleFS mount failed; formatting filesystem.");
    if (LittleFS.format()) {
      littleFsAvailable = LittleFS.begin();
    }
    if (!littleFsAvailable) {
      Serial.println("[OTA] LittleFS unavailable after format; OTA disabled.");
      return;
    }
    Serial.println("[OTA] LittleFS formatted and mounted.");
  }
  if (!LittleFS.exists(OTA_PENDING_FILE)) return;

  uint8_t attempts = 0;
  File f = LittleFS.open(OTA_PENDING_FILE, "r");
  if (f) {
    f.read(&attempts, 1);
    f.close();
  }
  attempts++;
  Serial.printf("[OTA] Candidate boot attempt %u/%u\n", attempts, OTA_MAX_BOOT_ATTEMPTS);

  if (attempts >= OTA_MAX_BOOT_ATTEMPTS && LittleFS.exists(OTA_PREVIOUS_FILE)) {
    LittleFS.remove(OTA_PENDING_FILE);
    if (otaScheduleFile(OTA_PREVIOUS_FILE)) {
      Serial.println("[OTA] Candidate failed repeatedly; rolling back to previous firmware.");
      delay(100);
      rp2040.reboot();
    }
    Serial.println("[OTA] Rollback scheduling failed.");
  }

  otaWritePendingAttempts(attempts);
  otaPendingBoot = true;
  otaBootStartedMs = millis();
}

static void otaMarkHealthyIfReady() {
  if (!otaPendingBoot || millis() - otaBootStartedMs < OTA_HEALTH_CONFIRM_MS) return;
  if (Ethernet.linkStatus() == LinkOFF) {
    Serial.println("[OTA] Candidate health check failed: Ethernet link is down; rebooting.");
    delay(100);
    rp2040.reboot();
  }
  if (!LittleFS.exists(OTA_PENDING_FILE) || LittleFS.remove(OTA_PENDING_FILE)) {
    if (LittleFS.exists(OTA_CANDIDATE_FILE) && !LittleFS.remove(OTA_CANDIDATE_FILE)) {
      Serial.println("[OTA] Candidate marked healthy; staged image cleanup failed.");
    } else {
      Serial.println("[OTA] Candidate marked healthy; rollback image retained.");
    }
    otaPendingBoot = false;
    return;
  }
  // Do not forget the in-RAM pending state when the persistent marker could
  // not be removed. Otherwise a later reset would count this healthy image as
  // another failed candidate boot and could trigger an incorrect rollback.
  Serial.println("[OTA] Candidate is healthy, but ota.pending removal failed; will retry.");
  otaBootStartedMs = millis();
}

static bool otaBackupRunningFirmware() {
  if (!littleFsAvailable) return false;
  const uint8_t *imageStart = reinterpret_cast<const uint8_t *>(XIP_BASE);
  size_t imageSize = reinterpret_cast<uintptr_t>(&__flash_binary_end) - XIP_BASE;
  if (imageSize == 0 || imageSize > 1024UL * 1024UL) return false;

  LittleFS.remove(OTA_PREVIOUS_TEMP_FILE);
  File f = LittleFS.open(OTA_PREVIOUS_TEMP_FILE, "w");
  if (!f) return false;
  uint8_t buffer[1024];
  size_t written = 0;
  while (written < imageSize) {
    size_t chunk = min(sizeof(buffer), imageSize - written);
    memcpy(buffer, imageStart + written, chunk);
    if (f.write(buffer, chunk) != chunk) {
      f.close();
      LittleFS.remove(OTA_PREVIOUS_TEMP_FILE);
      return false;
    }
    written += chunk;
    rp2040.wdt_reset();
  }
  f.close();
  // Preserve the last known rollback image until the newly copied image is
  // safely in place. A failed rename must not leave the unit without either
  // the old or the new rollback image.
  if (!LittleFS.exists(OTA_PREVIOUS_FILE) && LittleFS.exists(OTA_PREVIOUS_OLD_FILE) &&
      !LittleFS.rename(OTA_PREVIOUS_OLD_FILE, OTA_PREVIOUS_FILE)) {
    LittleFS.remove(OTA_PREVIOUS_TEMP_FILE);
    Serial.println("[OTA] Previous firmware backup failed while recovering old rollback image.");
    return false;
  }
  if (LittleFS.exists(OTA_PREVIOUS_FILE)) LittleFS.remove(OTA_PREVIOUS_OLD_FILE);
  bool hadPrevious = LittleFS.exists(OTA_PREVIOUS_FILE);
  if (hadPrevious && !LittleFS.rename(OTA_PREVIOUS_FILE, OTA_PREVIOUS_OLD_FILE)) {
    LittleFS.remove(OTA_PREVIOUS_TEMP_FILE);
    Serial.println("[OTA] Previous firmware backup failed while preserving old rollback image.");
    return false;
  }

  bool ok = LittleFS.rename(OTA_PREVIOUS_TEMP_FILE, OTA_PREVIOUS_FILE);
  if (ok) {
    LittleFS.remove(OTA_PREVIOUS_OLD_FILE);
  } else if (hadPrevious) {
    LittleFS.rename(OTA_PREVIOUS_OLD_FILE, OTA_PREVIOUS_FILE);
  }
  Serial.printf("[OTA] Previous firmware backup %s (%lu bytes).\n",
                ok ? "saved" : "failed", (unsigned long)imageSize);
  return ok;
}

// ============================================================================
// AT24C256 persistence
// ============================================================================
static bool at24WriteChunk(uint16_t addr, const uint8_t *data, size_t len) {
  Wire.beginTransmission(AT24C256_ADDR);
  Wire.write((uint8_t)(addr >> 8));
  Wire.write((uint8_t)(addr & 0xFF));
  for (size_t i = 0; i < len; i++) Wire.write(data[i]);
  if (Wire.endTransmission() != 0) return false;
  delay(6);
  return true;
}

static bool at24WriteBytes(uint16_t addr, const uint8_t *data, size_t len) {
  size_t offset = 0;
  while (offset < len) {
    uint16_t pageOffset = (addr + offset) % EEPROM_PAGE_SIZE;
    size_t chunk = min((size_t)(EEPROM_PAGE_SIZE - pageOffset), len - offset);
    if (!at24WriteChunk(addr + offset, data + offset, chunk)) return false;
    offset += chunk;
  }
  return true;
}

static bool at24ReadBytes(uint16_t addr, uint8_t *data, size_t len) {
  Wire.beginTransmission(AT24C256_ADDR);
  Wire.write((uint8_t)(addr >> 8));
  Wire.write((uint8_t)(addr & 0xFF));
  if (Wire.endTransmission(false) != 0) return false;

  size_t offset = 0;
  while (offset < len) {
    size_t chunk = min((size_t)32, len - offset);
    size_t read = Wire.requestFrom((int)AT24C256_ADDR, (int)chunk);
    if (read != chunk) return false;
    for (size_t i = 0; i < chunk; i++) data[offset + i] = Wire.read();
    offset += chunk;
  }
  return true;
}

static void loadDefaultConfig() {
  memset(piConfig, 0, sizeof(piConfig));

  piConfig[0].relayState = true;
  piConfig[0].enabled = true;
  strncpy(piConfig[0].shutdownUrl, "http://192.168.6.10/api/shutdown", sizeof(piConfig[0].shutdownUrl) - 1);

  piConfig[1].relayState = true;
  piConfig[1].enabled = true;
  strncpy(piConfig[1].shutdownUrl, "http://192.168.6.11/api/shutdown", sizeof(piConfig[1].shutdownUrl) - 1);

  sensor.remainingWh = usableEnergyWh() * 0.5f;
  sensor.remainingMah = BATTERY_CAPACITY_AH * 1000.0f * USABLE_CAPACITY_PCT * 0.5f;
}

static void applyPersistedState(const PersistedState &state) {
  sensor.remainingWh = constrain(state.remainingWh, 0.0f, state.usableWh > 0.0f ? state.usableWh : usableEnergyWh());
  sensor.remainingMah = max(0.0f, state.remainingMah);

  lowBatteryLockout = (state.reserved0 & PERSIST_FLAG_LOW_BATTERY_LOCKOUT) != 0;
  for (uint8_t i = 0; i < 2; i++) {
    piConfig[i].relayState = lowBatteryLockout ? false : true;
    piConfig[i].enabled = state.piEnabled[i];
    strncpy(piConfig[i].shutdownUrl, state.shutdownUrl[i], sizeof(piConfig[i].shutdownUrl) - 1);
    piConfig[i].shutdownUrl[sizeof(piConfig[i].shutdownUrl) - 1] = '\0';
  }
}

static bool loadPersistedState() {
  loadDefaultConfig();
  if (!sensor.eepromPresent) {
    Serial.println("[EEPROM] AT24C256 missing, using defaults.");
    return false;
  }

  PersistedState state;
  if (!at24ReadBytes(0, reinterpret_cast<uint8_t*>(&state), sizeof(state))) {
    Serial.println("[EEPROM] Read failed, using defaults.");
    return false;
  }
  if (state.magic != PERSIST_MAGIC || state.version != PERSIST_VERSION) {
    Serial.printf("[EEPROM] Invalid header magic/version (magic=0x%08lX version=%u), using defaults.\n",
                  (unsigned long)state.magic, state.version);
    return false;
  }

  uint32_t expected = state.crc32;
  state.crc32 = 0;
  uint32_t actual = crc32Bytes(reinterpret_cast<const uint8_t*>(&state), sizeof(state));
  if (expected != actual) {
    Serial.printf("[EEPROM] CRC mismatch (stored=0x%08lX actual=0x%08lX), using defaults.\n",
                  (unsigned long)expected, (unsigned long)actual);
    return false;
  }

  applyPersistedState(state);
  Serial.printf("[EEPROM] Restored state: remainingWh=%.2f Pi1=%s Pi2=%s lowBatteryLockout=%s\n",
                sensor.remainingWh,
                piConfig[0].relayState ? "on" : "off",
                piConfig[1].relayState ? "on" : "off",
                lowBatteryLockout ? "true" : "false");
  return true;
}

static bool savePersistedState() {
  if (!sensor.eepromPresent) {
    Serial.println("[EEPROM] Save skipped, AT24C256 missing.");
    return false;
  }

  PersistedState state{};
  state.magic = PERSIST_MAGIC;
  state.version = PERSIST_VERSION;
  state.reserved0 = lowBatteryLockout ? PERSIST_FLAG_LOW_BATTERY_LOCKOUT : 0;
  state.remainingWh = sensor.remainingWh;
  state.remainingMah = sensor.remainingMah;
  state.usableWh = usableEnergyWh();
  state.batteryCapacityAh = BATTERY_CAPACITY_AH;
  state.usableCapacityPct = USABLE_CAPACITY_PCT;
  state.shutdownSoc = 5.0f;
  state.adsThresholdVolts = 1.0f;
  state.shutdownDelaySec = SHUTDOWN_GRACE_MS / 1000UL;
  state.powerCycleMs = 8000;
  state.relayActiveHigh = RELAY_ACTIVE_HIGH ? 1 : 0;
  state.relayPins[0] = XL9535_RELAY_PIN_PI1;
  state.relayPins[1] = XL9535_RELAY_PIN_PI2;

  for (uint8_t i = 0; i < 2; i++) {
    state.relayState[i] = piConfig[i].relayState;
    state.piEnabled[i] = piConfig[i].enabled;
    strncpy(state.shutdownUrl[i], piConfig[i].shutdownUrl, sizeof(state.shutdownUrl[i]) - 1);
  }

  state.crc32 = 0;
  state.crc32 = crc32Bytes(reinterpret_cast<const uint8_t*>(&state), sizeof(state));

  bool ok = at24WriteBytes(0, reinterpret_cast<const uint8_t*>(&state), sizeof(state));
  Serial.printf("[EEPROM] Save %s: remainingWh=%.2f Pi1=%s Pi2=%s lowBatteryLockout=%s\n",
                ok ? "ok" : "failed",
                sensor.remainingWh,
                piConfig[0].relayState ? "on" : "off",
                piConfig[1].relayState ? "on" : "off",
                lowBatteryLockout ? "true" : "false");
  return ok;
}

static bool shouldSaveState() {
  if (forceSave) return true;

  unsigned long now = millis();
  float percentNow = usableEnergyWh() > 0.0f ? sensor.remainingWh / usableEnergyWh() * 100.0f : 0.0f;
  float lastPercent = usableEnergyWh() > 0.0f ? lastSavedWh / usableEnergyWh() * 100.0f : 0.0f;
  bool intervalPassed = now - lastSaveMs >= SAVE_INTERVAL_MS;
  bool meaningful = fabsf(sensor.remainingWh - lastSavedWh) >= SAVE_DELTA_WH ||
                    fabsf(percentNow - lastPercent) >= SAVE_DELTA_PERCENT;
  bool stateChanged = batteryFlowState(sensor.currentA) != lastSavedState;
  bool spacingOk = now - lastAnySaveMs >= MIN_SAVE_INTERVAL_MS;

  if (stateChanged && spacingOk) return true;
  if (intervalPassed && meaningful) return true;
  return false;
}

static void savePersistedStateIfNeeded() {
  if (!shouldSaveState()) return;
  if (!savePersistedState()) return;

  unsigned long now = millis();
  lastSaveMs = now;
  lastAnySaveMs = now;
  lastSavedWh = sensor.remainingWh;
  lastSavedState = batteryFlowState(sensor.currentA);
  forceSave = false;
}

// ============================================================================
// XL9535 relay control
// ============================================================================
static uint16_t xl9535OutputState = 0xFFFF;
static uint16_t xl9535ConfigState = 0xFFFF;

static bool xl9535WriteRegPair(uint8_t regBase, uint16_t value) {
  Wire.beginTransmission(XL9535_ADDR);
  Wire.write(regBase);
  Wire.write((uint8_t)(value & 0xFF));
  Wire.write((uint8_t)((value >> 8) & 0xFF));
  return Wire.endTransmission() == 0;
}

static bool xl9535Init() {
  if (!sensor.ioPresent) {
    Serial.println("[XL9535] Init skipped, device missing.");
    return false;
  }
  // Program the output latch before changing the relay pins to outputs. This
  // prevents an EEPROM-restored low-battery lockout from briefly energizing
  // the relays during boot.
  xl9535OutputState = 0xFFFF;
  const uint8_t relayPins[2] = { XL9535_RELAY_PIN_PI1, XL9535_RELAY_PIN_PI2 };
  for (uint8_t i = 0; i < 2; i++) {
    bool rawLevel = RELAY_ACTIVE_HIGH ? piConfig[i].relayState : !piConfig[i].relayState;
    if (rawLevel) xl9535OutputState |= (1U << relayPins[i]);
    else xl9535OutputState &= ~(1U << relayPins[i]);
  }
  xl9535WriteRegPair(0x02, xl9535OutputState);
  xl9535ConfigState |= (1U << XL9535_RELAY_PIN_PI1);
  xl9535ConfigState |= (1U << XL9535_RELAY_PIN_PI2);
  xl9535ConfigState &= ~(1U << XL9535_RELAY_PIN_PI1);
  xl9535ConfigState &= ~(1U << XL9535_RELAY_PIN_PI2);
  bool ok = xl9535WriteRegPair(0x06, xl9535ConfigState);
  Serial.printf("[XL9535] Init %s output=0x%04X config=0x%04X\n",
                ok ? "ok" : "failed", xl9535OutputState, xl9535ConfigState);
  return ok;
}

static bool setRelayLevelRaw(uint8_t pin, bool level) {
  if (!sensor.ioPresent) return false;
  if (level) xl9535OutputState |= (1U << pin);
  else xl9535OutputState &= ~(1U << pin);
  return xl9535WriteRegPair(0x02, xl9535OutputState);
}

static void resetAdsPiRailProof(uint8_t piIndex) {
  if (piIndex >= 2) return;
  adsPiRailSeenOn[piIndex] = false;
  adsPiRailOffConfirmed[piIndex] = false;
  adsPiRailOnCandidateStartMs[piIndex] = 0;
  adsPiRailOffCandidateStartMs[piIndex] = 0;
}

static void updateAdsPiRailProof(uint8_t piIndex, bool valid, float volts) {
  if (piIndex >= 2) return;
  if (!valid || !isfinite(volts)) {
    // Fail closed. Once the ADS or this channel disappears, a fresh, stable
    // active-low running observation is required before a later high reading
    // can authorize removal of power. Relay-command state never counts.
    resetAdsPiRailProof(piIndex);
    return;
  }

  const unsigned long now = millis();
  if (volts <= ADS_PI_ON_MAX_V) {
    adsPiRailOffCandidateStartMs[piIndex] = 0;
    adsPiRailOffConfirmed[piIndex] = false;
    if (adsPiRailOnCandidateStartMs[piIndex] == 0) {
      adsPiRailOnCandidateStartMs[piIndex] = now;
    } else if (!adsPiRailSeenOn[piIndex] &&
               now - adsPiRailOnCandidateStartMs[piIndex] >= ADS_PI_RAIL_ON_CONFIRM_MS) {
      adsPiRailSeenOn[piIndex] = true;
      Serial.printf("[ADS1115] Pi%u active-low running state confirmed at %.3fV.\n",
                    piIndex + 1, volts);
    }
    return;
  }

  adsPiRailOnCandidateStartMs[piIndex] = 0;
  if (volts >= ADS_PI_OFF_MIN_V && adsPiRailSeenOn[piIndex]) {
    if (adsPiRailOffCandidateStartMs[piIndex] == 0) {
      adsPiRailOffCandidateStartMs[piIndex] = now;
      adsPiRailOffConfirmed[piIndex] = false;
      Serial.printf("[ADS1115] Pi%u PC817 output high at %.3fV; confirming host-off for %lus.\n",
                    piIndex + 1,
                    volts,
                    (unsigned long)(ADS_PI_RAIL_OFF_CONFIRM_MS / 1000UL));
    } else if (!adsPiRailOffConfirmed[piIndex] &&
               now - adsPiRailOffCandidateStartMs[piIndex] >= ADS_PI_RAIL_OFF_CONFIRM_MS) {
      adsPiRailOffConfirmed[piIndex] = true;
      Serial.printf("[ADS1115] Pi%u host-off confirmed from PC817 output at %.3fV.\n",
                    piIndex + 1, volts);
    }
    return;
  }

  // The indeterminate band is neither a valid powered rail nor a safe halt.
  adsPiRailOffCandidateStartMs[piIndex] = 0;
  adsPiRailOffConfirmed[piIndex] = false;
}

static bool relayCutoffConfirmedByAds(uint8_t piIndex) {
  return piIndex < 2 &&
         sensor.adsPresent &&
         !piStatus[piIndex].sensedEstimated &&
         adsPiRailSeenOn[piIndex] &&
         adsPiRailOffConfirmed[piIndex] &&
         piStatus[piIndex].adsVolts >= ADS_PI_OFF_MIN_V;
}

static float displayedPiVoltage(uint8_t piIndex) {
  if (piIndex >= 2 || !sensor.adsPresent || piStatus[piIndex].sensedEstimated) return 0.0f;
  return constrain(PC817_LOGICAL_HIGH_V - piStatus[piIndex].adsVolts,
                   0.0f,
                   PC817_LOGICAL_HIGH_V);
}

static bool utilityPowerRecoveryNow();
static bool allRelayCutoffsConfirmedByAds();

static bool noAdsRecoveryCycleAuthorized() {
  return !sensor.adsPresent &&
         noAdsRestartPending &&
         noAdsRecoveryCandidateStartMs != 0 &&
         millis() - noAdsRecoveryCandidateStartMs >= AUTO_RESTORE_CONFIRM_MS &&
         utilityPowerRecoveryNow();
}

static bool applyRelayState(uint8_t piIndex, bool on, bool allowNoAdsRecoveryCycle = false) {
  if (piIndex >= 2) return false;
  const bool noAdsRecoveryBypass = allowNoAdsRecoveryCycle &&
                                   noAdsRecoveryCycleAuthorized();
  if (!on && !relayCutoffConfirmedByAds(piIndex) && !noAdsRecoveryBypass) {
    const unsigned long now = millis();
    if (lastRelayCutoffBlockedLogMs[piIndex] == 0 ||
        now - lastRelayCutoffBlockedLogMs[piIndex] >= 30000UL) {
      Serial.printf(
        "[RELAY] Pi%u off blocked: requires PC817 output <=%.2fV while on, then >=%.2fV for %lus.\n",
        piIndex + 1,
        ADS_PI_ON_MAX_V,
        ADS_PI_OFF_MIN_V,
        (unsigned long)(ADS_PI_RAIL_OFF_CONFIRM_MS / 1000UL));
      lastRelayCutoffBlockedLogMs[piIndex] = now;
    }
    return false;
  }
  uint8_t pin = (piIndex == 0) ? XL9535_RELAY_PIN_PI1 : XL9535_RELAY_PIN_PI2;
  bool level = RELAY_ACTIVE_HIGH ? on : !on;
  bool ok = setRelayLevelRaw(pin, level);
  if (ok) {
    piConfig[piIndex].relayState = on;
    piStatus[piIndex].relayOn = on;
    if (!sensor.adsPresent) {
      piStatus[piIndex].sensedOn = on;
      piStatus[piIndex].sensedEstimated = true;
      piStatus[piIndex].adsVolts = 0.0f;
    }
    if (on) resetAdsPiRailProof(piIndex);
    forceSave = true;
  }
  Serial.printf("[RELAY] Pi%u -> %s raw_level=%u output=0x%04X (%s)\n",
                piIndex + 1,
                on ? "on" : "off",
                level ? 1 : 0,
                xl9535OutputState,
                ok ? "ok" : "failed");
  return ok;
}

static bool batteryCriticalNow() {
  return sensor.valid &&
         sensor.vbus < FULL_FLOAT_VOLTAGE &&
         sensor.currentA < -CURRENT_DEADBAND_A &&
         sensor.soc <= 5.0f;
}

static void setBootRelayTargets() {
  if (lowBatteryLockout) {
    for (uint8_t i = 0; i < 2; i++) piConfig[i].relayState = false;
    lowBatteryStage = LOW_BATTERY_DONE;
    Serial.println("[RELAY] Boot lockout active; outputs remain off pending confirmed utility recovery.");
    return;
  }
  for (uint8_t i = 0; i < 2; i++) {
    piConfig[i].relayState = true;
  }
  Serial.println("[RELAY] Boot output enabled; low-battery protection requires confirmation.");
}

// ============================================================================
// ADS1115
// ============================================================================
static bool adsWriteConfig(uint16_t config) {
  Wire.beginTransmission(ADS1115_ADDR);
  Wire.write(0x01);
  Wire.write((uint8_t)(config >> 8));
  Wire.write((uint8_t)(config & 0xFF));
  return Wire.endTransmission() == 0;
}

static bool adsReadConversion(int16_t &raw) {
  Wire.beginTransmission(ADS1115_ADDR);
  Wire.write(0x00);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)ADS1115_ADDR, 2) != 2) return false;
  raw = (int16_t)((Wire.read() << 8) | Wire.read());
  return true;
}

static float adsReadSingleEndedVolts(uint8_t channel) {
  if (!sensor.adsPresent || channel > 3) return NAN;

  const uint16_t mux[] = { 0x4000, 0x5000, 0x6000, 0x7000 };
  uint16_t config = 0x8000 | mux[channel] | 0x0200 | 0x0100 | 0x0080 | 0x0003;
  if (!adsWriteConfig(config)) return NAN;
  delay(9);

  int16_t raw = 0;
  if (!adsReadConversion(raw)) return NAN;
  return ((float)raw * 4.096f) / 32767.0f;
}

static void refreshPiStatusFromAds() {
  if (!sensor.adsPresent) {
    for (uint8_t i = 0; i < 2; i++) {
      resetAdsPiRailProof(i);
      piStatus[i].relayOn = piConfig[i].relayState;
      piStatus[i].sensedOn = sensor.ioPresent && piConfig[i].relayState;
      piStatus[i].sensedEstimated = true;
      piStatus[i].adsVolts = 0.0f;
    }
    if (!adsMissingLogged) {
      Serial.println("[ADS1115] Device missing; using confirmed relay command state as fallback.");
      adsMissingLogged = true;
    }
    return;
  }
  adsMissingLogged = false;
  for (uint8_t i = 0; i < 2; i++) {
    float volts = adsReadSingleEndedVolts(i);
    if (!isnan(volts)) {
      piStatus[i].adsVolts = volts;
      piStatus[i].sensedOn = volts <= ADS_PI_ON_MAX_V;
      piStatus[i].sensedEstimated = false;
      updateAdsPiRailProof(i, true, volts);
      if (adsReadFailureLogged[i]) {
        Serial.printf("[ADS1115] A%u read recovered: %.3fV sensed=%s\n", i, volts, piStatus[i].sensedOn ? "on" : "off");
        adsReadFailureLogged[i] = false;
      }
      if (!lastPiSensedStateKnown[i] || lastPiSensedState[i] != piStatus[i].sensedOn) {
        Serial.printf("[ADS1115] A%u state -> %s at %.3fV\n", i, piStatus[i].sensedOn ? "on" : "off", volts);
        lastPiSensedState[i] = piStatus[i].sensedOn;
        lastPiSensedStateKnown[i] = true;
      }
    } else {
      updateAdsPiRailProof(i, false, NAN);
      piStatus[i].adsVolts = 0.0f;
      piStatus[i].sensedOn = sensor.ioPresent && piConfig[i].relayState;
      piStatus[i].sensedEstimated = true;
      if (!adsReadFailureLogged[i]) {
        Serial.printf("[ADS1115] A%u read failed; using relay command fallback\n", i);
        adsReadFailureLogged[i] = true;
      }
    }
    piStatus[i].relayOn = piConfig[i].relayState;
  }
}

// ============================================================================
// LTC2944
// ============================================================================
static bool ltcWrite8(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(LTC2944_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

static bool ltcRead16(uint8_t reg, uint16_t &value) {
  Wire.beginTransmission(LTC2944_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)LTC2944_ADDR, 2) != 2) return false;
  value = ((uint16_t)Wire.read() << 8) | Wire.read();
  return true;
}

static bool initLtc2944() {
  if (!sensor.ltcPresent) {
    Serial.println("[LTC2944] Init skipped, device missing.");
    return false;
  }
  // Enable continuous voltage/current/temperature conversions. This value is a
  // practical default for the common breakout configuration.
  bool ok = ltcWrite8(0x01, 0xFC);
  Serial.printf("[LTC2944] Init %s\n", ok ? "ok" : "failed");
  return ok;
}

static void discoverI2cDevices(bool verbose, bool initializeRecovered) {
  bool oldLtc = sensor.ltcPresent;
  bool oldAds = sensor.adsPresent;
  bool oldIo = sensor.ioPresent;
  bool oldEeprom = sensor.eepromPresent;

  if (verbose) i2cScan();

  sensor.ltcPresent = findDeviceInRange("LTC2944", 0x64, 0x67, LTC2944_ADDR);
  sensor.adsPresent = findDeviceInRange("ADS1115", 0x48, 0x4B, ADS1115_ADDR);
  sensor.ioPresent = findDeviceInRange("XL9535", 0x20, 0x27, XL9535_ADDR);
  sensor.eepromPresent = findDeviceInRange("AT24C256", 0x50, 0x57, AT24C256_ADDR);
  logModulePresence("LTC2944", LTC2944_ADDR, sensor.ltcPresent);
  logModulePresence("ADS1115", ADS1115_ADDR, sensor.adsPresent);
  logModulePresence("XL9535", XL9535_ADDR, sensor.ioPresent);
  logModulePresence("AT24C256", AT24C256_ADDR, sensor.eepromPresent);

  if (initializeRecovered && !oldIo && sensor.ioPresent) {
    xl9535Init();
    applyRelayState(0, piConfig[0].relayState);
    applyRelayState(1, piConfig[1].relayState);
  }
  if (initializeRecovered && !oldLtc && sensor.ltcPresent) {
    initLtc2944();
    ltcReadFailureLogged = false;
  }
  if (initializeRecovered && !oldAds && sensor.adsPresent) {
    adsMissingLogged = false;
    adsReadFailureLogged[0] = false;
    adsReadFailureLogged[1] = false;
    lastPiSensedStateKnown[0] = false;
    lastPiSensedStateKnown[1] = false;
  }
  if (initializeRecovered && !oldEeprom && sensor.eepromPresent) {
    forceSave = true;
  }
}

static bool readLtc2944(float &voltageV, float &currentA, float &temperatureC) {
  uint16_t rawVoltage = 0;
  uint16_t rawCurrent = 0;
  uint16_t rawTemperature = 0;
  if (!ltcRead16(0x08, rawVoltage)) return false;
  if (!ltcRead16(0x0E, rawCurrent)) return false;
  if (!ltcRead16(0x14, rawTemperature)) return false;

  voltageV = (rawVoltage / 65535.0f) * 70.8f;
  float senseV = (((int32_t)rawCurrent - 32767) / 32767.0f) * 0.064f;
  currentA = senseV / LTC_SHUNT_OHMS;
  temperatureC = (rawTemperature * 510.0f / 65535.0f) - 273.15f;
  return true;
}

// ============================================================================
// Battery model
// ============================================================================
static void restoreSaveTracking() {
  unsigned long now = millis();
  lastSavedWh = sensor.remainingWh;
  lastSaveMs = now;
  lastAnySaveMs = now;
  lastSavedState = BATTERY_IDLE;
}

static bool calibrateBatteryPercent(float percent) {
  sensor.remainingWh = usableEnergyWh() * constrain(percent, 0.0f, 100.0f) / 100.0f;
  sensor.remainingMah = (BATTERY_CAPACITY_AH * 1000.0f * USABLE_CAPACITY_PCT) * constrain(percent, 0.0f, 100.0f) / 100.0f;
  forceSave = true;
  savePersistedStateIfNeeded();
  return true;
}

static bool calibrateBatteryWh(float wh) {
  sensor.remainingWh = constrain(wh, 0.0f, usableEnergyWh());
  sensor.remainingMah = BATTERY_CAPACITY_AH > 0.0f ? (sensor.remainingWh / BATTERY_NOMINAL_V) * 1000.0f : 0.0f;
  forceSave = true;
  savePersistedStateIfNeeded();
  return true;
}

static void updateBatteryModel() {
  static bool initialized = false;
  static unsigned long lastEnergyMs = 0;

  float vbus = 0.0f;
  float currentA = 0.0f;
  float temperatureC = NAN;
  bool ok = readLtc2944(vbus, currentA, temperatureC);
  if (!ok) {
    sensor.valid = false;
    if (!ltcReadFailureLogged) {
      Serial.println("[LTC2944] Read failed.");
      ltcReadFailureLogged = true;
    }
    return;
  }
  if (ltcReadFailureLogged) {
    Serial.println("[LTC2944] Read recovered.");
    ltcReadFailureLogged = false;
  }

  unsigned long now = millis();
  if (!initialized) {
    initialized = true;
    lastEnergyMs = now;
  } else if (now != lastEnergyMs) {
    float elapsedHours = min((now - lastEnergyMs) / 3600000.0f, 1.0f);
    lastEnergyMs = now;
    float deltaWh = fabsf(vbus * currentA) * elapsedHours;
    if (currentA < -CURRENT_DEADBAND_A) {
      sensor.remainingWh -= deltaWh;
    } else if (currentA > CURRENT_DEADBAND_A) {
      sensor.remainingWh += deltaWh * CHARGE_EFFICIENCY;
    }
  }

  bool looksFull = vbus >= FULL_FLOAT_VOLTAGE &&
                   currentA >= -CURRENT_DEADBAND_A &&
                   currentA <= FULL_CURRENT_A;
  if (looksFull) {
    if (fullCandidateStartMs == 0) fullCandidateStartMs = now;
    if (now - fullCandidateStartMs >= FULL_CONFIRM_MS) {
      sensor.remainingWh = usableEnergyWh();
      forceSave = true;
    }
  } else {
    fullCandidateStartMs = 0;
  }

  float criticalWh = usableEnergyWh() * 0.05f;
  if (vbus <= CRITICAL_VOLTAGE && currentA < -CURRENT_DEADBAND_A && sensor.remainingWh > criticalWh) {
    sensor.remainingWh = criticalWh;
    forceSave = true;
  }

  sensor.remainingWh = constrain(sensor.remainingWh, 0.0f, usableEnergyWh());
  sensor.remainingMah = BATTERY_NOMINAL_V > 0.0f ? (sensor.remainingWh / BATTERY_NOMINAL_V) * 1000.0f : 0.0f;
  sensor.vbus = vbus;
  sensor.currentA = currentA;
  sensor.powerW = vbus * currentA;
  sensor.temperatureC = temperatureC;
  sensor.soc = usableEnergyWh() > 0.0f ? (sensor.remainingWh / usableEnergyWh()) * 100.0f : 0.0f;
  // Runtime is a remaining-discharge estimate. While charging or idle there
  // is no meaningful remaining runtime, so publish the established -1
  // sentinel and let each UI render an explicit Charging/Idle state.
  sensor.runtimeMin = batteryFlowState(currentA) == BATTERY_DISCHARGING
                        ? runtimeMinutes(sensor.remainingWh, sensor.powerW)
                        : -1.0f;
  sensor.valid = true;
}

// ============================================================================
// HTTP helpers
// ============================================================================
static bool queryParam(const char *query, const char *key, char *buf, size_t bufLen) {
  size_t keyLen = strlen(key);
  const char *p = query;
  while (p && *p) {
    if (strncmp(p, key, keyLen) == 0 && p[keyLen] == '=') {
      p += keyLen + 1;
      size_t i = 0;
      while (*p && *p != '&' && i + 1 < bufLen) buf[i++] = *p++;
      buf[i] = '\0';
      return true;
    }
    p = strchr(p, '&');
    if (p) p++;
  }
  return false;
}

static void sendJson(EthernetClient &client, const char *statusLine, const char *body) {
  client.println(statusLine);
  client.println(F("Content-Type: application/json"));
  client.print(F("Content-Length: "));
  client.println(strlen(body));
  client.println(F("Connection: close"));
  client.println(F("Access-Control-Allow-Origin: *"));
  client.println();
  client.print(body);
  client.flush();
  client.stop();
}

static void sendDiagnosticLog(EthernetClient &client) {
  const size_t contentLength = bufferedSerialLog.size();
  client.println(F("HTTP/1.1 200 OK"));
  client.println(F("Content-Type: text/plain; charset=utf-8"));
  client.print(F("Content-Length: "));
  client.println((unsigned long)contentLength);
  client.println(F("Cache-Control: no-store"));
  client.println(F("Connection: close"));
  client.println();

  uint8_t chunk[256];
  size_t sent = 0;
  while (sent < contentLength) {
    size_t chunkLength = min(sizeof(chunk), contentLength - sent);
    for (size_t i = 0; i < chunkLength; i++) {
      chunk[i] = (uint8_t)bufferedSerialLog.at(sent + i);
    }
    client.write(chunk, chunkLength);
    sent += chunkLength;
    rp2040.wdt_reset();
  }
  client.flush();
  client.stop();
}

static float finiteOrZero(float value) {
  return isfinite(value) ? value : 0.0f;
}

static const char* gracefulShutdownStageName(uint8_t piIndex) {
  if (piIndex >= 2) return "unknown";
  if (gracefulShutdowns[piIndex].stage == GRACEFUL_SHUTDOWN_REQUESTING) return "requesting";
  if (gracefulShutdowns[piIndex].stage == GRACEFUL_SHUTDOWN_WAITING) return "grace_period";
  return "idle";
}

static unsigned long gracefulShutdownRemainingSec(uint8_t piIndex) {
  if (piIndex >= 2 || gracefulShutdowns[piIndex].stage != GRACEFUL_SHUTDOWN_WAITING) return 0;
  unsigned long elapsed = millis() - gracefulShutdowns[piIndex].acceptedMs;
  if (elapsed >= SHUTDOWN_GRACE_MS) return 0;
  return (SHUTDOWN_GRACE_MS - elapsed + 999UL) / 1000UL;
}

static unsigned long pveUpsHandoffRecoveryRemainingSec() {
  if (!pveUpsHandoffArmed || pveUpsHandoffRecoveryStartMs == 0) return 0;
  unsigned long elapsed = millis() - pveUpsHandoffRecoveryStartMs;
  if (elapsed >= PVE_UPS_HANDOFF_RECOVERY_HOLD_MS) return 0;
  return (PVE_UPS_HANDOFF_RECOVERY_HOLD_MS - elapsed + 999UL) / 1000UL;
}

static void legacyStatusJson(char *buf, size_t len) {
  snprintf(
    buf,
    len,
    "{\"id\":\"main\",\"vbus\":%.3f,\"i\":%.3f,\"p\":%.3f,\"soc\":%.1f,"
    "\"remaining_wh\":%.2f,\"runtime_min\":%.1f,\"temperature_c\":%.2f,\"capacity_ah\":%.1f,\"ina\":%s}",
    sensor.vbus,
    sensor.currentA,
    sensor.powerW,
    sensor.soc,
    sensor.remainingWh,
    sensor.runtimeMin,
    finiteOrZero(sensor.temperatureC),
    BATTERY_CAPACITY_AH,
    sensor.valid ? "true" : "false"
  );
}

static void fullStatusJson(char *buf, size_t len) {
  snprintf(
    buf,
    len,
    "{"
    "\"firmware_version\":\"%s\","
    "\"battery\":{\"voltage_v\":%.3f,\"current_a\":%.3f,\"power_w\":%.3f,\"temperature_c\":%.2f,\"soc\":%.1f,"
    "\"remaining_wh\":%.2f,\"remaining_mah\":%.1f,\"runtime_min\":%.1f},"
    "\"pis\":["
    "{\"id\":1,\"relay_on\":%s,\"sensed_on\":%s,\"sensed_estimated\":%s,\"sense_source\":\"%s\",\"ads_v\":%.3f,\"ads_raw_v\":%.3f,\"ads_rail_seen_on\":%s,\"ads_power_off_confirmed\":%s,\"relay_cutoff_permitted\":%s,\"shutdown_url\":\"%s\",\"graceful_shutdown_state\":\"%s\",\"graceful_shutdown_remaining_sec\":%lu},"
    "{\"id\":2,\"relay_on\":%s,\"sensed_on\":%s,\"sensed_estimated\":%s,\"sense_source\":\"%s\",\"ads_v\":%.3f,\"ads_raw_v\":%.3f,\"ads_rail_seen_on\":%s,\"ads_power_off_confirmed\":%s,\"relay_cutoff_permitted\":%s,\"shutdown_url\":\"%s\",\"graceful_shutdown_state\":\"%s\",\"graceful_shutdown_remaining_sec\":%lu}"
    "],"
    "\"hardware\":{\"ltc2944\":%s,\"ads1115\":%s,\"xl9535\":%s,\"at24c256\":%s,\"ethernet_link\":%s,\"littlefs\":%s},"
    "\"low_battery_stage\":\"%s\",\"low_battery_lockout\":%s,"
    "\"pve_ups_handoff_armed\":%s,\"pve_ups_handoff_cutoff_ready\":%s,\"pve_ups_handoff_recovery_remaining_sec\":%lu,"
    "\"no_ads_restart_pending\":%s,\"no_ads_recovery_cycle_started\":%s"
    "}",
    FIRMWARE_VERSION,
    sensor.vbus,
    sensor.currentA,
    sensor.powerW,
    finiteOrZero(sensor.temperatureC),
    sensor.soc,
    sensor.remainingWh,
    sensor.remainingMah,
    sensor.runtimeMin,
    piStatus[0].relayOn ? "true" : "false",
    piStatus[0].sensedOn ? "true" : "false",
    piStatus[0].sensedEstimated ? "true" : "false",
    piStatus[0].sensedEstimated ? "relay_command" : "ads1115",
    displayedPiVoltage(0),
    piStatus[0].adsVolts,
    adsPiRailSeenOn[0] ? "true" : "false",
    adsPiRailOffConfirmed[0] ? "true" : "false",
    relayCutoffConfirmedByAds(0) ? "true" : "false",
    piConfig[0].shutdownUrl,
    gracefulShutdownStageName(0),
    gracefulShutdownRemainingSec(0),
    piStatus[1].relayOn ? "true" : "false",
    piStatus[1].sensedOn ? "true" : "false",
    piStatus[1].sensedEstimated ? "true" : "false",
    piStatus[1].sensedEstimated ? "relay_command" : "ads1115",
    displayedPiVoltage(1),
    piStatus[1].adsVolts,
    adsPiRailSeenOn[1] ? "true" : "false",
    adsPiRailOffConfirmed[1] ? "true" : "false",
    relayCutoffConfirmedByAds(1) ? "true" : "false",
    piConfig[1].shutdownUrl,
    gracefulShutdownStageName(1),
    gracefulShutdownRemainingSec(1),
    sensor.ltcPresent ? "true" : "false",
    sensor.adsPresent ? "true" : "false",
    sensor.ioPresent ? "true" : "false",
    sensor.eepromPresent ? "true" : "false",
    Ethernet.linkStatus() != LinkOFF ? "true" : "false",
    littleFsAvailable ? "true" : "false",
    lowBatteryStage == LOW_BATTERY_WAITING ? "waiting" :
    lowBatteryStage == LOW_BATTERY_DONE ? "done" : "idle",
    lowBatteryLockout ? "true" : "false",
    pveUpsHandoffArmed ? "true" : "false",
    (pveUpsHandoffArmed && allRelayCutoffsConfirmedByAds()) ? "true" : "false",
    pveUpsHandoffRecoveryRemainingSec(),
    noAdsRestartPending ? "true" : "false",
    noAdsRecoveryCycleStarted ? "true" : "false"
  );
}

static void batteryStatusJson(char *buf, size_t len) {
  snprintf(buf, len,
           "{\"id\":\"main\",\"remaining_wh\":%.3f,\"usable_wh\":%.3f,\"soc\":%.1f}",
           sensor.remainingWh, usableEnergyWh(), sensor.soc);
}

static void handleCalibrateHTTP(EthernetClient &client, const char *query) {
  char tokenBuf[40], valueBuf[24], actionBuf[16], body[256];
  Serial.printf("[HTTP] GET /calibrate?%s\n", query ? query : "");

  if (!queryParam(query, "token", tokenBuf, sizeof(tokenBuf)) ||
      strcmp(tokenBuf, CALIBRATION_TOKEN) != 0) {
    snprintf(body, sizeof(body), "{\"ok\":false,\"error\":\"bad_token\"}");
    sendJson(client, "HTTP/1.1 403 Forbidden", body);
    return;
  }

  bool ok = false;
  if (queryParam(query, "percent", valueBuf, sizeof(valueBuf))) {
    ok = calibrateBatteryPercent(atof(valueBuf));
  } else if (queryParam(query, "wh", valueBuf, sizeof(valueBuf))) {
    ok = calibrateBatteryWh(atof(valueBuf));
  } else if (queryParam(query, "action", actionBuf, sizeof(actionBuf))) {
    if      (strcmp(actionBuf, "full") == 0)  ok = calibrateBatteryPercent(100.0f);
    else if (strcmp(actionBuf, "reset") == 0) ok = calibrateBatteryPercent(50.0f);
    else if (strcmp(actionBuf, "save") == 0)  { forceSave = true; savePersistedStateIfNeeded(); ok = true; }
  }

  if (ok) {
    char statusBuf[160];
    batteryStatusJson(statusBuf, sizeof(statusBuf));
    snprintf(body, sizeof(body), "{\"ok\":true,\"status\":%s}", statusBuf);
    Serial.println("[CALIBRATE] Command applied successfully.");
  } else {
    snprintf(body, sizeof(body), "{\"ok\":false,\"error\":\"missing_or_invalid_command\"}");
    Serial.println("[CALIBRATE] Command failed.");
  }
  sendJson(client, "HTTP/1.1 200 OK", body);
}

static bool beginPowerCycle(uint8_t piIndex, bool allowNoAdsRecoveryCycle = false) {
  if (piIndex >= 2) return false;
  if (gracefulShutdowns[piIndex].stage != GRACEFUL_SHUTDOWN_IDLE) {
    Serial.printf("[POWER] Pi%u power-cycle rejected: graceful shutdown active\n", piIndex + 1);
    return false;
  }
  if (batteryCriticalNow()) {
    Serial.printf("[POWER] Pi%u power-cycle rejected: battery critical\n", piIndex + 1);
    return false;
  }
  if (allowNoAdsRecoveryCycle &&
      !noAdsRecoveryCycleAuthorized()) {
    Serial.printf("[POWER] Pi%u no-ADS recovery cycle rejected: recovery interlock not satisfied\n",
                  piIndex + 1);
    return false;
  }
  Serial.printf("[POWER] Pi%u power-cycle start\n", piIndex + 1);
  powerActions[piIndex].action = POWER_ACTION_RESTORE_ON;
  powerActions[piIndex].dueMs = millis() + POWER_CYCLE_OFF_MS;
  powerCycleStartedMs[piIndex] = millis();
  if (!applyRelayState(piIndex, false, allowNoAdsRecoveryCycle)) {
    Serial.printf("[POWER] Pi%u power-cycle off failed; restoring immediately\n", piIndex + 1);
    applyRelayState(piIndex, true);
    powerActions[piIndex].action = POWER_ACTION_NONE;
    powerCycleStartedMs[piIndex] = 0;
    return false;
  }
  Serial.printf("[POWER] Pi%u power-cycle restore scheduled in %lums\n",
                piIndex + 1,
                (unsigned long)POWER_CYCLE_OFF_MS);
  return true;
}

static bool startManualGracefulShutdown(uint8_t piIndex);

static void cancelManualGracefulShutdown(uint8_t piIndex, const char *reason) {
  if (piIndex >= 2 || gracefulShutdowns[piIndex].stage == GRACEFUL_SHUTDOWN_IDLE) return;
  Serial.printf("[GRACEFUL_SHUTDOWN] Pi%u cancelled: %s\n", piIndex + 1, reason ? reason : "manual override");
  gracefulShutdowns[piIndex].stage = GRACEFUL_SHUTDOWN_IDLE;
  gracefulShutdowns[piIndex].lastAttemptMs = 0;
  gracefulShutdowns[piIndex].acceptedMs = 0;
}

static bool handlePowerCommandTarget(int target, const char *command) {
  bool ok = true;
  int start = target == 3 ? 0 : target - 1;
  int end = target == 3 ? 1 : target - 1;
  for (int i = start; i <= end; i++) {
    if (strcmp(command, "on") == 0) {
      cancelManualGracefulShutdown(i, "power-on command");
      ok &= applyRelayState(i, true);
    } else if (strcmp(command, "off") == 0) {
      cancelManualGracefulShutdown(i, "emergency power-off command");
      ok &= applyRelayState(i, false);
    }
    else if (strcmp(command, "power-cycle") == 0) ok &= beginPowerCycle(i);
    else if (strcmp(command, "graceful-shutdown") == 0) ok &= startManualGracefulShutdown(i);
    else ok = false;
  }
  if (ok && strcmp(command, "on") == 0) {
    if (pveUpsHandoffArmed) {
      pveUpsHandoffArmed = false;
      pveUpsHandoffArmedMs = 0;
      pveUpsHandoffRecoveryStartMs = 0;
      Serial.println("[PVE_UPS_HANDOFF] Armed state cleared by explicit power-on command.");
    }
    if (lowBatteryLockout) {
      lowBatteryLockout = false;
      lowBatteryStage = LOW_BATTERY_IDLE;
      lowBatteryStageStartedMs = 0;
      autoRestoreCandidateStartMs = 0;
      shutdownRecoveryCandidateStartMs = 0;
      forceSave = true;
      savePersistedStateIfNeeded();
      Serial.println("[LOW_BATTERY] Persistent lockout cleared by explicit power-on command.");
    }
  }
  return ok;
}

// ============================================================================
// MQTT
// ============================================================================
static void mqttCopyPayload(char *dst, size_t dstLen, const byte *payload, unsigned int length) {
  if (dstLen == 0) return;
  size_t copyLen = min((size_t)length, dstLen - 1);
  memcpy(dst, payload, copyLen);
  dst[copyLen] = '\0';

  while (copyLen > 0 && (dst[copyLen - 1] == '\r' || dst[copyLen - 1] == '\n' ||
                         dst[copyLen - 1] == ' ' || dst[copyLen - 1] == '\t')) {
    dst[--copyLen] = '\0';
  }

  char *start = dst;
  while (*start == ' ' || *start == '\t' || *start == '\r' || *start == '\n') start++;
  if (start != dst) memmove(dst, start, strlen(start) + 1);
  for (char *p = dst; *p; p++) {
    if (*p >= 'A' && *p <= 'Z') *p = *p - 'A' + 'a';
  }
}

static bool parseJsonNumberField(const char *payload, const char *field, float &value) {
  char needle[24];
  snprintf(needle, sizeof(needle), "\"%s\"", field);
  const char *p = strstr(payload, needle);
  if (!p) return false;
  p = strchr(p, ':');
  if (!p) return false;
  value = atof(p + 1);
  return true;
}

static bool parseJsonStringField(const char *payload, const char *field, char *value, size_t valueLen) {
  char needle[24];
  snprintf(needle, sizeof(needle), "\"%s\"", field);
  const char *p = strstr(payload, needle);
  if (!p) return false;
  p = strchr(p, ':');
  if (!p) return false;
  p++;
  while (*p == ' ' || *p == '\t') p++;
  if (*p != '"') return false;
  p++;

  size_t i = 0;
  while (*p && *p != '"' && i + 1 < valueLen) value[i++] = *p++;
  value[i] = '\0';
  return i > 0;
}

static void mqttPublishCalibrationStatus(const char *action, bool ok, const char *error) {
  if (!mqttClient.connected()) return;

  char statusBuf[160];
  char body[320];
  batteryStatusJson(statusBuf, sizeof(statusBuf));

  if (ok) {
    snprintf(body, sizeof(body), "{\"ok\":true,\"action\":\"%s\",\"status\":%s}", action, statusBuf);
  } else {
    snprintf(body, sizeof(body), "{\"ok\":false,\"error\":\"%s\"}", error ? error : "unknown_error");
  }
  mqttClient.publish(MQTT_CALIBRATE_STATUS_TOPIC, body, true);
}

static void mqttPublishPowerStatus(const char *command, int target, bool ok) {
  if (!mqttClient.connected()) return;

  char body[192];
  snprintf(body, sizeof(body),
           "{\"ok\":%s,\"command\":\"%s\",\"target\":%d,\"pi1\":%s,\"pi2\":%s}",
           ok ? "true" : "false",
           command ? command : "unknown",
           target,
           piConfig[0].relayState ? "true" : "false",
           piConfig[1].relayState ? "true" : "false");
  mqttClient.publish(MQTT_POWER_STATUS_TOPIC, body, true);
}

static void mqttPublishState() {
  if (!mqttClient.connected()) return;
  char body[2048];
  fullStatusJson(body, sizeof(body));
  mqttClient.publish(MQTT_STATE_TOPIC, body, true);
}

static void mqttPublishConfig(const char *topic, const char *payload) {
  mqttClient.publish(topic, payload, true);
}

static void mqttPublishSensorDiscovery(const char *objectId,
                                       const char *name,
                                       const char *valueTemplate,
                                       const char *unit,
                                       const char *deviceClass,
                                       const char *stateClass,
                                       const char *icon,
                                       const char *defaultEntityId = nullptr,
                                       int displayPrecision = -1,
                                       const char *stateAvailabilityTemplate = nullptr) {
  char topic[128];
  char payload[1536];
  char precisionField[56] = {0};
  char entityIdField[112] = {0};
  char availabilityField[640] = {0};
  if (displayPrecision >= 0) {
    snprintf(precisionField, sizeof(precisionField),
             ",\"suggested_display_precision\":%d", displayPrecision);
  }
  if (defaultEntityId) {
    snprintf(entityIdField, sizeof(entityIdField),
             ",\"default_entity_id\":\"%s\"", defaultEntityId);
  }
  if (stateAvailabilityTemplate) {
    snprintf(availabilityField, sizeof(availabilityField),
             "\"availability\":["
             "{\"topic\":\"%s\",\"payload_available\":\"online\",\"payload_not_available\":\"offline\"},"
             "{\"topic\":\"%s\",\"value_template\":\"%s\",\"payload_available\":\"online\",\"payload_not_available\":\"offline\"}"
             "],\"availability_mode\":\"all\",",
             MQTT_AVAILABILITY_TOPIC, MQTT_STATE_TOPIC, stateAvailabilityTemplate);
  } else {
    snprintf(availabilityField, sizeof(availabilityField),
             "\"availability_topic\":\"%s\",", MQTT_AVAILABILITY_TOPIC);
  }
  snprintf(topic, sizeof(topic), "homeassistant/sensor/pico_ups_%s/config", objectId);
  snprintf(payload, sizeof(payload),
           "{"
           "\"name\":\"%s\","
           "\"unique_id\":\"pico_ups_%s\","
           "\"state_topic\":\"%s\","
           "%s"
           "\"value_template\":\"%s\""
           "%s"
           "%s%s%s"
           "%s%s%s"
           "%s%s%s"
           "%s"
           "%s%s%s,"
           "\"device\":{\"identifiers\":[\"pico_ups\"],\"name\":\"Pico UPS\",\"manufacturer\":\"Custom\",\"model\":\"Pico W5500 UPS\",\"sw_version\":\"%s\"}"
           "}",
           name,
           objectId,
           MQTT_STATE_TOPIC,
           availabilityField,
           valueTemplate,
           entityIdField,
           unit ? ",\"unit_of_measurement\":\"" : "", unit ? unit : "", unit ? "\"" : "",
           deviceClass ? ",\"device_class\":\"" : "", deviceClass ? deviceClass : "", deviceClass ? "\"" : "",
           stateClass ? ",\"state_class\":\"" : "", stateClass ? stateClass : "", stateClass ? "\"" : "",
           precisionField,
           icon ? ",\"icon\":\"" : "", icon ? icon : "", icon ? "\"" : "",
           FIRMWARE_VERSION);
  mqttPublishConfig(topic, payload);
}

static void mqttPublishBinarySensorDiscovery(const char *objectId,
                                             const char *name,
                                             const char *valueTemplate,
                                             const char *icon) {
  char topic[128];
  char payload[1024];
  snprintf(topic, sizeof(topic), "homeassistant/binary_sensor/pico_ups_%s/config", objectId);
  snprintf(payload, sizeof(payload),
           "{"
           "\"name\":\"%s\","
           "\"unique_id\":\"pico_ups_%s\","
           "\"state_topic\":\"%s\","
           "\"availability_topic\":\"%s\","
           "\"value_template\":\"%s\","
           "\"payload_on\":\"True\","
           "\"payload_off\":\"False\""
           "%s%s%s,"
           "\"device\":{\"identifiers\":[\"pico_ups\"],\"name\":\"Pico UPS\",\"manufacturer\":\"Custom\",\"model\":\"Pico W5500 UPS\",\"sw_version\":\"%s\"}"
           "}",
           name,
           objectId,
           MQTT_STATE_TOPIC,
           MQTT_AVAILABILITY_TOPIC,
           valueTemplate,
           icon ? ",\"icon\":\"" : "", icon ? icon : "", icon ? "\"" : "",
           FIRMWARE_VERSION);
  mqttPublishConfig(topic, payload);
}

static void mqttPublishButtonDiscovery(const char *objectId,
                                       const char *name,
                                       const char *commandTopic,
                                       const char *payloadPress,
                                       const char *icon) {
  char topic[128];
  char payload[768];
  snprintf(topic, sizeof(topic), "homeassistant/button/pico_ups_%s/config", objectId);
  snprintf(payload, sizeof(payload),
           "{"
           "\"name\":\"%s\","
           "\"unique_id\":\"pico_ups_%s\","
           "\"command_topic\":\"%s\","
           "\"payload_press\":\"%s\","
           "\"availability_topic\":\"%s\""
           "%s%s%s,"
           "\"device\":{\"identifiers\":[\"pico_ups\"],\"name\":\"Pico UPS\",\"manufacturer\":\"Custom\",\"model\":\"Pico W5500 UPS\",\"sw_version\":\"%s\"}"
           "}",
           name,
           objectId,
           commandTopic,
           payloadPress,
           MQTT_AVAILABILITY_TOPIC,
           icon ? ",\"icon\":\"" : "", icon ? icon : "", icon ? "\"" : "",
           FIRMWARE_VERSION);
  mqttPublishConfig(topic, payload);
}

static void mqttPublishDiscovery() {
  // Remove legacy discovery entries that used ambiguous names. The MQTT and
  // HTTP sets now have explicit source-specific entity IDs so both can coexist.
  mqttClient.publish("homeassistant/sensor/pico_ups_soc/config", "", true);
  mqttClient.publish("homeassistant/sensor/pico_ups_voltage/config", "", true);
  mqttClient.publish("homeassistant/sensor/pico_ups_current/config", "", true);
  mqttClient.publish("homeassistant/sensor/pico_ups_power/config", "", true);
  mqttClient.publish("homeassistant/sensor/pico_ups_temperature/config", "", true);
  mqttClient.publish("homeassistant/sensor/pico_ups_remaining_wh/config", "", true);
  mqttClient.publish("homeassistant/sensor/pico_ups_runtime/config", "", true);

  mqttPublishSensorDiscovery("mqtt_soc", "SOC",
                             "{{ value_json.battery.soc | default(0) }}",
                             "%", "battery", "measurement", "mdi:battery",
                             "sensor.ups_battery_main_mqtt_soc", 1);
  mqttPublishSensorDiscovery("mqtt_voltage", "Voltage",
                             "{{ value_json.battery.voltage_v | default(0) }}",
                             "V", "voltage", "measurement", nullptr,
                             "sensor.ups_battery_main_mqtt_voltage", 3);
  mqttPublishSensorDiscovery("mqtt_current", "Current",
                             "{{ value_json.battery.current_a | default(0) }}",
                             "A", "current", "measurement", nullptr,
                             "sensor.ups_battery_main_mqtt_current", 3);
  mqttPublishSensorDiscovery("mqtt_power", "Power",
                             "{{ value_json.battery.power_w | default(0) }}",
                             "W", "power", "measurement", "mdi:lightning-bolt",
                             "sensor.ups_battery_main_mqtt_power", 3);
  mqttPublishSensorDiscovery("mqtt_temperature", "Temperature",
                             "{{ value_json.battery.temperature_c | default(0) }}",
                             "°C", "temperature", "measurement", nullptr,
                             "sensor.ups_battery_main_mqtt_temperature", 2);
  mqttPublishSensorDiscovery("mqtt_remaining_wh", "Energy",
                             "{{ value_json.battery.remaining_wh | default(0) }}",
                             "Wh", "energy_storage", "measurement", "mdi:battery-clock",
                             "sensor.ups_battery_main_mqtt_remaining_wh", 2);
  mqttPublishSensorDiscovery("mqtt_runtime", "Runtime",
                             "{% set c = value_json.battery.current_a | float(0) %}{% set r = value_json.battery.runtime_min | float(-1) %}{% if c > 0.05 %}Charging{% elif c < -0.05 and r >= 0 %}{{ r | round(1) }} min{% elif c < -0.05 %}Discharging{% else %}Idle{% endif %}",
                             nullptr, nullptr, nullptr, "mdi:timer-outline",
                             "sensor.ups_battery_main_mqtt_runtime");
  mqttPublishSensorDiscovery("mqtt_runtime_minutes", "Runtime Minutes",
                             "{{ value_json.battery.runtime_min | float(-1) | round(1) }}",
                             "min", "duration", "measurement", "mdi:timer-outline",
                             "sensor.ups_battery_main_mqtt_runtime_minutes", 1,
                             "{{ 'online' if (value_json.battery.current_a | float(0)) < -0.05 and (value_json.battery.runtime_min | float(-1)) >= 0 else 'offline' }}");
  mqttPublishSensorDiscovery("low_battery_stage", "UPS Low Battery Stage", "{{ value_json.low_battery_stage | default('unknown') }}", nullptr, nullptr, nullptr, "mdi:battery-alert");
  mqttPublishSensorDiscovery("pi1_ads_voltage", "UPS Pi1 ADS Voltage", "{{ value_json.pis[0].ads_v | default(0) }}", "V", "voltage", "measurement", nullptr);
  mqttPublishSensorDiscovery("pi2_ads_voltage", "UPS Pi2 ADS Voltage", "{{ value_json.pis[1].ads_v | default(0) }}", "V", "voltage", "measurement", nullptr);

  mqttPublishBinarySensorDiscovery("pi1_relay_on", "UPS Pi1 Relay On", "{{ value_json.pis[0].relay_on | default(false) }}", "mdi:electric-switch");
  mqttPublishBinarySensorDiscovery("pi2_relay_on", "UPS Pi2 Relay On", "{{ value_json.pis[1].relay_on | default(false) }}", "mdi:electric-switch");
  mqttPublishBinarySensorDiscovery("pi1_sensed_on", "UPS Pi1 Sensed On", "{{ value_json.pis[0].sensed_on | default(false) }}", "mdi:raspberry-pi");
  mqttPublishBinarySensorDiscovery("pi2_sensed_on", "UPS Pi2 Sensed On", "{{ value_json.pis[1].sensed_on | default(false) }}", "mdi:raspberry-pi");
  mqttPublishBinarySensorDiscovery("hardware_ltc2944", "UPS Hardware LTC2944", "{{ value_json.hardware.ltc2944 | default(false) }}", nullptr);
  mqttPublishBinarySensorDiscovery("hardware_ads1115", "UPS Hardware ADS1115", "{{ value_json.hardware.ads1115 | default(false) }}", nullptr);
  mqttPublishBinarySensorDiscovery("hardware_xl9535", "UPS Hardware XL9535", "{{ value_json.hardware.xl9535 | default(false) }}", nullptr);
  mqttPublishBinarySensorDiscovery("hardware_at24c256", "UPS Hardware AT24C256", "{{ value_json.hardware.at24c256 | default(false) }}", nullptr);
  mqttPublishBinarySensorDiscovery("ethernet_link", "UPS Ethernet Link", "{{ value_json.hardware.ethernet_link | default(false) }}", "mdi:ethernet");

  mqttPublishButtonDiscovery("calibrate_full", "UPS Battery Main Calibrate Full", MQTT_CALIBRATE_SET_TOPIC, "full", "mdi:battery-charging-100");
  mqttPublishButtonDiscovery("reset_50", "UPS Battery Main Reset 50%", MQTT_CALIBRATE_SET_TOPIC, "reset", "mdi:battery-50");
  mqttPublishButtonDiscovery("force_save", "UPS Battery Main Force Save", MQTT_CALIBRATE_SET_TOPIC, "save", "mdi:content-save");
  mqttPublishButtonDiscovery("pi1_power_on", "UPS Pi1 Power On", MQTT_PI1_POWER_SET_TOPIC, "on", "mdi:power-plug");
  mqttClient.publish("homeassistant/button/pico_ups_pi1_power_off/config", "", true);
  mqttPublishButtonDiscovery("pi1_graceful_shutdown", "UPS Pi1 Graceful Shutdown", MQTT_PI1_POWER_SET_TOPIC, "graceful-shutdown", "mdi:power");
  mqttPublishButtonDiscovery("pi1_power_cycle", "UPS Pi1 Power Cycle", MQTT_PI1_POWER_SET_TOPIC, "power-cycle", "mdi:restart");
  mqttPublishButtonDiscovery("pi2_power_on", "UPS Pi2 Power On", MQTT_PI2_POWER_SET_TOPIC, "on", "mdi:power-plug");
  mqttClient.publish("homeassistant/button/pico_ups_pi2_power_off/config", "", true);
  mqttPublishButtonDiscovery("pi2_graceful_shutdown", "UPS Pi2 Graceful Shutdown", MQTT_PI2_POWER_SET_TOPIC, "graceful-shutdown", "mdi:power");
  mqttPublishButtonDiscovery("pi2_power_cycle", "UPS Pi2 Power Cycle", MQTT_PI2_POWER_SET_TOPIC, "power-cycle", "mdi:restart");
  mqttPublishButtonDiscovery("all_power_on", "UPS All Power On", MQTT_ALL_POWER_SET_TOPIC, "on", "mdi:power-plug");
  mqttClient.publish("homeassistant/button/pico_ups_all_power_off/config", "", true);
  mqttPublishButtonDiscovery("all_graceful_shutdown", "UPS All Graceful Shutdown", MQTT_ALL_POWER_SET_TOPIC, "graceful-shutdown", "mdi:power");
  mqttPublishButtonDiscovery("all_power_cycle", "UPS All Power Cycle", MQTT_ALL_POWER_SET_TOPIC, "power-cycle", "mdi:restart");
  mqttPublishButtonDiscovery("pico_restart", "UPS Pico Restart", MQTT_SYSTEM_RESTART_TOPIC, "restart", "mdi:restart");
}

static void handleMqttCalibrationCommand(const char *payload) {
  bool ok = false;
  const char *action = "unknown";
  float value = 0.0f;
  char jsonAction[16];

  if (strcmp(payload, "full") == 0) {
    ok = calibrateBatteryPercent(100.0f);
    action = "full";
  } else if (strcmp(payload, "reset") == 0) {
    ok = calibrateBatteryPercent(50.0f);
    action = "reset";
  } else if (strcmp(payload, "save") == 0) {
    forceSave = true;
    savePersistedStateIfNeeded();
    ok = true;
    action = "save";
  } else if (strncmp(payload, "percent:", 8) == 0) {
    ok = calibrateBatteryPercent(atof(payload + 8));
    action = "percent";
  } else if (strncmp(payload, "wh:", 3) == 0) {
    ok = calibrateBatteryWh(atof(payload + 3));
    action = "wh";
  } else if (parseJsonNumberField(payload, "percent", value)) {
    ok = calibrateBatteryPercent(value);
    action = "percent";
  } else if (parseJsonNumberField(payload, "wh", value)) {
    ok = calibrateBatteryWh(value);
    action = "wh";
  } else if (parseJsonStringField(payload, "action", jsonAction, sizeof(jsonAction))) {
    if (strcmp(jsonAction, "full") == 0) {
      ok = calibrateBatteryPercent(100.0f);
      action = "full";
    } else if (strcmp(jsonAction, "reset") == 0) {
      ok = calibrateBatteryPercent(50.0f);
      action = "reset";
    } else if (strcmp(jsonAction, "save") == 0) {
      forceSave = true;
      savePersistedStateIfNeeded();
      ok = true;
      action = "save";
    }
  }

  Serial.printf("[MQTT] Calibration command payload=%s result=%s\n", payload, ok ? "ok" : "failed");
  mqttPublishCalibrationStatus(action, ok, ok ? nullptr : "unknown_command");
  if (ok) mqttPublishState();
}

static const char* normalizePowerCommand(const char *payload) {
  if (strcmp(payload, "on") == 0) return "on";
  if (strcmp(payload, "off") == 0) return "off";
  if (strcmp(payload, "cycle") == 0) return "power-cycle";
  if (strcmp(payload, "power-cycle") == 0) return "power-cycle";
  if (strcmp(payload, "restart") == 0) return "power-cycle";
  if (strcmp(payload, "reboot") == 0) return "power-cycle";
  if (strcmp(payload, "shutdown") == 0) return "graceful-shutdown";
  if (strcmp(payload, "graceful-shutdown") == 0) return "graceful-shutdown";
  return nullptr;
}

static void handleMqttPowerCommand(int target, const char *payload) {
  const char *command = normalizePowerCommand(payload);
  bool ok = command && handlePowerCommandTarget(target, command);
  refreshPiStatusFromAds();
  Serial.printf("[MQTT] Power command target=%d payload=%s result=%s\n", target, payload, ok ? "ok" : "failed");
  mqttPublishPowerStatus(command ? command : payload, target, ok);
  mqttPublishState();
}

static void mqttCallback(char *topic, byte *payload, unsigned int length) {
  char payloadBuf[128];
  mqttCopyPayload(payloadBuf, sizeof(payloadBuf), payload, length);

  if (strcmp(topic, MQTT_HOME_ASSISTANT_STATUS_TOPIC) == 0 &&
      strcmp(payloadBuf, "online") == 0) {
    Serial.println("[MQTT] Home Assistant birth received; republishing discovery.");
    mqttPublishDiscovery();
    mqttPublishState();
    return;
  }

  if (strcmp(topic, MQTT_CALIBRATE_SET_TOPIC) == 0) {
    handleMqttCalibrationCommand(payloadBuf);
    return;
  }
  if (strcmp(topic, MQTT_PI1_POWER_SET_TOPIC) == 0) {
    handleMqttPowerCommand(1, payloadBuf);
    return;
  }
  if (strcmp(topic, MQTT_PI2_POWER_SET_TOPIC) == 0) {
    handleMqttPowerCommand(2, payloadBuf);
    return;
  }
  if (strcmp(topic, MQTT_ALL_POWER_SET_TOPIC) == 0) {
    handleMqttPowerCommand(3, payloadBuf);
    return;
  }
  if (strcmp(topic, MQTT_SYSTEM_RESTART_TOPIC) == 0 &&
      (strcmp(payloadBuf, "restart") == 0 || strcmp(payloadBuf, "reboot") == 0)) {
    Serial.println("[MQTT] Pico soft restart requested.");
    mqttClient.publish(MQTT_AVAILABILITY_TOPIC, "offline", true);
    restartRequested = true;
    restartDueMs = millis() + 500UL;
    return;
  }

  Serial.printf("[MQTT] Unknown topic=%s payload=%s\n", topic, payloadBuf);
}

static void mqttConnectIfNeeded() {
  if (mqttClient.connected()) return;

  unsigned long now = millis();
  if (lastMqttReconnectAttemptMs != 0 &&
      now - lastMqttReconnectAttemptMs < MQTT_RECONNECT_INTERVAL_MS) return;
  lastMqttReconnectAttemptMs = now;

  Serial.printf("[MQTT] Connecting to %u.%u.%u.%u:%u\n",
                MQTT_BROKER_IP[0], MQTT_BROKER_IP[1], MQTT_BROKER_IP[2], MQTT_BROKER_IP[3], MQTT_PORT);

  bool ok = false;
  if (strlen(MQTT_USER) > 0) {
    ok = mqttClient.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASSWORD,
                            MQTT_AVAILABILITY_TOPIC, 1, true, "offline");
  } else {
    ok = mqttClient.connect(MQTT_CLIENT_ID, MQTT_AVAILABILITY_TOPIC, 1, true, "offline");
  }

  if (!ok) {
    Serial.printf("[MQTT] Connect failed rc=%d\n", mqttClient.state());
    return;
  }

  Serial.println("[MQTT] Connected.");
  mqttClient.publish(MQTT_AVAILABILITY_TOPIC, "online", true);
  mqttClient.subscribe(MQTT_CALIBRATE_SET_TOPIC);
  mqttClient.subscribe(MQTT_PI1_POWER_SET_TOPIC);
  mqttClient.subscribe(MQTT_PI2_POWER_SET_TOPIC);
  mqttClient.subscribe(MQTT_ALL_POWER_SET_TOPIC);
  mqttClient.subscribe(MQTT_SYSTEM_RESTART_TOPIC);
  mqttClient.subscribe(MQTT_HOME_ASSISTANT_STATUS_TOPIC);
  mqttPublishDiscovery();
  mqttPublishState();
}

static void processEthernetLinkWatchdog() {
  bool linkUp = Ethernet.linkStatus() != LinkOFF;
  unsigned long now = millis();

  if (linkUp) {
    if (!lastEthLinkUp) Serial.println("[ETH] Link watchdog: link up.");
    lastEthLinkUp = true;
    ethLinkDownStartedMs = 0;
    return;
  }

  if (lastEthLinkUp || ethLinkDownStartedMs == 0) {
    Serial.println("[ETH] Link watchdog: link down.");
    ethLinkDownStartedMs = now;
  }
  lastEthLinkUp = false;

  if (!ETH_LINK_WATCHDOG_ENABLED) return;
  if (ETH_LINK_WATCHDOG_RELAY_INDEX >= 2) return;
  if (batteryCriticalNow()) return;
  if (powerActions[ETH_LINK_WATCHDOG_RELAY_INDEX].action != POWER_ACTION_NONE) return;
  if (now - ethLinkDownStartedMs < ETH_LINK_DOWN_POWER_CYCLE_MS) return;
  if (lastEthLinkPowerCycleMs != 0 &&
      now - lastEthLinkPowerCycleMs < ETH_LINK_POWER_CYCLE_COOLDOWN_MS) return;

  // Apply the cooldown to every attempt, including a failed XL9535 write.
  // Otherwise a missing expander causes a tight retry loop that floods serial
  // output and repeatedly hammers the I2C bus until the device reappears.
  lastEthLinkPowerCycleMs = now;
  if (!sensor.ioPresent) {
    Serial.printf("[ETH] Link down for %lums; relay power-cycle skipped because XL9535 is unavailable.\n",
                  (unsigned long)(now - ethLinkDownStartedMs));
    return;
  }

  Serial.printf("[ETH] Link down for %lums; power-cycling relay Pi%u\n",
                (unsigned long)(now - ethLinkDownStartedMs),
                ETH_LINK_WATCHDOG_RELAY_INDEX + 1);
  beginPowerCycle(ETH_LINK_WATCHDOG_RELAY_INDEX);
}

// ============================================================================
// Read-only NUT server (RFC 9271 subset used by PVE-UPS)
// ============================================================================
static bool nutBatteryLow() {
  if (lowBatteryStage != LOW_BATTERY_IDLE || lowBatteryLockout) return true;
  return sensor.valid &&
         nutPowerSource == NUT_POWER_BATTERY &&
         sensor.vbus < FULL_FLOAT_VOLTAGE &&
         sensor.soc <= 5.0f;
}

static const char* nutStatusFlags() {
  if (lowBatteryStage != LOW_BATTERY_IDLE || lowBatteryLockout) return "OB LB";
  if (!sensor.valid) return "NOCOMM";
  if (nutPowerSource == NUT_POWER_BATTERY) {
    return nutBatteryLow() ? "OB LB DISCHRG" : "OB DISCHRG";
  }
  if (nutPowerSource == NUT_POWER_MAINS) {
    return sensor.currentA > CURRENT_DEADBAND_A ? "OL CHRG" : "OL";
  }
  // Never invent OL while the Pico has no evidence for either power source.
  // PVE-UPS treats this as unreachable/alarm rather than as restored utility.
  return "NOCOMM";
}

static void nutSendVariable(EthernetClient &client, const char *name, const char *value) {
  client.print(F("VAR "));
  client.print(NUT_UPS_NAME);
  client.print(' ');
  client.print(name);
  client.print(F(" \""));
  client.print(value);
  client.println('"');
}

static void nutSendListVariables(EthernetClient &client) {
  char value[32];

  client.print(F("BEGIN LIST VAR "));
  client.println(NUT_UPS_NAME);
  nutSendVariable(client, "ups.status", nutStatusFlags());
  nutSendVariable(client, "device.mfr", "DIY");
  nutSendVariable(client, "device.model", NUT_UPS_DESCRIPTION);
  nutSendVariable(client, "ups.firmware", FIRMWARE_VERSION);

  if (sensor.valid) {
    snprintf(value, sizeof(value), "%.1f", constrain(sensor.soc, 0.0f, 100.0f));
    nutSendVariable(client, "battery.charge", value);

    // The firmware intentionally has no runtime estimate while charging or
    // idle. Publish this variable only when it is a real discharge estimate.
    if (sensor.runtimeMin >= 0.0f && isfinite(sensor.runtimeMin)) {
      snprintf(value, sizeof(value), "%.0f", sensor.runtimeMin * 60.0f);
      nutSendVariable(client, "battery.runtime", value);
    }
  }

  client.print(F("END LIST VAR "));
  client.println(NUT_UPS_NAME);
}

static void handleNUT() {
  EthernetClient client = nutServer.available();
  if (!client) return;

  char line[96] = {0};
  size_t length = 0;
  bool complete = false;
  bool overflow = false;
  const unsigned long started = millis();

  while (client.connected() && millis() - started < 250UL && !complete) {
    while (client.available()) {
      const char c = (char)client.read();
      if (c == '\r') continue;
      if (c == '\n') {
        complete = true;
        break;
      }
      if (length + 1 < sizeof(line)) line[length++] = c;
      else overflow = true;
    }
    rp2040.wdt_reset();
  }
  line[length] = '\0';

  if (!complete || overflow) {
    client.println(F("ERR INVALID-ARGUMENT"));
  } else {
    char command[12] = {0};
    char argument[12] = {0};
    char upsName[40] = {0};
    const int fields = sscanf(line, "%11s %11s %39s", command, argument, upsName);

    if (fields == 3 && strcasecmp(command, "LIST") == 0 &&
        strcasecmp(argument, "VAR") == 0) {
      if (strcmp(upsName, NUT_UPS_NAME) == 0) nutSendListVariables(client);
      else client.println(F("ERR UNKNOWN-UPS"));
    } else if (fields == 2 && strcasecmp(command, "LIST") == 0 &&
               strcasecmp(argument, "UPS") == 0) {
      client.println(F("BEGIN LIST UPS"));
      client.print(F("UPS "));
      client.print(NUT_UPS_NAME);
      client.print(F(" \""));
      client.print(NUT_UPS_DESCRIPTION);
      client.println(F("\""));
      client.println(F("END LIST UPS"));
    } else if (fields == 1 && strcasecmp(command, "VER") == 0) {
      client.print(F("Pico UPS "));
      client.println(FIRMWARE_VERSION);
    } else if (fields == 1 && strcasecmp(command, "PROTVER") == 0) {
      client.println(F("1.3"));
    } else if (fields == 1 && strcasecmp(command, "LOGOUT") == 0) {
      client.println(F("OK Goodbye"));
    } else if (fields >= 1 &&
               (strcasecmp(command, "USERNAME") == 0 ||
                strcasecmp(command, "PASSWORD") == 0)) {
      // Read access is intentionally anonymous on the trusted management LAN.
      // Reject credentials instead of pretending they were authenticated.
      client.println(F("ERR ACCESS-DENIED"));
    } else {
      client.println(F("ERR INVALID-ARGUMENT"));
    }
  }

  client.flush();
  client.stop();
}

static void handlePowerCommand(EthernetClient &client, const char *path) {
  int target = 0;
  const char *command = nullptr;

  if (strncmp(path, "/api/pi/1/power/", 16) == 0) {
    target = 1;
    command = path + 16;
  } else if (strncmp(path, "/api/pi/2/power/", 16) == 0) {
    target = 2;
    command = path + 16;
  } else if (strncmp(path, "/api/pi/all/power/", 18) == 0) {
    target = 3;
    command = path + 18;
  } else if (strcmp(path, "/api/pi/1/power-cycle") == 0) {
    target = 1;
    command = "power-cycle";
  } else if (strcmp(path, "/api/pi/2/power-cycle") == 0) {
    target = 2;
    command = "power-cycle";
  } else if (strcmp(path, "/api/pi/all/power-cycle") == 0) {
    target = 3;
    command = "power-cycle";
  } else if (strcmp(path, "/api/pi/1/shutdown") == 0) {
    target = 1;
    command = "graceful-shutdown";
  } else if (strcmp(path, "/api/pi/2/shutdown") == 0) {
    target = 2;
    command = "graceful-shutdown";
  } else if (strcmp(path, "/api/pi/all/shutdown") == 0) {
    target = 3;
    command = "graceful-shutdown";
  }

  char body[256];
  if (target == 0 || !command) {
    snprintf(body, sizeof(body), "{\"ok\":false,\"error\":\"unknown_path\"}");
    sendJson(client, "HTTP/1.1 404 Not Found", body);
    return;
  }

  bool ok = handlePowerCommandTarget(target, command);
  refreshPiStatusFromAds();
  Serial.printf("[HTTP] Power command target=%d command=%s result=%s\n", target, command, ok ? "ok" : "failed");
  mqttPublishPowerStatus(command, target, ok);
  mqttPublishState();
  snprintf(body, sizeof(body), "{\"ok\":%s,\"command\":\"%s\",\"target\":%d}", ok ? "true" : "false", command, target);
  const char *successStatus = strcmp(command, "graceful-shutdown") == 0 ? "HTTP/1.1 202 Accepted" : "HTTP/1.1 200 OK";
  sendJson(client, ok ? successStatus : "HTTP/1.1 500 Internal Server Error", body);
}

static void handleStatusRequest(EthernetClient &client, bool fullApi) {
  char body[2048];
  if (fullApi) fullStatusJson(body, sizeof(body));
  else legacyStatusJson(body, sizeof(body));
  sendJson(client, "HTTP/1.1 200 OK", body);
}

static void handleBatteryStatus(EthernetClient &client) {
  char body[160];
  batteryStatusJson(body, sizeof(body));
  sendJson(client, "HTTP/1.1 200 OK", body);
}

// ============================================================================
// Shutdown API
// ============================================================================
static bool parseHttpUrl(const char *url, char *host, size_t hostLen, uint16_t &port, char *path, size_t pathLen) {
  if (!url || strncmp(url, "http://", 7) != 0) return false;
  const char *p = url + 7;
  const char *slash = strchr(p, '/');
  const char *colon = strchr(p, ':');

  port = 80;
  if (slash) {
    strncpy(path, slash, pathLen - 1);
    path[pathLen - 1] = '\0';
  } else {
    strncpy(path, "/", pathLen - 1);
    path[pathLen - 1] = '\0';
  }

  size_t hostPartLen = slash ? (size_t)(slash - p) : strlen(p);
  if (colon && (!slash || colon < slash)) {
    hostPartLen = (size_t)(colon - p);
    port = (uint16_t)atoi(colon + 1);
  }
  if (hostPartLen == 0 || hostPartLen >= hostLen) return false;

  memcpy(host, p, hostPartLen);
  host[hostPartLen] = '\0';
  return true;
}

static bool sendShutdownRequest(const char *url) {
  char host[64];
  char path[128];
  uint16_t port = 80;
  if (!parseHttpUrl(url, host, sizeof(host), port, path, sizeof(path))) {
    Serial.printf("[SHUTDOWN] Invalid URL: %s\n", url ? url : "(null)");
    return false;
  }

  EthernetClient client;
  client.setConnectionTimeout(2000);
  if (!client.connect(host, port)) {
    Serial.printf("[SHUTDOWN] Connect failed: %s:%u%s\n", host, port, path);
    return false;
  }

  client.print(F("POST "));
  client.print(path);
  client.println(F(" HTTP/1.1"));
  client.print(F("Host: "));
  client.println(host);
  client.println(F("Connection: close"));
  client.println(F("Content-Length: 0"));
  client.println();

  char statusLine[64] = {0};
  size_t statusLen = 0;
  unsigned long deadline = millis() + 2000;
  while (millis() < deadline && (client.connected() || client.available())) {
    while (client.available()) {
      char c = client.read();
      if (c == '\r') continue;
      if (c == '\n') {
        deadline = millis();
        break;
      }
      if (statusLen + 1 < sizeof(statusLine)) statusLine[statusLen++] = c;
    }
  }
  client.stop();
  bool accepted = strncmp(statusLine, "HTTP/1.1 2", 10) == 0 ||
                  strncmp(statusLine, "HTTP/1.0 2", 10) == 0;
  Serial.printf("[SHUTDOWN] POST %s:%u%s -> %s (%s)\n",
                host, port, path,
                statusLine[0] ? statusLine : "no response",
                accepted ? "accepted" : "failed");
  return accepted;
}

static void attemptManualGracefulShutdown(uint8_t piIndex) {
  if (piIndex >= 2 || gracefulShutdowns[piIndex].stage != GRACEFUL_SHUTDOWN_REQUESTING) return;
  gracefulShutdowns[piIndex].lastAttemptMs = millis();
  bool accepted = sendShutdownRequest(piConfig[piIndex].shutdownUrl);
  if (accepted) {
    gracefulShutdowns[piIndex].stage = GRACEFUL_SHUTDOWN_WAITING;
    gracefulShutdowns[piIndex].acceptedMs = millis();
    Serial.printf("[GRACEFUL_SHUTDOWN] Pi%u accepted; relay cutoff in %lus\n",
                  piIndex + 1, (unsigned long)(SHUTDOWN_GRACE_MS / 1000UL));
  } else {
    Serial.printf("[GRACEFUL_SHUTDOWN] Pi%u request failed; retrying in %lus\n",
                  piIndex + 1, (unsigned long)(SHUTDOWN_RETRY_MS / 1000UL));
  }
}

static bool startManualGracefulShutdown(uint8_t piIndex) {
  if (piIndex >= 2) return false;
  if (!piConfig[piIndex].enabled || !piConfig[piIndex].relayState) return true;
  if (strlen(piConfig[piIndex].shutdownUrl) == 0) return false;
  if (gracefulShutdowns[piIndex].stage != GRACEFUL_SHUTDOWN_IDLE) return true;

  powerActions[piIndex].action = POWER_ACTION_NONE;
  powerCycleStartedMs[piIndex] = 0;
  gracefulShutdowns[piIndex].stage = GRACEFUL_SHUTDOWN_REQUESTING;
  gracefulShutdowns[piIndex].lastAttemptMs = 0;
  gracefulShutdowns[piIndex].acceptedMs = 0;
  Serial.printf("[GRACEFUL_SHUTDOWN] Pi%u requested.\n", piIndex + 1);
  attemptManualGracefulShutdown(piIndex);
  return true;
}

static void processManualGracefulShutdowns() {
  for (uint8_t i = 0; i < 2; i++) {
    GracefulShutdownState &state = gracefulShutdowns[i];
    if (state.stage == GRACEFUL_SHUTDOWN_REQUESTING &&
        (state.lastAttemptMs == 0 || millis() - state.lastAttemptMs >= SHUTDOWN_RETRY_MS)) {
      attemptManualGracefulShutdown(i);
    }
    if (state.stage == GRACEFUL_SHUTDOWN_WAITING &&
        millis() - state.acceptedMs >= SHUTDOWN_GRACE_MS) {
      Serial.printf("[GRACEFUL_SHUTDOWN] Pi%u grace period complete; checking ADS host-off proof.\n", i + 1);
      bool relayOff = applyRelayState(i, false);
      if (relayOff) {
        state.stage = GRACEFUL_SHUTDOWN_IDLE;
        state.lastAttemptMs = 0;
        state.acceptedMs = 0;
      } else {
        Serial.printf("[GRACEFUL_SHUTDOWN] Pi%u relay cutoff blocked or failed; will retry.\n", i + 1);
      }
    }
  }
}

static void attemptPendingShutdownRequests() {
  lastShutdownRetryMs = millis();
  for (uint8_t i = 0; i < 2; i++) {
    if (!piConfig[i].enabled || shutdownApiAttempted[i] || strlen(piConfig[i].shutdownUrl) == 0) continue;
    shutdownApiAttempted[i] = sendShutdownRequest(piConfig[i].shutdownUrl);
    if (shutdownApiAttempted[i]) shutdownAcceptedMs[i] = millis();
    Serial.printf("[LOW_BATTERY] Pi%u shutdown request %s\n",
                  i + 1, shutdownApiAttempted[i] ? "accepted" : "failed; will retry");
  }
}

static void startLowBatterySequence() {
  lowBatteryStage = LOW_BATTERY_WAITING;
  lowBatteryStageStartedMs = millis();
  shutdownRecoveryCandidateStartMs = 0;
  Serial.println("[LOW_BATTERY] Stage -> waiting. Sending shutdown requests.");
  for (uint8_t i = 0; i < 2; i++) {
    shutdownApiAttempted[i] = false;
    shutdownAcceptedMs[i] = 0;
  }
  attemptPendingShutdownRequests();
}

static bool allRelayCutoffsConfirmedByAds() {
  if (!sensor.adsPresent) return false;
  for (uint8_t i = 0; i < 2; i++) {
    if (piConfig[i].relayState && !relayCutoffConfirmedByAds(i)) return false;
  }
  return true;
}

static bool finishLowBatterySequence() {
  Serial.println("[LOW_BATTERY] Wait complete. Re-checking output state and cutting relays if needed.");
  refreshPiStatusFromAds();
  if (!allRelayCutoffsConfirmedByAds()) {
    Serial.println("[LOW_BATTERY] Relay cutoff blocked: every powered output requires active-low PC817 on-to-off proof.");
    return false;
  }
  // Commit the lockout before removing host power. If the Pico browns out or
  // resets during relay cutoff, boot will read this flag and keep both outputs
  // off instead of immediately powering the hosts again.
  lowBatteryLockout = true;
  forceSave = true;
  if (!savePersistedState()) {
    Serial.println("[LOW_BATTERY] WARNING: persistent lockout save failed; continuing emergency cutoff.");
  } else {
    unsigned long now = millis();
    lastSaveMs = now;
    lastAnySaveMs = now;
    lastSavedWh = sensor.remainingWh;
    lastSavedState = batteryFlowState(sensor.currentA);
    forceSave = false;
  }

  for (uint8_t i = 0; i < 2; i++) {
    if (piConfig[i].relayState) applyRelayState(i, false);
  }
  autoRestoreCandidateStartMs = 0;
  shutdownRecoveryCandidateStartMs = 0;
  pveUpsHandoffArmed = false;
  pveUpsHandoffArmedMs = 0;
  pveUpsHandoffRecoveryStartMs = 0;
  lowBatteryStage = LOW_BATTERY_DONE;
  Serial.println("[LOW_BATTERY] Stage -> done.");
  return true;
}

static bool utilityPowerRecoveryNow() {
  return sensor.valid &&
         sensor.vbus >= AUTO_RESTORE_MIN_VOLTAGE &&
         sensor.currentA >= -CURRENT_DEADBAND_A &&
         sensor.soc >= AUTO_RESTORE_MIN_SOC;
}

static bool anyEnabledRelayOn() {
  for (uint8_t i = 0; i < 2; i++) {
    if (piConfig[i].enabled && piConfig[i].relayState) return true;
  }
  return false;
}

static bool pveUpsHandoffUtilityRecoveryNow() {
  return sensor.valid &&
         sensor.vbus >= AUTO_RESTORE_MIN_VOLTAGE &&
         sensor.currentA >= -CURRENT_DEADBAND_A;
}

static bool processPveUpsShutdownHandoff() {
  if (lowBatteryStage == LOW_BATTERY_DONE || lowBatteryLockout) {
    pveUpsHandoffArmed = false;
    pveUpsHandoffArmedMs = 0;
    pveUpsHandoffRecoveryStartMs = 0;
    return false;
  }

  if (!pveUpsHandoffArmed &&
      sensor.adsPresent &&
      sensor.valid &&
      nutPowerSource == NUT_POWER_BATTERY &&
      sensor.soc <= PVE_UPS_HANDOFF_SOC &&
      anyEnabledRelayOn()) {
    pveUpsHandoffArmed = true;
    pveUpsHandoffArmedMs = millis();
    pveUpsHandoffRecoveryStartMs = 0;
    Serial.printf(
      "[PVE_UPS_HANDOFF] Confirmed outage at %.1f%% SOC; waiting for final GPIO-OFF proof from every powered host.\n",
      sensor.soc);
  }

  if (!pveUpsHandoffArmed) return false;

  // Utility can return while PVE-UPS is already completing its shutdown plan.
  // Retain this RAM latch long enough to catch the final GPIO transitions and
  // perform one safe OFF-to-ON recovery. If no transition occurs, expiry avoids
  // applying this old outage expectation to a future unrelated host shutdown.
  if (pveUpsHandoffUtilityRecoveryNow()) {
    if (pveUpsHandoffRecoveryStartMs == 0) {
      pveUpsHandoffRecoveryStartMs = millis();
      Serial.printf(
        "[PVE_UPS_HANDOFF] Utility recovery detected; retaining handoff for %lus while shutdown may finish.\n",
        (unsigned long)(PVE_UPS_HANDOFF_RECOVERY_HOLD_MS / 1000UL));
    }
  } else {
    if (pveUpsHandoffRecoveryStartMs != 0) {
      Serial.println("[PVE_UPS_HANDOFF] Utility recovery evidence lost; expiry timer reset.");
    }
    pveUpsHandoffRecoveryStartMs = 0;
  }

  const bool graceComplete = millis() - pveUpsHandoffArmedMs >= SHUTDOWN_GRACE_MS;
  if (graceComplete && allRelayCutoffsConfirmedByAds()) {
    Serial.println(
      "[PVE_UPS_HANDOFF] Final GPIO-OFF proof received; committing persistent lockout and cutting relays.");
    return finishLowBatterySequence();
  }

  if (pveUpsHandoffRecoveryStartMs != 0 &&
      millis() - pveUpsHandoffRecoveryStartMs >= PVE_UPS_HANDOFF_RECOVERY_HOLD_MS) {
    Serial.println(
      "[PVE_UPS_HANDOFF] Recovery hold expired without complete GPIO-OFF proof; clearing handoff without changing relays.");
    pveUpsHandoffArmed = false;
    pveUpsHandoffArmedMs = 0;
    pveUpsHandoffRecoveryStartMs = 0;
  }
  return false;
}

static void clearNoAdsRestartState() {
  noAdsRestartPending = false;
  noAdsRecoveryCycleStarted = false;
  noAdsRecoveryCandidateStartMs = 0;
  for (uint8_t i = 0; i < 2; i++) noAdsRecoveryCycleRequested[i] = false;
}

static void armNoAdsRestartIfNeeded() {
  if (sensor.adsPresent || noAdsRestartPending || !sensor.valid ||
      lowBatteryStage == LOW_BATTERY_DONE) return;
  bool anyEnabledRelayOn = false;
  for (uint8_t i = 0; i < 2; i++) {
    if (piConfig[i].enabled && piConfig[i].relayState) anyEnabledRelayOn = true;
  }
  if (!anyEnabledRelayOn) return;
  const bool flatWithoutUtility = sensor.soc <= NO_ADS_RESTART_ARM_SOC &&
                                  sensor.vbus < FULL_FLOAT_VOLTAGE;
  if (!flatWithoutUtility) return;

  noAdsRestartPending = true;
  noAdsRecoveryCycleStarted = false;
  noAdsRecoveryCandidateStartMs = 0;
  for (uint8_t i = 0; i < 2; i++) noAdsRecoveryCycleRequested[i] = false;
  Serial.printf(
    "[NO_ADS_RECOVERY] SOC <= %.1f%% without ADS; assuming hosts halted. Relays stay on until utility returns.\n",
    NO_ADS_RESTART_ARM_SOC);
}

static void processNoAdsRecoveryRestart() {
  if (!noAdsRestartPending) {
    noAdsRecoveryCandidateStartMs = 0;
    return;
  }

  if (sensor.adsPresent) {
    Serial.println("[NO_ADS_RECOVERY] ADS1115 is available again; cancelling inferred host-off recovery cycle.");
    clearNoAdsRestartState();
    return;
  }

  if (!utilityPowerRecoveryNow()) {
    if (noAdsRecoveryCandidateStartMs != 0) {
      Serial.println("[NO_ADS_RECOVERY] Utility recovery evidence lost; confirmation timer reset.");
    }
    noAdsRecoveryCandidateStartMs = 0;
    return;
  }

  if (noAdsRecoveryCandidateStartMs == 0) {
    noAdsRecoveryCandidateStartMs = millis();
    Serial.printf(
      "[NO_ADS_RECOVERY] Utility recovery detected; confirming for %lus before one relay cycle.\n",
      (unsigned long)(AUTO_RESTORE_CONFIRM_MS / 1000UL));
    return;
  }

  if (millis() - noAdsRecoveryCandidateStartMs < AUTO_RESTORE_CONFIRM_MS) return;

  // Check every not-yet-cycled output before changing any additional relay.
  // Outputs already cycling are left alone and will be restored by the normal
  // power-action failsafe even if a later output temporarily cannot start.
  for (uint8_t i = 0; i < 2; i++) {
    if (!piConfig[i].enabled || noAdsRecoveryCycleRequested[i] || !piConfig[i].relayState) continue;
    if (gracefulShutdowns[i].stage != GRACEFUL_SHUTDOWN_IDLE ||
        powerActions[i].action != POWER_ACTION_NONE) {
      return;
    }
  }

  for (uint8_t i = 0; i < 2; i++) {
    if (!piConfig[i].enabled || noAdsRecoveryCycleRequested[i] || !piConfig[i].relayState) continue;
    if (beginPowerCycle(i, true)) {
      noAdsRecoveryCycleRequested[i] = true;
      noAdsRecoveryCycleStarted = true;
      Serial.printf("[NO_ADS_RECOVERY] Pi%u one-time recovery cycle started.\n", i + 1);
    }
  }

  bool anyTarget = false;
  bool allFinished = true;
  for (uint8_t i = 0; i < 2; i++) {
    if (!piConfig[i].enabled) continue;
    if (noAdsRecoveryCycleRequested[i]) {
      anyTarget = true;
      if (powerActions[i].action != POWER_ACTION_NONE || !piConfig[i].relayState) {
        allFinished = false;
      }
    } else if (piConfig[i].relayState) {
      anyTarget = true;
      allFinished = false;
    }
  }

  if ((!anyTarget || noAdsRecoveryCycleStarted) && allFinished) {
    Serial.println("[NO_ADS_RECOVERY] Recovery relay cycle complete; RAM latch cleared.");
    clearNoAdsRestartState();
  }
}

static void processAutomaticRelayRestore() {
  if (lowBatteryStage != LOW_BATTERY_DONE) {
    autoRestoreCandidateStartMs = 0;
    return;
  }

  if (!utilityPowerRecoveryNow()) {
    if (autoRestoreCandidateStartMs != 0) {
      Serial.println("[AUTO_RESTORE] Recovery evidence lost; confirmation timer reset.");
    }
    autoRestoreCandidateStartMs = 0;
    return;
  }

  if (autoRestoreCandidateStartMs == 0) {
    autoRestoreCandidateStartMs = millis();
    Serial.printf("[AUTO_RESTORE] Utility recovery detected; confirming for %lus (V>=%.2f SOC>=%.1f%% I>=%.2fA).\n",
                  (unsigned long)(AUTO_RESTORE_CONFIRM_MS / 1000UL),
                  AUTO_RESTORE_MIN_VOLTAGE,
                  AUTO_RESTORE_MIN_SOC,
                  -CURRENT_DEADBAND_A);
    return;
  }

  if (millis() - autoRestoreCandidateStartMs < AUTO_RESTORE_CONFIRM_MS) return;

  Serial.println("[AUTO_RESTORE] Utility recovery confirmed; enabling output relays.");
  bool allRestored = true;
  for (uint8_t i = 0; i < 2; i++) {
    if (!piConfig[i].relayState) {
      if (!applyRelayState(i, true)) allRestored = false;
    }
  }

  if (!allRestored) {
    Serial.println("[AUTO_RESTORE] Relay restore incomplete; keeping restored outputs on and retrying missing outputs after confirmation.");
    autoRestoreCandidateStartMs = 0;
    return;
  }

  lowBatteryStage = LOW_BATTERY_IDLE;
  lowBatteryStageStartedMs = 0;
  autoRestoreCandidateStartMs = 0;
  shutdownRecoveryCandidateStartMs = 0;
  criticalCandidateStartMs = 0;
  lowBatteryLockout = false;
  forceSave = true;
  // Persist the cleared latch immediately so a reset after successful restore
  // follows the normal boot-on policy.
  if (!savePersistedState()) {
    Serial.println("[AUTO_RESTORE] WARNING: failed to persist cleared shutdown lockout.");
  } else {
    unsigned long now = millis();
    lastSaveMs = now;
    lastAnySaveMs = now;
    lastSavedWh = sensor.remainingWh;
    lastSavedState = batteryFlowState(sensor.currentA);
    forceSave = false;
  }
  Serial.println("[AUTO_RESTORE] Output relays restored; low-battery stage -> idle.");
}

static void processLowBatteryProtection() {
  armNoAdsRestartIfNeeded();
  processNoAdsRecoveryRestart();

  if (processPveUpsShutdownHandoff()) return;

  if (lowBatteryStage == LOW_BATTERY_DONE) {
    processAutomaticRelayRestore();
    return;
  }

  bool critical = batteryCriticalNow();

  // Once shutdown has started, reduced host current is expected and must not
  // cancel the sequence. Only charger-level voltage plus non-discharging
  // current, held for a confirmation period, proves that utility returned.
  if (lowBatteryStage == LOW_BATTERY_WAITING) {
    bool utilityRecovered = sensor.valid &&
                            sensor.vbus >= AUTO_RESTORE_MIN_VOLTAGE &&
                            sensor.currentA >= -CURRENT_DEADBAND_A;
    if (utilityRecovered) {
      if (shutdownRecoveryCandidateStartMs == 0) {
        shutdownRecoveryCandidateStartMs = millis();
        Serial.println("[LOW_BATTERY] Utility recovery detected during shutdown; confirming before cancellation.");
      } else if (millis() - shutdownRecoveryCandidateStartMs >= UTILITY_RECOVERY_CANCEL_CONFIRM_MS) {
        lowBatteryStage = LOW_BATTERY_IDLE;
        lowBatteryStageStartedMs = 0;
        shutdownRecoveryCandidateStartMs = 0;
        criticalCandidateStartMs = 0;
        lastShutdownRetryMs = 0;
        for (uint8_t i = 0; i < 2; i++) {
          shutdownApiAttempted[i] = false;
          shutdownAcceptedMs[i] = 0;
        }
        Serial.println("[LOW_BATTERY] Pending shutdown cancelled after confirmed utility recovery.");
        return;
      }
    } else {
      shutdownRecoveryCandidateStartMs = 0;
    }

    if (millis() - lastShutdownRetryMs >= SHUTDOWN_RETRY_MS) {
      attemptPendingShutdownRequests();
    }

    bool allGracefulShutdownsReady = true;
    for (uint8_t i = 0; i < 2; i++) {
      if (!piConfig[i].enabled) continue;
      if (!shutdownApiAttempted[i] || shutdownAcceptedMs[i] == 0 ||
          millis() - shutdownAcceptedMs[i] < SHUTDOWN_GRACE_MS) {
        allGracefulShutdownsReady = false;
      }
    }

    bool emergencyCutoff = sensor.valid && sensor.vbus <= CRITICAL_VOLTAGE;
    if (allGracefulShutdownsReady || emergencyCutoff) {
      if (allRelayCutoffsConfirmedByAds()) {
        Serial.println(allGracefulShutdownsReady
                         ? "[LOW_BATTERY] Shutdowns accepted, grace period complete, and ADS confirms hosts off."
                         : "[LOW_BATTERY] Emergency voltage reached and ADS confirms hosts off.");
        finishLowBatterySequence();
      } else if (lastLowBatteryCutoffBlockedLogMs == 0 ||
                 millis() - lastLowBatteryCutoffBlockedLogMs >= 30000UL) {
        Serial.println(emergencyCutoff
                         ? "[LOW_BATTERY] Emergency voltage reached, but relay cutoff is blocked without ADS host-off proof."
                         : "[LOW_BATTERY] Grace period complete; waiting for active-low PC817 host-off proof before relay cutoff.");
        lastLowBatteryCutoffBlockedLogMs = millis();
      }
    }
    return;
  }

  if (!critical) {
    criticalCandidateStartMs = 0;
    return;
  }

  if (criticalCandidateStartMs == 0) {
    criticalCandidateStartMs = millis();
    Serial.println("[LOW_BATTERY] Critical reading detected; waiting for confirmation.");
    return;
  }

  if (millis() - criticalCandidateStartMs < LOW_BATTERY_CONFIRM_MS) return;

  if (lowBatteryStage == LOW_BATTERY_IDLE) {
    for (uint8_t i = 0; i < 2; i++) {
      if (powerActions[i].action == POWER_ACTION_RESTORE_ON) {
        Serial.println("[LOW_BATTERY] Critical state ignored while power-cycle restore is pending.");
        return;
      }
    }
    startLowBatterySequence();
    return;
  }

}

// ============================================================================
// HTTP server
// ============================================================================
static bool otaAuthorized(const char *query) {
  char token[64];
  return queryParam(query, "token", token, sizeof(token)) && strcmp(token, UPS_OTA_TOKEN) == 0;
}

static bool readFirmwareHeaders(EthernetClient &client, size_t &contentLength,
                                uint32_t &expectedCrc32, bool &hasExpectedCrc32) {
  contentLength = 0;
  expectedCrc32 = 0;
  hasExpectedCrc32 = false;
  char line[160];
  while (client.connected()) {
    size_t len = 0;
    unsigned long deadline = millis() + 2000UL;
    while (millis() < deadline) {
      if (!client.available()) {
        rp2040.wdt_reset();
        continue;
      }
      char c = client.read();
      if (c == '\r') continue;
      if (c == '\n') break;
      if (len + 1 < sizeof(line)) line[len++] = c;
    }
    line[len] = '\0';
    if (len == 0) return contentLength > 0;
    if (strncasecmp(line, "Content-Length:", 15) == 0) {
      contentLength = strtoul(line + 15, nullptr, 10);
    } else if (strncasecmp(line, "X-Firmware-CRC32:", 17) == 0) {
      const char *value = line + 17;
      while (*value == ' ' || *value == '\t') value++;
      char *end = nullptr;
      unsigned long parsed = strtoul(value, &end, 16);
      while (end && (*end == ' ' || *end == '\t')) end++;
      if (end && end != value && *end == '\0') {
        expectedCrc32 = (uint32_t)parsed;
        hasExpectedCrc32 = true;
      }
    }
  }
  return false;
}

static void otaDiscardCandidateFiles() {
  if (LittleFS.exists(OTA_CANDIDATE_TEMP_FILE)) LittleFS.remove(OTA_CANDIDATE_TEMP_FILE);
  if (LittleFS.exists(OTA_CANDIDATE_FILE)) LittleFS.remove(OTA_CANDIDATE_FILE);
}

static void handleFirmwareUpload(EthernetClient &client, const char *query) {
  if (!otaAuthorized(query)) {
    sendJson(client, "HTTP/1.1 403 Forbidden", "{\"ok\":false,\"error\":\"bad_token\"}");
    return;
  }

  size_t contentLength = 0;
  uint32_t expectedCrc32 = 0;
  bool hasExpectedCrc32 = false;
  if (!readFirmwareHeaders(client, contentLength, expectedCrc32, hasExpectedCrc32) ||
      contentLength < OTA_MIN_IMAGE_SIZE || contentLength > OTA_MAX_IMAGE_SIZE) {
    sendJson(client, "HTTP/1.1 411 Length Required", "{\"ok\":false,\"error\":\"invalid_content_length\"}");
    return;
  }
  if (!hasExpectedCrc32) {
    sendJson(client, "HTTP/1.1 400 Bad Request", "{\"ok\":false,\"error\":\"missing_firmware_crc32\"}");
    return;
  }
  if (!littleFsAvailable) {
    sendJson(client, "HTTP/1.1 507 Insufficient Storage", "{\"ok\":false,\"error\":\"littlefs_unavailable\"}");
    return;
  }
  if (otaPendingBoot) {
    sendJson(client, "HTTP/1.1 409 Conflict", "{\"ok\":false,\"error\":\"candidate_not_healthy\"}");
    return;
  }

  Serial.printf("[OTA] Receiving firmware (%lu bytes, CRC32=%08lX).\n",
                (unsigned long)contentLength, (unsigned long)expectedCrc32);
  otaDiscardCandidateFiles();
  if (!otaBackupRunningFirmware()) {
    sendJson(client, "HTTP/1.1 507 Insufficient Storage", "{\"ok\":false,\"error\":\"backup_failed\"}");
    return;
  }

  FSInfo fsInfo;
  if (!LittleFS.info(fsInfo) || fsInfo.totalBytes < fsInfo.usedBytes ||
      fsInfo.totalBytes - fsInfo.usedBytes < contentLength + OTA_FS_RESERVE_BYTES) {
    sendJson(client, "HTTP/1.1 507 Insufficient Storage", "{\"ok\":false,\"error\":\"candidate_no_space\"}");
    return;
  }

  File candidate = LittleFS.open(OTA_CANDIDATE_TEMP_FILE, "w");
  if (!candidate) {
    sendJson(client, "HTTP/1.1 507 Insufficient Storage", "{\"ok\":false,\"error\":\"candidate_open_failed\"}");
    return;
  }

  uint8_t buffer[1024];
  uint8_t imageHeader[OTA_RP2040_HEADER_SIZE];
  size_t headerBytes = 0;
  size_t received = 0;
  bool writeOk = true;
  uint32_t receivedCrc32 = 0xFFFFFFFFUL;
  unsigned long lastDataMs = millis();
  while (received < contentLength && millis() - lastDataMs < OTA_UPLOAD_IDLE_TIMEOUT_MS) {
    int available = client.available();
    if (available <= 0) {
      rp2040.wdt_reset();
      delay(1);
      continue;
    }
    size_t wanted = min((size_t)available, min(sizeof(buffer), contentLength - received));
    int count = client.read(buffer, wanted);
    if (count <= 0) continue;
    size_t copyCount = min((size_t)count, OTA_RP2040_HEADER_SIZE - headerBytes);
    if (copyCount > 0) {
      memcpy(imageHeader + headerBytes, buffer, copyCount);
      headerBytes += copyCount;
    }
    for (int i = 0; i < count; i++) receivedCrc32 = crc32Update(receivedCrc32, buffer[i]);
    if (candidate.write(buffer, count) != (size_t)count) {
      writeOk = false;
      break;
    }
    received += count;
    lastDataMs = millis();
    rp2040.wdt_reset();
  }
  candidate.flush();
  size_t stagedSize = candidate.size();
  candidate.close();
  receivedCrc32 = ~receivedCrc32;

  bool imageHeaderOk = headerBytes == OTA_RP2040_HEADER_SIZE &&
                       otaValidateRp2040ImageHeader(imageHeader, contentLength);
  bool ok = writeOk && received == contentLength && stagedSize == contentLength &&
            receivedCrc32 == expectedCrc32 && imageHeaderOk;
  if (!ok) {
    LittleFS.remove(OTA_CANDIDATE_TEMP_FILE);
    Serial.printf("[OTA] Upload rejected received=%lu/%lu staged=%lu write=%u header=%u CRC32=%08lX/%08lX.\n",
                  (unsigned long)received, (unsigned long)contentLength,
                  (unsigned long)stagedSize, writeOk ? 1 : 0, imageHeaderOk ? 1 : 0,
                  (unsigned long)receivedCrc32, (unsigned long)expectedCrc32);
    sendJson(client, "HTTP/1.1 400 Bad Request", "{\"ok\":false,\"error\":\"invalid_or_incomplete_firmware\"}");
    return;
  }

  if (!LittleFS.rename(OTA_CANDIDATE_TEMP_FILE, OTA_CANDIDATE_FILE)) {
    LittleFS.remove(OTA_CANDIDATE_TEMP_FILE);
    sendJson(client, "HTTP/1.1 507 Insufficient Storage", "{\"ok\":false,\"error\":\"candidate_commit_failed\"}");
    return;
  }
  // The boot-attempt marker is part of update safety, not optional metadata.
  // Persist it before scheduling the bootloader command so no candidate can be
  // installed without the rollback guard knowing it is pending.
  if (!otaWritePendingAttempts(0)) {
    LittleFS.remove(OTA_CANDIDATE_FILE);
    sendJson(client, "HTTP/1.1 507 Insufficient Storage", "{\"ok\":false,\"error\":\"pending_marker_failed\"}");
    return;
  }
  if (!otaScheduleFile(OTA_CANDIDATE_FILE)) {
    LittleFS.remove(OTA_PENDING_FILE);
    LittleFS.remove(OTA_CANDIDATE_FILE);
    sendJson(client, "HTTP/1.1 507 Insufficient Storage", "{\"ok\":false,\"error\":\"staging_failed\"}");
    return;
  }

  sendJson(client, "HTTP/1.1 202 Accepted", "{\"ok\":true,\"status\":\"staged\",\"restarting\":true}");
  Serial.printf("[OTA] Firmware validated, CRC32=%08lX, and staged; restarting.\n",
                (unsigned long)receivedCrc32);
  restartRequested = true;
  restartDueMs = millis() + 500UL;
}

static void handleFirmwareRollback(EthernetClient &client, const char *query) {
  if (!otaAuthorized(query)) {
    sendJson(client, "HTTP/1.1 403 Forbidden", "{\"ok\":false,\"error\":\"bad_token\"}");
    return;
  }
  if (!otaScheduleFile(OTA_PREVIOUS_FILE)) {
    sendJson(client, "HTTP/1.1 404 Not Found", "{\"ok\":false,\"error\":\"no_rollback_image\"}");
    return;
  }
  LittleFS.remove(OTA_PENDING_FILE);
  sendJson(client, "HTTP/1.1 202 Accepted", "{\"ok\":true,\"status\":\"rollback_staged\",\"restarting\":true}");
  Serial.println("[OTA] Manual rollback requested; restarting.");
  restartRequested = true;
  restartDueMs = millis() + 500UL;
}

static void handlePicoRestart(EthernetClient &client, const char *query) {
  if (!otaAuthorized(query)) {
    sendJson(client, "HTTP/1.1 403 Forbidden", "{\"ok\":false,\"error\":\"bad_token\"}");
    return;
  }
  sendJson(client, "HTTP/1.1 202 Accepted", "{\"ok\":true,\"restarting\":true}");
  Serial.println("[HTTP] Pico soft restart requested.");
  restartRequested = true;
  restartDueMs = millis() + 500UL;
}

static void handleHTTP() {
  EthernetClient client = httpServer.available();
  if (!client) return;

  char reqLine[192];
  uint8_t reqLen = 0;
  unsigned long started = millis();
  while (client.connected() && millis() - started < 1000) {
    if (!client.available()) continue;
    char c = client.read();
    if (c == '\r') continue;
    if (c == '\n') break;
    if (reqLen < sizeof(reqLine) - 1) reqLine[reqLen++] = c;
  }
  reqLine[reqLen] = '\0';

  char method[8] = {0};
  char pathBuf[160] = {0};
  sscanf(reqLine, "%7s %159s", method, pathBuf);

  char *query = strchr(pathBuf, '?');
  if (query) *query++ = '\0';
  Serial.printf("[HTTP] %s %s\n", method, pathBuf);

  if (strcmp(pathBuf, "/status") == 0) {
    handleStatusRequest(client, false);
    return;
  }
  if (strcmp(pathBuf, "/api/status") == 0) {
    handleStatusRequest(client, true);
    return;
  }
  if (strcmp(method, "GET") == 0 && strcmp(pathBuf, "/api/log") == 0) {
    sendDiagnosticLog(client);
    return;
  }
  if (strcmp(pathBuf, "/api/battery/status") == 0) {
    handleBatteryStatus(client);
    return;
  }
  if (strcmp(pathBuf, "/calibrate") == 0) {
    handleCalibrateHTTP(client, query ? query : "");
    return;
  }
  if (strcmp(method, "POST") == 0 && strcmp(pathBuf, "/api/firmware") == 0) {
    handleFirmwareUpload(client, query ? query : "");
    return;
  }
  if (strcmp(method, "POST") == 0 && strcmp(pathBuf, "/api/firmware/rollback") == 0) {
    handleFirmwareRollback(client, query ? query : "");
    return;
  }
  if (strcmp(method, "POST") == 0 && strcmp(pathBuf, "/api/system/restart") == 0) {
    handlePicoRestart(client, query ? query : "");
    return;
  }
  if (strcmp(method, "POST") == 0 && strncmp(pathBuf, "/api/pi/", 8) == 0) {
    handlePowerCommand(client, pathBuf);
    return;
  }

  sendJson(client, "HTTP/1.1 404 Not Found", "{\"ok\":false,\"error\":\"not_found\"}");
}

// ============================================================================
// Setup / loop
// ============================================================================
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 5000) delay(10);
  Serial.println("\n[UPS Battery Monitor]");
  Serial.printf("Firmware: %s\n", FIRMWARE_VERSION);
  otaBootGuardBegin();
  rp2040.wdt_begin(OTA_WATCHDOG_MS);

  recoverI2cBusPins();
  delay(I2C_BOOT_SETTLE_MS);
  rp2040.wdt_reset();
  Wire.setSDA(I2C_SDA);
  Wire.setSCL(I2C_SCL);
  Wire.begin();
  Wire.setClock(I2C_CLOCK_HZ);

  discoverI2cDevices(true, false);
  lastI2cRediscoveryMs = millis();

  loadPersistedState();
  restoreSaveTracking();

  xl9535Init();
  initLtc2944();
  updateBatteryModel();
  setBootRelayTargets();
  applyRelayState(0, piConfig[0].relayState);
  applyRelayState(1, piConfig[1].relayState);
  refreshPiStatusFromAds();
  updateNutPowerSource();

  SPI.setSCK(W5500_SCK);
  SPI.setTX(W5500_MOSI);
  SPI.setRX(W5500_MISO);
  SPI.setCS(W5500_CS);
  SPI.begin();

  pinMode(W5500_RST, OUTPUT);
  digitalWrite(W5500_RST, LOW);
  delay(20);
  digitalWrite(W5500_RST, HIGH);
  delay(150);

  Ethernet.init(W5500_CS);
  Ethernet.begin(mac, FALLBACK_IP, FALLBACK_DNS, FALLBACK_GW, FALLBACK_SN);
  Serial.println("[ETH] W5500 initialized with static IP.");
  if (Ethernet.linkStatus() == LinkOFF) Serial.println("[ETH] Link down.");

  httpServer.begin();
  nutServer.begin();
  mdns.begin(Ethernet.localIP(), HOSTNAME);
  mqttClient.setServer(MQTT_BROKER_IP, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setBufferSize(2048);
  mqttClient.setSocketTimeout(3);
  lastEthLinkUp = Ethernet.linkStatus() != LinkOFF;
  Serial.print("IP: ");
  Serial.println(Ethernet.localIP());
  Serial.printf("HTTP: http://%s.local/status\n", HOSTNAME);
  Serial.printf("NUT: %s.local:%u UPS=%s (read-only)\n", HOSTNAME, NUT_PORT, NUT_UPS_NAME);
  Serial.printf("MQTT: %u.%u.%u.%u:%u base=%s\n",
                MQTT_BROKER_IP[0], MQTT_BROKER_IP[1], MQTT_BROKER_IP[2], MQTT_BROKER_IP[3], MQTT_PORT, MQTT_BASE_TOPIC);
}

void loop() {
  rp2040.wdt_reset();
  otaMarkHealthyIfReady();
  if (restartRequested && millis() >= restartDueMs) {
    mqttClient.disconnect();
    delay(20);
    rp2040.reboot();
  }
  Ethernet.maintain();
  mdns.run();
  mqttConnectIfNeeded();
  mqttClient.loop();
  processEthernetLinkWatchdog();
  handleNUT();
  handleHTTP();
  processManualGracefulShutdowns();

  if (millis() - lastI2cRediscoveryMs >= I2C_REDISCOVERY_INTERVAL_MS) {
    lastI2cRediscoveryMs = millis();
    if (!sensor.ltcPresent || !sensor.adsPresent || !sensor.ioPresent || !sensor.eepromPresent) {
      Serial.println("[I2C] Re-scanning missing devices.");
      discoverI2cDevices(true, true);
    }
  }

  for (uint8_t i = 0; i < 2; i++) {
    if (powerActions[i].action == POWER_ACTION_RESTORE_ON && millis() >= powerActions[i].dueMs) {
      Serial.printf("[POWER] Pi%u power-cycle restore on\n", i + 1);
      if (applyRelayState(i, true)) {
        powerActions[i].action = POWER_ACTION_NONE;
        powerCycleStartedMs[i] = 0;
      } else {
        powerActions[i].dueMs = millis() + 1000UL;
      }
    } else if (powerActions[i].action == POWER_ACTION_RESTORE_ON &&
               powerCycleStartedMs[i] != 0 &&
               millis() - powerCycleStartedMs[i] >= POWER_CYCLE_FAILSAFE_MS) {
      Serial.printf("[POWER] Pi%u power-cycle failsafe restore on\n", i + 1);
      if (applyRelayState(i, true)) {
        powerActions[i].action = POWER_ACTION_NONE;
        powerCycleStartedMs[i] = 0;
      } else {
        powerActions[i].dueMs = millis() + 1000UL;
      }
    }
  }

  if (millis() - lastTelemetryMs >= TELEMETRY_INTERVAL_MS) {
    lastTelemetryMs = millis();
    updateBatteryModel();
    refreshPiStatusFromAds();
    savePersistedStateIfNeeded();
    processLowBatteryProtection();
    updateNutPowerSource();
    mqttPublishState();

    Serial.printf(
      "SOC=%.1f%% Wh=%.2f Runtime=%.1f V=%.2f I=%.3f T=%.1fC Pi1=%d/%.2fV Pi2=%d/%.2fV\n",
      sensor.soc,
      sensor.remainingWh,
      sensor.runtimeMin,
      sensor.vbus,
      sensor.currentA,
      finiteOrZero(sensor.temperatureC),
      piStatus[0].sensedOn ? 1 : 0,
      displayedPiVoltage(0),
      piStatus[1].sensedOn ? 1 : 0,
      displayedPiVoltage(1)
    );
  }
}
