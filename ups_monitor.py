#!/usr/bin/env python3
"""
UPS Battery Monitor - Raspberry Pi / Proxmox VE shutdown agent
Polls the Pico W5500 HTTP endpoint, logs readings to SQLite, and shuts down
gracefully when the battery is low.

The Pico itself does the battery energy counting (with flash persistence
and manual calibration) and reports soc/remaining_wh/runtime_min directly -
this script trusts those numbers rather than re-deriving its own estimate
from voltage.

Install:
    apt install python3-requests
    sudo cp ups_monitor.py /usr/local/bin/ups_monitor.py
    sudo cp ups_monitor.service /etc/systemd/system/
    sudo systemctl enable --now ups_monitor
"""

import json
import logging
import os
import shutil
import sqlite3
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit

import requests

def env_bool(name, default):
    value = os.environ.get(name)
    if value is None:
        return default
    return value.strip().lower() in {"1", "true", "yes", "on"}


# Config
PICO_URLS = [
    os.environ.get("PICO_URL"),
    "http://192.168.6.122/status",  # Arduino fallback IP; avoids mDNS on Proxmox/Debian
    "http://pico-ups.local/status",
]
POLL_INTERVAL = int(os.environ.get("UPS_POLL_INTERVAL", "30"))
FETCH_TIMEOUT = float(os.environ.get("UPS_FETCH_TIMEOUT", "5"))
FAIL_LIMIT = int(os.environ.get("UPS_FAIL_LIMIT", "5"))
LOG_FILE = os.environ.get("UPS_LOG_FILE", "/var/log/ups_monitor.log")
DB_PATH = os.environ.get("UPS_MONITOR_DB", "/var/lib/ups_monitor/ups_monitor.sqlite3")
API_ENABLED = env_bool("UPS_SHUTDOWN_API_ENABLED", True)
API_HOST = os.environ.get("UPS_SHUTDOWN_API_HOST", "0.0.0.0")
API_PORT = int(os.environ.get("UPS_SHUTDOWN_API_PORT", "80"))
API_ALLOWED_IP = os.environ.get("UPS_SHUTDOWN_ALLOWED_IP", "192.168.6.122").strip()

CURRENT_DEADBAND_A = 0.05   # matches the Pico firmware's charge/discharge deadband
FULL_FLOAT_VOLTAGE = 13.40  # critical shutdown is valid only below charging/float voltage
CRITICAL_VOLTAGE = 10.80    # matches the Pico firmware's emergency clamp

SOC_WARN = 30.0
SOC_SHUTDOWN = 5.0
RUNTIME_WARN_MIN = 20.0
RUNTIME_SHUTDOWN_MIN = -1.0  # disabled; shutdown is SOC-based, with voltage as emergency protection
LOW_BATTERY_CONFIRM = float(os.environ.get("UPS_LOW_BATTERY_CONFIRM", "30"))

shutdown_requested = threading.Event()

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s  %(levelname)-8s  %(message)s",
    handlers=[
        logging.FileHandler(LOG_FILE),
        logging.StreamHandler(sys.stdout),
    ],
)
log = logging.getLogger(__name__)


class ShutdownRequestHandler(BaseHTTPRequestHandler):
    server_version = "UPSShutdownAgent/1.0"

    def send_json(self, status, payload):
        body = json.dumps(payload, separators=(",", ":")).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Connection", "close")
        self.end_headers()
        self.wfile.write(body)
        self.wfile.flush()

    def do_GET(self):
        if urlsplit(self.path).path != "/health":
            self.send_json(404, {"error": "not found"})
            return
        self.send_json(200, {"status": "ok", "shutdown_requested": shutdown_requested.is_set()})

    def do_POST(self):
        if urlsplit(self.path).path != "/api/shutdown":
            self.send_json(404, {"error": "not found"})
            return
        client_ip = self.client_address[0]
        if API_ALLOWED_IP and client_ip != API_ALLOWED_IP:
            log.warning("Rejected shutdown request from %s", client_ip)
            self.send_json(403, {"error": "forbidden"})
            return
        self.send_json(202, {"status": "accepted"})
        log.critical("Firmware shutdown request accepted from %s", client_ip)
        shutdown_requested.set()

    def log_message(self, fmt, *args):
        log.info("Shutdown API %s - %s", self.client_address[0], fmt % args)


def start_shutdown_api():
    if not API_ENABLED:
        log.info("Shutdown API disabled by configuration.")
        return None
    server = ThreadingHTTPServer((API_HOST, API_PORT), ShutdownRequestHandler)
    server.daemon_threads = True
    thread = threading.Thread(target=server.serve_forever, name="shutdown-api", daemon=True)
    thread.start()
    log.info(
        "Shutdown API listening on %s:%d; POST /api/shutdown allowed from %s",
        API_HOST,
        API_PORT,
        API_ALLOWED_IP or "any address",
    )
    return server


def unique_urls():
    seen = set()
    for url in PICO_URLS:
        if url and url not in seen:
            seen.add(url)
            yield url


def fetch_status():
    """Return (status_dict, url) from the first reachable Pico endpoint."""
    errors = []
    for url in unique_urls():
        try:
            r = requests.get(url, timeout=FETCH_TIMEOUT)
            r.raise_for_status()
            return r.json(), url
        except Exception as e:
            errors.append(f"{url}: {e}")
    log.warning("Fetch failed: %s", " | ".join(errors))
    return None, None


READINGS_COLUMNS = ["ts", "url", "vbus", "current_a", "power_w", "soc", "remaining_wh", "runtime_min", "ina"]
READINGS_COLUMN_DEFS = """
    ts REAL NOT NULL,
    url TEXT,
    vbus REAL,
    current_a REAL,
    power_w REAL,
    soc REAL,
    remaining_wh REAL,
    runtime_min REAL,
    ina INTEGER
"""


class ReadingLog:
    """Logs each Pico reading to SQLite for local history/diagnostics. The
    Pico - not this script - owns the battery energy estimate."""

    def __init__(self, path):
        self.path = path
        os.makedirs(os.path.dirname(path), exist_ok=True)
        self.db = sqlite3.connect(path)
        self.db.execute("PRAGMA journal_mode=DELETE")
        self.db.execute(f"CREATE TABLE IF NOT EXISTS readings ({READINGS_COLUMN_DEFS})")

        existing = [row[1] for row in self.db.execute("PRAGMA table_info(readings)")]
        if existing != READINGS_COLUMNS:
            log.warning("readings table schema changed - dropping old local history (%s)", self.path)
            self.db.execute("DROP TABLE readings")
            self.db.execute(f"CREATE TABLE readings ({READINGS_COLUMN_DEFS})")
        self.db.commit()

    def record(self, status, url):
        now = time.time()
        power_w = abs(float(status.get("p", 0) or 0))

        self.db.execute(
            """
            INSERT INTO readings
            (ts, url, vbus, current_a, power_w, soc, remaining_wh, runtime_min, ina)
            VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
            """,
            (
                now,
                url,
                float(status.get("vbus", 0) or 0),
                float(status.get("i", 0) or 0),
                power_w,
                float(status.get("soc", -1) or -1),
                float(status.get("remaining_wh", -1) or -1),
                float(status.get("runtime_min", -1) or -1),
                int(bool(status.get("ina", False))),
            ),
        )
        self.db.commit()

        status["power_w"] = power_w
        return status


def do_shutdown():
    log.critical("Initiating system shutdown now.")
    try:
        if shutil.which("qm"):
            log.critical("Stopping all Proxmox VMs...")
            vms = subprocess.run(["qm", "list"], capture_output=True, text=True)
            for line in vms.stdout.splitlines()[1:]:
                vmid = line.split()[0]
                log.critical("Shutting down VM %s", vmid)
                subprocess.run(["qm", "shutdown", vmid, "--timeout", "30"], check=False)
        if shutil.which("pct"):
            log.critical("Stopping all Proxmox containers...")
            cts = subprocess.run(["pct", "list"], capture_output=True, text=True)
            for line in cts.stdout.splitlines()[1:]:
                ctid = line.split()[0]
                log.critical("Shutting down CT %s", ctid)
                subprocess.run(["pct", "shutdown", ctid, "--timeout", "30"], check=False)
    except Exception as e:
        log.error("Error during guest shutdown: %s", e)
    subprocess.run(["poweroff"], check=False)


def shutdown_reason(status):
    soc = float(status.get("soc", -1) or -1)
    vbus = float(status.get("vbus", 0) or 0)
    current_a = float(status.get("i", 0) or 0)
    runtime_min = float(status.get("runtime_min", -1) or -1)
    if not bool(status.get("ina", False)):
        return None  # firmware has no valid LTC2944 battery reading
    if current_a > -CURRENT_DEADBAND_A:
        return None  # charging or idle, not actually discharging - never shut down for that
    if vbus <= 0 or vbus >= FULL_FLOAT_VOLTAGE:
        return None
    if 0 <= soc <= SOC_SHUTDOWN:
        return f"battery SOC {soc:.1f}%"
    if 0 <= runtime_min <= RUNTIME_SHUTDOWN_MIN:
        return f"estimated runtime {runtime_min:.1f} min"
    return None


def warn_if_low(status):
    soc = float(status.get("soc", -1) or -1)
    runtime_min = float(status.get("runtime_min", -1) or -1)
    if 0 <= soc and SOC_SHUTDOWN < soc <= SOC_WARN:
        log.warning("Battery low: %.1f%%.", soc)
    if RUNTIME_SHUTDOWN_MIN < runtime_min <= RUNTIME_WARN_MIN:
        log.warning("Battery runtime low: %.1f min estimated.", runtime_min)


def main():
    log.info("UPS monitor started. Polling %s every %ds", list(unique_urls()), POLL_INTERVAL)
    api_server = start_shutdown_api()
    store = ReadingLog(DB_PATH)
    fail_count = 0
    critical_since = None
    armed_reason = None

    try:
        while not shutdown_requested.is_set():
            raw_status, url = fetch_status()

            if raw_status is None:
                fail_count += 1
                log.warning("Consecutive fetch failures: %d/%d", fail_count, FAIL_LIMIT)
                if fail_count >= FAIL_LIMIT:
                    log.error("Pico unreachable for too long - NOT shutting down without data.")
                    fail_count = 0
                shutdown_requested.wait(POLL_INTERVAL)
                continue

            fail_count = 0
            status = store.record(raw_status, url)
            log.info(
                "SOC=%.1f%%  Wh=%.2f  Runtime=%.1fmin  V=%.2fV  I=%.2fA  P=%.2fW  sensor=%s  url=%s",
                float(status.get("soc", -1) or -1),
                float(status.get("remaining_wh", -1) or -1),
                float(status.get("runtime_min", -1) or -1),
                float(status.get("vbus", 0) or 0),
                float(status.get("i", 0) or 0),
                status["power_w"],
                status.get("ina", False),
                url,
            )

            warn_if_low(status)
            reason = shutdown_reason(status)

            if reason and critical_since is None:
                critical_since = time.monotonic()
                armed_reason = reason
                log.critical(
                    "Battery critical (%s) - confirming for %.0fs unless it recovers.",
                    reason,
                    LOW_BATTERY_CONFIRM,
                )

            if critical_since is not None and not reason:
                critical_since = None
                armed_reason = None
                log.info("Battery recovered - fallback shutdown cancelled.")

            if critical_since is not None and reason and reason != armed_reason:
                armed_reason = reason
                log.critical("Shutdown reason updated: %s.", reason)

            if critical_since is not None and time.monotonic() - critical_since >= LOW_BATTERY_CONFIRM:
                log.critical("Critical battery confirmed; fallback shutdown requested.")
                shutdown_requested.set()
                break

            shutdown_requested.wait(5 if critical_since is not None else POLL_INTERVAL)
    finally:
        if api_server is not None:
            api_server.shutdown()
            api_server.server_close()

    if shutdown_requested.is_set():
        do_shutdown()


if __name__ == "__main__":
    main()
