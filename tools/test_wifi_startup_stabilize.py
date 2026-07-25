#!/usr/bin/env python3
import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "include/edog_config.h"
WIFI_TASK = ROOT / "src/wifi_mqtt_task.c"


def read(path):
    return path.read_text(encoding="utf-8-sig")


def macro_value(text, name):
    match = re.search(rf"#define\s+{name}\s+(\d+)\b", text)
    if not match:
        raise AssertionError(f"missing macro {name}")
    return int(match.group(1))


def function_body(text, name):
    match = re.search(rf"void\s+{name}\s*\([^)]*\)\s*\{{(?P<body>.*?)\n\}}", text, re.S)
    if not match:
        raise AssertionError(f"missing function {name}")
    return match.group("body")


def main():
    config = read(CONFIG)
    wifi_task = read(WIFI_TASK)

    startup_ms = macro_value(config, "EDOG_WIFI_STARTUP_STABILIZE_MS")
    if startup_ms < 1000:
        raise AssertionError("WiFi startup stabilize delay must leave board init time")

    mqtt_priority = macro_value(config, "EDOG_TASK_WIFI_MQTT_PRIORITY")
    motion_priority = macro_value(config, "EDOG_TASK_MOTION_PRIORITY")
    if mqtt_priority >= motion_priority:
        raise AssertionError("WiFi/MQTT task must still preempt motion after startup")

    body = function_body(wifi_task, "WifiTask")
    delay_token = "LOS_Msleep(EDOG_WIFI_STARTUP_STABILIZE_MS);"
    if delay_token not in body:
        raise AssertionError("WifiTask must wait before starting WiFi association")
    first_delay = body.find(delay_token)
    first_connect = body.find("EnsureProvisionedNetworkReady()")
    if first_connect < 0:
        raise AssertionError("WifiTask missing provisioning/connect loop")
    if first_delay < 0 or first_delay > first_connect:
        raise AssertionError("WiFi startup wait must happen before first connect attempt")

    print("WiFi startup stabilization contract checks passed")


if __name__ == "__main__":
    try:
        main()
    except AssertionError as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        sys.exit(1)
