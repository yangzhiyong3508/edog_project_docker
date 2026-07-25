#!/usr/bin/env python3
import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CONFIG_SRC = ROOT / "include/edog_config.h"


def require_priority(config, name):
    match = re.search(rf"#define\s+{name}\s+(\d+)\b", config)
    if not match:
        raise AssertionError(f"missing {name}")
    return int(match.group(1))


def main():
    config = CONFIG_SRC.read_text(encoding="utf-8")
    mqtt_priority = require_priority(config, "EDOG_TASK_WIFI_MQTT_PRIORITY")
    motion_priority = require_priority(config, "EDOG_TASK_MOTION_PRIORITY")

    if mqtt_priority >= motion_priority:
        raise AssertionError(
            "WiFi/MQTT task priority must be numerically lower than motion task "
            "so MQTT can preempt gait playback when web commands arrive"
        )

    print("MQTT task priority preemption check passed")


if __name__ == "__main__":
    try:
        main()
    except AssertionError as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        sys.exit(1)
