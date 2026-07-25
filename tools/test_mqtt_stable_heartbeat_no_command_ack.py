#!/usr/bin/env python3
import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CONFIG_SRC = ROOT / "include/edog_config.h"
IOT_SRC = ROOT / "utils/src/iot.c"
WIFI_TASK_SRC = ROOT / "src/wifi_mqtt_task.c"


def function_body(text, signature):
    match = re.search(signature + r"\s*\{", text)
    if not match:
        raise AssertionError(f"missing function matching {signature}")
    start = match.end()
    depth = 1
    pos = start
    while pos < len(text) and depth:
        if text[pos] == "{":
            depth += 1
        elif text[pos] == "}":
            depth -= 1
        pos += 1
    if depth != 0:
        raise AssertionError(f"unterminated function matching {signature}")
    return text[start:pos - 1]


def require(text, token):
    if token not in text:
        raise AssertionError(f"missing token: {token}")


def forbid(text, token):
    if token in text:
        raise AssertionError(f"forbidden token present: {token}")


def require_define(config, name, expected_value):
    pattern = rf"#define\s+{name}\s+{expected_value}\b"
    if not re.search(pattern, config):
        raise AssertionError(f"{name} must be {expected_value}")


def main():
    config = CONFIG_SRC.read_text(encoding="utf-8")
    iot = IOT_SRC.read_text(encoding="utf-8")
    wifi_task = WIFI_TASK_SRC.read_text(encoding="utf-8")

    require_define(config, "EDOG_WIFI_MQTT_HEARTBEAT_MS", "30000")

    heartbeat_body = function_body(iot, r"void\s+mqtt_send_heartbeat\s*\([^)]*\)")
    require(heartbeat_body, "MQTTPublish(&client, heartbeat_topic, &message)")
    require(heartbeat_body, "MarkMqttPublishFailure(\"heartbeat\", rc)")
    require(heartbeat_body, "MarkMqttPublishSuccess();")

    edog_body = function_body(iot, r"void\s+mqtt_edog_message_arrived\s*\([^)]*\)")
    require(edog_body, "[MQTT] edog command handled rc=%d")
    forbid(edog_body, "mqtt_send_command_ack")

    init_body = function_body(iot, r"int\s+mqtt_init\s*\([^)]*\)")
    forbid(init_body, "mqtt_send_heartbeat();")

    require(wifi_task, "heartbeatElapsedMs >= EDOG_WIFI_MQTT_HEARTBEAT_MS")
    forbid(iot, "static void mqtt_send_command_ack")
    forbid(iot, "cmd_ack")
    forbid(iot, "CommandAck")

    print("MQTT stable heartbeat and no custom command ack checks passed")


if __name__ == "__main__":
    try:
        main()
    except AssertionError as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        sys.exit(1)
