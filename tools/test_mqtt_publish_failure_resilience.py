#!/usr/bin/env python3
import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
IOT_SRC = ROOT / "utils/src/iot.c"


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


def main():
    iot = IOT_SRC.read_text(encoding="utf-8")

    for token in [
        "static unsigned int mqttPublishFailureCount = 0;",
        "EDOG_MQTT_PUBLISH_FAILURE_RECONNECT_THRESHOLD",
        "static void MarkMqttPublishFailure(const char *source, int rc)",
        "mqttPublishFailureCount++",
        "mqttPublishFailureCount >= EDOG_MQTT_PUBLISH_FAILURE_RECONNECT_THRESHOLD",
        "static void MarkMqttPublishSuccess(void)",
    ]:
        require(iot, token)

    failure_body = function_body(iot, r"static\s+void\s+MarkMqttPublishFailure\s*\([^)]*\)")
    require(failure_body, "mqttConnectFlag = 0;")

    for name in [
        "PublishSystemCommandResponse",
        "mqtt_send_heartbeat",
        "send_servo_calibration_properties",
        "send_servo_status_properties",
        "send_msg_to_mqtt",
    ]:
        body = function_body(iot, rf"(?:static\s+void|void)\s+{name}\s*\([^)]*\)")
        require(body, "MarkMqttPublishFailure(")
        require(body, "MarkMqttPublishSuccess();")
        if "mqttConnectFlag = 0;" in body:
            raise AssertionError(f"{name} must not clear MQTT on the first publish failure")

    disconnect_body = function_body(iot, r"void\s+mqtt_disconnect\s*\([^)]*\)")
    init_body = function_body(iot, r"int\s+mqtt_init\s*\([^)]*\)")
    require(disconnect_body, "mqttPublishFailureCount = 0;")
    require(init_body, "mqttPublishFailureCount = 0;")

    print("MQTT publish failure resilience checks passed")


if __name__ == "__main__":
    try:
        main()
    except AssertionError as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        sys.exit(1)
