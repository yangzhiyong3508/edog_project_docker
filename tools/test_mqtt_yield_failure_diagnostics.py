#!/usr/bin/env python3
import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
IOT_SRC = ROOT / "utils/src/iot.c"
MQTT_CLIENT_SRC = ROOT.parents[4] / "third_party/paho_mqtt/MQTTClient-C/src/MQTTClient.c"
MQTT_LITEOS_SRC = ROOT.parents[4] / "third_party/paho_mqtt/MQTTClient-C/src/liteOS/MQTTLiteOS.c"


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
    mqtt_client = MQTT_CLIENT_SRC.read_text(encoding="utf-8")
    liteos = MQTT_LITEOS_SRC.read_text(encoding="utf-8")

    wait_body = function_body(iot, r"int\s+wait_message\s*\([^)]*\)")
    for token in [
        "static unsigned int mqttYieldFailureCount = 0;",
        "EDOG_MQTT_YIELD_FAILURE_RECONNECT_THRESHOLD",
        "mqttYieldFailureCount++",
        "mqttYieldFailureCount >= EDOG_MQTT_YIELD_FAILURE_RECONNECT_THRESHOLD",
        "yield failed rc=%d consecutive=%u",
        "mqttYieldFailureCount = 0;",
    ]:
        require(iot if token.startswith("static") or token.startswith("EDOG_") else wait_body, token)

    for token in [
        "[MQTT] readPacket header read failed rc=%d left_ms=%d",
        "[MQTT] cycle fatal packet_type=%d connected=%d ping_out=%d",
        "[MQTT] keepalive failed connected=%d ping_out=%d",
        "[MQTT] yield cycle failed cycle_rc=%d left_ms=%d",
    ]:
        require(mqtt_client, token)

    for token in [
        "[MQTT] liteos select failed errno=%d socket=%d",
        "[MQTT] liteos recv failed errno=%d socket=%d",
        "[MQTT] liteos recv peer closed socket=%d",
        "errno != EAGAIN && errno != EWOULDBLOCK && errno != ETIMEDOUT",
        "errno != EINTR && errno != ECONNABORTED",
    ]:
        require(liteos, token)

    print("MQTT yield failure diagnostics checks passed")


if __name__ == "__main__":
    try:
        main()
    except AssertionError as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        sys.exit(1)
