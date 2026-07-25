#!/usr/bin/env python3
import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CONFIG_SRC = ROOT / "include/edog_config.h"
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


def require(text, token, label):
    if token not in text:
        raise AssertionError(f"{label} missing token: {token}")


def main():
    config = CONFIG_SRC.read_text(encoding="utf-8")
    mqtt_client = MQTT_CLIENT_SRC.read_text(encoding="utf-8")
    liteos = MQTT_LITEOS_SRC.read_text(encoding="utf-8")

    if not re.search(r"#define\s+EDOG_WIFI_MQTT_YIELD_MS\s+100\b", config):
        raise AssertionError("MQTT yield polling must be 100ms so commands are received promptly")

    yield_body = function_body(mqtt_client, r"int\s+MQTTYield\s*\([^)]*\)")
    require(yield_body, "MqttMutexLock(&c->mutex);", "MQTTYield")
    require(yield_body, "MqttMutexUnlock(&c->mutex);", "MQTTYield")

    publish_body = function_body(mqtt_client, r"int\s+MQTTPublish\s*\([^)]*\)")
    require(publish_body, "MqttMutexLock(&c->mutex);", "MQTTPublish")
    require(publish_body, "MqttMutexUnlock(&c->mutex);", "MQTTPublish")

    mutex_init_body = function_body(liteos, r"void\s+MqttMutexInit\s*\([^)]*\)")
    require(mutex_init_body, "osMutexAttr_t attr", "MqttMutexInit")
    require(mutex_init_body, "osMutexRecursive", "MqttMutexInit")
    require(mutex_init_body, "osMutexNew(&attr)", "MqttMutexInit")

    print("MQTT yield/publish mutex checks passed")


if __name__ == "__main__":
    try:
        main()
    except AssertionError as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        sys.exit(1)
