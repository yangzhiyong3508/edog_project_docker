#!/usr/bin/env python3
import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
IOT_SRC = ROOT / "utils/src/iot.c"
IOT_CONTROL_SRC = ROOT / "utils/src/iot_control.c"


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
    iot = IOT_SRC.read_text(encoding="utf-8")
    control = IOT_CONTROL_SRC.read_text(encoding="utf-8")

    init_body = function_body(iot, r"int\s+mqtt_init\s*\([^)]*\)")
    require(init_body, "\"$oc/devices/%s/sys/commands/#\"", "mqtt_init")
    require(init_body, "MQTTSubscribe(&client, subcribe_topic, 0, mqtt_message_arrived)", "mqtt_init")
    require(init_body, "\"$oc/devices/%s/sys/properties/set/#\"", "mqtt_init")
    require(init_body, "MQTTSubscribe(&client, property_set_topic, 0, mqtt_property_set_arrived)", "mqtt_init")
    require(init_body, "MQTTSubscribe(&client, EDOG_SUB_TOPIC, 0, mqtt_edog_message_arrived)", "mqtt_init")

    edog_body = function_body(iot, r"void\s+mqtt_edog_message_arrived\s*\([^)]*\)")
    require(edog_body, "[MQTT] down edog topic=%.*s len=%d qos=%d", "mqtt_edog_message_arrived")
    require(edog_body, "HandleIncomingCommandText(payloadText)", "mqtt_edog_message_arrived")

    system_body = function_body(iot, r"void\s+mqtt_message_arrived\s*\([^)]*\)")
    require(system_body, "[MQTT] down system topic=%s len=%d qos=%d", "mqtt_message_arrived")

    property_body = function_body(iot, r"void\s+mqtt_property_set_arrived\s*\([^)]*\)")
    require(property_body, "[MQTT] down property topic=%s len=%d qos=%d", "mqtt_property_set_arrived")
    require(property_body, "HandleIncomingCommandObject(root)", "mqtt_property_set_arrived")
    require(property_body, "PublishPropertySetResponse(request_id, resultCode, resultText)", "mqtt_property_set_arrived")
    require(iot, "$oc/devices/%s/sys/properties/set/response", "iot.c")

    enqueue_body = function_body(control, r"bool\s+IotControl_EnqueueCommand\s*\([^)]*\)")
    require(enqueue_body, "[Motion] command queued type=%d text=%s", "IotControl_EnqueueCommand")

    print("MQTT QoS0 command downlink diagnostics checks passed")


if __name__ == "__main__":
    try:
        main()
    except AssertionError as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        sys.exit(1)
