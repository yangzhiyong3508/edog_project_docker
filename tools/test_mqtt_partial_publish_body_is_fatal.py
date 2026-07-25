#!/usr/bin/env python3
import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MQTT_CLIENT_REL = Path("third_party/paho_mqtt/MQTTClient-C/src/MQTTClient.c")


def find_mqtt_client_source():
    for base in (ROOT, *ROOT.parents):
        candidate = base / MQTT_CLIENT_REL
        if candidate.exists():
            return candidate
    raise AssertionError(f"missing source file {MQTT_CLIENT_REL}")


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


def require(token, text, label):
    if token not in text:
        raise AssertionError(f"{label} missing token: {token}")


def main():
    mqtt_client_path = find_mqtt_client_source()
    mqtt_client = mqtt_client_path.read_text(encoding="utf-8")
    read_packet = function_body(mqtt_client, r"static\s+int\s+readPacket\s*\([^)]*\)")

    require("static int MqttReadTimeoutMs(MQTTClient* c, Timer* timer)", mqtt_client, "MQTTClient.c")
    require("int decoded_len = decodePacket(c, &rem_len, MqttReadTimeoutMs(c, timer));", read_packet, "readPacket")
    require("readPacket remaining length read failed rc=%d left_ms=%d", read_packet, "readPacket")
    require("readPacket body read failed rc=%d rem_len=%d left_ms=%d", read_packet, "readPacket")
    require("rc = MQTTPACKET_READ_ERROR;", read_packet, "readPacket")
    require("goto exit;", read_packet, "readPacket")

    liteos_path = mqtt_client_path.parent / "liteOS" / "MQTTLiteOS.c"
    liteos = liteos_path.read_text(encoding="utf-8")
    linux_read = function_body(liteos, r"int\s+linux_read\s*\([^)]*\)")
    require("total_timeout_ms", linux_read, "linux_read")
    require("select_timeout_ms", linux_read, "linux_read")
    require("bytes += rc;", linux_read, "linux_read")
    require("continue;", linux_read, "linux_read")
    require("while (bytes < len)", linux_read, "linux_read")

    if "decodePacket(c, &rem_len, TimerLeftMS(timer))" in read_packet:
        raise AssertionError("MQTT remaining length read must not use an exhausted yield timer")
    if "decodePacket(c, &rem_len, TimerLeftMS(timer));" in read_packet:
        raise AssertionError("MQTT remaining length read result must be checked")
    if "return rc;" in linux_read:
        raise AssertionError("linux_read must accumulate short recv results instead of returning a partial body")

    print("MQTT partial publish body short-read recovery checks passed")


if __name__ == "__main__":
    try:
        main()
    except AssertionError as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        sys.exit(1)
