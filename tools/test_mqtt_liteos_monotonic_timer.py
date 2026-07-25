#!/usr/bin/env python3
import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MQTT_LITEOS_SRC = ROOT.parents[4] / "third_party/paho_mqtt/MQTTClient-C/src/liteOS/MQTTLiteOS.c"
MQTT_LITEOS_HDR = ROOT.parents[4] / "third_party/paho_mqtt/MQTTClient-C/src/liteOS/MQTTLiteOS.h"


def function_body(text, name):
    match = re.search(rf"(?:char|void|int)\s+{name}\s*\([^)]*\)\s*\{{", text)
    if not match:
        raise AssertionError(f"missing function {name}")
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
        raise AssertionError(f"unterminated function {name}")
    return text[start:pos - 1]


def require(text, token):
    if token not in text:
        raise AssertionError(f"missing token: {token}")


def forbid(text, token):
    if token in text:
        raise AssertionError(f"forbidden token present: {token}")


def main():
    src = MQTT_LITEOS_SRC.read_text(encoding="utf-8")
    hdr = MQTT_LITEOS_HDR.read_text(encoding="utf-8")
    timer_bodies = "\n".join(
        function_body(src, name)
        for name in ("TimerInit", "TimerIsExpired", "TimerCountdownMS", "TimerCountdown", "TimerLeftMS")
    )

    require(hdr, "#include <stdint.h>")
    require(hdr, "uint32_t end_tick;")
    require(src, "osKernelGetTickCount()")
    require(src, "osKernelGetTickFreq()")
    require(src, "MqttMsToTicks")
    require(function_body(src, "TimerIsExpired"), "(int32_t)(osKernelGetTickCount() - timer->end_tick) >= 0")

    for token in ("gettimeofday", "timeradd", "timersub", "struct timeval end_time"):
        forbid(timer_bodies + "\n" + hdr, token)

    print("MQTT LiteOS monotonic timer checks passed")


if __name__ == "__main__":
    try:
        main()
    except AssertionError as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        sys.exit(1)
