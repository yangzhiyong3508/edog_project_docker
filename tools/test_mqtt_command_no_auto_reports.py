#!/usr/bin/env python3
import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
IOT_CONTROL_SRC = ROOT / "utils/src/iot_control.c"


def function_body(text, name):
    match = re.search(rf"bool\s+{name}\s*\([^)]*\)\s*\{{", text)
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


def main():
    text = IOT_CONTROL_SRC.read_text(encoding="utf-8")
    auto_command_handlers = [
        "IotControl_SetSingleServoAngle",
        "IotControl_SetServoBatchAngles",
        "IotControl_SetServoCenterTrim",
        "IotControl_SetAllServoCenterTrims",
        "IotControl_StraightenLegs",
    ]
    forbidden = [
        "IotControl_ReportServoStatus(",
        "IotControl_ReportServoCalibration(",
    ]
    for name in auto_command_handlers:
        body = function_body(text, name)
        for token in forbidden:
            if token in body:
                raise AssertionError(
                    f"{name} must not auto-publish MQTT reports via {token}; "
                    "reports can race MQTTYield and trigger reconnects"
                )

    for name in ["IotControl_ReportServoStatus", "IotControl_ReportServoCalibration"]:
        if f"bool {name}" not in text:
            raise AssertionError(f"explicit report function {name} must remain available")

    print("MQTT command auto-report suppression checks passed")


if __name__ == "__main__":
    try:
        main()
    except AssertionError as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        sys.exit(1)
