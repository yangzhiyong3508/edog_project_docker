#!/usr/bin/env python3
import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MOTION = ROOT / "12_DOF_Version/src/motion_utils_12dof.c"
CONTROL = ROOT / "utils/src/iot_control.c"


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
    motion = MOTION.read_text(encoding="utf-8")
    control = CONTROL.read_text(encoding="utf-8")

    start_runtime = function_body(
        motion,
        r"static\s+void\s+startServoTableRuntime\s*\([^)]*\)",
    )
    run_runtime = function_body(
        motion,
        r"static\s+int\s+runServoGaitTableCycle\s*\([^)]*\)",
    )
    execute_motion = function_body(
        control,
        r"static\s+void\s+ExecuteMotionWithRuntimeParams\s*\([^)]*\)",
    )

    require("tableParamsChanged(&g_servoTableRuntime", run_runtime, "runServoGaitTableCycle")
    require("alreadyRunning = repeatCount == -1 && g_currentMotion == cmd", execute_motion,
            "ExecuteMotionWithRuntimeParams")
    require("if (alreadyRunning) {\n        return;\n    }", execute_motion,
            "ExecuteMotionWithRuntimeParams")

    require("previousActive", start_runtime, "startServoTableRuntime")
    require("previousBaseIndex", start_runtime, "startServoTableRuntime")
    require("previousFrameCount", start_runtime, "startServoTableRuntime")
    require("preservePhase", start_runtime, "startServoTableRuntime")
    require("previousBaseIndex % g_servoGaitTable.frameCount",
            start_runtime, "startServoTableRuntime")

    if re.search(r"g_servoTableRuntime\.baseIndex\s*=\s*0\s*;", start_runtime):
        raise AssertionError("startServoTableRuntime must not reset active gait phase to frame 0")

    print("12DOF gait phase continuity checks passed")


if __name__ == "__main__":
    try:
        main()
    except AssertionError as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        sys.exit(1)
