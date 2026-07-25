#!/usr/bin/env python3
import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MOTION_TASK = ROOT / "utils/src/iot_control.c"


def function_body(source, signature):
    start = source.find(signature)
    if start < 0:
        raise AssertionError(f"missing function: {signature}")
    while start >= 0:
        next_semicolon = source.find(";", start)
        next_brace = source.find("{", start)
        if next_brace >= 0 and (next_semicolon < 0 or next_brace < next_semicolon):
            break
        start = source.find(signature, start + len(signature))
    if start < 0:
        raise AssertionError(f"missing function body: {signature}")
    brace_start = source.find("{", start)
    if brace_start < 0:
        raise AssertionError(f"missing function body: {signature}")

    depth = 0
    for index in range(brace_start, len(source)):
        char = source[index]
        if char == "{":
            depth += 1
        elif char == "}":
            depth -= 1
            if depth == 0:
                return source[brace_start:index + 1]
    raise AssertionError(f"unterminated function body: {signature}")


def assert_ordered(body, first, second, message):
    first_pos = body.find(first)
    second_pos = body.find(second)
    if first_pos < 0 or second_pos < 0 or first_pos > second_pos:
        raise AssertionError(message)


def main():
    task = MOTION_TASK.read_text(encoding="utf-8")
    active_task = task.split("void IotControl_MotionTaskLegacyUnused", 1)[0]

    if "StopMotionForManualServoControl" in active_task:
        raise AssertionError("manual servo commands must not use the Stop helper")

    if "static void PrepareManualServoControl(void)" not in active_task:
        raise AssertionError("missing manual servo hold preparation helper")

    prepare_body = function_body(active_task, "static void PrepareManualServoControl(void)")
    if "stopCurrentMotion();" in prepare_body:
        raise AssertionError("manual servo preparation must not raise the Stop flag")
    if "g_stopReturnPending = 1" in prepare_body or "SetMotionState(MOTION_CMD_NONE, -1, 1)" in prepare_body:
        raise AssertionError("manual servo preparation must not request stop-to-stand")

    enqueue_body = function_body(active_task, "bool IotControl_EnqueueCommand(const IotControlCommand *command)")
    manual_enqueue = re.search(
        r"else\s+if\s*\([^{}]*IOT_CONTROL_COMMAND_SERVO_SET[^{}]*\)\s*\{(?P<body>.*?)\}",
        enqueue_body,
        re.S,
    )
    if manual_enqueue and "stopCurrentMotion();" in manual_enqueue.group("body"):
        raise AssertionError("manual servo commands must not trigger stopCurrentMotion when queued")

    stop_body = function_body(active_task, "static void ExecuteMotionWithRuntimeParams(MotionCommand cmd, int repeatCount")
    stop_branch = re.search(r"if\s*\(cmd == MOTION_CMD_NONE\)\s*\{(?P<body>.*?)\n\s*\}", stop_body, re.S)
    if not stop_branch:
        raise AssertionError("missing explicit stop command branch")
    if "ClearManualPoseHold();" not in stop_branch.group("body") or "stopCurrentMotion();" not in stop_branch.group("body"):
        raise AssertionError("only the explicit stop command should clear manual hold and raise the Stop flag")

    single_body = function_body(task, "bool IotControl_SetSingleServoAngle(int channel, int angle)")
    if "PrepareManualServoControl();" not in single_body:
        raise AssertionError("single-servo command must enter manual mode")
    assert_ordered(
        single_body,
        "setDogServoAngleTracked(channel, angle)",
        "EnableManualPoseHold();",
        "single-servo command must hold the applied angle instead of falling back to stand",
    )

    batch_body = function_body(task, "bool IotControl_SetServoBatchAngles(int count, const int channels[], const int angles[])")
    if "PrepareManualServoControl();" not in batch_body:
        raise AssertionError("batch-servo command must enter manual mode")
    assert_ordered(
        batch_body,
        "setDogServoAngleTracked(channels[i], angles[i])",
        "EnableManualPoseHold();",
        "batch-servo command must hold the applied angles instead of falling back to stand",
    )

    trim_body = function_body(task, "bool IotControl_SetServoCenterTrim(int channel, int trim, int applyNow)")
    if "PrepareManualServoControl();" not in trim_body:
        raise AssertionError("single trim command must enter manual mode")
    if "EnableManualPoseHold();" not in trim_body:
        raise AssertionError("single trim command must keep the adjusted pose until Stop")

    straight_body = function_body(task, "bool IotControl_StraightenLegs(void)")
    if "PrepareManualServoControl();" not in straight_body:
        raise AssertionError("straight-leg command must enter manual mode")
    assert_ordered(
        straight_body,
        "setDogLegsStraightPose()",
        "EnableManualPoseHold();",
        "straight-leg command must keep the straight pose until Stop",
    )

    print("manual servo hold mode checks passed")


if __name__ == "__main__":
    try:
        main()
    except AssertionError as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        sys.exit(1)
