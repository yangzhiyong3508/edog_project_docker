#!/usr/bin/env python3
import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MOTION_TASK = ROOT / "utils/src/iot_control.c"
MOTION_HEADER = ROOT / "12_DOF_Version/include/motion_utils_12dof.h"


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


def main():
    task = MOTION_TASK.read_text(encoding="utf-8")
    header = MOTION_HEADER.read_text(encoding="utf-8")
    active_task = task.split("void IotControl_MotionTaskLegacyUnused", 1)[0]

    if "int balance_stand_frame(void);" not in header:
        raise AssertionError("motion_utils_12dof.h must export balance_stand_frame")

    if "static void RunIdleStandBalanceFrame(void)" not in active_task:
        raise AssertionError("missing idle stand balance wrapper")
    if "(void)balance_stand_frame();" not in active_task:
        raise AssertionError("idle path must keep running stand balance frames")
    if "RunIdleStandBalanceFrame();" not in active_task:
        raise AssertionError("MOTION_CMD_NONE branch must call idle stand balance wrapper")
    if "g_idleBalancePauseFrames = EDOG_12DOF_IDLE_BALANCE_MANUAL_PAUSE_FRAMES" not in active_task:
        raise AssertionError("manual servo control must pause idle balance briefly")
    if "usleep(EDOG_12DOF_GAIT_FRAME_PERIOD_US)" not in active_task:
        raise AssertionError("manual pause should keep command loop responsive at gait frame period")
    if "usleep(100000)" in active_task:
        raise AssertionError("idle branch must not fall back to 100ms sleep-only loop")

    if "static volatile int g_manualPoseHoldActive = 0;" not in active_task:
        raise AssertionError("straight-leg pose needs a manual hold flag")
    if "static void EnableManualPoseHold(void)" not in active_task:
        raise AssertionError("missing helper to enable manual pose hold")
    if "static void ClearManualPoseHold(void)" not in active_task:
        raise AssertionError("missing helper to clear manual pose hold")

    idle_body = function_body(active_task, "static void RunIdleStandBalanceFrame(void)")
    hold_pos = idle_body.find("g_manualPoseHoldActive")
    balance_pos = idle_body.find("balance_stand_frame")
    if hold_pos < 0 or balance_pos < 0 or hold_pos > balance_pos:
        raise AssertionError("idle balance must check manual pose hold before balance_stand_frame")

    manual_prepare_body = function_body(active_task, "static void PrepareManualServoControl(void)")
    if "ClearManualPoseHold();" not in manual_prepare_body:
        raise AssertionError("manual servo commands should exit any previous pose hold before applying a new manual pose")
    if "stopCurrentMotion();" in manual_prepare_body:
        raise AssertionError("manual servo commands must not raise the Stop flag")
    if "SetMotionState(MOTION_CMD_NONE, -1, 1)" in manual_prepare_body:
        raise AssertionError("manual servo commands must not request stop-to-stand")

    motion_body = function_body(active_task, "static void ExecuteMotionWithRuntimeParams(MotionCommand cmd, int repeatCount")
    if motion_body.count("ClearManualPoseHold();") < 2:
        raise AssertionError("stop and normal motion commands must clear manual pose hold")

    straight_body = function_body(task, "bool IotControl_StraightenLegs(void)")
    pose_pos = straight_body.find("setDogLegsStraightPose()")
    enable_pos = straight_body.find("EnableManualPoseHold();")
    if pose_pos < 0 or enable_pos < 0 or pose_pos > enable_pos:
        raise AssertionError("straight-leg hold should start only after the straight pose is applied")

    leg_group_body = function_body(task, "static bool IotControl_RunLegGaitWithRuntimeParams(int legMask, int repeatCount")
    if "ClearManualPoseHold();" not in leg_group_body:
        raise AssertionError("multi-leg gait commands must clear manual pose hold")

    print("idle stand balance task checks passed")


if __name__ == "__main__":
    try:
        main()
    except AssertionError as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        sys.exit(1)
