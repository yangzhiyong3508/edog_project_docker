#!/usr/bin/env python3
import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "include/edog_config.h"
MOTION = ROOT / "12_DOF_Version/src/motion_utils_12dof.c"
CONTROL = ROOT / "utils/src/iot_control.c"


def body(name, text):
    match = re.search(rf"(?:static\s+)?(?:int|void)\s+{name}\s*\([^)]*\)\s*\{{(?P<body>.*?)\n\}}", text, re.S)
    if not match:
        raise AssertionError(f"missing function {name}")
    return match.group("body")


def require(token, text, label):
    if token not in text:
        raise AssertionError(f"{label} missing token: {token}")


def main():
    config = CONFIG.read_text(encoding="utf-8")
    motion = MOTION.read_text(encoding="utf-8")
    control = CONTROL.read_text(encoding="utf-8")

    for token in [
        "#define EDOG_SERVO_HIP_SPEED_60_DEG_MS        330",
        "#define EDOG_SERVO_LEG_SPEED_60_DEG_MS        250",
        "#define EDOG_SERVO_SPEED_SAFETY_NUM            8",
        "#define EDOG_SERVO_SPEED_SAFETY_DEN            10",
    ]:
        require(token, config, "config")

    straight = body("setDogLegsStraightPose", motion)
    require("90deg + trim", straight, "setDogLegsStraightPose")
    require("EDOG_SERVO_CENTER_ANGLE * EDOG_12DOF_CENTI_PER_DEG +", straight, "setDogLegsStraightPose")
    require("g_servoCenterTrim[channel] * EDOG_12DOF_CENTI_PER_DEG", straight, "setDogLegsStraightPose")
    require("motionSetServoPhysicalNoTrimCenti(channel, calibratedCenterCenti)", straight, "setDogLegsStraightPose")

    stop = body("smooth_stop_to_stand", motion)
    require("buildStandPoseCenti(target, step_length, step_height)", stop, "smooth_stop_to_stand")
    require("servoTableCentiToTrimmedTarget(channel, target[channel])", stop, "smooth_stop_to_stand")
    require("approachServoTargetCenti(current, finalTarget, maxStep)", stop, "smooth_stop_to_stand")
    require("return allDone", stop, "smooth_stop_to_stand")
    if "for (int step = 1; step <=" in stop:
        raise AssertionError("smooth_stop_to_stand must not run a blocking fixed interpolation loop")

    task = body("IotControl_MotionTask", control)
    require("result > 0 && FinishOneMotionCycle()", task, "IotControl_MotionTask")
    require("result == 0 && isStopFlag()", task, "IotControl_MotionTask")

    print("12DOF table stop, straight-leg calibration, and repeat semantics checks passed")


if __name__ == "__main__":
    try:
        main()
    except AssertionError as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        sys.exit(1)
