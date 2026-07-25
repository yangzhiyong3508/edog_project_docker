#!/usr/bin/env python3
import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CONTROL = ROOT / "utils/src/iot_control.c"
MOTION = ROOT / "12_DOF_Version/src/motion_utils_12dof.c"


def body_of_public_function(name, text):
    match = re.search(rf"int\s+{name}\s*\([^)]*\)\s*\{{(?P<body>.*?)\n\}}", text, re.S)
    if not match:
        raise AssertionError(f"missing function {name}")
    return match.group("body")


def require(token, text, label):
    if token not in text:
        raise AssertionError(f"{label} missing {token}")


def require_no_active_arc(motion):
    for name in [
        "trot_left_front_cycle",
        "trot_right_front_cycle",
        "trot_left_back_cycle",
        "trot_right_back_cycle",
    ]:
        body = body_of_public_function(name, motion)
        require("return 0;", body, name)
        require("not supported", body, name)
        if "trotArcCycleInternal" in body or "runServoGaitTableCycle" in body:
            raise AssertionError(f"{name} must not plan active arc gait")

    if "trotArcCycleInternal" in motion:
        raise AssertionError("active arc gait planner must be removed")
    if "EDOG_TABLE_MODE_ARC" in motion:
        raise AssertionError("active arc gait table modes must be removed")


def main():
    control = CONTROL.read_text(encoding="utf-8")
    motion = MOTION.read_text(encoding="utf-8")

    require("Edog12DofContinuousTrotRuntime", motion, "continuous whole-body trot runtime")
    require("g_continuousTrotRuntime.frameIndex", motion, "continuous whole-body trot frame index")

    for pattern in [
        r"case\s+MOTION_CMD_TROT:\s*result\s*=\s*trot_cycle\s*\(\s*stepLengthM\s*,\s*stepHeightM\s*\)\s*;",
        r"case\s+MOTION_CMD_TROT_BACK:\s*result\s*=\s*trot_back_cycle\s*\(\s*stepLengthM\s*,\s*stepHeightM\s*\)\s*;",
        r"case\s+MOTION_CMD_TURN_LEFT:\s*result\s*=\s*diversion_left_cycle\s*\(\s*stepLengthM\s*,\s*stepHeightM\s*\)\s*;",
        r"case\s+MOTION_CMD_TURN_RIGHT:\s*result\s*=\s*diversion_right_cycle\s*\(\s*stepLengthM\s*,\s*stepHeightM\s*\)\s*;",
    ]:
        if not re.search(pattern, control, re.S):
            raise AssertionError(f"motion command dispatch missing pattern {pattern}")

    for name, expected in {
        "trot_cycle": "trotCycleInternal(step_length, step_height, 0, 100, 100)",
        "trot_back_cycle": "trotCycleInternal(step_length, step_height, 1, 100, 100)",
    }.items():
        body = body_of_public_function(name, motion)
        require(expected, body, name)
        if "spotMicroCrawlCycleInternal" in body:
            raise AssertionError(f"{name} must not use SpotMicro crawl")
        if "runServoGaitTableCycle" in body:
            raise AssertionError(f"{name} must not use servo gait table scheduler")

    for name, expected in {
        "diversion_left_cycle": "runServoGaitTableCycle(EDOG_TABLE_MODE_TURN_LEFT, step_length, step_height, 0x0F)",
        "diversion_right_cycle": "runServoGaitTableCycle(EDOG_TABLE_MODE_TURN_RIGHT, step_length, step_height, 0x0F)",
    }.items():
        body = body_of_public_function(name, motion)
        require(expected, body, name)
        if "spotMicroCrawlCycleInternal" in body:
            raise AssertionError(f"{name} must not use SpotMicro crawl")
        if "trotTurnCycleInternal" in body:
            raise AssertionError(f"{name} must not use legacy forward/reverse turn mixer")

    in_place = body_of_public_function("trot_in_place_cycle", motion)
    if "spotMicroCrawlCycleInternal" in in_place:
        raise AssertionError("trot_in_place_cycle must not use SpotMicro crawl")
    require("trotCycleInternal(0.0, step_height, 0, 100, 100)", in_place, "trot_in_place_cycle")

    crawl_body = body_of_public_function("crawl_cycle", motion)
    require("spotMicroCrawlCycleInternal(step_length, 0.0, 0.0, step_height)", crawl_body, "crawl_cycle")

    require_no_active_arc(motion)

    single_leg = body_of_public_function("single_leg_gait_cycle", motion)
    leg_group = body_of_public_function("leg_group_gait_cycle", motion)
    require("return leg_group_gait_cycle(1 << leg_index, step_length, step_height);", single_leg, "single_leg_gait_cycle")
    require("runServoGaitTableCycle(EDOG_TABLE_MODE_FORWARD, step_length, step_height, leg_mask)", leg_group, "leg_group_gait_cycle")

    for token in [
        "typedef struct {\n    int centiDeg[EDOG_SERVO_CHANNEL_COUNT];",
        "static int runServoGaitTableCycle",
        "static void buildServoGaitTable",
        "static int computeServoTableMinFrameMs",
        "static int servoTableCentiToTrimmedTarget",
        "static int spotMicroCrawlCycleInternal",
        "static const unsigned char g_staticCrawlContactPhases[EDOG_12DOF_LEG_COUNT][EDOG_STATIC_CRAWL_NUM_PHASES]",
        "staticCrawlNextSwingLeg",
    ]:
        require(token, motion, "motion_utils_12dof.c")

    print("12DOF public whole-body motion entries use diagonal trot; explicit crawl backup remains")


if __name__ == "__main__":
    try:
        main()
    except AssertionError as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        sys.exit(1)
