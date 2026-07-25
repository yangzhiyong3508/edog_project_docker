#!/usr/bin/env python3
import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MOTION = ROOT / "12_DOF_Version/src/motion_utils_12dof.c"


def body_of(name, text):
    match = re.search(rf"int\s+{name}\s*\([^)]*\)\s*\{{(?P<body>.*?)\n\}}", text, re.S)
    if not match:
        raise AssertionError(f"missing function {name}")
    return match.group("body")


def require_trot_entry(name, body, expected):
    if expected not in body:
        raise AssertionError(f"{name} must use {expected}")
    if "spotMicroCrawlCycleInternal" in body:
        raise AssertionError(f"{name} must not use SpotMicro crawl path")
    if "runServoGaitTableCycle" in body:
        raise AssertionError(f"{name} must not use servo gait table scheduler")


def require_turn_entry(name, body, expected):
    if expected not in body:
        raise AssertionError(f"{name} must use {expected}")
    if "spotMicroCrawlCycleInternal" in body:
        raise AssertionError(f"{name} must not use SpotMicro crawl path")
    if "trotTurnCycleInternal" in body:
        raise AssertionError(f"{name} must not use legacy forward/reverse turn mixer")


def main():
    motion = MOTION.read_text(encoding="utf-8")

    require_trot_entry(
        "trot_cycle",
        body_of("trot_cycle", motion),
        "trotCycleInternal(step_length, step_height, 0, 100, 100)",
    )
    require_trot_entry(
        "trot_back_cycle",
        body_of("trot_back_cycle", motion),
        "trotCycleInternal(step_length, step_height, 1, 100, 100)",
    )
    require_turn_entry(
        "diversion_right_cycle",
        body_of("diversion_right_cycle", motion),
        "runServoGaitTableCycle(EDOG_TABLE_MODE_TURN_RIGHT, step_length, step_height, 0x0F)",
    )
    require_turn_entry(
        "diversion_left_cycle",
        body_of("diversion_left_cycle", motion),
        "runServoGaitTableCycle(EDOG_TABLE_MODE_TURN_LEFT, step_length, step_height, 0x0F)",
    )

    in_place = body_of("trot_in_place_cycle", motion)
    if "spotMicroCrawlCycleInternal" in in_place:
        raise AssertionError("trot_in_place_cycle must not use SpotMicro crawl path")
    if "trotCycleInternal(0.0, step_height, 0, 100, 100)" not in in_place:
        raise AssertionError("trot_in_place_cycle must keep the continuous trot in-place path")

    crawl = body_of("crawl_cycle", motion)
    if "spotMicroCrawlCycleInternal(step_length, 0.0, 0.0, step_height)" not in crawl:
        raise AssertionError("crawl_cycle must remain the explicit SpotMicro crawl path")

    for name in [
        "trot_left_front_cycle",
        "trot_right_front_cycle",
        "trot_left_back_cycle",
        "trot_right_back_cycle",
    ]:
        body = body_of(name, motion)
        if "return 0;" not in body or "not supported" not in body:
            raise AssertionError(f"{name} must reject unsupported arc gait")
        if "trotArcCycleInternal" in body or "runServoGaitTableCycle" in body:
            raise AssertionError(f"{name} must not plan active arc gait")

    if "trotArcCycleInternal" in motion:
        raise AssertionError("active arc gait planner must be removed")
    if "EDOG_TABLE_MODE_ARC" in motion:
        raise AssertionError("active arc gait table modes must be removed")

    single_leg = body_of("single_leg_gait_cycle", motion)
    leg_group = body_of("leg_group_gait_cycle", motion)
    if "return leg_group_gait_cycle(1 << leg_index, step_length, step_height);" not in single_leg:
        raise AssertionError("single_leg_gait_cycle must continue to delegate through leg_group_gait_cycle")
    if "runServoGaitTableCycle(EDOG_TABLE_MODE_FORWARD, step_length, step_height, leg_mask)" not in leg_group:
        raise AssertionError("leg_group_gait_cycle must keep the table scheduler for leg-mask debug output")

    for token in [
        "static int applyServoCenterTrimCenti",
        "g_servoCenterTrim[channel] * EDOG_12DOF_CENTI_PER_DEG",
        "motionSetServoPhysicalSmoothCenti",
        "int target = applyServoCenterTrimCenti(channel, clampServoAngleCenti(centiAngle));",
        "return setLegAnglesStaggeredSmooth(&g_legs[legIndex], &outputAngles, delayUs);",
        "int saveDogServoCenterTrim(int channel, int trimDeg)",
        "int loadDogServoCenterTrims(void)",
        "motionSetServoPhysicalNoTrimCenti",
        "90deg + trim",
    ]:
        if token not in motion:
            raise AssertionError(f"servo calibration trim path missing token: {token}")

    for forbidden_define in [
        "#define EDOG_12DOF_CRAWL_PRE_SHIFT_HIP_BIAS_DEG",
        "#define EDOG_12DOF_CRAWL_BALANCE_HIP_BIAS_DEG",
        "#define EDOG_12DOF_DEFAULT_FOOT_X_MM",
        "#define EDOG_12DOF_REALTIME_MAX_FORWARD_MM",
    ]:
        if forbidden_define in motion:
            raise AssertionError(f"motion_utils_12dof.c must use edog_config.h, not local override: {forbidden_define}")

    for token in [
        "static int spotMicroCrawlCycleInternal",
        "staticCrawlNextSwingLeg",
        "static double pyAppleCrawlCycloidProgress",
        "static double pyAppleCrawlCycloidLift",
        "pyAppleCrawlCycloidProgress(t)",
        "pyAppleCrawlCycloidLift(swingT)",
        "applyHipWeightShiftBias(frame, swingState, preShiftLeg",
    ]:
        if token not in motion:
            raise AssertionError(f"explicit crawl backup support missing token: {token}")
    for token in [
        "spotMicroCrawlSwingLift",
        "spotMicroCrawlBezier",
    ]:
        if token in motion:
            raise AssertionError(f"explicit crawl backup must not use old SpotMicro swing helper: {token}")

    print("whole-body motion entries use trot; leg debug tables and trim safeguards remain")


if __name__ == "__main__":
    try:
        main()
    except AssertionError as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        sys.exit(1)
