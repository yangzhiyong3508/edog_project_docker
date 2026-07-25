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


def require_body(name, text, expected):
    body = body_of(name, text)
    if expected not in body:
        raise AssertionError(f"{name} must call {expected}")
    if "spotMicroCrawlCycleInternal" in body:
        raise AssertionError(f"{name} must not call SpotMicro crawl")


def require_trot_body(name, text, expected):
    body = body_of(name, text)
    require_body(name, text, expected)
    if "runServoGaitTableCycle" in body:
        raise AssertionError(f"{name} must not use servo table scheduler")


def require_turn_body(name, text, expected):
    body = body_of(name, text)
    if expected not in body:
        raise AssertionError(f"{name} must call {expected}")
    if "trotTurnCycleInternal" in body:
        raise AssertionError(f"{name} must not call legacy forward/reverse turn mixer")
    if "spotMicroCrawlCycleInternal" in body:
        raise AssertionError(f"{name} must not call SpotMicro crawl")


def main():
    motion = MOTION.read_text(encoding="utf-8")

    if "Edog12DofContinuousTrotRuntime" not in motion:
        raise AssertionError("whole-body trot must use continuous per-frame runtime")
    if "g_continuousTrotRuntime.frameIndex" not in motion:
        raise AssertionError("whole-body trot must preserve frame index across task loop calls")

    require_trot_body("trot_cycle", motion, "trotCycleInternal(step_length, step_height, 0, 100, 100)")
    require_trot_body("trot_back_cycle", motion, "trotCycleInternal(step_length, step_height, 1, 100, 100)")
    require_turn_body(
        "diversion_left_cycle",
        motion,
        "runServoGaitTableCycle(EDOG_TABLE_MODE_TURN_LEFT, step_length, step_height, 0x0F)",
    )
    require_turn_body(
        "diversion_right_cycle",
        motion,
        "runServoGaitTableCycle(EDOG_TABLE_MODE_TURN_RIGHT, step_length, step_height, 0x0F)",
    )

    in_place = body_of("trot_in_place_cycle", motion)
    if "trotCycleInternal(0.0, step_height, 0, 100, 100)" not in in_place:
        raise AssertionError("trot_in_place_cycle must use continuous trot in-place path")
    if "spotMicroCrawlCycleInternal" in in_place:
        raise AssertionError("trot_in_place_cycle must not call SpotMicro crawl")

    crawl = body_of("crawl_cycle", motion)
    if "spotMicroCrawlCycleInternal(step_length, 0.0, 0.0, step_height)" not in crawl:
        raise AssertionError("crawl_cycle must remain explicit crawl backup")

    if "apply8DofTableFrameStaggered(*gaitSet, frameIndex, EDOG_12DOF_TROT_FRAME_COUNT / 2)" not in motion:
        raise AssertionError("whole-body trot must keep diagonal half-cycle staggering")
    if "configureModePhaseOffsets(&g_servoTableRuntime, mode);" not in motion:
        raise AssertionError("turn trot must use yaw table phase offsets")

    print("12DOF trot public entry contract checks passed")


if __name__ == "__main__":
    try:
        main()
    except AssertionError as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        sys.exit(1)
