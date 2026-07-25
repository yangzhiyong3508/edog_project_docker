#!/usr/bin/env python3
import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MOTION = ROOT / "12_DOF_Version/src/motion_utils_12dof.c"


def body_of(name, text):
    match = re.search(
        rf"(?:static\s+)?(?:int|void)\s+{name}\s*\([^)]*\)\s*\{{(?P<body>.*?)\n\}}",
        text,
        re.S,
    )
    if not match:
        raise AssertionError(f"missing public function {name}")
    return match.group("body")


def require_turn_entry(name, body, mode):
    expected = f"runServoGaitTableCycle({mode}, step_length, step_height, 0x0F)"
    if expected not in body:
        raise AssertionError(f"{name} must route to yaw turn table: {expected}")
    if "trotTurnCycleInternal" in body:
        raise AssertionError(f"{name} must not use legacy forward/reverse turn mixer")
    if "spotMicroCrawlCycleInternal" in body:
        raise AssertionError(f"{name} must not use crawl kernel for default turn")


def switch_case_body(switch_body, case_name):
    match = re.search(
        rf"case\s+{case_name}\s*:\s*(?P<body>.*?)(?:\n\s*case\s+|\n\s*default\s*:)",
        switch_body,
        re.S,
    )
    if not match:
        raise AssertionError(f"missing switch case {case_name}")
    return match.group("body")


def main():
    motion = MOTION.read_text(encoding="utf-8")
    build_table = body_of("buildServoGaitTable", motion)
    turn_left = switch_case_body(build_table, "EDOG_TABLE_MODE_TURN_LEFT")
    turn_right = switch_case_body(build_table, "EDOG_TABLE_MODE_TURN_RIGHT")

    require_turn_entry(
        "diversion_left_cycle",
        body_of("diversion_left_cycle", motion),
        "EDOG_TABLE_MODE_TURN_LEFT",
    )
    require_turn_entry(
        "diversion_right_cycle",
        body_of("diversion_right_cycle", motion),
        "EDOG_TABLE_MODE_TURN_RIGHT",
    )

    if "forwardStep = stepLength * 0.25;" not in turn_left:
        raise AssertionError("left turn must keep a small forward component")
    if "yawStep = -stepLength * 0.75;" not in turn_left:
        raise AssertionError("left turn yaw must be negative after direction correction")
    if "yawStep = stepLength * 0.75;" in turn_left:
        raise AssertionError("left turn must not use the old positive yaw")

    if "forwardStep = stepLength * 0.25;" not in turn_right:
        raise AssertionError("right turn must keep a small forward component")
    if "yawStep = stepLength * 0.75;" not in turn_right:
        raise AssertionError("right turn yaw must be positive after direction correction")
    if "yawStep = -stepLength * 0.75;" in turn_right:
        raise AssertionError("right turn must not use the old negative yaw")

    if "configureModePhaseOffsets(&g_servoTableRuntime, mode);" not in motion:
        raise AssertionError("turn table runtime must configure phase offsets")

    print("12DOF left/right turn entries route to yaw turn table")


if __name__ == "__main__":
    try:
        main()
    except AssertionError as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        sys.exit(1)
