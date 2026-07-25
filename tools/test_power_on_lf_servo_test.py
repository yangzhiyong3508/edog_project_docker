#!/usr/bin/env python3
import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "include/edog_config.h"
MAIN = ROOT / "src/main.c"


def require(token, text, where):
    if token not in text:
        raise AssertionError(f"{where} missing {token}")


def require_define(name, value, text):
    if not re.search(rf"#define\s+{name}\s+{value}\b", text):
        raise AssertionError(f"edog_config.h missing #define {name} {value}")


def main():
    config = CONFIG.read_text(encoding="utf-8")
    main_c = MAIN.read_text(encoding="utf-8")

    require_define("EDOG_POWER_ON_LF_SERVO_TEST_ENABLED", 0, config)
    require_define("EDOG_POWER_ON_LF_SERVO_TEST_STEP_MS", 20, config)
    require_define("EDOG_POWER_ON_LF_SERVO_TEST_MIN_PULSE_US", 1351, config)
    require_define("EDOG_POWER_ON_LF_SERVO_TEST_MAX_PULSE_US", 1648, config)
    require_define("EDOG_POWER_ON_LF_SERVO_TEST_FRAME_COUNT", 96, config)

    for token in [
        "static int LeftFrontServoTestSmoothPulseUs(int phase)",
        "static void LeftFrontServoTestTask(void)",
        "EDOG_POWER_ON_LF_SERVO_TEST_FIXED_SCALE",
        "smooth = (t * t * (3 * scale - 2 * t) + scale * scale / 2) /",
        "pulseRange * smooth + scale / 2",
        "pulseUs = LeftFrontServoTestSmoothPulseUs(phase)",
        "setServoPulseUs(LF_HIP, hipPulseUs)",
        "setServoPulseUs(LF_THIGH, thighPulseUs)",
        "setServoPulseUs(LF_CALF, calfPulseUs)",
        "LOS_Msleep(EDOG_POWER_ON_LF_SERVO_TEST_STEP_MS)",
        "#if EDOG_POWER_ON_LF_SERVO_TEST_ENABLED",
        "CreateTask(LeftFrontServoTestTask, \"lf_servo_test_task\"",
        "return;",
    ]:
        require(token, main_c, "main.c")

    task_start = main_c.find("static void LeftFrontServoTestTask(void)")
    app_start = main_c.find("static void AppInit(void)")
    if task_start < 0 or app_start < 0 or task_start >= app_start:
        raise AssertionError("main.c must define LeftFrontServoTestTask before AppInit")
    task_body = main_c[task_start:app_start]
    if "setServo(" in task_body:
        raise AssertionError("LF servo test must bypass angle mapping and call setServoPulseUs directly")
    if "pulseUs += direction * EDOG_POWER_ON_LF_SERVO_TEST_PULSE_STEP_US" in task_body:
        raise AssertionError("LF servo test must use eased phase motion instead of linear pulse steps")
    if "long smooth;" in task_body or "/ 1024;" in task_body:
        raise AssertionError("LF servo test must avoid truncating the easing curve to coarse integer steps")

    app_start = main_c.find("static void AppInit(void)")
    branch_start = main_c.find("#if EDOG_POWER_ON_LF_SERVO_TEST_ENABLED", app_start)
    branch_return = main_c.find("return;", branch_start)
    normal_start = main_c.find("CreateTask(ServoInitTask", app_start)
    if app_start < 0 or branch_start < 0 or branch_return < 0 or normal_start < 0:
        raise AssertionError("main.c AppInit must contain test branch and normal startup path")
    if not (app_start < branch_start < branch_return < normal_start):
        raise AssertionError("power-on LF servo test branch must return before normal startup tasks")

    print("power-on normal startup checks passed")


if __name__ == "__main__":
    try:
        main()
    except AssertionError as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        sys.exit(1)
