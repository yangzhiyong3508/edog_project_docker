#!/usr/bin/env python3
import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "include/edog_config.h"
MOTION = ROOT / "12_DOF_Version/src/motion_utils_12dof.c"
GAIT = ROOT / "12_DOF_Version/src/gait_generate_12dof.c"
CONTROL = ROOT / "utils/src/iot_control.c"
MAIN = ROOT / "src/main.c"


def require(text, token, label):
    if token not in text:
        raise AssertionError(f"{label} missing token: {token}")


def require_pattern(text, pattern, label):
    if not re.search(pattern, text, re.S):
        raise AssertionError(f"{label} missing pattern: {pattern}")


def reject(text, token, label):
    if token in text:
        raise AssertionError(f"{label} still contains old token: {token}")


def main():
    config = CONFIG.read_text(encoding="utf-8")
    motion = MOTION.read_text(encoding="utf-8")
    gait = GAIT.read_text(encoding="utf-8")
    control = CONTROL.read_text(encoding="utf-8")
    main_c = MAIN.read_text(encoding="utf-8")

    require(config, "#define EDOG_12DOF_GAIT_FRAME_FPS              100", "config")
    require(config, "#define EDOG_SERVO_HIP_SPEED_60_DEG_MS        330", "config")
    require(config, "#define EDOG_SERVO_LEG_SPEED_60_DEG_MS        250", "config")
    require(config, "#define EDOG_SERVO_SPEED_SAFETY_NUM            8", "config")
    require(config, "#define EDOG_SERVO_SPEED_SAFETY_DEN            10", "config")
    require(config, "#define EDOG_12DOF_COMMAND_STEP_HEIGHT_M       0.003", "config")
    require(config, "#define EDOG_PUPPER_MIN_LIFT_MM                0.0", "config")
    reject(config, "#define EDOG_12DOF_MAX_LIFT_MM", "config")
    reject(config, "#define EDOG_PUPPER_MAX_LIFT_MM", "config")

    for function in [
        "trot_cycle",
        "trot_back_cycle",
        "diversion_left_cycle",
        "diversion_right_cycle",
        "trot_left_front_cycle",
        "trot_right_front_cycle",
        "trot_left_back_cycle",
        "trot_right_back_cycle",
        "smooth_stop_to_stand",
        "init_dog",
    ]:
        require_pattern(
            control,
            rf"{function}\s*\(\s*stepLengthM\s*,\s*stepHeightM\s*\)",
            "control",
        )
    for leg in range(4):
        require_pattern(
            control,
            rf"single_leg_gait_cycle\s*\(\s*{leg}\s*,\s*stepLengthM\s*,\s*stepHeightM\s*\)",
            "control",
        )
    require(control, "GetMotionStepSnapshot(&stepLengthM, &stepHeightM)", "control")
    require_pattern(
        control,
        r"leg_group_gait_cycle\s*\(\s*GetCurrentLegMaskSnapshot\s*\(\s*\)\s*,\s*stepLengthM\s*,\s*stepHeightM\s*\)",
        "control",
    )
    require_pattern(
        control,
        r"leg_group_gait_cycle\s*\(\s*g_currentLegMask\s*,\s*EDOG_12DOF_COMMAND_STEP_LENGTH_M\s*,\s*EDOG_12DOF_COMMAND_STEP_HEIGHT_M\s*\)",
        "control",
    )
    require(main_c,
            "init_dog(EDOG_12DOF_COMMAND_STEP_LENGTH_M, EDOG_12DOF_COMMAND_STEP_HEIGHT_M)",
            "main")

    for old in [
        "#define EDOG_12DOF_GAIT_FRAME_FPS              40",
        "#define EDOG_12DOF_MAX_LIFT_MM 14.0",
        "#define EDOG_12DOF_MAX_LIFT_MM 10.0",
        "#define EDOG_PUPPER_MIN_LIFT_MM 6.0",
        "#define EDOG_PUPPER_MAX_LIFT_MM 10.0",
        "0.03, 0.010",
        "0.03, 0.006",
        "0.03, 0.008",
    ]:
        reject(config + motion + gait + control + main_c, old, "project")

    require(motion, "g_servoTableRuntime.cycleMs = requestedCycleMs > g_servoGaitTable.minCycleMs", "motion")
    require(motion, "table->minCycleMs = table->minFrameMs * table->frameCount", "motion")

    print("12DOF low-lift servo-speed-limited cadence checks passed")


if __name__ == "__main__":
    main()
