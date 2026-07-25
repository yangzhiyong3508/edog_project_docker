#!/usr/bin/env python3
import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "include/edog_config.h"
GAIT = ROOT / "12_DOF_Version/src/gait_generate_12dof.c"
MOTION = ROOT / "12_DOF_Version/src/motion_utils_12dof.c"
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


def require_define(text, name, value, label):
    pattern = rf"#define\s+{re.escape(name)}\s+{re.escape(str(value))}\b"
    if not re.search(pattern, text):
        raise AssertionError(f"{label} missing define {name} {value}")


def main():
    config = CONFIG.read_text(encoding="utf-8")
    gait = GAIT.read_text(encoding="utf-8")
    motion = MOTION.read_text(encoding="utf-8")
    control = CONTROL.read_text(encoding="utf-8")
    main_c = MAIN.read_text(encoding="utf-8")
    project = "\n".join([config, gait, motion, control, main_c])

    require_define(config, "EDOG_12DOF_GAIT_FRAME_FPS", 50, "config")
    require(config,
            "#define EDOG_12DOF_GAIT_FRAME_PERIOD_US        (1000000 / EDOG_12DOF_GAIT_FRAME_FPS)",
            "config")
    require_define(config, "EDOG_12DOF_COMMAND_STEP_LENGTH_M", "0.01", "config")
    require_define(config, "EDOG_12DOF_COMMAND_STEP_HEIGHT_M", "0.003", "config")
    require_define(config, "EDOG_12DOF_MIN_STRIDE_MM", "1.0", "config")
    require_define(config, "EDOG_12DOF_MIN_LIFT_MM", "0.0", "config")
    reject(config, "#define EDOG_12DOF_MAX_STRIDE_MM", "config")
    reject(config, "#define EDOG_12DOF_MAX_LIFT_MM", "config")
    require(gait,
            "double strideMm = signedStepMmWithMinimum(effectiveForwardM,\n"
            "                                            EDOG_12DOF_MIN_STRIDE_MM);",
            "gait")
    require(gait,
            "double liftMm = minimumLiftMm(fabs(stepHeightM) * 1000.0,\n"
            "                                EDOG_12DOF_MIN_LIFT_MM);",
            "gait")

    require_define(config, "EDOG_PUPPER_MIN_LIFT_MM", "0.0", "config")
    reject(config, "#define EDOG_12DOF_REALTIME_MAX_FORWARD_MM", "config")
    reject(config, "#define EDOG_PUPPER_MAX_LIFT_MM", "config")
    reject(config, "#define EDOG_12DOF_REALTIME_LEGACY_MAX_LIFT_MM", "config")
    require(motion,
            "double liftMm = minimumLiftMm(cmd->stepHeightM * 1000.0,\n"
            "                                EDOG_12DOF_REALTIME_LEGACY_MIN_LIFT_MM);",
            "motion")
    require(motion,
            "liftMm = minimumLiftMm(cmd->stepHeightM * 1000.0,\n"
            "                         EDOG_PUPPER_MIN_LIFT_MM);",
            "motion")

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
        "#define EDOG_12DOF_MAX_STRIDE_MM 35.0",
        "#define EDOG_12DOF_MAX_LIFT_MM 10.0",
        "#define EDOG_12DOF_REALTIME_MAX_FORWARD_MM 32.0",
        "#define EDOG_PUPPER_MIN_LIFT_MM 6.0",
        "#define EDOG_PUPPER_MAX_LIFT_MM 10.0",
        "clampSignedStepMm(effectiveForwardM, 6.0",
        "clampDouble(fabs(stepHeightM) * 1000.0, 4.0",
        "clampDoubleValue(cmd->stepHeightM * 1000.0, 4.0, 22.0)",
        "0.03, 0.006",
        "0.03, 0.008",
    ]:
        reject(project, old, "project")

    require(motion, "static int getRequestedCycleMs(void)", "motion")
    require(motion, "static const int requestedCycleMs[7] = {1600, 1200, 900, 700, 560, 480, 480}", "motion")
    require(motion, "g_servoTableRuntime.cycleMs = requestedCycleMs > g_servoGaitTable.minCycleMs", "motion")

    print("12DOF tiny-step table-cycle speed limit checks passed")


if __name__ == "__main__":
    main()
