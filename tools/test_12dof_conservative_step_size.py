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
        raise AssertionError(f"{label} still contains oversized token: {token}")


def main():
    config = CONFIG.read_text(encoding="utf-8")
    motion = MOTION.read_text(encoding="utf-8")
    gait = GAIT.read_text(encoding="utf-8")
    control = CONTROL.read_text(encoding="utf-8")
    main_c = MAIN.read_text(encoding="utf-8")

    require(config, "#define EDOG_PUPPER_MIN_LIFT_MM                0.0", "config")
    for token in [
        "#define EDOG_12DOF_MAX_STRIDE_MM",
        "#define EDOG_12DOF_MAX_LIFT_MM",
        "#define EDOG_12DOF_REALTIME_MAX_FORWARD_MM",
        "#define EDOG_12DOF_REALTIME_LEGACY_MAX_LIFT_MM",
        "#define EDOG_PUPPER_MAX_LIFT_MM",
    ]:
        reject(config, token, "config")

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
    require_pattern(
        control,
        r"leg_group_gait_cycle\s*\(\s*GetCurrentLegMaskSnapshot\s*\(\s*\)\s*,\s*stepLengthM\s*,\s*stepHeightM\s*\)",
        "control",
    )
    require(main_c,
            "init_dog(EDOG_12DOF_COMMAND_STEP_LENGTH_M, EDOG_12DOF_COMMAND_STEP_HEIGHT_M)",
            "main")

    for oversized in [
        "trot_cycle(0.03, 0.006)",
        "trot_back_cycle(0.03, 0.006)",
        "diversion_left_cycle(0.03, 0.006)",
        "diversion_right_cycle(0.03, 0.006)",
        "trot_left_front_cycle(0.03, 0.006)",
        "trot_right_front_cycle(0.03, 0.006)",
        "trot_left_back_cycle(0.03, 0.006)",
        "trot_right_back_cycle(0.03, 0.006)",
        "single_leg_gait_cycle(0, 0.03, 0.008)",
        "single_leg_gait_cycle(1, 0.03, 0.008)",
        "single_leg_gait_cycle(2, 0.03, 0.008)",
        "single_leg_gait_cycle(3, 0.03, 0.008)",
        "leg_group_gait_cycle(GetCurrentLegMaskSnapshot(), 0.03, 0.008)",
        "leg_group_gait_cycle(g_currentLegMask, 0.03, 0.008)",
        "trot_cycle(0.04, 0.012)",
        "trot_back_cycle(0.04, 0.012)",
        "diversion_left_cycle(0.04, 0.012)",
        "diversion_right_cycle(0.04, 0.012)",
        "trot_left_front_cycle(0.06, 0.02)",
        "trot_right_front_cycle(0.06, 0.02)",
        "trot_left_back_cycle(0.06, 0.02)",
        "trot_right_back_cycle(0.06, 0.02)",
        "single_leg_gait_cycle(0, 0.04, 0.015)",
        "single_leg_gait_cycle(1, 0.04, 0.015)",
        "single_leg_gait_cycle(2, 0.04, 0.015)",
        "single_leg_gait_cycle(3, 0.04, 0.015)",
        "leg_group_gait_cycle(GetCurrentLegMaskSnapshot(), 0.04, 0.015)",
        "leg_group_gait_cycle(g_currentLegMask, 0.04, 0.015)",
        "init_dog(0.04, 0.01)",
        "smooth_stop_to_stand(0.04, 0.01)",
    ]:
        reject(control + main_c, oversized, "control/main")

    print("12DOF tiny step-size checks passed")


if __name__ == "__main__":
    main()
