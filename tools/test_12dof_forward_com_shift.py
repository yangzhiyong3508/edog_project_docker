#!/usr/bin/env python3
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MOTION = ROOT / "12_DOF_Version/src/motion_utils_12dof.c"
GAIT = ROOT / "12_DOF_Version/src/gait_generate_12dof.c"
HEADER = ROOT / "12_DOF_Version/include/gait_generate_12dof.h"
CONFIG = ROOT / "include/edog_config.h"


def require(text, token, label):
    if token not in text:
        raise AssertionError(f"{label} missing token: {token}")


def main():
    motion = MOTION.read_text(encoding="utf-8")
    gait = GAIT.read_text(encoding="utf-8")
    header = HEADER.read_text(encoding="utf-8")
    config = CONFIG.read_text(encoding="utf-8")

    require(motion, "Edog12Dof_DefaultFootXForLeg", "motion")
    require(motion, "Edog12Dof_DefaultFootZForLeg", "motion")
    require(gait, "Edog12Dof_DefaultFootXForLeg(isFrontLeg)", "gait")
    require(gait, "Edog12Dof_DefaultFootZForLeg(isFrontLeg)", "gait")
    require(config, "#define EDOG_12DOF_REAR_FOOT_BACK_OFFSET_MM    0.0", "config")
    require(config, "#define EDOG_12DOF_DEFAULT_FOOT_Z_MM           140.0", "config")
    require(config, "#define EDOG_12DOF_REAR_FOOT_UP_OFFSET_MM      0.0", "config")

    require(header, "#define EDOG_12DOF_STAND_THIGH_DELTA_DEG -61", "header")
    require(header, "#define EDOG_12DOF_STAND_CALF_DELTA_DEG 20", "header")

    require(motion, "static const Edog12DofJointAngles g_standJointAngles", "motion")
    require(motion, "Edog12Dof_DefaultFootXForLeg(leg == 0 || leg == 1)", "motion default rear offset")
    require(motion, "Edog12Dof_DefaultFootZForLeg(leg == 0 || leg == 1)", "motion default rear height")
    require(motion, "Edog12DofFootPoint foot = getDefaultFootPoint(leg);", "balance stand")
    require(motion, "Edog12DofFootPoint neutral = getDefaultFootPoint(leg);", "realtime neutral")
    require(motion, "state->leg[leg].foot = getDefaultFootPoint(leg);", "realtime init")
    require(motion, "legState->touchdown = neutral;", "realtime touchdown")

    require(gait, "EDOG_12DOF_DEFAULT_FOOT_X_MM - EDOG_12DOF_REAR_FOOT_BACK_OFFSET_MM", "gait rear foot")
    require(gait, "EDOG_12DOF_DEFAULT_FOOT_Z_MM - EDOG_12DOF_REAR_FOOT_UP_OFFSET_MM", "gait rear foot Z")
    require(gait, "Edog12Dof_IK(&foot, isRightLeg, &gaitTable[k])", "gait IK")

    print("12DOF forward COM shift checks passed")


if __name__ == "__main__":
    main()
