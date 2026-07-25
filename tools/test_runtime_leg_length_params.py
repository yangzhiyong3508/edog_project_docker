#!/usr/bin/env python3
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read(path):
    return (ROOT / path).read_text(encoding="utf-8")


def require(text, token, label):
    if token not in text:
        raise AssertionError(f"{label} missing token: {token}")


def main():
    config = read("include/edog_config.h")
    iot_header = read("utils/include/iot_control.h")
    iot = read("utils/src/iot.c")
    control = read("utils/src/iot_control.c")
    gait_header = read("12_DOF_Version/include/gait_generate_12dof.h")
    gait = read("12_DOF_Version/src/gait_generate_12dof.c")

    require(config, "#define EDOG_12DOF_THIGH_LENGTH_DEFAULT_M       0.107", "edog_config.h")
    require(config, "#define EDOG_12DOF_CALF_LENGTH_DEFAULT_M        0.135", "edog_config.h")
    require(config, "#define EDOG_12DOF_LINK_LENGTH_MIN_M           0.001", "edog_config.h")

    require(iot_header, "double thighLengthM;", "iot_control.h")
    require(iot_header, "double calfLengthM;", "iot_control.h")

    require(iot, '"thigh_length_m"', "iot.c")
    require(iot, '"calf_length_m"', "iot.c")
    require(iot, "ParseGaitGeometryParameters", "iot.c")
    require(iot, "EnqueueTextCommand(commandText, stepLengthM, stepHeightM,", "iot.c")
    require(iot, "hipAdductionDeg, thighLengthM, calfLengthM", "iot.c")

    require(control, "SetMotionLegLengthState", "iot_control.c")
    require(control, "GetMotionLegLengthSnapshot(&thighLengthM, &calfLengthM)", "iot_control.c")
    require(control, "Edog12Dof_SetLinkLengthsM(thighLengthM, calfLengthM)", "iot_control.c")

    require(gait_header, "void Edog12Dof_SetLinkLengthsM(double thighLengthM, double calfLengthM);", "gait_generate_12dof.h")
    require(gait_header, "double Edog12Dof_GetThighLengthMm(void);", "gait_generate_12dof.h")
    require(gait_header, "double Edog12Dof_GetCalfLengthMm(void);", "gait_generate_12dof.h")
    require(gait, "static double g_runtimeThighLengthMm", "gait_generate_12dof.c")
    require(gait, "static double g_runtimeCalfLengthMm", "gait_generate_12dof.c")
    require(gait, "Edog12Dof_SetLinkLengthsM", "gait_generate_12dof.c")
    require(gait, "double femur = Edog12Dof_GetThighLengthMm();", "gait_generate_12dof.c")
    require(gait, "double tibia = Edog12Dof_GetCalfLengthMm();", "gait_generate_12dof.c")

    print("runtime leg length parameter checks passed")


if __name__ == "__main__":
    main()
