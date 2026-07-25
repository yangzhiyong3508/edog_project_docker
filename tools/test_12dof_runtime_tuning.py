#!/usr/bin/env python3
import subprocess
import tempfile
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read(path):
    return (ROOT / path).read_text(encoding="utf-8")


def require(text, token, label):
    if token not in text:
        raise AssertionError(f"{label} missing token: {token}")


def require_not(text, token, label):
    if token in text:
        raise AssertionError(f"{label} must not contain token: {token}")


def run_gait_geometry_behavior_test():
    source = r'''
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "include/edog_config.h"
#include "12_DOF_Version/include/gait_generate_12dof.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static void require_close(const char *name, double got, double expected, double tolerance)
{
    double diff = fabs(got - expected);
    if (diff > tolerance) {
        fprintf(stderr, "%s expected %.6f +/- %.6f, got %.6f\n",
                name, expected, tolerance, got);
        exit(1);
    }
}

int main(void)
{
    double frontHip = 0.0;
    double rearHip = 0.0;
    double thigh = 0.0;
    double calf = 0.0;
    double frontZDelta = 0.0;
    double rearZDelta = 0.0;
    double frontZ;
    double rearZ;

    Edog12Dof_ResetRuntimeGeometry();
    Edog12Dof_GetRuntimeGeometryForFrontRear(&frontHip, &rearHip, &thigh, &calf);
    Edog12Dof_GetRuntimeFootZDeltas(&frontZDelta, &rearZDelta);
    require_close("default front hip", frontHip, -13.0, 0.001);
    require_close("default rear hip", rearHip, -8.0, 0.001);
    require_close("default front body height delta", frontZDelta, 0.0, 0.001);
    require_close("default rear body height delta", rearZDelta, 0.0, 0.001);

    frontZ = Edog12Dof_DefaultFootZForLeg(1);
    rearZ = Edog12Dof_DefaultFootZForLeg(0);
    require_close("front Y uses front adduction", Edog12Dof_DefaultFootYForLeg(0),
                  tan(-13.0 * M_PI / 180.0) * frontZ, 0.001);
    require_close("rear Y uses rear adduction", Edog12Dof_DefaultFootYForLeg(2),
                  tan(-8.0 * M_PI / 180.0) * rearZ, 0.001);
    require_close("right front mirrors front Y", Edog12Dof_DefaultFootYForLeg(1),
                  -tan(-13.0 * M_PI / 180.0) * frontZ, 0.001);

    Edog12Dof_SetRuntimeGeometryForFrontRear(-20.0, -5.0, 107.0, 135.0);
    Edog12Dof_GetRuntimeGeometryForFrontRear(&frontHip, &rearHip, &thigh, &calf);
    require_close("set front hip", frontHip, -20.0, 0.001);
    require_close("set rear hip", rearHip, -5.0, 0.001);
    require_close("updated front Y", Edog12Dof_DefaultFootYForLeg(0),
                  tan(-20.0 * M_PI / 180.0) * frontZ, 0.001);
    require_close("updated rear Y", Edog12Dof_DefaultFootYForLeg(2),
                  tan(-5.0 * M_PI / 180.0) * rearZ, 0.001);

    Edog12Dof_SetRuntimeGeometry(-11.0, 107.0, 135.0);
    Edog12Dof_GetRuntimeGeometryForFrontRear(&frontHip, &rearHip, &thigh, &calf);
    require_close("legacy setter front", frontHip, -11.0, 0.001);
    require_close("legacy setter rear", rearHip, -11.0, 0.001);

    Edog12Dof_SetRuntimeFootZDeltas(12.0, -9.0);
    Edog12Dof_GetRuntimeFootZDeltas(&frontZDelta, &rearZDelta);
    require_close("set front body height delta", frontZDelta, 12.0, 0.001);
    require_close("set rear body height delta", rearZDelta, -9.0, 0.001);
    require_close("front default Z with runtime delta", Edog12Dof_DefaultFootZForLeg(1), 152.0, 0.001);
    require_close("rear default Z with runtime delta", Edog12Dof_DefaultFootZForLeg(0), 131.0, 0.001);

    Edog12Dof_SetRuntimeFootZDeltas(-80.0, 80.0);
    Edog12Dof_GetRuntimeFootZDeltas(&frontZDelta, &rearZDelta);
    require_close("front body height delta clamp", frontZDelta, -40.0, 0.001);
    require_close("rear body height delta clamp", rearZDelta, 40.0, 0.001);

    Edog12Dof_ResetRuntimeGeometry();
    require_close("reset front default Z", Edog12Dof_DefaultFootZForLeg(1), 140.0, 0.001);
    require_close("reset rear default Z", Edog12Dof_DefaultFootZForLeg(0), 140.0, 0.001);
    return 0;
}
'''
    with tempfile.TemporaryDirectory() as tmpdir:
        tmp = Path(tmpdir)
        stub_include = tmp / "stubs"
        test_c = tmp / "test_12dof_runtime_tuning.c"
        binary = tmp / "test_12dof_runtime_tuning"
        stub_include.mkdir()
        (stub_include / "iot_gpio.h").write_text(
            "#ifndef IOT_GPIO_H\n"
            "#define IOT_GPIO_H\n"
            "#define GPIO0_PC7 0\n"
            "#define GPIO0_PB5 0\n"
            "#define GPIO0_PB4 0\n"
            "#define GPIO1_PD0 0\n"
            "#endif\n",
            encoding="utf-8",
        )
        test_c.write_text(source, encoding="utf-8")
        cmd = [
            "gcc",
            "-std=c99",
            "-D_DEFAULT_SOURCE",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-I",
            str(stub_include),
            "-I",
            str(ROOT),
            str(test_c),
            str(ROOT / "12_DOF_Version/src/gait_generate_12dof.c"),
            "-lm",
            "-o",
            str(binary),
        ]
        subprocess.run(cmd, check=True)
        subprocess.run([str(binary)], check=True)


def test_static_runtime_tuning_contracts():
    gait_header = read("12_DOF_Version/include/gait_generate_12dof.h")
    gait = read("12_DOF_Version/src/gait_generate_12dof.c")
    motion_header = read("12_DOF_Version/include/motion_utils_12dof.h")
    motion = read("12_DOF_Version/src/motion_utils_12dof.c")
    iot_header = read("utils/include/iot_control.h")
    iot_control = read("utils/src/iot_control.c")
    iot = read("utils/src/iot.c")

    for token in [
        "EDOG_12DOF_FRONT_HIP_ADDUCTION_DEFAULT_DEG",
        "EDOG_12DOF_REAR_HIP_ADDUCTION_DEFAULT_DEG",
        "void Edog12Dof_SetRuntimeGeometryForFrontRear",
        "void Edog12Dof_GetRuntimeGeometryForFrontRear",
        "void Edog12Dof_SetRuntimeFootZDeltas",
        "void Edog12Dof_GetRuntimeFootZDeltas",
        "double Edog12Dof_HipAdductionDegForLeg(int legIndex);",
    ]:
        require(gait_header, token, "gait_generate_12dof.h")
    require(gait, "g_runtimeFrontHipAdductionDeg", "gait_generate_12dof.c")
    require(gait, "g_runtimeRearHipAdductionDeg", "gait_generate_12dof.c")
    require(gait, "g_runtimeFrontFootZDeltaMm", "gait_generate_12dof.c")
    require(gait, "g_runtimeRearFootZDeltaMm", "gait_generate_12dof.c")
    require(gait, "Edog12Dof_HipAdductionDegForLeg", "gait_generate_12dof.c")
    require(gait, "Edog12Dof_FootYOnHipPlaneForLeg", "gait_generate_12dof.c")
    require(gait, "EDOG_12DOF_BODY_HEIGHT_DELTA_MIN_MM", "gait_generate_12dof.c")
    require(gait, "baseZMm + g_runtimeFrontFootZDeltaMm", "gait_generate_12dof.c")
    require(gait, "baseZMm + g_runtimeRearFootZDeltaMm", "gait_generate_12dof.c")
    require_not(gait, "static double g_runtimeHipAdductionDeg", "gait_generate_12dof.c")

    require(motion_header, "setRuntimeGaitGeometryForFrontRear", "motion_utils_12dof.h")
    require(motion_header, "setImuBalanceStrengthPercent", "motion_utils_12dof.h")
    require(motion, "g_imuBalanceStrengthPercent", "motion_utils_12dof.c")
    require(motion, "setImuBalanceStrengthPercent", "motion_utils_12dof.c")
    require(motion, "loadRuntimeTuningFromKv", "motion_utils_12dof.c")
    require(motion, "saveRuntimeTuningToKv", "motion_utils_12dof.c")
    require(motion, "EDOG_RUNTIME_FRONT_BODY_HEIGHT_KEY", "motion_utils_12dof.c")
    require(motion, "EDOG_RUNTIME_REAR_BODY_HEIGHT_KEY", "motion_utils_12dof.c")
    require(motion, "Edog12Dof_GetRuntimeFootZDeltas", "motion_utils_12dof.c")
    require(motion, "Edog12Dof_SetRuntimeFootZDeltas", "motion_utils_12dof.c")
    require(motion, "bodyZUpCompMm = bodyZUpCompMm * (double)g_imuBalanceStrengthPercent / 100.0", "motion_utils_12dof.c")
    require(motion, "Edog12Dof_FootYOnHipPlaneForLeg", "motion_utils_12dof.c")

    require(iot_header, "IOT_CONTROL_COMMAND_RUNTIME_TUNING_SET", "iot_control.h")
    require(iot_header, "double frontHipAdductionDeg;", "iot_control.h")
    require(iot_header, "double rearHipAdductionDeg;", "iot_control.h")
    require(iot_header, "double frontBodyHeightDeltaMm;", "iot_control.h")
    require(iot_header, "double rearBodyHeightDeltaMm;", "iot_control.h")
    require(iot_header, "int imuBalanceStrengthPercent;", "iot_control.h")
    require(iot_control, "IotControl_SetRuntimeTuning", "iot_control.c")
    require(iot_control, "setRuntimeGaitGeometryForFrontRear", "iot_control.c")
    require(iot_control, "setImuBalanceStrengthPercent", "iot_control.c")
    require(iot_control, "saveRuntimeTuningToKv", "iot_control.c")
    require(iot, '"runtime_tuning_set"', "iot.c")
    require(iot, '"front_hip_adduction_deg"', "iot.c")
    require(iot, '"rear_hip_adduction_deg"', "iot.c")
    require(iot, '"front_body_height_delta_mm"', "iot.c")
    require(iot, '"rear_body_height_delta_mm"', "iot.c")
    require(iot, '"imu_balance_strength_percent"', "iot.c")


def main():
    run_gait_geometry_behavior_test()
    test_static_runtime_tuning_contracts()
    print("12DOF runtime tuning checks passed")


if __name__ == "__main__":
    main()
