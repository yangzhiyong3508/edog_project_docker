#!/usr/bin/env python3
import re
import subprocess
import sys
import tempfile
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
GAIT_SRC = ROOT / "12_DOF_Version/src/gait_generate_12dof.c"
MOTION_SRC = ROOT / "12_DOF_Version/src/motion_utils_12dof.c"


def require_source_contract():
    text = GAIT_SRC.read_text(encoding="utf-8")
    motion = MOTION_SRC.read_text(encoding="utf-8")
    for token in [
        "static double bezierQuintic",
        "static Edog12DofFootPoint quinticBezierSwingFoot",
        "static Edog12DofFootPoint pyAppleCycloidSwingFoot",
        "Edog12Dof_SampleReferenceTrotFootPoint",
        "Edog12Dof_ReferencePhaseForFrame",
        "Edog12Dof_ReferenceSwingEnvelopeForPhase",
        "rearSwingBoostEnvelope",
        "static int clampJointStepCenti",
        "static void limitGaitTableJointStepCenti",
        "EDOG_12DOF_GAIT_MAX_JOINT_STEP_CENTI",
    ]:
        if token not in text:
            raise AssertionError(f"gait source missing quintic smoothness token: {token}")

    if "static double bezierQuadratic" in text:
        raise AssertionError("swing trajectory must not keep the old quadratic Bezier helper")
    if "quadraticBezierSwingFoot" in text:
        raise AssertionError("swing trajectory must not use the old quadratic helper")

    if "double swingPortion = EDOG_12DOF_TROT_SWING_PORTION" not in text:
        raise AssertionError("swing lift must use the configured shortened trot swing portion")
    if "return pyAppleCycloidLift(phase / swingPortion);" not in text:
        raise AssertionError("default swing lift must use the Py-Apple cycloid 0 -> 1 -> 0 envelope")
    if "return bezierSwingLift(phase / swingPortion);" not in text:
        raise AssertionError("quintic Bezier swing lift fallback must remain available")
    if "EDOG_12DOF_STANCE_PRESS_MM * sin" in text:
        raise AssertionError("8DOF-reference stance must keep neutral Z instead of sinusoidal press")
    for token in [
        "static int planServoTrapezoidStepCenti(int channel, int targetCenti)",
        "writeServoProfileAngleCenti(channel, planServoTrapezoidStepCenti(channel, target))",
        "servoVelocityCentiPerFrame[channel] = next - current",
    ]:
        if token not in motion:
            raise AssertionError(f"servo output must keep smooth interpolation token: {token}")


def compile_and_run_smoothness_probe():
    source = r'''
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "include/edog_config.h"
#include "12_DOF_Version/include/gait_generate_12dof.h"

static int abs_int(int value)
{
    return value < 0 ? -value : value;
}

static int max3(int a, int b, int c)
{
    int max = a > b ? a : b;
    return max > c ? max : c;
}

static void require_close(const char *name, double got, double expected, double tolerance)
{
    double diff = fabs(got - expected);
    if (diff > tolerance) {
        fprintf(stderr, "%s expected %.6f +/- %.6f, got %.6f\n",
                name, expected, tolerance, got);
        exit(1);
    }
}

static void require_smooth_table(double stepM, double liftM,
                                 int isRightLeg, int isFrontLeg, int reversed)
{
    Edog12DofJointAngles gait[EDOG_12DOF_TROT_FRAME_COUNT];
    int maxDelta = 0;
    int maxFrame = -1;
    int maxWrapDelta = 0;

    Edog12Dof_ResetRuntimeGeometry();
    Edog12Dof_GenerateDirectionalTrotTable(gait, stepM, 0.0, 0.0,
                                           liftM, isRightLeg, isFrontLeg, reversed);

    for (int i = 0; i < EDOG_12DOF_TROT_FRAME_COUNT; i++) {
        int next = (i + 1) % EDOG_12DOF_TROT_FRAME_COUNT;
        int delta = max3(
            abs_int(gait[next].hipAngleCentiDeg - gait[i].hipAngleCentiDeg),
            abs_int(gait[next].femurAngleCentiDeg - gait[i].femurAngleCentiDeg),
            abs_int(gait[next].tibiaAngleCentiDeg - gait[i].tibiaAngleCentiDeg));
        if (delta > maxDelta) {
            maxDelta = delta;
            maxFrame = i;
        }
        if (i >= EDOG_12DOF_TROT_FRAME_COUNT - 2 || i == 0) {
            if (delta > maxWrapDelta) {
                maxWrapDelta = delta;
            }
        }
    }

    if (maxDelta > EDOG_12DOF_GAIT_MAX_JOINT_STEP_CENTI) {
        fprintf(stderr,
                "adjacent gait frame jump too large: step=%.3f lift=%.3f right=%d front=%d rev=%d max=%d centi at frame %d limit=%d\n",
                stepM, liftM, isRightLeg, isFrontLeg, reversed,
                maxDelta, maxFrame, EDOG_12DOF_GAIT_MAX_JOINT_STEP_CENTI);
        exit(1);
    }
    if (maxWrapDelta > EDOG_12DOF_GAIT_MAX_JOINT_STEP_CENTI) {
        fprintf(stderr, "wrap-end gait frame jump too large: max=%d limit=%d\n",
                maxWrapDelta, EDOG_12DOF_GAIT_MAX_JOINT_STEP_CENTI);
        exit(1);
    }
}

int main(void)
{
    require_close("lift boundary start", Edog12Dof_ReferenceSwingEnvelopeForPhase(0.0), 0.0, 0.000001);
    require_close("lift quarter", Edog12Dof_ReferenceSwingEnvelopeForPhase(EDOG_12DOF_TROT_SWING_PORTION * 0.25), 0.5, 0.000001);
    require_close("lift middle", Edog12Dof_ReferenceSwingEnvelopeForPhase(EDOG_12DOF_TROT_SWING_PORTION * 0.50), 1.0, 0.000001);
    require_close("lift three-quarter", Edog12Dof_ReferenceSwingEnvelopeForPhase(EDOG_12DOF_TROT_SWING_PORTION * 0.75), 0.5, 0.000001);
    require_close("lift boundary end", Edog12Dof_ReferenceSwingEnvelopeForPhase(EDOG_12DOF_TROT_SWING_PORTION), 0.0, 0.000001);
    require_close("lift stance", Edog12Dof_ReferenceSwingEnvelopeForPhase(0.75), 0.0, 0.000001);

    if (EDOG_12DOF_GAIT_MAX_JOINT_STEP_CENTI > 360 ||
        EDOG_12DOF_GAIT_MAX_JOINT_STEP_CENTI < 300) {
        fprintf(stderr, "gait target step limit should allow 3deg target changes but stay bounded, got %d centi-deg\n",
                EDOG_12DOF_GAIT_MAX_JOINT_STEP_CENTI);
        return 1;
    }

    for (int right = 0; right <= 1; right++) {
        for (int front = 0; front <= 1; front++) {
            for (int reversed = 0; reversed <= 1; reversed++) {
                require_smooth_table(0.010, 0.003, right, front, reversed);
                require_smooth_table(0.020, 0.006, right, front, reversed);
                require_smooth_table(0.030, 0.010, right, front, reversed);
            }
        }
    }
    return 0;
}
'''

    with tempfile.TemporaryDirectory() as tmpdir:
        stub_include = Path(tmpdir) / "stubs"
        test_c = Path(tmpdir) / "test_12dof_bezier_swing_smoothness.c"
        binary = Path(tmpdir) / "test_12dof_bezier_swing_smoothness"
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


def main():
    require_source_contract()
    compile_and_run_smoothness_probe()
    print("12DOF cycloid default and quintic fallback swing smoothness checks passed")


if __name__ == "__main__":
    try:
        main()
    except (AssertionError, subprocess.CalledProcessError) as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        sys.exit(1)
