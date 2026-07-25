#!/usr/bin/env python3
import math
import re
import subprocess
import sys
import tempfile
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
GAIT_SRC = ROOT / "12_DOF_Version/src/gait_generate_12dof.c"
GAIT_HEADER = ROOT / "12_DOF_Version/include/gait_generate_12dof.h"
MOTION_SRC = ROOT / "12_DOF_Version/src/motion_utils_12dof.c"


def require_source_contract():
    gait = GAIT_SRC.read_text(encoding="utf-8")
    header = GAIT_HEADER.read_text(encoding="utf-8")
    motion = MOTION_SRC.read_text(encoding="utf-8")

    if "EDOG_12DOF_8DOF_REFERENCE_KEY_FRAME_COUNT" not in header:
        raise AssertionError("12DOF header must keep the 8DOF reference key-frame compatibility macro")
    for token in [
        "Edog12Dof_ReferencePhaseForFrame",
        "Edog12Dof_ReferenceSwingEnvelopeForPhase",
        "Edog12Dof_SampleReferenceTrotFootPoint",
    ]:
        if token not in header or token not in gait:
            raise AssertionError(f"12DOF gait must expose unified reference helper: {token}")
    if "EDOG_12DOF_GAIT_FRAME_COUNT" not in gait:
        raise AssertionError("continuous trot phase sampling must use the full 30-frame gait target count")

    if "EDOG_12DOF_STANCE_PRESS_MM * sin" in gait:
        raise AssertionError("8DOF-reference stance must keep neutral Z, not add sinusoidal stance press")
    if "static double pyAppleCycloidProgress" not in gait or "pyAppleCycloidSwingFoot" not in gait:
        raise AssertionError("8DOF-reference trot swing must use the Py-Apple cycloid foot trajectory")
    if "static double bezierQuintic" not in gait or "quinticBezierSwingFoot" not in gait:
        raise AssertionError("8DOF-reference trot must keep the quintic Bezier fallback trajectory")
    if "static double bezierQuadratic" in gait or "quadraticBezierSwingFoot" in gait:
        raise AssertionError("8DOF-reference trot swing must not keep the old quadratic Bezier trajectory")
    if "rearSwingBoostEnvelope" not in gait:
        raise AssertionError("rear swing boost must use a smooth envelope with zero boost at swing boundaries")

    if "trotArcCycleInternal" in motion:
        raise AssertionError("arc gait planner must be removed from active 12DOF motion code")
    for name in [
        "trot_left_front_cycle",
        "trot_right_front_cycle",
        "trot_left_back_cycle",
        "trot_right_back_cycle",
    ]:
        match = re.search(r"int\s+" + name + r"\s*\([^)]*\)\s*\{(?P<body>.*?)\n\}", motion, re.S)
        if not match:
            raise AssertionError(f"missing compatibility arc entry {name}")
        body = match.group("body")
        if "return 0;" not in body or "not supported" not in body:
            raise AssertionError(f"{name} must reject unsupported arc gait instead of planning motion")

    if "apply8DofTableFrameStaggered(*gaitSet, frameIndex, EDOG_12DOF_TROT_FRAME_COUNT / 2)" not in motion:
        raise AssertionError("whole-body trot must keep 8DOF diagonal half-cycle phase")
    if "phaseShift = EDOG_12DOF_TROT_FRAME_COUNT / 4" not in motion:
        raise AssertionError("turn gait must keep 8DOF quarter-cycle phase staggering")


def compile_and_run_reference_probe():
    source = r'''
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "include/edog_config.h"
#include "12_DOF_Version/include/gait_generate_12dof.h"

static void require_close(const char *name, double got, double expected, double tolerance)
{
    double diff = fabs(got - expected);
    if (diff > tolerance) {
        fprintf(stderr, "%s expected %.6f +/- %.6f, got %.6f\n",
                name, expected, tolerance, got);
        exit(1);
    }
}

static int abs_int(int value)
{
    return value < 0 ? -value : value;
}

static int max3(int a, int b, int c)
{
    int max = a > b ? a : b;
    return max > c ? max : c;
}

static void require_smooth_table(const char *name, int isRightLeg, int isFrontLeg, int reversed)
{
    Edog12DofJointAngles gait[EDOG_12DOF_TROT_FRAME_COUNT];
    int maxDelta = 0;
    int maxWrapDelta = 0;

    Edog12Dof_GenerateDirectionalTrotTable(gait, 0.020, 0.0, 0.0, 0.006,
                                           isRightLeg, isFrontLeg, reversed);
    for (int i = 0; i < EDOG_12DOF_TROT_FRAME_COUNT; i++) {
        int next = (i + 1) % EDOG_12DOF_TROT_FRAME_COUNT;
        int delta = max3(
            abs_int(gait[next].hipAngleCentiDeg - gait[i].hipAngleCentiDeg),
            abs_int(gait[next].femurAngleCentiDeg - gait[i].femurAngleCentiDeg),
            abs_int(gait[next].tibiaAngleCentiDeg - gait[i].tibiaAngleCentiDeg));
        if (delta > maxDelta) {
            maxDelta = delta;
        }
        if (i >= EDOG_12DOF_TROT_FRAME_COUNT - 2 || i == 0) {
            if (delta > maxWrapDelta) {
                maxWrapDelta = delta;
            }
        }
    }
    if (maxDelta > EDOG_12DOF_GAIT_MAX_JOINT_STEP_CENTI) {
        fprintf(stderr, "%s adjacent joint jump too large: %d centi-deg\n", name, maxDelta);
        exit(1);
    }
    if (maxWrapDelta > EDOG_12DOF_GAIT_MAX_JOINT_STEP_CENTI / 2) {
        fprintf(stderr, "%s wrap-end jump should be quiet, got %d centi-deg\n", name, maxWrapDelta);
        exit(1);
    }
}

int main(void)
{
    Edog12DofFootPoint f0;
    Edog12DofFootPoint f10;
    Edog12DofFootPoint f20;
    int isSwing = 0;
    double strideMm = EDOG_12DOF_HARDWARE_FORWARD_SIGN * 20.0;
    double bodyShiftMm = EDOG_12DOF_TROT_BODY_X_SHIFT_MM;
    double baseX = Edog12Dof_DefaultFootXForLeg(1);
    double baseZ = Edog12Dof_DefaultFootZForLeg(1);

    if (EDOG_12DOF_GAIT_FRAME_COUNT != 30 ||
        EDOG_12DOF_8DOF_REFERENCE_KEY_FRAME_COUNT != 20) {
        fprintf(stderr, "12DOF gait must keep 20 reference key frames and use 30 output target frames\n");
        return 1;
    }

    require_close("phase fwd 0", Edog12Dof_ReferencePhaseForFrame(0, 0), 0.0, 0.000001);
    require_close("phase fwd 1", Edog12Dof_ReferencePhaseForFrame(1, 0), 1.0 / 30.0, 0.000001);
    require_close("phase fwd 2", Edog12Dof_ReferencePhaseForFrame(2, 0), 2.0 / 30.0, 0.000001);
    require_close("phase fwd 28", Edog12Dof_ReferencePhaseForFrame(28, 0), 28.0 / 30.0, 0.000001);
    require_close("phase fwd 29", Edog12Dof_ReferencePhaseForFrame(29, 0), 29.0 / 30.0, 0.000001);
    require_close("phase rev 0", Edog12Dof_ReferencePhaseForFrame(0, 1), 0.0, 0.000001);
    require_close("phase rev 1", Edog12Dof_ReferencePhaseForFrame(1, 1), 29.0 / 30.0, 0.000001);
    require_close("phase rev 2", Edog12Dof_ReferencePhaseForFrame(2, 1), 28.0 / 30.0, 0.000001);
    require_close("phase rev 28", Edog12Dof_ReferencePhaseForFrame(28, 1), 2.0 / 30.0, 0.000001);
    require_close("phase rev 29", Edog12Dof_ReferencePhaseForFrame(29, 1), 1.0 / 30.0, 0.000001);

    require_close("boost boundary start", Edog12Dof_ReferenceSwingEnvelopeForPhase(0.0), 0.0, 0.000001);
    require_close("boost quarter", Edog12Dof_ReferenceSwingEnvelopeForPhase(0.125), 0.5, 0.000001);
    require_close("boost middle", Edog12Dof_ReferenceSwingEnvelopeForPhase(0.25), 1.0, 0.000001);
    require_close("boost three-quarter", Edog12Dof_ReferenceSwingEnvelopeForPhase(0.375), 0.5, 0.000001);
    require_close("boost boundary end", Edog12Dof_ReferenceSwingEnvelopeForPhase(0.5), 0.0, 0.000001);
    require_close("boost stance", Edog12Dof_ReferenceSwingEnvelopeForPhase(0.75), 0.0, 0.000001);

    f0 = Edog12Dof_SampleReferenceTrotFootPoint(0.020, 0.0, 0.0, 0.006,
                                                0, 1, 0, 0, 0, &isSwing);
    if (!isSwing) {
        fprintf(stderr, "frame 0 should be swing start\n");
        return 1;
    }
    require_close("frame0 x", f0.xMm, baseX - bodyShiftMm - strideMm / 2.0, 0.001);
    require_close("frame0 z", f0.zMm, baseZ, 0.001);

    f10 = Edog12Dof_SampleReferenceTrotFootPoint(0.020, 0.0, 0.0, 0.006,
                                                 0, 1, 0, 0, 8, &isSwing);
    if (!isSwing || f10.zMm >= baseZ) {
        fprintf(stderr, "mid swing should lift the foot above neutral Z-down height\n");
        return 1;
    }

    f20 = Edog12Dof_SampleReferenceTrotFootPoint(0.020, 0.0, 0.0, 0.006,
                                                 0, 1, 0, 0, 15, &isSwing);
    if (isSwing) {
        fprintf(stderr, "frame 20 should be stance\n");
        return 1;
    }
    require_close("stance z neutral", f20.zMm, baseZ, 0.001);

    require_smooth_table("LF forward", 0, 1, 0);
    require_smooth_table("RF forward", 1, 1, 0);
    require_smooth_table("LB forward", 0, 0, 0);
    require_smooth_table("RB forward", 1, 0, 0);
    require_smooth_table("LF reverse", 0, 1, 1);
    require_smooth_table("RB reverse", 1, 0, 1);
    return 0;
}
'''

    with tempfile.TemporaryDirectory() as tmpdir:
        stub_include = Path(tmpdir) / "stubs"
        test_c = Path(tmpdir) / "test_12dof_8dof_reference_gait.c"
        binary = Path(tmpdir) / "test_12dof_8dof_reference_gait"
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
    compile_and_run_reference_probe()
    print("12DOF 8DOF-reference gait checks passed")


if __name__ == "__main__":
    try:
        main()
    except (AssertionError, subprocess.CalledProcessError) as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        sys.exit(1)
