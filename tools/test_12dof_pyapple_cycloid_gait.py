#!/usr/bin/env python3
import subprocess
import sys
import tempfile
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "include/edog_config.h"
GAIT_SRC = ROOT / "12_DOF_Version/src/gait_generate_12dof.c"


def require(token, text, where):
    if token not in text:
        raise AssertionError(f"{where} missing {token}")


def require_source_contract():
    config = CONFIG.read_text(encoding="utf-8")
    gait = GAIT_SRC.read_text(encoding="utf-8")

    for token in [
        "#define EDOG_12DOF_TROT_TRAJECTORY_CYCLOID     1",
        "#define EDOG_12DOF_TROT_BODY_X_SHIFT_MM        5.0",
    ]:
        require(token, config, "edog_config.h")

    for token in [
        "static double pyAppleCycloidProgress",
        "static double pyAppleCycloidLift",
        "static Edog12DofFootPoint pyAppleCycloidSwingFoot",
        "static Edog12DofFootPoint quinticBezierSwingFoot",
        "#if EDOG_12DOF_TROT_TRAJECTORY_CYCLOID",
        "foot.xMm -= bodyShiftMm;",
        "foot.zMm = baseZMm;",
    ]:
        require(token, gait, "gait_generate_12dof.c")

    if "EDOG_12DOF_STANCE_PRESS_MM * sin" in gait:
        raise AssertionError("support phase must not add sinusoidal vertical press")


def compile_and_run_probe():
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

static void require_gt(const char *name, double lhs, double rhs)
{
    if (!(lhs > rhs)) {
        fprintf(stderr, "%s expected %.6f > %.6f\n", name, lhs, rhs);
        exit(1);
    }
}

static void require_lt(const char *name, double lhs, double rhs)
{
    if (!(lhs < rhs)) {
        fprintf(stderr, "%s expected %.6f < %.6f\n", name, lhs, rhs);
        exit(1);
    }
}

int main(void)
{
    Edog12DofFootPoint f0;
    Edog12DofFootPoint f2;
    Edog12DofFootPoint f6;
    Edog12DofFootPoint f10;
    Edog12DofFootPoint f12;
    Edog12DofFootPoint f29;
    Edog12DofFootPoint back0;
    int isSwing = 0;
    double baseX = Edog12Dof_DefaultFootXForLeg(1);
    double baseZ = Edog12Dof_DefaultFootZForLeg(1);
    double strideMm = EDOG_12DOF_HARDWARE_FORWARD_SIGN * 20.0;
    double bodyShift = EDOG_12DOF_TROT_BODY_X_SHIFT_MM;

    if (EDOG_12DOF_TROT_TRAJECTORY_CYCLOID != 1) {
        fprintf(stderr, "Py-Apple cycloid trajectory must be the default test mode\n");
        return 1;
    }

    require_close("cycloid lift start", Edog12Dof_ReferenceSwingEnvelopeForPhase(0.0), 0.0, 0.000001);
    require_close("cycloid lift quarter", Edog12Dof_ReferenceSwingEnvelopeForPhase(EDOG_12DOF_TROT_SWING_PORTION * 0.25), 0.5, 0.000001);
    require_close("cycloid lift middle", Edog12Dof_ReferenceSwingEnvelopeForPhase(EDOG_12DOF_TROT_SWING_PORTION * 0.50), 1.0, 0.000001);
    require_close("cycloid lift three-quarter", Edog12Dof_ReferenceSwingEnvelopeForPhase(EDOG_12DOF_TROT_SWING_PORTION * 0.75), 0.5, 0.000001);
    require_close("cycloid lift end", Edog12Dof_ReferenceSwingEnvelopeForPhase(EDOG_12DOF_TROT_SWING_PORTION), 0.0, 0.000001);
    require_close("cycloid lift stance", Edog12Dof_ReferenceSwingEnvelopeForPhase(0.75), 0.0, 0.000001);

    f0 = Edog12Dof_SampleReferenceTrotFootPoint(0.020, 0.0, 0.0, 0.006,
                                                0, 1, 0, 0, 0, &isSwing);
    if (!isSwing) {
        fprintf(stderr, "frame 0 should be swing start\n");
        return 1;
    }
    require_close("forward swing start x includes body shift",
                  f0.xMm, baseX - bodyShift - strideMm / 2.0, 0.001);
    require_close("forward swing start z", f0.zMm, baseZ, 0.001);

    f2 = Edog12Dof_SampleReferenceTrotFootPoint(0.020, 0.0, 0.0, 0.006,
                                                0, 1, 0, 0, 2, &isSwing);
    f6 = Edog12Dof_SampleReferenceTrotFootPoint(0.020, 0.0, 0.0, 0.006,
                                                0, 1, 0, 0, 6, &isSwing);
    f10 = Edog12Dof_SampleReferenceTrotFootPoint(0.020, 0.0, 0.0, 0.006,
                                                 0, 1, 0, 0, 10, &isSwing);
    if (!isSwing) {
        fprintf(stderr, "frame 10 should still be shortened swing\n");
        return 1;
    }
    require_lt("mid swing lifts foot in Z-down coordinates", f6.zMm, baseZ);
    require_lt("mid swing is higher than early swing", f6.zMm, f2.zMm);
    require_lt("mid swing is higher than late swing", f6.zMm, f10.zMm);
    require_gt("cycloid swing progresses forward/back smoothly", f0.xMm, f6.xMm);
    require_gt("cycloid swing keeps moving through middle", f6.xMm, f10.xMm);

    f12 = Edog12Dof_SampleReferenceTrotFootPoint(0.020, 0.0, 0.0, 0.006,
                                                 0, 1, 0, 0, 12, &isSwing);
    if (isSwing) {
        fprintf(stderr, "frame 12 should be stance after shortened swing\n");
        return 1;
    }
    require_close("stance keeps neutral z", f12.zMm, baseZ, 0.001);
    {
        double stancePhase = Edog12Dof_ReferencePhaseForFrame(12, 0);
        double stanceT = (stancePhase - EDOG_12DOF_TROT_SWING_PORTION) /
            (1.0 - EDOG_12DOF_TROT_SWING_PORTION);
        require_close("stance x follows extended support phase",
                      f12.xMm,
                      baseX - bodyShift + strideMm / 2.0 - strideMm * stanceT,
                      0.001);
    }

    f29 = Edog12Dof_SampleReferenceTrotFootPoint(0.020, 0.0, 0.0, 0.006,
                                                 0, 1, 0, 0, 29, &isSwing);
    if (isSwing) {
        fprintf(stderr, "frame 29 should be stance before wrap\n");
        return 1;
    }
    require_close("late stance z neutral", f29.zMm, baseZ, 0.001);
    require_lt("late stance wraps close to next swing start", fabs(f0.xMm - f29.xMm), 2.0);

    back0 = Edog12Dof_SampleReferenceTrotFootPoint(0.020, 0.0, 0.0, 0.006,
                                                   0, 1, 0, 1, 0, &isSwing);
    require_close("reverse phase keeps forward neutral body shift",
                  back0.xMm, baseX - bodyShift - strideMm / 2.0, 0.001);
    return 0;
}
'''
    with tempfile.TemporaryDirectory() as tmp:
        stub_include = Path(tmp) / "stubs"
        src = Path(tmp) / "probe.c"
        exe = Path(tmp) / "probe"
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
        src.write_text(source, encoding="utf-8")
        cmd = [
            "gcc",
            "-std=c99",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-I",
            str(stub_include),
            "-I.",
            str(src),
            "12_DOF_Version/src/gait_generate_12dof.c",
            "-lm",
            "-o",
            str(exe),
        ]
        subprocess.run(cmd, cwd=ROOT, check=True)
        subprocess.run([str(exe)], cwd=ROOT, check=True)


def main():
    require_source_contract()
    compile_and_run_probe()
    print("12DOF Py-Apple cycloid trajectory checks passed")


if __name__ == "__main__":
    try:
        main()
    except (AssertionError, subprocess.CalledProcessError) as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        sys.exit(1)
