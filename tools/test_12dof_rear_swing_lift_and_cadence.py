#!/usr/bin/env python3
import re
import subprocess
import sys
import tempfile
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "include/edog_config.h"
GAIT_SRC = ROOT / "12_DOF_Version/src/gait_generate_12dof.c"


def require_define(text, name, value):
    if not re.search(rf"#define\s+{name}\s+{re.escape(value)}\b", text):
        raise AssertionError(f"missing #define {name} {value}")


def require_source_contract():
    config = CONFIG.read_text(encoding="utf-8")
    gait = GAIT_SRC.read_text(encoding="utf-8")
    require_define(config, "EDOG_12DOF_GAIT_FRAME_FPS", "50")
    require_define(config, "EDOG_12DOF_REAR_SWING_LIFT_EXTRA_MM", "6.0")
    for token in [
        "EDOG_12DOF_REAR_SWING_LIFT_EXTRA_MM",
        "!isFrontLeg",
        "defaultSwingFoot(&swingStart, &swingEnd, effectiveLiftMm, legIndex, swingT)",
    ]:
        if token not in gait:
            raise AssertionError(f"rear swing lift source contract missing {token}")


def compile_and_run_probe():
    source = r'''
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "include/edog_config.h"
#include "12_DOF_Version/include/gait_generate_12dof.h"

#ifndef EDOG_12DOF_REAR_SWING_LIFT_EXTRA_MM
#error EDOG_12DOF_REAR_SWING_LIFT_EXTRA_MM must be defined
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

static void require_lt(const char *name, double lhs, double rhs)
{
    if (!(lhs < rhs)) {
        fprintf(stderr, "%s expected %.6f < %.6f\n", name, lhs, rhs);
        exit(1);
    }
}

int main(void)
{
    int isSwingFront = 0;
    int isSwingRear = 0;
    double baseFrontZ;
    double baseRearZ;
    double midEnvelope;
    Edog12DofFootPoint frontStart;
    Edog12DofFootPoint rearStart;
    Edog12DofFootPoint frontMid;
    Edog12DofFootPoint rearMid;
    Edog12DofFootPoint frontStance;
    Edog12DofFootPoint rearStance;

    if (EDOG_12DOF_GAIT_FRAME_COUNT != 30) {
        fprintf(stderr, "expected 30 target frames, got %d\n", EDOG_12DOF_GAIT_FRAME_COUNT);
        return 1;
    }
    if (EDOG_12DOF_GAIT_FRAME_FPS != 50) {
        fprintf(stderr, "expected 50FPS gait cadence, got %d\n", EDOG_12DOF_GAIT_FRAME_FPS);
        return 1;
    }
    if (EDOG_12DOF_GAIT_FRAME_PERIOD_US != 20000) {
        fprintf(stderr, "expected 20ms frame period at 50FPS, got %d\n", EDOG_12DOF_GAIT_FRAME_PERIOD_US);
        return 1;
    }
    if (EDOG_12DOF_GAIT_FRAME_COUNT * EDOG_12DOF_GAIT_FRAME_PERIOD_US != 600000) {
        fprintf(stderr, "expected 600ms 30-frame cycle at 50FPS\n");
        return 1;
    }
    require_close("rear extra lift macro", EDOG_12DOF_REAR_SWING_LIFT_EXTRA_MM, 6.0, 0.000001);

    baseFrontZ = Edog12Dof_DefaultFootZForLeg(1);
    baseRearZ = Edog12Dof_DefaultFootZForLeg(0);
    require_close("same neutral front/rear z", baseRearZ, baseFrontZ, 0.001);

    frontStart = Edog12Dof_SampleReferenceTrotFootPoint(0.020, 0.0, 0.0, 0.006,
                                                        0, 1, 0, 0, 0, &isSwingFront);
    rearStart = Edog12Dof_SampleReferenceTrotFootPoint(0.020, 0.0, 0.0, 0.006,
                                                       0, 0, 2, 0, 0, &isSwingRear);
    if (!isSwingFront || !isSwingRear) {
        fprintf(stderr, "frame 0 should be swing start for both sampled legs\n");
        return 1;
    }
    require_close("front swing boundary neutral", frontStart.zMm, baseFrontZ, 0.001);
    require_close("rear swing boundary has no extra lift", rearStart.zMm, baseRearZ, 0.001);

    frontMid = Edog12Dof_SampleReferenceTrotFootPoint(0.020, 0.0, 0.0, 0.006,
                                                      0, 1, 0, 0, 8, &isSwingFront);
    rearMid = Edog12Dof_SampleReferenceTrotFootPoint(0.020, 0.0, 0.0, 0.006,
                                                     0, 0, 2, 0, 8, &isSwingRear);
    if (!isSwingFront || !isSwingRear) {
        fprintf(stderr, "frame 8 should be swing for both sampled legs\n");
        return 1;
    }
    midEnvelope = Edog12Dof_ReferenceSwingEnvelopeForPhase(8.0 / 30.0);
    require_lt("rear mid swing should lift higher than front", rearMid.zMm, frontMid.zMm);
    require_close("rear extra lift follows swing envelope",
                  frontMid.zMm - rearMid.zMm,
                  EDOG_12DOF_REAR_SWING_LIFT_EXTRA_MM * midEnvelope,
                  0.01);

    frontStance = Edog12Dof_SampleReferenceTrotFootPoint(0.020, 0.0, 0.0, 0.006,
                                                         0, 1, 0, 0, 15, &isSwingFront);
    rearStance = Edog12Dof_SampleReferenceTrotFootPoint(0.020, 0.0, 0.0, 0.006,
                                                        0, 0, 2, 0, 15, &isSwingRear);
    if (isSwingFront || isSwingRear) {
        fprintf(stderr, "frame 15 should be stance for both sampled legs\n");
        return 1;
    }
    require_close("front stance neutral", frontStance.zMm, baseFrontZ, 0.001);
    require_close("rear stance neutral", rearStance.zMm, baseRearZ, 0.001);
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
    print("12DOF rear swing lift and 50FPS cadence checks passed")


if __name__ == "__main__":
    try:
        main()
    except (AssertionError, subprocess.CalledProcessError) as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        sys.exit(1)
