#!/usr/bin/env python3
import re
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


def require_define(text, name, value):
    if not re.search(rf"#define\s+{name}\s+{value}\b", text):
        raise AssertionError(f"missing #define {name} {value}")


def require_source_contract():
    config = CONFIG.read_text(encoding="utf-8")
    gait = GAIT_SRC.read_text(encoding="utf-8")

    require_define(config, "EDOG_12DOF_TROT_SWING_PORTION", "0.38")
    for token in [
        "EDOG_12DOF_TROT_SWING_PORTION",
        "double swingPortion = EDOG_12DOF_TROT_SWING_PORTION",
        "phase < swingPortion",
        "phase / swingPortion",
        "double stancePortion = 1.0 - swingPortion",
        "(phase - swingPortion) / stancePortion",
    ]:
        require(token, gait, "gait_generate_12dof.c")

    for forbidden in [
        "phase < 0.5",
        "phase / 0.5",
        "(phase - 0.5) / 0.5",
    ]:
        if forbidden in gait:
            raise AssertionError(f"trot phase split must not keep hard-coded 50/50 token: {forbidden}")


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

static int count_swing_frames(int isRightLeg, int isFrontLeg, int reversed)
{
    int swingFrames = 0;

    for (int frame = 0; frame < EDOG_12DOF_TROT_FRAME_COUNT; frame++) {
        int isSwing = 0;
        (void)Edog12Dof_SampleReferenceTrotFootPoint(
            0.020, 0.0, 0.0, 0.010,
            isRightLeg, isFrontLeg,
            isFrontLeg ? (isRightLeg ? 1 : 0) : (isRightLeg ? 3 : 2),
            reversed, frame, &isSwing);
        if (isSwing) {
            swingFrames++;
        }
    }
    return swingFrames;
}

static void require_trot_support_contract(int isRightLeg, int isFrontLeg, int reversed)
{
    int swingFrames = count_swing_frames(isRightLeg, isFrontLeg, reversed);
    int stanceFrames = EDOG_12DOF_TROT_FRAME_COUNT - swingFrames;

    if (swingFrames < 11 || swingFrames > 12) {
        fprintf(stderr, "expected 11-12 swing frames, got %d\n", swingFrames);
        exit(1);
    }
    if (stanceFrames < 18 || stanceFrames > 19) {
        fprintf(stderr, "expected 18-19 stance frames, got %d\n", stanceFrames);
        exit(1);
    }
    if (stanceFrames <= swingFrames) {
        fprintf(stderr, "stance must be longer than swing, stance=%d swing=%d\n",
                stanceFrames, swingFrames);
        exit(1);
    }
}

int main(void)
{
    int isSwing = 0;
    Edog12DofFootPoint swingEnd;
    Edog12DofFootPoint stanceStart;
    double baseZ = Edog12Dof_DefaultFootZForLeg(1);

    if (EDOG_12DOF_TROT_FRAME_COUNT != 30) {
        fprintf(stderr, "expected 30-frame target table\n");
        return 1;
    }
    require_close("trot swing portion", EDOG_12DOF_TROT_SWING_PORTION, 0.38, 0.000001);
    if (!(EDOG_12DOF_TROT_SWING_PORTION < 0.5)) {
        fprintf(stderr, "trot swing portion must be shorter than half-cycle\n");
        return 1;
    }
    require_close("lift start", Edog12Dof_ReferenceSwingEnvelopeForPhase(0.0), 0.0, 0.000001);
    require_close("lift middle",
                  Edog12Dof_ReferenceSwingEnvelopeForPhase(EDOG_12DOF_TROT_SWING_PORTION / 2.0),
                  1.0, 0.000001);
    require_close("lift touchdown",
                  Edog12Dof_ReferenceSwingEnvelopeForPhase(EDOG_12DOF_TROT_SWING_PORTION),
                  0.0, 0.000001);
    require_close("lift stance",
                  Edog12Dof_ReferenceSwingEnvelopeForPhase(0.75),
                  0.0, 0.000001);

    for (int right = 0; right <= 1; right++) {
        for (int front = 0; front <= 1; front++) {
            require_trot_support_contract(right, front, 0);
            require_trot_support_contract(right, front, 1);
        }
    }

    swingEnd = Edog12Dof_SampleReferenceTrotFootPoint(0.020, 0.0, 0.0, 0.010,
                                                      0, 1, 0, 0, 11, &isSwing);
    if (!isSwing) {
        fprintf(stderr, "frame 11 should be the last shortened swing frame\n");
        return 1;
    }
    if (!(swingEnd.zMm < baseZ)) {
        fprintf(stderr, "late swing should still lift before touchdown\n");
        return 1;
    }

    stanceStart = Edog12Dof_SampleReferenceTrotFootPoint(0.020, 0.0, 0.0, 0.010,
                                                         0, 1, 0, 0, 12, &isSwing);
    if (isSwing) {
        fprintf(stderr, "frame 12 should be stance after shortened swing\n");
        return 1;
    }
    require_close("stance z neutral", stanceStart.zMm, baseZ, 0.001);

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
    print("12DOF trot shortened swing and longer support phase checks passed")


if __name__ == "__main__":
    try:
        main()
    except (AssertionError, subprocess.CalledProcessError) as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        sys.exit(1)
