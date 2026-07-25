#!/usr/bin/env python3
import subprocess
import sys
import tempfile
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
GAIT_SRC = ROOT / "12_DOF_Version/src/gait_generate_12dof.c"


def require_source_contract():
    gait = GAIT_SRC.read_text(encoding="utf-8")
    if "bodyShiftSign = forwardStepM < 0.0 ? -1.0 : 1.0" not in gait:
        raise AssertionError("body X shift must follow command direction, not reversed phase flag")


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

static void compare_foot_reverse(int isRightLeg, int isFrontLeg, int legIndex)
{
    for (int k = 0; k < EDOG_12DOF_TROT_FRAME_COUNT; k++) {
        int forwardIndex = (EDOG_12DOF_TROT_FRAME_COUNT - k) % EDOG_12DOF_TROT_FRAME_COUNT;
        int forwardSwing = -1;
        int backwardSwing = -1;
        Edog12DofFootPoint forward = Edog12Dof_SampleReferenceTrotFootPoint(
            0.020, 0.0, 0.0, 0.006, isRightLeg, isFrontLeg, legIndex,
            0, forwardIndex, &forwardSwing);
        Edog12DofFootPoint backward = Edog12Dof_SampleReferenceTrotFootPoint(
            0.020, 0.0, 0.0, 0.006, isRightLeg, isFrontLeg, legIndex,
            1, k, &backwardSwing);

        if (forwardSwing != backwardSwing) {
            fprintf(stderr, "leg %d frame %d swing mismatch: forward[%d]=%d backward=%d\n",
                    legIndex, k, forwardIndex, forwardSwing, backwardSwing);
            exit(1);
        }
        require_close("reverse foot x", backward.xMm, forward.xMm, 0.001);
        require_close("reverse foot y", backward.yMm, forward.yMm, 0.001);
        require_close("reverse foot z", backward.zMm, forward.zMm, 0.001);
    }
}

static void compare_joint_reverse(int isRightLeg, int isFrontLeg)
{
    Edog12DofJointAngles forward[EDOG_12DOF_TROT_FRAME_COUNT];
    Edog12DofJointAngles backward[EDOG_12DOF_TROT_FRAME_COUNT];

    Edog12Dof_GenerateDirectionalTrotTable(forward, 0.020, 0.0, 0.0, 0.006,
                                           isRightLeg, isFrontLeg, 0);
    Edog12Dof_GenerateDirectionalTrotTable(backward, 0.020, 0.0, 0.0, 0.006,
                                           isRightLeg, isFrontLeg, 1);

    for (int k = 0; k < EDOG_12DOF_TROT_FRAME_COUNT; k++) {
        int forwardIndex = (EDOG_12DOF_TROT_FRAME_COUNT - k) % EDOG_12DOF_TROT_FRAME_COUNT;
        if (backward[k].hipAngleCentiDeg != forward[forwardIndex].hipAngleCentiDeg ||
            backward[k].femurAngleCentiDeg != forward[forwardIndex].femurAngleCentiDeg ||
            backward[k].tibiaAngleCentiDeg != forward[forwardIndex].tibiaAngleCentiDeg) {
            fprintf(stderr,
                    "joint reverse mismatch leg right=%d front=%d frame=%d forwardIndex=%d: "
                    "back=(%d,%d,%d) fwd=(%d,%d,%d)\n",
                    isRightLeg, isFrontLeg, k, forwardIndex,
                    backward[k].hipAngleCentiDeg,
                    backward[k].femurAngleCentiDeg,
                    backward[k].tibiaAngleCentiDeg,
                    forward[forwardIndex].hipAngleCentiDeg,
                    forward[forwardIndex].femurAngleCentiDeg,
                    forward[forwardIndex].tibiaAngleCentiDeg);
            exit(1);
        }
    }
}

int main(void)
{
    compare_foot_reverse(0, 1, 0);
    compare_foot_reverse(1, 1, 1);
    compare_foot_reverse(0, 0, 2);
    compare_foot_reverse(1, 0, 3);
    compare_joint_reverse(0, 1);
    compare_joint_reverse(1, 1);
    compare_joint_reverse(0, 0);
    compare_joint_reverse(1, 0);
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
        subprocess.run(
            [
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
            ],
            cwd=ROOT,
            check=True,
        )
        subprocess.run([str(exe)], cwd=ROOT, check=True)


def main():
    require_source_contract()
    compile_and_run_probe()
    print("12DOF backward-as-forward-reverse gait checks passed")


if __name__ == "__main__":
    try:
        main()
    except (AssertionError, subprocess.CalledProcessError) as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        sys.exit(1)
