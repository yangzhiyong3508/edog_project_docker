#!/usr/bin/env python3
import subprocess
import sys
import tempfile
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
GAIT_SRC = ROOT / "12_DOF_Version/src/gait_generate_12dof.c"


def require_source_contract():
    gait = GAIT_SRC.read_text(encoding="utf-8")
    forbidden = "gaitTable[EDOG_12DOF_TROT_FRAME_COUNT - 1] = gaitTable[0]"
    if forbidden in gait:
        raise AssertionError("trot gait table must not duplicate frame 0 at the last frame")


def compile_and_run_probe():
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

static void require_close(const char *name, double got, double expected)
{
    double diff = fabs(got - expected);
    if (diff > 0.000001) {
        fprintf(stderr, "%s expected %.6f, got %.6f\n", name, expected, got);
        exit(1);
    }
}

int main(void)
{
    Edog12DofJointAngles gait[EDOG_12DOF_TROT_FRAME_COUNT];
    int wrapDelta;
    int sameLastFirst;

    if (EDOG_12DOF_TROT_FRAME_COUNT != 30) {
        fprintf(stderr, "expected 30-frame 12DOF gait target table\n");
        return 1;
    }

    require_close("phase fwd 0", Edog12Dof_ReferencePhaseForFrame(0, 0), 0.0 / 30.0);
    require_close("phase fwd 1", Edog12Dof_ReferencePhaseForFrame(1, 0), 1.0 / 30.0);
    require_close("phase fwd 2", Edog12Dof_ReferencePhaseForFrame(2, 0), 2.0 / 30.0);
    require_close("phase fwd 28", Edog12Dof_ReferencePhaseForFrame(28, 0), 28.0 / 30.0);
    require_close("phase fwd 29", Edog12Dof_ReferencePhaseForFrame(29, 0), 29.0 / 30.0);
    require_close("phase rev 0", Edog12Dof_ReferencePhaseForFrame(0, 1), 0.0 / 30.0);
    require_close("phase rev 1", Edog12Dof_ReferencePhaseForFrame(1, 1), 29.0 / 30.0);
    require_close("phase rev 2", Edog12Dof_ReferencePhaseForFrame(2, 1), 28.0 / 30.0);
    require_close("phase rev 28", Edog12Dof_ReferencePhaseForFrame(28, 1), 2.0 / 30.0);
    require_close("phase rev 29", Edog12Dof_ReferencePhaseForFrame(29, 1), 1.0 / 30.0);

    Edog12Dof_GenerateDirectionalTrotTable(gait, 0.030, 0.0, 0.0, 0.006, 0, 1, 0);
    sameLastFirst =
        gait[EDOG_12DOF_TROT_FRAME_COUNT - 1].hipAngleCentiDeg == gait[0].hipAngleCentiDeg &&
        gait[EDOG_12DOF_TROT_FRAME_COUNT - 1].femurAngleCentiDeg == gait[0].femurAngleCentiDeg &&
        gait[EDOG_12DOF_TROT_FRAME_COUNT - 1].tibiaAngleCentiDeg == gait[0].tibiaAngleCentiDeg;
    if (sameLastFirst) {
        fprintf(stderr, "last frame must not repeat first frame during continuous trot\n");
        return 1;
    }

    wrapDelta = max3(
        abs_int(gait[0].hipAngleCentiDeg - gait[EDOG_12DOF_TROT_FRAME_COUNT - 1].hipAngleCentiDeg),
        abs_int(gait[0].femurAngleCentiDeg - gait[EDOG_12DOF_TROT_FRAME_COUNT - 1].femurAngleCentiDeg),
        abs_int(gait[0].tibiaAngleCentiDeg - gait[EDOG_12DOF_TROT_FRAME_COUNT - 1].tibiaAngleCentiDeg));
    if (wrapDelta > EDOG_12DOF_GAIT_MAX_JOINT_STEP_CENTI) {
        fprintf(stderr, "wrap delta too large for continuous trot: %d centi-deg\n", wrapDelta);
        return 1;
    }

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
    print("12DOF continuous trot wrap checks passed")


if __name__ == "__main__":
    try:
        main()
    except (AssertionError, subprocess.CalledProcessError) as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        sys.exit(1)
