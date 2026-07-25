#!/usr/bin/env python3
import re
import subprocess
import sys
import tempfile
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "include/edog_config.h"
GAIT_HEADER = ROOT / "12_DOF_Version/include/gait_generate_12dof.h"
SERVO = ROOT / "utils/src/servo_control.c"
MOTION = ROOT / "12_DOF_Version/src/motion_utils_12dof.c"


def require(token, text, where):
    if token not in text:
        raise AssertionError(f"{where} missing {token}")


def require_define(text, name, value):
    if not re.search(rf"#define\s+{name}\s+{value}\b", text):
        raise AssertionError(f"missing #define {name} {value}")


def compile_and_run_probe():
    source = r'''
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

static int countLargeTargetSteps(int isRightLeg, int isFrontLeg, int *maxDeltaOut)
{
    Edog12DofJointAngles gait[EDOG_12DOF_TROT_FRAME_COUNT];
    int maxDelta = 0;
    int largeTargetSteps = 0;
    int wrapDelta;

    Edog12Dof_GenerateDirectionalTrotTable(gait, 0.030, 0.0, 0.0, 0.006, isRightLeg, isFrontLeg, 0);
    for (int i = 0; i < EDOG_12DOF_TROT_FRAME_COUNT; i++) {
        int next = (i + 1) % EDOG_12DOF_TROT_FRAME_COUNT;
        int delta = max3(
            abs_int(gait[next].hipAngleCentiDeg - gait[i].hipAngleCentiDeg),
            abs_int(gait[next].femurAngleCentiDeg - gait[i].femurAngleCentiDeg),
            abs_int(gait[next].tibiaAngleCentiDeg - gait[i].tibiaAngleCentiDeg));
        if (delta > maxDelta) {
            maxDelta = delta;
        }
        if (delta >= 280) {
            largeTargetSteps++;
        }
    }

    if (maxDelta > EDOG_12DOF_GAIT_MAX_JOINT_STEP_CENTI) {
        fprintf(stderr, "target gait exceeded configured joint step limit: %d\n", maxDelta);
        return -1;
    }

    wrapDelta = max3(
        abs_int(gait[0].hipAngleCentiDeg - gait[EDOG_12DOF_TROT_FRAME_COUNT - 1].hipAngleCentiDeg),
        abs_int(gait[0].femurAngleCentiDeg - gait[EDOG_12DOF_TROT_FRAME_COUNT - 1].femurAngleCentiDeg),
        abs_int(gait[0].tibiaAngleCentiDeg - gait[EDOG_12DOF_TROT_FRAME_COUNT - 1].tibiaAngleCentiDeg));
    if (wrapDelta > EDOG_12DOF_GAIT_MAX_JOINT_STEP_CENTI) {
        fprintf(stderr, "wrap delta too large: %d\n", wrapDelta);
        return -1;
    }

    if (maxDeltaOut != NULL) {
        *maxDeltaOut = maxDelta;
    }
    return largeTargetSteps;
}

int main(void)
{
    int frontMaxDelta = 0;
    int rearMaxDelta = 0;
    int frontLargeSteps;
    int rearLargeSteps;

    if (EDOG_12DOF_TROT_FRAME_COUNT != 30) {
        fprintf(stderr, "expected 30 target frames, got %d\n", EDOG_12DOF_TROT_FRAME_COUNT);
        return 1;
    }
    if (EDOG_12DOF_GAIT_MAX_JOINT_STEP_CENTI < 280) {
        fprintf(stderr, "target-table joint step limit must allow visible target changes near 3deg\n");
        return 1;
    }

    frontLargeSteps = countLargeTargetSteps(0, 1, &frontMaxDelta);
    rearLargeSteps = countLargeTargetSteps(0, 0, &rearMaxDelta);

    if (frontLargeSteps < 1 || rearLargeSteps < 2) {
        fprintf(stderr,
                "same-height 30-frame target gait should keep visible >=2.8deg target changes, front=%d rear=%d\n",
                frontLargeSteps, rearLargeSteps);
        return 1;
    }
    if (frontMaxDelta < 280 || rearMaxDelta < 300) {
        fprintf(stderr, "same-height target gait max deltas too small, front=%d rear=%d\n",
                frontMaxDelta, rearMaxDelta);
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
    config = CONFIG.read_text(encoding="utf-8")
    gait_header = GAIT_HEADER.read_text(encoding="utf-8")
    servo = SERVO.read_text(encoding="utf-8")
    motion = MOTION.read_text(encoding="utf-8")

    require_define(gait_header, "EDOG_12DOF_GAIT_FRAME_COUNT", 30)
    require_define(config, "EDOG_12DOF_GAIT_FRAME_FPS", 50)
    require_define(config, "EDOG_12DOF_GAIT_MAX_JOINT_STEP_CENTI", 360)
    require_define(config, "EDOG_SERVO_MOTION_STEP_DELAY_US", 0)
    require(
        "EDOG_12DOF_GAIT_FRAME_PERIOD_US        (1000000 / EDOG_12DOF_GAIT_FRAME_FPS)",
        config,
        "edog_config.h",
    )

    for token in [
        "motionSetServoSmoothCenti(leg->hip, angles->hipAngleCentiDeg)",
        "motionSetServoSmoothCenti(leg->thigh, angles->femurAngleCentiDeg)",
        "motionSetServoSmoothCenti(leg->calf, angles->tibiaAngleCentiDeg)",
        "static int planServoTrapezoidStepCenti(int channel, int targetCenti)",
        "servoVelocityCentiPerFrame[channel] = next - current",
        "EDOG_SERVO_TRAPEZOID_TARGET_EPS_CENTI",
        "writeServoProfileAngleCenti(channel, planServoTrapezoidStepCenti(channel, target))",
    ]:
        require(token, motion, "motion_utils_12dof.c")

    for token in [
        "static int centiDegToPwmCount(int centiDeg)",
        "const long long rangeCenti = (long long)EDOG_SERVO_ANGLE_RANGE_DEG * 100",
        "return setServoPwmCount(channel, count)",
        "int setServoPulseUs(int channel, int pulseUs)",
    ]:
        require(token, servo, "servo_control.c")

    if "pulseUs = EDOG_SERVO_PULSE_MIN_US +" in servo:
        raise AssertionError("setServoCentiDeg must not truncate centi-degree angle to integer microseconds")
    if "return setServoPulseUs(channel, pulseUs);" in servo:
        raise AssertionError("setServoCentiDeg must send direct rounded PWM counts")

    compile_and_run_probe()
    print("12DOF 30-frame target and smooth servo interpolation checks passed")


if __name__ == "__main__":
    try:
        main()
    except (AssertionError, subprocess.CalledProcessError) as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        sys.exit(1)
