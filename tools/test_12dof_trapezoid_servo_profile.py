#!/usr/bin/env python3
import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "include/edog_config.h"
MOTION = ROOT / "12_DOF_Version/src/motion_utils_12dof.c"


def require(token, text, where):
    if token not in text:
        raise AssertionError(f"{where} missing {token}")


def require_define(text, name, value):
    if not re.search(rf"#define\s+{name}\s+{value}\b", text):
        raise AssertionError(f"missing #define {name} {value}")


def main():
    config = CONFIG.read_text(encoding="utf-8")
    motion = MOTION.read_text(encoding="utf-8")

    for name, value in [
        ("EDOG_SERVO_TRAPEZOID_MAX_SPEED_CENTI_PER_FRAME", 420),
        ("EDOG_SERVO_TRAPEZOID_MAX_ACCEL_CENTI_PER_FRAME2", 140),
        ("EDOG_SERVO_TRAPEZOID_TARGET_EPS_CENTI", 2),
    ]:
        require_define(config, name, value)

    for token in [
        "static int servoVelocityCentiPerFrame[EDOG_SERVO_CHANNEL_COUNT]",
        "resetServoMotionProfile(channel, limited)",
        "static int planServoTrapezoidStepCenti(int channel, int targetCenti)",
        "brakeDistance = (long long)velocityAbs * velocityAbs /",
        "EDOG_SERVO_TRAPEZOID_MAX_ACCEL_CENTI_PER_FRAME2",
        "if ((long long)distanceAbs <= brakeDistance)",
        "nextVelocity = velocity + direction * EDOG_SERVO_TRAPEZOID_MAX_ACCEL_CENTI_PER_FRAME2",
        "EDOG_SERVO_TRAPEZOID_MAX_SPEED_CENTI_PER_FRAME",
        "servoVelocityCentiPerFrame[channel] = next - current",
        "static int writeServoProfileAngleCenti(int channel, int centiAngle)",
        "return writeServoProfileAngleCenti(channel, planServoTrapezoidStepCenti(channel, target))",
    ]:
        require(token, motion, "motion_utils_12dof.c")

    forbidden = [
        "step = delta * EDOG_SERVO_GAIT_SMOOTHING_NUM / EDOG_SERVO_GAIT_SMOOTHING_DEN",
        "return writeServoPhysicalAngleCenti(channel, slewServoAngleCenti(channel, target))",
    ]
    for token in forbidden:
        if token in motion:
            raise AssertionError(f"old slew smoothing still present: {token}")

    print("12DOF trapezoid servo profile checks passed")


if __name__ == "__main__":
    try:
        main()
    except AssertionError as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        sys.exit(1)
