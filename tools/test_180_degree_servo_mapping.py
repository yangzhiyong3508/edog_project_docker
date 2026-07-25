#!/usr/bin/env python3
import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "include/edog_config.h"
SERVO = ROOT / "utils/src/servo_control.c"
SERVO_HEADER = ROOT / "utils/include/servo_control.h"
MOTION = ROOT / "12_DOF_Version/src/motion_utils_12dof.c"
MOTION_HEADER = ROOT / "12_DOF_Version/include/motion_utils_12dof.h"
GAIT = ROOT / "12_DOF_Version/src/gait_generate_12dof.c"
GAIT_HEADER = ROOT / "12_DOF_Version/include/gait_generate_12dof.h"


def require_define(text, name, value):
    if not re.search(rf"#define\s+{name}\s+{value}\b", text):
        raise AssertionError(f"missing #define {name} {value}")


def require(token, text, where):
    if token not in text:
        raise AssertionError(f"{where} missing {token}")


def forbid(token, text, where):
    if token in text:
        raise AssertionError(f"{where} still contains legacy 270/135-degree token {token}")


def forbid_legacy_header_angle(text, where):
    if "270" in text:
        raise AssertionError(f"{where} still contains legacy 270-degree token 270")
    allowed_lengths = [
        r"#define\s+SPOTMICRO_CALF_LENGTH_MM\s+135\.0\b",
        r"#define\s+EDOG_12DOF_L3_MM\s+135\.0\b",
    ]
    scrubbed = text
    for pattern in allowed_lengths:
        scrubbed = re.sub(pattern, "", scrubbed)
    if "135" in scrubbed:
        raise AssertionError(f"{where} still contains legacy 135-degree servo-center token 135")


def main():
    config = CONFIG.read_text(encoding="utf-8")
    servo = SERVO.read_text(encoding="utf-8")
    servo_header = SERVO_HEADER.read_text(encoding="utf-8")
    motion = MOTION.read_text(encoding="utf-8")
    motion_header = MOTION_HEADER.read_text(encoding="utf-8")
    gait = GAIT.read_text(encoding="utf-8")
    gait_header = GAIT_HEADER.read_text(encoding="utf-8")

    require_define(config, "EDOG_SERVO_ANGLE_RANGE_DEG", 180)
    require_define(config, "EDOG_SERVO_MIN_ANGLE", 0)
    require_define(config, "EDOG_SERVO_MAX_ANGLE", 180)
    require_define(config, "EDOG_SERVO_CENTER_ANGLE", 90)
    require_define(config, "EDOG_SERVO_PULSE_MIN_US", 500)
    require_define(config, "EDOG_SERVO_PULSE_MAX_US", 2500)

    for token in [
        "EDOG_SERVO_ANGLE_RANGE_DEG",
        "EDOG_SERVO_PULSE_MIN_US",
        "EDOG_SERVO_PULSE_MAX_US",
        "static int centiDegToPwmCount(int centiDeg)",
        "return setServoPwmCount(channel, count)",
        "const long long rangeCenti = (long long)EDOG_SERVO_ANGLE_RANGE_DEG * 100",
    ]:
        require(token, servo, "servo_control.c")

    for token in [
        "if (angle > 90)",
        "return 90",
        "if (angle < -90)",
        "return -90",
        "if (centiAngle > 9000)",
        "return 9000",
        "if (centiAngle < -9000)",
        "return -9000",
    ]:
        require(token, motion + gait, "12DOF motion/gait clamp")

    for token in [
        "EDOG_SERVO_CENTER_ANGLE + g_servoDirection[channel] * jointDeltaDeg",
        "EDOG_SERVO_CENTER_ANGLE * EDOG_12DOF_CENTI_PER_DEG",
    ]:
        require(token, motion, "motion_utils_12dof.c")

    for path_name, text in [
        ("utils/include/servo_control.h", servo_header),
        ("12_DOF_Version/include/motion_utils_12dof.h", motion_header),
        ("12_DOF_Version/include/gait_generate_12dof.h", gait_header),
    ]:
        forbid_legacy_header_angle(text, path_name)

    print("180-degree servo mapping with 0-180 range checks passed")


if __name__ == "__main__":
    try:
        main()
    except AssertionError as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        sys.exit(1)
