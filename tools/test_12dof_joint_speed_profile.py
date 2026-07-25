#!/usr/bin/env python3
import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "include/edog_config.h"
MOTION = ROOT / "12_DOF_Version/src/motion_utils_12dof.c"


def require(token, text, label):
    if token not in text:
        raise AssertionError(f"{label} missing token: {token}")


def require_pattern(pattern, text, label):
    if not re.search(pattern, text, re.S):
        raise AssertionError(f"{label} missing pattern: {pattern}")


def main():
    config = CONFIG.read_text(encoding="utf-8")
    motion = MOTION.read_text(encoding="utf-8")

    for token in [
        "#define EDOG_SERVO_HIP_SPEED_60_DEG_MS        330",
        "#define EDOG_SERVO_LEG_SPEED_60_DEG_MS        250",
        "#define EDOG_SERVO_SPEED_SAFETY_NUM            8",
        "#define EDOG_SERVO_SPEED_SAFETY_DEN            10",
    ]:
        require(token, config, "config")

    for token in [
        "static int isHipServoChannel(int channel)",
        "channel == LF_HIP || channel == RF_HIP ||",
        "channel == LB_HIP || channel == RB_HIP",
        "static int getServoSpeed60DegMsForChannel(int channel)",
        "return isHipServoChannel(channel) ? EDOG_SERVO_HIP_SPEED_60_DEG_MS :",
        "EDOG_SERVO_LEG_SPEED_60_DEG_MS",
        "static int getSafeServoCentiDegPerMs(int channel)",
    ]:
        require(token, motion, "motion")

    require_pattern(
        r"getSafeServoCentiDegPerMs\s*\(\s*channel\s*\)",
        motion,
        "motion",
    )
    if "EDOG_SERVO_SPEED_60_DEG_MS" in config or "EDOG_SERVO_SPEED_60_DEG_MS" in motion:
        raise AssertionError("uniform servo speed macro must be replaced by joint-specific speed macros")

    print("12DOF joint-specific servo speed profile checks passed")


if __name__ == "__main__":
    try:
        main()
    except AssertionError as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        sys.exit(1)
