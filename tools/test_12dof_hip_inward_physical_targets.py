#!/usr/bin/env python3
import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "include/edog_config.h"
GAIT_HEADER = ROOT / "12_DOF_Version/include/gait_generate_12dof.h"
GAIT_SOURCE = ROOT / "12_DOF_Version/src/gait_generate_12dof.c"
MOTION = ROOT / "12_DOF_Version/src/motion_utils_12dof.c"


def require(token, text, where):
    if token not in text:
        raise AssertionError(f"{where} missing token: {token}")


def read_int_macro(text, name):
    match = re.search(rf"#define\s+{name}\s+(-?\d+)\b", text)
    if not match:
        raise AssertionError(f"missing macro {name}")
    return int(match.group(1))


def read_float_macro(text, name):
    match = re.search(rf"#define\s+{name}\s+(-?\d+(?:\.\d+)?)\b", text)
    if not match:
        raise AssertionError(f"missing macro {name}")
    return float(match.group(1))


def read_direction_table(text):
    match = re.search(
        r"static\s+const\s+int\s+g_servoDirection\s*\[\s*EDOG_SERVO_CHANNEL_COUNT\s*\]\s*=\s*\{(?P<body>.*?)\};",
        text,
        re.S,
    )
    if not match:
        raise AssertionError("missing g_servoDirection table")
    body = re.sub(r"/\*.*?\*/", "", match.group("body"), flags=re.S)
    values = [int(value) for value in re.findall(r"[-+]?\d+", body)]
    if len(values) != 16:
        raise AssertionError(f"g_servoDirection must contain 16 values, got {len(values)}")
    return values


def main():
    config = CONFIG.read_text(encoding="utf-8")
    gait_header = GAIT_HEADER.read_text(encoding="utf-8")
    gait_source = GAIT_SOURCE.read_text(encoding="utf-8")
    motion = MOTION.read_text(encoding="utf-8")

    stand_hip = read_int_macro(gait_header, "EDOG_12DOF_STAND_HIP_DELTA_DEG")
    default_y = read_float_macro(config, "EDOG_12DOF_DEFAULT_FOOT_Y_MM")

    if stand_hip != 0:
        raise AssertionError(f"power-on hip delta must be centered at 0 deg, got {stand_hip}")
    if abs(default_y) > 0.001:
        raise AssertionError(f"default foot body-Y must be 0.0 mm for centered standing hips, got {default_y}")

    require("y = isRightLeg ? -foot->yMm : foot->yMm;", gait_source, "12DOF IK")
    if "(void)isRightLeg;" in gait_source:
        raise AssertionError("12DOF IK must use isRightLeg instead of discarding it")
    require("Edog12Dof_FootYOnHipPlaneForLeg(legIndex, foot.zMm)",
            gait_source, "table gait hip-plane foot-Y")
    if "Edog12Dof_DefaultFootYForLeg(isRightLeg ? 1 : 0)" in gait_source:
        raise AssertionError("table gait must not use fixed default foot-Y after changing foot Z")
    require("Edog12Dof_DefaultFootYForLeg(leg)",
            motion, "realtime gait default foot-Y")

    directions = read_direction_table(motion)
    expected_hip_dirs = {
        0: 1,   # LF_HIP
        4: -1,  # RF_HIP
        8: 1,   # LB_HIP
        12: -1, # RB_HIP
    }
    for channel, expected in expected_hip_dirs.items():
        if directions[channel] != expected:
            raise AssertionError(
                f"hip channel {channel} direction must stay {expected}, got {directions[channel]}"
            )

    physical_targets = {
        channel: 90 + directions[channel] * stand_hip
        for channel in expected_hip_dirs
    }
    if physical_targets != {0: 90, 4: 90, 8: 90, 12: 90}:
        raise AssertionError(f"standing hip targets must stay centered: {physical_targets}")

    print("12DOF hip centered target checks passed: LF/RF/LB/RB=90 deg")


if __name__ == "__main__":
    try:
        main()
    except AssertionError as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        sys.exit(1)
