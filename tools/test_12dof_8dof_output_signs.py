#!/usr/bin/env python3
import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MOTION = ROOT / "12_DOF_Version/src/motion_utils_12dof.c"

EXPECTED_PHYSICAL_DIRECTIONS = {
    0: 1,
    1: -1,
    2: -1,
    4: -1,
    5: 1,
    6: 1,
    8: 1,
    9: -1,
    10: -1,
    12: -1,
    13: 1,
    14: 1,
}


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
    motion = MOTION.read_text(encoding="utf-8")
    directions = read_direction_table(motion)

    for channel, expected in EXPECTED_PHYSICAL_DIRECTIONS.items():
        actual = directions[channel]
        if actual != expected:
            raise AssertionError(
                f"channel {channel} physical direction must remain {expected} so standing pose is not flipped, got {actual}"
            )

    for channel in (3, 7, 11, 15):
        if directions[channel] != 0:
            raise AssertionError(f"reserved channel {channel} direction must stay 0")

    for token in [
        "static void apply8DofMotionOutputSign",
        "static int setGaitLegAnglesStaggeredSmooth",
        "g_servoDirection 表已完整处理所有腿的物理方向镜像",
        "前后腿无符号差异，无需额外变换",
        "原 mirror(LF/RB)",
    ]:
        if token not in motion:
            raise AssertionError(f"missing gait-layer no-extra-mirror token: {token}")

    if motion.count("setGaitLegAnglesStaggeredSmooth(") < 10:
        raise AssertionError("gait outputs must use setGaitLegAnglesStaggeredSmooth instead of raw standing output")
    if "balance_stand_frame" not in motion or "setLegAnglesStaggeredSmooth(&g_legs[leg], &frame[leg]" not in motion:
        raise AssertionError("standing balance path must keep the raw physical output path")

    print("12DOF standing directions stay physical; gait layer avoids extra dynamic mirroring")


if __name__ == "__main__":
    try:
        main()
    except AssertionError as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        sys.exit(1)
