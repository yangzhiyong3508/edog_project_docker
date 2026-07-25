#!/usr/bin/env python3
import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MOTION = ROOT / "12_DOF_Version/src/motion_utils_12dof.c"


def body_of(name, text):
    match = re.search(
        rf"(?:static\s+)?(?:int|void)\s+{name}\s*\([^)]*\)\s*\{{(?P<body>.*?)\n\}}",
        text,
        re.S,
    )
    if not match:
        raise AssertionError(f"missing function {name}")
    return match.group("body")


def main():
    motion = MOTION.read_text(encoding="utf-8")
    configure_trot = body_of("configureTrotPhaseOffsets", motion)
    configure_mode = body_of("configureModePhaseOffsets", motion)

    required_trot_offsets = [
        "runtime->phaseOffset[0] = 0;",
        "runtime->phaseOffset[3] = 0;",
        "runtime->phaseOffset[1] = EDOG_12DOF_GAIT_FRAME_COUNT / 2;",
        "runtime->phaseOffset[2] = EDOG_12DOF_GAIT_FRAME_COUNT / 2;",
    ]
    for token in required_trot_offsets:
        if token not in configure_trot:
            raise AssertionError(f"diagonal trot phase missing token: {token}")

    if "EDOG_TABLE_MODE_TURN_LEFT" in configure_mode or "EDOG_TABLE_MODE_TURN_RIGHT" in configure_mode:
        raise AssertionError("turn modes must not override diagonal trot phase offsets")
    if "EDOG_12DOF_GAIT_FRAME_COUNT / 8" in configure_mode:
        raise AssertionError("turn modes must not add one-eighth phase staggering")
    if "EDOG_12DOF_GAIT_FRAME_COUNT * 5 / 8" in configure_mode:
        raise AssertionError("turn modes must not add five-eighths phase staggering")

    frame_count = 30
    swing_frames = {
        index for index in range(frame_count)
        if index / frame_count < 0.38
    }
    offsets = {"LF": 0, "RF": 15, "LB": 15, "RB": 0}
    if offsets["LF"] != offsets["RB"] or offsets["RF"] != offsets["LB"]:
        raise AssertionError("diagonal trot must pair LF/RB and RF/LB")
    if (offsets["RF"] - offsets["LF"]) % frame_count != frame_count // 2:
        raise AssertionError("diagonal trot phase groups must be half-cycle apart")

    for base in range(frame_count):
        swinging = {
            leg for leg, offset in offsets.items()
            if ((base + offset) % frame_count) in swing_frames
        }
        if swinging and swinging not in ({"LF", "RB"}, {"RF", "LB"}):
            raise AssertionError(
                f"turn trot must not create isolated or non-diagonal swing legs at base={base}: {sorted(swinging)}"
            )

    print("12DOF turn phase contract checks passed")


if __name__ == "__main__":
    try:
        main()
    except AssertionError as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        sys.exit(1)
