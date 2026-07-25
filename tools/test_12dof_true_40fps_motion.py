#!/usr/bin/env python3
import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "include/edog_config.h"
GAIT_HEADER = ROOT / "12_DOF_Version/include/gait_generate_12dof.h"
MOTION = ROOT / "12_DOF_Version/src/motion_utils_12dof.c"


def main():
    config = CONFIG.read_text(encoding="utf-8")
    gait_header = GAIT_HEADER.read_text(encoding="utf-8")
    motion = MOTION.read_text(encoding="utf-8")

    if "#define EDOG_12DOF_GAIT_FRAME_COUNT 30" not in gait_header:
        raise AssertionError("12DOF gait should use a 30-frame target table")
    if "#define EDOG_12DOF_GAIT_FRAME_FPS              40" not in config:
        raise AssertionError("12DOF servo update cadence should remain 40 FPS for smooth interpolation")
    if "#define EDOG_SERVO_MOTION_STEP_DELAY_US        0" not in config:
        raise AssertionError("motion gait frames must not add per-joint delay between servo writes")
    if "#define EDOG_SERVO_STARTUP_STEP_DELAY_US       120000" not in config:
        raise AssertionError("startup stagger delay should remain for power safety")
    if "#define EDOG_SERVO_STOP_STEP_DELAY_US          6000" not in config:
        raise AssertionError("stop stagger delay should remain for power safety")

    body = re.search(
        r"static\s+void\s+apply8DofTableFrameStaggered\s*\([^)]*\)\s*\{(?P<body>.*?)\n\}",
        motion,
        re.S,
    )
    if not body:
        raise AssertionError("missing apply8DofTableFrameStaggered")
    if "EDOG_SERVO_MOTION_STEP_DELAY_US" not in body.group("body"):
        raise AssertionError("whole-body trot should use the shared motion delay macro")

    print("12DOF 30-frame target cadence checks passed")


if __name__ == "__main__":
    try:
        main()
    except AssertionError as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        sys.exit(1)
