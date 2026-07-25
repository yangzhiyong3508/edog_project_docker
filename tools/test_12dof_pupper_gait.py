#!/usr/bin/env python3
import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MOTION = ROOT / "12_DOF_Version/src/motion_utils_12dof.c"
CONFIG = ROOT / "include/edog_config.h"


def main():
    text = MOTION.read_text(encoding="utf-8")
    config = CONFIG.read_text(encoding="utf-8")

    for token in [
        "#define EDOG_PUPPER_NUM_PHASES",
        "#define EDOG_PUPPER_SWING_TICKS",
        "#define EDOG_PUPPER_STANCE_TICKS",
    ]:
        if token not in config:
            raise AssertionError(f"legacy Pupper constants should remain available: {token}")

    if "static int runRealtimeGaitCycle" in text:
        raise AssertionError("old runRealtimeGaitCycle entry name must not remain")
    if "legacyRealtimeGaitCycle" not in text:
        raise AssertionError("legacy realtime controller should be explicitly marked as legacy")

    for name in ["trot_cycle", "trot_back_cycle", "diversion_left_cycle", "diversion_right_cycle"]:
        match = re.search(rf"int\s+{name}\s*\([^)]*\)\s*\{{(?P<body>.*?)\n\}}", text, re.S)
        if not match:
            raise AssertionError(f"missing {name}")
        body = match.group("body")
        if "runServoGaitTableCycle" not in body:
            raise AssertionError(f"{name} must use servo gait table scheduler")
        if "legacyRealtimeGaitCycle" in body:
            raise AssertionError(f"{name} must not call legacy realtime controller")

    for token in [
        "table->minCycleMs = table->minFrameMs * table->frameCount",
        "g_servoTableRuntime.cycleMs = requestedCycleMs > g_servoGaitTable.minCycleMs",
        "return completedCycle ? 1 : -1",
    ]:
        if token not in text:
            raise AssertionError(f"non-blocking table scheduler missing token: {token}")

    print("12DOF Pupper-era helpers are legacy; public motion uses non-blocking tables")


if __name__ == "__main__":
    try:
        main()
    except AssertionError as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        sys.exit(1)
