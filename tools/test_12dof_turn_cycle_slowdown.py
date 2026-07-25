#!/usr/bin/env python3
import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "include/edog_config.h"
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


def require(token, text, label):
    if token not in text:
        raise AssertionError(f"{label} missing token: {token}")


def main():
    config = CONFIG.read_text(encoding="utf-8")
    motion = MOTION.read_text(encoding="utf-8")

    require("#define EDOG_12DOF_TURN_CYCLE_SLOWDOWN_NUM", config, "config")
    require("#define EDOG_12DOF_TURN_CYCLE_SLOWDOWN_DEN", config, "config")

    start_runtime = body_of("startServoTableRuntime", motion)
    require("isTurnTableMode(mode)", motion, "motion")
    require("EDOG_12DOF_TURN_CYCLE_SLOWDOWN_NUM", start_runtime, "startServoTableRuntime")
    require("EDOG_12DOF_TURN_CYCLE_SLOWDOWN_DEN", start_runtime, "startServoTableRuntime")
    require("if (isTurnTableMode(mode))", start_runtime, "startServoTableRuntime")

    if not re.search(
        r"requestedCycleMs\s*=\s*\(requestedCycleMs\s*\*\s*EDOG_12DOF_TURN_CYCLE_SLOWDOWN_NUM\s*\+\s*EDOG_12DOF_TURN_CYCLE_SLOWDOWN_DEN\s*-\s*1\)\s*/\s*EDOG_12DOF_TURN_CYCLE_SLOWDOWN_DEN\s*;",
        start_runtime,
    ):
        raise AssertionError("turn cycle slowdown must round up requested cycle ms")

    print("12DOF turn cycle slowdown checks passed")


if __name__ == "__main__":
    try:
        main()
    except AssertionError as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        sys.exit(1)
