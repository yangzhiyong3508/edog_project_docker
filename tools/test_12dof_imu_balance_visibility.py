#!/usr/bin/env python3
import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "include/edog_config.h"
MAIN_C = ROOT / "src/main.c"
MOTION_C = ROOT / "12_DOF_Version/src/motion_utils_12dof.c"


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def main():
    config = CONFIG.read_text(encoding="utf-8")
    main_c = MAIN_C.read_text(encoding="utf-8")
    motion_c = MOTION_C.read_text(encoding="utf-8")

    require("#define EDOG_MPU_RGB_LIGHT_TASK_ENABLED          0" in config,
            "RGB MPU task must be disabled while validating IMU self-balance")
    require("#define EDOG_12DOF_IMU_BALANCE_ENABLED          1" in config,
            "IMU damping experiment must enable the 12DOF balance path")
    require("#define EDOG_12DOF_IMU_BALANCE_DEBUG_ENABLED    0" in config,
            "IMU diagnostics must stay quiet by default")
    require("#define EDOG_12DOF_IMU_BALANCE_DEBUG_INTERVAL_FRAMES" in config,
            "IMU self-balance diagnostic interval should remain defined for future re-enable")

    task_pos = main_c.find("CreateTask(MpuMotionLightTask")
    guard_pos = main_c.rfind("#if EDOG_MPU_RGB_LIGHT_TASK_ENABLED", 0, task_pos)
    require(task_pos >= 0 and guard_pos >= 0,
            "MpuMotionLightTask creation must be guarded by EDOG_MPU_RGB_LIGHT_TASK_ENABLED")
    require("[AppInit] MpuMotionLightTask disabled" not in main_c,
            "boot must not print normal disabled-state debug logs")

    require("EDOG_IMU_BALANCE_STATUS_OK" in motion_c,
            "IMU self-balance statuses should remain available for future re-enable")
    require("EDOG_IMU_BALANCE_STATUS_INIT_FAIL" in motion_c,
            "diagnostics must distinguish MPU init failures")
    require("EDOG_IMU_BALANCE_STATUS_READ_FAIL" in motion_c,
            "diagnostics must distinguish accel read failures")
    require("EDOG_IMU_BALANCE_STATUS_TILT_REJECT" in motion_c,
            "diagnostics must distinguish rejected accel vectors")
    require("gyroCentiDps=" not in motion_c,
            "periodic IMU debug strings must not be compiled by default")
    require(not re.search(r"footZ=.*LF.*RF.*LB.*RB", motion_c, re.S),
            "per-leg IMU debug strings must not be compiled by default")

    print("12DOF IMU quiet diagnostics checks passed")


if __name__ == "__main__":
    try:
        main()
    except AssertionError as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        sys.exit(1)
