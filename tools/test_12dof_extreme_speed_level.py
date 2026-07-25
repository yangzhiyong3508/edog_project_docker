#!/usr/bin/env python3
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MOTION = ROOT / "12_DOF_Version/src/motion_utils_12dof.c"
IOT = ROOT / "utils/src/iot.c"
IOT_CONTROL = ROOT / "utils/src/iot_control.c"


def require(text, token, label):
    if token not in text:
        raise AssertionError(f"{label} missing token: {token}")


def reject(text, token, label):
    if token in text:
        raise AssertionError(f"{label} still contains old token: {token}")


def main():
    motion = MOTION.read_text(encoding="utf-8")
    iot = IOT.read_text(encoding="utf-8")
    iot_control = IOT_CONTROL.read_text(encoding="utf-8")

    require(motion, "static const double speedTable[7] = {1.5, 1.0, 0.8, 0.6, 0.4, 0.3, 0.3}", "motion")
    require(motion, "static const uint8_t framePeriodPercent[7] = {140, 100, 75, 45, 35, 30, 30}", "motion")
    require(motion, "static const int requestedCycleMs[7] = {1600, 1200, 900, 700, 560, 480, 480}", "motion")
    reject(motion, "level > 4", "motion")
    require(motion, "level > 6", "motion")

    require(iot, "level > 6", "iot")
    require(iot, "valid range is 0~6", "iot")
    reject(iot, "valid range is 0~4", "iot")

    require(iot_control, "level > 6", "iot_control")
    require(iot_control, "valid range is 0~6", "iot_control")
    reject(iot_control, "valid range is 0~4", "iot_control")

    print("12DOF extreme speed level checks passed")


if __name__ == "__main__":
    main()
