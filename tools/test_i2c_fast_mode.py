#!/usr/bin/env python3
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
I2C_GUARD_HEADER = ROOT / "utils/include/i2c_bus_guard.h"
SERVO = ROOT / "utils/src/servo_control.c"
MPU = ROOT / "utils/src/mpu6050_motion_light.c"


def main():
    guard_header = I2C_GUARD_HEADER.read_text(encoding="utf-8")
    servo = SERVO.read_text(encoding="utf-8")
    mpu = MPU.read_text(encoding="utf-8")

    if "#define EDOG_I2C_BUS_FREQUENCY EI2C_FRE_400K" not in guard_header:
        raise AssertionError("shared EDOG I2C bus must be configured for 400kHz fast mode")

    for path_name, text in [
        ("utils/src/servo_control.c", servo),
        ("utils/src/mpu6050_motion_light.c", mpu),
    ]:
        if "IoTI2cInit" not in text:
            raise AssertionError(f"{path_name} must initialize the shared I2C bus")
        if "IoTI2cInit(" in text and "EDOG_I2C_BUS_FREQUENCY" not in text:
            raise AssertionError(f"{path_name} must use the shared I2C speed macro")
        if "EI2C_FRE_100K" in text:
            raise AssertionError(f"{path_name} must not force the shared bus back to 100kHz")

    print("I2C fast mode checks passed")


if __name__ == "__main__":
    try:
        main()
    except AssertionError as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        sys.exit(1)
