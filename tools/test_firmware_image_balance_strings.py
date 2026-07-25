#!/usr/bin/env python3
import sys
from pathlib import Path


REQUIRED_STRINGS = [
    b"[MQTT] command=%s",
]

FORBIDDEN_STRINGS = [
    b"[AppInit] MpuMotionLightTask disabled",
    b"[IMU-BALANCE] MPU6050 initialized for self balance",
    b"gyroCentiDps=",
    b"EDOG_12DOF_BUILD_PARAMS",
    b"[MPU6050] motion RGB task start",
    b"[MotionTask] start motion command loop",
    b"Edog published to",
    b"mqtt publish success:",
]


def main():
    if len(sys.argv) != 2:
        print("usage: test_firmware_image_balance_strings.py <Firmware_12_DOF.img>",
              file=sys.stderr)
        return 2

    image = Path(sys.argv[1])
    if not image.is_file():
        print(f"FAIL: image not found: {image}", file=sys.stderr)
        return 1

    data = image.read_bytes()
    missing = [s.decode("ascii") for s in REQUIRED_STRINGS if s not in data]
    if missing:
        print(f"FAIL: firmware image is missing required strings: {', '.join(missing)}",
              file=sys.stderr)
        return 1
    present = [s.decode("ascii") for s in FORBIDDEN_STRINGS if s in data]
    if present:
        print(f"FAIL: firmware image still contains noisy debug strings: {', '.join(present)}",
              file=sys.stderr)
        return 1

    print("firmware image quiet-log strings checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
