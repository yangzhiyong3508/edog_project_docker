#!/usr/bin/env python3
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SERVO = ROOT / "utils/src/servo_control.c"


def require(token, text):
    if token not in text:
        raise AssertionError(f"servo_control.c missing {token}")


def main():
    servo = SERVO.read_text(encoding="utf-8")

    for token in [
        "static uint16_t g_pwmOffCount[EDOG_SERVO_CHANNEL_COUNT]",
        "static unsigned char g_pwmOffCountValid[EDOG_SERVO_CHANNEL_COUNT]",
        "static int pulseUsToPwmCount(int pulseUs)",
        "if (g_pwmOffCountValid[channel] && g_pwmOffCount[channel] == (uint16_t)count)",
        "g_pwmOffCount[channel] = (uint16_t)count",
        "g_pwmOffCountValid[channel] = 1",
        "+ denominator / 2",
    ]:
        require(token, servo)

    if "count = pulseUs * EDOG_SERVO_PWM_FREQUENCY_HZ * 4096 / 1000000;" in servo:
        raise AssertionError("setServoPulseUs must round PWM counts instead of truncating down")

    print("servo PWM quantization checks passed")


if __name__ == "__main__":
    try:
        main()
    except AssertionError as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        sys.exit(1)
