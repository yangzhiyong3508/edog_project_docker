#!/usr/bin/env python3
import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "include/edog_config.h"
MPU_H = ROOT / "utils/include/mpu6050_motion_light.h"
MPU_C = ROOT / "utils/src/mpu6050_motion_light.c"
MOTION = ROOT / "12_DOF_Version/src/motion_utils_12dof.c"


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def require_token(text, token, label):
    require(token in text, f"{label} missing {token}")


def macro_number(text, name):
    match = re.search(rf"#define\s+{re.escape(name)}\s+([0-9.]+)", text)
    require(match is not None, f"missing macro {name}")
    value = match.group(1)
    return float(value) if "." in value else int(value)


def main():
    config = CONFIG.read_text(encoding="utf-8")
    mpu_h = MPU_H.read_text(encoding="utf-8")
    mpu_c = MPU_C.read_text(encoding="utf-8")
    motion = MOTION.read_text(encoding="utf-8")

    require_token(config, "#define EDOG_12DOF_IMU_BALANCE_ENABLED          1", "config")
    require_token(config, "#define EDOG_MPU_RGB_LIGHT_TASK_ENABLED          0", "config")
    require(macro_number(config, "EDOG_12DOF_GAIT_FRAME_FPS") == 100,
            "12DOF gait/servo control must stay at 100Hz")
    require(macro_number(config, "EDOG_IMU_FUSION_SAMPLE_INTERVAL_MS") == 5,
            "IMU fusion sampling must run at 200Hz")
    require(macro_number(config, "EDOG_IMU_FUSION_COMPLEMENTARY_ALPHA_PERCENT") == 97,
            "complementary filter alpha must default to 97 percent")
    require(macro_number(config, "EDOG_12DOF_IMU_BALANCE_MAX_FOOT_Z_MM") == 16.0,
            "IMU foot-Z compensation must use the stronger 16mm experiment limit")
    require(macro_number(config, "EDOG_12DOF_IMU_BALANCE_DEADBAND_CENTI_DEG") >= 80,
            "IMU deadband should be at least 0.8deg for stop anti-shake damping")
    require(macro_number(config, "EDOG_12DOF_IMU_BALANCE_ROLL_KP_MM_PER_DEG") == 1.2,
            "roll Kp should be increased for the sign/frequency correction experiment")
    require(macro_number(config, "EDOG_12DOF_IMU_BALANCE_PITCH_KP_MM_PER_DEG") == 0.6,
            "pitch Kp should be increased for the sign/frequency correction experiment")
    require(macro_number(config, "EDOG_12DOF_IMU_BALANCE_ROLL_KD_MM_PER_DEG_PER_SEC") == 0.050,
            "roll Kd should be increased for stronger damping")
    require(macro_number(config, "EDOG_12DOF_IMU_BALANCE_PITCH_KD_MM_PER_DEG_PER_SEC") == 0.025,
            "pitch Kd should be increased for stronger damping")
    require(macro_number(config, "EDOG_12DOF_IMU_BALANCE_SWING_SCALE_PERCENT") == 50,
            "swing compensation scale should be 50 percent")
    require(macro_number(config, "EDOG_12DOF_IMU_BALANCE_STOP_SETTLE_MS") == 800,
            "stop settling must wait before re-enabling IMU balance")
    require(macro_number(config, "EDOG_12DOF_IMU_BALANCE_STOP_MAX_SETTLE_MS") == 2500,
            "stop settling must have a bounded max wait")
    require(macro_number(config, "EDOG_12DOF_IMU_BALANCE_STOP_STABLE_FRAMES") == 15,
            "stop settling must require consecutive stable frames")
    require(macro_number(config, "EDOG_12DOF_IMU_BALANCE_STOP_STABLE_TILT_CENTI_DEG") == 400,
            "stop stable tilt threshold should be 4deg")
    require(macro_number(config, "EDOG_12DOF_IMU_BALANCE_STOP_EXIT_TILT_CENTI_DEG") == 600,
            "ramp-in exit tilt threshold should be 6deg")
    require(macro_number(config, "EDOG_12DOF_IMU_BALANCE_STOP_STABLE_RATE_CENTI_DPS") == 8000,
            "stop stable gyro threshold should be 80deg/s")
    require(macro_number(config, "EDOG_12DOF_IMU_BALANCE_STOP_EXIT_RATE_CENTI_DPS") == 12000,
            "ramp-in exit gyro threshold should be 120deg/s")
    require(macro_number(config, "EDOG_12DOF_IMU_BALANCE_RAMP_IN_MS") == 1200,
            "IMU balance should ramp in over 1.2s after stop")
    require(macro_number(config, "EDOG_12DOF_IMU_BALANCE_ROLL_STEP_LIMIT_CENTI_DEG") == 167,
            "roll control should be rate-limited per frame")
    require(macro_number(config, "EDOG_12DOF_IMU_BALANCE_PITCH_STEP_LIMIT_CENTI_DEG") == 167,
            "pitch control should be rate-limited per frame")
    require(macro_number(config, "EDOG_12DOF_IMU_BALANCE_INTEGRAL_KI_PERCENT") == 0,
            "PID integral term must stay disabled for the first anti-shake version")

    for token in [
        "EDOG_12DOF_IMU_BALANCE_ROLL_KD_MM_PER_DEG_PER_SEC",
        "EDOG_12DOF_IMU_BALANCE_PITCH_KD_MM_PER_DEG_PER_SEC",
    ]:
        require_token(config, token, "config")

    for token in [
        "int MpuMotionLight_ReadGyroCentiDps(int *xCentiDps, int *yCentiDps, int *zCentiDps);",
        "int MpuMotionLight_ReadMotion(int *xMg, int *yMg, int *zMg,",
    ]:
        require_token(mpu_h, token, "mpu6050_motion_light.h")

    for token in [
        "#define MPU6050_REG_GYRO_XOUT_H",
        "#define MPU6050_GYRO_LSB_PER_DPS",
        "Mpu6050_WriteReg(MPU6050_REG_CONFIG, 0x03)",
        "MpuMotionLight_ReadGyroCentiDps",
        "rawX * 100 / MPU6050_GYRO_LSB_PER_DPS",
        "MpuMotionLight_ReadMotion",
    ]:
        require_token(mpu_c, token, "mpu6050_motion_light.c")

    for token in [
        "readImuBalanceMotionCenti",
        "rollRateCentiDps",
        "pitchRateCentiDps",
        "EDOG_12DOF_IMU_BALANCE_ROLL_KD_MM_PER_DEG_PER_SEC",
        "EDOG_12DOF_IMU_BALANCE_PITCH_KD_MM_PER_DEG_PER_SEC",
        "rollRateDegPerSec",
        "pitchRateDegPerSec",
        "controlRateCentiDps",
        "imuFusionSamplesPerControlFrame",
        "updateImuFusionSample",
        "EDOG_IMU_FUSION_COMPLEMENTARY_ALPHA_PERCENT",
        "EDOG_IMU_FUSION_SAMPLE_INTERVAL_MS",
        "computeImuBodyZUpCompMm",
        "applyImuBalanceFootCompensation",
        "table->baseFeet[frameIndex][leg]",
        "jointAnglesToServoFrameLeg",
        "table->isSwing[frameIndex][leg]",
        "typedef enum {\n    EDOG_IMU_BALANCE_MODE_NORMAL",
        "EDOG_IMU_BALANCE_MODE_STOP_SETTLING",
        "EDOG_IMU_BALANCE_MODE_RAMP_IN",
        "static EdogImuBalanceMode g_balanceMode",
        "static void clearImuBalanceControlState",
        "static void enterImuBalanceMode",
        "static int imuBalanceMotionIsStable",
        "static int imuBalanceMotionExceedsExit",
        "static void applyImuBalanceRampScale",
        "enterImuBalanceMode(EDOG_IMU_BALANCE_MODE_STOP_SETTLING)",
        "enterImuBalanceMode(EDOG_IMU_BALANCE_MODE_NORMAL)",
        "EDOG_12DOF_IMU_BALANCE_ROLL_STEP_LIMIT_CENTI_DEG",
        "EDOG_12DOF_IMU_BALANCE_INTEGRAL_KI_PERCENT",
    ]:
        require_token(motion, token, "motion_utils_12dof.c")

    require("MpuMotionLight_ReadMotion(&xMg, &yMg, &zMg, &gyroXCentiDps" in motion,
            "IMU balance must read accel and gyro in one path")
    require("controlRateCentiDps = -gyroXCentiDps" in motion,
            "roll-rate gyro sign must match body X-forward/Y-left/Z-up coordinates")
    require("controlRateCentiDps = -gyroYCentiDps" in motion,
            "pitch-rate gyro sign must match body X-forward/Y-left/Z-up coordinates")
    require("g_balanceRollDeg = filteredRollDeg" in motion,
            "IMU roll angle must be produced by complementary-filter state")
    require("g_balancePitchDeg = filteredPitchDeg" in motion,
            "IMU pitch angle must be produced by complementary-filter state")
    require("EDOG_12DOF_IMU_BALANCE_FILTER_NUM" not in config + motion,
            "old one-pole low-pass numerator must be removed from IMU balance")
    require("EDOG_12DOF_IMU_BALANCE_FILTER_DEN" not in config + motion,
            "old one-pole low-pass denominator must be removed from IMU balance")
    require("Kalman" not in config + motion + mpu_c and "kalman" not in config + motion + mpu_c,
            "first IMU fusion version must not add Kalman filtering")
    require("g_balanceRollRateCentiDps * EDOG_12DOF_IMU_BALANCE_FAIL_DECAY_PERCENT / 100" in motion,
            "gyro damping state must decay on IMU read failure")
    require("foot->zMm -= bodyZUpCompMm" in motion,
            "body Z-up compensation must be converted into IK Z-down coordinates")
    require("foot->yMm = Edog12Dof_FootYOnHipPlaneForLeg(leg, foot->zMm)" in motion,
            "foot Y must still be reprojected per leg after IMU Z compensation")
    require("applyImuDampingServoTargetCenti(" not in motion,
            "table runtime must not use hard-coded thigh/calf servo-angle IMU damping")

    print("12DOF MPU6050 damping experiment checks passed")


if __name__ == "__main__":
    try:
        main()
    except AssertionError as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        sys.exit(1)
