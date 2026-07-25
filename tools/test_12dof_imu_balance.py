#!/usr/bin/env python3
import subprocess
import tempfile
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def main():
    source = r'''
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "include/edog_config.h"
#include "12_DOF_Version/include/motion_utils_12dof.h"

#if !EDOG_12DOF_IMU_BALANCE_ENABLED
#error "EDOG_12DOF_IMU_BALANCE_ENABLED must be 1 for the MPU damping experiment"
#endif

static int g_lastServoCenti[EDOG_SERVO_CHANNEL_COUNT];
static int g_servoWriteCount;
static int g_mockAccelXMg = 0;
static int g_mockAccelYMg = 0;
static int g_mockAccelZMg = 1000;
static int g_mockGyroXCentiDps = 0;
static int g_mockGyroYCentiDps = 0;
static int g_mockGyroZCentiDps = 0;
static int g_mockImuReadOk = 0;

int setServoCentiDeg(int channel, int centiDeg)
{
    if (channel >= 0 && channel < EDOG_SERVO_CHANNEL_COUNT) {
        g_lastServoCenti[channel] = centiDeg;
        g_servoWriteCount++;
    }
    return 0;
}

int setServo(int channel, int angle)
{
    (void)channel;
    (void)angle;
    return 0;
}

int setServoPulseUs(int channel, int pulseUs)
{
    (void)channel;
    (void)pulseUs;
    return 0;
}

int initPCA9685(void)
{
    return 0;
}

int setZero(void)
{
    return 0;
}

int UtilsGetValue(const char *key, char *value, unsigned int len)
{
    (void)key;
    (void)value;
    (void)len;
    return 0;
}

int UtilsSetValue(const char *key, const char *value)
{
    (void)key;
    (void)value;
    return 0;
}

void MpuMotionLightTask(void)
{
}

int MpuMotionLight_Init(void)
{
    return 0;
}

int MpuMotionLight_ReadAccelMg(int *xMg, int *yMg, int *zMg)
{
    if (!g_mockImuReadOk || xMg == NULL || yMg == NULL || zMg == NULL) {
        return -1;
    }
    *xMg = g_mockAccelXMg;
    *yMg = g_mockAccelYMg;
    *zMg = g_mockAccelZMg;
    return 0;
}

int MpuMotionLight_ReadGyroCentiDps(int *xCentiDps, int *yCentiDps, int *zCentiDps)
{
    if (!g_mockImuReadOk || xCentiDps == NULL || yCentiDps == NULL || zCentiDps == NULL) {
        return -1;
    }
    *xCentiDps = g_mockGyroXCentiDps;
    *yCentiDps = g_mockGyroYCentiDps;
    *zCentiDps = g_mockGyroZCentiDps;
    return 0;
}

int MpuMotionLight_ReadMotion(int *xMg, int *yMg, int *zMg,
                              int *gyroXCentiDps, int *gyroYCentiDps,
                              int *gyroZCentiDps)
{
    if (MpuMotionLight_ReadAccelMg(xMg, yMg, zMg) != 0) {
        return -1;
    }
    return MpuMotionLight_ReadGyroCentiDps(gyroXCentiDps, gyroYCentiDps, gyroZCentiDps);
}

#include "12_DOF_Version/src/motion_utils_12dof.c"

static void require_close(const char *name, int got, int expected, int tolerance)
{
    int diff = got - expected;
    if (diff < 0) {
        diff = -diff;
    }
    if (diff > tolerance) {
        fprintf(stderr, "%s expected %d +/- %d, got %d\n",
                name, expected, tolerance, got);
        exit(1);
    }
}

static void require_gt(const char *name, double left, double right)
{
    if (!(left > right)) {
        fprintf(stderr, "%s expected %.3f > %.3f\n", name, left, right);
        exit(1);
    }
}

static void require_lt(const char *name, double left, double right)
{
    if (!(left < right)) {
        fprintf(stderr, "%s expected %.3f < %.3f\n", name, left, right);
        exit(1);
    }
}

static void require_double_close(const char *name, double got, double expected, double tolerance)
{
    double diff = fabs(got - expected);
    if (diff > tolerance) {
        fprintf(stderr, "%s expected %.3f +/- %.3f, got %.3f\n",
                name, expected, tolerance, got);
        exit(1);
    }
}

int main(void)
{
    int rollCenti = 123;
    int pitchCenti = 456;
    Edog12DofFootPoint lf;
    Edog12DofFootPoint rf;
    Edog12DofFootPoint lb;
    Edog12DofFootPoint rb;
    Edog12DofFootPoint swingLf;
    Edog12DofFootPoint stanceLf;
    int neutralLfThigh;
    int neutralRfThigh;
    int tiltedLfThigh;
    int tiltedRfThigh;
    int dampedLfThigh;
    int dampedRfThigh;
    int rollRateCentiDps;
    int pitchRateCentiDps;
    Edog12DofFootPoint rollLf;
    Edog12DofFootPoint rollRf;
    double lateralOffset;
    Edog12DofRealtimeLegState legState;
    Edog12DofRealtimeCommand realtimeCmd = {
        0.01,
        0.0,
        0.0,
        0.006,
        0,
        0
    };

    if (computeImuTiltCentiDeg(0, 0, 1000, &rollCenti, &pitchCenti) != 0) {
        fprintf(stderr, "level IMU vector should be valid\n");
        return 1;
    }
    require_close("level roll", rollCenti, 0, 2);
    require_close("level pitch", pitchCenti, 0, 2);

    if (computeImuTiltCentiDeg(0, -174, 985, &rollCenti, &pitchCenti) != 0) {
        fprintf(stderr, "roll vector should be valid\n");
        return 1;
    }
    require_close("left-down roll", rollCenti, 1000, 60);

    if (computeImuTiltCentiDeg(-174, 0, 985, &rollCenti, &pitchCenti) != 0) {
        fprintf(stderr, "pitch vector should be valid\n");
        return 1;
    }
    require_close("nose-down pitch", pitchCenti, -1000, 60);

    lf = getDefaultFootPoint(0);
    rf = getDefaultFootPoint(1);
    lateralOffset = 7.0;
    lf.yMm += lateralOffset;
    applyImuBalanceFootCompensation(&lf, 0, 0, 1000, 0);
    applyImuBalanceFootCompensation(&rf, 1, 0, 1000, 0);
    require_gt("positive roll body-up compensation should push left support foot down", lf.zMm,
               Edog12Dof_DefaultFootZForLeg(1));
    require_lt("positive roll body-up compensation should pull right support foot up", rf.zMm,
               Edog12Dof_DefaultFootZForLeg(1));
    require_double_close("IMU compensation must keep left foot on hip plane plus lateral offset",
                         lf.yMm,
                         Edog12Dof_FootYOnHipPlaneForLeg(0, lf.zMm) + lateralOffset,
                         0.01);
    require_double_close("IMU compensation must keep right foot on mirrored hip plane",
                         rf.yMm,
                         Edog12Dof_FootYOnHipPlaneForLeg(1, rf.zMm),
                         0.01);

    lf = getDefaultFootPoint(0);
    lb = getDefaultFootPoint(2);
    applyImuBalanceFootCompensation(&lf, 0, 0, 0, -1000);
    applyImuBalanceFootCompensation(&lb, 2, 0, 0, -1000);
    require_gt("negative pitch body-up compensation should push front support foot down", lf.zMm,
               Edog12Dof_DefaultFootZForLeg(1));
    require_lt("negative pitch body-up compensation should pull rear support foot up", lb.zMm,
               Edog12Dof_DefaultFootZForLeg(0));

    lf = getDefaultFootPoint(0);
    lb = getDefaultFootPoint(2);
    applyImuBalanceFootCompensation(&lf, 0, 0, 0, 1000);
    applyImuBalanceFootCompensation(&lb, 2, 0, 0, 1000);
    require_lt("positive pitch body-up compensation should lift front support foot", lf.zMm,
               Edog12Dof_DefaultFootZForLeg(1));
    require_gt("positive pitch body-up compensation should lower rear support foot", lb.zMm,
               Edog12Dof_DefaultFootZForLeg(0));

    if (computeImuTiltCentiDeg(-605, -416, 716, &rollCenti, &pitchCenti) != 0) {
        fprintf(stderr, "front-left corner-down IMU vector should be valid\n");
        return 1;
    }
    require_gt("front-left corner-down roll should be positive",
               rollCenti, 0);
    require_lt("front-left corner-down pitch should be negative",
               pitchCenti, 0);
    lf = getDefaultFootPoint(0);
    rb = getDefaultFootPoint(3);
    applyImuBalanceFootCompensation(&lf, 0, 0, rollCenti, pitchCenti);
    applyImuBalanceFootCompensation(&rb, 3, 0, rollCenti, pitchCenti);
    require_double_close("mixed roll/pitch compensation must keep LF hip plane",
                         lf.yMm,
                         Edog12Dof_FootYOnHipPlaneForLeg(0, lf.zMm),
                         0.01);
    require_double_close("mixed roll/pitch compensation must keep RB hip plane",
                         rb.yMm,
                         Edog12Dof_FootYOnHipPlaneForLeg(3, rb.zMm),
                         0.01);

    stanceLf = getDefaultFootPoint(0);
    swingLf = getDefaultFootPoint(0);
    applyImuBalanceFootCompensation(&stanceLf, 0, 0, 1000, -1000);
    applyImuBalanceFootCompensation(&swingLf, 0, 1, 1000, -1000);
    require_lt("swing compensation must be softer than stance",
               fabs(swingLf.zMm - Edog12Dof_DefaultFootZForLeg(1)),
               fabs(stanceLf.zMm - Edog12Dof_DefaultFootZForLeg(1)));
    require_double_close("swing IMU compensation must also reproject hip-plane Y",
                         swingLf.yMm,
                         Edog12Dof_FootYOnHipPlaneForLeg(0, swingLf.zMm),
                         0.01);

    Edog12Dof_SetRuntimeGeometry(-13.0, 107.0, 135.0);
    legState.foot = getDefaultFootPoint(0);
    legState.liftOff = getDefaultFootPoint(0);
    legState.touchdown = getDefaultFootPoint(0);
    legState.touchdown.yMm += 8.0;
    legState.isSwing = 1;
    updateRealtimeSwingFoot(&legState, &realtimeCmd, 0, 0.5);
    require_double_close("legacy realtime swing must track hip plane plus interpolated side offset",
                         legState.foot.yMm,
                         Edog12Dof_FootYOnHipPlaneForLeg(0, legState.foot.zMm) + 4.0,
                         0.01);

    legState.foot = getDefaultFootPoint(1);
    legState.liftOff = getDefaultFootPoint(1);
    legState.touchdown = getDefaultFootPoint(1);
    legState.touchdown.yMm -= 8.0;
    legState.isSwing = 1;
    updatePupperSwingFoot(&legState, &realtimeCmd, 1, 0.5);
    require_double_close("Pupper swing must track mirrored hip plane plus side offset",
                         legState.foot.yMm,
                         Edog12Dof_FootYOnHipPlaneForLeg(1, legState.foot.zMm) - 8.0 / (double)realtimeSwingTicks(&realtimeCmd) / 0.5,
                         0.01);
    Edog12Dof_ResetRuntimeGeometry();

    g_mockImuReadOk = 1;
    g_mockAccelXMg = 0;
    g_mockAccelYMg = 0;
    g_mockAccelZMg = 1000;
    g_mockGyroXCentiDps = 0;
    g_mockGyroYCentiDps = 0;
    g_mockGyroZCentiDps = 0;
    g_servoWriteCount = 0;
    if (balance_stand_frame() != 0) {
        fprintf(stderr, "level stand balance frame should output a standing pose\n");
        return 1;
    }
    if (g_servoWriteCount < EDOG_12DOF_ACTIVE_SERVO_COUNT) {
        fprintf(stderr, "stand balance frame should update all active servos, wrote %d\n",
                g_servoWriteCount);
        return 1;
    }
    neutralLfThigh = g_lastServoCenti[LF_THIGH];
    neutralRfThigh = g_lastServoCenti[RF_THIGH];

    g_balanceControlReady = 0;
    g_balanceRollControlCentiDeg = 0;
    g_balancePitchControlCentiDeg = 0;
#if EDOG_12DOF_IMU_BALANCE_ENABLED
    g_balanceFilterReady = 0;
    g_balanceRollRateCentiDps = 0;
    g_balancePitchRateCentiDps = 0;
#endif
    g_mockAccelXMg = 0;
    g_mockAccelYMg = -174;
    g_mockAccelZMg = 985;
    g_mockGyroXCentiDps = 0;
    g_mockGyroYCentiDps = 0;
    g_mockGyroZCentiDps = 0;
    g_servoWriteCount = 0;
    if (balance_stand_frame() != 0) {
        fprintf(stderr, "tilted stand balance frame should still output pose\n");
        return 1;
    }
    tiltedLfThigh = g_lastServoCenti[LF_THIGH];
    tiltedRfThigh = g_lastServoCenti[RF_THIGH];
    if (EDOG_12DOF_IMU_BALANCE_ENABLED) {
        if (tiltedLfThigh == neutralLfThigh && tiltedRfThigh == neutralRfThigh) {
            fprintf(stderr, "tilted stand balance should change support leg targets\n");
            return 1;
        }
        if (tiltedLfThigh == tiltedRfThigh) {
            fprintf(stderr, "roll stand balance should create left/right asymmetric support\n");
            return 1;
        }
    } else {
        if (tiltedLfThigh != neutralLfThigh || tiltedRfThigh != neutralRfThigh) {
            fprintf(stderr, "disabled IMU balance should keep tilted stand targets neutral\n");
            return 1;
        }
    }

    g_balanceControlReady = 0;
    g_balanceRollControlCentiDeg = 0;
    g_balancePitchControlCentiDeg = 0;
#if EDOG_12DOF_IMU_BALANCE_ENABLED
    g_balanceFilterReady = 0;
    g_balanceRollRateCentiDps = 0;
    g_balancePitchRateCentiDps = 0;
#endif
    g_mockAccelXMg = 0;
    g_mockAccelYMg = 0;
    g_mockAccelZMg = 1000;
    g_mockGyroXCentiDps = 12000;
    g_mockGyroYCentiDps = 0;
    g_mockGyroZCentiDps = 0;
    g_servoWriteCount = 0;
    if (balance_stand_frame() != 0) {
        fprintf(stderr, "gyro-damped stand balance frame should still output pose\n");
        return 1;
    }
    dampedLfThigh = g_lastServoCenti[LF_THIGH];
    dampedRfThigh = g_lastServoCenti[RF_THIGH];
    if (dampedLfThigh == neutralLfThigh && dampedRfThigh == neutralRfThigh) {
        fprintf(stderr, "roll-rate damping should change support leg targets even at level accel\n");
        return 1;
    }
    if (dampedLfThigh == dampedRfThigh) {
        fprintf(stderr, "roll-rate damping should create left/right asymmetric support\n");
        return 1;
    }

    stopCurrentMotion();
    if (g_balanceMode != EDOG_IMU_BALANCE_MODE_STOP_SETTLING) {
        fprintf(stderr, "stop should put IMU balance into stop-settling mode\n");
        return 1;
    }
    if (g_balanceRollControlCentiDeg != 0 || g_balancePitchControlCentiDeg != 0 ||
        g_balanceControlReady != 0) {
        fprintf(stderr, "stop should clear IMU control history and outputs\n");
        return 1;
    }
    g_mockAccelXMg = 0;
    g_mockAccelYMg = -174;
    g_mockAccelZMg = 985;
    g_mockGyroXCentiDps = 0;
    g_mockGyroYCentiDps = 0;
    getImuBalanceControlCentiDeg(&rollCenti, &pitchCenti);
    if (rollCenti != 0 || pitchCenti != 0) {
        fprintf(stderr, "stop-settling must read IMU but output zero compensation\n");
        return 1;
    }

    g_balanceModeStartMs = getTimeMs() - EDOG_12DOF_IMU_BALANCE_STOP_SETTLE_MS - 1;
    g_mockAccelXMg = 0;
    g_mockAccelYMg = 0;
    g_mockAccelZMg = 1000;
    g_mockGyroXCentiDps = 0;
    g_mockGyroYCentiDps = 0;
    for (int i = 0; i < EDOG_12DOF_IMU_BALANCE_STOP_STABLE_FRAMES; i++) {
        getImuBalanceControlCentiDeg(&rollCenti, &pitchCenti);
    }
    if (g_balanceMode != EDOG_IMU_BALANCE_MODE_RAMP_IN) {
        fprintf(stderr, "stable stop-settling frames should enter ramp-in mode\n");
        return 1;
    }

    g_balanceMode = EDOG_IMU_BALANCE_MODE_RAMP_IN;
    g_balanceModeStartMs = getTimeMs() - EDOG_12DOF_IMU_BALANCE_RAMP_IN_MS / 2;
    g_balanceRollControlCentiDeg = 0;
    g_balancePitchControlCentiDeg = 0;
    g_mockAccelXMg = 0;
    g_mockAccelYMg = -174;
    g_mockAccelZMg = 985;
    g_mockGyroXCentiDps = 0;
    g_mockGyroYCentiDps = 0;
    for (int i = 0; i < 4; i++) {
        getImuBalanceControlCentiDeg(&rollCenti, &pitchCenti);
    }
    if (g_balanceMode != EDOG_IMU_BALANCE_MODE_RAMP_IN ||
        (rollCenti == 0 && pitchCenti == 0)) {
        fprintf(stderr, "ramp-in should gradually restore nonzero IMU compensation\n");
        return 1;
    }

    g_balanceMode = EDOG_IMU_BALANCE_MODE_RAMP_IN;
    g_balanceModeStartMs = getTimeMs() - 100;
    g_mockAccelXMg = 0;
    g_mockAccelYMg = 0;
    g_mockAccelZMg = 1000;
    g_mockGyroXCentiDps = EDOG_12DOF_IMU_BALANCE_STOP_EXIT_RATE_CENTI_DPS + 100;
    g_mockGyroYCentiDps = 0;
    getImuBalanceControlCentiDeg(&rollCenti, &pitchCenti);
    if (g_balanceMode != EDOG_IMU_BALANCE_MODE_STOP_SETTLING ||
        rollCenti != 0 || pitchCenti != 0) {
        fprintf(stderr, "ramp-in should fall back to stop-settling on large gyro shake\n");
        return 1;
    }

    g_balanceMode = EDOG_IMU_BALANCE_MODE_NORMAL;
    g_balanceControlReady = 0;
    prepareImuBalanceControlCentiDeg(70, -70, 0, 0, &rollCenti, &pitchCenti);
    if (rollCenti != 0 || pitchCenti != 0) {
        fprintf(stderr, "normal stand deadband should suppress sub-0.8deg noise\n");
        return 1;
    }
    prepareImuBalanceControlCentiDeg(900, 0, 0, 0, &rollCenti, &pitchCenti);
    if (abs(rollCenti) > EDOG_12DOF_IMU_BALANCE_ROLL_STEP_LIMIT_CENTI_DEG) {
        fprintf(stderr, "IMU roll control should be rate-limited per frame\n");
        return 1;
    }

    g_balanceMpuReady = 1;
    g_balanceFilterReady = 0;
    g_mockImuReadOk = 1;
    g_mockAccelXMg = 0;
    g_mockAccelYMg = 0;
    g_mockAccelZMg = 1000;
    g_mockGyroXCentiDps = 12000;
    g_mockGyroYCentiDps = 8000;
    g_mockGyroZCentiDps = 0;
    if (readImuBalanceMotionCenti(&rollCenti, &pitchCenti,
                                  &rollRateCentiDps, &pitchRateCentiDps) != 0) {
        fprintf(stderr, "mock IMU motion read should succeed\n");
        return 1;
    }
    require_lt("gyro X positive should map to negative roll rate",
               rollRateCentiDps, 0);
    require_lt("gyro Y positive should map to negative pitch rate",
               pitchRateCentiDps, 0);

    g_balanceMpuReady = 1;
    g_balanceFilterReady = 0;
    g_mockImuReadOk = 1;
    g_mockAccelXMg = 0;
    g_mockAccelYMg = 0;
    g_mockAccelZMg = 1000;
    g_mockGyroXCentiDps = -10000;
    g_mockGyroYCentiDps = 0;
    g_mockGyroZCentiDps = 0;
    if (readImuBalanceMotionCenti(&rollCenti, &pitchCenti,
                                  &rollRateCentiDps, &pitchRateCentiDps) != 0) {
        fprintf(stderr, "complementary gyro-predicted IMU read should succeed\n");
        return 1;
    }
    if (rollCenti <= 50 || rollCenti >= 130) {
        fprintf(stderr, "200Hz complementary filter should integrate about 1deg roll per 100dps control frame, got %d\n",
                rollCenti);
        return 1;
    }
    require_close("level accel should keep gyro-predicted pitch near zero", pitchCenti, 0, 4);

    g_mockAccelZMg = 2000;
    g_mockGyroXCentiDps = -10000;
    if (readImuBalanceMotionCenti(&rollCenti, &pitchCenti,
                                  &rollRateCentiDps, &pitchRateCentiDps) != 0) {
        fprintf(stderr, "invalid accel magnitude should still allow short gyro prediction\n");
        return 1;
    }
    if (rollCenti <= 100) {
        fprintf(stderr, "invalid accel sample should not reset to accel tilt; got roll %d\n", rollCenti);
        return 1;
    }

    if (computeImuBodyZUpCompMm(0, 1000, 0, 0) >= 0.0 ||
        computeImuBodyZUpCompMm(2, 1000, 0, 0) >= 0.0 ||
        computeImuBodyZUpCompMm(1, 1000, 0, 0) <= 0.0 ||
        computeImuBodyZUpCompMm(3, 1000, 0, 0) <= 0.0) {
        fprintf(stderr, "positive roll should push left legs down and pull right legs up in body Z-up coordinates\n");
        return 1;
    }
    if (computeImuBodyZUpCompMm(0, 0, -1000, 0) >= 0.0 ||
        computeImuBodyZUpCompMm(1, 0, -1000, 0) >= 0.0 ||
        computeImuBodyZUpCompMm(2, 0, -1000, 0) <= 0.0 ||
        computeImuBodyZUpCompMm(3, 0, -1000, 0) <= 0.0) {
        fprintf(stderr, "negative pitch should push front legs down and pull rear legs up in body Z-up coordinates\n");
        return 1;
    }
    if (computeImuBodyZUpCompMm(0, 0, 1000, 0) <= 0.0 ||
        computeImuBodyZUpCompMm(1, 0, 1000, 0) <= 0.0 ||
        computeImuBodyZUpCompMm(2, 0, 1000, 0) >= 0.0 ||
        computeImuBodyZUpCompMm(3, 0, 1000, 0) >= 0.0) {
        fprintf(stderr, "positive pitch should lift front legs and lower rear legs in body Z-up coordinates\n");
        return 1;
    }
    if (fabs(computeImuBodyZUpCompMm(0, 9000, -9000, 0)) >
        EDOG_12DOF_IMU_BALANCE_MAX_FOOT_Z_MM + 0.001) {
        fprintf(stderr, "body Z-up compensation should clamp to configured max foot Z\n");
        return 1;
    }

    rollLf = getDefaultFootPoint(0);
    rollRf = getDefaultFootPoint(1);
    applyImuBalanceFootCompensation(&rollLf, 0, 0, 1000, 0);
    applyImuBalanceFootCompensation(&rollRf, 1, 0, 1000, 0);
    if (!(rollLf.zMm > Edog12Dof_DefaultFootZForLeg(1) &&
          rollRf.zMm < Edog12Dof_DefaultFootZForLeg(1))) {
        fprintf(stderr, "positive roll foot compensation must convert body Z-up into IK Z-down correctly\n");
        return 1;
    }

    return 0;
}
'''

    with tempfile.TemporaryDirectory() as tmpdir:
        tmp = Path(tmpdir)
        stub_include = tmp / "stubs"
        test_c = tmp / "test_12dof_imu_balance.c"
        binary = tmp / "test_12dof_imu_balance"
        stub_include.mkdir()
        (stub_include / "iot_gpio.h").write_text(
            "#ifndef IOT_GPIO_H\n"
            "#define IOT_GPIO_H\n"
            "#define GPIO0_PC7 0\n"
            "#define GPIO0_PB5 0\n"
            "#define GPIO0_PB4 0\n"
            "#define GPIO1_PD0 0\n"
            "#endif\n",
            encoding="utf-8",
        )
        (stub_include / "kv_store.h").write_text(
            "#ifndef KV_STORE_H\n"
            "#define KV_STORE_H\n"
            "int UtilsGetValue(const char *key, char *value, unsigned int len);\n"
            "int UtilsSetValue(const char *key, const char *value);\n"
            "#endif\n",
            encoding="utf-8",
        )
        test_c.write_text(source, encoding="utf-8")
        cmd = [
            "gcc",
            "-std=c99",
            "-D_DEFAULT_SOURCE",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-Wno-unused-function",
            "-I",
            str(stub_include),
            "-I",
            str(ROOT),
            str(test_c),
            str(ROOT / "12_DOF_Version/src/gait_generate_12dof.c"),
            "-lm",
            "-o",
            str(binary),
        ]
        subprocess.run(cmd, check=True)
        subprocess.run([str(binary)], check=True)


if __name__ == "__main__":
    main()
