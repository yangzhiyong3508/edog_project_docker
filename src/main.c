#include "ohos_init.h"
#include "los_task.h"

#include "../include/edog_config.h"
#include "../include/wifi_mqtt_task.h"
#include "../utils/include/i2c_bus_guard.h"
#include "../utils/include/iot_control.h"
#include "../utils/include/mpu6050_motion_light.h"
#include "../utils/include/motion_utils.h"
#include "../utils/include/servo_control.h"
#include "../utils/include/task_util.h"
#include "../utils/include/wifi_tool.h"

#include <stdio.h>

#if EDOG_POWER_ON_LF_SERVO_TEST_ENABLED
#define EDOG_POWER_ON_LF_SERVO_TEST_FIXED_SCALE 4096
#endif

static void ServoInitTask(void)
{
    if (initPCA9685() != 0) {
        printf("[ServoInitTask] PCA9685 init FAILED\n");
        (void)LOS_TaskDelete(LOS_CurTaskIDGet());
        return;
    }
    (void)clearDogServoCenterTrimsOnceForRealAngleCalibration();
    (void)loadDogServoCenterTrims();
    setSpeedLevel(3);
    init_dog(EDOG_12DOF_COMMAND_STEP_LENGTH_M, EDOG_12DOF_COMMAND_STEP_HEIGHT_M);
    (void)LOS_TaskDelete(LOS_CurTaskIDGet());
}

// ======================================================
//                APP 入口
// ======================================================
#if EDOG_POWER_ON_LF_SERVO_TEST_ENABLED
static int LeftFrontServoTestSmoothPulseUs(int phase)
{
    const int frameCount = EDOG_POWER_ON_LF_SERVO_TEST_FRAME_COUNT;
    const int minPulse = EDOG_POWER_ON_LF_SERVO_TEST_MIN_PULSE_US;
    const int pulseRange = EDOG_POWER_ON_LF_SERVO_TEST_MAX_PULSE_US -
        EDOG_POWER_ON_LF_SERVO_TEST_MIN_PULSE_US;
    const long long scale = EDOG_POWER_ON_LF_SERVO_TEST_FIXED_SCALE;
    long long t;
    long long smooth;

    if (phase < 0) {
        phase = 0;
    } else if (phase > frameCount) {
        phase = frameCount;
    }

    /*
     * Fixed-point smoothstep: s = 3t^2 - 2t^3, with t in [0, scale].
     * Rounding each division avoids a systematic downward stair.
     */
    t = ((long long)phase * scale + frameCount / 2) / frameCount;
    smooth = (t * t * (3 * scale - 2 * t) + scale * scale / 2) /
        (scale * scale);
    return minPulse + (int)(((long long)pulseRange * smooth + scale / 2) /
        scale);
}

static void LeftFrontServoTestTask(void)
{
    int phase = 0;
    int direction = 1;
    int pulseUs;

    if (initPCA9685() != 0) {
        printf("[LFServoTest] PCA9685 init FAILED\n");
        (void)LOS_TaskDelete(LOS_CurTaskIDGet());
        return;
    }

    while (1) {
        int hipPulseUs;
        int thighPulseUs;
        int calfPulseUs;

        pulseUs = LeftFrontServoTestSmoothPulseUs(phase);
        hipPulseUs = pulseUs;
        thighPulseUs = EDOG_POWER_ON_LF_SERVO_TEST_MIN_PULSE_US +
            EDOG_POWER_ON_LF_SERVO_TEST_MAX_PULSE_US - pulseUs;
        calfPulseUs = pulseUs;

        (void)setServoPulseUs(LF_HIP, hipPulseUs);
        (void)setServoPulseUs(LF_THIGH, thighPulseUs);
        (void)setServoPulseUs(LF_CALF, calfPulseUs);

        phase += direction;
        if (phase >= EDOG_POWER_ON_LF_SERVO_TEST_FRAME_COUNT) {
            phase = EDOG_POWER_ON_LF_SERVO_TEST_FRAME_COUNT;
            direction = -1;
        } else if (phase <= 0) {
            phase = 0;
            direction = 1;
        }
        LOS_Msleep(EDOG_POWER_ON_LF_SERVO_TEST_STEP_MS);
    }
}
#endif

static void AppInit(void)
{
    UINT32 ret;

    if (EdogI2cBusGuard_Init() != 0) {
        printf("[AppInit] I2C bus mutex create FAILED\n");
    }

#if EDOG_POWER_ON_LF_SERVO_TEST_ENABLED
    ret = CreateTask(LeftFrontServoTestTask, "lf_servo_test_task",
                     EDOG_TASK_SERVO_INIT_STACK_SIZE,
                     EDOG_TASK_SERVO_INIT_PRIORITY);
    if (ret != LOS_OK) {
        printf("[AppInit] LF servo test task create FAILED\n");
    }
    return;
#endif

    ret = CreateTask(ServoInitTask, "servo_init_task",
                     EDOG_TASK_SERVO_INIT_STACK_SIZE,
                     EDOG_TASK_SERVO_INIT_PRIORITY);
    if (ret != LOS_OK) {
        printf("[AppInit] ServoInitTask create FAILED\n");
    }

    ret = CreateTask(IotControl_MotionTask, "motion_control_task",
                     EDOG_TASK_MOTION_STACK_SIZE,
                     EDOG_TASK_MOTION_PRIORITY);
    if (ret != LOS_OK) {
        printf("[AppInit] MotionTask create FAILED\n");
    }

    ret = CreateTask(WifiTool_ResetButtonTask, "wifi_reset_task",
                     EDOG_TASK_WIFI_RESET_STACK_SIZE,
                     EDOG_TASK_WIFI_RESET_PRIORITY);
    if (ret != LOS_OK) {
        printf("[AppInit] WifiResetTask create FAILED\n");
    }

    ret = CreateTask(WifiTask, "wifi_mqtt_task",
                     EDOG_TASK_WIFI_MQTT_STACK_SIZE,
                     EDOG_TASK_WIFI_MQTT_PRIORITY);
    if (ret != LOS_OK) {
        printf("[AppInit] WifiTask create FAILED\n");
    }

#if EDOG_MPU_RGB_LIGHT_TASK_ENABLED
    ret = CreateTask(MpuMotionLightTask, "mpu_motion_light_task",
                     EDOG_TASK_MPU_LIGHT_STACK_SIZE,
                     EDOG_TASK_MPU_LIGHT_PRIORITY);
    if (ret != LOS_OK) {
        printf("[AppInit] MpuMotionLightTask create FAILED\n");
    }
#endif
}

APP_FEATURE_INIT(AppInit);
