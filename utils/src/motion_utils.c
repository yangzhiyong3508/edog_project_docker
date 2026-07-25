#include "../include/servo_control.h"
#include "../include/gait_generate.h"
#include "../../include/edog_config.h"
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>

// ==================== 通道定义 ====================
#define LF_THIGH  0
#define LF_SHANK  2
#define RF_THIGH  12
#define RF_SHANK  14
#define LH_THIGH  4
#define LH_SHANK  6
#define RH_THIGH  8
#define RH_SHANK  10
#define SERVO_CHANNEL_COUNT 16
#define ACTIVE_SERVO_COUNT  8

// ==================== 全局控制变量 ====================
static volatile int stopFlag = 0;
static volatile int motionActive = 0;
static volatile int speedLevel = 0; // 速度档位（0~4）
static int servoAngles[SERVO_CHANNEL_COUNT] = {0};
static unsigned char servoAngleValid[SERVO_CHANNEL_COUNT] = {0};
static const int activeServoChannels[ACTIVE_SERVO_COUNT] = {
    LF_THIGH, LF_SHANK, RH_THIGH, RH_SHANK,
    LH_THIGH, LH_SHANK, RF_THIGH, RF_SHANK
};

static int clampServoAngle(int angle)
{
    if (angle > EDOG_SERVO_MAX_ANGLE) return EDOG_SERVO_MAX_ANGLE;
    if (angle < EDOG_SERVO_MIN_ANGLE) return EDOG_SERVO_MIN_ANGLE;
    return angle;
}

static int motionSetServo(int channel, int angle)
{
    int limited = clampServoAngle(angle);
    int ret = setServo(channel, limited);

    if (ret != 0) {
        return ret;
    }
    if (channel >= 0 && channel < SERVO_CHANNEL_COUNT) {
        servoAngles[channel] = limited;
        servoAngleValid[channel] = 1;
    }
    return 0;
}

// 本文件内所有舵机输出都走记录包装，停止时才能从当前姿态平滑回站姿。
#define setServo motionSetServo

int setDogServoAngleTracked(int channel, int angle)
{
    return motionSetServo(channel, angle);
}

int setDogLegsStraightPose(void)
{
    for (int k = 0; k < ACTIVE_SERVO_COUNT; k++) {
        if (motionSetServo(activeServoChannels[k], 0) != 0) {
            return -1;
        }
        usleep(5000);
    }
    return 0;
}

// ==================== 速度控制接口 ====================
void setSpeedLevel(int level) {
    if (level < 0) level = 0;
    if (level > 4) level = 4;
    speedLevel = level;
}

int getSpeedLevel(void) {
    return speedLevel;
}

// ==================== 延时控制（随速度变化） ====================
static void safeSleep(uint64_t microseconds) {
    static const double speedTable[5] = {1.5, 1.0, 0.8, 0.6, 0.4}; // 每档延时倍率
    double factor = speedTable[speedLevel];
    uint64_t adjusted = (uint64_t)(microseconds * factor);

    uint64_t elapsed = 0;
    while (elapsed < adjusted && !stopFlag) {
        usleep(1000);
        elapsed += 1000;
    }
}

// ==================== 动作控制通用 ====================
// 移除阻塞式的 startMotion/endMotion，改为非阻塞的状态标记
// 由外部任务控制循环

void stopCurrentMotion(void) {
    stopFlag = 1;
}

void resetStopFlag(void) {
    stopFlag = 0;
}

int isStopFlag(void) {
    return stopFlag;
}

// ==================== 初始化姿态 ====================
void init_dog(double step_length, double step_height) {
    // 初始化时忽略 stopFlag，强制复位
    int init[20][2];
    generateGaitTable20(init, step_length, step_height);

    int thigh = init[0][0], shank = init[0][1];
    setServo(LF_THIGH, thigh); safeSleep(5000);
    setServo(LF_SHANK, shank);
    setServo(RH_THIGH, -thigh); safeSleep(5000);
    setServo(RH_SHANK, -shank);
    setServo(LH_THIGH, -thigh); safeSleep(5000);
    setServo(LH_SHANK, -shank);
    setServo(RF_THIGH, thigh); safeSleep(5000);
    setServo(RF_SHANK, shank);
}

static void buildStandPose(int target[SERVO_CHANNEL_COUNT],
                           double step_length, double step_height)
{
    int init[20][2];
    generateGaitTable20(init, step_length, step_height);

    int thigh = init[0][0], shank = init[0][1];
    target[LF_THIGH] = thigh;
    target[LF_SHANK] = shank;
    target[RH_THIGH] = -thigh;
    target[RH_SHANK] = -shank;
    target[LH_THIGH] = -thigh;
    target[LH_SHANK] = -shank;
    target[RF_THIGH] = thigh;
    target[RF_SHANK] = shank;
}

int smooth_stop_to_stand(double step_length, double step_height)
{
    int start[SERVO_CHANNEL_COUNT] = {0};
    int target[SERVO_CHANNEL_COUNT] = {0};
    const int steps = 12;
    const int stepDelayUs = 15000;

    buildStandPose(target, step_length, step_height);

    for (int k = 0; k < ACTIVE_SERVO_COUNT; k++) {
        int channel = activeServoChannels[k];
        start[channel] = servoAngleValid[channel] ? servoAngles[channel] : target[channel];
    }

    for (int step = 1; step <= steps; step++) {
        if (!stopFlag) {
            return 0;
        }

        for (int k = 0; k < ACTIVE_SERVO_COUNT; k++) {
            int channel = activeServoChannels[k];
            int delta = target[channel] - start[channel];
            int angle = start[channel] + delta * step / steps;
            setServo(channel, angle);
        }
        usleep(stepDelayUs);
    }

    return 1;
}

// ==================== 单周期动作函数 ====================

// 执行一个周期的前进动作
// 返回值：1=完成，0=被中断
int trot_cycle(double step_length, double step_height) {
    int gait[20][2];
    generateGaitTable20(gait, step_length, step_height);

    int N = 20, phase = N / 2;
    for (int i = 0; i < N; i++) {
        if (stopFlag) return 0; // 检测停止标志

        int thigh1 = gait[i][0], shank1 = gait[i][1];
        int j = (i + phase) % N;
        int thigh2 = gait[j][0], shank2 = gait[j][1];

        setServo(LF_THIGH, thigh1);
        setServo(LF_SHANK, shank1);
        setServo(RH_THIGH, -thigh1);
        setServo(RH_SHANK, -shank1);
        setServo(LH_THIGH, -thigh2);
        setServo(LH_SHANK, -shank2);
        setServo(RF_THIGH, thigh2);
        setServo(RF_SHANK, shank2);
        safeSleep(30000);
    }
    return 1;
}

// 执行一个周期的后退动作
int trot_back_cycle(double step_length, double step_height) {
    int gait[20][2];
    generateGaitTable20(gait, step_length, step_height);

    int N = 20, phase = N / 2;
    for (int i = N - 1; i >= 0; i--) {
        if (stopFlag) return 0;

        int thigh1 = gait[i][0], shank1 = gait[i][1];
        int j = (i + phase) % N;
        int thigh2 = gait[j][0], shank2 = gait[j][1];

        setServo(LF_THIGH, thigh1);
        setServo(LF_SHANK, shank1);
        setServo(RH_THIGH, -thigh1);
        setServo(RH_SHANK, -shank1);
        setServo(LH_THIGH, -thigh2);
        setServo(LH_SHANK, -shank2);
        setServo(RF_THIGH, thigh2);
        setServo(RF_SHANK, shank2);
        safeSleep(30000);
    }
    return 1;
}

// 执行一个周期的右转动作
int diversion_right_cycle(double step_length, double step_height) {
    int gait[20][2], gait_r[20][2];
    generateGaitTable20(gait, step_length, step_height);
    generateGaitTable20_reversed(gait_r, step_length, step_height);

    int N = 20, phaseShift = N / 4;
    for (int i = 0; i < N; i++) {
        if (stopFlag) return 0;

        setServo(LF_THIGH, gait[i][0]);
        setServo(LF_SHANK, gait[i][1]);
        int j1 = (i + phaseShift) % N;
        setServo(LH_THIGH, -gait_r[j1][0]);
        setServo(LH_SHANK, -gait_r[j1][1]);
        int j2 = (i + 2 * phaseShift) % N;
        setServo(RH_THIGH, -gait_r[j2][0]);
        setServo(RH_SHANK, -gait_r[j2][1]);
        int j3 = (i + 3 * phaseShift) % N;
        setServo(RF_THIGH, gait[j3][0]);
        setServo(RF_SHANK, gait[j3][1]);
        safeSleep(20000);
    }
    return 1;
}

// 执行一个周期的左转动作
int diversion_left_cycle(double step_length, double step_height) {
    int gait[20][2], gait_r[20][2];
    generateGaitTable20_reversed(gait, step_length, step_height);
    generateGaitTable20(gait_r, step_length, step_height);

    int N = 20, phaseShift = N / 4;
    for (int i = 0; i < N; i++) {
        if (stopFlag) return 0;

        setServo(LF_THIGH, gait[i][0]);
        setServo(LF_SHANK, gait[i][1]);
        int j1 = (i + phaseShift) % N;
        setServo(LH_THIGH, -gait_r[j1][0]);
        setServo(LH_SHANK, -gait_r[j1][1]);
        int j2 = (i + 2 * phaseShift) % N;
        setServo(RH_THIGH, -gait_r[j2][0]);
        setServo(RH_SHANK, -gait_r[j2][1]);
        int j3 = (i + 3 * phaseShift) % N;
        setServo(RF_THIGH, gait[j3][0]);
        setServo(RF_SHANK, gait[j3][1]);
        safeSleep(20000);
    }
    return 1;
}

static void generateDirectionalGait(int gait[20][2], double step_length,
                                    double step_height, int backward)
{
    if (backward) {
        generateGaitTable20_reversed(gait, step_length, step_height);
    } else {
        generateGaitTable20(gait, step_length, step_height);
    }
}

// 执行一个周期的斜向弧线动作
// 不是动作拼接：在同一个 trot 周期里通过左右侧步幅差实现转向趋势。
static int trot_arc_cycle(double step_length, double step_height,
                          int backward, int leftArc)
{
    int gaitLeft[20][2], gaitRight[20][2];
    const double innerScale = 0.45;
    const double outerScale = 1.00;
    double leftStep = leftArc ? step_length * innerScale : step_length * outerScale;
    double rightStep = leftArc ? step_length * outerScale : step_length * innerScale;

    generateDirectionalGait(gaitLeft, leftStep, step_height, backward);
    generateDirectionalGait(gaitRight, rightStep, step_height, backward);

    int N = 20, phase = N / 2;
    for (int i = 0; i < N; i++) {
        if (stopFlag) return 0;

        int j = (i + phase) % N;

        setServo(LF_THIGH, gaitLeft[i][0]);
        setServo(LF_SHANK, gaitLeft[i][1]);
        setServo(RH_THIGH, -gaitRight[i][0]);
        setServo(RH_SHANK, -gaitRight[i][1]);
        setServo(LH_THIGH, -gaitLeft[j][0]);
        setServo(LH_SHANK, -gaitLeft[j][1]);
        setServo(RF_THIGH, gaitRight[j][0]);
        setServo(RF_SHANK, gaitRight[j][1]);
        safeSleep(30000);
    }
    return 1;
}

int trot_left_front_cycle(double step_length, double step_height) {
    return trot_arc_cycle(step_length, step_height, 0, 1);
}

int trot_right_front_cycle(double step_length, double step_height) {
    return trot_arc_cycle(step_length, step_height, 0, 0);
}

int trot_left_back_cycle(double step_length, double step_height) {
    return trot_arc_cycle(step_length, step_height, 1, 1);
}

int trot_right_back_cycle(double step_length, double step_height) {
    return trot_arc_cycle(step_length, step_height, 1, 0);
}

// 保留旧接口以兼容（如果需要），或者直接删除旧接口
// 这里我们删除旧的 trot/trot_back 等循环函数，因为我们要用 Task 控制
