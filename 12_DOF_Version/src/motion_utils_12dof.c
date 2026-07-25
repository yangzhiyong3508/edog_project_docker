#include "../include/motion_utils_12dof.h"
#include "../include/gait_generate_12dof.h"
#include "../../utils/include/mpu6050_motion_light.h"
#include "../../utils/include/servo_control.h"
#include "../../include/edog_config.h"
#include "kv_store.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>

#define EDOG_12DOF_IMU_CENTI_PER_DEG 100
#define EDOG_SERVO_TRIM_RESET_MIGRATION_KEY "servo_trim_real_angle_v1"
#define EDOG_SERVO_TRIM_RESET_MIGRATION_DONE "1"
#define EDOG_RUNTIME_FRONT_HIP_KEY "edog.rt.front_hip"
#define EDOG_RUNTIME_REAR_HIP_KEY "edog.rt.rear_hip"
#define EDOG_RUNTIME_FRONT_BODY_HEIGHT_KEY "edog.rt.front_body_height"
#define EDOG_RUNTIME_REAR_BODY_HEIGHT_KEY "edog.rt.rear_body_height"
#define EDOG_RUNTIME_IMU_STRENGTH_KEY "edog.rt.imu_strength"

#if defined(__GNUC__)
#define EDOG_UNUSED __attribute__((unused))
#else
#define EDOG_UNUSED
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

typedef struct {
    int hip;
    int thigh;
    int calf;
    int isRightLeg;
} Edog12DofLeg;

typedef struct {
    int channel;
    const char *name;
} Edog12DofJoint;

typedef Edog12DofJointAngles Edog12DofGaitSet[EDOG_12DOF_LEG_COUNT][EDOG_12DOF_TROT_FRAME_COUNT];

typedef struct {
    int centiDeg[EDOG_SERVO_CHANNEL_COUNT];
} Edog12DofServoFrame;

typedef struct {
    Edog12DofServoFrame frames[EDOG_12DOF_GAIT_FRAME_COUNT];
    Edog12DofFootPoint baseFeet[EDOG_12DOF_GAIT_FRAME_COUNT][EDOG_12DOF_LEG_COUNT];
    unsigned char isSwing[EDOG_12DOF_GAIT_FRAME_COUNT][EDOG_12DOF_LEG_COUNT];
    int frameCount;
    int minFrameMs;
    int minCycleMs;
} Edog12DofServoGaitTable;

typedef struct {
    int mode;
    int baseIndex;
    int phaseOffset[EDOG_12DOF_LEG_COUNT];
    int cycleMs;
    int frameMs;
    uint64_t lastUpdateMs;
    int active;
    int legMask;
    int stepLengthCentiMm;
    int stepHeightCentiMm;
} Edog12DofTableRuntime;

typedef struct {
    int active;
    int mode;
    int frameIndex;
    int stepLengthCentiMm;
    int stepHeightCentiMm;
    int backward;
    int leftScalePercent;
    int rightScalePercent;
    int turnLeft;
} Edog12DofContinuousTrotRuntime;

typedef enum {
    EDOG_TABLE_MODE_NONE = 0,
    EDOG_TABLE_MODE_FORWARD,
    EDOG_TABLE_MODE_BACKWARD,
    EDOG_TABLE_MODE_TURN_LEFT,
    EDOG_TABLE_MODE_TURN_RIGHT,
    EDOG_TABLE_MODE_IN_PLACE,
} Edog12DofTableMode;

typedef enum {
    EDOG_IMU_BALANCE_MODE_NORMAL = 0,
    EDOG_IMU_BALANCE_MODE_STOP_SETTLING,
    EDOG_IMU_BALANCE_MODE_RAMP_IN,
} EdogImuBalanceMode;

typedef struct {
    double vxMps;
    double vyMps;
    double yawRate;
    double stepHeightM;
    int crawlMode;
    int backward;
} Edog12DofRealtimeCommand;

typedef struct {
    Edog12DofFootPoint foot;
    Edog12DofFootPoint liftOff;
    Edog12DofFootPoint touchdown;
    int isSwing;
} Edog12DofRealtimeLegState;

typedef struct {
    Edog12DofRealtimeLegState leg[EDOG_12DOF_LEG_COUNT];
} Edog12DofRealtimeGaitState;

typedef enum {
    EDOG_IMU_BALANCE_STATUS_OK = 0,
    EDOG_IMU_BALANCE_STATUS_DISABLED,
    EDOG_IMU_BALANCE_STATUS_ARG_ERROR,
    EDOG_IMU_BALANCE_STATUS_INIT_WAIT,
    EDOG_IMU_BALANCE_STATUS_INIT_FAIL,
    EDOG_IMU_BALANCE_STATUS_READ_FAIL,
    EDOG_IMU_BALANCE_STATUS_TILT_REJECT,
} EdogImuBalanceStatus;

static const Edog12DofJointAngles g_standJointAngles = {
    EDOG_12DOF_STAND_HIP_DELTA_DEG,
    EDOG_12DOF_STAND_THIGH_DELTA_DEG,
    EDOG_12DOF_STAND_CALF_DELTA_DEG,
    /* 厘度字段必须显式初始化，否则静态零初始化会把站姿误设成 0.00°。 */
    EDOG_12DOF_STAND_HIP_DELTA_DEG * EDOG_12DOF_CENTI_PER_DEG,
    EDOG_12DOF_STAND_THIGH_DELTA_DEG * EDOG_12DOF_CENTI_PER_DEG,
    EDOG_12DOF_STAND_CALF_DELTA_DEG * EDOG_12DOF_CENTI_PER_DEG,
};

static const Edog12DofLeg g_legs[EDOG_12DOF_LEG_COUNT] = {
    {LF_HIP, LF_THIGH, LF_CALF, 0},
    {RF_HIP, RF_THIGH, RF_CALF, 1},
    {LB_HIP, LB_THIGH, LB_CALF, 0},
    {RB_HIP, RB_THIGH, RB_CALF, 1},
};

/* Power-on standing sequence: rear legs first (LB -> RB), then front legs (LF -> RF). */
static const int g_startupLegOrder[EDOG_12DOF_LEG_COUNT] = {2, 3, 0, 1};

/* Static crawl swing order: LF -> RB -> RF -> LB, one swing leg at a time. */
static const int g_crawlSwingOrder[EDOG_12DOF_LEG_COUNT] = {0, 3, 1, 2};

static const int g_pupperPhaseTicks[EDOG_PUPPER_NUM_PHASES] = {
    EDOG_PUPPER_OVERLAP_TICKS,
    EDOG_PUPPER_SWING_TICKS,
    EDOG_PUPPER_OVERLAP_TICKS,
    EDOG_PUPPER_SWING_TICKS,
};

static const unsigned char g_pupperContactPhases[EDOG_12DOF_LEG_COUNT][EDOG_PUPPER_NUM_PHASES] = {
    {1, 0, 1, 1}, /* LF */
    {1, 1, 1, 0}, /* RF */
    {1, 1, 1, 0}, /* LB */
    {1, 0, 1, 1}, /* RB */
};

static const int g_staticCrawlPhaseTicks[EDOG_STATIC_CRAWL_NUM_PHASES] = {
    EDOG_STATIC_CRAWL_OVERLAP_TICKS,
    EDOG_STATIC_CRAWL_SWING_TICKS,
    EDOG_STATIC_CRAWL_OVERLAP_TICKS,
    EDOG_STATIC_CRAWL_SWING_TICKS,
    EDOG_STATIC_CRAWL_OVERLAP_TICKS,
    EDOG_STATIC_CRAWL_SWING_TICKS,
    EDOG_STATIC_CRAWL_OVERLAP_TICKS,
    EDOG_STATIC_CRAWL_SWING_TICKS,
};

static const unsigned char g_staticCrawlContactPhases[EDOG_12DOF_LEG_COUNT][EDOG_STATIC_CRAWL_NUM_PHASES] = {
    {1, 0, 1, 1, 1, 1, 1, 1}, /* LF */
    {1, 1, 1, 1, 1, 0, 1, 1}, /* RF */
    {1, 1, 1, 1, 1, 1, 1, 0}, /* LB */
    {1, 1, 1, 0, 1, 1, 1, 1}, /* RB */
};

static int getCrawlOrderPosition(int legIndex)
{
    for (int i = 0; i < EDOG_12DOF_LEG_COUNT; i++) {
        if (g_crawlSwingOrder[i] == legIndex) {
            return i;
        }
    }
    return 0;
}

static void applyCrawlBalanceBias(Edog12DofJointAngles frame[EDOG_12DOF_LEG_COUNT],
                                  const int swingState[EDOG_12DOF_LEG_COUNT],
                                  int swingLeg);
static void applyHipWeightShiftBias(Edog12DofJointAngles frame[EDOG_12DOF_LEG_COUNT],
                                    const int swingState[EDOG_12DOF_LEG_COUNT],
                                    int targetLeg, int biasDeg);
static void apply8DofMotionOutputSign(int legIndex, Edog12DofJointAngles *angles);

/* 12 个活动关节，预留通道 3/7/11/15 不参与步态输出。 */
static const Edog12DofJoint g_activeJoints[EDOG_12DOF_ACTIVE_SERVO_COUNT] = {
    {LF_HIP, "LF_HIP"}, {LF_THIGH, "LF_THIGH"}, {LF_CALF, "LF_CALF"},
    {RF_HIP, "RF_HIP"}, {RF_THIGH, "RF_THIGH"}, {RF_CALF, "RF_CALF"},
    {LB_HIP, "LB_HIP"}, {LB_THIGH, "LB_THIGH"}, {LB_CALF, "LB_CALF"},
    {RB_HIP, "RB_HIP"}, {RB_THIGH, "RB_THIGH"}, {RB_CALF, "RB_CALF"},
};

/*
 * 每通道物理安装方向表。
 *
 * 运动学正方向：
 *   Hip   : 向外张开为正
 *   Thigh : 向前摆动为正
 *   Calf  : 向后折叠为正
 *
 * 物理装配方向：
 *   Hip 按左右胯关节实际安装方向镜像；
 *   Thigh/Calf 按左右腿舵盘朝向镜像。
 *
 * 注意：这张表是站立/停止/前进共用的底层物理映射，
 * 不要在这里套用 8DOF 步态层的前后腿符号，否则会把正确站姿一起翻掉。
 *
 * 输出公式：
 *   servoAngle = EDOG_SERVO_CENTER_ANGLE + g_servoDirection[channel] * jointDeltaDeg
 */
static const int g_servoDirection[EDOG_SERVO_CHANNEL_COUNT] = {
     1, -1, -1,  0,
    -1,  1,  1,  0,
     1, -1, -1,  0,
    -1,  1,  1,  0,
};

static volatile int stopFlag = 0;
static volatile int speedLevel = 0;
static int g_servoCenterTrim[EDOG_SERVO_CHANNEL_COUNT] = {0};
static int servoAngles[EDOG_SERVO_CHANNEL_COUNT] = {0};
static unsigned char servoAngleValid[EDOG_SERVO_CHANNEL_COUNT] = {0};
static int g_imuBalanceStrengthPercent = 100;
/*
 * 厘度(1/100 度)精度的舵机姿态累加器，是 slew 滤波与 PCA9685 输出的真实状态。
 * servoAngles[] 仍同步保留整数度，供调试读数与停止回站姿等旧逻辑使用。
 */
static int servoAnglesCenti[EDOG_SERVO_CHANNEL_COUNT] = {0};
static int servoVelocityCentiPerFrame[EDOG_SERVO_CHANNEL_COUNT] = {0};
static Edog12DofServoGaitTable g_servoGaitTable;
static Edog12DofGaitSet g_tableBuildGaitSet;
static Edog12DofGaitSet g_legacyMotionGaitSet;
static Edog12DofGaitSet g_legacyTurnForwardGaitSet;
static Edog12DofGaitSet g_legacyTurnReverseGaitSet;
static Edog12DofContinuousTrotRuntime g_continuousTrotRuntime = {
    0,
    EDOG_TABLE_MODE_NONE,
    0,
    0,
    0,
    0,
    100,
    100,
    0,
};
static Edog12DofTableRuntime g_servoTableRuntime = {
    EDOG_TABLE_MODE_NONE,
    0,
    {0, 0, 0, 0},
    0,
    0,
    0,
    0,
    0x0F,
    0,
    0,
};
#if EDOG_12DOF_IMU_BALANCE_ENABLED
static int g_balanceRollCentiDeg = 0;
static int g_balancePitchCentiDeg = 0;
static int g_balanceRollRateCentiDps = 0;
static int g_balancePitchRateCentiDps = 0;
static double g_balanceRollDeg = 0.0;
static double g_balancePitchDeg = 0.0;
static unsigned int g_balanceFusionSampleCount = 0;
static unsigned char g_balanceFilterReady = 0;
static unsigned char g_balanceMpuReady = 0;
static int g_balanceMpuInitRetryFrames = 0;
#endif
static int g_balancePrevRollErrorCentiDeg = 0;
static int g_balancePrevPitchErrorCentiDeg = 0;
static int g_balanceRollControlCentiDeg = 0;
static int g_balancePitchControlCentiDeg = 0;
static int g_balanceRollIntegralCentiDeg = 0;
static int g_balancePitchIntegralCentiDeg = 0;
static int g_balanceLastRollOutputCentiDeg = 0;
static int g_balanceLastPitchOutputCentiDeg = 0;
static unsigned char g_balanceControlReady = 0;
static int g_balanceLastReadStatus = EDOG_IMU_BALANCE_STATUS_DISABLED;
static EdogImuBalanceMode g_balanceMode = EDOG_IMU_BALANCE_MODE_NORMAL;
static uint64_t g_balanceModeStartMs = 0;
static int g_balanceStableFrames = 0;
#if EDOG_12DOF_IMU_BALANCE_DEBUG_ENABLED
static int g_balanceDebugLastStatus = -1;
#endif

static int clampServoAngleCenti(int centiAngle);
static int writeServoPhysicalAngleCenti(int channel, int centiAngle);
static int writeServoProfileAngleCenti(int channel, int centiAngle);
static int planServoTrapezoidStepCenti(int channel, int targetCenti);
static int getSafeServoCentiDegPerMs(int channel);
static int approachServoTargetCenti(int currentCenti, int targetCenti, int maxStepCenti);
static int servoTableCentiToTrimmedTarget(int channel, int tableCenti);
static int normalizeImuBalanceStrengthPercent(int strengthPercent);
static double computeImuBodyZUpCompMm(int leg, int rollCentiDeg,
                                      int pitchCentiDeg, int isSwing);
static void applyImuBalanceFootCompensation(Edog12DofFootPoint *foot, int leg,
                                            int isSwing, int rollCentiDeg,
                                            int pitchCentiDeg);
static int motionSetServoPhysicalNoTrimCenti(int channel, int centiAngle);
static int computeServoTableMinFrameMs(const Edog12DofServoGaitTable *table);
static void buildServoGaitTable(Edog12DofServoGaitTable *table,
                                int mode, double stepLength, double stepHeight);
static int runServoGaitTableCycle(int mode, double stepLength,
                                  double stepHeight, int legMask);
static int doubleToCentiMm(double value);
static const char *getImuBalanceStatusName(int status);
static void printImuBalanceDebugFrame(int status,
                                      int rollControlCentiDeg,
                                      int pitchControlCentiDeg,
                                      const Edog12DofFootPoint feet[EDOG_12DOF_LEG_COUNT],
                                      const Edog12DofJointAngles frame[EDOG_12DOF_LEG_COUNT]);
static int computeImuTiltCentiDeg(int xMg, int yMg, int zMg,
                                  int *rollCentiDeg, int *pitchCentiDeg);
static int readImuBalanceMotionCenti(int *rollCentiDeg, int *pitchCentiDeg,
                                     int *rollRateCentiDps, int *pitchRateCentiDps);
static int readImuBalanceTiltCentiDeg(int *rollCentiDeg, int *pitchCentiDeg);
static int imuFusionSamplesPerControlFrame(void);
static int roundDoubleToCentiDeg(double valueDeg);
static int updateImuFusionSample(int *lastStatus);
static void prepareImuBalanceControlCentiDeg(int measuredRollCentiDeg,
                                             int measuredPitchCentiDeg,
                                             int rollRateCentiDps,
                                             int pitchRateCentiDps,
                                             int *rollControlCentiDeg,
                                             int *pitchControlCentiDeg);
static void clearImuBalanceControlState(void);
static void enterImuBalanceMode(EdogImuBalanceMode mode);
static int imuBalanceMotionIsStable(int rollCentiDeg, int pitchCentiDeg,
                                    int rollRateCentiDps, int pitchRateCentiDps);
static int imuBalanceMotionExceedsExit(int rollCentiDeg, int pitchCentiDeg,
                                       int rollRateCentiDps, int pitchRateCentiDps);
static int limitImuControlStep(int previousCentiDeg, int targetCentiDeg,
                               int maxStepCentiDeg);
static void applyImuBalanceRampScale(int *rollControlCentiDeg,
                                     int *pitchControlCentiDeg,
                                     uint64_t elapsedMs);
static void getImuBalanceControlCentiDeg(int *rollControlCentiDeg,
                                         int *pitchControlCentiDeg);
static Edog12DofFootPoint sampleDirectionalFootPoint(double forwardStepM,
                                                     double sideStepM,
                                                     double yawStepM,
                                                     double stepHeightM,
                                                     int isRightLeg,
                                                     int isFrontLeg,
                                                     int legIndex,
                                                     int reversed,
                                                     int frameIndex,
                                                     int *isSwingOut);
static Edog12DofFootPoint getDefaultFootPoint(int leg);
static Edog12DofJointAngles getStandJointAnglesForLeg(int leg);
static void fillStandFrame(Edog12DofJointAngles frame[EDOG_12DOF_LEG_COUNT]);

static void resetServoMotionProfile(int channel, int limited)
{
    if (channel < 0 || channel >= EDOG_SERVO_CHANNEL_COUNT) {
        return;
    }
    servoVelocityCentiPerFrame[channel] = 0;
    servoAnglesCenti[channel] = limited;
    servoAngles[channel] = (limited + (limited >= 0 ? 50 : -50)) /
        EDOG_12DOF_CENTI_PER_DEG;
    servoAngleValid[channel] = 1;
}

static void buildServoTrimKey(int channel, char *key, size_t keyLen)
{
    snprintf(key, keyLen, "edog.trim.%02d", channel);
}

static int clampServoAngle(int angle)
{
    if (angle > EDOG_SERVO_MAX_ANGLE) {
        return EDOG_SERVO_MAX_ANGLE;
    }
    if (angle < EDOG_SERVO_MIN_ANGLE) {
        return EDOG_SERVO_MIN_ANGLE;
    }
    return angle;
}

static int clampJointDeltaDeg(int angle)
{
    if (angle > 90) {
        return 90;
    }
    if (angle < -90) {
        return -90;
    }
    return angle;
}

static int clampJointDeltaCenti(int centiAngle)
{
    if (centiAngle > 9000) {
        return 9000;
    }
    if (centiAngle < -9000) {
        return -9000;
    }
    return centiAngle;
}

/*
 * 后腿摆动相关节空间补偿。
 * 后腿因 REAR_FOOT_UP_OFFSET 伸直后，IK 对抬腿的灵敏度下降，
 * 在摆动相把关节增量（相对站立姿态）乘以补偿倍数，使后腿抬腿幅度与前腿一致。
 */
static void applyRearLegSwingBoost(int legIndex, int isSwing,
                                   Edog12DofJointAngles *angles)
{
    int isFrontLeg;
    int isRightLeg;
    double baseZMm;
    Edog12DofFootPoint stanceFoot;
    Edog12DofJointAngles stanceAngles;

    if (angles == NULL || !isSwing) {
        return;
    }

    isFrontLeg = (legIndex == 0 || legIndex == 1);
    if (isFrontLeg) {
        return;
    }

    isRightLeg = (legIndex == 1 || legIndex == 3);
    baseZMm = Edog12Dof_DefaultFootZForLeg(isFrontLeg);
    stanceFoot.xMm = Edog12Dof_DefaultFootXForLeg(isFrontLeg);
    stanceFoot.zMm = baseZMm + EDOG_12DOF_STANCE_PRESS_MM;
    stanceFoot.yMm = Edog12Dof_FootYOnHipPlaneForLeg(legIndex, stanceFoot.zMm);

    if (Edog12Dof_IK(&stanceFoot, isRightLeg, &stanceAngles) != 0) {
        return;
    }

    {
        int deltaFemurCenti = angles->femurAngleCentiDeg - stanceAngles.femurAngleCentiDeg;
        int deltaTibiaCenti = angles->tibiaAngleCentiDeg - stanceAngles.tibiaAngleCentiDeg;
        angles->femurAngleCentiDeg = clampJointDeltaCenti(stanceAngles.femurAngleCentiDeg +
            deltaFemurCenti * EDOG_12DOF_REAR_FEMUR_BOOST_NUM / EDOG_12DOF_REAR_FEMUR_BOOST_DEN);
        angles->tibiaAngleCentiDeg = clampJointDeltaCenti(stanceAngles.tibiaAngleCentiDeg +
            deltaTibiaCenti * EDOG_12DOF_REAR_TIBIA_BOOST_NUM / EDOG_12DOF_REAR_TIBIA_BOOST_DEN);
        angles->femurAngleDeg = clampJointDeltaDeg((int)((double)angles->femurAngleCentiDeg / 100.0 +
            (angles->femurAngleCentiDeg >= 0 ? 0.5 : -0.5)));
        angles->tibiaAngleDeg = clampJointDeltaDeg((int)((double)angles->tibiaAngleCentiDeg / 100.0 +
            (angles->tibiaAngleCentiDeg >= 0 ? 0.5 : -0.5)));
    }
}

static int jointDeltaToServoAngleCenti(int channel, int jointDeltaCentiDeg)
{
    int centerCenti;
    int servoCenti;

    if (channel < 0 || channel >= EDOG_SERVO_CHANNEL_COUNT) {
        return EDOG_SERVO_CENTER_ANGLE * EDOG_12DOF_CENTI_PER_DEG;
    }
    /* servoAngle = CENTER + dir * jointDelta，整套换算放大 100 倍到厘度。 */
    centerCenti = EDOG_SERVO_CENTER_ANGLE * EDOG_12DOF_CENTI_PER_DEG;
    servoCenti = centerCenti + g_servoDirection[channel] * jointDeltaCentiDeg;
    if (servoCenti > EDOG_SERVO_MAX_ANGLE * EDOG_12DOF_CENTI_PER_DEG) {
        servoCenti = EDOG_SERVO_MAX_ANGLE * EDOG_12DOF_CENTI_PER_DEG;
    }
    if (servoCenti < EDOG_SERVO_MIN_ANGLE * EDOG_12DOF_CENTI_PER_DEG) {
        servoCenti = EDOG_SERVO_MIN_ANGLE * EDOG_12DOF_CENTI_PER_DEG;
    }
    return servoCenti;
}

static int jointDeltaToServoAngle(int channel, int jointDeltaDeg)
{
    if (channel < 0 || channel >= EDOG_SERVO_CHANNEL_COUNT) {
        return EDOG_SERVO_CENTER_ANGLE;
    }
    return clampServoAngle(EDOG_SERVO_CENTER_ANGLE + g_servoDirection[channel] * jointDeltaDeg);
}

static int applyServoCenterTrim(int channel, int angle)
{
    if (channel < 0 || channel >= EDOG_SERVO_CHANNEL_COUNT) {
        return clampServoAngle(angle);
    }
    /* 中位校准偏移在最终物理角度上叠加，所有站立/步态/单舵机输出共用。 */
    return clampServoAngle(angle + g_servoCenterTrim[channel]);
}

static int writeServoPhysicalAngle(int channel, int angle)
{
    int limited = clampServoAngle(angle);
    int limitedCenti = limited * EDOG_12DOF_CENTI_PER_DEG;
    int ret = setServoCentiDeg(channel, limitedCenti);

    if (ret != 0) {
        return ret;
    }
    if (channel >= 0 && channel < EDOG_SERVO_CHANNEL_COUNT) {
        servoAngles[channel] = limited;
        /* 同步厘度累加器，保证整数度/厘度两条写入路径状态自洽，
         * 否则停止回站姿(整数路径)后首帧 trot 的 slew 增量会算错。 */
        servoAnglesCenti[channel] = limitedCenti;
        resetServoMotionProfile(channel, limitedCenti);
        servoAngleValid[channel] = 1;
    }
    return 0;
}

static int motionSetServoPhysical(int channel, int angle)
{
    return writeServoPhysicalAngle(channel, applyServoCenterTrim(channel, clampServoAngle(angle)));
}

static int motionSetServoPhysicalSmooth(int channel, int angle)
{
    int target = applyServoCenterTrim(channel, clampServoAngle(angle));
    return writeServoProfileAngleCenti(
        channel, planServoTrapezoidStepCenti(
            channel, target * EDOG_12DOF_CENTI_PER_DEG));
}

static int motionSetServo(int channel, int jointDeltaDeg)
{
    return motionSetServoPhysical(channel, jointDeltaToServoAngle(channel, jointDeltaDeg));
}

static int motionSetServoSmooth(int channel, int jointDeltaDeg)
{
    return motionSetServoPhysicalSmooth(channel, jointDeltaToServoAngle(channel, jointDeltaDeg));
}

/* ==================== 厘度(亚度精度)输出链 ==================== */

static int clampServoAngleCenti(int centiAngle)
{
    int lo = EDOG_SERVO_MIN_ANGLE * EDOG_12DOF_CENTI_PER_DEG;
    int hi = EDOG_SERVO_MAX_ANGLE * EDOG_12DOF_CENTI_PER_DEG;

    if (centiAngle > hi) {
        return hi;
    }
    if (centiAngle < lo) {
        return lo;
    }
    return centiAngle;
}

static int applyServoCenterTrimCenti(int channel, int centiAngle)
{
    if (channel < 0 || channel >= EDOG_SERVO_CHANNEL_COUNT) {
        return clampServoAngleCenti(centiAngle);
    }
    /* 中位校准偏移仍以整数度存储，这里换算到厘度叠加。 */
    return clampServoAngleCenti(centiAngle +
                                g_servoCenterTrim[channel] * EDOG_12DOF_CENTI_PER_DEG);
}

static int servoTableCentiToTrimmedTarget(int channel, int tableCenti)
{
    return applyServoCenterTrimCenti(channel, tableCenti);
}

static int writeServoPhysicalAngleCenti(int channel, int centiAngle)
{
    int limited = clampServoAngleCenti(centiAngle);
    int ret = setServoCentiDeg(channel, limited);

    if (ret != 0) {
        return ret;
    }
    if (channel >= 0 && channel < EDOG_SERVO_CHANNEL_COUNT) {
        servoAnglesCenti[channel] = limited;
        /* 同步整数度（四舍五入），供调试读数与停止回站姿逻辑使用。 */
        servoAngles[channel] = (limited + (limited >= 0 ? 50 : -50)) / EDOG_12DOF_CENTI_PER_DEG;
        resetServoMotionProfile(channel, limited);
    }
    return 0;
}

static int motionSetServoPhysicalNoTrimCenti(int channel, int centiAngle)
{
    return writeServoProfileAngleCenti(channel, clampServoAngleCenti(centiAngle));
}

static int writeServoProfileAngleCenti(int channel, int centiAngle)
{
    int limited = clampServoAngleCenti(centiAngle);
    int ret = setServoCentiDeg(channel, limited);

    if (ret != 0) {
        return ret;
    }
    if (channel >= 0 && channel < EDOG_SERVO_CHANNEL_COUNT) {
        servoAnglesCenti[channel] = limited;
        servoAngles[channel] = (limited + (limited >= 0 ? 50 : -50)) /
            EDOG_12DOF_CENTI_PER_DEG;
        servoAngleValid[channel] = 1;
    }
    return 0;
}

static int isHipServoChannel(int channel)
{
    return channel == LF_HIP || channel == RF_HIP ||
        channel == LB_HIP || channel == RB_HIP;
}

static int getServoSpeed60DegMsForChannel(int channel)
{
    return isHipServoChannel(channel) ? EDOG_SERVO_HIP_SPEED_60_DEG_MS :
        EDOG_SERVO_LEG_SPEED_60_DEG_MS;
}

static int getSafeServoCentiDegPerMs(int channel)
{
    const int speedMs = getServoSpeed60DegMsForChannel(channel);
    const int rated = (60 * EDOG_12DOF_CENTI_PER_DEG + speedMs - 1) /
        speedMs;
    int safe = rated * EDOG_SERVO_SPEED_SAFETY_NUM / EDOG_SERVO_SPEED_SAFETY_DEN;

    return safe > 0 ? safe : 1;
}

static int approachServoTargetCenti(int currentCenti, int targetCenti, int maxStepCenti)
{
    int delta = targetCenti - currentCenti;

    if (maxStepCenti < 1) {
        maxStepCenti = 1;
    }
    if (delta > maxStepCenti) {
        return currentCenti + maxStepCenti;
    }
    if (delta < -maxStepCenti) {
        return currentCenti - maxStepCenti;
    }
    return targetCenti;
}

/*
 * 厘度 slew 限速：next = cur + clamp(delta * NUM/DEN, +/- MAX_STEP)
 * 单位全部为厘度。关键修复：当 |delta| 不足以产生整数度步进时，
 * 最小步进取 ±1 厘度(0.01°)，远细于舵机分辨率，因此能平滑收敛，
 * 不再像整数度版本那样被迫 ±1° 反复过冲（支撑相抖动的根因）。
 */
static int planServoTrapezoidStepCenti(int channel, int targetCenti)
{
    int current;
    int target;
    int distance;
    int direction;
    int distanceAbs;
    int velocity;
    int velocityAbs;
    int nextVelocity;
    int next;
    long long brakeDistance;

    if (channel < 0 || channel >= EDOG_SERVO_CHANNEL_COUNT) {
        return clampServoAngleCenti(targetCenti);
    }
    if (!servoAngleValid[channel]) {
        servoVelocityCentiPerFrame[channel] = 0;
        return clampServoAngleCenti(targetCenti);
    }

    current = servoAnglesCenti[channel];
    target = clampServoAngleCenti(targetCenti);
    distance = target - current;
    distanceAbs = distance >= 0 ? distance : -distance;
    if (distanceAbs <= EDOG_SERVO_TRAPEZOID_TARGET_EPS_CENTI) {
        servoVelocityCentiPerFrame[channel] = 0;
        return target;
    }

    direction = distance > 0 ? 1 : -1;
    velocity = servoVelocityCentiPerFrame[channel];
    if (velocity * direction < 0) {
        nextVelocity = velocity + direction * EDOG_SERVO_TRAPEZOID_MAX_ACCEL_CENTI_PER_FRAME2;
    } else {
        velocityAbs = velocity >= 0 ? velocity : -velocity;
        brakeDistance = (long long)velocityAbs * velocityAbs /
            (2LL * EDOG_SERVO_TRAPEZOID_MAX_ACCEL_CENTI_PER_FRAME2);
        if ((long long)distanceAbs <= brakeDistance) {
            nextVelocity = velocity - direction * EDOG_SERVO_TRAPEZOID_MAX_ACCEL_CENTI_PER_FRAME2;
        } else {
            nextVelocity = velocity + direction * EDOG_SERVO_TRAPEZOID_MAX_ACCEL_CENTI_PER_FRAME2;
        }
    }

    if (nextVelocity > EDOG_SERVO_TRAPEZOID_MAX_SPEED_CENTI_PER_FRAME) {
        nextVelocity = EDOG_SERVO_TRAPEZOID_MAX_SPEED_CENTI_PER_FRAME;
    } else if (nextVelocity < -EDOG_SERVO_TRAPEZOID_MAX_SPEED_CENTI_PER_FRAME) {
        nextVelocity = -EDOG_SERVO_TRAPEZOID_MAX_SPEED_CENTI_PER_FRAME;
    }
    if (nextVelocity == 0) {
        nextVelocity = direction;
    }

    if ((distance > 0 && nextVelocity > distance) ||
        (distance < 0 && nextVelocity < distance)) {
        next = target;
        nextVelocity = next - current;
    } else {
        next = current + nextVelocity;
    }

    servoVelocityCentiPerFrame[channel] = next - current;
    return clampServoAngleCenti(next);
}

static int motionSetServoPhysicalSmoothCenti(int channel, int centiAngle)
{
    int target = applyServoCenterTrimCenti(channel, clampServoAngleCenti(centiAngle));
    return writeServoProfileAngleCenti(channel, planServoTrapezoidStepCenti(channel, target));
}

static int motionSetServoSmoothCenti(int channel, int jointDeltaCentiDeg)
{
    return motionSetServoPhysicalSmoothCenti(channel,
        jointDeltaToServoAngleCenti(channel, jointDeltaCentiDeg));
}

/* 保留 setServo 包装语义，检查脚本也用它确认运动层输出会记录当前姿态。 */
#define setServo motionSetServo

static void EDOG_UNUSED safeSleep(uint64_t microseconds)
{
    static const double speedTable[7] = {1.5, 1.0, 0.8, 0.6, 0.4, 0.3, 0.3};
    int level = speedLevel;
    double factor;
    uint64_t adjusted;
    uint64_t elapsed = 0;

    if (level < 0) {
        level = 0;
    } else if (level > 6) {
        level = 6;
    }
    factor = speedTable[level];
    adjusted = (uint64_t)(microseconds * factor);

    while (elapsed < adjusted && !stopFlag) {
        usleep(1000);
        elapsed += 1000;
    }
}

static void sleepUntilStopOrTimeout(uint64_t microseconds)
{
    uint64_t elapsed = 0;

    while (elapsed < microseconds && !stopFlag) {
        usleep(1000);
        elapsed += 1000;
    }
}

static uint64_t getSpeedAdjustedFramePeriodUs(void)
{
    static const uint8_t framePeriodPercent[7] = {140, 100, 75, 45, 35, 30, 30};
    int level = speedLevel;
    uint8_t percent;

    if (level < 0) {
        level = 0;
    } else if (level > 6) {
        level = 4;
    }

    percent = framePeriodPercent[level];
    return ((uint64_t)EDOG_12DOF_GAIT_FRAME_PERIOD_US * percent + 50) / 100;
}

static void sleepRemainingFrameTime(uint64_t usedUs)
{
    const uint64_t framePeriodUs = getSpeedAdjustedFramePeriodUs();

    if (usedUs >= framePeriodUs) {
        return;
    }
    sleepUntilStopOrTimeout(framePeriodUs - usedUs);
}

static void addFrameUsedTime(uint64_t *usedUs, int elapsedUs)
{
    if (elapsedUs > 0) {
        *usedUs += (uint64_t)elapsedUs;
    }
}

static int setLegAnglesStaggered(const Edog12DofLeg *leg,
                                 const Edog12DofJointAngles *angles,
                                 uint64_t delayUs)
{
    if (setServo(leg->hip, angles->hipAngleDeg) != 0) {
        return -1;
    }
    usleep(delayUs);
    if (setServo(leg->thigh, angles->femurAngleDeg) != 0) {
        return -1;
    }
    usleep(delayUs);
    if (setServo(leg->calf, angles->tibiaAngleDeg) != 0) {
        return -1;
    }
    usleep(delayUs);
    return (int)(delayUs * EDOG_12DOF_JOINTS_PER_LEG);
}

static int setLegAnglesStaggeredSmooth(const Edog12DofLeg *leg,
                                       const Edog12DofJointAngles *angles,
                                       uint64_t delayUs)
{
    /* 走厘度平滑链：保留 IK 亚度精度，消除支撑相整数度阶梯抖动。 */
    if (motionSetServoSmoothCenti(leg->hip, angles->hipAngleCentiDeg) != 0) {
        return -1;
    }
    usleep(delayUs);
    if (motionSetServoSmoothCenti(leg->thigh, angles->femurAngleCentiDeg) != 0) {
        return -1;
    }
    usleep(delayUs);
    if (motionSetServoSmoothCenti(leg->calf, angles->tibiaAngleCentiDeg) != 0) {
        return -1;
    }
    usleep(delayUs);
    return (int)(delayUs * EDOG_12DOF_JOINTS_PER_LEG);
}

static void EDOG_UNUSED setLegCenter(const Edog12DofLeg *leg)
{
    (void)motionSetServoPhysical(leg->hip, EDOG_SERVO_CENTER_ANGLE);
    (void)motionSetServoPhysical(leg->thigh, EDOG_SERVO_CENTER_ANGLE);
    (void)motionSetServoPhysical(leg->calf, EDOG_SERVO_CENTER_ANGLE);
}

int setDogServoAngleTracked(int channel, int angle)
{
    return setDogServoPhysicalAngleTracked(channel, angle);
}

int setDogServoPhysicalAngleTracked(int channel, int angle)
{
    return writeServoPhysicalAngle(channel, clampServoAngle(angle));
}

int setDogServoCenterTrim(int channel, int trimDeg)
{
    if (channel < 0 || channel >= EDOG_SERVO_CHANNEL_COUNT) {
        printf("[ServoTrim] invalid channel=%d\n", channel);
        return -1;
    }
    if (trimDeg < -45 || trimDeg > 45) {
        printf("[ServoTrim] invalid trim=%d, valid range is -45~45\n", trimDeg);
        return -1;
    }

    g_servoCenterTrim[channel] = trimDeg;
    return 0;
}

int loadDogServoCenterTrims(void)
{
    int loaded = 0;

    for (int channel = 0; channel < EDOG_SERVO_CHANNEL_COUNT; channel++) {
        char key[24] = {0};
        char value[12] = {0};
        int ret;
        int trim;

        buildServoTrimKey(channel, key, sizeof(key));
        ret = UtilsGetValue(key, value, sizeof(value));
        if (ret <= 0) {
            continue;
        }
        trim = atoi(value);
        if (trim < -45 || trim > 45) {
            printf("[ServoTrim] ignore stored invalid trim channel=%d value=%d\n", channel, trim);
            continue;
        }
        g_servoCenterTrim[channel] = trim;
        loaded++;
    }
    return loaded;
}

int clearDogServoCenterTrimsOnceForRealAngleCalibration(void)
{
    char value[8] = {0};
    int ret = UtilsGetValue(EDOG_SERVO_TRIM_RESET_MIGRATION_KEY, value, sizeof(value));

    if (ret > 0 && strcmp(value, EDOG_SERVO_TRIM_RESET_MIGRATION_DONE) == 0) {
        return 0;
    }

    for (int channel = 0; channel < EDOG_SERVO_CHANNEL_COUNT; channel++) {
        char key[24] = {0};
        buildServoTrimKey(channel, key, sizeof(key));
        g_servoCenterTrim[channel] = 0;
        (void)UtilsSetValue(key, "0");
    }
    (void)UtilsSetValue(EDOG_SERVO_TRIM_RESET_MIGRATION_KEY,
                        EDOG_SERVO_TRIM_RESET_MIGRATION_DONE);
    return 1;
}

int saveDogServoCenterTrim(int channel, int trimDeg)
{
    char key[24] = {0};
    char value[12] = {0};
    int ret;

    if (setDogServoCenterTrim(channel, trimDeg) != 0) {
        return -1;
    }

    buildServoTrimKey(channel, key, sizeof(key));
    snprintf(value, sizeof(value), "%d", trimDeg);
    ret = UtilsSetValue(key, value);
    if (ret != 0) {
        printf("[ServoTrim] persist failed channel=%d trim=%d ret=%d\n", channel, trimDeg, ret);
        return -1;
    }
    return 0;
}

int getDogServoCenterTrim(int channel)
{
    if (channel < 0 || channel >= EDOG_SERVO_CHANNEL_COUNT) {
        return 0;
    }
    return g_servoCenterTrim[channel];
}

int getDogServoTrackedAngle(int channel)
{
    if (channel < 0 || channel >= EDOG_SERVO_CHANNEL_COUNT || !servoAngleValid[channel]) {
        return applyServoCenterTrim(channel, EDOG_SERVO_CENTER_ANGLE);
    }
    return servoAngles[channel];
}

int getDogServoTrackedAngles(int angles[], int count)
{
    if (angles == NULL || count < EDOG_SERVO_CHANNEL_COUNT) {
        return -1;
    }
    for (int channel = 0; channel < EDOG_SERVO_CHANNEL_COUNT; channel++) {
        angles[channel] = getDogServoTrackedAngle(channel);
    }
    return EDOG_SERVO_CHANNEL_COUNT;
}

void setRuntimeGaitGeometry(double hipAdductionDeg, double thighLengthM, double calfLengthM)
{
    setRuntimeGaitGeometryForFrontRear(hipAdductionDeg, hipAdductionDeg,
                                       thighLengthM, calfLengthM);
}

void setRuntimeGaitGeometryForFrontRear(double frontHipAdductionDeg,
                                        double rearHipAdductionDeg,
                                        double thighLengthM,
                                        double calfLengthM)
{
    double thighLengthMm = thighLengthM > 0.0 ? thighLengthM * 1000.0 : SPOTMICRO_THIGH_LENGTH_MM;
    double calfLengthMm = calfLengthM > 0.0 ? calfLengthM * 1000.0 : SPOTMICRO_CALF_LENGTH_MM;

    Edog12Dof_SetRuntimeGeometryForFrontRear(frontHipAdductionDeg, rearHipAdductionDeg,
                                             thighLengthMm, calfLengthMm);
}

static int normalizeImuBalanceStrengthPercent(int strengthPercent)
{
    if (strengthPercent < 0) {
        return 0;
    }
    if (strengthPercent > 200) {
        return 200;
    }
    return strengthPercent;
}

int setImuBalanceStrengthPercent(int strengthPercent)
{
    g_imuBalanceStrengthPercent = normalizeImuBalanceStrengthPercent(strengthPercent);
    return g_imuBalanceStrengthPercent;
}

int getImuBalanceStrengthPercent(void)
{
    return g_imuBalanceStrengthPercent;
}

static void loadRuntimeTuningOnce(void)
{
    static int loaded = 0;

    if (loaded) {
        return;
    }
    loaded = 1;
    (void)loadRuntimeTuningFromKv();
}

int saveRuntimeTuningToKv(void)
{
    double frontHipDeg = 0.0;
    double rearHipDeg = 0.0;
    double thighMm = 0.0;
    double calfMm = 0.0;
    double frontBodyHeightDeltaMm = 0.0;
    double rearBodyHeightDeltaMm = 0.0;
    char value[24] = {0};
    int ret = 0;

    Edog12Dof_GetRuntimeGeometryForFrontRear(&frontHipDeg, &rearHipDeg, &thighMm, &calfMm);
    Edog12Dof_GetRuntimeFootZDeltas(&frontBodyHeightDeltaMm, &rearBodyHeightDeltaMm);
    snprintf(value, sizeof(value), "%.2f", frontHipDeg);
    if (UtilsSetValue(EDOG_RUNTIME_FRONT_HIP_KEY, value) != 0) {
        ret = -1;
    }
    snprintf(value, sizeof(value), "%.2f", rearHipDeg);
    if (UtilsSetValue(EDOG_RUNTIME_REAR_HIP_KEY, value) != 0) {
        ret = -1;
    }
    snprintf(value, sizeof(value), "%.2f", frontBodyHeightDeltaMm);
    if (UtilsSetValue(EDOG_RUNTIME_FRONT_BODY_HEIGHT_KEY, value) != 0) {
        ret = -1;
    }
    snprintf(value, sizeof(value), "%.2f", rearBodyHeightDeltaMm);
    if (UtilsSetValue(EDOG_RUNTIME_REAR_BODY_HEIGHT_KEY, value) != 0) {
        ret = -1;
    }
    snprintf(value, sizeof(value), "%d", g_imuBalanceStrengthPercent);
    if (UtilsSetValue(EDOG_RUNTIME_IMU_STRENGTH_KEY, value) != 0) {
        ret = -1;
    }
    return ret;
}

int loadRuntimeTuningFromKv(void)
{
    double frontHipDeg = EDOG_12DOF_FRONT_HIP_ADDUCTION_DEFAULT_DEG;
    double rearHipDeg = EDOG_12DOF_REAR_HIP_ADDUCTION_DEFAULT_DEG;
    double thighMm = 0.0;
    double calfMm = 0.0;
    double frontBodyHeightDeltaMm = 0.0;
    double rearBodyHeightDeltaMm = 0.0;
    char value[24] = {0};
    int loaded = 0;

    Edog12Dof_GetRuntimeGeometryForFrontRear(NULL, NULL, &thighMm, &calfMm);
    Edog12Dof_GetRuntimeFootZDeltas(&frontBodyHeightDeltaMm, &rearBodyHeightDeltaMm);
    if (UtilsGetValue(EDOG_RUNTIME_FRONT_HIP_KEY, value, sizeof(value)) > 0) {
        frontHipDeg = atof(value);
        loaded++;
    }
    memset(value, 0, sizeof(value));
    if (UtilsGetValue(EDOG_RUNTIME_REAR_HIP_KEY, value, sizeof(value)) > 0) {
        rearHipDeg = atof(value);
        loaded++;
    }
    memset(value, 0, sizeof(value));
    if (UtilsGetValue(EDOG_RUNTIME_FRONT_BODY_HEIGHT_KEY, value, sizeof(value)) > 0) {
        frontBodyHeightDeltaMm = atof(value);
        loaded++;
    }
    memset(value, 0, sizeof(value));
    if (UtilsGetValue(EDOG_RUNTIME_REAR_BODY_HEIGHT_KEY, value, sizeof(value)) > 0) {
        rearBodyHeightDeltaMm = atof(value);
        loaded++;
    }
    memset(value, 0, sizeof(value));
    if (UtilsGetValue(EDOG_RUNTIME_IMU_STRENGTH_KEY, value, sizeof(value)) > 0) {
        (void)setImuBalanceStrengthPercent(atoi(value));
        loaded++;
    } else {
        (void)setImuBalanceStrengthPercent(100);
    }
    Edog12Dof_SetRuntimeGeometryForFrontRear(frontHipDeg, rearHipDeg, thighMm, calfMm);
    Edog12Dof_SetRuntimeFootZDeltas(frontBodyHeightDeltaMm, rearBodyHeightDeltaMm);
    return loaded;
}

int setDogLegsStraightPose(void)
{
    for (int k = 0; k < EDOG_12DOF_ACTIVE_SERVO_COUNT; k++) {
        int channel = g_activeJoints[k].channel;
        /* 90deg + trim: straight-leg calibration uses the real calibrated center angle. */
        int calibratedCenterCenti = EDOG_SERVO_CENTER_ANGLE * EDOG_12DOF_CENTI_PER_DEG +
            g_servoCenterTrim[channel] * EDOG_12DOF_CENTI_PER_DEG;
        if (motionSetServoPhysicalNoTrimCenti(channel, calibratedCenterCenti) != 0) {
            return -1;
        }
        usleep(EDOG_SERVO_STARTUP_STEP_DELAY_US);
    }
    return 0;
}

void setSpeedLevel(int level)
{
    if (level < 0) {
        level = 0;
    }
    if (level > 6) {
        level = 4;
    }
    speedLevel = level;
}

int getSpeedLevel(void)
{
    return speedLevel;
}

void stopCurrentMotion(void)
{
    g_servoTableRuntime.active = 0;
    g_servoTableRuntime.mode = EDOG_TABLE_MODE_NONE;
    g_continuousTrotRuntime.active = 0;
    g_continuousTrotRuntime.mode = EDOG_TABLE_MODE_NONE;
    g_continuousTrotRuntime.frameIndex = 0;
    enterImuBalanceMode(EDOG_IMU_BALANCE_MODE_STOP_SETTLING);
    stopFlag = 1;
}

void resetStopFlag(void)
{
    stopFlag = 0;
}

int isStopFlag(void)
{
    return stopFlag;
}

void init_dog(double step_length, double step_height)
{
    Edog12DofJointAngles standFrame[EDOG_12DOF_LEG_COUNT];

    (void)step_length;
    (void)step_height;
    loadRuntimeTuningOnce();
    fillStandFrame(standFrame);

    for (int i = 0; i < EDOG_12DOF_LEG_COUNT; i++) {
        int legIndex = g_startupLegOrder[i];
        (void)setLegAnglesStaggered(&g_legs[legIndex], &standFrame[legIndex], EDOG_SERVO_STARTUP_STEP_DELAY_US);
    }
}

static void buildStandPose(int target[EDOG_SERVO_CHANNEL_COUNT],
                           double stepLength, double stepHeight)
{
    Edog12DofJointAngles standFrame[EDOG_12DOF_LEG_COUNT];

    (void)stepLength;
    (void)stepHeight;
    for (int i = 0; i < EDOG_SERVO_CHANNEL_COUNT; i++) {
        target[i] = EDOG_SERVO_CENTER_ANGLE;
    }

    fillStandFrame(standFrame);
    target[LF_HIP] = jointDeltaToServoAngle(LF_HIP, standFrame[0].hipAngleDeg);
    target[LF_THIGH] = jointDeltaToServoAngle(LF_THIGH, standFrame[0].femurAngleDeg);
    target[LF_CALF] = jointDeltaToServoAngle(LF_CALF, standFrame[0].tibiaAngleDeg);
    target[RF_HIP] = jointDeltaToServoAngle(RF_HIP, standFrame[1].hipAngleDeg);
    target[RF_THIGH] = jointDeltaToServoAngle(RF_THIGH, standFrame[1].femurAngleDeg);
    target[RF_CALF] = jointDeltaToServoAngle(RF_CALF, standFrame[1].tibiaAngleDeg);
    target[LB_HIP] = jointDeltaToServoAngle(LB_HIP, standFrame[2].hipAngleDeg);
    target[LB_THIGH] = jointDeltaToServoAngle(LB_THIGH, standFrame[2].femurAngleDeg);
    target[LB_CALF] = jointDeltaToServoAngle(LB_CALF, standFrame[2].tibiaAngleDeg);
    target[RB_HIP] = jointDeltaToServoAngle(RB_HIP, standFrame[3].hipAngleDeg);
    target[RB_THIGH] = jointDeltaToServoAngle(RB_THIGH, standFrame[3].femurAngleDeg);
    target[RB_CALF] = jointDeltaToServoAngle(RB_CALF, standFrame[3].tibiaAngleDeg);
}

static void buildStandPoseCenti(int target[EDOG_SERVO_CHANNEL_COUNT],
                                double stepLength, double stepHeight)
{
    int standDeg[EDOG_SERVO_CHANNEL_COUNT] = {0};

    buildStandPose(standDeg, stepLength, stepHeight);
    for (int channel = 0; channel < EDOG_SERVO_CHANNEL_COUNT; channel++) {
        target[channel] = standDeg[channel] * EDOG_12DOF_CENTI_PER_DEG;
    }
}

int smooth_stop_to_stand(double step_length, double step_height)
{
    int target[EDOG_SERVO_CHANNEL_COUNT] = {0};
    int allDone = 1;

    buildStandPoseCenti(target, step_length, step_height);

    for (int k = 0; k < EDOG_12DOF_ACTIVE_SERVO_COUNT; k++) {
        int channel = g_activeJoints[k].channel;
        int safeSpeed = getSafeServoCentiDegPerMs(channel);
        int maxStep = EDOG_SERVO_STOP_STEP_DELAY_US / 1000 * safeSpeed;
        int finalTarget = servoTableCentiToTrimmedTarget(channel, target[channel]);
        int current = servoAngleValid[channel] ? servoAnglesCenti[channel] : finalTarget;
        int next;

        if (maxStep < safeSpeed) {
            maxStep = safeSpeed;
        }
        if (!stopFlag) {
            return 0;
        }
        next = approachServoTargetCenti(current, finalTarget, maxStep);
        if (next != finalTarget) {
            allDone = 0;
        }
        if (writeServoProfileAngleCenti(channel, next) != 0) {
            return 0;
        }
        usleep(EDOG_SERVO_STOP_STEP_DELAY_US);
    }
    return allDone;
}

int balance_stand_frame(void)
{
    Edog12DofJointAngles frame[EDOG_12DOF_LEG_COUNT];
    Edog12DofFootPoint feet[EDOG_12DOF_LEG_COUNT];
    int rollControlCentiDeg = 0;
    int pitchControlCentiDeg = 0;
    uint64_t usedUs = 0;

#if EDOG_12DOF_IMU_BALANCE_ENABLED
    /* IMU 自稳开启时才走足端补偿 + IK 路径。 */
    getImuBalanceControlCentiDeg(&rollControlCentiDeg, &pitchControlCentiDeg);
    for (int leg = 0; leg < EDOG_12DOF_LEG_COUNT; leg++) {
        Edog12DofFootPoint foot = getDefaultFootPoint(leg);
        applyImuBalanceFootCompensation(&foot, leg, 0,
                                        rollControlCentiDeg, pitchControlCentiDeg);
        feet[leg] = foot;
        if (Edog12Dof_IK(&foot, g_legs[leg].isRightLeg, &frame[leg]) != 0) {
            frame[leg] = getStandJointAnglesForLeg(leg);
        }
    }

    printImuBalanceDebugFrame(g_balanceLastReadStatus,
                              rollControlCentiDeg, pitchControlCentiDeg,
                              feet, frame);
#else
    /* IMU 自稳关闭时补偿恒为 0，直接用站姿表跳过 IK，省去每帧规划开销。 */
    fillStandFrame(frame);
    (void)rollControlCentiDeg;
    (void)pitchControlCentiDeg;
    (void)feet;
#endif

    for (int leg = 0; leg < EDOG_12DOF_LEG_COUNT; leg++) {
        addFrameUsedTime(&usedUs, setLegAnglesStaggeredSmooth(&g_legs[leg], &frame[leg],
                                                              EDOG_12DOF_REALTIME_SERVO_STEP_DELAY_US));
    }
    sleepRemainingFrameTime(usedUs);
    return 0;
}

static void EDOG_UNUSED buildDirectionalGaitSet(Edog12DofGaitSet gaitSet,
                                                double forwardStep, double sideStep,
                                                double yawStep, double stepHeight,
                                                int backward)
{
    Edog12Dof_GenerateDirectionalTrotTable(gaitSet[0], forwardStep, sideStep, yawStep,
                                           stepHeight, 0, 1, backward);
    Edog12Dof_GenerateDirectionalTrotTable(gaitSet[1], forwardStep, sideStep, yawStep,
                                           stepHeight, 1, 1, backward);
    Edog12Dof_GenerateDirectionalTrotTable(gaitSet[2], forwardStep, sideStep, yawStep,
                                           stepHeight, 0, 0, backward);
    Edog12Dof_GenerateDirectionalTrotTable(gaitSet[3], forwardStep, sideStep, yawStep,
                                           stepHeight, 1, 0, backward);
}

static void buildScaledGaitSet(Edog12DofGaitSet gaitSet,
                               double stepLength, double stepHeight,
                               int backward, int leftScalePercent,
                               int rightScalePercent)
{
    double leftStep = stepLength * (double)leftScalePercent / 100.0;
    double rightStep = stepLength * (double)rightScalePercent / 100.0;

    Edog12Dof_GenerateDirectionalTrotTable(gaitSet[0], leftStep, 0.0, 0.0,
                                           stepHeight, 0, 1, backward);
    Edog12Dof_GenerateDirectionalTrotTable(gaitSet[1], rightStep, 0.0, 0.0,
                                           stepHeight, 1, 1, backward);
    Edog12Dof_GenerateDirectionalTrotTable(gaitSet[2], leftStep, 0.0, 0.0,
                                           stepHeight, 0, 0, backward);
    Edog12Dof_GenerateDirectionalTrotTable(gaitSet[3], rightStep, 0.0, 0.0,
                                           stepHeight, 1, 0, backward);
}

static void mirrorDynamicAroundStand(Edog12DofJointAngles *angles)
{
    Edog12DofJointAngles standAngles = getStandJointAnglesForLeg(0);

    if (angles == NULL) {
        return;
    }

    angles->femurAngleDeg = clampJointDeltaDeg(
        standAngles.femurAngleDeg * 2 - angles->femurAngleDeg);
    angles->tibiaAngleDeg = clampJointDeltaDeg(
        standAngles.tibiaAngleDeg * 2 - angles->tibiaAngleDeg);
    angles->femurAngleCentiDeg = clampJointDeltaCenti(
        standAngles.femurAngleCentiDeg * 2 -
        angles->femurAngleCentiDeg);
    angles->tibiaAngleCentiDeg = clampJointDeltaCenti(
        standAngles.tibiaAngleCentiDeg * 2 -
        angles->tibiaAngleCentiDeg);
}

static void apply8DofMotionOutputSign(int legIndex, Edog12DofJointAngles *angles)
{
    (void)legIndex;
    (void)angles;
    /*
     * g_servoDirection 表已完整处理所有腿的物理方向镜像。
     * IK 对所有腿输出统一的运动学增量（前摆为正，后折为正），
     * 前后腿无符号差异，无需额外变换。
     *
     * 原 mirror(LF/RB) 基于“LB/RB 输出负值”的错误假设，会使 LF/RB
     * 步伐方向反转，是动态行走错乱的根本原因，已移除。
     */
}

static int setGaitLegAnglesStaggeredSmooth(int legIndex,
                                           const Edog12DofJointAngles *angles,
                                           uint64_t delayUs)
{
    Edog12DofJointAngles outputAngles;

    if (legIndex < 0 || legIndex >= EDOG_12DOF_LEG_COUNT || angles == NULL) {
        return -1;
    }

    outputAngles = *angles;
    apply8DofMotionOutputSign(legIndex, &outputAngles);
    return setLegAnglesStaggeredSmooth(&g_legs[legIndex], &outputAngles, delayUs);
}

static void apply8DofTableFrameStaggered(const Edog12DofGaitSet gaitSet, int index, int phase)
{
    int delayed = (index + phase) % EDOG_12DOF_TROT_FRAME_COUNT;
    uint64_t usedUs = 0;

    addFrameUsedTime(&usedUs, setGaitLegAnglesStaggeredSmooth(0, &gaitSet[0][index],
                                                              EDOG_SERVO_MOTION_STEP_DELAY_US));
    addFrameUsedTime(&usedUs, setGaitLegAnglesStaggeredSmooth(3, &gaitSet[3][index],
                                                              EDOG_SERVO_MOTION_STEP_DELAY_US));
    addFrameUsedTime(&usedUs, setGaitLegAnglesStaggeredSmooth(2, &gaitSet[2][delayed],
                                                              EDOG_SERVO_MOTION_STEP_DELAY_US));
    addFrameUsedTime(&usedUs, setGaitLegAnglesStaggeredSmooth(1, &gaitSet[1][delayed],
                                                              EDOG_SERVO_MOTION_STEP_DELAY_US));
    sleepRemainingFrameTime(usedUs);
}

static uint64_t getTimeMs(void)
{
    struct timeval tv;

    if (gettimeofday(&tv, NULL) != 0) {
        return 0;
    }
    return (uint64_t)tv.tv_sec * 1000ULL + (uint64_t)tv.tv_usec / 1000ULL;
}

static int absIntValue(int value)
{
    return value < 0 ? -value : value;
}

static void clearImuBalanceControlState(void)
{
    g_balancePrevRollErrorCentiDeg = 0;
    g_balancePrevPitchErrorCentiDeg = 0;
    g_balanceRollControlCentiDeg = 0;
    g_balancePitchControlCentiDeg = 0;
    g_balanceRollIntegralCentiDeg = 0;
    g_balancePitchIntegralCentiDeg = 0;
    g_balanceLastRollOutputCentiDeg = 0;
    g_balanceLastPitchOutputCentiDeg = 0;
    g_balanceControlReady = 0;
#if EDOG_12DOF_IMU_BALANCE_ENABLED
    g_balanceRollRateCentiDps = 0;
    g_balancePitchRateCentiDps = 0;
    g_balanceFusionSampleCount = 0;
#endif
}

static void enterImuBalanceMode(EdogImuBalanceMode mode)
{
    if (g_balanceMode == mode) {
        return;
    }

    g_balanceMode = mode;
    g_balanceModeStartMs = getTimeMs();
    g_balanceStableFrames = 0;
    if (mode == EDOG_IMU_BALANCE_MODE_STOP_SETTLING ||
        mode == EDOG_IMU_BALANCE_MODE_NORMAL) {
        clearImuBalanceControlState();
    }
}

static int imuBalanceMotionIsStable(int rollCentiDeg, int pitchCentiDeg,
                                    int rollRateCentiDps, int pitchRateCentiDps)
{
    return absIntValue(rollCentiDeg) <=
        EDOG_12DOF_IMU_BALANCE_STOP_STABLE_TILT_CENTI_DEG &&
        absIntValue(pitchCentiDeg) <=
        EDOG_12DOF_IMU_BALANCE_STOP_STABLE_TILT_CENTI_DEG &&
        absIntValue(rollRateCentiDps) <=
        EDOG_12DOF_IMU_BALANCE_STOP_STABLE_RATE_CENTI_DPS &&
        absIntValue(pitchRateCentiDps) <=
        EDOG_12DOF_IMU_BALANCE_STOP_STABLE_RATE_CENTI_DPS;
}

static int imuBalanceMotionExceedsExit(int rollCentiDeg, int pitchCentiDeg,
                                       int rollRateCentiDps, int pitchRateCentiDps)
{
    return absIntValue(rollCentiDeg) >
        EDOG_12DOF_IMU_BALANCE_STOP_EXIT_TILT_CENTI_DEG ||
        absIntValue(pitchCentiDeg) >
        EDOG_12DOF_IMU_BALANCE_STOP_EXIT_TILT_CENTI_DEG ||
        absIntValue(rollRateCentiDps) >
        EDOG_12DOF_IMU_BALANCE_STOP_EXIT_RATE_CENTI_DPS ||
        absIntValue(pitchRateCentiDps) >
        EDOG_12DOF_IMU_BALANCE_STOP_EXIT_RATE_CENTI_DPS;
}

static int limitImuControlStep(int previousCentiDeg, int targetCentiDeg,
                               int maxStepCentiDeg)
{
    return approachServoTargetCenti(previousCentiDeg, targetCentiDeg, maxStepCentiDeg);
}

static void applyImuBalanceRampScale(int *rollControlCentiDeg,
                                     int *pitchControlCentiDeg,
                                     uint64_t elapsedMs)
{
    uint64_t scalePercent;

    if (rollControlCentiDeg == NULL || pitchControlCentiDeg == NULL) {
        return;
    }
    if (EDOG_12DOF_IMU_BALANCE_RAMP_IN_MS <= 0) {
        return;
    }

    scalePercent = elapsedMs * 100ULL / EDOG_12DOF_IMU_BALANCE_RAMP_IN_MS;
    if (scalePercent > 100ULL) {
        scalePercent = 100ULL;
    }
    *rollControlCentiDeg =
        (int)((int64_t)(*rollControlCentiDeg) * (int64_t)scalePercent / 100);
    *pitchControlCentiDeg =
        (int)((int64_t)(*pitchControlCentiDeg) * (int64_t)scalePercent / 100);
    g_balanceRollControlCentiDeg = *rollControlCentiDeg;
    g_balancePitchControlCentiDeg = *pitchControlCentiDeg;
    g_balanceLastRollOutputCentiDeg = *rollControlCentiDeg;
    g_balanceLastPitchOutputCentiDeg = *pitchControlCentiDeg;
}

static int jointAnglesToServoFrameLeg(Edog12DofServoFrame *frame, int leg,
                                      const Edog12DofJointAngles *angles)
{
    Edog12DofJointAngles outputAngles;

    if (frame == NULL || angles == NULL || leg < 0 || leg >= EDOG_12DOF_LEG_COUNT) {
        return -1;
    }
    outputAngles = *angles;
    apply8DofMotionOutputSign(leg, &outputAngles);
    frame->centiDeg[g_legs[leg].hip] =
        jointDeltaToServoAngleCenti(g_legs[leg].hip, outputAngles.hipAngleCentiDeg);
    frame->centiDeg[g_legs[leg].thigh] =
        jointDeltaToServoAngleCenti(g_legs[leg].thigh, outputAngles.femurAngleCentiDeg);
    frame->centiDeg[g_legs[leg].calf] =
        jointDeltaToServoAngleCenti(g_legs[leg].calf, outputAngles.tibiaAngleCentiDeg);
    return 0;
}

static void initServoFrameCenter(Edog12DofServoFrame *frame)
{
    if (frame == NULL) {
        return;
    }
    for (int channel = 0; channel < EDOG_SERVO_CHANNEL_COUNT; channel++) {
        frame->centiDeg[channel] = EDOG_SERVO_CENTER_ANGLE * EDOG_12DOF_CENTI_PER_DEG;
    }
}

static void fillStandServoFrame(Edog12DofServoFrame *frame)
{
    Edog12DofJointAngles standFrame[EDOG_12DOF_LEG_COUNT];

    if (frame == NULL) {
        return;
    }
    initServoFrameCenter(frame);
    fillStandFrame(standFrame);
    for (int leg = 0; leg < EDOG_12DOF_LEG_COUNT; leg++) {
        (void)jointAnglesToServoFrameLeg(frame, leg, &standFrame[leg]);
    }
}

static int computeServoTableMinFrameMs(const Edog12DofServoGaitTable *table)
{
    int minFrameMs = 1;

    if (table == NULL || table->frameCount <= 0) {
        return 1;
    }
    for (int i = 0; i < table->frameCount; i++) {
        int next = (i + 1) % table->frameCount;
        for (int k = 0; k < EDOG_12DOF_ACTIVE_SERVO_COUNT; k++) {
            int channel = g_activeJoints[k].channel;
            int safeSpeed = getSafeServoCentiDegPerMs(channel);
            int delta = absIntValue(table->frames[next].centiDeg[channel] -
                                    table->frames[i].centiDeg[channel]);
            int channelFrameMs = (delta + safeSpeed - 1) / safeSpeed;
            if (channelFrameMs > minFrameMs) {
                minFrameMs = channelFrameMs;
            }
        }
    }
    return minFrameMs > 1 ? minFrameMs : 1;
}

static int getRequestedCycleMs(void)
{
    static const int requestedCycleMs[7] = {1600, 1200, 900, 700, 560, 480, 480};
    int level = speedLevel;

    if (level < 0) {
        level = 0;
    } else if (level > 6) {
        level = 4;
    }
    return requestedCycleMs[level];
}

static int isTurnTableMode(int mode)
{
    return mode == EDOG_TABLE_MODE_TURN_LEFT ||
        mode == EDOG_TABLE_MODE_TURN_RIGHT;
}

static void buildServoGaitTable(Edog12DofServoGaitTable *table,
                                int mode, double stepLength, double stepHeight)
{
    Edog12DofGaitSet *gaitSet = &g_tableBuildGaitSet;
    double forwardStep = stepLength;
    double sideStep = 0.0;
    double yawStep = 0.0;
    int backward = 0;

    if (table == NULL) {
        return;
    }
    table->frameCount = EDOG_12DOF_GAIT_FRAME_COUNT;
    table->minFrameMs = 1;
    table->minCycleMs = EDOG_12DOF_GAIT_FRAME_COUNT;

    switch (mode) {
        case EDOG_TABLE_MODE_BACKWARD:
            backward = 1;
            break;
        case EDOG_TABLE_MODE_TURN_LEFT:
            forwardStep = 0.0;
            yawStep = -stepLength;
            break;
        case EDOG_TABLE_MODE_TURN_RIGHT:
            forwardStep = 0.0;
            yawStep = stepLength;
            break;
        case EDOG_TABLE_MODE_IN_PLACE:
            forwardStep = 0.0;
            break;
        default:
            break;
    }

    buildDirectionalGaitSet(*gaitSet, forwardStep, sideStep, yawStep, stepHeight, backward);
    for (int frameIndex = 0; frameIndex < table->frameCount; frameIndex++) {
        initServoFrameCenter(&table->frames[frameIndex]);
        for (int leg = 0; leg < EDOG_12DOF_LEG_COUNT; leg++) {
            int sourceIndex = frameIndex;
            int isFrontLeg = (leg == 0 || leg == 1);
            int isSwing = 0;

            table->baseFeet[frameIndex][leg] =
                sampleDirectionalFootPoint(forwardStep, sideStep, yawStep, stepHeight,
                                           g_legs[leg].isRightLeg, isFrontLeg, leg,
                                           backward, sourceIndex, &isSwing);
            table->isSwing[frameIndex][leg] = isSwing ? 1 : 0;
            (void)jointAnglesToServoFrameLeg(&table->frames[frameIndex], leg,
                                             &(*gaitSet)[leg][sourceIndex]);
        }
    }
    table->minFrameMs = computeServoTableMinFrameMs(table);
    table->minCycleMs = table->minFrameMs * table->frameCount;
}

static int tableParamsChanged(const Edog12DofTableRuntime *runtime,
                              int mode, double stepLength,
                              double stepHeight, int legMask)
{
    int stepLengthCentiMm = doubleToCentiMm(stepLength * 1000.0);
    int stepHeightCentiMm = doubleToCentiMm(stepHeight * 1000.0);

    if (runtime == NULL || !runtime->active) {
        return 1;
    }
    return runtime->mode != mode ||
        runtime->stepLengthCentiMm != stepLengthCentiMm ||
        runtime->stepHeightCentiMm != stepHeightCentiMm ||
        runtime->legMask != (legMask & 0x0F);
}

static void configureTrotPhaseOffsets(Edog12DofTableRuntime *runtime)
{
    if (runtime == NULL) {
        return;
    }
    runtime->phaseOffset[0] = 0;
    runtime->phaseOffset[3] = 0;
    runtime->phaseOffset[1] = EDOG_12DOF_GAIT_FRAME_COUNT / 2;
    runtime->phaseOffset[2] = EDOG_12DOF_GAIT_FRAME_COUNT / 2;
}

static void configureModePhaseOffsets(Edog12DofTableRuntime *runtime, int mode)
{
    (void)mode;
    configureTrotPhaseOffsets(runtime);
}

static void startServoTableRuntime(int mode, double stepLength,
                                   double stepHeight, int legMask)
{
    int requestedCycleMs;
    uint64_t now = getTimeMs();
    int previousActive = g_servoTableRuntime.active;
    int previousBaseIndex = g_servoTableRuntime.baseIndex;
    int previousFrameCount = g_servoGaitTable.frameCount;
    int preservePhase = previousActive && previousFrameCount > 0;

    buildServoGaitTable(&g_servoGaitTable, mode, stepLength, stepHeight);
    requestedCycleMs = getRequestedCycleMs();
    if (isTurnTableMode(mode)) {
        requestedCycleMs =
            (requestedCycleMs * EDOG_12DOF_TURN_CYCLE_SLOWDOWN_NUM +
             EDOG_12DOF_TURN_CYCLE_SLOWDOWN_DEN - 1) /
            EDOG_12DOF_TURN_CYCLE_SLOWDOWN_DEN;
    }
    g_servoTableRuntime.mode = mode;
    g_servoTableRuntime.baseIndex = preservePhase ?
        previousBaseIndex % g_servoGaitTable.frameCount : 0;
    g_servoTableRuntime.cycleMs = requestedCycleMs > g_servoGaitTable.minCycleMs ?
        requestedCycleMs : g_servoGaitTable.minCycleMs;
    g_servoTableRuntime.frameMs = g_servoTableRuntime.cycleMs / g_servoGaitTable.frameCount;
    if (g_servoTableRuntime.frameMs < g_servoGaitTable.minFrameMs) {
        g_servoTableRuntime.frameMs = g_servoGaitTable.minFrameMs;
    }
    g_servoTableRuntime.cycleMs = g_servoTableRuntime.frameMs * g_servoGaitTable.frameCount;
    g_servoTableRuntime.lastUpdateMs = now;
    g_servoTableRuntime.active = 1;
    g_servoTableRuntime.legMask = legMask & 0x0F;
    g_servoTableRuntime.stepLengthCentiMm = doubleToCentiMm(stepLength * 1000.0);
    g_servoTableRuntime.stepHeightCentiMm = doubleToCentiMm(stepHeight * 1000.0);
    configureModePhaseOffsets(&g_servoTableRuntime, mode);
    enterImuBalanceMode(EDOG_IMU_BALANCE_MODE_NORMAL);
}

static int applyServoGaitTableFrame(const Edog12DofServoGaitTable *table,
                                    const Edog12DofTableRuntime *runtime,
                                    uint64_t elapsedMs)
{
    int rollControlCentiDeg = 0;
    int pitchControlCentiDeg = 0;

    if (table == NULL || runtime == NULL || table->frameCount <= 0) {
        return -1;
    }
    getImuBalanceControlCentiDeg(&rollControlCentiDeg, &pitchControlCentiDeg);
    for (int leg = 0; leg < EDOG_12DOF_LEG_COUNT; leg++) {
        int frameIndex = runtime->baseIndex + runtime->phaseOffset[leg];
        Edog12DofServoFrame imuFrame;
        int useCompensatedFrame = 0;

        while (frameIndex >= table->frameCount) {
            frameIndex -= table->frameCount;
        }
        if ((runtime->legMask & (1 << leg)) == 0) {
            continue;
        }
        initServoFrameCenter(&imuFrame);
#if EDOG_12DOF_IMU_BALANCE_ENABLED
        {
            Edog12DofFootPoint foot = table->baseFeet[frameIndex][leg];
            Edog12DofJointAngles angles;

            applyImuBalanceFootCompensation(&foot, leg,
                                            table->isSwing[frameIndex][leg],
                                            rollControlCentiDeg, pitchControlCentiDeg);
            if (Edog12Dof_IK(&foot, g_legs[leg].isRightLeg, &angles) == 0) {
                applyRearLegSwingBoost(leg, table->isSwing[frameIndex][leg], &angles);
                if (jointAnglesToServoFrameLeg(&imuFrame, leg, &angles) == 0) {
                    useCompensatedFrame = 1;
                }
            }
        }
#else
        (void)imuFrame;
        (void)rollControlCentiDeg;
        (void)pitchControlCentiDeg;
#endif
        for (int joint = 0; joint < EDOG_12DOF_JOINTS_PER_LEG; joint++) {
            int channel = joint == 0 ? g_legs[leg].hip :
                (joint == 1 ? g_legs[leg].thigh : g_legs[leg].calf);
            int tableCenti = table->frames[frameIndex].centiDeg[channel];
            int safeSpeed = getSafeServoCentiDegPerMs(channel);
            int safeStep = (int)elapsedMs * safeSpeed;
            if (useCompensatedFrame) {
                tableCenti = imuFrame.centiDeg[channel];
            }
            if (safeStep < 1) {
                safeStep = safeSpeed;
            }
            int target = servoTableCentiToTrimmedTarget(
                channel, tableCenti);
            int current = servoAngleValid[channel] ?
                servoAnglesCenti[channel] : target;
            int next = approachServoTargetCenti(current, target, safeStep);

            if (writeServoProfileAngleCenti(channel, next) != 0) {
                return -1;
            }
        }
    }
    return 0;
}

static int runServoGaitTableCycle(int mode, double stepLength,
                                  double stepHeight, int legMask)
{
    uint64_t now;
    uint64_t elapsedMs;
    int steps;
    int previousIndex;
    int completedCycle = 0;
    int started = 0;

    if (stopFlag) {
        g_servoTableRuntime.active = 0;
        return 0;
    }
    if ((legMask & 0x0F) == 0) {
        printf("[ServoTable] empty leg mask=%d\n", legMask);
        return 0;
    }
    if (tableParamsChanged(&g_servoTableRuntime, mode, stepLength, stepHeight, legMask)) {
        startServoTableRuntime(mode, stepLength, stepHeight, legMask);
        started = 1;
    }

    now = getTimeMs();
    elapsedMs = now >= g_servoTableRuntime.lastUpdateMs ?
        now - g_servoTableRuntime.lastUpdateMs : 0;
    if (started) {
        elapsedMs = (uint64_t)g_servoTableRuntime.frameMs;
    } else if (elapsedMs < (uint64_t)g_servoTableRuntime.frameMs) {
        return -1;
    }
    steps = (int)(elapsedMs / (uint64_t)g_servoTableRuntime.frameMs);
    if (steps < 1) {
        steps = 1;
    }
    if (steps > g_servoGaitTable.frameCount) {
        steps = g_servoGaitTable.frameCount;
    }
    previousIndex = g_servoTableRuntime.baseIndex;
    g_servoTableRuntime.baseIndex =
        (g_servoTableRuntime.baseIndex + steps) % g_servoGaitTable.frameCount;
    if (!started && previousIndex + steps >= g_servoGaitTable.frameCount) {
        completedCycle = 1;
    }
    g_servoTableRuntime.lastUpdateMs += (uint64_t)steps *
        (uint64_t)g_servoTableRuntime.frameMs;
    if (applyServoGaitTableFrame(&g_servoGaitTable, &g_servoTableRuntime,
                                 elapsedMs) != 0) {
        return 0;
    }
    return completedCycle ? 1 : -1;
}

static void apply8DofTurnFrameStaggered(const Edog12DofGaitSet forwardGait,
                                        const Edog12DofGaitSet reverseGait,
                                        int index, int turnLeft)
{
    const int phaseShift = EDOG_12DOF_TROT_FRAME_COUNT / 4;
    const Edog12DofJointAngles (*leftGait)[EDOG_12DOF_TROT_FRAME_COUNT] =
        turnLeft ? reverseGait : forwardGait;
    const Edog12DofJointAngles (*rightGait)[EDOG_12DOF_TROT_FRAME_COUNT] =
        turnLeft ? forwardGait : reverseGait;
    int j1 = (index + phaseShift) % EDOG_12DOF_TROT_FRAME_COUNT;
    int j2 = (index + 2 * phaseShift) % EDOG_12DOF_TROT_FRAME_COUNT;
    int j3 = (index + 3 * phaseShift) % EDOG_12DOF_TROT_FRAME_COUNT;
    uint64_t usedUs = 0;

    addFrameUsedTime(&usedUs, setGaitLegAnglesStaggeredSmooth(0, &leftGait[0][index],
                                                              EDOG_SERVO_MOTION_STEP_DELAY_US));
    addFrameUsedTime(&usedUs, setGaitLegAnglesStaggeredSmooth(2, &leftGait[2][j1],
                                                              EDOG_SERVO_MOTION_STEP_DELAY_US));
    addFrameUsedTime(&usedUs, setGaitLegAnglesStaggeredSmooth(3, &rightGait[3][j2],
                                                              EDOG_SERVO_MOTION_STEP_DELAY_US));
    addFrameUsedTime(&usedUs, setGaitLegAnglesStaggeredSmooth(1, &rightGait[1][j3],
                                                              EDOG_SERVO_MOTION_STEP_DELAY_US));
    sleepRemainingFrameTime(usedUs);
}

static double clampDoubleValue(double value, double minValue, double maxValue)
{
    if (value < minValue) {
        return minValue;
    }
    if (value > maxValue) {
        return maxValue;
    }
    return value;
}

static double minimumLiftMm(double liftMm, double minMm)
{
    return liftMm < minMm ? minMm : liftMm;
}

static Edog12DofFootPoint sampleDirectionalFootPoint(double forwardStepM,
                                                     double sideStepM,
                                                     double yawStepM,
                                                     double stepHeightM,
                                                     int isRightLeg,
                                                     int isFrontLeg,
                                                     int legIndex,
                                                     int reversed,
                                                     int frameIndex,
                                                     int *isSwingOut)
{
    return Edog12Dof_SampleReferenceTrotFootPoint(
        forwardStepM, sideStepM, yawStepM, stepHeightM,
        isRightLeg, isFrontLeg, legIndex, reversed, frameIndex, isSwingOut);
}

static int clampIntValue(int value, int minValue, int maxValue)
{
    if (value < minValue) {
        return minValue;
    }
    if (value > maxValue) {
        return maxValue;
    }
    return value;
}

static int doubleToCentiMm(double value)
{
    return (int)(value * 100.0 + (value >= 0.0 ? 0.5 : -0.5));
}

static const char *getImuBalanceStatusName(int status)
{
    switch (status) {
        case EDOG_IMU_BALANCE_STATUS_OK:
            return "OK";
        case EDOG_IMU_BALANCE_STATUS_DISABLED:
            return "DISABLED";
        case EDOG_IMU_BALANCE_STATUS_ARG_ERROR:
            return "ARG_ERROR";
        case EDOG_IMU_BALANCE_STATUS_INIT_WAIT:
            return "INIT_WAIT";
        case EDOG_IMU_BALANCE_STATUS_INIT_FAIL:
            return "INIT_FAIL";
        case EDOG_IMU_BALANCE_STATUS_READ_FAIL:
            return "READ_FAIL";
        case EDOG_IMU_BALANCE_STATUS_TILT_REJECT:
            return "TILT_REJECT";
        default:
            return "UNKNOWN";
    }
}

static void printImuBalanceDebugFrame(int status,
                                      int rollControlCentiDeg,
                                      int pitchControlCentiDeg,
                                      const Edog12DofFootPoint feet[EDOG_12DOF_LEG_COUNT],
                                      const Edog12DofJointAngles frame[EDOG_12DOF_LEG_COUNT])
{
#if EDOG_12DOF_IMU_BALANCE_DEBUG_ENABLED
    if (status == g_balanceDebugLastStatus) {
        return;
    }
    g_balanceDebugLastStatus = status;
    printf("[IMU-BALANCE] %s\n", getImuBalanceStatusName(status));
#else
    (void)status;
    (void)rollControlCentiDeg;
    (void)pitchControlCentiDeg;
    (void)feet;
    (void)frame;
#endif
}

static int applyBalanceDeadbandCentiDeg(int errorCentiDeg)
{
    int absError = errorCentiDeg >= 0 ? errorCentiDeg : -errorCentiDeg;

    if (absError <= EDOG_12DOF_IMU_BALANCE_DEADBAND_CENTI_DEG) {
        return 0;
    }
    return errorCentiDeg > 0 ?
        errorCentiDeg - EDOG_12DOF_IMU_BALANCE_DEADBAND_CENTI_DEG :
        errorCentiDeg + EDOG_12DOF_IMU_BALANCE_DEADBAND_CENTI_DEG;
}

static int computeImuTiltCentiDeg(int xMg, int yMg, int zMg,
                                  int *rollCentiDeg, int *pitchCentiDeg)
{
    double ax;
    double ay;
    double az;
    double accelNormMg;
    double rollDeg;
    double pitchDeg;

    if (rollCentiDeg == NULL || pitchCentiDeg == NULL) {
        return -1;
    }

    ax = (double)xMg;
    ay = (double)yMg;
    /*
     * The user's mounted MPU6050 reports about +1000mg on raw Z while level.
     * Keep the raw vector in body coordinates: X forward, Y left, Z up.
     */
    az = (double)zMg;
    accelNormMg = sqrt(ax * ax + ay * ay + az * az);
    if (accelNormMg < (double)EDOG_12DOF_IMU_BALANCE_MIN_ACCEL_MG ||
        accelNormMg > (double)EDOG_12DOF_IMU_BALANCE_MAX_ACCEL_MG) {
        return -1;
    }

    /*
     * MPU 安装坐标：X 前、Y 左、Z 上。加速度计平放 rawZ 约为 +1000mg。
     * 机身左侧下压时 rawY 变负，定义 roll 为正；机身前侧下压时
     * rawX 变负，定义 pitch 为负。
     */
    rollDeg = atan2(-ay, az) * 180.0 / M_PI;
    pitchDeg = atan2(ax, sqrt(ay * ay + az * az)) * 180.0 / M_PI;
    *rollCentiDeg = (int)(rollDeg * EDOG_12DOF_IMU_CENTI_PER_DEG +
                          (rollDeg >= 0.0 ? 0.5 : -0.5));
    *pitchCentiDeg = (int)(pitchDeg * EDOG_12DOF_IMU_CENTI_PER_DEG +
                           (pitchDeg >= 0.0 ? 0.5 : -0.5));
    return 0;
}

static int imuFusionSamplesPerControlFrame(void)
{
    int samples;

    if (EDOG_IMU_FUSION_SAMPLE_INTERVAL_MS <= 0) {
        return 1;
    }

    samples = 1000 / (EDOG_12DOF_GAIT_FRAME_FPS *
                      EDOG_IMU_FUSION_SAMPLE_INTERVAL_MS);
    return samples > 0 ? samples : 1;
}

static int roundDoubleToCentiDeg(double valueDeg)
{
    return (int)(valueDeg * (double)EDOG_12DOF_IMU_CENTI_PER_DEG +
                 (valueDeg >= 0.0 ? 0.5 : -0.5));
}

static int updateImuFusionSample(int *lastStatus)
{
    int xMg;
    int yMg;
    int zMg;
    int gyroXCentiDps;
    int gyroYCentiDps;
    int gyroZCentiDps;
    int measuredRoll;
    int measuredPitch;
    int controlRateCentiDps;
    int accelValid;
    const double dtSec = (double)EDOG_IMU_FUSION_SAMPLE_INTERVAL_MS / 1000.0;
    const double alpha = (double)EDOG_IMU_FUSION_COMPLEMENTARY_ALPHA_PERCENT / 100.0;
    double predictedRollDeg;
    double predictedPitchDeg;
    double accelRollDeg;
    double accelPitchDeg;
    double filteredRollDeg;
    double filteredPitchDeg;

    if (lastStatus == NULL) {
        return -1;
    }

    if (MpuMotionLight_ReadMotion(&xMg, &yMg, &zMg, &gyroXCentiDps,
                                  &gyroYCentiDps, &gyroZCentiDps) != 0) {
        *lastStatus = EDOG_IMU_BALANCE_STATUS_READ_FAIL;
        return -1;
    }

    /*
     * MPU 安装坐标下横滚主要绕 X 轴，俯仰主要绕 Y 轴。
     * 第一版只做阻尼，直接使用角速度定点值；若实机自激，优先翻转对应轴符号。
     */
    controlRateCentiDps = -gyroXCentiDps;
    g_balanceRollRateCentiDps = controlRateCentiDps;
    controlRateCentiDps = -gyroYCentiDps;
    g_balancePitchRateCentiDps = controlRateCentiDps;

    accelValid = (computeImuTiltCentiDeg(xMg, yMg, zMg,
                                         &measuredRoll, &measuredPitch) == 0);
    if (!accelValid) {
        *lastStatus = EDOG_IMU_BALANCE_STATUS_TILT_REJECT;
    }

    if (!g_balanceFilterReady) {
        if (accelValid) {
            g_balanceRollDeg = (double)measuredRoll /
                (double)EDOG_12DOF_IMU_CENTI_PER_DEG;
            g_balancePitchDeg = (double)measuredPitch /
                (double)EDOG_12DOF_IMU_CENTI_PER_DEG;
        }
        g_balanceFilterReady = 1;
    }

    predictedRollDeg = g_balanceRollDeg +
        ((double)g_balanceRollRateCentiDps /
         (double)EDOG_12DOF_IMU_CENTI_PER_DEG) * dtSec;
    predictedPitchDeg = g_balancePitchDeg +
        ((double)g_balancePitchRateCentiDps /
         (double)EDOG_12DOF_IMU_CENTI_PER_DEG) * dtSec;

    if (accelValid) {
        accelRollDeg = (double)measuredRoll /
            (double)EDOG_12DOF_IMU_CENTI_PER_DEG;
        accelPitchDeg = (double)measuredPitch /
            (double)EDOG_12DOF_IMU_CENTI_PER_DEG;
        filteredRollDeg = alpha * predictedRollDeg + (1.0 - alpha) * accelRollDeg;
        filteredPitchDeg = alpha * predictedPitchDeg + (1.0 - alpha) * accelPitchDeg;
        g_balanceRollDeg = filteredRollDeg;
        g_balancePitchDeg = filteredPitchDeg;
    } else {
        g_balanceRollDeg = predictedRollDeg;
        g_balancePitchDeg = predictedPitchDeg;
    }

    g_balanceRollCentiDeg = roundDoubleToCentiDeg(g_balanceRollDeg);
    g_balancePitchCentiDeg = roundDoubleToCentiDeg(g_balancePitchDeg);
    g_balanceFusionSampleCount++;

    if (accelValid) {
        *lastStatus = EDOG_IMU_BALANCE_STATUS_OK;
    }
    return 0;
}

static int readImuBalanceMotionCenti(int *rollCentiDeg, int *pitchCentiDeg,
                                     int *rollRateCentiDps, int *pitchRateCentiDps)
{
#if EDOG_12DOF_IMU_BALANCE_ENABLED
    int sampleCount;
    int lastStatus = EDOG_IMU_BALANCE_STATUS_OK;
    int successfulSamples = 0;

    if (rollCentiDeg == NULL || pitchCentiDeg == NULL ||
        rollRateCentiDps == NULL || pitchRateCentiDps == NULL) {
        g_balanceLastReadStatus = EDOG_IMU_BALANCE_STATUS_ARG_ERROR;
        return -1;
    }

    if (!g_balanceMpuReady) {
        if (g_balanceMpuInitRetryFrames > 0) {
            g_balanceMpuInitRetryFrames--;
            g_balanceLastReadStatus = EDOG_IMU_BALANCE_STATUS_INIT_WAIT;
            return -1;
        }
        if (MpuMotionLight_Init() != 0) {
            g_balanceMpuInitRetryFrames = EDOG_12DOF_GAIT_FRAME_FPS;
            g_balanceLastReadStatus = EDOG_IMU_BALANCE_STATUS_INIT_FAIL;
            return -1;
        }
        g_balanceMpuReady = 1;
    }

    sampleCount = imuFusionSamplesPerControlFrame();
    for (int i = 0; i < sampleCount; i++) {
        if (updateImuFusionSample(&lastStatus) == 0) {
            successfulSamples++;
        }
    }

    if (successfulSamples == 0) {
        g_balanceLastReadStatus = lastStatus;
        return -1;
    }

    *rollCentiDeg = g_balanceRollCentiDeg;
    *pitchCentiDeg = g_balancePitchCentiDeg;
    *rollRateCentiDps = g_balanceRollRateCentiDps;
    *pitchRateCentiDps = g_balancePitchRateCentiDps;
    g_balanceLastReadStatus = lastStatus;
    return 0;
#else
    (void)rollCentiDeg;
    (void)pitchCentiDeg;
    (void)rollRateCentiDps;
    (void)pitchRateCentiDps;
    g_balanceLastReadStatus = EDOG_IMU_BALANCE_STATUS_DISABLED;
    return -1;
#endif
}

static int readImuBalanceTiltCentiDeg(int *rollCentiDeg, int *pitchCentiDeg)
{
    int rollRateCentiDps = 0;
    int pitchRateCentiDps = 0;

    return readImuBalanceMotionCenti(rollCentiDeg, pitchCentiDeg,
                                     &rollRateCentiDps, &pitchRateCentiDps);
}

static void prepareImuBalanceControlCentiDeg(int measuredRollCentiDeg,
                                             int measuredPitchCentiDeg,
                                             int rollRateCentiDps,
                                             int pitchRateCentiDps,
                                             int *rollControlCentiDeg,
                                             int *pitchControlCentiDeg)
{
    int rollError;
    int pitchError;
    int rollDerivative;
    int pitchDerivative;
    double rollRateDegPerSec;
    double pitchRateDegPerSec;
    double rollControlDeg;
    double pitchControlDeg;
    int limitedRoll;
    int limitedPitch;

    if (rollControlCentiDeg == NULL || pitchControlCentiDeg == NULL) {
        return;
    }

    rollError = applyBalanceDeadbandCentiDeg(
        measuredRollCentiDeg - EDOG_12DOF_IMU_BALANCE_ROLL_TARGET_CENTI_DEG);
    pitchError = applyBalanceDeadbandCentiDeg(
        measuredPitchCentiDeg - EDOG_12DOF_IMU_BALANCE_PITCH_TARGET_CENTI_DEG);
    rollError = clampIntValue(rollError,
                              -EDOG_12DOF_IMU_BALANCE_MAX_TILT_CENTI_DEG,
                              EDOG_12DOF_IMU_BALANCE_MAX_TILT_CENTI_DEG);
    pitchError = clampIntValue(pitchError,
                               -EDOG_12DOF_IMU_BALANCE_MAX_TILT_CENTI_DEG,
                               EDOG_12DOF_IMU_BALANCE_MAX_TILT_CENTI_DEG);

#if EDOG_12DOF_IMU_BALANCE_INTEGRAL_KI_PERCENT != 0
    g_balanceRollIntegralCentiDeg =
        clampIntValue(g_balanceRollIntegralCentiDeg + rollError,
                      -EDOG_12DOF_IMU_BALANCE_MAX_TILT_CENTI_DEG,
                      EDOG_12DOF_IMU_BALANCE_MAX_TILT_CENTI_DEG);
    g_balancePitchIntegralCentiDeg =
        clampIntValue(g_balancePitchIntegralCentiDeg + pitchError,
                      -EDOG_12DOF_IMU_BALANCE_MAX_TILT_CENTI_DEG,
                      EDOG_12DOF_IMU_BALANCE_MAX_TILT_CENTI_DEG);
#else
    g_balanceRollIntegralCentiDeg = 0;
    g_balancePitchIntegralCentiDeg = 0;
#endif

    if (!g_balanceControlReady) {
        g_balancePrevRollErrorCentiDeg = rollError;
        g_balancePrevPitchErrorCentiDeg = pitchError;
        g_balanceLastRollOutputCentiDeg = 0;
        g_balanceLastPitchOutputCentiDeg = 0;
        g_balanceControlReady = 1;
    }
    rollDerivative = rollError - g_balancePrevRollErrorCentiDeg;
    pitchDerivative = pitchError - g_balancePrevPitchErrorCentiDeg;
    g_balancePrevRollErrorCentiDeg = rollError;
    g_balancePrevPitchErrorCentiDeg = pitchError;

    rollRateDegPerSec = (double)rollRateCentiDps / (double)EDOG_12DOF_IMU_CENTI_PER_DEG;
    pitchRateDegPerSec = (double)pitchRateCentiDps / (double)EDOG_12DOF_IMU_CENTI_PER_DEG;
    rollControlDeg =
        (double)(rollError + rollDerivative * EDOG_12DOF_IMU_BALANCE_D_TERM_PERCENT / 100) /
        (double)EDOG_12DOF_IMU_CENTI_PER_DEG +
#if EDOG_12DOF_IMU_BALANCE_INTEGRAL_KI_PERCENT != 0
        (double)g_balanceRollIntegralCentiDeg *
        EDOG_12DOF_IMU_BALANCE_INTEGRAL_KI_PERCENT /
        (100.0 * (double)EDOG_12DOF_IMU_CENTI_PER_DEG) +
#endif
        rollRateDegPerSec * EDOG_12DOF_IMU_BALANCE_ROLL_KD_MM_PER_DEG_PER_SEC /
        EDOG_12DOF_IMU_BALANCE_ROLL_KP_MM_PER_DEG;
    pitchControlDeg =
        (double)(pitchError + pitchDerivative * EDOG_12DOF_IMU_BALANCE_D_TERM_PERCENT / 100) /
        (double)EDOG_12DOF_IMU_CENTI_PER_DEG +
#if EDOG_12DOF_IMU_BALANCE_INTEGRAL_KI_PERCENT != 0
        (double)g_balancePitchIntegralCentiDeg *
        EDOG_12DOF_IMU_BALANCE_INTEGRAL_KI_PERCENT /
        (100.0 * (double)EDOG_12DOF_IMU_CENTI_PER_DEG) +
#endif
        pitchRateDegPerSec * EDOG_12DOF_IMU_BALANCE_PITCH_KD_MM_PER_DEG_PER_SEC /
        EDOG_12DOF_IMU_BALANCE_PITCH_KP_MM_PER_DEG;

    limitedRoll = clampIntValue(
        (int)(rollControlDeg * EDOG_12DOF_IMU_CENTI_PER_DEG +
              (rollControlDeg >= 0.0 ? 0.5 : -0.5)),
        -EDOG_12DOF_IMU_BALANCE_MAX_TILT_CENTI_DEG,
        EDOG_12DOF_IMU_BALANCE_MAX_TILT_CENTI_DEG);
    limitedPitch = clampIntValue(
        (int)(pitchControlDeg * EDOG_12DOF_IMU_CENTI_PER_DEG +
              (pitchControlDeg >= 0.0 ? 0.5 : -0.5)),
        -EDOG_12DOF_IMU_BALANCE_MAX_TILT_CENTI_DEG,
        EDOG_12DOF_IMU_BALANCE_MAX_TILT_CENTI_DEG);
    *rollControlCentiDeg =
        limitImuControlStep(g_balanceLastRollOutputCentiDeg, limitedRoll,
                            EDOG_12DOF_IMU_BALANCE_ROLL_STEP_LIMIT_CENTI_DEG);
    *pitchControlCentiDeg =
        limitImuControlStep(g_balanceLastPitchOutputCentiDeg, limitedPitch,
                            EDOG_12DOF_IMU_BALANCE_PITCH_STEP_LIMIT_CENTI_DEG);
    g_balanceRollControlCentiDeg = *rollControlCentiDeg;
    g_balancePitchControlCentiDeg = *pitchControlCentiDeg;
    g_balanceLastRollOutputCentiDeg = *rollControlCentiDeg;
    g_balanceLastPitchOutputCentiDeg = *pitchControlCentiDeg;
}

static void decayImuBalanceControlCentiDeg(int *rollControlCentiDeg,
                                           int *pitchControlCentiDeg)
{
    if (rollControlCentiDeg == NULL || pitchControlCentiDeg == NULL) {
        return;
    }
    g_balanceRollControlCentiDeg =
        g_balanceRollControlCentiDeg * EDOG_12DOF_IMU_BALANCE_FAIL_DECAY_PERCENT / 100;
    g_balancePitchControlCentiDeg =
        g_balancePitchControlCentiDeg * EDOG_12DOF_IMU_BALANCE_FAIL_DECAY_PERCENT / 100;
    g_balanceLastRollOutputCentiDeg = g_balanceRollControlCentiDeg;
    g_balanceLastPitchOutputCentiDeg = g_balancePitchControlCentiDeg;
#if EDOG_12DOF_IMU_BALANCE_ENABLED
    g_balanceRollRateCentiDps =
        g_balanceRollRateCentiDps * EDOG_12DOF_IMU_BALANCE_FAIL_DECAY_PERCENT / 100;
    g_balancePitchRateCentiDps =
        g_balancePitchRateCentiDps * EDOG_12DOF_IMU_BALANCE_FAIL_DECAY_PERCENT / 100;
#endif
    *rollControlCentiDeg = g_balanceRollControlCentiDeg;
    *pitchControlCentiDeg = g_balancePitchControlCentiDeg;
    if (g_balanceRollControlCentiDeg == 0 && g_balancePitchControlCentiDeg == 0) {
        g_balanceControlReady = 0;
    }
}

static void getImuBalanceControlCentiDeg(int *rollControlCentiDeg,
                                         int *pitchControlCentiDeg)
{
    int rollCentiDeg = 0;
    int pitchCentiDeg = 0;
    int rollRateCentiDps = 0;
    int pitchRateCentiDps = 0;
    int readOk;
    uint64_t now;
    uint64_t elapsedMs;

    if (rollControlCentiDeg == NULL || pitchControlCentiDeg == NULL) {
        return;
    }

    now = getTimeMs();
    elapsedMs = now >= g_balanceModeStartMs ? now - g_balanceModeStartMs : 0;
    readOk = readImuBalanceMotionCenti(&rollCentiDeg, &pitchCentiDeg,
                                       &rollRateCentiDps, &pitchRateCentiDps) == 0;

    if (g_balanceMode == EDOG_IMU_BALANCE_MODE_STOP_SETTLING) {
        if (readOk && imuBalanceMotionIsStable(rollCentiDeg, pitchCentiDeg,
                                               rollRateCentiDps, pitchRateCentiDps)) {
            g_balanceStableFrames++;
        } else {
            g_balanceStableFrames = 0;
        }
        clearImuBalanceControlState();
        *rollControlCentiDeg = 0;
        *pitchControlCentiDeg = 0;
        if ((elapsedMs >= EDOG_12DOF_IMU_BALANCE_STOP_SETTLE_MS &&
             g_balanceStableFrames >= EDOG_12DOF_IMU_BALANCE_STOP_STABLE_FRAMES) ||
            elapsedMs >= EDOG_12DOF_IMU_BALANCE_STOP_MAX_SETTLE_MS) {
            enterImuBalanceMode(EDOG_IMU_BALANCE_MODE_RAMP_IN);
        }
        return;
    }

    if (!readOk) {
        decayImuBalanceControlCentiDeg(rollControlCentiDeg, pitchControlCentiDeg);
        return;
    }

    if (g_balanceMode == EDOG_IMU_BALANCE_MODE_RAMP_IN &&
        imuBalanceMotionExceedsExit(rollCentiDeg, pitchCentiDeg,
                                    rollRateCentiDps, pitchRateCentiDps)) {
        enterImuBalanceMode(EDOG_IMU_BALANCE_MODE_STOP_SETTLING);
        *rollControlCentiDeg = 0;
        *pitchControlCentiDeg = 0;
        return;
    }

    prepareImuBalanceControlCentiDeg(rollCentiDeg, pitchCentiDeg,
                                     rollRateCentiDps, pitchRateCentiDps,
                                     rollControlCentiDeg, pitchControlCentiDeg);
    if (g_balanceMode == EDOG_IMU_BALANCE_MODE_RAMP_IN) {
        applyImuBalanceRampScale(rollControlCentiDeg, pitchControlCentiDeg,
                                 elapsedMs);
        if (elapsedMs >= EDOG_12DOF_IMU_BALANCE_RAMP_IN_MS) {
            enterImuBalanceMode(EDOG_IMU_BALANCE_MODE_NORMAL);
        }
    } else {
        g_balanceMode = EDOG_IMU_BALANCE_MODE_NORMAL;
    }
}

static double footHipPlaneYOffset(const Edog12DofFootPoint *foot, int leg)
{
    if (foot == NULL) {
        return 0.0;
    }
    return foot->yMm - Edog12Dof_FootYOnHipPlaneForLeg(leg, foot->zMm);
}

static double computeImuBodyZUpCompMm(int leg, int rollCentiDeg,
                                      int pitchCentiDeg, int isSwing)
{
    double leftSign;
    double frontSign;
    double rollErrorDeg;
    double pitchErrorDeg;
    double bodyZUpCompMm;

    if (leg < 0 || leg >= EDOG_12DOF_LEG_COUNT) {
        return 0.0;
    }

    leftSign = (leg == 0 || leg == 2) ? 1.0 : -1.0;
    frontSign = (leg == 0 || leg == 1) ? 1.0 : -1.0;
    rollErrorDeg = (double)rollCentiDeg / (double)EDOG_12DOF_IMU_CENTI_PER_DEG;
    pitchErrorDeg = (double)pitchCentiDeg / (double)EDOG_12DOF_IMU_CENTI_PER_DEG;

    /*
     * Body coordinates: X forward, Y left, Z up.
     * Positive compensation means lifting that foot in body Z-up coordinates,
     * which (since foot.zMm is Z-down) corresponds to foot.zMm decreasing =
     * leg retracting (shrinking). Callers convert with zMm -= comp.
     *
     * Self-balance goal: HIGH side legs retract (zMm down), LOW side legs extend
     * (zMm up) so the body is pushed back to level.
     *
     * MPU tilt sign: left-side down -> roll > 0; front-side down -> pitch < 0.
     * leftSign  = +1 for left legs (LF/LB), -1 for right legs (RF/RB).
     * frontSign = +1 for front legs (LF/RF), -1 for rear legs (LB/RB).
     *
     * With the signs below:
     *  - Left high / right low (roll < 0): left legs bodyZUp > 0 -> retract,
     *    right legs bodyZUp < 0 -> extend.
     *  - Front high / rear low (pitch > 0): front legs bodyZUp > 0 -> retract,
     *    rear legs bodyZUp < 0 -> extend.
     */
    bodyZUpCompMm =
        -leftSign * rollErrorDeg * EDOG_12DOF_IMU_BALANCE_ROLL_KP_MM_PER_DEG +
         frontSign * pitchErrorDeg * EDOG_12DOF_IMU_BALANCE_PITCH_KP_MM_PER_DEG;
    if (isSwing) {
        bodyZUpCompMm =
            bodyZUpCompMm * EDOG_12DOF_IMU_BALANCE_SWING_SCALE_PERCENT / 100.0;
    }
    bodyZUpCompMm = bodyZUpCompMm * (double)g_imuBalanceStrengthPercent / 100.0;
    return clampDoubleValue(bodyZUpCompMm,
                            -EDOG_12DOF_IMU_BALANCE_MAX_FOOT_Z_MM,
                            EDOG_12DOF_IMU_BALANCE_MAX_FOOT_Z_MM);
}

static void applyImuBalanceFootCompensation(Edog12DofFootPoint *foot, int leg,
                                            int isSwing, int rollCentiDeg,
                                            int pitchCentiDeg)
{
    double lateralOffsetMm;
    double bodyZUpCompMm;

    if (foot == NULL) {
        return;
    }

    lateralOffsetMm = footHipPlaneYOffset(foot, leg);
    bodyZUpCompMm = computeImuBodyZUpCompMm(leg, rollCentiDeg, pitchCentiDeg, isSwing);
    foot->zMm -= bodyZUpCompMm;
    foot->yMm = Edog12Dof_FootYOnHipPlaneForLeg(leg, foot->zMm) + lateralOffsetMm;
}

static Edog12DofFootPoint getDefaultFootPoint(int leg)
{
    Edog12DofFootPoint foot = {
        Edog12Dof_DefaultFootXForLeg(leg == 0 || leg == 1),
        Edog12Dof_DefaultFootYForLeg(leg),
        Edog12Dof_DefaultFootZForLeg(leg == 0 || leg == 1)
    };
    return foot;
}
static Edog12DofJointAngles getStandJointAnglesForLeg(int leg)
{
    Edog12DofJointAngles angles = g_standJointAngles;
    Edog12DofFootPoint foot;

    if (leg < 0 || leg >= EDOG_12DOF_LEG_COUNT) {
        return angles;
    }

    foot = getDefaultFootPoint(leg);
    if (Edog12Dof_IK(&foot, g_legs[leg].isRightLeg, &angles) != 0) {
        return g_standJointAngles;
    }
    return angles;
}

static void fillStandFrame(Edog12DofJointAngles frame[EDOG_12DOF_LEG_COUNT])
{
    if (frame == NULL) {
        return;
    }

    for (int leg = 0; leg < EDOG_12DOF_LEG_COUNT; leg++) {
        frame[leg] = getStandJointAnglesForLeg(leg);
    }
}

static int realtimeLegIsSwing(int leg, int index, int crawlMode, double *phaseOut)
{
    double cyclePhase = (double)index / (double)EDOG_12DOF_TROT_FRAME_COUNT;
    double phase = cyclePhase;
    double swingPortion = crawlMode ? EDOG_12DOF_CRAWL_SWING_PORTION :
        EDOG_12DOF_REALTIME_TROT_SWING_PORTION;

    if (crawlMode) {
        phase -= (double)getCrawlOrderPosition(leg) * EDOG_12DOF_CRAWL_PHASE_SPACING;
    } else if (leg == 1 || leg == 2) {
        phase -= 0.5;
    }

    while (phase < 0.0) {
        phase += 1.0;
    }
    while (phase >= 1.0) {
        phase -= 1.0;
    }
    if (phaseOut != NULL) {
        *phaseOut = phase / swingPortion;
        if (*phaseOut > 1.0) {
            *phaseOut = 1.0;
        }
    }
    return phase < swingPortion;
}

static double pyAppleCrawlCycloidProgress(double t)
{
    double sigma;

    t = clampDoubleValue(t, 0.0, 1.0);
    sigma = 2.0 * M_PI * t;
    return (sigma - sin(sigma)) / (2.0 * M_PI);
}

static double pyAppleCrawlCycloidLift(double t)
{
    double sigma;

    t = clampDoubleValue(t, 0.0, 1.0);
    sigma = 2.0 * M_PI * t;
    return (1.0 - cos(sigma)) / 2.0;
}

static double legacyPupperSwingLift(double swingT)
{
    double u;

    swingT = clampDoubleValue(swingT, 0.0, 1.0);
    u = 1.0 - swingT;
    return 4.0 * u * swingT;
}

static void planRealtimeTouchdown(Edog12DofRealtimeLegState *legState,
                                  const Edog12DofRealtimeCommand *cmd,
                                  int leg)
{
    Edog12DofFootPoint neutral = getDefaultFootPoint(leg);
    double legSideSign = (leg == 0 || leg == 2) ? 1.0 : -1.0;
    double frontSign = (leg == 0 || leg == 1) ? 1.0 : -1.0;
    double strideMm = cmd->vxMps * 1000.0;
    double sideMm = clampDoubleValue(cmd->vyMps * 1000.0,
                                     -EDOG_12DOF_REALTIME_MAX_SIDE_MM,
                                     EDOG_12DOF_REALTIME_MAX_SIDE_MM);
    double yawStepMm = clampDoubleValue(cmd->yawRate * 1000.0,
                                        -EDOG_12DOF_REALTIME_MAX_YAW_MM,
                                        EDOG_12DOF_REALTIME_MAX_YAW_MM);
    double hardwareStrideMm = EDOG_12DOF_HARDWARE_FORWARD_SIGN * strideMm;

    legState->liftOff = legState->foot;
    legState->touchdown = neutral;
    legState->touchdown.xMm += hardwareStrideMm + legSideSign * yawStepMm;
    legState->touchdown.yMm += legSideSign * sideMm + frontSign * legSideSign * yawStepMm;
}

static void initRealtimeGaitState(Edog12DofRealtimeGaitState *state,
                                  const Edog12DofRealtimeCommand *cmd)
{
    (void)cmd;
    for (int leg = 0; leg < EDOG_12DOF_LEG_COUNT; leg++) {
        state->leg[leg].foot = getDefaultFootPoint(leg);
        state->leg[leg].liftOff = state->leg[leg].foot;
        state->leg[leg].touchdown = state->leg[leg].foot;
        state->leg[leg].isSwing = 0;
    }
}

static void updateRealtimeStanceFoot(Edog12DofRealtimeLegState *legState,
                                     const Edog12DofRealtimeCommand *cmd,
                                     int leg)
{
    Edog12DofFootPoint neutral = getDefaultFootPoint(leg);
    double legSideSign = (leg == 0 || leg == 2) ? 1.0 : -1.0;
    double frontSign = (leg == 0 || leg == 1) ? 1.0 : -1.0;
    double dx = clampDoubleValue(cmd->vxMps * 1000.0 / (double)EDOG_12DOF_TROT_FRAME_COUNT,
                                -4.0, 4.0);
    double dy = clampDoubleValue(cmd->vyMps * 1000.0 / (double)EDOG_12DOF_TROT_FRAME_COUNT,
                                -3.0, 3.0);
    double yawStepMm = clampDoubleValue(cmd->yawRate * 1000.0 /
                                           (double)EDOG_12DOF_TROT_FRAME_COUNT,
                                       -3.0, 3.0);
    double hardwareDx = EDOG_12DOF_HARDWARE_FORWARD_SIGN * dx;

    legState->foot.xMm -= hardwareDx + legSideSign * yawStepMm;
    legState->foot.yMm -= legSideSign * dy + frontSign * legSideSign * yawStepMm;
    legState->foot.xMm += (neutral.xMm - legState->foot.xMm) * 0.05;
    legState->foot.yMm += (neutral.yMm - legState->foot.yMm) * 0.05;
    legState->foot.zMm = neutral.zMm + EDOG_12DOF_REALTIME_STANCE_PRESS_MM;
    legState->foot.yMm = Edog12Dof_FootYOnHipPlaneForLeg(leg, legState->foot.zMm) +
        (legState->foot.yMm - neutral.yMm);
}

static double realtimeSwingLift(double swingT)
{
    if (swingT < 0.5) {
        return swingT / 0.5;
    }
    return (1.0 - swingT) / 0.5;
}

static void updateRealtimeSwingFoot(Edog12DofRealtimeLegState *legState,
                                    const Edog12DofRealtimeCommand *cmd,
                                    int leg,
                                    double swingT)
{
    double liftMm = minimumLiftMm(cmd->stepHeightM * 1000.0,
                                EDOG_12DOF_REALTIME_LEGACY_MIN_LIFT_MM);
    double swingLift = realtimeSwingLift(swingT);
    double u = clampDoubleValue(swingT, 0.0, 1.0);
    double defaultFootZMm = Edog12Dof_DefaultFootZForLeg(leg == 0 || leg == 1);
    double lateralOffsetMm = footHipPlaneYOffset(&legState->liftOff, leg) +
        (footHipPlaneYOffset(&legState->touchdown, leg) -
         footHipPlaneYOffset(&legState->liftOff, leg)) * u;

    legState->foot.xMm = legState->liftOff.xMm +
        (legState->touchdown.xMm - legState->liftOff.xMm) * u;
    legState->foot.zMm = defaultFootZMm - liftMm * swingLift;
    legState->foot.yMm = Edog12Dof_FootYOnHipPlaneForLeg(leg, legState->foot.zMm) +
        lateralOffsetMm;
}

static void EDOG_UNUSED keepRealtimeLegacyHelpersReferenced(void)
{
    (void)realtimeLegIsSwing;
    (void)planRealtimeTouchdown;
    (void)updateRealtimeStanceFoot;
    (void)realtimeSwingLift;
    (void)updateRealtimeSwingFoot;
}

static int pupperPhaseIndex(int tick)
{
    int phaseTime;
    int phaseSum = 0;

    if (tick < 0) {
        tick = 0;
    }
    phaseTime = tick % EDOG_PUPPER_PHASE_LENGTH;
    for (int phase = 0; phase < EDOG_PUPPER_NUM_PHASES; phase++) {
        phaseSum += g_pupperPhaseTicks[phase];
        if (phaseTime < phaseSum) {
            return phase;
        }
    }
    return EDOG_PUPPER_NUM_PHASES - 1;
}

static int pupperSubphaseTicks(int tick)
{
    int phaseTime;
    int phaseSum = 0;

    if (tick < 0) {
        tick = 0;
    }
    phaseTime = tick % EDOG_PUPPER_PHASE_LENGTH;
    for (int phase = 0; phase < EDOG_PUPPER_NUM_PHASES; phase++) {
        phaseSum += g_pupperPhaseTicks[phase];
        if (phaseTime < phaseSum) {
            return phaseTime - phaseSum + g_pupperPhaseTicks[phase];
        }
    }
    return 0;
}

static int pupperLegIsContact(int leg, int phaseIndex)
{
    if (leg < 0 || leg >= EDOG_12DOF_LEG_COUNT ||
        phaseIndex < 0 || phaseIndex >= EDOG_PUPPER_NUM_PHASES) {
        return 1;
    }
    return g_pupperContactPhases[leg][phaseIndex] != 0;
}

static int staticCrawlPhaseIndex(int tick)
{
    int phaseTime;
    int phaseSum = 0;

    if (tick < 0) {
        tick = 0;
    }
    phaseTime = tick % EDOG_STATIC_CRAWL_PHASE_LENGTH;
    for (int phase = 0; phase < EDOG_STATIC_CRAWL_NUM_PHASES; phase++) {
        phaseSum += g_staticCrawlPhaseTicks[phase];
        if (phaseTime < phaseSum) {
            return phase;
        }
    }
    return EDOG_STATIC_CRAWL_NUM_PHASES - 1;
}

static int staticCrawlSubphaseTicks(int tick)
{
    int phaseTime;
    int phaseSum = 0;

    if (tick < 0) {
        tick = 0;
    }
    phaseTime = tick % EDOG_STATIC_CRAWL_PHASE_LENGTH;
    for (int phase = 0; phase < EDOG_STATIC_CRAWL_NUM_PHASES; phase++) {
        phaseSum += g_staticCrawlPhaseTicks[phase];
        if (phaseTime < phaseSum) {
            return phaseTime - phaseSum + g_staticCrawlPhaseTicks[phase];
        }
    }
    return 0;
}

static int staticCrawlLegIsContact(int leg, int phaseIndex)
{
    if (leg < 0 || leg >= EDOG_12DOF_LEG_COUNT ||
        phaseIndex < 0 || phaseIndex >= EDOG_STATIC_CRAWL_NUM_PHASES) {
        return 1;
    }
    return g_staticCrawlContactPhases[leg][phaseIndex] != 0;
}

static int staticCrawlSwingLegForPhase(int phaseIndex)
{
    if (phaseIndex < 0 || phaseIndex >= EDOG_STATIC_CRAWL_NUM_PHASES ||
        (phaseIndex % 2) == 0) {
        return -1;
    }
    return g_crawlSwingOrder[phaseIndex / 2];
}

static int staticCrawlNextSwingLeg(int phaseIndex)
{
    int activeSwingLeg = staticCrawlSwingLegForPhase(phaseIndex);

    if (phaseIndex < 0 || phaseIndex >= EDOG_STATIC_CRAWL_NUM_PHASES) {
        return -1;
    }
    if (activeSwingLeg >= 0) {
        return -1;
    }
    return g_crawlSwingOrder[phaseIndex / 2];
}

static int realtimePhaseIndex(const Edog12DofRealtimeCommand *cmd, int tick)
{
    if (cmd != NULL && cmd->crawlMode) {
        return staticCrawlPhaseIndex(tick);
    }
    return pupperPhaseIndex(tick);
}

static int realtimeSubphaseTicks(const Edog12DofRealtimeCommand *cmd, int tick)
{
    if (cmd != NULL && cmd->crawlMode) {
        return staticCrawlSubphaseTicks(tick);
    }
    return pupperSubphaseTicks(tick);
}

static int realtimeLegIsContact(const Edog12DofRealtimeCommand *cmd, int leg, int phaseIndex)
{
    if (cmd != NULL && cmd->crawlMode) {
        return staticCrawlLegIsContact(leg, phaseIndex);
    }
    return pupperLegIsContact(leg, phaseIndex);
}

static int realtimeSwingTicks(const Edog12DofRealtimeCommand *cmd)
{
    if (cmd != NULL && cmd->crawlMode) {
        return EDOG_STATIC_CRAWL_SWING_TICKS;
    }
    return EDOG_PUPPER_SWING_TICKS;
}

static int realtimeStanceTicks(const Edog12DofRealtimeCommand *cmd)
{
    if (cmd != NULL && cmd->crawlMode) {
        return EDOG_STATIC_CRAWL_STANCE_TICKS;
    }
    return EDOG_PUPPER_STANCE_TICKS;
}

static int realtimePhaseLength(const Edog12DofRealtimeCommand *cmd)
{
    if (cmd != NULL && cmd->crawlMode) {
        return EDOG_STATIC_CRAWL_PHASE_LENGTH;
    }
    return EDOG_PUPPER_PHASE_LENGTH;
}

static double pupperCommandDeltaMm(const Edog12DofRealtimeCommand *cmd,
                                   double commandMps)
{
    double stanceSeconds = (double)realtimeStanceTicks(cmd) * EDOG_PUPPER_DT_SEC;
    double velocityMps = commandMps * EDOG_PUPPER_COMMAND_VELOCITY_SCALE;

    return EDOG_PUPPER_ALPHA * stanceSeconds * velocityMps * 1000.0;
}

static double pupperCommandVelocityStepMm(double commandMps, double maxPerFrameMm)
{
    double velocityMps = commandMps * EDOG_PUPPER_COMMAND_VELOCITY_SCALE;

    return clampDoubleValue(velocityMps * 1000.0 * EDOG_PUPPER_DT_SEC,
                            -maxPerFrameMm, maxPerFrameMm);
}

static void pupperPlanTouchdown(Edog12DofRealtimeLegState *legState,
                                const Edog12DofRealtimeCommand *cmd,
                                int leg)
{
    Edog12DofFootPoint neutral = getDefaultFootPoint(leg);
    double legSideSign = (leg == 0 || leg == 2) ? 1.0 : -1.0;
    double frontSign = (leg == 0 || leg == 1) ? 1.0 : -1.0;
    double stanceSeconds = (double)realtimeStanceTicks(cmd) * EDOG_PUPPER_DT_SEC;
    double deltaXmm = pupperCommandDeltaMm(cmd, cmd->vxMps);
    double deltaYmm = clampDoubleValue(pupperCommandDeltaMm(cmd, cmd->vyMps),
                                       -EDOG_12DOF_REALTIME_MAX_SIDE_MM,
                                       EDOG_12DOF_REALTIME_MAX_SIDE_MM);
    double yawStepMm = clampDoubleValue(cmd->yawRate * 1000.0,
                                        -EDOG_12DOF_REALTIME_MAX_YAW_MM,
                                        EDOG_12DOF_REALTIME_MAX_YAW_MM);
    double yawRateRad = 0.0;
    double theta;
    double yawArmX;
    double yawArmY;
    double rotatedX;
    double rotatedY;
    double yawDeltaX;
    double yawDeltaY;
    double hardwareStrideMm = EDOG_12DOF_HARDWARE_FORWARD_SIGN * deltaXmm;

    if (stanceSeconds > 0.001) {
        yawRateRad = yawStepMm / (EDOG_PUPPER_YAW_RADIUS_MM * stanceSeconds);
    }
    theta = EDOG_PUPPER_BETA * stanceSeconds * yawRateRad;
    yawArmX = frontSign * EDOG_PUPPER_YAW_ARM_MM;
    yawArmY = legSideSign * EDOG_PUPPER_YAW_RADIUS_MM;
    rotatedX = yawArmX * cos(theta) - yawArmY * sin(theta);
    rotatedY = yawArmX * sin(theta) + yawArmY * cos(theta);
    yawDeltaX = rotatedX - yawArmX;
    yawDeltaY = rotatedY - yawArmY;

    legState->liftOff = legState->foot;
    legState->touchdown = neutral;
    legState->touchdown.xMm += hardwareStrideMm - yawDeltaX;
    legState->touchdown.yMm += legSideSign * deltaYmm + yawDeltaY;
    legState->touchdown.zMm = neutral.zMm;
}

static void updatePupperStanceFoot(Edog12DofRealtimeLegState *legState,
                                   const Edog12DofRealtimeCommand *cmd,
                                   int leg)
{
    Edog12DofFootPoint neutral = getDefaultFootPoint(leg);
    double legSideSign = (leg == 0 || leg == 2) ? 1.0 : -1.0;
    double frontSign = (leg == 0 || leg == 1) ? 1.0 : -1.0;
    double dx = pupperCommandVelocityStepMm(cmd->vxMps, 4.0);
    double dy = pupperCommandVelocityStepMm(cmd->vyMps, 3.0);
    double yawStepMm = clampDoubleValue(cmd->yawRate * 1000.0,
                                        -EDOG_12DOF_REALTIME_MAX_YAW_MM,
                                        EDOG_12DOF_REALTIME_MAX_YAW_MM);
    double yawVelocityMm = clampDoubleValue(yawStepMm / (double)realtimeStanceTicks(cmd),
                                           -3.0, 3.0);
    double yawTheta = -yawVelocityMm / EDOG_PUPPER_YAW_RADIUS_MM;
    double yawArmX = frontSign * EDOG_PUPPER_YAW_ARM_MM;
    double yawArmY = legSideSign * EDOG_PUPPER_YAW_RADIUS_MM;
    double rotatedX = yawArmX * cos(yawTheta) - yawArmY * sin(yawTheta);
    double rotatedY = yawArmX * sin(yawTheta) + yawArmY * cos(yawTheta);
    double yawDx = -(rotatedX - yawArmX);
    double yawDy = rotatedY - yawArmY;
    double hardwareDx = EDOG_12DOF_HARDWARE_FORWARD_SIGN * dx;
    double hardwareDy = legSideSign * dy;

    legState->foot.xMm -= hardwareDx;
    legState->foot.yMm -= hardwareDy;
    legState->foot.xMm -= yawDx;
    legState->foot.yMm -= yawDy;
    /* stance velocity already applied via hardwareDx */
    /* stance velocity already applied via hardwareDy */
    if (cmd != NULL && cmd->crawlMode) {
        legState->foot.zMm = neutral.zMm;
    } else {
        legState->foot.zMm += (neutral.zMm - legState->foot.zMm) * 0.45;
    }
    legState->foot.yMm = Edog12Dof_FootYOnHipPlaneForLeg(leg, legState->foot.zMm) +
        (legState->foot.yMm - neutral.yMm);
}

static double pupperSwingHeight(const Edog12DofRealtimeCommand *cmd, double swingT)
{
    double liftMm;

    if (cmd == NULL) {
        return 0.0;
    }
    liftMm = minimumLiftMm(cmd->stepHeightM * 1000.0,
                         EDOG_PUPPER_MIN_LIFT_MM);
    if (cmd->crawlMode) {
        return liftMm * pyAppleCrawlCycloidLift(swingT);
    }
    return liftMm * legacyPupperSwingLift(swingT);
}

static void updatePupperSwingFoot(Edog12DofRealtimeLegState *legState,
                                  const Edog12DofRealtimeCommand *cmd,
                                  int leg,
                                  double swingT)
{
    double swingHeightMm;
    double defaultFootZMm = Edog12Dof_DefaultFootZForLeg(leg == 0 || leg == 1);
    double lateralOffsetMm;
    double t = clampDoubleValue(swingT, 0.0, 1.0);
    double progress = (cmd != NULL && cmd->crawlMode) ?
        pyAppleCrawlCycloidProgress(t) : t;

    swingHeightMm = pupperSwingHeight(cmd, t);
    legState->foot.xMm = legState->liftOff.xMm +
        (legState->touchdown.xMm - legState->liftOff.xMm) * progress;
    legState->foot.yMm = legState->liftOff.yMm +
        (legState->touchdown.yMm - legState->liftOff.yMm) * progress;
    lateralOffsetMm = footHipPlaneYOffset(&legState->foot, leg);
    legState->foot.zMm = defaultFootZMm - swingHeightMm;
    legState->foot.yMm = Edog12Dof_FootYOnHipPlaneForLeg(leg, legState->foot.zMm) +
        lateralOffsetMm;
}

static void applyRealtimeBalanceBias(Edog12DofJointAngles frame[EDOG_12DOF_LEG_COUNT],
                                     const int swingState[EDOG_12DOF_LEG_COUNT])
{
    int swingLeg = -1;

    for (int leg = 0; leg < EDOG_12DOF_LEG_COUNT; leg++) {
        if (swingState[leg]) {
            swingLeg = leg;
            break;
        }
    }
    applyCrawlBalanceBias(frame, swingState, swingLeg);
}

static void applyRealtimeFrame(Edog12DofRealtimeGaitState *state,
                               const Edog12DofRealtimeCommand *cmd,
                               int tick)
{
    Edog12DofJointAngles frame[EDOG_12DOF_LEG_COUNT];
    int swingState[EDOG_12DOF_LEG_COUNT] = {0};
    int rollControlCentiDeg = 0;
    int pitchControlCentiDeg = 0;
    uint64_t usedUs = 0;
    int phaseIndex = realtimePhaseIndex(cmd, tick);
    int subphaseTicks = realtimeSubphaseTicks(cmd, tick);
    int swingTicks = realtimeSwingTicks(cmd);
    int preShiftLeg = staticCrawlNextSwingLeg(phaseIndex);

    getImuBalanceControlCentiDeg(&rollControlCentiDeg, &pitchControlCentiDeg);

    for (int leg = 0; leg < EDOG_12DOF_LEG_COUNT; leg++) {
        double swingT = 0.0;
        int isContact = realtimeLegIsContact(cmd, leg, phaseIndex);

        if (!isContact) {
            swingT = (double)subphaseTicks / (double)swingTicks;
            swingT = clampDoubleValue(swingT, 0.0, 1.0);
        }
        if (!isContact && !state->leg[leg].isSwing) {
            pupperPlanTouchdown(&state->leg[leg], cmd, leg);
        }
        state->leg[leg].isSwing = !isContact;
        if (!isContact) {
            updatePupperSwingFoot(&state->leg[leg], cmd, leg, swingT);
        } else {
            updatePupperStanceFoot(&state->leg[leg], cmd, leg);
        }
        swingState[leg] = !isContact;
        applyImuBalanceFootCompensation(&state->leg[leg].foot, leg, !isContact,
                                        rollControlCentiDeg, pitchControlCentiDeg);
        if (Edog12Dof_IK(&state->leg[leg].foot, g_legs[leg].isRightLeg, &frame[leg]) != 0) {
            frame[leg] = getStandJointAnglesForLeg(leg);
        } else {
            applyRearLegSwingBoost(leg, !isContact, &frame[leg]);
        }
    }

    if (cmd->crawlMode) {
        applyHipWeightShiftBias(frame, swingState, preShiftLeg, EDOG_12DOF_CRAWL_PRE_SHIFT_HIP_BIAS_DEG);
        applyRealtimeBalanceBias(frame, swingState);
    }
    for (int leg = 0; leg < EDOG_12DOF_LEG_COUNT; leg++) {
        addFrameUsedTime(&usedUs, setGaitLegAnglesStaggeredSmooth(leg, &frame[leg],
                                                                  EDOG_12DOF_REALTIME_SERVO_STEP_DELAY_US));
    }
    sleepRemainingFrameTime(usedUs);
}

static int runRealtimeGaitCycle(const Edog12DofRealtimeCommand *cmd)
{
    Edog12DofRealtimeGaitState state;

    if (cmd == NULL) {
        return 0;
    }
    initRealtimeGaitState(&state, cmd);
    for (int i = 0; i < realtimePhaseLength(cmd); i++) {
        if (stopFlag) {
            return 0;
        }
        applyRealtimeFrame(&state, cmd, i);
    }
    return 1;
}

static int EDOG_UNUSED trotCycleInternal(double stepLength, double stepHeight,
                                         int backward, int leftScalePercent,
                                         int rightScalePercent)
{
    Edog12DofGaitSet *gaitSet = &g_legacyMotionGaitSet;
    int stepLengthCentiMm = doubleToCentiMm(stepLength * 1000.0);
    int stepHeightCentiMm = doubleToCentiMm(stepHeight * 1000.0);
    int mode = backward ? EDOG_TABLE_MODE_BACKWARD : EDOG_TABLE_MODE_FORWARD;
    int frameIndex;

    if (stopFlag) {
        g_continuousTrotRuntime.active = 0;
        return 0;
    }
    if (!g_continuousTrotRuntime.active ||
        g_continuousTrotRuntime.mode != mode ||
        g_continuousTrotRuntime.stepLengthCentiMm != stepLengthCentiMm ||
        g_continuousTrotRuntime.stepHeightCentiMm != stepHeightCentiMm ||
        g_continuousTrotRuntime.backward != backward ||
        g_continuousTrotRuntime.leftScalePercent != leftScalePercent ||
        g_continuousTrotRuntime.rightScalePercent != rightScalePercent) {
        g_continuousTrotRuntime.active = 1;
        g_continuousTrotRuntime.mode = mode;
        g_continuousTrotRuntime.frameIndex = 0;
        g_continuousTrotRuntime.stepLengthCentiMm = stepLengthCentiMm;
        g_continuousTrotRuntime.stepHeightCentiMm = stepHeightCentiMm;
        g_continuousTrotRuntime.backward = backward;
        g_continuousTrotRuntime.leftScalePercent = leftScalePercent;
        g_continuousTrotRuntime.rightScalePercent = rightScalePercent;
        buildScaledGaitSet(*gaitSet, stepLength, stepHeight, backward,
                           leftScalePercent, rightScalePercent);
        enterImuBalanceMode(EDOG_IMU_BALANCE_MODE_NORMAL);
    }

    frameIndex = g_continuousTrotRuntime.frameIndex;
    apply8DofTableFrameStaggered(*gaitSet, frameIndex, EDOG_12DOF_TROT_FRAME_COUNT / 2);
    g_continuousTrotRuntime.frameIndex =
        (g_continuousTrotRuntime.frameIndex + 1) % EDOG_12DOF_TROT_FRAME_COUNT;
    return g_continuousTrotRuntime.frameIndex == 0 ? 1 : -1;
}

static int EDOG_UNUSED trotTurnCycleInternal(double stepLength, double stepHeight, int turnLeft)
{
    Edog12DofGaitSet *forwardGait = &g_legacyTurnForwardGaitSet;
    Edog12DofGaitSet *reverseGait = &g_legacyTurnReverseGaitSet;
    int stepLengthCentiMm = doubleToCentiMm(stepLength * 1000.0);
    int stepHeightCentiMm = doubleToCentiMm(stepHeight * 1000.0);
    int mode = turnLeft ? EDOG_TABLE_MODE_TURN_LEFT : EDOG_TABLE_MODE_TURN_RIGHT;
    int frameIndex;

    if (stopFlag) {
        g_continuousTrotRuntime.active = 0;
        return 0;
    }
    if (!g_continuousTrotRuntime.active ||
        g_continuousTrotRuntime.mode != mode ||
        g_continuousTrotRuntime.stepLengthCentiMm != stepLengthCentiMm ||
        g_continuousTrotRuntime.stepHeightCentiMm != stepHeightCentiMm ||
        g_continuousTrotRuntime.turnLeft != turnLeft) {
        g_continuousTrotRuntime.active = 1;
        g_continuousTrotRuntime.mode = mode;
        g_continuousTrotRuntime.frameIndex = 0;
        g_continuousTrotRuntime.stepLengthCentiMm = stepLengthCentiMm;
        g_continuousTrotRuntime.stepHeightCentiMm = stepHeightCentiMm;
        g_continuousTrotRuntime.turnLeft = turnLeft;
        buildScaledGaitSet(*forwardGait, stepLength, stepHeight, 0, 100, 100);
        buildScaledGaitSet(*reverseGait, stepLength, stepHeight, 1, 100, 100);
        enterImuBalanceMode(EDOG_IMU_BALANCE_MODE_NORMAL);
    }

    frameIndex = g_continuousTrotRuntime.frameIndex;
    apply8DofTurnFrameStaggered(*forwardGait, *reverseGait, frameIndex, turnLeft);
    g_continuousTrotRuntime.frameIndex =
        (g_continuousTrotRuntime.frameIndex + 1) % EDOG_12DOF_TROT_FRAME_COUNT;
    return g_continuousTrotRuntime.frameIndex == 0 ? 1 : -1;
}

static int EDOG_UNUSED legacyRealtimeDirectionalCycleInternal(double forwardStep, double sideStep,
                                                              double yawStep, double stepHeight,
                                                              int backward)
{
    Edog12DofRealtimeCommand cmd = {
        forwardStep,
        sideStep,
        yawStep,
        stepHeight,
        0,
        backward
    };

    return runRealtimeGaitCycle(&cmd);
}

static void buildCrawlLegFrame(Edog12DofJointAngles *out,
                               const Edog12DofJointAngles gait[EDOG_12DOF_TROT_FRAME_COUNT],
                               double phase, int *isSwing)
{
    int gaitIndex;

    if (out == NULL || gait == NULL) {
        return;
    }

    while (phase < 0.0) {
        phase += 1.0;
    }
    while (phase >= 1.0) {
        phase -= 1.0;
    }

    if (phase < EDOG_12DOF_CRAWL_SWING_PORTION) {
        double swingPhase = phase / EDOG_12DOF_CRAWL_SWING_PORTION;
        gaitIndex = (int)(swingPhase * (double)(EDOG_12DOF_TROT_FRAME_COUNT / 2 - 1) + 0.5);
        if (isSwing != NULL) {
            *isSwing = 1;
        }
        *out = gait[gaitIndex];
        return;
    }

    phase = (phase - EDOG_12DOF_CRAWL_SWING_PORTION) /
        (1.0 - EDOG_12DOF_CRAWL_SWING_PORTION);
    gaitIndex = EDOG_12DOF_TROT_FRAME_COUNT / 2 +
        (int)(phase * (double)(EDOG_12DOF_TROT_FRAME_COUNT - EDOG_12DOF_TROT_FRAME_COUNT / 2 - 1) + 0.5);
    if (gaitIndex >= EDOG_12DOF_TROT_FRAME_COUNT) {
        gaitIndex = EDOG_12DOF_TROT_FRAME_COUNT - 1;
    }
    if (isSwing != NULL) {
        *isSwing = 0;
    }
    *out = gait[gaitIndex];
    out->tibiaAngleDeg = clampJointDeltaDeg(
        out->tibiaAngleDeg + EDOG_12DOF_CRAWL_SUPPORT_CALF_BIAS_DEG);
    /* 厘度字段叠加同一偏置（厘度单位），保留 IK 亚度精度。 */
    out->tibiaAngleCentiDeg = clampJointDeltaCenti(
        out->tibiaAngleCentiDeg +
        EDOG_12DOF_CRAWL_SUPPORT_CALF_BIAS_DEG * EDOG_12DOF_CENTI_PER_DEG);
}

static void applyCrawlBalanceBias(Edog12DofJointAngles frame[EDOG_12DOF_LEG_COUNT],
                                  const int swingState[EDOG_12DOF_LEG_COUNT],
                                  int swingLeg)
{
    int swingIsLeft;

    if (frame == NULL || swingState == NULL || swingLeg < 0 ||
        swingLeg >= EDOG_12DOF_LEG_COUNT || !swingState[swingLeg]) {
        return;
    }

    /*
     * 静态爬行每次只抬一条腿。抬左腿时，把右侧支撑腿髋关节轻微外展；
     * 抬右腿时同理把左侧支撑腿外展，给机身留出反向支撑裕量，减少向摆动腿侧倾。
     */
    swingIsLeft = (swingLeg == 0 || swingLeg == 2);
    for (int leg = 0; leg < EDOG_12DOF_LEG_COUNT; leg++) {
        if (swingState[leg]) {
            continue;
        }
        if (swingIsLeft && g_legs[leg].isRightLeg) {
            frame[leg].hipAngleDeg = clampJointDeltaDeg(
                frame[leg].hipAngleDeg + EDOG_12DOF_CRAWL_BALANCE_HIP_BIAS_DEG);
            frame[leg].hipAngleCentiDeg = clampJointDeltaCenti(
                frame[leg].hipAngleCentiDeg +
                EDOG_12DOF_CRAWL_BALANCE_HIP_BIAS_DEG * EDOG_12DOF_CENTI_PER_DEG);
        } else if (!swingIsLeft && !g_legs[leg].isRightLeg) {
            frame[leg].hipAngleDeg = clampJointDeltaDeg(
                frame[leg].hipAngleDeg + EDOG_12DOF_CRAWL_BALANCE_HIP_BIAS_DEG);
            frame[leg].hipAngleCentiDeg = clampJointDeltaCenti(
                frame[leg].hipAngleCentiDeg +
                EDOG_12DOF_CRAWL_BALANCE_HIP_BIAS_DEG * EDOG_12DOF_CENTI_PER_DEG);
        }
    }
}

static void applyHipWeightShiftBias(Edog12DofJointAngles frame[EDOG_12DOF_LEG_COUNT],
                                    const int swingState[EDOG_12DOF_LEG_COUNT],
                                    int targetLeg, int biasDeg)
{
    int targetIsLeft;

    if (frame == NULL || swingState == NULL || targetLeg < 0 ||
        targetLeg >= EDOG_12DOF_LEG_COUNT || biasDeg == 0) {
        return;
    }

    targetIsLeft = (targetLeg == 0 || targetLeg == 2);
    for (int leg = 0; leg < EDOG_12DOF_LEG_COUNT; leg++) {
        if (swingState[leg]) {
            continue;
        }
        if (targetIsLeft && g_legs[leg].isRightLeg) {
            frame[leg].hipAngleDeg = clampJointDeltaDeg(
                frame[leg].hipAngleDeg + biasDeg);
            frame[leg].hipAngleCentiDeg = clampJointDeltaCenti(
                frame[leg].hipAngleCentiDeg + biasDeg * EDOG_12DOF_CENTI_PER_DEG);
        } else if (!targetIsLeft && !g_legs[leg].isRightLeg) {
            frame[leg].hipAngleDeg = clampJointDeltaDeg(
                frame[leg].hipAngleDeg + biasDeg);
            frame[leg].hipAngleCentiDeg = clampJointDeltaCenti(
                frame[leg].hipAngleCentiDeg + biasDeg * EDOG_12DOF_CENTI_PER_DEG);
        }
    }
}

static void applyCrawlFrameStaggered(const Edog12DofGaitSet gaitSet, int index)
{
    Edog12DofJointAngles frame[EDOG_12DOF_LEG_COUNT];
    int swingState[EDOG_12DOF_LEG_COUNT] = {0};
    uint64_t usedUs = 0;
    int swingLeg = -1;
    double cyclePhase = (double)index / (double)EDOG_12DOF_TROT_FRAME_COUNT;
    int preShiftLeg = -1;

    for (int leg = 0; leg < EDOG_12DOF_LEG_COUNT; leg++) {
        double phase = cyclePhase;
        int isSwing = 0;

        phase -= (double)getCrawlOrderPosition(leg) * EDOG_12DOF_CRAWL_PHASE_SPACING;
        buildCrawlLegFrame(&frame[leg], gaitSet[leg], phase, &isSwing);
        swingState[leg] = isSwing;
        if (isSwing) {
            swingLeg = leg;
        }
    }

    if (swingLeg < 0) {
        int orderSlot = (index * EDOG_12DOF_LEG_COUNT) / EDOG_12DOF_TROT_FRAME_COUNT;
        if (orderSlot < 0) {
            orderSlot = 0;
        } else if (orderSlot >= EDOG_12DOF_LEG_COUNT) {
            orderSlot = EDOG_12DOF_LEG_COUNT - 1;
        }
        preShiftLeg = g_crawlSwingOrder[orderSlot];
    }

    applyHipWeightShiftBias(frame, swingState, preShiftLeg, EDOG_12DOF_CRAWL_PRE_SHIFT_HIP_BIAS_DEG);
    applyCrawlBalanceBias(frame, swingState, swingLeg);

    for (int leg = 0; leg < EDOG_12DOF_LEG_COUNT; leg++) {
        if (!swingState[leg]) {
            addFrameUsedTime(&usedUs, setGaitLegAnglesStaggeredSmooth(leg, &frame[leg],
                                                                      EDOG_SERVO_MOTION_STEP_DELAY_US));
        }
    }
    if (swingLeg >= 0) {
        addFrameUsedTime(&usedUs, setGaitLegAnglesStaggeredSmooth(swingLeg, &frame[swingLeg],
                                                                  EDOG_SERVO_MOTION_STEP_DELAY_US));
    }
    sleepRemainingFrameTime(usedUs);
}

static int EDOG_UNUSED crawlCycleInternal(double stepLength, double stepHeight,
                                          int backward, int leftScalePercent,
                                          int rightScalePercent)
{
    Edog12DofGaitSet *gaitSet = &g_legacyMotionGaitSet;

    buildScaledGaitSet(*gaitSet, stepLength, stepHeight, backward,
                       leftScalePercent, rightScalePercent);

    for (int i = 0; i < EDOG_12DOF_TROT_FRAME_COUNT; i++) {
        if (stopFlag) {
            return 0;
        }
        applyCrawlFrameStaggered(*gaitSet, i);
    }
    return 1;
}

static int spotMicroCrawlCycleInternal(double forwardStep, double sideStep,
                                       double yawStep, double stepHeight)
{
    Edog12DofRealtimeCommand cmd = {0};

    cmd.vxMps = forwardStep;
    cmd.vyMps = sideStep;
    cmd.yawRate = yawStep;
    cmd.stepHeightM = stepHeight;
    cmd.crawlMode = 1;
    cmd.backward = forwardStep < 0.0 ? 1 : 0;

    return runRealtimeGaitCycle(&cmd);
}

int single_leg_gait_cycle(int leg_index, double step_length, double step_height)
{
    if (leg_index < 0 || leg_index >= EDOG_12DOF_LEG_COUNT) {
        printf("[SingleLeg] invalid leg index=%d\n", leg_index);
        return 0;
    }

    return leg_group_gait_cycle(1 << leg_index, step_length, step_height);
}

int leg_group_gait_cycle(int leg_mask, double step_length, double step_height)
{
    if ((leg_mask & 0x0F) == 0) {
        printf("[LegGroup] empty leg mask=%d\n", leg_mask);
        return 0;
    }
    return runServoGaitTableCycle(EDOG_TABLE_MODE_FORWARD, step_length, step_height, leg_mask);
}

int trot_cycle(double step_length, double step_height)
{
    return trotCycleInternal(step_length, step_height, 0, 100, 100);
}

int crawl_cycle(double step_length, double step_height)
{
    return spotMicroCrawlCycleInternal(step_length, 0.0, 0.0, step_height);
}

int trot_in_place_cycle(double step_height)
{
    return trotCycleInternal(0.0, step_height, 0, 100, 100);
}

int trot_back_cycle(double step_length, double step_height)
{
    return trotCycleInternal(step_length, step_height, 1, 100, 100);
}

int diversion_right_cycle(double step_length, double step_height)
{
    return runServoGaitTableCycle(EDOG_TABLE_MODE_TURN_RIGHT, step_length, step_height, 0x0F);
}

int diversion_left_cycle(double step_length, double step_height)
{
    return runServoGaitTableCycle(EDOG_TABLE_MODE_TURN_LEFT, step_length, step_height, 0x0F);
}

int trot_left_front_cycle(double step_length, double step_height)
{
    (void)step_length;
    (void)step_height;
    printf("[12DOF] arc gait not supported: left front\n");
    return 0;
}

int trot_right_front_cycle(double step_length, double step_height)
{
    (void)step_length;
    (void)step_height;
    printf("[12DOF] arc gait not supported: right front\n");
    return 0;
}

int trot_left_back_cycle(double step_length, double step_height)
{
    (void)step_length;
    (void)step_height;
    printf("[12DOF] arc gait not supported: left back\n");
    return 0;
}

int trot_right_back_cycle(double step_length, double step_height)
{
    (void)step_length;
    (void)step_height;
    printf("[12DOF] arc gait not supported: right back\n");
    return 0;
}
