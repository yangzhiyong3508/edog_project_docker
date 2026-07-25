#include "iot_control.h"
#include "../../include/edog_config.h"
#include "iot.h"
#include "motion_utils.h"
#include "servo_control.h"
#include "../../12_DOF_Version/include/gait_generate_12dof.h"
#include "los_mux.h"
#include "los_queue.h"
#include "los_task.h"
#include <stdio.h>
#include <string.h>
#include <unistd.h>

typedef enum {
    MOTION_CMD_NONE = 0,
    MOTION_CMD_TROT,
    MOTION_CMD_TROT_IN_PLACE,
    MOTION_CMD_TROT_BACK,
    MOTION_CMD_TURN_LEFT,
    MOTION_CMD_TURN_RIGHT,
    MOTION_CMD_LEFT_FRONT,
    MOTION_CMD_RIGHT_FRONT,
    MOTION_CMD_LEFT_BACK,
    MOTION_CMD_RIGHT_BACK,
    MOTION_CMD_SINGLE_LEG_LF,
    MOTION_CMD_SINGLE_LEG_RF,
    MOTION_CMD_SINGLE_LEG_LB,
    MOTION_CMD_SINGLE_LEG_RB,
    MOTION_CMD_LEG_GROUP,
    MOTION_CMD_UNKNOWN
} MotionCommand;

static MotionCommand g_currentMotion = MOTION_CMD_NONE;
static int g_motionRepeatCount = -1; // -1: infinite, >0: finite times
static int g_currentLegMask = 0;
static double g_currentStepLengthM = EDOG_12DOF_COMMAND_STEP_LENGTH_M;
static double g_currentStepHeightM = EDOG_12DOF_COMMAND_STEP_HEIGHT_M;
static double g_currentHipAdductionDeg = EDOG_12DOF_STAND_HIP_DELTA_DEG;
static double g_currentFrontHipAdductionDeg = EDOG_12DOF_FRONT_HIP_ADDUCTION_DEFAULT_DEG;
static double g_currentRearHipAdductionDeg = EDOG_12DOF_REAR_HIP_ADDUCTION_DEFAULT_DEG;
static double g_currentFrontBodyHeightDeltaMm = 0.0;
static double g_currentRearBodyHeightDeltaMm = 0.0;
static double g_currentThighLengthM = SPOTMICRO_THIGH_LENGTH_MM / 1000.0;
static double g_currentCalfLengthM = SPOTMICRO_CALF_LENGTH_MM / 1000.0;
static volatile int g_stopReturnPending = 0;
static int g_idleBalancePauseFrames = 0;
static volatile int g_manualPoseHoldActive = 0;
static UINT32 g_motionStateMux;
static UINT32 g_commandQueueId;
static volatile int g_motionStateMuxReady = 0;
static volatile int g_commandQueueReady = 0;

static int EnsureCommandRuntime(void);
static bool ApplyQueuedCommand(const IotControlCommand *command);
static void ExecuteMotionWithSteps(MotionCommand cmd, int repeatCount,
                                   double stepLengthM, double stepHeightM);
static void ExecuteMotionWithRuntimeParams(MotionCommand cmd, int repeatCount,
                                           double stepLengthM, double stepHeightM,
                                           double frontHipAdductionDeg,
                                           double rearHipAdductionDeg,
                                           double frontBodyHeightDeltaMm,
                                           double rearBodyHeightDeltaMm,
                                           double thighLengthM,
                                           double calfLengthM);
static bool IotControl_HandleCommandStringWithRuntimeParams(const char *command,
                                                            double stepLengthM,
                                                            double stepHeightM,
                                                            double frontHipAdductionDeg,
                                                            double rearHipAdductionDeg,
                                                            double frontBodyHeightDeltaMm,
                                                            double rearBodyHeightDeltaMm,
                                                            double thighLengthM,
                                                            double calfLengthM);
static bool IotControl_RunLegGaitWithRuntimeParams(int legMask, int repeatCount,
                                                   double stepLengthM, double stepHeightM,
                                                   double frontHipAdductionDeg,
                                                   double rearHipAdductionDeg,
                                                   double frontBodyHeightDeltaMm,
                                                   double rearBodyHeightDeltaMm,
                                                   double thighLengthM,
                                                   double calfLengthM);
static void ApplyRuntimeGaitGeometryForFrontRearNoLock(double frontHipAdductionDeg,
                                                       double rearHipAdductionDeg,
                                                       double thighLengthM,
                                                       double calfLengthM);

static int IsServoChannelValid(int channel)
{
    return channel >= 0 && channel < EDOG_SERVO_CHANNEL_COUNT;
}

static int EnsureCommandRuntime(void)
{
    UINT32 ret;

    if (!g_motionStateMuxReady) {
        ret = LOS_MuxCreate(&g_motionStateMux);
        if (ret != LOS_OK) {
            printf("[Motion] create state mutex failed: %u\n", ret);
            return -1;
        }
        g_motionStateMuxReady = 1;
    }

    if (!g_commandQueueReady) {
        ret = LOS_QueueCreate("edog_cmd_q",
                              EDOG_MOTION_COMMAND_QUEUE_LENGTH,
                              &g_commandQueueId,
                              0,
                              sizeof(IotControlCommand));
        if (ret != LOS_OK) {
            printf("[Motion] create command queue failed: %u\n", ret);
            return -1;
        }
        g_commandQueueReady = 1;
    }

    return 0;
}

static void LockMotionState(void)
{
    if (EnsureCommandRuntime() == 0) {
        (void)LOS_MuxPend(g_motionStateMux, LOS_WAIT_FOREVER);
    }
}

static void UnlockMotionState(void)
{
    if (g_motionStateMuxReady) {
        (void)LOS_MuxPost(g_motionStateMux);
    }
}

static double NormalizeStepMinimumM(double stepM)
{
    if (stepM < 0.0) {
        return 0.0;
    }
    return stepM;
}

static double NormalizeHipAdductionDeg(double hipAdductionDeg)
{
    if (hipAdductionDeg < -45.0) {
        return -45.0;
    }
    if (hipAdductionDeg > 45.0) {
        return 45.0;
    }
    return hipAdductionDeg;
}

static double NormalizeLegLengthM(double lengthM, double defaultM)
{
    if (lengthM <= 0.0) {
        return defaultM;
    }
    if (lengthM < EDOG_12DOF_LINK_LENGTH_MIN_M) {
        return EDOG_12DOF_LINK_LENGTH_MIN_M;
    }
    if (lengthM > EDOG_12DOF_LINK_LENGTH_MAX_M) {
        return EDOG_12DOF_LINK_LENGTH_MAX_M;
    }
    return lengthM;
}

static double NormalizeBodyHeightDeltaMm(double deltaMm)
{
    if (deltaMm < EDOG_12DOF_BODY_HEIGHT_DELTA_MIN_MM) {
        return EDOG_12DOF_BODY_HEIGHT_DELTA_MIN_MM;
    }
    if (deltaMm > EDOG_12DOF_BODY_HEIGHT_DELTA_MAX_MM) {
        return EDOG_12DOF_BODY_HEIGHT_DELTA_MAX_MM;
    }
    return deltaMm;
}

static void GetRuntimeBodyHeightDefaults(double *frontBodyHeightDeltaMm,
                                         double *rearBodyHeightDeltaMm)
{
    double frontDeltaMm = 0.0;
    double rearDeltaMm = 0.0;

    Edog12Dof_GetRuntimeFootZDeltas(&frontDeltaMm, &rearDeltaMm);
    if (frontBodyHeightDeltaMm != NULL) {
        *frontBodyHeightDeltaMm = NormalizeBodyHeightDeltaMm(frontDeltaMm);
    }
    if (rearBodyHeightDeltaMm != NULL) {
        *rearBodyHeightDeltaMm = NormalizeBodyHeightDeltaMm(rearDeltaMm);
    }
}

static void GetRuntimeGaitGeometryDefaults(double *frontHipAdductionDeg,
                                           double *rearHipAdductionDeg,
                                           double *thighLengthM,
                                           double *calfLengthM)
{
    double frontHipDeg = EDOG_12DOF_FRONT_HIP_ADDUCTION_DEFAULT_DEG;
    double rearHipDeg = EDOG_12DOF_REAR_HIP_ADDUCTION_DEFAULT_DEG;
    double thighMm = SPOTMICRO_THIGH_LENGTH_MM;
    double calfMm = SPOTMICRO_CALF_LENGTH_MM;

    Edog12Dof_GetRuntimeGeometryForFrontRear(&frontHipDeg, &rearHipDeg, &thighMm, &calfMm);
    if (frontHipAdductionDeg != NULL) {
        *frontHipAdductionDeg = NormalizeHipAdductionDeg(frontHipDeg);
    }
    if (rearHipAdductionDeg != NULL) {
        *rearHipAdductionDeg = NormalizeHipAdductionDeg(rearHipDeg);
    }
    if (thighLengthM != NULL) {
        *thighLengthM = NormalizeLegLengthM(thighMm / 1000.0,
                                            SPOTMICRO_THIGH_LENGTH_MM / 1000.0);
    }
    if (calfLengthM != NULL) {
        *calfLengthM = NormalizeLegLengthM(calfMm / 1000.0,
                                           SPOTMICRO_CALF_LENGTH_MM / 1000.0);
    }
}

static void SetMotionLegLengthState(double thighLengthM, double calfLengthM)
{
    g_currentThighLengthM = NormalizeLegLengthM(thighLengthM,
                                                EDOG_12DOF_THIGH_LENGTH_DEFAULT_M);
    g_currentCalfLengthM = NormalizeLegLengthM(calfLengthM,
                                               EDOG_12DOF_CALF_LENGTH_DEFAULT_M);
}

static void SetMotionStepState(double stepLengthM, double stepHeightM)
{
    g_currentStepLengthM = NormalizeStepMinimumM(stepLengthM);
    g_currentStepHeightM = NormalizeStepMinimumM(stepHeightM);
}

static void ApplyRuntimeBodyHeightNoLock(double frontBodyHeightDeltaMm,
                                         double rearBodyHeightDeltaMm)
{
    g_currentFrontBodyHeightDeltaMm = NormalizeBodyHeightDeltaMm(frontBodyHeightDeltaMm);
    g_currentRearBodyHeightDeltaMm = NormalizeBodyHeightDeltaMm(rearBodyHeightDeltaMm);
    Edog12Dof_SetRuntimeFootZDeltas(g_currentFrontBodyHeightDeltaMm,
                                    g_currentRearBodyHeightDeltaMm);
}

static void ApplyRuntimeGaitGeometryNoLock(double hipAdductionDeg,
                                           double thighLengthM,
                                           double calfLengthM)
{
    ApplyRuntimeGaitGeometryForFrontRearNoLock(hipAdductionDeg, hipAdductionDeg,
                                               thighLengthM, calfLengthM);
}

static void ApplyRuntimeGaitGeometryForFrontRearNoLock(double frontHipAdductionDeg,
                                                       double rearHipAdductionDeg,
                                                       double thighLengthM,
                                                       double calfLengthM)
{
    g_currentFrontHipAdductionDeg = NormalizeHipAdductionDeg(frontHipAdductionDeg);
    g_currentRearHipAdductionDeg = NormalizeHipAdductionDeg(rearHipAdductionDeg);
    g_currentHipAdductionDeg = g_currentFrontHipAdductionDeg;
    SetMotionLegLengthState(thighLengthM, calfLengthM);
    thighLengthM = g_currentThighLengthM;
    calfLengthM = g_currentCalfLengthM;
    Edog12Dof_SetLinkLengthsM(thighLengthM, calfLengthM);
    setRuntimeGaitGeometryForFrontRear(g_currentFrontHipAdductionDeg,
                                       g_currentRearHipAdductionDeg,
                                       thighLengthM,
                                       calfLengthM);
}

static void SetMotionState(MotionCommand cmd, int repeatCount, int stopReturnPending)
{
    LockMotionState();
    g_currentMotion = cmd;
    g_motionRepeatCount = repeatCount;
    g_stopReturnPending = stopReturnPending;
    if (cmd != MOTION_CMD_LEG_GROUP) {
        g_currentLegMask = 0;
    }
    UnlockMotionState();
}

static void EnableManualPoseHold(void)
{
    g_manualPoseHoldActive = 1;
}

static void ClearManualPoseHold(void)
{
    g_manualPoseHoldActive = 0;
}

static void PrepareManualServoControl(void)
{
    ClearManualPoseHold();
    SetMotionState(MOTION_CMD_NONE, -1, 0);
    resetStopFlag();
    g_idleBalancePauseFrames = EDOG_12DOF_IDLE_BALANCE_MANUAL_PAUSE_FRAMES;
}

static const char *MotionCommandName(MotionCommand cmd)
{
    switch (cmd) {
        case MOTION_CMD_TROT:
            return "trot";
        case MOTION_CMD_TROT_IN_PLACE:
            return "trot_in_place";
        case MOTION_CMD_TROT_BACK:
            return "trot_back";
        case MOTION_CMD_TURN_LEFT:
            return "turn_left";
        case MOTION_CMD_TURN_RIGHT:
            return "turn_right";
        case MOTION_CMD_LEFT_FRONT:
            return "left_front";
        case MOTION_CMD_RIGHT_FRONT:
            return "right_front";
        case MOTION_CMD_LEFT_BACK:
            return "left_back";
        case MOTION_CMD_RIGHT_BACK:
            return "right_back";
        case MOTION_CMD_SINGLE_LEG_LF:
            return "single_leg_lf";
        case MOTION_CMD_SINGLE_LEG_RF:
            return "single_leg_rf";
        case MOTION_CMD_SINGLE_LEG_LB:
            return "single_leg_lb";
        case MOTION_CMD_SINGLE_LEG_RB:
            return "single_leg_rb";
        case MOTION_CMD_LEG_GROUP:
            return "leg_group";
        default:
            return "stop";
    }
}

static MotionCommand SingleLegCommandFromIndex(int legIndex)
{
    switch (legIndex) {
        case 0:
            return MOTION_CMD_SINGLE_LEG_LF;
        case 1:
            return MOTION_CMD_SINGLE_LEG_RF;
        case 2:
            return MOTION_CMD_SINGLE_LEG_LB;
        case 3:
            return MOTION_CMD_SINGLE_LEG_RB;
        default:
            return MOTION_CMD_UNKNOWN;
    }
}

static MotionCommand MotionCommandFromString(const char *content)
{
    if (content == NULL) {
        return MOTION_CMD_NONE;
    }
    if (strcmp(content, "trot") == 0) {
        return MOTION_CMD_TROT;
    }
    if (strcmp(content, "trot_in_place") == 0 || strcmp(content, "trot_place") == 0 ||
        strcmp(content, "step_in_place") == 0 || strcmp(content, "in_place") == 0) {
        return MOTION_CMD_TROT_IN_PLACE;
    }
    if (strcmp(content, "1") == 0) {
        return MOTION_CMD_TROT;
    }
    if (strcmp(content, "trot_back") == 0) {
        return MOTION_CMD_TROT_BACK;
    }
    if (strcmp(content, "turn_left") == 0) {
        return MOTION_CMD_TURN_LEFT;
    }
    if (strcmp(content, "turn_right") == 0) {
        return MOTION_CMD_TURN_RIGHT;
    }
    if (strcmp(content, "left_front") == 0 || strcmp(content, "front_left") == 0 ||
        strcmp(content, "left_forward") == 0 || strcmp(content, "forward_left") == 0 ||
        strcmp(content, "lf") == 0 || strcmp(content, "左前") == 0) {
        return MOTION_CMD_LEFT_FRONT;
    }
    if (strcmp(content, "right_front") == 0 || strcmp(content, "front_right") == 0 ||
        strcmp(content, "right_forward") == 0 || strcmp(content, "forward_right") == 0 ||
        strcmp(content, "rf") == 0 || strcmp(content, "右前") == 0) {
        return MOTION_CMD_RIGHT_FRONT;
    }
    if (strcmp(content, "left_back") == 0 || strcmp(content, "back_left") == 0 ||
        strcmp(content, "left_backward") == 0 || strcmp(content, "backward_left") == 0 ||
        strcmp(content, "lb") == 0 || strcmp(content, "左后") == 0) {
        return MOTION_CMD_LEFT_BACK;
    }
    if (strcmp(content, "right_back") == 0 || strcmp(content, "back_right") == 0 ||
        strcmp(content, "right_backward") == 0 || strcmp(content, "backward_right") == 0 ||
        strcmp(content, "rb") == 0 || strcmp(content, "右后") == 0) {
        return MOTION_CMD_RIGHT_BACK;
    }
    if (strcmp(content, "single_leg_lf") == 0 || strcmp(content, "leg_lf") == 0) {
        return MOTION_CMD_SINGLE_LEG_LF;
    }
    if (strcmp(content, "single_leg_rf") == 0 || strcmp(content, "leg_rf") == 0) {
        return MOTION_CMD_SINGLE_LEG_RF;
    }
    if (strcmp(content, "single_leg_lb") == 0 || strcmp(content, "leg_lb") == 0) {
        return MOTION_CMD_SINGLE_LEG_LB;
    }
    if (strcmp(content, "single_leg_rb") == 0 || strcmp(content, "leg_rb") == 0) {
        return MOTION_CMD_SINGLE_LEG_RB;
    }
    if (strcmp(content, "leg_group") == 0 || strcmp(content, "leg_gait") == 0) {
        return MOTION_CMD_LEG_GROUP;
    }
    if (strcmp(content, "stop") == 0) {
        return MOTION_CMD_NONE;
    }
    return MOTION_CMD_UNKNOWN;
}

static void CopyCommandBase(const char *command, char *buffer, size_t bufferLen)
{
    const char *suffix = "_coze";
    size_t cmdLen;
    size_t suffixLen;
    size_t copyLen;

    if (buffer == NULL || bufferLen == 0) {
        return;
    }
    buffer[0] = '\0';
    if (command == NULL) {
        return;
    }

    cmdLen = strlen(command);
    suffixLen = strlen(suffix);
    copyLen = cmdLen;
    if (cmdLen > suffixLen && strcmp(command + cmdLen - suffixLen, suffix) == 0) {
        copyLen = cmdLen - suffixLen;
    }
    if (copyLen >= bufferLen) {
        copyLen = bufferLen - 1;
    }
    memcpy(buffer, command, copyLen);
    buffer[copyLen] = '\0';
}

bool IotControl_IsMotionCommandString(const char *command)
{
    char baseCommand[EDOG_MOTION_COMMAND_TEXT_LENGTH] = {0};
    MotionCommand cmd;

    CopyCommandBase(command, baseCommand, sizeof(baseCommand));
    cmd = MotionCommandFromString(baseCommand);

    return cmd != MOTION_CMD_UNKNOWN;
}

bool IotControl_EnqueueCommand(const IotControlCommand *command)
{
    IotControlCommand copy;
    UINT32 ret;

    if (command == NULL || EnsureCommandRuntime() != 0) {
        return false;
    }

    copy = *command;
    copy.text[sizeof(copy.text) - 1] = '\0';

    if (copy.type == IOT_CONTROL_COMMAND_TEXT) {
        char baseCommand[EDOG_MOTION_COMMAND_TEXT_LENGTH] = {0};
        CopyCommandBase(copy.text, baseCommand, sizeof(baseCommand));
        if (MotionCommandFromString(baseCommand) == MOTION_CMD_NONE) {
            stopCurrentMotion();
        }
    }

    ret = LOS_QueueWriteCopy(g_commandQueueId, &copy, sizeof(copy), 0);
    if (ret != LOS_OK) {
        printf("[Motion] command queue full or unavailable: %u\n", ret);
        return false;
    }

    printf("[Motion] command queued type=%d text=%s\n", copy.type, copy.text);
    return true;
}

static int MotionStepChanged(double currentLengthM, double currentHeightM,
                             double newLengthM, double newHeightM)
{
    double lengthDiff = currentLengthM - newLengthM;
    double heightDiff = currentHeightM - newHeightM;

    if (lengthDiff < 0.0) {
        lengthDiff = -lengthDiff;
    }
    if (heightDiff < 0.0) {
        heightDiff = -heightDiff;
    }
    return lengthDiff > 0.000001 || heightDiff > 0.000001;
}

static int MotionGeometryChanged(double currentFrontHipDeg, double currentRearHipDeg,
                                 double currentThighM, double currentCalfM,
                                 double newFrontHipDeg, double newRearHipDeg,
                                 double newThighM, double newCalfM)
{
    double frontHipDiff = currentFrontHipDeg - newFrontHipDeg;
    double rearHipDiff = currentRearHipDeg - newRearHipDeg;
    double thighDiff = currentThighM - newThighM;
    double calfDiff = currentCalfM - newCalfM;

    if (frontHipDiff < 0.0) {
        frontHipDiff = -frontHipDiff;
    }
    if (rearHipDiff < 0.0) {
        rearHipDiff = -rearHipDiff;
    }
    if (thighDiff < 0.0) {
        thighDiff = -thighDiff;
    }
    if (calfDiff < 0.0) {
        calfDiff = -calfDiff;
    }
    return frontHipDiff > 0.001 || rearHipDiff > 0.001 ||
        thighDiff > 0.000001 || calfDiff > 0.000001;
}

static int MotionBodyHeightChanged(double currentFrontBodyHeightDeltaMm,
                                   double currentRearBodyHeightDeltaMm,
                                   double newFrontBodyHeightDeltaMm,
                                   double newRearBodyHeightDeltaMm)
{
    double frontDiff = currentFrontBodyHeightDeltaMm - newFrontBodyHeightDeltaMm;
    double rearDiff = currentRearBodyHeightDeltaMm - newRearBodyHeightDeltaMm;

    if (frontDiff < 0.0) {
        frontDiff = -frontDiff;
    }
    if (rearDiff < 0.0) {
        rearDiff = -rearDiff;
    }
    return frontDiff > 0.001 || rearDiff > 0.001;
}

static void ExecuteMotionWithSteps(MotionCommand cmd, int repeatCount,
                                   double stepLengthM, double stepHeightM)
{
    double frontHipAdductionDeg = EDOG_12DOF_FRONT_HIP_ADDUCTION_DEFAULT_DEG;
    double rearHipAdductionDeg = EDOG_12DOF_REAR_HIP_ADDUCTION_DEFAULT_DEG;
    double frontBodyHeightDeltaMm = 0.0;
    double rearBodyHeightDeltaMm = 0.0;
    double thighLengthM = SPOTMICRO_THIGH_LENGTH_MM / 1000.0;
    double calfLengthM = SPOTMICRO_CALF_LENGTH_MM / 1000.0;

    GetRuntimeGaitGeometryDefaults(&frontHipAdductionDeg, &rearHipAdductionDeg,
                                   &thighLengthM, &calfLengthM);
    GetRuntimeBodyHeightDefaults(&frontBodyHeightDeltaMm, &rearBodyHeightDeltaMm);
    ExecuteMotionWithRuntimeParams(cmd, repeatCount,
                                   stepLengthM, stepHeightM,
                                   frontHipAdductionDeg,
                                   rearHipAdductionDeg,
                                   frontBodyHeightDeltaMm,
                                   rearBodyHeightDeltaMm,
                                   thighLengthM,
                                   calfLengthM);
}

static void ExecuteMotionWithRuntimeParams(MotionCommand cmd, int repeatCount,
                                           double stepLengthM, double stepHeightM,
                                           double frontHipAdductionDeg,
                                           double rearHipAdductionDeg,
                                           double frontBodyHeightDeltaMm,
                                           double rearBodyHeightDeltaMm,
                                           double thighLengthM,
                                           double calfLengthM)
{
    MotionCommand previousMotion;
    int alreadyRunning;
    double normalizedStepLengthM = NormalizeStepMinimumM(stepLengthM);
    double normalizedStepHeightM = NormalizeStepMinimumM(stepHeightM);
    double normalizedFrontHipAdductionDeg = NormalizeHipAdductionDeg(frontHipAdductionDeg);
    double normalizedRearHipAdductionDeg = NormalizeHipAdductionDeg(rearHipAdductionDeg);
    double normalizedFrontBodyHeightDeltaMm = NormalizeBodyHeightDeltaMm(frontBodyHeightDeltaMm);
    double normalizedRearBodyHeightDeltaMm = NormalizeBodyHeightDeltaMm(rearBodyHeightDeltaMm);
    double normalizedThighLengthM = NormalizeLegLengthM(thighLengthM,
                                                        SPOTMICRO_THIGH_LENGTH_MM / 1000.0);
    double normalizedCalfLengthM = NormalizeLegLengthM(calfLengthM,
                                                       SPOTMICRO_CALF_LENGTH_MM / 1000.0);

    if (cmd == MOTION_CMD_NONE) {
        printf("[Motion] stop command received\n");
        ClearManualPoseHold();
        SetMotionState(MOTION_CMD_NONE, -1, 1);
        stopCurrentMotion();
        return;
    }

    LockMotionState();
    previousMotion = g_currentMotion;
    alreadyRunning = repeatCount == -1 && g_currentMotion == cmd &&
        !MotionStepChanged(g_currentStepLengthM, g_currentStepHeightM,
                           normalizedStepLengthM, normalizedStepHeightM) &&
        !MotionGeometryChanged(g_currentFrontHipAdductionDeg, g_currentRearHipAdductionDeg,
                               g_currentThighLengthM, g_currentCalfLengthM,
                               normalizedFrontHipAdductionDeg, normalizedRearHipAdductionDeg,
                               normalizedThighLengthM, normalizedCalfLengthM) &&
        !MotionBodyHeightChanged(g_currentFrontBodyHeightDeltaMm,
                                 g_currentRearBodyHeightDeltaMm,
                                 normalizedFrontBodyHeightDeltaMm,
                                 normalizedRearBodyHeightDeltaMm);
    if (!alreadyRunning) {
        ClearManualPoseHold();
        g_stopReturnPending = 0;
        resetStopFlag();
        g_currentMotion = cmd;
        g_motionRepeatCount = repeatCount;
        SetMotionStepState(normalizedStepLengthM, normalizedStepHeightM);
        ApplyRuntimeGaitGeometryForFrontRearNoLock(normalizedFrontHipAdductionDeg,
                                                   normalizedRearHipAdductionDeg,
                                                   normalizedThighLengthM,
                                                   normalizedCalfLengthM);
        ApplyRuntimeBodyHeightNoLock(normalizedFrontBodyHeightDeltaMm,
                                     normalizedRearBodyHeightDeltaMm);
        g_idleBalancePauseFrames = 0;
    }
    UnlockMotionState();

    if (alreadyRunning) {
        return;
    }

    printf("[Motion] switch action: %s -> %s (repeat: %d step=%.3fm lift=%.3fm frontHip=%.1f rearHip=%.1f frontZ=%.1fmm rearZ=%.1fmm thigh=%.3fm calf=%.3fm)\n",
           MotionCommandName(previousMotion), MotionCommandName(cmd), repeatCount,
           normalizedStepLengthM, normalizedStepHeightM,
           normalizedFrontHipAdductionDeg, normalizedRearHipAdductionDeg,
           normalizedFrontBodyHeightDeltaMm, normalizedRearBodyHeightDeltaMm,
           normalizedThighLengthM, normalizedCalfLengthM);
}

static void ExecuteMotion(MotionCommand cmd, int repeatCount)
{
    ExecuteMotionWithSteps(cmd, repeatCount,
                           EDOG_12DOF_COMMAND_STEP_LENGTH_M,
                           EDOG_12DOF_COMMAND_STEP_HEIGHT_M);
}

static void ExecuteMotionLegacyUnused(MotionCommand cmd, int repeatCount)
{
    if (cmd == MOTION_CMD_NONE) {
        printf("[Motion] 收到停止指令，立即停止当前动作\n");
        g_currentMotion = MOTION_CMD_NONE;
        g_motionRepeatCount = -1;
        g_stopReturnPending = 1;
        stopCurrentMotion(); // 设置停止标志，中断当前循环
        return;
    }

    // If it's a Coze command (repeatCount > 0), we always update to ensure it runs the requested times
    // If it's a normal command (repeatCount == -1), we check if it's already running
    if (repeatCount == -1 && g_currentMotion == cmd) {
        printf("[Motion] 动作 %s 已在执行，保持状态\n", MotionCommandName(cmd));
        return;
    }

    printf("[Motion] 切换动作：%s -> %s (次数: %d)\n",
           MotionCommandName(g_currentMotion), MotionCommandName(cmd), repeatCount);
    
    // 切换动作时，不强制中断当前周期，而是更新状态让下一个周期执行新动作
    // 这样可以实现"丝滑切换"，避免动作生硬中断
    // 除非之前的状态是停止（stopFlag可能为1），所以需要重置标志
    g_stopReturnPending = 0;
    resetStopFlag();
    g_currentMotion = cmd;
    g_motionRepeatCount = repeatCount;
}

static int ReadQueuedCommand(IotControlCommand *command)
{
    UINT32 size = sizeof(*command);

    if (command == NULL || EnsureCommandRuntime() != 0) {
        return 0;
    }

    return LOS_QueueReadCopy(g_commandQueueId, command, &size, 0) == LOS_OK;
}

static MotionCommand GetCurrentMotionSnapshot(void)
{
    MotionCommand cmd;

    LockMotionState();
    cmd = g_currentMotion;
    UnlockMotionState();
    return cmd;
}

static int TakeStopReturnPending(void)
{
    int pending;

    LockMotionState();
    pending = g_stopReturnPending;
    if (pending) {
        g_stopReturnPending = 0;
    }
    UnlockMotionState();
    return pending;
}

static int GetCurrentLegMaskSnapshot(void)
{
    int legMask;

    LockMotionState();
    legMask = g_currentLegMask;
    UnlockMotionState();
    return legMask;
}

static void GetMotionStepSnapshot(double *stepLengthM, double *stepHeightM)
{
    LockMotionState();
    if (stepLengthM != NULL) {
        *stepLengthM = g_currentStepLengthM;
    }
    if (stepHeightM != NULL) {
        *stepHeightM = g_currentStepHeightM;
    }
    UnlockMotionState();
}

static void GetMotionLegLengthSnapshot(double *thighLengthM, double *calfLengthM)
{
    LockMotionState();
    if (thighLengthM != NULL) {
        *thighLengthM = g_currentThighLengthM;
    }
    if (calfLengthM != NULL) {
        *calfLengthM = g_currentCalfLengthM;
    }
    UnlockMotionState();
}

static void MarkMotionStopped(void)
{
    SetMotionState(MOTION_CMD_NONE, -1, g_stopReturnPending);
}

static int FinishOneMotionCycle(void)
{
    int shouldStand = 0;

    LockMotionState();
    if (g_motionRepeatCount > 0) {
        g_motionRepeatCount--;
        if (g_motionRepeatCount == 0) {
            g_currentMotion = MOTION_CMD_NONE;
            g_currentLegMask = 0;
            g_motionRepeatCount = -1;
            shouldStand = 1;
        }
    }
    UnlockMotionState();

    return shouldStand;
}

static void RunIdleStandBalanceFrame(void)
{
    if (g_manualPoseHoldActive) {
        usleep(EDOG_12DOF_GAIT_FRAME_PERIOD_US);
        return;
    }
    if (g_idleBalancePauseFrames > 0) {
        g_idleBalancePauseFrames--;
        usleep(EDOG_12DOF_GAIT_FRAME_PERIOD_US);
        return;
    }
    (void)balance_stand_frame();
}

static bool ApplyQueuedCommand(const IotControlCommand *command)
{
    if (command == NULL) {
        return false;
    }

    switch (command->type) {
        case IOT_CONTROL_COMMAND_TEXT:
            return IotControl_HandleCommandStringWithRuntimeParams(command->text,
                                                                   command->stepLengthM,
                                                                   command->stepHeightM,
                                                                   command->frontHipAdductionDeg,
                                                                   command->rearHipAdductionDeg,
                                                                   command->frontBodyHeightDeltaMm,
                                                                   command->rearBodyHeightDeltaMm,
                                                                   command->thighLengthM,
                                                                   command->calfLengthM);
        case IOT_CONTROL_COMMAND_NUMBER:
            return IotControl_HandleCommandNumber(command->value);
        case IOT_CONTROL_COMMAND_SPEED_LEVEL:
            return IotControl_SetMotionSpeedLevel(command->value);
        case IOT_CONTROL_COMMAND_SERVO_SET:
            return IotControl_SetSingleServoAngle(command->channel, command->angle);
        case IOT_CONTROL_COMMAND_SERVO_BATCH_SET:
            return IotControl_SetServoBatchAngles(command->count, command->channels, command->angles);
        case IOT_CONTROL_COMMAND_SERVO_TRIM_SET:
            return IotControl_SetServoCenterTrim(command->channel, command->trim, command->applyNow);
        case IOT_CONTROL_COMMAND_SERVO_TRIM_BATCH_SET:
            return IotControl_SetAllServoCenterTrims(command->angles, command->count, command->applyNow);
        case IOT_CONTROL_COMMAND_SERVO_CALIBRATION_REPORT:
            return IotControl_ReportServoCalibration();
        case IOT_CONTROL_COMMAND_SERVO_STATUS_REPORT:
            return IotControl_ReportServoStatus("status_read");
        case IOT_CONTROL_COMMAND_STRAIGHTEN_LEGS:
            return IotControl_StraightenLegs();
        case IOT_CONTROL_COMMAND_LEG_GAIT:
            return IotControl_RunLegGaitWithRuntimeParams(command->legMask, command->repeatCount,
                                                          command->stepLengthM,
                                                          command->stepHeightM,
                                                          command->frontHipAdductionDeg,
                                                          command->rearHipAdductionDeg,
                                                          command->frontBodyHeightDeltaMm,
                                                          command->rearBodyHeightDeltaMm,
                                                          command->thighLengthM,
                                                          command->calfLengthM);
        case IOT_CONTROL_COMMAND_RUNTIME_TUNING_SET:
            return IotControl_SetRuntimeTuning(command->frontHipAdductionDeg,
                                               command->rearHipAdductionDeg,
                                               command->frontBodyHeightDeltaMm,
                                               command->rearBodyHeightDeltaMm,
                                               command->imuBalanceStrengthPercent,
                                               command->thighLengthM,
                                               command->calfLengthM,
                                               command->applyNow);
        default:
            return false;
    }
}

void IotControl_MotionTask(void)
{
    IotControlCommand queuedCommand;

    if (EnsureCommandRuntime() != 0) {
        printf("[MotionTask] command runtime unavailable\n");
    }
    {
        double frontHipAdductionDeg = EDOG_12DOF_FRONT_HIP_ADDUCTION_DEFAULT_DEG;
        double rearHipAdductionDeg = EDOG_12DOF_REAR_HIP_ADDUCTION_DEFAULT_DEG;
        double frontBodyHeightDeltaMm = 0.0;
        double rearBodyHeightDeltaMm = 0.0;
        double thighLengthM = SPOTMICRO_THIGH_LENGTH_MM / 1000.0;
        double calfLengthM = SPOTMICRO_CALF_LENGTH_MM / 1000.0;

        GetRuntimeGaitGeometryDefaults(&frontHipAdductionDeg, &rearHipAdductionDeg,
                                       &thighLengthM, &calfLengthM);
        GetRuntimeBodyHeightDefaults(&frontBodyHeightDeltaMm, &rearBodyHeightDeltaMm);
        LockMotionState();
        ApplyRuntimeGaitGeometryForFrontRearNoLock(frontHipAdductionDeg,
                                                   rearHipAdductionDeg,
                                                   thighLengthM,
                                                   calfLengthM);
        ApplyRuntimeBodyHeightNoLock(frontBodyHeightDeltaMm, rearBodyHeightDeltaMm);
        UnlockMotionState();
    }

    while (1) {
        MotionCommand currentMotion;
        double stepLengthM = EDOG_12DOF_COMMAND_STEP_LENGTH_M;
        double stepHeightM = EDOG_12DOF_COMMAND_STEP_HEIGHT_M;
        int result = 1;

        while (ReadQueuedCommand(&queuedCommand)) {
            (void)ApplyQueuedCommand(&queuedCommand);
        }

        currentMotion = GetCurrentMotionSnapshot();
        GetMotionStepSnapshot(&stepLengthM, &stepHeightM);
        if (currentMotion == MOTION_CMD_NONE) {
            if (TakeStopReturnPending()) {
                int stopDone = smooth_stop_to_stand(stepLengthM, stepHeightM);
                if (stopDone) {
                    resetStopFlag();
                }
                g_idleBalancePauseFrames = 0;
            }
            RunIdleStandBalanceFrame();
            continue;
        }

        switch (currentMotion) {
            case MOTION_CMD_TROT:
                result = trot_cycle(stepLengthM, stepHeightM);
                break;
            case MOTION_CMD_TROT_IN_PLACE:
                result = trot_in_place_cycle(stepHeightM);
                break;
            case MOTION_CMD_TROT_BACK:
                result = trot_back_cycle(stepLengthM, stepHeightM);
                break;
            case MOTION_CMD_TURN_LEFT:
                result = diversion_left_cycle(stepLengthM, stepHeightM);
                break;
            case MOTION_CMD_TURN_RIGHT:
                result = diversion_right_cycle(stepLengthM, stepHeightM);
                break;
            case MOTION_CMD_LEFT_FRONT:
                result = trot_left_front_cycle(stepLengthM, stepHeightM);
                break;
            case MOTION_CMD_RIGHT_FRONT:
                result = trot_right_front_cycle(stepLengthM, stepHeightM);
                break;
            case MOTION_CMD_LEFT_BACK:
                result = trot_left_back_cycle(stepLengthM, stepHeightM);
                break;
            case MOTION_CMD_RIGHT_BACK:
                result = trot_right_back_cycle(stepLengthM, stepHeightM);
                break;
            case MOTION_CMD_SINGLE_LEG_LF:
                result = single_leg_gait_cycle(0, stepLengthM, stepHeightM);
                break;
            case MOTION_CMD_SINGLE_LEG_RF:
                result = single_leg_gait_cycle(1, stepLengthM, stepHeightM);
                break;
            case MOTION_CMD_SINGLE_LEG_LB:
                result = single_leg_gait_cycle(2, stepLengthM, stepHeightM);
                break;
            case MOTION_CMD_SINGLE_LEG_RB:
                result = single_leg_gait_cycle(3, stepLengthM, stepHeightM);
                break;
            case MOTION_CMD_LEG_GROUP:
                result = leg_group_gait_cycle(GetCurrentLegMaskSnapshot(),
                                              stepLengthM,
                                              stepHeightM);
                break;
            default:
                break;
        }

        if (result == 0 && isStopFlag()) {
            MarkMotionStopped();
            if (TakeStopReturnPending()) {
                int stopDone = smooth_stop_to_stand(stepLengthM, stepHeightM);
                if (stopDone) {
                    resetStopFlag();
                }
            } else {
                resetStopFlag();
            }
        } else if (result > 0 && FinishOneMotionCycle()) {
            init_dog(stepLengthM, stepHeightM);
            g_idleBalancePauseFrames = 0;
        }

        usleep(1000);
    }
}

void IotControl_MotionTaskLegacyUnused(void)
{
    while (1) {
        if (g_currentMotion == MOTION_CMD_NONE) {
            if (g_stopReturnPending) {
                int stopDone = smooth_stop_to_stand(EDOG_12DOF_COMMAND_STEP_LENGTH_M,
                                                    EDOG_12DOF_COMMAND_STEP_HEIGHT_M);
                g_stopReturnPending = 0;
                if (stopDone) {
                    resetStopFlag();
                }
            }
            usleep(100000); // 100ms 轮询
            continue;
        }

        // 根据当前状态执行一个周期
        // 注意：这些函数内部会检查 stopFlag，如果被置位会立即返回0
        int result = 1;
        switch (g_currentMotion) {
            case MOTION_CMD_TROT:
                result = trot_cycle(EDOG_12DOF_COMMAND_STEP_LENGTH_M, EDOG_12DOF_COMMAND_STEP_HEIGHT_M);
                break;
            case MOTION_CMD_TROT_BACK:
                result = trot_back_cycle(EDOG_12DOF_COMMAND_STEP_LENGTH_M, EDOG_12DOF_COMMAND_STEP_HEIGHT_M);
                break;
            case MOTION_CMD_TURN_LEFT:
                result = diversion_left_cycle(EDOG_12DOF_COMMAND_STEP_LENGTH_M, EDOG_12DOF_COMMAND_STEP_HEIGHT_M);
                break;
            case MOTION_CMD_TURN_RIGHT:
                result = diversion_right_cycle(EDOG_12DOF_COMMAND_STEP_LENGTH_M, EDOG_12DOF_COMMAND_STEP_HEIGHT_M);
                break;
            case MOTION_CMD_LEFT_FRONT:
                result = trot_left_front_cycle(EDOG_12DOF_COMMAND_STEP_LENGTH_M, EDOG_12DOF_COMMAND_STEP_HEIGHT_M);
                break;
            case MOTION_CMD_RIGHT_FRONT:
                result = trot_right_front_cycle(EDOG_12DOF_COMMAND_STEP_LENGTH_M, EDOG_12DOF_COMMAND_STEP_HEIGHT_M);
                break;
            case MOTION_CMD_LEFT_BACK:
                result = trot_left_back_cycle(EDOG_12DOF_COMMAND_STEP_LENGTH_M, EDOG_12DOF_COMMAND_STEP_HEIGHT_M);
                break;
            case MOTION_CMD_RIGHT_BACK:
                result = trot_right_back_cycle(EDOG_12DOF_COMMAND_STEP_LENGTH_M, EDOG_12DOF_COMMAND_STEP_HEIGHT_M);
                break;
            case MOTION_CMD_SINGLE_LEG_LF:
                result = single_leg_gait_cycle(0, EDOG_12DOF_COMMAND_STEP_LENGTH_M,
                                               EDOG_12DOF_COMMAND_STEP_HEIGHT_M);
                break;
            case MOTION_CMD_SINGLE_LEG_RF:
                result = single_leg_gait_cycle(1, EDOG_12DOF_COMMAND_STEP_LENGTH_M,
                                               EDOG_12DOF_COMMAND_STEP_HEIGHT_M);
                break;
            case MOTION_CMD_SINGLE_LEG_LB:
                result = single_leg_gait_cycle(2, EDOG_12DOF_COMMAND_STEP_LENGTH_M,
                                               EDOG_12DOF_COMMAND_STEP_HEIGHT_M);
                break;
            case MOTION_CMD_SINGLE_LEG_RB:
                result = single_leg_gait_cycle(3, EDOG_12DOF_COMMAND_STEP_LENGTH_M,
                                               EDOG_12DOF_COMMAND_STEP_HEIGHT_M);
                break;
            case MOTION_CMD_LEG_GROUP:
                result = leg_group_gait_cycle(g_currentLegMask,
                                              EDOG_12DOF_COMMAND_STEP_LENGTH_M,
                                              EDOG_12DOF_COMMAND_STEP_HEIGHT_M);
                break;
            default:
                break;
        }

        // 如果动作被中断（result=0），说明可能收到了停止指令
        if (result == 0) {
            // 确保状态同步
            if (isStopFlag()) {
                g_currentMotion = MOTION_CMD_NONE;
                g_motionRepeatCount = -1;
                if (g_stopReturnPending) {
                    int stopDone = smooth_stop_to_stand(EDOG_12DOF_COMMAND_STEP_LENGTH_M,
                                                        EDOG_12DOF_COMMAND_STEP_HEIGHT_M);
                    g_stopReturnPending = 0;
                    if (stopDone) {
                        resetStopFlag(); // 重置标志，准备下一次指令
                    }
                } else {
                    resetStopFlag(); // 重置标志，准备下一次指令
                }
            }
        } else {
            // Cycle completed successfully
            if (g_motionRepeatCount > 0) {
                g_motionRepeatCount--;
                if (g_motionRepeatCount == 0) {
                    g_currentMotion = MOTION_CMD_NONE;
                    g_motionRepeatCount = -1;
                    init_dog(EDOG_12DOF_COMMAND_STEP_LENGTH_M, EDOG_12DOF_COMMAND_STEP_HEIGHT_M);
                }
            }
        }
        
        // 稍微延时，避免CPU占用过高（虽然cycle函数里有延时）
        usleep(1000);
    }
}

bool IotControl_HandleCommandStringWithSteps(const char *command, double stepLengthM, double stepHeightM)
{
    double frontHipAdductionDeg = EDOG_12DOF_FRONT_HIP_ADDUCTION_DEFAULT_DEG;
    double rearHipAdductionDeg = EDOG_12DOF_REAR_HIP_ADDUCTION_DEFAULT_DEG;
    double frontBodyHeightDeltaMm = 0.0;
    double rearBodyHeightDeltaMm = 0.0;
    double thighLengthM = SPOTMICRO_THIGH_LENGTH_MM / 1000.0;
    double calfLengthM = SPOTMICRO_CALF_LENGTH_MM / 1000.0;

    GetRuntimeGaitGeometryDefaults(&frontHipAdductionDeg, &rearHipAdductionDeg,
                                   &thighLengthM, &calfLengthM);
    GetRuntimeBodyHeightDefaults(&frontBodyHeightDeltaMm, &rearBodyHeightDeltaMm);
    return IotControl_HandleCommandStringWithRuntimeParams(command,
                                                           stepLengthM,
                                                           stepHeightM,
                                                           frontHipAdductionDeg,
                                                           rearHipAdductionDeg,
                                                           frontBodyHeightDeltaMm,
                                                           rearBodyHeightDeltaMm,
                                                           thighLengthM,
                                                           calfLengthM);
}

static bool IotControl_HandleCommandStringWithRuntimeParams(const char *command,
                                                            double stepLengthM,
                                                            double stepHeightM,
                                                            double frontHipAdductionDeg,
                                                            double rearHipAdductionDeg,
                                                            double frontBodyHeightDeltaMm,
                                                            double rearBodyHeightDeltaMm,
                                                            double thighLengthM,
                                                            double calfLengthM)
{
    if (command == NULL) {
        return false;
    }

    bool isCoze = false;
    char baseCommand[EDOG_MOTION_COMMAND_TEXT_LENGTH] = {0};
    const char *suffix = "_coze";
    size_t cmdLen = strlen(command);
    size_t suffixLen = strlen(suffix);

    if (cmdLen > suffixLen && strcmp(command + cmdLen - suffixLen, suffix) == 0) {
        isCoze = true;
    }
    CopyCommandBase(command, baseCommand, sizeof(baseCommand));

    MotionCommand cmd = MotionCommandFromString(baseCommand);
    if (cmd == MOTION_CMD_UNKNOWN) {
        printf("[Edog] content=%s, execute unknown command\n", command);
        return false;
    }

    if (cmd == MOTION_CMD_NONE) {
        printf("[Edog] content=stop, 停止当前动作\n");
        ExecuteMotionWithRuntimeParams(MOTION_CMD_NONE, -1,
                                       stepLengthM, stepHeightM,
                                       frontHipAdductionDeg, rearHipAdductionDeg,
                                       frontBodyHeightDeltaMm, rearBodyHeightDeltaMm,
                                       thighLengthM, calfLengthM);
        return true;
    }

    if (isCoze) {
        printf("[Edog] Coze command detected: %s, executing 2 times\n", command);
        ExecuteMotionWithRuntimeParams(cmd, 2, stepLengthM, stepHeightM,
                                       frontHipAdductionDeg, rearHipAdductionDeg,
                                       frontBodyHeightDeltaMm, rearBodyHeightDeltaMm,
                                       thighLengthM, calfLengthM);
    } else {
        printf("[Edog] content=%s, 准备执行\n", command);
        ExecuteMotionWithRuntimeParams(cmd, -1, stepLengthM, stepHeightM,
                                       frontHipAdductionDeg, rearHipAdductionDeg,
                                       frontBodyHeightDeltaMm, rearBodyHeightDeltaMm,
                                       thighLengthM, calfLengthM);
    }
    return true;
}

bool IotControl_HandleCommandString(const char *command)
{
    return IotControl_HandleCommandStringWithSteps(command,
                                                   EDOG_12DOF_COMMAND_STEP_LENGTH_M,
                                                   EDOG_12DOF_COMMAND_STEP_HEIGHT_M);
}

bool IotControl_SetMotionSpeedLevel(int level)
{
    if (level < 0 || level > 6) {
        printf("[Motion] invalid speed level=%d, valid range is 0~6\n", level);
        return false;
    }

    setSpeedLevel(level);
    return true;
}

bool IotControl_SetRuntimeGaitGeometry(double hipAdductionDeg, double thighLengthM, double calfLengthM)
{
    LockMotionState();
    ApplyRuntimeGaitGeometryNoLock(hipAdductionDeg, thighLengthM, calfLengthM);
    UnlockMotionState();
    GetMotionLegLengthSnapshot(&thighLengthM, &calfLengthM);
    return true;
}

bool IotControl_SetRuntimeTuning(double frontHipAdductionDeg, double rearHipAdductionDeg,
                                 double frontBodyHeightDeltaMm, double rearBodyHeightDeltaMm,
                                 int imuBalanceStrengthPercent, double thighLengthM,
                                 double calfLengthM, int saveToKv)
{
    double normalizedThighLengthM = NormalizeLegLengthM(thighLengthM,
                                                        SPOTMICRO_THIGH_LENGTH_MM / 1000.0);
    double normalizedCalfLengthM = NormalizeLegLengthM(calfLengthM,
                                                       SPOTMICRO_CALF_LENGTH_MM / 1000.0);
    double normalizedFrontBodyHeightDeltaMm = NormalizeBodyHeightDeltaMm(frontBodyHeightDeltaMm);
    double normalizedRearBodyHeightDeltaMm = NormalizeBodyHeightDeltaMm(rearBodyHeightDeltaMm);

    LockMotionState();
    ApplyRuntimeGaitGeometryForFrontRearNoLock(frontHipAdductionDeg, rearHipAdductionDeg,
                                               normalizedThighLengthM, normalizedCalfLengthM);
    ApplyRuntimeBodyHeightNoLock(normalizedFrontBodyHeightDeltaMm,
                                 normalizedRearBodyHeightDeltaMm);
    (void)setImuBalanceStrengthPercent(imuBalanceStrengthPercent);
    UnlockMotionState();
    if (saveToKv) {
        (void)saveRuntimeTuningToKv();
    }
    return true;
}

bool IotControl_HandleCommandNumber(int value)
{
    if (value == 1) {
        printf("[Edog] command value=1, execute trot\n");
        ExecuteMotion(MOTION_CMD_TROT, -1);
        return true;
    }
    return false;
}

void IotControl_StopMotion(void)
{
    ExecuteMotion(MOTION_CMD_NONE, -1);
}

bool IotControl_SetSingleServoAngle(int channel, int angle)
{
    if (!IsServoChannelValid(channel)) {
        printf("[Servo] invalid channel=%d, valid range is 0~%d\n",
               channel, EDOG_SERVO_CHANNEL_COUNT - 1);
        return false;
    }

    if (angle < EDOG_SERVO_MIN_ANGLE || angle > EDOG_SERVO_MAX_ANGLE) {
        printf("[Servo] invalid angle=%d, valid range is %d~%d\n",
               angle, EDOG_SERVO_MIN_ANGLE, EDOG_SERVO_MAX_ANGLE);
        return false;
    }

    if (initPCA9685() != 0) {
        printf("[Servo] PCA9685 init failed, manual servo command rejected\n");
        return false;
    }
    PrepareManualServoControl();
    if (setDogServoAngleTracked(channel, angle) != 0) {
        printf("[Servo] channel=%d angle=%d apply failed\n", channel, angle);
        return false;
    }
    EnableManualPoseHold();
    printf("[Servo] command channel=%d angle=%d\n", channel, angle);
    return true;
}

bool IotControl_SetServoBatchAngles(int count, const int channels[], const int angles[])
{
    if (count <= 0 || count > EDOG_SERVO_CHANNEL_COUNT || channels == NULL || angles == NULL) {
        printf("[ServoBatch] invalid count=%d\n", count);
        return false;
    }

    for (int i = 0; i < count; i++) {
        if (!IsServoChannelValid(channels[i])) {
            printf("[ServoBatch] invalid channel=%d at index=%d\n", channels[i], i);
            return false;
        }
        if (angles[i] < EDOG_SERVO_MIN_ANGLE || angles[i] > EDOG_SERVO_MAX_ANGLE) {
            printf("[ServoBatch] invalid angle=%d at index=%d\n", angles[i], i);
            return false;
        }
    }

    if (initPCA9685() != 0) {
        printf("[ServoBatch] PCA9685 init failed, batch command rejected\n");
        return false;
    }
    PrepareManualServoControl();

    for (int i = 0; i < count; i++) {
        if (setDogServoAngleTracked(channels[i], angles[i]) != 0) {
            printf("[ServoBatch] channel=%d angle=%d apply failed\n", channels[i], angles[i]);
            return false;
        }
        usleep(EDOG_SERVO_MOTION_STEP_DELAY_US);
    }

    EnableManualPoseHold();
    printf("[ServoBatch] command count=%d\n", count);
    return true;
}

bool IotControl_SetServoCenterTrim(int channel, int trim, int applyNow)
{
    if (!IsServoChannelValid(channel)) {
        printf("[ServoTrim] invalid channel=%d, valid range is 0~%d\n",
               channel, EDOG_SERVO_CHANNEL_COUNT - 1);
        return false;
    }
    if (trim < -45 || trim > 45) {
        printf("[ServoTrim] invalid trim=%d, valid range is -45~45\n", trim);
        return false;
    }

    if (initPCA9685() != 0) {
        printf("[ServoTrim] PCA9685 init failed, trim command rejected\n");
        return false;
    }
    PrepareManualServoControl();
    if (saveDogServoCenterTrim(channel, trim) != 0) {
        return false;
    }
    if (applyNow && setDogServoAngleTracked(channel, EDOG_SERVO_CENTER_ANGLE) != 0) {
        printf("[ServoTrim] channel=%d center apply failed\n", channel);
        return false;
    }

    EnableManualPoseHold();
    printf("[ServoTrim] command channel=%d trim=%d applyNow=%d\n", channel, trim, applyNow);
    return true;
}

bool IotControl_SetAllServoCenterTrims(const int trims[], int count, int resetToInit)
{
    if (trims == NULL || count != EDOG_SERVO_CHANNEL_COUNT) {
        printf("[ServoTrim] invalid batch trim count=%d\n", count);
        return false;
    }
    for (int channel = 0; channel < EDOG_SERVO_CHANNEL_COUNT; channel++) {
        if (trims[channel] < -45 || trims[channel] > 45) {
            printf("[ServoTrim] invalid batch trim channel=%d trim=%d\n", channel, trims[channel]);
            return false;
        }
    }
    if (initPCA9685() != 0) {
        printf("[ServoTrim] PCA9685 init failed, batch trim command rejected\n");
        return false;
    }

    PrepareManualServoControl();
    for (int channel = 0; channel < EDOG_SERVO_CHANNEL_COUNT; channel++) {
        if (saveDogServoCenterTrim(channel, trims[channel]) != 0) {
            return false;
        }
        usleep(EDOG_SERVO_MOTION_STEP_DELAY_US);
    }

    if (resetToInit) {
        resetStopFlag();
        init_dog(EDOG_12DOF_COMMAND_STEP_LENGTH_M, EDOG_12DOF_COMMAND_STEP_HEIGHT_M);
    } else {
        EnableManualPoseHold();
    }
    printf("[ServoTrim] batch command count=%d resetToInit=%d\n", count, resetToInit);
    return true;
}

bool IotControl_ReportServoCalibration(void)
{
    char payload[EDOG_MQTT_BUFFER_LENGTH] = {0};
    char offsets[128] = {0};
    int offsetValues[EDOG_SERVO_CHANNEL_COUNT] = {0};
    size_t used = 0;

    used += snprintf(offsets + used, sizeof(offsets) - used, "[");
    for (int channel = 0; channel < EDOG_SERVO_CHANNEL_COUNT && used < sizeof(offsets); channel++) {
        offsetValues[channel] = getDogServoCenterTrim(channel);
        used += snprintf(offsets + used,
                         sizeof(offsets) - used,
                         "%s%d",
                         channel == 0 ? "" : ",",
                         offsetValues[channel]);
    }
    (void)snprintf(offsets + used, sizeof(offsets) - used, "]");

    snprintf(payload,
             sizeof(payload),
             "{\"type\":\"servo_calibration_report\",\"offsets\":%s,\"center_angle\":%d}",
             offsets,
             EDOG_SERVO_CENTER_ANGLE);
    send_msg_to_edog(payload);
    send_servo_calibration_properties(offsetValues, EDOG_SERVO_CHANNEL_COUNT);
    return true;
}

bool IotControl_ReportServoStatus(const char *state)
{
    char payload[EDOG_MQTT_BUFFER_LENGTH] = {0};
    char anglesText[160] = {0};
    char offsetsText[160] = {0};
    int angles[EDOG_SERVO_CHANNEL_COUNT] = {0};
    int offsets[EDOG_SERVO_CHANNEL_COUNT] = {0};
    size_t angleUsed = 0;
    size_t offsetUsed = 0;

    if (getDogServoTrackedAngles(angles, EDOG_SERVO_CHANNEL_COUNT) < 0) {
        return false;
    }
    angleUsed += snprintf(anglesText + angleUsed, sizeof(anglesText) - angleUsed, "[");
    offsetUsed += snprintf(offsetsText + offsetUsed, sizeof(offsetsText) - offsetUsed, "[");
    for (int channel = 0; channel < EDOG_SERVO_CHANNEL_COUNT; channel++) {
        offsets[channel] = getDogServoCenterTrim(channel);
        angleUsed += snprintf(anglesText + angleUsed,
                              sizeof(anglesText) - angleUsed,
                              "%s%d",
                              channel == 0 ? "" : ",",
                              angles[channel]);
        offsetUsed += snprintf(offsetsText + offsetUsed,
                               sizeof(offsetsText) - offsetUsed,
                               "%s%d",
                               channel == 0 ? "" : ",",
                               offsets[channel]);
    }
    (void)snprintf(anglesText + angleUsed, sizeof(anglesText) - angleUsed, "]");
    (void)snprintf(offsetsText + offsetUsed, sizeof(offsetsText) - offsetUsed, "]");

    snprintf(payload,
             sizeof(payload),
             "{\"type\":\"servo_status_report\",\"state\":\"%s\",\"angles\":%s,"
             "\"offsets\":%s,\"center_angle\":%d,\"formula\":\"trim=physical_center_angle-90\"}",
             (state == NULL || state[0] == '\0') ? "running" : state,
             anglesText,
             offsetsText,
             EDOG_SERVO_CENTER_ANGLE);
    send_msg_to_edog(payload);
    send_servo_status_properties(angles, offsets, EDOG_SERVO_CHANNEL_COUNT,
                                 (state == NULL || state[0] == '\0') ? "running" : state);
    return true;
}

bool IotControl_StraightenLegs(void)
{
    if (initPCA9685() != 0) {
        printf("[Servo] PCA9685 init failed, straight-leg command rejected\n");
        return false;
    }
    PrepareManualServoControl();
    if (setDogLegsStraightPose() != 0) {
        printf("[Servo] straight-leg calibration pose apply failed\n");
        return false;
    }
    EnableManualPoseHold();
    printf("[Servo] command straight_legs\n");
    return true;
}

bool IotControl_RunSingleLegGait(int legIndex, int repeatCount)
{
    MotionCommand cmd = SingleLegCommandFromIndex(legIndex);

    if (cmd == MOTION_CMD_UNKNOWN) {
        printf("[SingleLeg] invalid leg index=%d\n", legIndex);
        return false;
    }
    if (repeatCount == 0 || repeatCount < -1) {
        printf("[SingleLeg] invalid repeat count=%d\n", repeatCount);
        return false;
    }

    printf("[SingleLeg] leg=%d repeat=%d\n", legIndex, repeatCount);
    ExecuteMotion(cmd, repeatCount == -1 ? -1 : repeatCount);
    return true;
}

bool IotControl_RunLegGait(int legMask, int repeatCount)
{
    return IotControl_RunLegGaitWithSteps(legMask, repeatCount,
                                          EDOG_12DOF_COMMAND_STEP_LENGTH_M,
                                          EDOG_12DOF_COMMAND_STEP_HEIGHT_M);
}

bool IotControl_RunLegGaitWithSteps(int legMask, int repeatCount,
                                    double stepLengthM, double stepHeightM)
{
    double frontHipAdductionDeg = EDOG_12DOF_FRONT_HIP_ADDUCTION_DEFAULT_DEG;
    double rearHipAdductionDeg = EDOG_12DOF_REAR_HIP_ADDUCTION_DEFAULT_DEG;
    double frontBodyHeightDeltaMm = 0.0;
    double rearBodyHeightDeltaMm = 0.0;
    double thighLengthM = SPOTMICRO_THIGH_LENGTH_MM / 1000.0;
    double calfLengthM = SPOTMICRO_CALF_LENGTH_MM / 1000.0;

    GetRuntimeGaitGeometryDefaults(&frontHipAdductionDeg, &rearHipAdductionDeg,
                                   &thighLengthM, &calfLengthM);
    GetRuntimeBodyHeightDefaults(&frontBodyHeightDeltaMm, &rearBodyHeightDeltaMm);
    return IotControl_RunLegGaitWithRuntimeParams(legMask, repeatCount,
                                                  stepLengthM, stepHeightM,
                                                  frontHipAdductionDeg,
                                                  rearHipAdductionDeg,
                                                  frontBodyHeightDeltaMm,
                                                  rearBodyHeightDeltaMm,
                                                  thighLengthM,
                                                  calfLengthM);
}

static bool IotControl_RunLegGaitWithRuntimeParams(int legMask, int repeatCount,
                                                   double stepLengthM, double stepHeightM,
                                                   double frontHipAdductionDeg,
                                                   double rearHipAdductionDeg,
                                                   double frontBodyHeightDeltaMm,
                                                   double rearBodyHeightDeltaMm,
                                                   double thighLengthM,
                                                   double calfLengthM)
{
    if ((legMask & 0x0F) == 0 || (legMask & ~0x0F) != 0) {
        printf("[LegGroup] invalid leg mask=%d\n", legMask);
        return false;
    }
    if (repeatCount == 0 || repeatCount < -1) {
        printf("[LegGroup] invalid repeat count=%d\n", repeatCount);
        return false;
    }

    printf("[LegGroup] mask=%d repeat=%d\n", legMask, repeatCount);
    if (legMask == 1 || legMask == 2 || legMask == 4 || legMask == 8) {
        int legIndex = (legMask == 1) ? 0 : (legMask == 2) ? 1 : (legMask == 4) ? 2 : 3;
        MotionCommand cmd = SingleLegCommandFromIndex(legIndex);
        if (cmd == MOTION_CMD_UNKNOWN) {
            return false;
        }
        ExecuteMotionWithRuntimeParams(cmd, repeatCount == -1 ? -1 : repeatCount,
                                       stepLengthM, stepHeightM,
                                       frontHipAdductionDeg, rearHipAdductionDeg,
                                       frontBodyHeightDeltaMm, rearBodyHeightDeltaMm,
                                       thighLengthM, calfLengthM);
        return true;
    }

    LockMotionState();
    ClearManualPoseHold();
    g_currentMotion = MOTION_CMD_LEG_GROUP;
    g_currentLegMask = legMask;
    g_motionRepeatCount = repeatCount == -1 ? -1 : repeatCount;
    g_stopReturnPending = 0;
    SetMotionStepState(stepLengthM, stepHeightM);
    ApplyRuntimeGaitGeometryForFrontRearNoLock(frontHipAdductionDeg, rearHipAdductionDeg,
                                               thighLengthM, calfLengthM);
    ApplyRuntimeBodyHeightNoLock(frontBodyHeightDeltaMm, rearBodyHeightDeltaMm);
    resetStopFlag();
    UnlockMotionState();
    return true;
}
