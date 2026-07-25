#ifndef IOT_CONTROL_H
#define IOT_CONTROL_H

#include <stdbool.h>
#include "edog_config.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    IOT_CONTROL_COMMAND_TEXT = 0,
    IOT_CONTROL_COMMAND_NUMBER,
    IOT_CONTROL_COMMAND_SPEED_LEVEL,
    IOT_CONTROL_COMMAND_SERVO_SET,
    IOT_CONTROL_COMMAND_SERVO_BATCH_SET,
    IOT_CONTROL_COMMAND_SERVO_TRIM_SET,
    IOT_CONTROL_COMMAND_SERVO_TRIM_BATCH_SET,
    IOT_CONTROL_COMMAND_SERVO_CALIBRATION_REPORT,
    IOT_CONTROL_COMMAND_SERVO_STATUS_REPORT,
    IOT_CONTROL_COMMAND_STRAIGHTEN_LEGS,
    IOT_CONTROL_COMMAND_LEG_GAIT,
    IOT_CONTROL_COMMAND_RUNTIME_TUNING_SET
} IotControlCommandType;

typedef struct {
    IotControlCommandType type;
    char text[EDOG_MOTION_COMMAND_TEXT_LENGTH];
    int value;
    int channel;
    int angle;
    int trim;
    int applyNow;
    int legIndex;
    int legMask;
    int repeatCount;
    double stepLengthM;
    double stepHeightM;
    double hipAdductionDeg;
    double frontHipAdductionDeg;
    double rearHipAdductionDeg;
    double frontBodyHeightDeltaMm;
    double rearBodyHeightDeltaMm;
    double thighLengthM;
    double calfLengthM;
    int imuBalanceStrengthPercent;
    int count;
    int channels[EDOG_SERVO_CHANNEL_COUNT];
    int angles[EDOG_SERVO_CHANNEL_COUNT];
} IotControlCommand;

bool IotControl_EnqueueCommand(const IotControlCommand *command);
bool IotControl_HandleCommandString(const char *command);
bool IotControl_HandleCommandStringWithSteps(const char *command, double stepLengthM, double stepHeightM);
bool IotControl_SetRuntimeGaitGeometry(double hipAdductionDeg, double thighLengthM, double calfLengthM);
bool IotControl_SetRuntimeTuning(double frontHipAdductionDeg, double rearHipAdductionDeg,
                                 double frontBodyHeightDeltaMm, double rearBodyHeightDeltaMm,
                                 int imuBalanceStrengthPercent, double thighLengthM,
                                 double calfLengthM, int saveToKv);
bool IotControl_HandleCommandNumber(int value);
void IotControl_StopMotion(void);
bool IotControl_IsMotionCommandString(const char *command);
bool IotControl_SetMotionSpeedLevel(int level);
bool IotControl_SetSingleServoAngle(int channel, int angle);
bool IotControl_SetServoBatchAngles(int count, const int channels[], const int angles[]);
bool IotControl_SetServoCenterTrim(int channel, int trim, int applyNow);
bool IotControl_SetAllServoCenterTrims(const int trims[], int count, int resetToInit);
bool IotControl_ReportServoCalibration(void);
bool IotControl_ReportServoStatus(const char *state);
bool IotControl_StraightenLegs(void);
bool IotControl_RunSingleLegGait(int legIndex, int repeatCount);
bool IotControl_RunLegGait(int legMask, int repeatCount);
bool IotControl_RunLegGaitWithSteps(int legMask, int repeatCount, double stepLengthM, double stepHeightM);

/**
 * @brief 运动控制任务入口函数
 * 该函数应在一个独立的任务中运行，循环检测当前指令并执行动作
 */
void IotControl_MotionTask(void);

#ifdef __cplusplus
}
#endif

#endif // IOT_CONTROL_H
