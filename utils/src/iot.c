#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "../../include/edog_config.h"
#include "MQTTClient.h"
#include "cJSON.h"
#include "config_network.h"
#include "iot.h"
#include "motion_utils.h"
#include "gait_generate.h"
#include "los_task.h"
#include "ohos_init.h"
#include "mqtt_connect.h"
#include "iot_control.h"
#include "../../12_DOF_Version/include/gait_generate_12dof.h"

// ============================================================
// [新增] 定义 edog 相关的 Topic
// 注意：如果在华为云IoTDA使用自定义Topic，通常需要在控制台定义或使用 user/ 前缀
// 这里严格按照要求使用指定字符串
// ============================================================
#define EDOG_SUB_TOPIC "$oc/devices/" EDOG_MQTT_DEVICE_ID "/sys/messages/down"
#define EDOG_PUB_TOPIC "$oc/devices/" EDOG_MQTT_DEVICE_ID "/sys/messages/up"

static unsigned char sendBuf[EDOG_MQTT_PACKET_BUFFER_LENGTH];
static unsigned char readBuf[EDOG_MQTT_PACKET_BUFFER_LENGTH];

Network network;
MQTTClient client;

static char mqtt_clientid[64]  = EDOG_MQTT_CLIENT_ID;
static char mqtt_deviceid[64]  = EDOG_MQTT_DEVICE_ID;
static char mqtt_pwd[72]       = EDOG_MQTT_DEVICE_PASSWORD;
static char mqtt_username[64]  = EDOG_MQTT_USERNAME;

static char publish_topic[128]  = {0};
static char subcribe_topic[128] = {0};
static char response_topic[128] = {0};
static char heartbeat_topic[128] = {0};
static char property_set_topic[128] = {0};

static volatile unsigned int mqttConnectFlag = 0;
static int mqttNetworkReady = 0;
static unsigned int mqttYieldFailureCount = 0;
static unsigned int mqttPublishFailureCount = 0;

#define EDOG_MQTT_YIELD_FAILURE_RECONNECT_THRESHOLD 3
#define EDOG_MQTT_PUBLISH_FAILURE_RECONNECT_THRESHOLD 3

// 外部状态
extern bool motor_state;
extern bool light_state;
extern bool auto_state;

static int HandleIncomingCommandObject(cJSON *root);
static int HandleIncomingCommandText(const char *text);

static unsigned int MqttStringLen(const char *value)
{
    return value == NULL ? 0U : (unsigned int)strlen(value);
}

static void PrintMqttConnectDiagnostics(void)
{
    printf("[MQTT] diag host=%s port=1883\n", EDOG_MQTT_HOST_ADDR);
    printf("[MQTT] diag deviceId=%s len=%u\n",
           mqtt_deviceid, MqttStringLen(mqtt_deviceid));
    printf("[MQTT] diag clientId=%s len=%u\n",
           mqtt_clientid, MqttStringLen(mqtt_clientid));
    printf("[MQTT] diag username=%s len=%u\n",
           mqtt_username, MqttStringLen(mqtt_username));
    printf("[MQTT] diag keepalive=%d timeout_ms=%d mqtt_version=4 clean_session=1\n",
           EDOG_MQTT_KEEPALIVE_SECONDS, EDOG_MQTT_COMMAND_TIMEOUT_MS);
    printf("[MQTT] diag password_len=%u password=***\n",
           MqttStringLen(mqtt_pwd));
}

static void MarkMqttPublishSuccess(void)
{
    mqttPublishFailureCount = 0;
}

static void MarkMqttPublishFailure(const char *source, int rc)
{
    mqttPublishFailureCount++;
    printf("[%s] publish failed rc=%d consecutive=%u\n",
           (source == NULL || source[0] == '\0') ? "MQTT" : source,
           rc,
           mqttPublishFailureCount);
    if (mqttPublishFailureCount >= EDOG_MQTT_PUBLISH_FAILURE_RECONNECT_THRESHOLD) {
        printf("[MQTT] publish failure threshold reached, reconnect required\n");
        mqttConnectFlag = 0;
    }
}

static cJSON *GetObjectItemByNames(cJSON *root, const char *const *names, size_t count)
{
    size_t i;

    if (root == NULL) {
        return NULL;
    }

    for (i = 0; i < count; i++) {
        cJSON *item = cJSON_GetObjectItem(root, names[i]);
        if (item != NULL) {
            return item;
        }
    }
    return NULL;
}

static int GetIntByNames(cJSON *root, const char *const *names, size_t count, int *value)
{
    cJSON *item = GetObjectItemByNames(root, names, count);

    if (!cJSON_IsNumber(item)) {
        return 0;
    }

    *value = item->valueint;
    return 1;
}

static int GetDoubleByNames(cJSON *root, const char *const *names, size_t count, double *value)
{
    cJSON *item = GetObjectItemByNames(root, names, count);

    if (!cJSON_IsNumber(item)) {
        return 0;
    }

    *value = item->valuedouble;
    return 1;
}

static int GetDoubleByNamesWithFallback(cJSON *root, cJSON *args,
                                        const char *const *names, size_t count,
                                        double *value)
{
    if (GetDoubleByNames(args, names, count, value)) {
        return 1;
    }
    if (args != root && GetDoubleByNames(root, names, count, value)) {
        return 1;
    }
    return 0;
}

static int GetIntByNamesWithFallback(cJSON *root, cJSON *args,
                                     const char *const *names, size_t count,
                                     int *value)
{
    if (GetIntByNames(args, names, count, value)) {
        return 1;
    }
    if (args != root && GetIntByNames(root, names, count, value)) {
        return 1;
    }
    return 0;
}

static int GetStringByNames(cJSON *root, const char *const *names, size_t count, const char **value)
{
    cJSON *item = GetObjectItemByNames(root, names, count);

    if (!cJSON_IsString(item)) {
        return 0;
    }

    *value = cJSON_GetStringValue(item);
    return *value != NULL;
}

static cJSON *GetArrayByNames(cJSON *root, const char *const *names, size_t count)
{
    cJSON *item = GetObjectItemByNames(root, names, count);

    if (!cJSON_IsArray(item)) {
        return NULL;
    }

    return item;
}

static double ClampGaitStepMinimum(double value, double minValue)
{
    if (value < minValue) {
        return minValue;
    }
    return value;
}

static void ParseGaitStepParameters(cJSON *root, cJSON *paras,
                                    double *stepLengthM, double *stepHeightM)
{
    static const char *const stepLengthMKeys[] = {
        "step_length_m", "stepLengthM", "step_length", "stepLength"
    };
    static const char *const stepHeightMKeys[] = {
        "step_height_m", "stepHeightM", "step_height", "stepHeight"
    };
    cJSON *args = (paras != NULL && cJSON_IsObject(paras)) ? paras : root;
    double lengthM = EDOG_12DOF_COMMAND_STEP_LENGTH_M;
    double heightM = EDOG_12DOF_COMMAND_STEP_HEIGHT_M;

    if (!GetDoubleByNames(args, stepLengthMKeys, sizeof(stepLengthMKeys) / sizeof(stepLengthMKeys[0]), &lengthM) &&
        args != root) {
        (void)GetDoubleByNames(root, stepLengthMKeys, sizeof(stepLengthMKeys) / sizeof(stepLengthMKeys[0]), &lengthM);
    }
    if (!GetDoubleByNames(args, stepHeightMKeys, sizeof(stepHeightMKeys) / sizeof(stepHeightMKeys[0]), &heightM) &&
        args != root) {
        (void)GetDoubleByNames(root, stepHeightMKeys, sizeof(stepHeightMKeys) / sizeof(stepHeightMKeys[0]), &heightM);
    }

    if (stepLengthM != NULL) {
        *stepLengthM = ClampGaitStepMinimum(lengthM, 0.0);
    }
    if (stepHeightM != NULL) {
        *stepHeightM = ClampGaitStepMinimum(heightM, 0.0);
    }
}

static void ParseGaitGeometryParameters(cJSON *root, cJSON *paras,
                                        double *frontHipAdductionDeg,
                                        double *rearHipAdductionDeg,
                                        double *thighLengthM,
                                        double *calfLengthM)
{
    static const char *const hipAdductionKeys[] = {
        "hip_adduction_deg", "hipAdductionDeg", "hip_adduction", "hipAdduction"
    };
    static const char *const frontHipAdductionKeys[] = {
        "front_hip_adduction_deg", "frontHipAdductionDeg",
        "front_hip_adduction", "frontHipAdduction"
    };
    static const char *const rearHipAdductionKeys[] = {
        "rear_hip_adduction_deg", "rearHipAdductionDeg",
        "rear_hip_adduction", "rearHipAdduction"
    };
    static const char *const thighLengthMKeys[] = {
        "thigh_length_m", "thighLengthM", "thigh_length", "thighLength"
    };
    static const char *const thighLengthMmKeys[] = {
        "thigh_length_mm", "thighLengthMm"
    };
    static const char *const calfLengthMKeys[] = {
        "calf_length_m", "calfLengthM", "calf_length", "calfLength"
    };
    static const char *const calfLengthMmKeys[] = {
        "calf_length_mm", "calfLengthMm"
    };
    cJSON *args = (paras != NULL && cJSON_IsObject(paras)) ? paras : root;
    double frontHipDeg = EDOG_12DOF_FRONT_HIP_ADDUCTION_DEFAULT_DEG;
    double rearHipDeg = EDOG_12DOF_REAR_HIP_ADDUCTION_DEFAULT_DEG;
    double thighM = SPOTMICRO_THIGH_LENGTH_MM / 1000.0;
    double calfM = SPOTMICRO_CALF_LENGTH_MM / 1000.0;
    double thighMm = SPOTMICRO_THIGH_LENGTH_MM;
    double calfMm = SPOTMICRO_CALF_LENGTH_MM;
    double lengthMm;
    double parsedDeg;

    Edog12Dof_GetRuntimeGeometryForFrontRear(&frontHipDeg, &rearHipDeg, &thighMm, &calfMm);
    if (thighMm > 0.0) {
        thighM = thighMm / 1000.0;
    }
    if (calfMm > 0.0) {
        calfM = calfMm / 1000.0;
    }

    if (GetDoubleByNamesWithFallback(root, args, hipAdductionKeys,
                                     sizeof(hipAdductionKeys) / sizeof(hipAdductionKeys[0]),
                                     &parsedDeg)) {
        frontHipDeg = parsedDeg;
        rearHipDeg = parsedDeg;
    }
    if (GetDoubleByNamesWithFallback(root, args, frontHipAdductionKeys,
                                     sizeof(frontHipAdductionKeys) / sizeof(frontHipAdductionKeys[0]),
                                     &parsedDeg)) {
        frontHipDeg = parsedDeg;
    }
    if (GetDoubleByNamesWithFallback(root, args, rearHipAdductionKeys,
                                     sizeof(rearHipAdductionKeys) / sizeof(rearHipAdductionKeys[0]),
                                     &parsedDeg)) {
        rearHipDeg = parsedDeg;
    }
    (void)GetDoubleByNamesWithFallback(root, args, thighLengthMKeys,
                                       sizeof(thighLengthMKeys) / sizeof(thighLengthMKeys[0]),
                                       &thighM);
    if (GetDoubleByNamesWithFallback(root, args, thighLengthMmKeys,
                                     sizeof(thighLengthMmKeys) / sizeof(thighLengthMmKeys[0]),
                                     &lengthMm)) {
        thighM = lengthMm / 1000.0;
    }
    (void)GetDoubleByNamesWithFallback(root, args, calfLengthMKeys,
                                       sizeof(calfLengthMKeys) / sizeof(calfLengthMKeys[0]),
                                       &calfM);
    if (GetDoubleByNamesWithFallback(root, args, calfLengthMmKeys,
                                     sizeof(calfLengthMmKeys) / sizeof(calfLengthMmKeys[0]),
                                     &lengthMm)) {
        calfM = lengthMm / 1000.0;
    }

    if (frontHipAdductionDeg != NULL) {
        *frontHipAdductionDeg = frontHipDeg;
    }
    if (rearHipAdductionDeg != NULL) {
        *rearHipAdductionDeg = rearHipDeg;
    }
    if (thighLengthM != NULL) {
        *thighLengthM = thighM;
    }
    if (calfLengthM != NULL) {
        *calfLengthM = calfM;
    }
}

static double ClampBodyHeightDeltaMm(double deltaMm)
{
    if (deltaMm < EDOG_12DOF_BODY_HEIGHT_DELTA_MIN_MM) {
        return EDOG_12DOF_BODY_HEIGHT_DELTA_MIN_MM;
    }
    if (deltaMm > EDOG_12DOF_BODY_HEIGHT_DELTA_MAX_MM) {
        return EDOG_12DOF_BODY_HEIGHT_DELTA_MAX_MM;
    }
    return deltaMm;
}

static void ParseBodyHeightDeltaParameters(cJSON *root, cJSON *paras,
                                           double *frontBodyHeightDeltaMm,
                                           double *rearBodyHeightDeltaMm)
{
    static const char *const bodyHeightKeys[] = {
        "body_height_delta_mm", "bodyHeightDeltaMm",
        "foot_z_delta_mm", "footZDeltaMm"
    };
    static const char *const frontBodyHeightKeys[] = {
        "front_body_height_delta_mm", "frontBodyHeightDeltaMm",
        "front_foot_z_delta_mm", "frontFootZDeltaMm"
    };
    static const char *const rearBodyHeightKeys[] = {
        "rear_body_height_delta_mm", "rearBodyHeightDeltaMm",
        "rear_foot_z_delta_mm", "rearFootZDeltaMm"
    };
    cJSON *args = (paras != NULL && cJSON_IsObject(paras)) ? paras : root;
    double frontDeltaMm = 0.0;
    double rearDeltaMm = 0.0;
    double sharedDeltaMm;
    double parsedDeltaMm;

    Edog12Dof_GetRuntimeFootZDeltas(&frontDeltaMm, &rearDeltaMm);
    if (GetDoubleByNamesWithFallback(root, args, bodyHeightKeys,
                                     sizeof(bodyHeightKeys) / sizeof(bodyHeightKeys[0]),
                                     &sharedDeltaMm)) {
        frontDeltaMm = sharedDeltaMm;
        rearDeltaMm = sharedDeltaMm;
    }
    if (GetDoubleByNamesWithFallback(root, args, frontBodyHeightKeys,
                                     sizeof(frontBodyHeightKeys) / sizeof(frontBodyHeightKeys[0]),
                                     &parsedDeltaMm)) {
        frontDeltaMm = parsedDeltaMm;
    }
    if (GetDoubleByNamesWithFallback(root, args, rearBodyHeightKeys,
                                     sizeof(rearBodyHeightKeys) / sizeof(rearBodyHeightKeys[0]),
                                     &parsedDeltaMm)) {
        rearDeltaMm = parsedDeltaMm;
    }

    if (frontBodyHeightDeltaMm != NULL) {
        *frontBodyHeightDeltaMm = ClampBodyHeightDeltaMm(frontDeltaMm);
    }
    if (rearBodyHeightDeltaMm != NULL) {
        *rearBodyHeightDeltaMm = ClampBodyHeightDeltaMm(rearDeltaMm);
    }
}

static int EnqueueTextCommand(const char *commandText, double stepLengthM, double stepHeightM,
                              double frontHipAdductionDeg, double rearHipAdductionDeg,
                              double frontBodyHeightDeltaMm, double rearBodyHeightDeltaMm,
                              double thighLengthM, double calfLengthM)
{
    IotControlCommand command = {0};

    if (commandText == NULL || commandText[0] == '\0') {
        return 0;
    }

    command.type = IOT_CONTROL_COMMAND_TEXT;
    snprintf(command.text, sizeof(command.text), "%s", commandText);
    command.stepLengthM = stepLengthM;
    command.stepHeightM = stepHeightM;
    command.hipAdductionDeg = frontHipAdductionDeg;
    command.frontHipAdductionDeg = frontHipAdductionDeg;
    command.rearHipAdductionDeg = rearHipAdductionDeg;
    command.frontBodyHeightDeltaMm = frontBodyHeightDeltaMm;
    command.rearBodyHeightDeltaMm = rearBodyHeightDeltaMm;
    command.thighLengthM = thighLengthM;
    command.calfLengthM = calfLengthM;
    return IotControl_EnqueueCommand(&command) ? 1 : -1;
}

static int EnqueueNumberCommand(int value)
{
    IotControlCommand command = {0};

    command.type = IOT_CONTROL_COMMAND_NUMBER;
    command.value = value;
    return IotControl_EnqueueCommand(&command) ? 1 : -1;
}

static int EnqueueSpeedLevelCommand(int level)
{
    IotControlCommand command = {0};

    command.type = IOT_CONTROL_COMMAND_SPEED_LEVEL;
    command.value = level;
    return IotControl_EnqueueCommand(&command) ? 1 : -1;
}

static int EnqueueServoSetCommand(int channel, int angle)
{
    IotControlCommand command = {0};

    command.type = IOT_CONTROL_COMMAND_SERVO_SET;
    command.channel = channel;
    command.angle = angle;
    return IotControl_EnqueueCommand(&command) ? 1 : -1;
}

static int EnqueueServoBatchCommand(int count, const int channels[], const int angles[])
{
    IotControlCommand command = {0};

    if (count <= 0 || count > EDOG_SERVO_CHANNEL_COUNT) {
        return -1;
    }

    command.type = IOT_CONTROL_COMMAND_SERVO_BATCH_SET;
    command.count = count;
    for (int i = 0; i < count; i++) {
        command.channels[i] = channels[i];
        command.angles[i] = angles[i];
    }
    return IotControl_EnqueueCommand(&command) ? 1 : -1;
}

static int EnqueueServoTrimCommand(int channel, int trim, int applyNow)
{
    IotControlCommand command = {0};

    command.type = IOT_CONTROL_COMMAND_SERVO_TRIM_SET;
    command.channel = channel;
    command.trim = trim;
    command.applyNow = applyNow;
    return IotControl_EnqueueCommand(&command) ? 1 : -1;
}

static int EnqueueServoTrimBatchCommand(const int trims[], int count, int resetToInit)
{
    IotControlCommand command = {0};

    if (trims == NULL || count != EDOG_SERVO_CHANNEL_COUNT) {
        return -1;
    }

    command.type = IOT_CONTROL_COMMAND_SERVO_TRIM_BATCH_SET;
    command.count = count;
    command.applyNow = resetToInit;
    for (int i = 0; i < count; i++) {
        command.angles[i] = trims[i];
    }
    return IotControl_EnqueueCommand(&command) ? 1 : -1;
}

static int EnqueueServoCalibrationReportCommand(void)
{
    IotControlCommand command = {0};

    command.type = IOT_CONTROL_COMMAND_SERVO_CALIBRATION_REPORT;
    return IotControl_EnqueueCommand(&command) ? 1 : -1;
}

static int EnqueueServoStatusReportCommand(void)
{
    IotControlCommand command = {0};

    command.type = IOT_CONTROL_COMMAND_SERVO_STATUS_REPORT;
    return IotControl_EnqueueCommand(&command) ? 1 : -1;
}

static int EnqueueStraightenLegsCommand(void)
{
    IotControlCommand command = {0};

    command.type = IOT_CONTROL_COMMAND_STRAIGHTEN_LEGS;
    return IotControl_EnqueueCommand(&command) ? 1 : -1;
}

static int EnqueueLegGaitCommand(int legMask, int repeatCount, double stepLengthM, double stepHeightM,
                                 double frontHipAdductionDeg, double rearHipAdductionDeg,
                                 double frontBodyHeightDeltaMm, double rearBodyHeightDeltaMm,
                                 double thighLengthM, double calfLengthM)
{
    IotControlCommand command = {0};

    if ((legMask & 0x0F) == 0 || (legMask & ~0x0F) != 0 || repeatCount == 0 || repeatCount < -1) {
        return -1;
    }

    command.type = IOT_CONTROL_COMMAND_LEG_GAIT;
    command.legMask = legMask;
    command.repeatCount = repeatCount;
    command.stepLengthM = stepLengthM;
    command.stepHeightM = stepHeightM;
    command.hipAdductionDeg = frontHipAdductionDeg;
    command.frontHipAdductionDeg = frontHipAdductionDeg;
    command.rearHipAdductionDeg = rearHipAdductionDeg;
    command.frontBodyHeightDeltaMm = frontBodyHeightDeltaMm;
    command.rearBodyHeightDeltaMm = rearBodyHeightDeltaMm;
    command.thighLengthM = thighLengthM;
    command.calfLengthM = calfLengthM;
    return IotControl_EnqueueCommand(&command) ? 1 : -1;
}

static int EnqueueRuntimeTuningCommand(double frontHipAdductionDeg, double rearHipAdductionDeg,
                                       double frontBodyHeightDeltaMm, double rearBodyHeightDeltaMm,
                                       int imuBalanceStrengthPercent, double thighLengthM,
                                       double calfLengthM, int saveToKv)
{
    IotControlCommand command = {0};

    command.type = IOT_CONTROL_COMMAND_RUNTIME_TUNING_SET;
    command.hipAdductionDeg = frontHipAdductionDeg;
    command.frontHipAdductionDeg = frontHipAdductionDeg;
    command.rearHipAdductionDeg = rearHipAdductionDeg;
    command.frontBodyHeightDeltaMm = frontBodyHeightDeltaMm;
    command.rearBodyHeightDeltaMm = rearBodyHeightDeltaMm;
    command.imuBalanceStrengthPercent = imuBalanceStrengthPercent;
    command.thighLengthM = thighLengthM;
    command.calfLengthM = calfLengthM;
    command.applyNow = saveToKv;
    return IotControl_EnqueueCommand(&command) ? 1 : -1;
}

static int IsServoSetCommand(const char *commandName)
{
    return commandName != NULL &&
           (strcmp(commandName, "servo_set") == 0 ||
            strcmp(commandName, "servo_calibrate") == 0 ||
            strcmp(commandName, "servo_move") == 0);
}

static int IsServoBatchCommand(const char *commandName)
{
    return commandName != NULL &&
           (strcmp(commandName, "servo_batch") == 0 ||
            strcmp(commandName, "servo_batch_set") == 0 ||
            strcmp(commandName, "servos_set") == 0);
}

static int IsServoTrimCommand(const char *commandName)
{
    return commandName != NULL &&
           (strcmp(commandName, "servo_trim") == 0 ||
            strcmp(commandName, "servo_center_trim") == 0 ||
            strcmp(commandName, "servo_offset") == 0 ||
            strcmp(commandName, "servo_calibration_set") == 0);
}

static int IsServoCalibrationReadCommand(const char *commandName)
{
    return commandName != NULL &&
           (strcmp(commandName, "servo_calibration_read") == 0 ||
            strcmp(commandName, "servo_trim_read") == 0 ||
            strcmp(commandName, "read_servo_calibration") == 0 ||
            strcmp(commandName, "get_servo_calibration") == 0);
}

static int IsServoCalibrationBatchCommand(const char *commandName)
{
    return commandName != NULL &&
           (strcmp(commandName, "servo_calibration_apply_all") == 0 ||
            strcmp(commandName, "servo_trim_batch") == 0 ||
            strcmp(commandName, "servo_calibration_batch") == 0 ||
            strcmp(commandName, "servo_calibration_clear") == 0);
}

static int IsServoStatusReadCommand(const char *commandName)
{
    return commandName != NULL &&
           (strcmp(commandName, "servo_status_read") == 0 ||
            strcmp(commandName, "servo_angles_read") == 0 ||
            strcmp(commandName, "read_servo_status") == 0 ||
            strcmp(commandName, "get_servo_status") == 0);
}

static int IsStraightLegsCommand(const char *commandName)
{
    return commandName != NULL &&
           (strcmp(commandName, "legs_straight") == 0 ||
            strcmp(commandName, "straight_legs") == 0 ||
            strcmp(commandName, "dog_legs_straight") == 0);
}

static int IsSpeedSetCommand(const char *commandName)
{
    return commandName != NULL &&
           (strcmp(commandName, "speed_set") == 0 ||
            strcmp(commandName, "motion_speed") == 0 ||
            strcmp(commandName, "set_speed_level") == 0 ||
            strcmp(commandName, "set_motion_speed") == 0);
}

static int IsRuntimeTuningCommand(const char *commandName)
{
    return commandName != NULL &&
           (strcmp(commandName, "runtime_tuning_set") == 0 ||
            strcmp(commandName, "runtime_tuning") == 0 ||
            strcmp(commandName, "set_runtime_tuning") == 0);
}

static int IsSingleLegGaitCommand(const char *commandName)
{
    return commandName != NULL &&
           (strcmp(commandName, "single_leg_gait") == 0 ||
            strcmp(commandName, "single_leg") == 0 ||
            strcmp(commandName, "leg_gait") == 0);
}

static int LegIndexFromString(const char *leg)
{
    if (leg == NULL) {
        return -1;
    }
    if (strcmp(leg, "LF") == 0 || strcmp(leg, "lf") == 0 ||
        strcmp(leg, "left_front") == 0 || strcmp(leg, "front_left") == 0) {
        return 0;
    }
    if (strcmp(leg, "RF") == 0 || strcmp(leg, "rf") == 0 ||
        strcmp(leg, "right_front") == 0 || strcmp(leg, "front_right") == 0) {
        return 1;
    }
    if (strcmp(leg, "LB") == 0 || strcmp(leg, "lb") == 0 ||
        strcmp(leg, "left_back") == 0 || strcmp(leg, "back_left") == 0) {
        return 2;
    }
    if (strcmp(leg, "RB") == 0 || strcmp(leg, "rb") == 0 ||
        strcmp(leg, "right_back") == 0 || strcmp(leg, "back_right") == 0) {
        return 3;
    }
    return -1;
}

static int LegMaskFromString(const char *leg)
{
    int legIndex = LegIndexFromString(leg);
    return legIndex >= 0 ? (1 << legIndex) : 0;
}

static int ApplySingleLegGait(cJSON *root, cJSON *paras)
{
    static const char *const legKeys[] = {
        "leg", "leg_id", "legId"
    };
    static const char *const legIndexKeys[] = {
        "leg_index", "legIndex"
    };
    static const char *const legMaskKeys[] = {
        "leg_mask", "legMask"
    };
    static const char *const legsKeys[] = {
        "legs", "leg_list", "legList"
    };
    static const char *const repeatKeys[] = {
        "repeat_count", "repeatCount", "repeat"
    };
    cJSON *args = (paras != NULL && cJSON_IsObject(paras)) ? paras : root;
    const char *legText = NULL;
    int legIndex = -1;
    int legMask = 0;
    int repeatCount = 1;
    double stepLengthM = EDOG_12DOF_COMMAND_STEP_LENGTH_M;
    double stepHeightM = EDOG_12DOF_COMMAND_STEP_HEIGHT_M;
    double frontHipAdductionDeg = EDOG_12DOF_FRONT_HIP_ADDUCTION_DEFAULT_DEG;
    double rearHipAdductionDeg = EDOG_12DOF_REAR_HIP_ADDUCTION_DEFAULT_DEG;
    double frontBodyHeightDeltaMm = 0.0;
    double rearBodyHeightDeltaMm = 0.0;
    double thighLengthM = SPOTMICRO_THIGH_LENGTH_MM / 1000.0;
    double calfLengthM = SPOTMICRO_CALF_LENGTH_MM / 1000.0;

    if (!GetIntByNames(args, legMaskKeys, sizeof(legMaskKeys) / sizeof(legMaskKeys[0]), &legMask) &&
        args != root) {
        (void)GetIntByNames(root, legMaskKeys, sizeof(legMaskKeys) / sizeof(legMaskKeys[0]), &legMask);
    }

    if (legMask == 0) {
        cJSON *legs = GetArrayByNames(args, legsKeys, sizeof(legsKeys) / sizeof(legsKeys[0]));
        if (legs == NULL && args != root) {
            legs = GetArrayByNames(root, legsKeys, sizeof(legsKeys) / sizeof(legsKeys[0]));
        }
        if (legs != NULL) {
            int count = cJSON_GetArraySize(legs);
            for (int i = 0; i < count; i++) {
                cJSON *item = cJSON_GetArrayItem(legs, i);
                int legBit;
                if (!cJSON_IsString(item)) {
                    return -1;
                }
                legBit = LegMaskFromString(cJSON_GetStringValue(item));
                if (legBit == 0) {
                    return -1;
                }
                legMask |= legBit;
            }
        }
    }

    if (!GetStringByNames(args, legKeys, sizeof(legKeys) / sizeof(legKeys[0]), &legText) && args != root) {
        (void)GetStringByNames(root, legKeys, sizeof(legKeys) / sizeof(legKeys[0]), &legText);
    }
    if (legMask == 0 && legText != NULL) {
        legIndex = LegIndexFromString(legText);
        if (legIndex >= 0) {
            legMask = 1 << legIndex;
        }
    }
    if (legMask == 0 &&
        !GetIntByNames(args, legIndexKeys, sizeof(legIndexKeys) / sizeof(legIndexKeys[0]), &legIndex) &&
        args != root) {
        (void)GetIntByNames(root, legIndexKeys, sizeof(legIndexKeys) / sizeof(legIndexKeys[0]), &legIndex);
    }
    if (legMask == 0 && legIndex >= 0) {
        legMask = 1 << legIndex;
    }
    if (!GetIntByNames(args, repeatKeys, sizeof(repeatKeys) / sizeof(repeatKeys[0]), &repeatCount) &&
        args != root) {
        (void)GetIntByNames(root, repeatKeys, sizeof(repeatKeys) / sizeof(repeatKeys[0]), &repeatCount);
    }

    if ((legMask & 0x0F) == 0 || (legMask & ~0x0F) != 0 || repeatCount == 0 || repeatCount < -1) {
        printf("[LegGait] invalid leg=%s legMask=%d repeat=%d\n",
               legText == NULL ? "" : legText, legMask, repeatCount);
        return -1;
    }
    ParseGaitStepParameters(root, args, &stepLengthM, &stepHeightM);
    ParseGaitGeometryParameters(root, args,
                                &frontHipAdductionDeg,
                                &rearHipAdductionDeg,
                                &thighLengthM,
                                &calfLengthM);
    ParseBodyHeightDeltaParameters(root, args,
                                   &frontBodyHeightDeltaMm,
                                   &rearBodyHeightDeltaMm);
    return EnqueueLegGaitCommand(legMask, repeatCount, stepLengthM, stepHeightM,
                                 frontHipAdductionDeg, rearHipAdductionDeg,
                                 frontBodyHeightDeltaMm, rearBodyHeightDeltaMm,
                                 thighLengthM, calfLengthM);
}

static int ApplyMotionSpeedLevel(cJSON *root, cJSON *paras)
{
    static const char *const speedLevelKeys[] = {
        "speed_level", "speedLevel", "level"
    };
    cJSON *args = (paras != NULL && cJSON_IsObject(paras)) ? paras : root;
    int level;

    if (GetIntByNames(args, speedLevelKeys, sizeof(speedLevelKeys) / sizeof(speedLevelKeys[0]), &level) ||
        (args != root &&
         GetIntByNames(root, speedLevelKeys, sizeof(speedLevelKeys) / sizeof(speedLevelKeys[0]), &level))) {
        if (level < 0 || level > 6) {
            printf("[Motion] invalid speed level=%d, valid range is 0~6\n", level);
            return -1;
        }
        return EnqueueSpeedLevelCommand(level);
    }

    return 0;
}

static int ApplyRuntimeTuning(cJSON *root, cJSON *paras)
{
    static const char *const imuStrengthKeys[] = {
        "imu_balance_strength_percent", "imuBalanceStrengthPercent",
        "imu_balance_strength", "imuBalanceStrength"
    };
    static const char *const saveToKvKeys[] = {
        "save_to_kv", "saveToKv", "save"
    };
    cJSON *args = (paras != NULL && cJSON_IsObject(paras)) ? paras : root;
    double frontHipAdductionDeg = EDOG_12DOF_FRONT_HIP_ADDUCTION_DEFAULT_DEG;
    double rearHipAdductionDeg = EDOG_12DOF_REAR_HIP_ADDUCTION_DEFAULT_DEG;
    double frontBodyHeightDeltaMm = 0.0;
    double rearBodyHeightDeltaMm = 0.0;
    double thighLengthM = SPOTMICRO_THIGH_LENGTH_MM / 1000.0;
    double calfLengthM = SPOTMICRO_CALF_LENGTH_MM / 1000.0;
    int imuBalanceStrengthPercent = 100;
    int saveToKv = 1;

    ParseGaitGeometryParameters(root, args,
                                &frontHipAdductionDeg,
                                &rearHipAdductionDeg,
                                &thighLengthM,
                                &calfLengthM);
    ParseBodyHeightDeltaParameters(root, args,
                                   &frontBodyHeightDeltaMm,
                                   &rearBodyHeightDeltaMm);
    (void)GetIntByNamesWithFallback(root, args, imuStrengthKeys,
                                    sizeof(imuStrengthKeys) / sizeof(imuStrengthKeys[0]),
                                    &imuBalanceStrengthPercent);
    (void)GetIntByNamesWithFallback(root, args, saveToKvKeys,
                                    sizeof(saveToKvKeys) / sizeof(saveToKvKeys[0]),
                                    &saveToKv);
    return EnqueueRuntimeTuningCommand(frontHipAdductionDeg, rearHipAdductionDeg,
                                       frontBodyHeightDeltaMm, rearBodyHeightDeltaMm,
                                       imuBalanceStrengthPercent,
                                       thighLengthM,
                                       calfLengthM,
                                       saveToKv != 0);
}

static int ParseServoBatchCommand(cJSON *root, cJSON *paras, int channels[], int angles[], int *count)
{
    static const char *const servoArrayKeys[] = {
        "servos", "servo_list", "servoList", "targets"
    };
    static const char *const servoIdKeys[] = {
        "servo_id", "servoId", "channel", "servo_no", "servoNo"
    };
    static const char *const angleKeys[] = {
        "angle", "degree", "servo_angle", "servoAngle"
    };
    cJSON *args = (paras != NULL && cJSON_IsObject(paras)) ? paras : root;
    cJSON *servoArray = GetArrayByNames(args, servoArrayKeys, sizeof(servoArrayKeys) / sizeof(servoArrayKeys[0]));
    int parsedCount = 0;

    if (servoArray == NULL && args != root) {
        servoArray = GetArrayByNames(root, servoArrayKeys, sizeof(servoArrayKeys) / sizeof(servoArrayKeys[0]));
    }
    if (servoArray == NULL) {
        return 0;
    }

    int arraySize = cJSON_GetArraySize(servoArray);
    if (arraySize <= 0 || arraySize > EDOG_SERVO_CHANNEL_COUNT) {
        printf("[ServoBatch] invalid servos array size=%d\n", arraySize);
        return -1;
    }

    for (int i = 0; i < arraySize; i++) {
        cJSON *item = cJSON_GetArrayItem(servoArray, i);
        int channel;
        int angle;

        if (!cJSON_IsObject(item) ||
            !GetIntByNames(item, servoIdKeys, sizeof(servoIdKeys) / sizeof(servoIdKeys[0]), &channel) ||
            !GetIntByNames(item, angleKeys, sizeof(angleKeys) / sizeof(angleKeys[0]), &angle)) {
            printf("[ServoBatch] missing channel or angle at index=%d\n", i);
            return -1;
        }
        if (channel < 0 || channel >= EDOG_SERVO_CHANNEL_COUNT ||
            angle < EDOG_SERVO_MIN_ANGLE || angle > EDOG_SERVO_MAX_ANGLE) {
            printf("[ServoBatch] invalid channel=%d angle=%d at index=%d\n", channel, angle, i);
            return -1;
        }

        channels[parsedCount] = channel;
        angles[parsedCount] = angle;
        parsedCount++;
    }

    *count = parsedCount;
    return parsedCount > 0 ? 1 : 0;
}

static int ParseServoCalibrationOffsets(cJSON *root, cJSON *paras, int trims[], int *count)
{
    static const char *const offsetArrayKeys[] = {
        "offsets", "trims", "calibration", "calibrations"
    };
    cJSON *args = (paras != NULL && cJSON_IsObject(paras)) ? paras : root;
    cJSON *offsetArray = GetArrayByNames(args, offsetArrayKeys, sizeof(offsetArrayKeys) / sizeof(offsetArrayKeys[0]));
    int parsedCount;

    if (offsetArray == NULL && args != root) {
        offsetArray = GetArrayByNames(root, offsetArrayKeys, sizeof(offsetArrayKeys) / sizeof(offsetArrayKeys[0]));
    }
    if (offsetArray == NULL || trims == NULL || count == NULL) {
        return 0;
    }

    parsedCount = cJSON_GetArraySize(offsetArray);
    if (parsedCount != EDOG_SERVO_CHANNEL_COUNT) {
        printf("[ServoTrim] offsets array must contain %d values, got %d\n",
               EDOG_SERVO_CHANNEL_COUNT, parsedCount);
        return -1;
    }

    for (int i = 0; i < parsedCount; i++) {
        cJSON *item = cJSON_GetArrayItem(offsetArray, i);
        int trim;

        if (!cJSON_IsNumber(item)) {
            printf("[ServoTrim] offset index=%d is not a number\n", i);
            return -1;
        }
        trim = item->valueint;
        if (trim < -45 || trim > 45) {
            printf("[ServoTrim] invalid offset channel=%d trim=%d\n", i, trim);
            return -1;
        }
        trims[i] = trim;
    }
    *count = parsedCount;
    return 1;
}

static int HandleServoCommand(const char *commandName, cJSON *root, cJSON *paras)
{
    static const char *const servoIdKeys[] = {
        "servo_id", "servoId", "channel", "servo_no", "servoNo"
    };
    static const char *const angleKeys[] = {
        "angle", "degree", "servo_angle", "servoAngle"
    };
    static const char *const trimKeys[] = {
        "offset", "trim", "center_offset", "centerOffset", "servo_trim", "servoTrim"
    };
    static const char *const physicalAngleKeys[] = {
        "physical_angle", "physicalAngle", "center_angle", "centerAngle"
    };
    static const char *const applyNowKeys[] = {
        "apply_now", "applyNow", "apply"
    };
    static const char *const resetToInitKeys[] = {
        "reset_to_init", "resetToInit", "init", "apply_now", "applyNow"
    };
    cJSON *args = (paras != NULL && cJSON_IsObject(paras)) ? paras : root;
    int channel;
    int angle;
    int trim;
    int applyNow = 0;
    int resetToInit = 1;

    if (IsServoBatchCommand(commandName)) {
        int channels[EDOG_SERVO_CHANNEL_COUNT] = {0};
        int angles[EDOG_SERVO_CHANNEL_COUNT] = {0};
        int count = 0;
        int rc = ParseServoBatchCommand(root, args, channels, angles, &count);

        if (rc <= 0) {
            printf("[ServoBatch] missing or invalid servos array in MQTT payload\n");
            return -1;
        }
        return EnqueueServoBatchCommand(count, channels, angles);
    }

    if (IsServoCalibrationBatchCommand(commandName)) {
        int trims[EDOG_SERVO_CHANNEL_COUNT] = {0};
        int count = 0;

        if (strcmp(commandName, "servo_calibration_clear") == 0) {
            return EnqueueServoTrimBatchCommand(trims, EDOG_SERVO_CHANNEL_COUNT, 1);
        }
        if (ParseServoCalibrationOffsets(root, args, trims, &count) <= 0) {
            printf("[ServoTrim] missing or invalid offsets array in MQTT payload\n");
            return -1;
        }
        (void)GetIntByNames(args, resetToInitKeys, sizeof(resetToInitKeys) / sizeof(resetToInitKeys[0]), &resetToInit);
        return EnqueueServoTrimBatchCommand(trims, count, resetToInit != 0);
    }

    if (IsServoTrimCommand(commandName)) {
        int hasTrim;
        if (!GetIntByNames(args, servoIdKeys, sizeof(servoIdKeys) / sizeof(servoIdKeys[0]), &channel) ||
            (!(hasTrim = GetIntByNames(args, trimKeys, sizeof(trimKeys) / sizeof(trimKeys[0]), &trim)) &&
             !GetIntByNames(args, physicalAngleKeys, sizeof(physicalAngleKeys) / sizeof(physicalAngleKeys[0]), &angle))) {
            printf("[ServoTrim] missing servo_id/channel or offset/physical_angle in MQTT payload\n");
            return -1;
        }
        if (!hasTrim) {
            if (angle < EDOG_SERVO_MIN_ANGLE || angle > EDOG_SERVO_MAX_ANGLE) {
                printf("[ServoTrim] invalid physical angle channel=%d angle=%d\n", channel, angle);
                return -1;
            }
            trim = angle - EDOG_SERVO_CENTER_ANGLE;
        }
        (void)GetIntByNames(args, applyNowKeys, sizeof(applyNowKeys) / sizeof(applyNowKeys[0]), &applyNow);
        if (channel < 0 || channel >= EDOG_SERVO_CHANNEL_COUNT || trim < -45 || trim > 45) {
            printf("[ServoTrim] invalid channel=%d trim=%d\n", channel, trim);
            return -1;
        }
        return EnqueueServoTrimCommand(channel, trim, applyNow != 0);
    }

    if (IsServoCalibrationReadCommand(commandName)) {
        return EnqueueServoCalibrationReportCommand();
    }

    if (IsServoStatusReadCommand(commandName)) {
        return EnqueueServoStatusReportCommand();
    }

    if (IsServoSetCommand(commandName)) {
        if (!GetIntByNames(args, servoIdKeys, sizeof(servoIdKeys) / sizeof(servoIdKeys[0]), &channel) ||
            !GetIntByNames(args, angleKeys, sizeof(angleKeys) / sizeof(angleKeys[0]), &angle)) {
            printf("[Servo] missing servo_id/channel or angle in MQTT payload\n");
            return -1;
        }
        if (channel < 0 || channel >= EDOG_SERVO_CHANNEL_COUNT) {
            printf("[Servo] invalid channel=%d\n", channel);
            return -1;
        }
        if (angle < EDOG_SERVO_MIN_ANGLE || angle > EDOG_SERVO_MAX_ANGLE) {
            printf("[Servo] invalid angle=%d\n", angle);
            return -1;
        }
        return EnqueueServoSetCommand(channel, angle);
    }

    if (IsStraightLegsCommand(commandName)) {
        return EnqueueStraightenLegsCommand();
    }

    return 0;
}

static int HandleCommandText(const char *commandText, cJSON *root, cJSON *args)
{
    int rc;

    if (commandText == NULL || commandText[0] == '\0') {
        return 0;
    }

    rc = HandleServoCommand(commandText, root, args);
    if (rc != 0) {
        return rc;
    }

    if (IsSpeedSetCommand(commandText)) {
        rc = ApplyMotionSpeedLevel(root, args);
        if (rc == 0) {
            printf("[Motion] missing speed_level/speedLevel in MQTT payload\n");
            return -1;
        }
        return rc;
    }

    if (IsRuntimeTuningCommand(commandText)) {
        return ApplyRuntimeTuning(root, args);
    }

    if (IsSingleLegGaitCommand(commandText)) {
        return ApplySingleLegGait(root, args);
    }

    if (IotControl_IsMotionCommandString(commandText)) {
        double stepLengthM = EDOG_12DOF_COMMAND_STEP_LENGTH_M;
        double stepHeightM = EDOG_12DOF_COMMAND_STEP_HEIGHT_M;
        double frontHipAdductionDeg = EDOG_12DOF_FRONT_HIP_ADDUCTION_DEFAULT_DEG;
        double rearHipAdductionDeg = EDOG_12DOF_REAR_HIP_ADDUCTION_DEFAULT_DEG;
        double frontBodyHeightDeltaMm = 0.0;
        double rearBodyHeightDeltaMm = 0.0;
        double thighLengthM = SPOTMICRO_THIGH_LENGTH_MM / 1000.0;
        double calfLengthM = SPOTMICRO_CALF_LENGTH_MM / 1000.0;
        rc = ApplyMotionSpeedLevel(root, args);
        if (rc < 0) {
            return rc;
        }
        ParseGaitStepParameters(root, args, &stepLengthM, &stepHeightM);
        ParseGaitGeometryParameters(root, args,
                                    &frontHipAdductionDeg,
                                    &rearHipAdductionDeg,
                                    &thighLengthM,
                                    &calfLengthM);
        ParseBodyHeightDeltaParameters(root, args,
                                       &frontBodyHeightDeltaMm,
                                       &rearBodyHeightDeltaMm);
        return EnqueueTextCommand(commandText, stepLengthM, stepHeightM,
                                  frontHipAdductionDeg, rearHipAdductionDeg,
                                  frontBodyHeightDeltaMm, rearBodyHeightDeltaMm,
                                  thighLengthM, calfLengthM);
    }

    return 0;
}

static int HandleCommandJsonString(const char *text)
{
    const char *cursor = text;
    cJSON *parsed;
    int rc;

    if (cursor == NULL) {
        return 0;
    }

    while (*cursor == ' ' || *cursor == '\t' || *cursor == '\r' || *cursor == '\n') {
        cursor++;
    }
    if (*cursor != '{') {
        return 0;
    }

    parsed = cJSON_Parse(cursor);
    if (parsed == NULL) {
        return 0;
    }

    rc = HandleIncomingCommandObject(parsed);
    cJSON_Delete(parsed);
    return rc;
}

static int HandleIncomingCommandText(const char *text)
{
    const char *cursor = text;

    if (cursor == NULL) {
        return 0;
    }

    while (*cursor == ' ' || *cursor == '\t' || *cursor == '\r' || *cursor == '\n') {
        cursor++;
    }
    if (*cursor == '\0') {
        return 0;
    }
    if (*cursor == '{') {
        return HandleCommandJsonString(cursor);
    }
    return HandleCommandText(cursor, NULL, NULL);
}

static int HandleCommandValue(cJSON *value, cJSON *root, cJSON *args)
{
    const char *text;
    int rc;

    if (value == NULL) {
        return 0;
    }

    if (cJSON_IsString(value)) {
        text = cJSON_GetStringValue(value);
        rc = HandleCommandText(text, root, args);
        if (rc != 0) {
            return rc;
        }
        return HandleCommandJsonString(text);
    }

    if (cJSON_IsNumber(value)) {
        rc = ApplyMotionSpeedLevel(root, args);
        if (rc < 0) {
            return rc;
        }
        if (value->valueint == 1) {
            return EnqueueNumberCommand(value->valueint);
        }
        return 0;
    }

    if (cJSON_IsObject(value)) {
        return HandleIncomingCommandObject(value);
    }

    return 0;
}

static int HandleIncomingCommandObject(cJSON *root)
{
    static const char *const commandKeys[] = {
        "content", "command", "action", "command_name", "type", "cmd", "name"
    };
    static const char *const nestedObjectKeys[] = {
        "paras", "data", "payload", "body", "content", "cmd_value", "message"
    };
    static const char *const servoIdKeys[] = {
        "servo_id", "servoId", "channel", "servo_no", "servoNo"
    };
    static const char *const angleKeys[] = {
        "angle", "degree", "servo_angle", "servoAngle"
    };
    const char *commandText = NULL;
    cJSON *args = NULL;
    size_t i;
    int channel;
    int angle;
    int rc;
    int batchChannels[EDOG_SERVO_CHANNEL_COUNT] = {0};
    int batchAngles[EDOG_SERVO_CHANNEL_COUNT] = {0};
    int batchCount = 0;

    if (!cJSON_IsObject(root)) {
        return 0;
    }

    args = GetObjectItemByNames(root, nestedObjectKeys, sizeof(nestedObjectKeys) / sizeof(nestedObjectKeys[0]));
    if (!cJSON_IsObject(args)) {
        args = root;
    }

    if (GetStringByNames(root, commandKeys, sizeof(commandKeys) / sizeof(commandKeys[0]), &commandText)) {
        rc = HandleCommandText(commandText, root, args);
        if (rc != 0) {
            return rc;
        }
        rc = HandleCommandJsonString(commandText);
        if (rc != 0) {
            return rc;
        }
    }

    if (GetIntByNames(root, servoIdKeys, sizeof(servoIdKeys) / sizeof(servoIdKeys[0]), &channel) &&
        GetIntByNames(root, angleKeys, sizeof(angleKeys) / sizeof(angleKeys[0]), &angle)) {
        if (channel < 0 || channel >= EDOG_SERVO_CHANNEL_COUNT ||
            angle < EDOG_SERVO_MIN_ANGLE || angle > EDOG_SERVO_MAX_ANGLE) {
            return -1;
        }
        return EnqueueServoSetCommand(channel, angle);
    }

    rc = ParseServoBatchCommand(root, args, batchChannels, batchAngles, &batchCount);
    if (rc != 0) {
        if (rc < 0) {
            return rc;
        }
        return EnqueueServoBatchCommand(batchCount, batchChannels, batchAngles);
    }

    for (i = 0; i < sizeof(nestedObjectKeys) / sizeof(nestedObjectKeys[0]); i++) {
        cJSON *item = cJSON_GetObjectItem(root, nestedObjectKeys[i]);
        rc = HandleCommandValue(item, root, item);
        if (rc != 0) {
            return rc;
        }
    }

    return 0;
}

static void PublishSystemCommandResponse(const char *requestId, int resultCode, const char *resultText)
{
    char payload[EDOG_MQTT_BUFFER_LENGTH];
    char rsptopic[128] = {0};
    MQTTMessage message;
    int rc;

    snprintf(response_topic, sizeof(response_topic),
             "$oc/devices/%s/sys/commands/response", mqtt_deviceid);
    if (requestId != NULL && requestId[0] != '\0') {
        snprintf(rsptopic, sizeof(rsptopic), "%s/request_id=%s", response_topic, requestId);
    } else {
        snprintf(rsptopic, sizeof(rsptopic), "%s", response_topic);
    }

    message.qos = 0;
    message.retained = 0;
    message.payload = payload;
    snprintf(payload, sizeof(payload),
             "{ \"result_code\": %d, \"response_name\": \"COMMAND_RESPONSE\", "
             "\"paras\": { \"result\": \"%s\" } }",
             resultCode, resultText);
    message.payloadlen = strlen(payload);

    rc = MQTTPublish(&client, rsptopic, &message);
    if (rc != 0) {
        MarkMqttPublishFailure("SystemResponse", rc);
    } else {
        MarkMqttPublishSuccess();
    }
}

static void PublishPropertySetResponse(const char *requestId, int resultCode, const char *resultText)
{
    char payload[EDOG_MQTT_BUFFER_LENGTH];
    char rsptopic[128] = {0};
    MQTTMessage message;
    int rc;

    snprintf(response_topic, sizeof(response_topic),
             "$oc/devices/%s/sys/properties/set/response", mqtt_deviceid);
    if (requestId != NULL && requestId[0] != '\0') {
        snprintf(rsptopic, sizeof(rsptopic), "%s/request_id=%s", response_topic, requestId);
    } else {
        snprintf(rsptopic, sizeof(rsptopic), "%s", response_topic);
    }

    message.qos = 0;
    message.retained = 0;
    message.payload = payload;
    snprintf(payload, sizeof(payload),
             "{ \"result_code\": %d, \"result_desc\": \"%s\" }",
             resultCode, resultText);
    message.payloadlen = strlen(payload);

    rc = MQTTPublish(&client, rsptopic, &message);
    if (rc != 0) {
        MarkMqttPublishFailure("PropertySetResponse", rc);
    } else {
        MarkMqttPublishSuccess();
    }
}


// ============================================================
//  心跳发送函数
// ============================================================
void mqtt_send_heartbeat(void)
{
    if (!mqttConnectFlag) {
        printf("[heartbeat] mqtt not connected\n");
        return;
    }

    char payload[128] = {0};
    snprintf(payload, sizeof(payload),
             "{\"msgType\":\"deviceReq\",\"data\":{\"heartbeat\":\"ping\"}}");

    MQTTMessage message;
    message.qos = 0;
    message.retained = 0;
    message.payload = payload;
    message.payloadlen = strlen(payload);

    int rc = MQTTPublish(&client, heartbeat_topic, &message);

    if (rc != 0) {
        MarkMqttPublishFailure("heartbeat", rc);
    } else {
        MarkMqttPublishSuccess();
        //printf("[heartbeat] ping\n");
    }
}

// ============================================================
// [新增] 专门发送消息到 edog_publish 的辅助函数 (可选使用)
// ============================================================
void send_msg_to_edog(char *msg)
{
    if (!mqttConnectFlag) return;
    
    MQTTMessage message;
    message.qos = 0;
    message.retained = 0;
    message.payload = msg;
    message.payloadlen = strlen(msg);

    int rc = MQTTPublish(&client, EDOG_PUB_TOPIC, &message);
    if (rc != 0) {
        MarkMqttPublishFailure("edog_publish", rc);
    } else {
        MarkMqttPublishSuccess();
    }
}

void send_servo_calibration_properties(const int offsets[], int count)
{
    int rc;
    MQTTMessage message;
    char payload[EDOG_MQTT_BUFFER_LENGTH] = {0};
    char offsetsText[160] = {0};
    size_t used = 0;

    if (mqttConnectFlag == 0 || offsets == NULL || count <= 0) {
        return;
    }

    used += snprintf(offsetsText + used, sizeof(offsetsText) - used, "[");
    for (int i = 0; i < count && i < EDOG_SERVO_CHANNEL_COUNT && used < sizeof(offsetsText); i++) {
        used += snprintf(offsetsText + used,
                         sizeof(offsetsText) - used,
                         "%s%d",
                         i == 0 ? "" : ",",
                         offsets[i]);
    }
    (void)snprintf(offsetsText + used, sizeof(offsetsText) - used, "]");

    snprintf(payload,
             sizeof(payload),
             "{\"services\":[{\"service_id\":\"smartHome\",\"properties\":{\"servoOffsets\":\"%s\",\"servoCenterAngle\":%d}}]}",
             offsetsText,
             EDOG_SERVO_CENTER_ANGLE);

    message.qos = 0;
    message.retained = 0;
    message.payload = payload;
    message.payloadlen = strlen(payload);

    snprintf(publish_topic, sizeof(publish_topic),
             "$oc/devices/%s/sys/properties/report", mqtt_deviceid);
    rc = MQTTPublish(&client, publish_topic, &message);
    if (rc != 0) {
        MarkMqttPublishFailure("ServoTrim", rc);
    } else {
        MarkMqttPublishSuccess();
    }
}

void send_servo_status_properties(const int angles[], const int offsets[], int count, const char *state)
{
    int rc;
    MQTTMessage message;
    char payload[EDOG_MQTT_BUFFER_LENGTH] = {0};
    char anglesText[160] = {0};
    char offsetsText[160] = {0};
    size_t angleUsed = 0;
    size_t offsetUsed = 0;

    if (mqttConnectFlag == 0 || angles == NULL || offsets == NULL || count <= 0) {
        return;
    }

    angleUsed += snprintf(anglesText + angleUsed, sizeof(anglesText) - angleUsed, "[");
    offsetUsed += snprintf(offsetsText + offsetUsed, sizeof(offsetsText) - offsetUsed, "[");
    for (int i = 0; i < count && i < EDOG_SERVO_CHANNEL_COUNT; i++) {
        angleUsed += snprintf(anglesText + angleUsed,
                              sizeof(anglesText) - angleUsed,
                              "%s%d",
                              i == 0 ? "" : ",",
                              angles[i]);
        offsetUsed += snprintf(offsetsText + offsetUsed,
                               sizeof(offsetsText) - offsetUsed,
                               "%s%d",
                               i == 0 ? "" : ",",
                               offsets[i]);
    }
    (void)snprintf(anglesText + angleUsed, sizeof(anglesText) - angleUsed, "]");
    (void)snprintf(offsetsText + offsetUsed, sizeof(offsetsText) - offsetUsed, "]");

    snprintf(payload,
             sizeof(payload),
             "{\"services\":[{\"service_id\":\"smartHome\",\"properties\":{"
             "\"servoAngles\":\"%s\",\"servoOffsets\":\"%s\","
             "\"servoState\":\"%s\",\"servoCenterAngle\":%d}}]}",
             anglesText,
             offsetsText,
             (state == NULL || state[0] == '\0') ? "running" : state,
             EDOG_SERVO_CENTER_ANGLE);

    message.qos = 0;
    message.retained = 0;
    message.payload = payload;
    message.payloadlen = strlen(payload);

    snprintf(publish_topic, sizeof(publish_topic),
             "$oc/devices/%s/sys/properties/report", mqtt_deviceid);
    rc = MQTTPublish(&client, publish_topic, &message);
    if (rc != 0) {
        MarkMqttPublishFailure("ServoStatus", rc);
    } else {
        MarkMqttPublishSuccess();
    }
}

/***************************************************************
* 函数名称: send_msg_to_mqtt
* 说    明: 发送信息到iot (原有的系统属性上报)
***************************************************************/
void send_msg_to_mqtt(e_iot_data *iot_data)
{
    int rc;
    MQTTMessage message;
    char payload[EDOG_MQTT_BUFFER_LENGTH] = {0};
    char str[EDOG_MQTT_STRING_LENGTH] = {0};

    if (mqttConnectFlag == 0) {
        printf("mqtt not connect\n");
        return;
    }

    cJSON *root = cJSON_CreateObject();
    if (root != NULL) {
        cJSON *serv_arr = cJSON_AddArrayToObject(root, "services");
        cJSON *arr_item = cJSON_CreateObject();
        cJSON_AddStringToObject(arr_item, "service_id", "smartHome");
        cJSON *pro_obj = cJSON_CreateObject();
        cJSON_AddItemToObject(arr_item, "properties", pro_obj);

        memset(str, 0, sizeof(str));
        snprintf(str, sizeof(str), "%5.2fLux", iot_data->illumination);
        cJSON_AddStringToObject(pro_obj, "illumination", str);
        snprintf(str, sizeof(str), "%5.2f℃", iot_data->temperature);
        cJSON_AddStringToObject(pro_obj, "temperature", str);
        snprintf(str, sizeof(str), "%5.2f%%", iot_data->humidity);
        cJSON_AddStringToObject(pro_obj, "humidity", str);
        
        if (iot_data->motor_state == true) {
            cJSON_AddStringToObject(pro_obj, "motorStatus", "ON");
        } else {
            cJSON_AddStringToObject(pro_obj, "motorStatus", "OFF");
        }
        if (iot_data->light_state == true) {
            cJSON_AddStringToObject(pro_obj, "lightStatus", "ON");
        } else {
            cJSON_AddStringToObject(pro_obj, "lightStatus", "OFF");
        }
        if (iot_data->auto_state == true) {
            cJSON_AddStringToObject(pro_obj, "autoStatus", "ON");
        } else {
            cJSON_AddStringToObject(pro_obj, "autoStatus", "OFF");
        }

        cJSON_AddItemToArray(serv_arr, arr_item);

        char *palyload_str = cJSON_PrintUnformatted(root);
        if (palyload_str != NULL) {
            if (snprintf(payload, sizeof(payload), "%s", palyload_str) >= (int)sizeof(payload)) {
                printf("mqtt payload too long, drop report\n");
                payload[0] = '\0';
            }
            cJSON_free(palyload_str);
        }
        cJSON_Delete(root);
    }

    if (payload[0] == '\0') {
        return;
    }

    message.qos = 0;
    message.retained = 0;
    message.payload = payload;
    message.payloadlen = strlen(payload);

    snprintf(publish_topic, sizeof(publish_topic),
             "$oc/devices/%s/sys/properties/report", mqtt_deviceid);
    if ((rc = MQTTPublish(&client, publish_topic, &message)) != 0) {
        MarkMqttPublishFailure("PropertyReport", rc);
    } else {
        MarkMqttPublishSuccess();
    }
}

// ... (此处省略 set_light_state, set_motor_state, set_auto_state 函数，保持不变) ...
void set_light_state(cJSON *root); // 前向声明
void set_motor_state(cJSON *root); // 前向声明
void set_auto_state(cJSON *root);  // 前向声明

// ============================================================
// [新增] Edog 消息回调函数
// 接收到 edog_subscribe 的消息后，通过串口打印
// ============================================================
void mqtt_edog_message_arrived(MessageData *data)
{
    int rc;
    char payloadText[EDOG_MQTT_BUFFER_LENGTH] = {0};
    int copyLen;

    printf("[MQTT] down edog topic=%.*s len=%d qos=%d\n",
           data->topicName->lenstring.len,
           data->topicName->lenstring.data,
           data->message->payloadlen,
           data->message->qos);

    copyLen = data->message->payloadlen;
    if (copyLen >= (int)sizeof(payloadText)) {
        copyLen = (int)sizeof(payloadText) - 1;
    }
    if (copyLen > 0) {
        memcpy(payloadText, data->message->payload, (size_t)copyLen);
    }

    // 解析 JSON
    cJSON *root = cJSON_ParseWithLength(
        (const char *)data->message->payload,
        data->message->payloadlen);

    if (root == NULL) {
        rc = HandleIncomingCommandText(payloadText);
        printf("[MQTT] edog text command handled rc=%d\n", rc);
        return;
    }

    rc = HandleIncomingCommandObject(root);
    if (rc == 0) {
        printf("content: (null or unsupported type)\n");
    } else if (rc < 0) {
        printf("content: command recognized but parameters invalid\n");
    }
    printf("[MQTT] edog command handled rc=%d\n", rc);

    // 释放 JSON
    cJSON_Delete(root);
}

/***************************************************************
* 函数名称: mqtt_message_arrived
* 说    明: 接收mqtt数据
***************************************************************/
void mqtt_message_arrived(MessageData *data)
{
    char topic[160] = {0};
    char request_id[64] = {0};
    char *request_id_idx;
    cJSON *cmd;
    cJSON *paras;
    cJSON *cmd_value;
    const char *cmd_name_str = NULL;
    int servoCommandResult;
    int resultCode = 1;
    const char *resultText = "unsupported";
    bool handled = false;

    cJSON *root = cJSON_ParseWithLength(data->message->payload, data->message->payloadlen);
    if (root == NULL) {
        printf("[Error] JSON Parse Failed\n");
        return;
    }

    snprintf(topic, sizeof(topic), "%.*s",
             data->topicName->lenstring.len, data->topicName->lenstring.data);
    printf("[MQTT] down system topic=%s len=%d qos=%d\n",
           topic,
           data->message->payloadlen,
           data->message->qos);
    request_id_idx = strstr(topic, "request_id=");
    if (request_id_idx != NULL) {
        snprintf(request_id, sizeof(request_id), "%s", request_id_idx + 11);
    }

    cmd = cJSON_GetObjectItem(root, "command_name");
    paras = cJSON_GetObjectItem(root, "paras");
    if (cJSON_IsString(cmd)) {
        cmd_name_str = cJSON_GetStringValue(cmd);
        printf("[MQTT] command=%s\n", cmd_name_str);
        servoCommandResult = HandleCommandText(cmd_name_str, root, paras);
        if (servoCommandResult != 0) {
            handled = servoCommandResult > 0;
        }
    }

    cmd_value = paras ? cJSON_GetObjectItem(paras, "cmd_value") : NULL;
    if (!handled) {
        servoCommandResult = HandleCommandValue(cmd_value, root, paras);
        if (servoCommandResult != 0) {
            handled = servoCommandResult > 0;
        }
    }

    if (handled) {
        resultCode = 0;
        resultText = "success";
    } else if (cmd_name_str != NULL || cmd_value != NULL) {
        resultText = "invalid_command_or_params";
    }

    PublishSystemCommandResponse(request_id, resultCode, resultText);
    cJSON_Delete(root);
}

void mqtt_property_set_arrived(MessageData *data)
{
    char topic[160] = {0};
    char request_id[64] = {0};
    char *request_id_idx;
    int rc;
    int resultCode = 1;
    const char *resultText = "unsupported";

    cJSON *root = cJSON_ParseWithLength(data->message->payload, data->message->payloadlen);
    if (root == NULL) {
        printf("[MQTT] property JSON Parse Failed\n");
        return;
    }

    snprintf(topic, sizeof(topic), "%.*s",
             data->topicName->lenstring.len, data->topicName->lenstring.data);
    printf("[MQTT] down property topic=%s len=%d qos=%d\n",
           topic,
           data->message->payloadlen,
           data->message->qos);
    request_id_idx = strstr(topic, "request_id=");
    if (request_id_idx != NULL) {
        snprintf(request_id, sizeof(request_id), "%s", request_id_idx + 11);
    }

    rc = HandleIncomingCommandObject(root);
    if (rc > 0) {
        resultCode = 0;
        resultText = "success";
    } else if (rc < 0) {
        resultText = "invalid_command_or_params";
    }

    PublishPropertySetResponse(request_id, resultCode, resultText);
    cJSON_Delete(root);
}

/***************************************************************
* 函数名称: wait_message
* 说    明: 等待信息
***************************************************************/
int wait_message(int timeoutMs)
{
    int rec;

    if (!mqttConnectFlag) {
        return 0;
    }

    rec = MQTTYield(&client, timeoutMs);
    if (rec != 0) {
        mqttYieldFailureCount++;
        printf("[MQTT] yield failed rc=%d consecutive=%u\n", rec, mqttYieldFailureCount);
        if (mqttYieldFailureCount >= EDOG_MQTT_YIELD_FAILURE_RECONNECT_THRESHOLD) {
            mqttConnectFlag = 0;
        }
    } else {
        mqttYieldFailureCount = 0;
    }
    return mqttConnectFlag;
}

// ============================================================
//  MQTT 初始化 + 启动心跳任务
// ============================================================
int mqtt_init(void)
{
    int rc;

    mqtt_disconnect();
    NetworkInit(&network);
    printf("[MQTT] TCP连接开始\n");
    rc = NetworkConnect(&network, EDOG_MQTT_HOST_ADDR, 1883);
    if (rc != 0) {
        printf("[MQTT] TCP连接失败 rc=%d\n", rc);
        mqtt_disconnect();
        return rc;
    }
    printf("[MQTT] TCP连接成功\n");
    mqttNetworkReady = 1;

    MQTTClientInit(&client, &network, EDOG_MQTT_COMMAND_TIMEOUT_MS,
                   sendBuf, sizeof(sendBuf),
                   readBuf, sizeof(readBuf));

    MQTTString clientId = MQTTString_initializer;
    clientId.cstring = mqtt_clientid;

    MQTTString userName = MQTTString_initializer;
    userName.cstring = mqtt_username;

    MQTTString password = MQTTString_initializer;
    password.cstring = mqtt_pwd;

    MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
    data.clientID = clientId;
    data.username = userName;
    data.password = password;
    /*
     * Huawei IoTDA requires the MQTT heartbeat interval to be within the
     * platform's accepted range. Use a long keepalive here to satisfy the
     * broker while still relying on the app-level 30-second heartbeat.
     */
    data.keepAliveInterval = EDOG_MQTT_KEEPALIVE_SECONDS;
    data.MQTTVersion = 4;
    data.cleansession = 1;

    PrintMqttConnectDiagnostics();
    rc = MQTTConnect(&client, &data);
    if (rc != 0) {
        printf("[MQTT] 登录失败 rc=%d\n", rc);
        mqtt_disconnect();
        return rc;
    }
    printf("[MQTT] 登录成功\n");

    snprintf(subcribe_topic, sizeof(subcribe_topic),
             "$oc/devices/%s/sys/commands/#", mqtt_deviceid);
    rc = MQTTSubscribe(&client, subcribe_topic, 0, mqtt_message_arrived);
    if (rc != 0) {
        printf("[MQTT] 系统命令订阅失败 rc=%d\n", rc);
        mqtt_disconnect();
        return rc;
    }
    printf("[MQTT] 订阅成功 system\n");

    snprintf(property_set_topic, sizeof(property_set_topic),
             "$oc/devices/%s/sys/properties/set/#", mqtt_deviceid);
    rc = MQTTSubscribe(&client, property_set_topic, 0, mqtt_property_set_arrived);
    if (rc != 0) {
        printf("[MQTT] 属性设置订阅失败 rc=%d\n", rc);
        mqtt_disconnect();
        return rc;
    }
    printf("[MQTT] 订阅成功 property\n");

    rc = MQTTSubscribe(&client, EDOG_SUB_TOPIC, 0, mqtt_edog_message_arrived);
    if (rc != 0) {
        printf("[MQTT] 自定义命令订阅失败 rc=%d\n", rc);
        mqtt_disconnect();
        return rc;
    }
    printf("[MQTT] 订阅成功 edog\n");

    snprintf(heartbeat_topic, sizeof(heartbeat_topic),
             "$oc/devices/%s/sys/messages/up", mqtt_deviceid);

    mqttConnectFlag = 1;
    mqttYieldFailureCount = 0;
    mqttPublishFailureCount = 0;
    printf("[MQTT] connected\n");
    return 0;
}

void mqtt_disconnect(void)
{
    if (mqttConnectFlag) {
        (void)MQTTDisconnect(&client);
    }
    if (mqttNetworkReady) {
        NetworkDisconnect(&network);
        mqttNetworkReady = 0;
    }
    mqttConnectFlag = 0;
    mqttYieldFailureCount = 0;
    mqttPublishFailureCount = 0;
}

unsigned int mqtt_is_connected()
{
    return mqttConnectFlag;
}
