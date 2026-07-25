#ifndef MOTION_UTILS_12DOF_H
#define MOTION_UTILS_12DOF_H

#include <stdint.h>

/*
 * 12DOF 舵机通道定义：PCA9685 每 4 个通道为一条腿，最后 1 路预留。
 *
 * 第一组 0~3：左前腿 LF
 * 第二组 4~7：右前腿 RF
 * 第三组 8~11：左后腿 LB
 * 第四组 12~15：右后腿 RB
 */
#define LF_HIP       0
#define LF_THIGH     1
#define LF_CALF      2
#define LF_RESERVED  3

#define RF_HIP       4
#define RF_THIGH     5
#define RF_CALF      6
#define RF_RESERVED  7

#define LB_HIP       8
#define LB_THIGH     9
#define LB_CALF      10
#define LB_RESERVED  11

#define RB_HIP       12
#define RB_THIGH     13
#define RB_CALF      14
#define RB_RESERVED  15

#define EDOG_12DOF_LEG_COUNT 4
#define EDOG_12DOF_JOINTS_PER_LEG 3
#define EDOG_12DOF_ACTIVE_SERVO_COUNT 12

void setSpeedLevel(int level);
int getSpeedLevel(void);

/**
 * @brief 设置指定通道的舵机角度，并同步内部姿态记录。
 * @param channel PCA9685 通道号 (0~15)
 * @param angle 180 度舵机物理角度 (0~180)
 */
int setDogServoAngleTracked(int channel, int angle);
int setDogServoPhysicalAngleTracked(int channel, int angle);

/*
 * 设置单个舵机的机械中位偏移。
 * 后续站立、行走等运动输出会自动叠加该偏移；手动舵机调角使用真实物理角直出。
 */
int setDogServoCenterTrim(int channel, int trimDeg);
int clearDogServoCenterTrimsOnceForRealAngleCalibration(void);
int loadDogServoCenterTrims(void);
int saveDogServoCenterTrim(int channel, int trimDeg);
int getDogServoCenterTrim(int channel);
int getDogServoTrackedAngle(int channel);
int getDogServoTrackedAngles(int angles[], int count);
void setRuntimeGaitGeometry(double hipAdductionDeg, double thighLengthM, double calfLengthM);
void setRuntimeGaitGeometryForFrontRear(double frontHipAdductionDeg,
                                        double rearHipAdductionDeg,
                                        double thighLengthM,
                                        double calfLengthM);
int setImuBalanceStrengthPercent(int strengthPercent);
int getImuBalanceStrengthPercent(void);
int saveRuntimeTuningToKv(void);
int loadRuntimeTuningFromKv(void);

/**
 * @brief 12 个活动腿部舵机进入机械中位校准姿态。
 */
int setDogLegsStraightPose(void);

void init_dog(double step_length, double step_height);
int smooth_stop_to_stand(double step_length, double step_height);
int balance_stand_frame(void);
int trot_cycle(double step_length, double step_height);
int crawl_cycle(double step_length, double step_height);
int trot_in_place_cycle(double step_height);
int trot_back_cycle(double step_length, double step_height);
int diversion_right_cycle(double step_length, double step_height);
int diversion_left_cycle(double step_length, double step_height);
int trot_left_front_cycle(double step_length, double step_height);
int trot_right_front_cycle(double step_length, double step_height);
int trot_left_back_cycle(double step_length, double step_height);
int trot_right_back_cycle(double step_length, double step_height);
/*
 * 腿部调试入口。
 * bit0=LF 左前, bit1=RF 右前, bit2=LB 左后, bit3=RB 右后。
 */
int single_leg_gait_cycle(int leg_index, double step_length, double step_height);
int leg_group_gait_cycle(int leg_mask, double step_length, double step_height);

void stopCurrentMotion(void);
void resetStopFlag(void);
int isStopFlag(void);

#endif /* MOTION_UTILS_12DOF_H */
