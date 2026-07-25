#ifndef GAIT_GENERATE_12DOF_H
#define GAIT_GENERATE_12DOF_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* EDOG measured mechanical constants, unit: mm. */
#define EDOG_12DOF_L_BODY_MM                   190.0
#define EDOG_12DOF_W_BODY_MM                   87.0
#define EDOG_12DOF_L1_MM                       50.0
#define EDOG_12DOF_L2_MM                       107.0
#define EDOG_12DOF_L3_MM                       135.0

/* SpotMicro 官方开源结构常用连杆尺寸，单位为 mm。 */
#define SPOTMICRO_THIGH_LENGTH_MM              107.0
#define SPOTMICRO_CALF_LENGTH_MM               135.0
#define SPOTMICRO_PRECISE_THIGH_LENGTH_MM      111.2
#define SPOTMICRO_PRECISE_CALF_LENGTH_MM       118.5

/*
 * 是否使用更精确的 SpotMicro 连杆长度。
 * 0: 使用 120.0mm / 120.0mm，便于机械加工和首轮调试。
 * 1: 使用 111.2mm / 118.5mm，适合完全按精确模型建模的腿。
 */
#define EDOG_12DOF_USE_PRECISE_SPOTMICRO_LINKS 0

/*
 * 12DOF 默认站立姿态，单位是运动学关节增量角。
 * Hip   = 0   : 髋关节不外展，左右保持居中。
 * Thigh = -30 : 大腿向后摆。运动学定义里“大腿向前摆”为正，所以向后为负。
 * Calf  = 45  : 小腿相对大腿向后折叠 45 度。
 */
/*
 * 低重心反 Z 站立姿态：
 * Hip=0 让胯骨关节保持中位，减少站立时左右胯额外预紧；
 * Thigh=-61 / Calf=20 对应默认足端 (-150, 0, 160) 附近的 IK 高站姿。
 */
#define EDOG_12DOF_STAND_HIP_DELTA_DEG 0
#define EDOG_12DOF_STAND_THIGH_DELTA_DEG -61
#define EDOG_12DOF_STAND_CALF_DELTA_DEG 20
#define EDOG_12DOF_FRONT_HIP_ADDUCTION_DEFAULT_DEG -13.0
#define EDOG_12DOF_REAR_HIP_ADDUCTION_DEFAULT_DEG -8.0

/*
 * 步态目标表使用 30 个采样点，运行时按每条腿的相位读取表。
 * 旧的 EDOG_12DOF_TROT_FRAME_COUNT 保留为兼容别名。
 */
#define EDOG_12DOF_GAIT_FRAME_COUNT 30
#define EDOG_12DOF_TROT_FRAME_COUNT EDOG_12DOF_GAIT_FRAME_COUNT
#define EDOG_12DOF_8DOF_REFERENCE_KEY_FRAME_COUNT 20

/*
 * 关节角同时保留“整数度”和“厘度(centi-degree, 1/100 度)”两种表示：
 *   - *AngleDeg   : 整数度，向后兼容所有偏置/站姿/调试逻辑，取值四舍五入。
 *   - *AngleCentiDeg : 厘度定点，保留 IK 原始亚度精度。
 * 运动输出层（slew 滤波 + PCA9685 计数）优先使用厘度，消除支撑相阶梯抖动。
 * 写厘度时务必同步整数度，反之亦然，二者必须始终自洽。
 */
typedef struct {
    int hipAngleDeg;    /* 胯部关节期望增量角：向外张开为正 */
    int femurAngleDeg;  /* 大腿关节期望增量角：向前摆动为正 */
    int tibiaAngleDeg;  /* 小腿关节期望增量角：向后折叠为正 */
    int hipAngleCentiDeg;   /* 胯部关节增量角，单位 1/100 度 */
    int femurAngleCentiDeg; /* 大腿关节增量角，单位 1/100 度 */
    int tibiaAngleCentiDeg; /* 小腿关节增量角，单位 1/100 度 */
} Edog12DofJointAngles;

/* 度 <-> 厘度换算与同步辅助，供生成层和运动层共用。 */
#define EDOG_12DOF_CENTI_PER_DEG 100

/* 由整数度字段反填厘度字段（用于站姿常量、兼容旧表）。 */
static inline void Edog12Dof_SyncCentiFromDeg(Edog12DofJointAngles *a)
{
    if (a == NULL) {
        return;
    }
    a->hipAngleCentiDeg = a->hipAngleDeg * EDOG_12DOF_CENTI_PER_DEG;
    a->femurAngleCentiDeg = a->femurAngleDeg * EDOG_12DOF_CENTI_PER_DEG;
    a->tibiaAngleCentiDeg = a->tibiaAngleDeg * EDOG_12DOF_CENTI_PER_DEG;
}

typedef struct {
    double xMm; /* 足端前后方向，向前为正 */
    double yMm; /* 足端左右方向，机身左侧为正 */
    double zMm; /* 足端垂直方向，向下为正 */
} Edog12DofFootPoint;

/**
 * @brief SpotMicro 12DOF 单腿逆运动学。
 * @param foot 足端目标点，单位 mm
 * @param isRightLeg 右腿传 1，左腿传 0，用于镜像胯部方向
 * @param outAngles 输出 3 个关节运动学增量角
 * @return 0=成功，-1=输入无效
 */
int Edog12Dof_IK(const Edog12DofFootPoint *foot, int isRightLeg,
                 Edog12DofJointAngles *outAngles);

void Edog12Dof_ResetRuntimeGeometry(void);
void Edog12Dof_SetRuntimeGeometry(double hipAdductionDeg,
                                  double thighLengthMm,
                                  double calfLengthMm);
void Edog12Dof_SetRuntimeGeometryForFrontRear(double frontHipAdductionDeg,
                                              double rearHipAdductionDeg,
                                              double thighLengthMm,
                                              double calfLengthMm);
void Edog12Dof_GetRuntimeGeometry(double *hipAdductionDeg,
                                  double *thighLengthMm,
                                  double *calfLengthMm);
void Edog12Dof_GetRuntimeGeometryForFrontRear(double *frontHipAdductionDeg,
                                              double *rearHipAdductionDeg,
                                              double *thighLengthMm,
                                              double *calfLengthMm);
void Edog12Dof_SetRuntimeFootZDeltas(double frontFootZDeltaMm,
                                     double rearFootZDeltaMm);
void Edog12Dof_GetRuntimeFootZDeltas(double *frontFootZDeltaMm,
                                     double *rearFootZDeltaMm);
void Edog12Dof_SetLinkLengthsM(double thighLengthM, double calfLengthM);
double Edog12Dof_GetThighLengthMm(void);
double Edog12Dof_GetCalfLengthMm(void);
double Edog12Dof_DefaultFootXForLeg(int isFrontLeg);
double Edog12Dof_DefaultFootZForLeg(int isFrontLeg);
double Edog12Dof_FootYOnHipPlane(int isRightLeg, double zMm);
double Edog12Dof_FootYOnHipPlaneForLeg(int legIndex, double zMm);
double Edog12Dof_DefaultFootYForLeg(int legIndex);
double Edog12Dof_HipAdductionDegForLeg(int legIndex);
double Edog12Dof_ReferencePhaseForFrame(int frameIndex, int reversed);
double Edog12Dof_ReferenceSwingEnvelopeForPhase(double phase);
Edog12DofFootPoint Edog12Dof_SampleReferenceTrotFootPoint(
    double forwardStepM, double sideStepM, double yawStepM, double stepHeightM,
    int isRightLeg, int isFrontLeg, int legIndex, int reversed,
    int frameIndex, int *isSwingOut);

/**
 * @brief 生成 30 点 trot 步态表，每个点包含 3 个关节角。
 */
void Edog12Dof_GenerateTrotTable(Edog12DofJointAngles gaitTable[EDOG_12DOF_TROT_FRAME_COUNT],
                                 double stepLengthM, double stepHeightM,
                                 int isRightLeg, int reversed);

/**
 * @brief Generate a foot-space gait from forward/side/yaw commands.
 *
 * Positive side commands open left legs and close right legs. Positive yaw
 * opens front-left/back-right and closes front-right/back-left, so callers can
 * keep simple high-level motion names while the gait layer handles per-leg
 * touchdown placement.
 */
void Edog12Dof_GenerateDirectionalTrotTable(
    Edog12DofJointAngles gaitTable[EDOG_12DOF_TROT_FRAME_COUNT],
    double forwardStepM, double sideStepM, double yawStepM, double stepHeightM,
    int isRightLeg, int isFrontLeg, int reversed);

#ifdef __cplusplus
}
#endif

#endif /* GAIT_GENERATE_12DOF_H */
