#ifndef MPU6050_MOTION_LIGHT_H
#define MPU6050_MOTION_LIGHT_H

#ifdef __cplusplus
extern "C" {
#endif

void MpuMotionLightTask(void);
int MpuMotionLight_Init(void);
int MpuMotionLight_ReadAccelMg(int *xMg, int *yMg, int *zMg);
int MpuMotionLight_ReadGyroCentiDps(int *xCentiDps, int *yCentiDps, int *zCentiDps);
int MpuMotionLight_ReadMotion(int *xMg, int *yMg, int *zMg,
                              int *gyroXCentiDps, int *gyroYCentiDps,
                              int *gyroZCentiDps);

#ifdef __cplusplus
}
#endif

#endif
