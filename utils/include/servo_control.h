#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 PCA9685 模块
 * 只需调用一次（内部带防重复保护）
 */
int initPCA9685(void);

/**
 * @brief 设置指定通道的舵机角度
 * @param channel PCA9685 通道号 (0~15)
 * @param angle 180 度舵机物理角度 (0° ~ 180°)，90° 为机械中位
 */
int setServo(int channel, int angle);

/**
 * @brief 以厘度(1/100 度)精度设置舵机角度，保留亚度分辨率。
 *
 * 与 setServo 共用同一线性脉宽映射，但角度以 ×100 定点传入，
 * 避免整数度量化造成的支撑相阶梯抖动。整数度输入
 * (centiDeg = angle * 100) 时输出与 setServo 逐值一致。
 *
 * @param channel PCA9685 通道号 (0~15)
 * @param centiDeg 物理角度 ×100，例如 9050 表示 90.50°
 */
int setServoCentiDeg(int channel, int centiDeg);

/**
 * @brief 直接设置指定通道的舵机脉宽，供机械限位和校准使用
 * @param channel PCA9685 通道号 (0~15)
 * @param pulseUs 舵机脉宽，默认限制在 500us~2500us
 */
int setServoPulseUs(int channel, int pulseUs);

/**
 * @brief 所有舵机回到机械中位（默认 90°）
 */
int setZero(void);

#ifdef __cplusplus
}
#endif

#endif // SERVO_CONTROL_H
