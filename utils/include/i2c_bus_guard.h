#ifndef I2C_BUS_GUARD_H
#define I2C_BUS_GUARD_H

#ifdef __cplusplus
extern "C" {
#endif

#define EDOG_I2C_BUS_FREQUENCY EI2C_FRE_400K

int EdogI2cBusGuard_Init(void);
void EdogI2cBusGuard_Lock(void);
void EdogI2cBusGuard_Unlock(void);

/*
 * 全局 I2C 总线初始化，只调一次 IoTI2cInit。
 * 多个设备（PCA9685/MPU6050）共享 EI2C0_M2，重复调用 IoTI2cInit
 * 会因 GPIO 已初始化而报错。此函数用全局标志保证只初始化一次。
 * 返回 0=成功（含已初始化），-1=失败。
 */
int EdogI2cBusGuard_EnsureBusInit(void);

#ifdef __cplusplus
}
#endif

#endif
