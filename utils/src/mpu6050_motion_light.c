#include "mpu6050_motion_light.h"
#include "i2c_bus_guard.h"
#include "../../include/edog_config.h"
#include "iot_errno.h"
#include "iot_gpio.h"
#include "iot_i2c.h"
#include "los_task.h"
#include <stdint.h>
#include <stdio.h>

#define MPU6050_I2C_PORT       EI2C0_M2
#define MPU6050_I2C_ADDRESS    0x68

#define MPU6050_REG_SMPLRT_DIV     0x19
#define MPU6050_REG_CONFIG         0x1A
#define MPU6050_REG_GYRO_CONFIG    0x1B
#define MPU6050_REG_ACCEL_CONFIG   0x1C
#define MPU6050_REG_ACCEL_XOUT_H   0x3B
#define MPU6050_REG_GYRO_XOUT_H    0x43
#define MPU6050_REG_PWR_MGMT_1     0x6B
#define MPU6050_REG_WHO_AM_I       0x75

#define MPU6050_ACCEL_LSB_PER_G    16384
#define MPU6050_GYRO_LSB_PER_DPS   131
#define MPU6050_AXIS_COUNT         3

typedef struct {
    unsigned char r;
    unsigned char g;
    unsigned char b;
} RgbColor;

static const RgbColor g_rainbow[] = {
    {1, 0, 0},
    {0, 1, 0},
    {0, 0, 1},
};

static int g_mpuReady = 0;
static int g_rgbReady = 0;
static int g_baselineMg[MPU6050_AXIS_COUNT] = {0};
static int g_levelBaselineMg[MPU6050_AXIS_COUNT] = {0};
static unsigned int g_rgbPhase = 0;

static int Mpu6050_WriteReg(uint8_t reg, uint8_t value)
{
    unsigned int ret;
    uint8_t data[2] = {reg, value};

    if (EdogI2cBusGuard_Init() != 0) {
        return -1;
    }

    EdogI2cBusGuard_Lock();
    ret = IoTI2cWrite(MPU6050_I2C_PORT, MPU6050_I2C_ADDRESS, data, sizeof(data));
    EdogI2cBusGuard_Unlock();

    if (ret != IOT_SUCCESS) {
        printf("[MPU6050] write reg 0x%02x failed: %u\n", reg, ret);
        return -1;
    }

    return 0;
}

static int Mpu6050_ReadRegs(uint8_t reg, uint8_t *data, unsigned int len)
{
    unsigned int ret;

    if (data == NULL || len == 0 || EdogI2cBusGuard_Init() != 0) {
        return -1;
    }

    EdogI2cBusGuard_Lock();
    ret = IoTI2cWrite(MPU6050_I2C_PORT, MPU6050_I2C_ADDRESS, &reg, 1);
    if (ret == IOT_SUCCESS) {
        ret = IoTI2cRead(MPU6050_I2C_PORT, MPU6050_I2C_ADDRESS, data, len);
    }
    EdogI2cBusGuard_Unlock();

    if (ret != IOT_SUCCESS) {
        printf("[MPU6050] read reg 0x%02x len=%u failed: %u\n", reg, len, ret);
        return -1;
    }

    return 0;
}

static int Mpu6050_InitBus(void)
{
    if (EdogI2cBusGuard_EnsureBusInit() != 0) {
        printf("[MPU6050] I2C bus init failed\n");
        return -1;
    }
    return 0;
}

static int Mpu6050_CheckIdentity(void)
{
    uint8_t whoAmI = 0;

    if (Mpu6050_ReadRegs(MPU6050_REG_WHO_AM_I, &whoAmI, 1) != 0) {
        return -1;
    }

    if (whoAmI != MPU6050_I2C_ADDRESS) {
        printf("[MPU6050] unexpected WHO_AM_I=0x%02x\n", whoAmI);
        return -1;
    }

    return 0;
}

static void Rgb_Init(void)
{
    if (g_rgbReady) {
        return;
    }

    IoTGpioInit(EDOG_RGB_LED_R_GPIO);
    IoTGpioInit(EDOG_RGB_LED_G_GPIO);
    IoTGpioInit(EDOG_RGB_LED_B_GPIO);
    IoTGpioSetDir(EDOG_RGB_LED_R_GPIO, IOT_GPIO_DIR_OUT);
    IoTGpioSetDir(EDOG_RGB_LED_G_GPIO, IOT_GPIO_DIR_OUT);
    IoTGpioSetDir(EDOG_RGB_LED_B_GPIO, IOT_GPIO_DIR_OUT);
    g_rgbReady = 1;
}

static void Rgb_SetColor(const RgbColor *color)
{
    IotGpioValue onValue = EDOG_RGB_ACTIVE_HIGH ? IOT_GPIO_VALUE1 : IOT_GPIO_VALUE0;
    IotGpioValue offValue = EDOG_RGB_ACTIVE_HIGH ? IOT_GPIO_VALUE0 : IOT_GPIO_VALUE1;

    if (color == NULL) {
        return;
    }

    Rgb_Init();
    IoTGpioSetOutputVal(EDOG_RGB_LED_R_GPIO, color->r ? onValue : offValue);
    IoTGpioSetOutputVal(EDOG_RGB_LED_G_GPIO, color->g ? onValue : offValue);
    IoTGpioSetOutputVal(EDOG_RGB_LED_B_GPIO, color->b ? onValue : offValue);
}

static void Rgb_Off(void)
{
    static const RgbColor off = {0, 0, 0};
    Rgb_SetColor(&off);
}

static void Rgb_ShowNextRainbowColor(void)
{
    Rgb_SetColor(&g_rainbow[g_rgbPhase]);
    g_rgbPhase = (g_rgbPhase + 1) % (sizeof(g_rainbow) / sizeof(g_rainbow[0]));
}

static void Rgb_ShowTiltState(void)
{
    static const RgbColor red = {1, 0, 0};
    Rgb_SetColor(&red);
}

static void Rgb_ShowBalanceState(void)
{
    static const RgbColor blue = {0, 0, 1};
    Rgb_SetColor(&blue);
}

int MpuMotionLight_ReadAccelMg(int *xMg, int *yMg, int *zMg)
{
    uint8_t data[6] = {0};
    int16_t rawX;
    int16_t rawY;
    int16_t rawZ;

    if (xMg == NULL || yMg == NULL || zMg == NULL) {
        return -1;
    }

    if (Mpu6050_ReadRegs(MPU6050_REG_ACCEL_XOUT_H, data, sizeof(data)) != 0) {
        return -1;
    }

    rawX = (int16_t)((data[0] << 8) | data[1]);
    rawY = (int16_t)((data[2] << 8) | data[3]);
    rawZ = (int16_t)((data[4] << 8) | data[5]);

    *xMg = rawX * 1000 / MPU6050_ACCEL_LSB_PER_G;
    *yMg = rawY * 1000 / MPU6050_ACCEL_LSB_PER_G;
    *zMg = rawZ * 1000 / MPU6050_ACCEL_LSB_PER_G;
    return 0;
}

int MpuMotionLight_ReadGyroCentiDps(int *xCentiDps, int *yCentiDps, int *zCentiDps)
{
    uint8_t data[6] = {0};
    int16_t rawX;
    int16_t rawY;
    int16_t rawZ;

    if (xCentiDps == NULL || yCentiDps == NULL || zCentiDps == NULL) {
        return -1;
    }

    if (Mpu6050_ReadRegs(MPU6050_REG_GYRO_XOUT_H, data, sizeof(data)) != 0) {
        return -1;
    }

    rawX = (int16_t)((data[0] << 8) | data[1]);
    rawY = (int16_t)((data[2] << 8) | data[3]);
    rawZ = (int16_t)((data[4] << 8) | data[5]);

    *xCentiDps = rawX * 100 / MPU6050_GYRO_LSB_PER_DPS;
    *yCentiDps = rawY * 100 / MPU6050_GYRO_LSB_PER_DPS;
    *zCentiDps = rawZ * 100 / MPU6050_GYRO_LSB_PER_DPS;
    return 0;
}

int MpuMotionLight_ReadMotion(int *xMg, int *yMg, int *zMg,
                              int *gyroXCentiDps, int *gyroYCentiDps,
                              int *gyroZCentiDps)
{
    if (MpuMotionLight_ReadAccelMg(xMg, yMg, zMg) != 0) {
        return -1;
    }
    return MpuMotionLight_ReadGyroCentiDps(gyroXCentiDps, gyroYCentiDps, gyroZCentiDps);
}

int MpuMotionLight_Init(void)
{
    if (g_mpuReady) {
        return 0;
    }

    Rgb_Init();
    Rgb_Off();

    if (Mpu6050_InitBus() != 0) {
        return -1;
    }

    if (Mpu6050_CheckIdentity() != 0) {
        return -1;
    }

    if (Mpu6050_WriteReg(MPU6050_REG_PWR_MGMT_1, 0x00) != 0) {
        return -1;
    }
    LOS_Msleep(100);

    if (Mpu6050_WriteReg(MPU6050_REG_SMPLRT_DIV, 0x00) != 0 ||
        Mpu6050_WriteReg(MPU6050_REG_CONFIG, 0x03) != 0 ||
        Mpu6050_WriteReg(MPU6050_REG_GYRO_CONFIG, 0x00) != 0 ||
        Mpu6050_WriteReg(MPU6050_REG_ACCEL_CONFIG, 0x00) != 0) {
        return -1;
    }

    g_mpuReady = 1;
    return 0;
}

static int MpuMotionLight_CalibrateBaseline(void)
{
    int x;
    int y;
    int z;
    int sum[MPU6050_AXIS_COUNT] = {0};

    for (int i = 0; i < EDOG_MPU_BASELINE_SAMPLES; i++) {
        if (MpuMotionLight_ReadAccelMg(&x, &y, &z) != 0) {
            return -1;
        }
        sum[0] += x;
        sum[1] += y;
        sum[2] += z;
        LOS_Msleep(EDOG_MPU_SAMPLE_INTERVAL_MS);
    }

    g_baselineMg[0] = sum[0] / EDOG_MPU_BASELINE_SAMPLES;
    g_baselineMg[1] = sum[1] / EDOG_MPU_BASELINE_SAMPLES;
    g_baselineMg[2] = sum[2] / EDOG_MPU_BASELINE_SAMPLES;
    g_levelBaselineMg[0] = g_baselineMg[0];
    g_levelBaselineMg[1] = g_baselineMg[1];
    g_levelBaselineMg[2] = g_baselineMg[2];
    return 0;
}

static int SelectForwardAxisMg(int xMg, int yMg, int zMg)
{
    switch (EDOG_MPU_FORWARD_AXIS) {
        case 1:
            return yMg;
        case 2:
            return zMg;
        default:
            return xMg;
    }
}

static int SelectForwardBaselineMg(void)
{
    if (EDOG_MPU_FORWARD_AXIS == 1) {
        return g_baselineMg[1];
    }
    if (EDOG_MPU_FORWARD_AXIS == 2) {
        return g_baselineMg[2];
    }
    return g_baselineMg[0];
}

static int IsForwardMotionDetected(int xMg, int yMg, int zMg)
{
    int forwardMg = SelectForwardAxisMg(xMg, yMg, zMg);
    int deltaMg = (forwardMg - SelectForwardBaselineMg()) * EDOG_MPU_FORWARD_DIRECTION;

    return deltaMg >= EDOG_MPU_MOTION_THRESHOLD_MG;
}

static int IsForwardMotionReleased(int xMg, int yMg, int zMg)
{
    int forwardMg = SelectForwardAxisMg(xMg, yMg, zMg);
    int deltaMg = (forwardMg - SelectForwardBaselineMg()) * EDOG_MPU_FORWARD_DIRECTION;

    return deltaMg <= EDOG_MPU_MOTION_RELEASE_MG;
}

static int IsBodyTilted(int xMg, int yMg, int zMg)
{
    int deltaX = xMg - g_levelBaselineMg[0];
    int deltaY = yMg - g_levelBaselineMg[1];
    int deltaZ = zMg - g_levelBaselineMg[2];

    if (deltaX < 0) {
        deltaX = -deltaX;
    }
    if (deltaY < 0) {
        deltaY = -deltaY;
    }
    if (deltaZ < 0) {
        deltaZ = -deltaZ;
    }

    return deltaX >= EDOG_MPU_TILT_THRESHOLD_MG ||
           deltaY >= EDOG_MPU_TILT_THRESHOLD_MG ||
           deltaZ >= EDOG_MPU_TILT_THRESHOLD_MG;
}

static void UpdateBaselineWhenQuiet(int xMg, int yMg, int zMg)
{
    g_baselineMg[0] = (g_baselineMg[0] * 31 + xMg) / 32;
    g_baselineMg[1] = (g_baselineMg[1] * 31 + yMg) / 32;
    g_baselineMg[2] = (g_baselineMg[2] * 31 + zMg) / 32;
}

void MpuMotionLightTask(void)
{
    unsigned int activeLoops = 0;
    unsigned int rotateElapsedMs = EDOG_RGB_ROTATE_INTERVAL_MS;
    int forwardMotionArmed = 1;

    while (MpuMotionLight_Init() != 0 || MpuMotionLight_CalibrateBaseline() != 0) {
        printf("[MPU6050] motion RGB task unavailable, RGB off, retry later\n");
        Rgb_Off();
        LOS_Msleep(EDOG_MPU_INIT_RETRY_MS);
    }

    while (1) {
        int x;
        int y;
        int z;

        if (MpuMotionLight_ReadAccelMg(&x, &y, &z) == 0) {
            if (forwardMotionArmed && IsForwardMotionDetected(x, y, z)) {
                activeLoops = (EDOG_RGB_ACTIVE_HOLD_MS + EDOG_MPU_SAMPLE_INTERVAL_MS - 1) /
                              EDOG_MPU_SAMPLE_INTERVAL_MS;
                forwardMotionArmed = 0;
            } else if (!forwardMotionArmed && IsForwardMotionReleased(x, y, z)) {
                forwardMotionArmed = 1;
            }

            if (activeLoops > 0) {
                rotateElapsedMs += EDOG_MPU_SAMPLE_INTERVAL_MS;
                if (rotateElapsedMs >= EDOG_RGB_ROTATE_INTERVAL_MS) {
                    Rgb_ShowNextRainbowColor();
                    rotateElapsedMs = 0;
                }
                activeLoops--;
            } else {
                rotateElapsedMs = EDOG_RGB_ROTATE_INTERVAL_MS;
                if (IsBodyTilted(x, y, z)) {
                    Rgb_ShowTiltState();
                } else {
                    Rgb_ShowBalanceState();
                }
                UpdateBaselineWhenQuiet(x, y, z);
            }
        } else {
            activeLoops = 0;
            forwardMotionArmed = 1;
            rotateElapsedMs = EDOG_RGB_ROTATE_INTERVAL_MS;
            Rgb_Off();
        }

        LOS_Msleep(EDOG_MPU_SAMPLE_INTERVAL_MS);
    }
}
