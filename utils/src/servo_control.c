#include "iot_i2c.h"
#include "iot_errno.h"
#include "i2c_bus_guard.h"
#include "../../include/edog_config.h"
#include <unistd.h>
#include <stdio.h>
#include <stdint.h>

#define PCA9685_I2C_PORT   EI2C0_M2
#define PCA9685_ADDRESS    0x40

#define MODE1_REG          0x00
#define MODE2_REG          0x01
#define LED0_ON_L          0x06
#define PRE_SCALE          0xFE

static int g_pca9685Initialized = 0;
static uint16_t g_pwmOffCount[EDOG_SERVO_CHANNEL_COUNT] = {0};
static unsigned char g_pwmOffCountValid[EDOG_SERVO_CHANNEL_COUNT] = {0};

static int i2cWriteBuffer(const uint8_t *buf, unsigned int len)
{
    unsigned int ret;

    if (buf == NULL || len == 0) {
        printf("[PCA9685] invalid I2C write buffer\n");
        return -1;
    }

    if (EdogI2cBusGuard_Init() != 0) {
        return -1;
    }

    EdogI2cBusGuard_Lock();
    ret = IoTI2cWrite(PCA9685_I2C_PORT, PCA9685_ADDRESS, buf, len);
    EdogI2cBusGuard_Unlock();
    if (ret != IOT_SUCCESS) {
        printf("[PCA9685] I2C write failed: ret=%u len=%u\n", ret, len);
        return -1;
    }

    return 0;
}

static int i2cWrite(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return i2cWriteBuffer(buf, sizeof(buf));
}

int initPCA9685(void)
{
    unsigned int ret;
    uint8_t prescaleVal;

    if (g_pca9685Initialized) {
        return 0;
    }

    if (EdogI2cBusGuard_EnsureBusInit() != 0) {
        printf("[PCA9685] I2C bus init failed\n");
        return -1;
    }

    if (i2cWrite(MODE1_REG, 0x10) != 0) {
        return -1;
    }
    usleep(5000);

    prescaleVal = (uint8_t)(25000000 / (4096 * EDOG_SERVO_PWM_FREQUENCY_HZ) - 1);
    if (i2cWrite(PRE_SCALE, prescaleVal) != 0) {
        return -1;
    }

    if (i2cWrite(MODE2_REG, 0x04) != 0) {
        return -1;
    }

    if (i2cWrite(MODE1_REG, 0x00) != 0) {
        return -1;
    }
    usleep(5000);

    if (i2cWrite(MODE1_REG, 0xA1) != 0) {
        return -1;
    }

    g_pca9685Initialized = 1;
    return 0;
}

static int setPWM(uint8_t channel, uint16_t on, uint16_t off)
{
    uint8_t buf[5];

    if (channel >= EDOG_SERVO_CHANNEL_COUNT) {
        printf("[PCA9685] invalid PWM channel=%u\n", channel);
        return -1;
    }

    buf[0] = LED0_ON_L + 4 * channel;
    buf[1] = on & 0xFF;
    buf[2] = (on >> 8) & 0x0F;
    buf[3] = off & 0xFF;
    buf[4] = (off >> 8) & 0x0F;

    return i2cWriteBuffer(buf, sizeof(buf));
}

static int pulseUsToPwmCount(int pulseUs)
{
    const long long denominator = 1000000LL;
    long long numerator =
        (long long)pulseUs * EDOG_SERVO_PWM_FREQUENCY_HZ * 4096;
    int count = (int)((numerator + denominator / 2) / denominator);

    if (count < 0) {
        count = 0;
    }
    if (count > 4095) {
        count = 4095;
    }
    return count;
}

static int centiDegToPwmCount(int centiDeg)
{
    int centiMin = EDOG_SERVO_MIN_ANGLE * 100;
    int centiMax = EDOG_SERVO_MAX_ANGLE * 100;
    const long long rangeCenti = (long long)EDOG_SERVO_ANGLE_RANGE_DEG * 100;
    const long long denominator = rangeCenti * 1000000LL;
    long long pulseNumeratorUs;
    long long numerator;
    int count;

    if (centiDeg > centiMax) {
        centiDeg = centiMax;
    }
    if (centiDeg < centiMin) {
        centiDeg = centiMin;
    }

    /*
     * 保持完整有理数链路：
     *   pulseUs = min + centiDelta * pulseRange / angleRangeCenti
     *   count   = round(pulseUs * freq * 4096 / 1e6)
     * 两式合并后直接四舍五入到 PCA9685 计数，避免先截断到整数微秒。
     */
    pulseNumeratorUs =
        (long long)EDOG_SERVO_PULSE_MIN_US * rangeCenti +
        (long long)(centiDeg - centiMin) *
            (EDOG_SERVO_PULSE_MAX_US - EDOG_SERVO_PULSE_MIN_US);
    numerator = pulseNumeratorUs * EDOG_SERVO_PWM_FREQUENCY_HZ * 4096;
    count = (int)((numerator + denominator / 2) / denominator);

    if (count < 0) {
        count = 0;
    }
    if (count > 4095) {
        count = 4095;
    }
    return count;
}

static int setServoPwmCount(int channel, int count)
{
    int ret;

    if (channel < 0 || channel >= EDOG_SERVO_CHANNEL_COUNT) {
        printf("[Servo] invalid channel=%d, valid range is 0~%d\n",
               channel, EDOG_SERVO_CHANNEL_COUNT - 1);
        return -1;
    }

    if (initPCA9685() != 0) {
        return -1;
    }

    if (count < 0) {
        count = 0;
    }
    if (count > 4095) {
        count = 4095;
    }
    if (g_pwmOffCountValid[channel] && g_pwmOffCount[channel] == (uint16_t)count) {
        return 0;
    }

    ret = setPWM((uint8_t)channel, 0, (uint16_t)count);
    if (ret != 0) {
        return ret;
    }

    g_pwmOffCount[channel] = (uint16_t)count;
    g_pwmOffCountValid[channel] = 1;
    return 0;
}

int setServoCentiDeg(int channel, int centiDeg)
{
    int centiMin = EDOG_SERVO_MIN_ANGLE * 100;
    int centiMax = EDOG_SERVO_MAX_ANGLE * 100;
    int count;

    if (centiDeg > centiMax) {
        centiDeg = centiMax;
    }
    if (centiDeg < centiMin) {
        centiDeg = centiMin;
    }

    /*
     * 180 度舵机角度直接映射到 PCA9685 计数，厘度(1/100 度)输入保留
     * 到最后一次四舍五入：
     *   0°   -> 500us
     *   90°  -> 1500us
     *   180° -> 2500us
     * PCA9685 计数量化约 4.88us，对 180 度舵机约等于 0.44°/count。
     * 用 64 位中间量避免乘法在 32 位 MCU 上溢出。
     */
    count = centiDegToPwmCount(centiDeg);
    return setServoPwmCount(channel, count);
}

int setServo(int channel, int angle)
{
    return setServoCentiDeg(channel, angle * 100);
}

int setServoPulseUs(int channel, int pulseUs)
{
    int count;

    if (pulseUs < EDOG_SERVO_PULSE_MIN_US) {
        pulseUs = EDOG_SERVO_PULSE_MIN_US;
    }
    if (pulseUs > EDOG_SERVO_PULSE_MAX_US) {
        pulseUs = EDOG_SERVO_PULSE_MAX_US;
    }

    /*
     * PCA9685 每周期 4096 个计数。50Hz 时周期为 20000us：
     * count = pulseUs / 20000us * 4096
     * 写成整数式避免在 LiteOS 嵌入式环境中引入浮点依赖。
     */
    count = pulseUsToPwmCount(pulseUs);
    return setServoPwmCount(channel, count);
}

int setZero(void)
{
    if (initPCA9685() != 0) {
        return -1;
    }

    for (int i = 0; i < EDOG_SERVO_CHANNEL_COUNT; i++) {
        if (setServo(i, EDOG_SERVO_CENTER_ANGLE) != 0) {
            return -1;
        }
        usleep(10000);
    }
    return 0;
}
