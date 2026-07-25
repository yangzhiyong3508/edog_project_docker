#include "i2c_bus_guard.h"
#include "iot_errno.h"
#include "iot_i2c.h"
#include "los_mux.h"
#include "los_task.h"
#include <stdio.h>

static UINT32 g_i2cBusMux;
static volatile int g_i2cBusMuxReady = 0;
static volatile int g_i2cBusInitialized = 0;

int EdogI2cBusGuard_Init(void)
{
    UINT32 ret;

    if (g_i2cBusMuxReady) {
        return 0;
    }

    ret = LOS_MuxCreate(&g_i2cBusMux);
    if (ret != LOS_OK) {
        printf("[I2C] create bus mutex failed: %u\n", ret);
        return -1;
    }

    g_i2cBusMuxReady = 1;
    return 0;
}

void EdogI2cBusGuard_Lock(void)
{
    if (EdogI2cBusGuard_Init() == 0) {
        (void)LOS_MuxPend(g_i2cBusMux, LOS_WAIT_FOREVER);
    }
}

void EdogI2cBusGuard_Unlock(void)
{
    if (g_i2cBusMuxReady) {
        (void)LOS_MuxPost(g_i2cBusMux);
    }
}

int EdogI2cBusGuard_EnsureBusInit(void)
{
    unsigned int ret;

    if (g_i2cBusInitialized) {
        return 0;
    }

    if (EdogI2cBusGuard_Init() != 0) {
        return -1;
    }

    EdogI2cBusGuard_Lock();
    if (!g_i2cBusInitialized) {
        ret = IoTI2cInit(EI2C0_M2, EDOG_I2C_BUS_FREQUENCY);
        if (ret != IOT_SUCCESS && ret != 1) {
            printf("[I2C] bus init failed ret=%u\n", ret);
            EdogI2cBusGuard_Unlock();
            return -1;
        }
        g_i2cBusInitialized = 1;
    }
    EdogI2cBusGuard_Unlock();
    return 0;
}
