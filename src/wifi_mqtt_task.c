#include "wifi_mqtt_task.h"

#include "../include/edog_config.h"
#include "../utils/include/iot.h"
#include "../utils/include/wifi_tool.h"

#include "los_task.h"
#include <stdio.h>
#include <string.h>

static int WaitForStaConnected(unsigned int timeoutMs)
{
    unsigned int elapsedMs = 0;

    while (elapsedMs < timeoutMs) {
        if (WifiTool_IsStaConnected()) {
            return 0;
        }
        LOS_Msleep(EDOG_WIFI_CONNECT_POLL_INTERVAL_MS);
        elapsedMs += EDOG_WIFI_CONNECT_POLL_INTERVAL_MS;
    }
    return -1;
}

static int TryConnectSavedNetworkFast(int retries)
{
    int attempt;

    for (attempt = 1; attempt <= retries; attempt++) {
        printf("[网络] WiFi连接开始 attempt=%d\n", attempt);
        if (WifiTool_ConnectSavedNetwork() == 0 &&
            WaitForStaConnected(EDOG_WIFI_CONNECT_FAST_TIMEOUT_MS) == 0) {
            printf("[网络] WiFi连接成功 attempt=%d\n", attempt);
            return 0;
        }
        printf("[网络] WiFi连接失败 attempt=%d\n", attempt);
        LOS_Msleep(EDOG_WIFI_FAST_RETRY_DELAY_MS);
    }
    return -1;
}

static int ConnectFixedNetwork(void)
{
    char savedSsid[64] = {0};
    char savedPassword[64] = {0};
    int needsSeed = 1;

    if (WifiTool_LoadSavedCredentials(savedSsid, sizeof(savedSsid),
                                      savedPassword, sizeof(savedPassword)) == 0) {
        needsSeed = strcmp(savedSsid, EDOG_WIFI_FIXED_ROUTE_SSID) != 0 ||
                    strcmp(savedPassword, EDOG_WIFI_FIXED_ROUTE_PASSWORD) != 0;
    }

    if (needsSeed &&
        WifiTool_SaveCredentials(EDOG_WIFI_FIXED_ROUTE_SSID, EDOG_WIFI_FIXED_ROUTE_PASSWORD) != 0) {
        printf("[网络] 固定 WiFi 凭据保存失败\n");
        return -1;
    }
    return TryConnectSavedNetworkFast(EDOG_WIFI_STA_CONNECT_RETRIES);
}

static int EnterProvisioningMode(void)
{
    while (1) {
        if (WifiTool_StartProvisioningAp() != 0) {
            printf("[配网] 启动 AP 配网失败\n");
            LOS_Msleep(EDOG_WIFI_FAST_RETRY_DELAY_MS);
            continue;
        }

        if (WifiTool_RunProvisioningPortal() != 0) {
            printf("[配网] 配网 HTTP 服务异常，重新启动 AP\n");
            WifiTool_StopProvisioningAp();
            LOS_Msleep(1000);
            continue;
        }

        LOS_Msleep(EDOG_WIFI_PROVISIONING_RESPONSE_MS);
        WifiTool_StopProvisioningAp();
        if (TryConnectSavedNetworkFast(EDOG_WIFI_STA_CONNECT_RETRIES) == 0) {
            return 0;
        }

        printf("[配网] 新网络连接失败，回到 AP 模式重新配置\n");
    }
}

static int EnsureProvisionedNetworkReady(void)
{
#if !EDOG_WIFI_PROVISIONING_ENABLED
    return ConnectFixedNetwork();
#else
    if (WifiTool_HasSavedCredentials()) {
        if (TryConnectSavedNetworkFast(EDOG_WIFI_STA_CONNECT_RETRIES) == 0) {
            return 0;
        }
        printf("[网络] 已保存网络连接失败，回到 AP 配网\n");
    } else {
    }
    return EnterProvisioningMode();
#endif
}

static unsigned int GetMqttRetryDelayMs(unsigned int mqttFailureCount)
{
    if (mqttFailureCount < EDOG_WIFI_MQTT_FAST_RETRY_COUNT) {
        return EDOG_WIFI_MQTT_FAST_RETRY_DELAY_MS;
    }
    return EDOG_WIFI_MQTT_RETRY_DELAY_MS;
}

static void RunMqttSession(void)
{
    unsigned int heartbeatElapsedMs = 0;

    while (1) {
        if (!WifiTool_IsStaConnected()) {
            printf("[平台] WiFi 已断开，停止 MQTT\n");
            mqtt_disconnect();
            return;
        }

        if (!wait_message(EDOG_WIFI_MQTT_YIELD_MS)) {
            printf("[平台] 连接中断，正在重试...\n");
            mqtt_disconnect();
            return;
        }

        heartbeatElapsedMs += EDOG_WIFI_MQTT_YIELD_MS;
        if (heartbeatElapsedMs >= EDOG_WIFI_MQTT_HEARTBEAT_MS) {
            mqtt_send_heartbeat();
            heartbeatElapsedMs = 0;
            if (!mqtt_is_connected()) {
                printf("[平台] 心跳失败，准备重连 MQTT\n");
                mqtt_disconnect();
                return;
            }
        }
    }
}

void WifiTask(void)
{
    unsigned int mqttFailureCount = 0;

    LOS_Msleep(EDOG_WIFI_STARTUP_STABILIZE_MS);

    while (1) {
        if (EnsureProvisionedNetworkReady() != 0) {
            LOS_Msleep(EDOG_WIFI_FAST_RETRY_DELAY_MS);
            continue;
        }

        while (WifiTool_IsStaConnected()) {
            if (mqtt_init() != 0) {
                printf("[平台] MQTT 连接失败，稍后重试\n");
                LOS_Msleep(GetMqttRetryDelayMs(mqttFailureCount));
                mqttFailureCount++;
                continue;
            }

            mqttFailureCount = 0;
            RunMqttSession();
            LOS_Msleep(1000);
        }

        printf("[网络] WiFi 连接丢失，准备重新连接\n");
        if (TryConnectSavedNetworkFast(EDOG_WIFI_STA_CONNECT_RETRIES) != 0) {
#if !EDOG_WIFI_PROVISIONING_ENABLED
            printf("[网络] fixed WiFi reconnect failed, retry later\n");
            LOS_Msleep(EDOG_WIFI_MQTT_RETRY_DELAY_MS);
#else
            printf("[网络] 多次重连失败，重新进入 AP 配网模式\n");
            (void)EnterProvisioningMode();
#endif
        }
    }
}

