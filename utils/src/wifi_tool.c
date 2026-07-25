#include "wifi_tool.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>

#include "../../include/edog_config.h"
#include "cJSON.h"
#include "config_network.h"
#include "iot_gpio.h"
#include "los_task.h"
#include "lwip/ip4_addr.h"
#include "lwip/netif.h"
#include "lwip/sockets.h"
#include "reset.h"

#include "wifi_portal_page.h"

typedef struct {
    char request[EDOG_WIFI_HTTP_BUFFER_LEN];
    char method[8];
    char path[64];
    char ssid[WIFI_TOOL_MAX_CREDENTIAL_LEN];
    char password[WIFI_TOOL_MAX_CREDENTIAL_LEN];
    char errorMessage[96];
    char response[160];
} WifiToolHttpContext;

/*
 * The provisioning portal runs in wifi_mqtt_task. Keep HTTP scratch buffers
 * out of that task's stack so the portal stays stable on LiteOS.
 */
static WifiToolHttpContext g_wifiToolHttpContext;

#define HTTP_RECV_TOO_LARGE (-2)

static void HttpSetSocketTimeouts(int fd)
{
    struct timeval timeout;

    timeout.tv_sec = EDOG_WIFI_HTTP_TIMEOUT_MS / 1000;
    timeout.tv_usec = (EDOG_WIFI_HTTP_TIMEOUT_MS % 1000) * 1000;
    (void)setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    (void)setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
}

static int HttpSendAll(int fd, const char *data, size_t len)
{
    size_t sent = 0;

    while (sent < len) {
        int rc = send(fd, data + sent, len - sent, 0);
        if (rc <= 0) {
            return -1;
        }
        sent += (size_t)rc;
    }
    return 0;
}

static int HttpSendResponse(int fd, const char *status,
                            const char *contentType, const char *body)
{
    char header[256];
    size_t bodyLen = strlen(body);
    int headerLen = snprintf(header, sizeof(header),
                             "HTTP/1.1 %s\r\n"
                             "Content-Type: %s\r\n"
                             "Content-Length: %u\r\n"
                             "Connection: close\r\n"
                             "Cache-Control: no-store\r\n"
                             "\r\n",
                             status, contentType, (unsigned int)bodyLen);
    if (headerLen < 0 || headerLen >= (int)sizeof(header)) {
        return -1;
    }

    if (HttpSendAll(fd, header, (size_t)headerLen) != 0) {
        return -1;
    }
    return HttpSendAll(fd, body, bodyLen);
}

static int HttpSendRedirect(int fd, const char *location)
{
    char response[256];
    int len = snprintf(response, sizeof(response),
                       "HTTP/1.1 302 Found\r\n"
                       "Location: %s\r\n"
                       "Connection: close\r\n"
                       "Cache-Control: no-store\r\n"
                       "\r\n",
                       location);
    if (len < 0 || len >= (int)sizeof(response)) {
        return -1;
    }
    return HttpSendAll(fd, response, (size_t)len);
}

static int HttpGetContentLength(const char *request)
{
    const char *header = strstr(request, "Content-Length:");
    char *end = NULL;
    long value;

    if (header == NULL) {
        return 0;
    }

    header += strlen("Content-Length:");
    while (*header == ' ' || *header == '\t') {
        header++;
    }
    value = strtol(header, &end, 10);
    if (end == header || value < 0 || value > EDOG_WIFI_HTTP_MAX_CONTENT_LENGTH) {
        return HTTP_RECV_TOO_LARGE;
    }
    return (int)value;
}

static int HttpReceiveRequest(int fd, char *buffer, size_t bufferLen)
{
    int total = 0;
    int contentLength = 0;
    char *body = NULL;

    if (buffer == NULL || bufferLen < 2) {
        return -1;
    }

    while (total < (int)bufferLen - 1) {
        int rc = recv(fd, buffer + total, bufferLen - 1 - total, 0);
        if (rc <= 0) {
            break;
        }
        total += rc;
        buffer[total] = '\0';

        if (body == NULL) {
            body = strstr(buffer, "\r\n\r\n");
            if (body != NULL) {
                body += 4;
                contentLength = HttpGetContentLength(buffer);
                if (contentLength == HTTP_RECV_TOO_LARGE) {
                    return HTTP_RECV_TOO_LARGE;
                }
                if ((body - buffer) + contentLength >= (int)bufferLen) {
                    return HTTP_RECV_TOO_LARGE;
                }
                if (contentLength == 0) {
                    break;
                }
            }
        }

        if (body != NULL && total >= (int)((body - buffer) + contentLength)) {
            break;
        }
    }

    return total;
}

static void TrimAsciiWhitespace(char *value)
{
    char *start = value;
    size_t len;

    if (value == NULL) {
        return;
    }

    while (*start != '\0' && isspace((unsigned char)*start)) {
        start++;
    }

    if (start != value) {
        memmove(value, start, strlen(start) + 1);
    }

    len = strlen(value);
    while (len > 0 && isspace((unsigned char)value[len - 1])) {
        value[--len] = '\0';
    }
}

static int ParseProvisioningBody(const char *body, size_t bodyLen,
                                 char *ssid, size_t ssidLen,
                                 char *password, size_t passwordLen,
                                 char *errorMessage, size_t errorMessageLen)
{
    cJSON *root;
    cJSON *ssidItem;
    cJSON *passwordItem;

    root = cJSON_ParseWithLength(body, bodyLen);
    if (root == NULL) {
        snprintf(errorMessage, errorMessageLen, "请求数据格式错误。");
        return -1;
    }

    ssidItem = cJSON_GetObjectItem(root, "ssid");
    passwordItem = cJSON_GetObjectItem(root, "password");
    if (!cJSON_IsString(ssidItem) || !cJSON_IsString(passwordItem)) {
        cJSON_Delete(root);
        snprintf(errorMessage, errorMessageLen, "必须填写 Wi-Fi 名称和密码。");
        return -1;
    }

    snprintf(ssid, ssidLen, "%s", cJSON_GetStringValue(ssidItem));
    snprintf(password, passwordLen, "%s", cJSON_GetStringValue(passwordItem));
    cJSON_Delete(root);

    TrimAsciiWhitespace(ssid);
    TrimAsciiWhitespace(password);
    if (ssid[0] == '\0') {
        snprintf(errorMessage, errorMessageLen, "必须填写 Wi-Fi 名称。");
        return -1;
    }
    return 0;
}

static int WifiTool_IsFactoryDefaultCredential(const char *ssid, const char *password)
{
    return strcmp(ssid, EDOG_WIFI_FACTORY_ROUTE_SSID) == 0 &&
           strcmp(password, EDOG_WIFI_FACTORY_ROUTE_PASSWORD) == 0;
}

static void WifiTool_GetProvisioningAddress(char *buffer, size_t len)
{
    wifi_config_t config = {0};

    if (buffer == NULL || len == 0) {
        return;
    }

    get_wifi_config(NULL, &config);
    if (config.gateway[0] != 0 || config.gateway[1] != 0 ||
        config.gateway[2] != 0 || config.gateway[3] != 0) {
        (void)snprintf(buffer, len, "%u.%u.%u.%u",
                       config.gateway[0], config.gateway[1],
                       config.gateway[2], config.gateway[3]);
        return;
    }

    (void)snprintf(buffer, len, "%s", EDOG_WIFI_AP_FALLBACK_IP);
}

int WifiTool_GetCurrentIPv4(char *buffer, size_t len)
{
    struct netif *netif;
    const ip4_addr_t *addr;

    if (buffer == NULL || len == 0) {
        return -1;
    }

    netif = netif_default;
    if (netif == NULL) {
        return -1;
    }

    addr = netif_ip4_addr(netif);
    if (addr == NULL || ip4_addr_isany_val(*addr)) {
        return -1;
    }

    if (ip4addr_ntoa_r(addr, buffer, len) == NULL) {
        return -1;
    }
    return 0;
}

int WifiTool_LoadSavedCredentials(char *ssid, size_t ssidLen,
                                  char *password, size_t passwordLen)
{
    wifi_config_t config = {0};

    if (ssid == NULL || password == NULL || ssidLen == 0 || passwordLen == 0) {
        return -1;
    }

    get_wifi_config(NULL, &config);
    snprintf(ssid, ssidLen, "%s", (const char *)config.route_ssid);
    snprintf(password, passwordLen, "%s", (const char *)config.route_password);
    return 0;
}

int WifiTool_HasSavedCredentials(void)
{
    char ssid[WIFI_TOOL_MAX_CREDENTIAL_LEN] = {0};
    char password[WIFI_TOOL_MAX_CREDENTIAL_LEN] = {0};

    if (WifiTool_LoadSavedCredentials(ssid, sizeof(ssid), password, sizeof(password)) != 0) {
        return 0;
    }
    if (WifiTool_IsFactoryDefaultCredential(ssid, password)) {
        return 0;
    }
    return ssid[0] != '\0';
}

int WifiTool_SaveCredentials(const char *ssid, const char *password)
{
    if (ssid == NULL || password == NULL || ssid[0] == '\0') {
        return -1;
    }

    set_wifi_config_route_ssid(NULL, (uint8_t *)ssid);
    set_wifi_config_route_passwd(NULL, (uint8_t *)password);
    set_wifi_config_mode(NULL, (uint8_t *)"STA");
    return 0;
}

void WifiTool_ClearCredentials(void)
{
    set_wifi_config_route_ssid(NULL, (uint8_t *)"");
    set_wifi_config_route_passwd(NULL, (uint8_t *)"");
    set_wifi_config_mode(NULL, (uint8_t *)"AP");
}

int WifiTool_IsStaConnected(void)
{
    WifiLinkedInfo info;

    memset(&info, 0, sizeof(info));
    if (GetLinkedInfo(&info) != WIFI_SUCCESS) {
        return 0;
    }
    return info.connState == WIFI_CONNECTED && info.ipAddress != 0;
}

int WifiTool_ConnectSavedNetwork(void)
{
    char ssid[WIFI_TOOL_MAX_CREDENTIAL_LEN] = {0};
    char password[WIFI_TOOL_MAX_CREDENTIAL_LEN] = {0};
    WifiErrorCode error;

    if (!WifiTool_HasSavedCredentials()) {
        printf("[WiFi] no saved credentials available\n");
        return -1;
    }

    WifiTool_LoadSavedCredentials(ssid, sizeof(ssid), password, sizeof(password));

    set_wifi_config_mode(NULL, (uint8_t *)"STA");
    (void)SetApModeOff();
    (void)SetWifiModeOff();

    error = SetWifiModeOn();
    if (error != WIFI_SUCCESS) {
        printf("[WiFi] failed to connect to saved network, error=%d\n", error);
        return -1;
    }

    return 0;
}

int WifiTool_StartProvisioningAp(void)
{
    char ipAddr[16] = {0};
    WifiErrorCode error;

    set_wifi_config_ssid(NULL, (uint8_t *)EDOG_WIFI_AP_SSID);
    set_wifi_config_passwd(NULL, (uint8_t *)EDOG_WIFI_AP_PASSWORD);
    set_wifi_config_mode(NULL, (uint8_t *)"AP");

    (void)SetWifiModeOff();
    (void)SetApModeOff();
    error = SetApModeOn();
    if (error != WIFI_SUCCESS) {
        printf("[WiFi] failed to start provisioning AP, error=%d\n", error);
        return -1;
    }

    WifiTool_GetProvisioningAddress(ipAddr, sizeof(ipAddr));
    return 0;
}

void WifiTool_StopProvisioningAp(void)
{
    (void)SetApModeOff();
}

static void HandleHttpClient(int clientFd, int *configured)
{
    WifiToolHttpContext *ctx = &g_wifiToolHttpContext;
    const char *body;
    int requestLen;
    int bodyLen;

    memset(ctx, 0, sizeof(*ctx));
    HttpSetSocketTimeouts(clientFd);

    requestLen = HttpReceiveRequest(clientFd, ctx->request, sizeof(ctx->request));
    if (requestLen == HTTP_RECV_TOO_LARGE) {
        (void)HttpSendResponse(clientFd, "413 Payload Too Large",
                               "application/json",
                               "{\"message\":\"request too large\"}");
        return;
    }
    if (requestLen <= 0) {
        return;
    }

    if (sscanf(ctx->request, "%7s %63s", ctx->method, ctx->path) != 2) {
        (void)HttpSendResponse(clientFd, "400 Bad Request",
                               "application/json",
                               "{\"message\":\"请求格式错误。\"}");
        return;
    }

    if (strcmp(ctx->method, "GET") == 0 || strcmp(ctx->method, "HEAD") == 0) {
        if (strcmp(ctx->path, "/") == 0 || strcmp(ctx->path, "/index.html") == 0) {
            (void)HttpSendResponse(clientFd, "200 OK",
                                   "text/html; charset=utf-8",
                                   WIFI_PORTAL_HTML);
            return;
        }
        (void)HttpSendRedirect(clientFd, "/");
        return;
    }

    if (strcmp(ctx->method, "POST") != 0 || strcmp(ctx->path, "/api/wifi/config") != 0) {
        (void)HttpSendResponse(clientFd, "405 Method Not Allowed",
                               "application/json",
                               "{\"message\":\"不支持的请求。\"}");
        return;
    }

    body = strstr(ctx->request, "\r\n\r\n");
    if (body == NULL) {
        (void)HttpSendResponse(clientFd, "400 Bad Request",
                               "application/json",
                               "{\"message\":\"请求体为空。\"}");
        return;
    }
    body += 4;
    bodyLen = requestLen - (int)(body - ctx->request);
    if (bodyLen <= 0) {
        (void)HttpSendResponse(clientFd, "400 Bad Request",
                               "application/json",
                               "{\"message\":\"请求体为空。\"}");
        return;
    }

    if (ParseProvisioningBody(body, (size_t)bodyLen,
                              ctx->ssid, sizeof(ctx->ssid),
                              ctx->password, sizeof(ctx->password),
                              ctx->errorMessage, sizeof(ctx->errorMessage)) != 0) {
        snprintf(ctx->response, sizeof(ctx->response),
                 "{\"message\":\"%s\"}", ctx->errorMessage);
        (void)HttpSendResponse(clientFd, "400 Bad Request",
                               "application/json", ctx->response);
        return;
    }

    if (WifiTool_SaveCredentials(ctx->ssid, ctx->password) != 0) {
        (void)HttpSendResponse(clientFd, "500 Internal Server Error",
                               "application/json",
                               "{\"message\":\"保存 Wi-Fi 信息失败。\"}");
        return;
    }

    *configured = 1;
    (void)HttpSendResponse(clientFd, "200 OK",
                           "application/json",
                           "{\"message\":\"Wi-Fi 信息已保存，eDog 正在连接网络。\"}");
}

int WifiTool_RunProvisioningPortal(void)
{
    int serverFd;
    int configured = 0;
    int flag = 1;
    int rc;
    struct sockaddr_in serverAddr = {0};

    serverFd = socket(AF_INET, SOCK_STREAM, 0);
    if (serverFd < 0) {
        printf("[WiFi] failed to create HTTP server socket\n");
        return -1;
    }

    rc = setsockopt(serverFd, SOL_SOCKET, SO_REUSEADDR, &flag, sizeof(flag));
    if (rc != 0) {
        printf("[WiFi] setsockopt SO_REUSEADDR failed: %d\n", rc);
    }

    serverAddr.sin_family = AF_INET;
    serverAddr.sin_addr.s_addr = htonl(INADDR_ANY);
    serverAddr.sin_port = htons(EDOG_WIFI_HTTP_PORT);

    rc = bind(serverFd, (struct sockaddr *)&serverAddr, sizeof(serverAddr));
    if (rc < 0) {
        printf("[WiFi] bind HTTP server failed\n");
        lwip_close(serverFd);
        return -1;
    }

    rc = listen(serverFd, 4);
    if (rc < 0) {
        printf("[WiFi] listen HTTP server failed\n");
        lwip_close(serverFd);
        return -1;
    }

    while (!configured) {
        int clientFd;
        socklen_t clientLen;
        struct sockaddr_in clientAddr = {0};

        clientLen = sizeof(clientAddr);
        clientFd = accept(serverFd, (struct sockaddr *)&clientAddr, &clientLen);
        if (clientFd < 0) {
            LOS_Msleep(100);
            continue;
        }

        HandleHttpClient(clientFd, &configured);
        lwip_close(clientFd);
    }

    lwip_close(serverFd);
    return 0;
}

void WifiTool_ResetButtonTask(void)
{
    int isPressed = 0;
    int pressCount = 0;

    IoTGpioInit(EDOG_WIFI_RESET_GPIO);
    IoTGpioSetDir(EDOG_WIFI_RESET_GPIO, IOT_GPIO_DIR_IN);

    while (1) {
        IotGpioValue value = IOT_GPIO_VALUE1;
        IoTGpioGetInputVal(EDOG_WIFI_RESET_GPIO, &value);

        if (value == EDOG_WIFI_RESET_PRESSED) {
            if (!isPressed) {
                LOS_Msleep(10);
                IoTGpioGetInputVal(EDOG_WIFI_RESET_GPIO, &value);
                if (value == EDOG_WIFI_RESET_PRESSED) {
                    isPressed = 1;
                    pressCount = 0;
                }
            } else {
                pressCount++;
                if (pressCount >= EDOG_WIFI_RESET_LONG_PRESS_COUNT) {
                    printf("[WiFi] long press detected, clearing credentials and rebooting\n");
                    WifiTool_ClearCredentials();
                    RebootDevice(3);
                }
            }
        } else if (isPressed) {
            isPressed = 0;
            pressCount = 0;
        }

        LOS_Msleep(50);
    }
}
