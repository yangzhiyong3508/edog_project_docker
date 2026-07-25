#ifndef WIFI_TOOL_H
#define WIFI_TOOL_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define WIFI_TOOL_MAX_CREDENTIAL_LEN 64

int WifiTool_GetCurrentIPv4(char *buffer, size_t len);
int WifiTool_HasSavedCredentials(void);
int WifiTool_LoadSavedCredentials(char *ssid, size_t ssidLen,
                                  char *password, size_t passwordLen);
int WifiTool_SaveCredentials(const char *ssid, const char *password);
void WifiTool_ClearCredentials(void);

int WifiTool_IsStaConnected(void);
int WifiTool_ConnectSavedNetwork(void);

int WifiTool_StartProvisioningAp(void);
void WifiTool_StopProvisioningAp(void);
int WifiTool_RunProvisioningPortal(void);

void WifiTool_ResetButtonTask(void);

#ifdef __cplusplus
}
#endif

#endif
