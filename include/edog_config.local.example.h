#ifndef EDOG_CONFIG_LOCAL_H
#define EDOG_CONFIG_LOCAL_H

/*
 * Copy this file to edog_config.local.h and fill in the values from Huawei
 * IoTDA before flashing a real device. Do not commit edog_config.local.h.
 */
#define EDOG_MQTT_DEVICE_PASSWORD              "your-device-secret"
#define EDOG_MQTT_HOST_ADDR                    "587a77885a.st1.iotda-device.cn-east-3.myhuaweicloud.com"
#define EDOG_MQTT_DEVICE_ID                    "your-device-id"
#define EDOG_MQTT_CLIENT_ID                    "your-client-id"
#define EDOG_MQTT_USERNAME                     EDOG_MQTT_DEVICE_ID

#endif
