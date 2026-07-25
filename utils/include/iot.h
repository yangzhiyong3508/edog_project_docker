#ifndef _IOT_H_
#define _IOT_H_

#include <stdbool.h>

typedef struct
{
    double illumination;
    double temperature;
    double humidity;
    bool motor_state;
    bool light_state;
    bool auto_state;
} e_iot_data;

#define IOT_CMD_LIGHT_ON 0x01
#define IOT_CMD_LIGHT_OFF 0x02
#define IOT_CMD_MOTOR_ON 0x03
#define IOT_CMD_MOTOR_OFF 0x04
#define IOT_CMD_AUTO_ON 0x05
#define IOT_CMD_AUTO_OFF 0x06

int wait_message(int timeoutMs);
int mqtt_init(void);
void mqtt_send_heartbeat(void);
void mqtt_disconnect(void);
unsigned int mqtt_is_connected();
void send_msg_to_mqtt(e_iot_data *iot_data);
void send_msg_to_edog(char *msg);
void send_servo_calibration_properties(const int offsets[], int count);
void send_servo_status_properties(const int angles[], const int offsets[], int count, const char *state);

#endif // _IOT_H_
