#!/usr/bin/env python3
import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CONFIG_SRC = ROOT / "include/edog_config.h"
IOT_SRC = ROOT / "utils/src/iot.c"
MQTT_CLIENT_SRC = ROOT.parents[4] / "third_party/paho_mqtt/MQTTClient-C/src/MQTTClient.c"


def require(text, token):
    if token not in text:
        raise AssertionError(f"missing token: {token}")


def main():
    config = CONFIG_SRC.read_text(encoding="utf-8")
    iot = IOT_SRC.read_text(encoding="utf-8")
    mqtt_client = MQTT_CLIENT_SRC.read_text(encoding="utf-8")

    match = re.search(r"#define\s+EDOG_MQTT_PACKET_BUFFER_LENGTH\s+(\d+)\b", config)
    if not match:
        raise AssertionError("missing EDOG_MQTT_PACKET_BUFFER_LENGTH")
    if int(match.group(1)) < 2048:
        raise AssertionError("MQTT packet buffer must be at least 2048 bytes for IoTDA system packets")

    require(iot, "static unsigned char sendBuf[EDOG_MQTT_PACKET_BUFFER_LENGTH];")
    require(iot, "static unsigned char readBuf[EDOG_MQTT_PACKET_BUFFER_LENGTH];")
    require(iot, "MQTTClientInit(&client, &network, EDOG_MQTT_COMMAND_TIMEOUT_MS")

    require(mqtt_client, "static int MqttReadTimeoutMs(MQTTClient* c, Timer* timer)")
    require(mqtt_client, "return left_ms > 0 ? left_ms : (int)c->command_timeout_ms;")
    require(mqtt_client, "[MQTT] readPacket overflow rem_len=%d readbuf_size=%u")
    require(mqtt_client, "rc = c->ipstack->mqttread(c->ipstack, c->readbuf + len, rem_len, MqttReadTimeoutMs(c, timer));")

    forbidden = "c->ipstack->mqttread(c->ipstack, c->readbuf + len, rem_len, TimerLeftMS(timer))"
    if forbidden in mqtt_client:
        raise AssertionError("MQTT body read must not use an exhausted yield timer")

    print("MQTT packet buffer and body timeout checks passed")


if __name__ == "__main__":
    try:
        main()
    except AssertionError as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        sys.exit(1)
