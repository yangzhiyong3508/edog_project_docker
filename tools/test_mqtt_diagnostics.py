#!/usr/bin/env python3
import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
IOT_SRC = ROOT / "utils/src/iot.c"
EDOG_CONFIG_SRC = ROOT / "include/edog_config.h"
WIFI_TASK_SRC = ROOT / "src/wifi_mqtt_task.c"
MQTT_LITEOS_SRC = ROOT.parents[4] / "third_party/paho_mqtt/MQTTClient-C/src/liteOS/MQTTLiteOS.c"


def main():
    text = IOT_SRC.read_text(encoding="utf-8")
    config_text = EDOG_CONFIG_SRC.read_text(encoding="utf-8")
    wifi_task_text = WIFI_TASK_SRC.read_text(encoding="utf-8")
    network_text = MQTT_LITEOS_SRC.read_text(encoding="utf-8")

    for token in [
        "static void PrintMqttConnectDiagnostics(void)",
        "[MQTT] diag host=%s port=1883",
        "[MQTT] diag deviceId=%s len=%u",
        "[MQTT] diag clientId=%s len=%u",
        "[MQTT] diag username=%s len=%u",
        "[MQTT] diag keepalive=%d timeout_ms=%d mqtt_version=4 clean_session=1",
        "[MQTT] diag password_len=%u password=***",
        "[MQTT] TCP连接开始",
        "[MQTT] TCP连接失败 rc=%d",
        "[MQTT] TCP连接成功",
        "NetworkConnect(&network, EDOG_MQTT_HOST_ADDR, 1883)",
        "MQTTConnect(&client, &data)",
        "PrintMqttConnectDiagnostics();",
    ]:
        if token not in text:
            raise AssertionError(f"missing MQTT diagnostic token: {token}")

    for forbidden in [
        "static int ConnectMqttTcpWithRetry(void)",
        "ConnectMqttTcpWithRetry();",
        "[MQTT] TCP连接开始 attempt=%d",
        "TCP连接最终失败",
        "MQTTConnectWithResults(&client, &data, &connack)",
        "[MQTT] CONNACK rc=%d session_present=%d",
        "[MQTT] 登录失败 rc=%d connack=%d",
    ]:
        if forbidden in text:
            raise AssertionError(f"mqtt_init must use the proven single TCP connect path, found: {forbidden}")

    timeout_match = re.search(r"#define\s+EDOG_MQTT_COMMAND_TIMEOUT_MS\s+(\d+)", config_text)
    if not timeout_match:
        raise AssertionError("missing MQTT command timeout config")
    if int(timeout_match.group(1)) != 1000:
        raise AssertionError("MQTT command timeout must match the known-good 6M22D value: 1000ms")
    for forbidden in [
        "#define EDOG_WIFI_NETWORK_STABILIZE_MS",
        "static void WaitForNetworkStabilized(void)",
        "LOS_Msleep(EDOG_WIFI_NETWORK_STABILIZE_MS)",
        "WaitForNetworkStabilized();",
    ]:
        if forbidden in config_text or forbidden in wifi_task_text:
            raise AssertionError(f"WiFi/MQTT task must match 6M22D connection timing, found: {forbidden}")

    for token in [
        "[MQTT] net getaddrinfo host=%s rc=%d errno=%d",
        "[MQTT] net resolved host=%s ip=%s port=%d",
        "[MQTT] net socket failed errno=%d",
        "[MQTT] net connect failed rc=%d errno=%d",
        "[MQTT] net connect ok socket=%d",
        "inet_ntoa(address.sin_addr)",
        "struct addrinfo *addr_list = NULL",
        "freeaddrinfo(addr_list);",
        "n->my_socket = -1;",
        "if (n->my_socket >= 0)",
        "rc = connect(n->my_socket, (struct sockaddr*)&address, sizeof(address));",
        "while (bytes < len)",
        "send(n->my_socket, &buffer[bytes], (size_t)(len - bytes), 0)",
        "errno != ETIMEDOUT",
    ]:
        if token not in network_text:
            raise AssertionError(f"missing MQTT network diagnostic token: {token}")

    if re.search(r"freeaddrinfo\s*\(\s*result\s*\)\s*;", network_text):
        raise AssertionError("NetworkConnect must free the original getaddrinfo list head, not the selected node")
    for forbidden in [
        "ConnectWithTimeout",
        "lwip_ioctl(socket_id, FIONBIO",
        "FD_SET(socket_id, &write_set)",
        "net connect timeout errno",
    ]:
        if forbidden in network_text:
            raise AssertionError(f"NetworkConnect must use the proven blocking connect path, found: {forbidden}")

    printf_calls = re.findall(r"printf\s*\((?P<args>.*?)\)\s*;", text, re.S)
    for args in printf_calls:
        if "mqtt_pwd" not in args and "EDOG_MQTT_DEVICE_PASSWORD" not in args:
            continue
        if "password_len=%u password=***" in args and "MqttStringLen(mqtt_pwd)" in args:
            continue
        raise AssertionError("MQTT diagnostics must never print the password value")

    init_match = re.search(r"int\s+mqtt_init\s*\([^)]*\)\s*\{(?P<body>.*?)\n\}", text, re.S)
    if not init_match:
        raise AssertionError("missing mqtt_init")
    init_body = init_match.group("body")
    if init_body.find("ConnectMqttTcpWithRetry();") > init_body.find("MQTTClientInit(&client"):
        raise AssertionError("MQTT TCP retry must complete before MQTTClientInit")
    if init_body.find("PrintMqttConnectDiagnostics();") > init_body.find("MQTTConnect(&client, &data)"):
        raise AssertionError("MQTT diagnostics must print before MQTTConnect")

    print("MQTT diagnostic logging checks passed")


if __name__ == "__main__":
    try:
        main()
    except AssertionError as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        sys.exit(1)
