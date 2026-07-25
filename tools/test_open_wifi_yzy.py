#!/usr/bin/env python3
import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read(relpath):
    return (ROOT / relpath).read_text(encoding="utf-8")


def main():
    config = read("include/edog_config.h")
    local_config = read("include/edog_config.local.h")
    wifi_tool = read("utils/src/wifi_tool.c")
    wifi_task = read("src/wifi_mqtt_task.c")
    portal = read("utils/src/wifi_portal_page.h")

    fixed_ssid_match = re.search(r'#define\s+EDOG_WIFI_FIXED_ROUTE_SSID\s+"([^"]*)"', local_config)
    fixed_password_match = re.search(r'#define\s+EDOG_WIFI_FIXED_ROUTE_PASSWORD\s+"([^"]*)"', local_config)
    factory_ssid_match = re.search(r'#define\s+EDOG_WIFI_FACTORY_ROUTE_SSID\s+"([^"]*)"', config)
    factory_password_match = re.search(r'#define\s+EDOG_WIFI_FACTORY_ROUTE_PASSWORD\s+"([^"]*)"', config)

    if not fixed_ssid_match or fixed_ssid_match.group(1) != "TP-LINK_1E32":
        raise AssertionError("fixed WiFi SSID must be TP-LINK_1E32")
    if not fixed_password_match or fixed_password_match.group(1) != "ilovescmtoo":
        raise AssertionError("fixed WiFi password must be ilovescmtoo")
    if (factory_ssid_match and factory_password_match and
            fixed_ssid_match.group(1) == factory_ssid_match.group(1) and
            fixed_password_match.group(1) == factory_password_match.group(1)):
        raise AssertionError("fixed WiFi credentials must not be filtered as factory defaults")

    save_match = re.search(
        r"int\s+WifiTool_SaveCredentials\s*\([^)]*\)\s*\{(?P<body>.*?)\n\}",
        wifi_tool,
        re.S,
    )
    if not save_match:
        raise AssertionError("missing WifiTool_SaveCredentials")
    save_body = save_match.group("body")
    if "password[0] == '\\0'" in save_body:
        raise AssertionError("WifiTool_SaveCredentials must allow empty passwords for open WiFi")
    if "set_wifi_config_route_passwd(NULL, (uint8_t *)password)" not in save_body:
        raise AssertionError("WifiTool_SaveCredentials must still write the password field")

    has_match = re.search(
        r"int\s+WifiTool_HasSavedCredentials\s*\([^)]*\)\s*\{(?P<body>.*?)\n\}",
        wifi_tool,
        re.S,
    )
    if not has_match:
        raise AssertionError("missing WifiTool_HasSavedCredentials")
    if "password[0] != '\\0'" in has_match.group("body"):
        raise AssertionError("saved credential check must allow empty passwords for open WiFi")

    connect_match = re.search(
        r"static\s+int\s+ConnectFixedNetwork\s*\([^)]*\)\s*\{(?P<body>.*?)\n\}",
        wifi_task,
        re.S,
    )
    if not connect_match:
        raise AssertionError("missing ConnectFixedNetwork")
    connect_body = connect_match.group("body")
    for token in [
        "WifiTool_LoadSavedCredentials",
        "strcmp(savedSsid, EDOG_WIFI_FIXED_ROUTE_SSID) != 0",
        "strcmp(savedPassword, EDOG_WIFI_FIXED_ROUTE_PASSWORD) != 0",
        "WifiTool_SaveCredentials(EDOG_WIFI_FIXED_ROUTE_SSID, EDOG_WIFI_FIXED_ROUTE_PASSWORD)",
    ]:
        if token not in connect_body:
            raise AssertionError(f"ConnectFixedNetwork missing fixed WiFi seeding token: {token}")

    if "if (!ssid)" not in portal or "|| !password" in portal:
        raise AssertionError("provisioning page must allow empty WiFi password")
    if 'id=\\"password\\" name=\\"password\\" type=\\"password\\" maxlength=\\"63\\" autocomplete=\\"current-password\\" required' in portal:
        raise AssertionError("password input must not be required for open WiFi")

    print("fixed TP-LINK_1E32 WiFi configuration checks passed")


if __name__ == "__main__":
    try:
        main()
    except AssertionError as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        sys.exit(1)
