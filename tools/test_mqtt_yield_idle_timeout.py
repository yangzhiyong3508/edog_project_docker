#!/usr/bin/env python3
import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MQTT_LITEOS_SRC = ROOT.parents[4] / "third_party/paho_mqtt/MQTTClient-C/src/liteOS/MQTTLiteOS.c"


def function_body(text, name):
    match = re.search(rf"int\s+{name}\s*\([^)]*\)\s*\{{", text)
    if not match:
        raise AssertionError(f"missing function {name}")
    start = match.end()
    depth = 1
    pos = start
    while pos < len(text) and depth:
        if text[pos] == "{":
            depth += 1
        elif text[pos] == "}":
            depth -= 1
        pos += 1
    if depth != 0:
        raise AssertionError(f"unterminated function {name}")
    return text[start:pos - 1]


def main():
    text = MQTT_LITEOS_SRC.read_text(encoding="utf-8")
    body = function_body(text, "linux_read")

    required = [
        "fd_set read_set",
        "FD_ZERO(&read_set)",
        "FD_SET(n->my_socket, &read_set)",
        "select(n->my_socket + 1, &read_set, NULL, NULL, &interval)",
        "if (rc == 0)",
        "break;",
        "recv(n->my_socket, &buffer[bytes], (size_t)(len - bytes), 0)",
        "total_timeout_ms",
        "select_timeout_ms",
        "bytes += rc;",
        "continue;",
        "errno != EAGAIN && errno != EWOULDBLOCK && errno != ETIMEDOUT",
        "errno != EINTR && errno != ECONNABORTED",
    ]
    for token in required:
        if token not in body:
            raise AssertionError(f"linux_read must use select-based idle timeout handling, missing: {token}")

    if "SO_RCVTIMEO" in body:
        raise AssertionError("linux_read must not depend on SO_RCVTIMEO for idle MQTTYield timeout")

    close_match = re.search(r"else\s+if\s*\(\s*rc\s*==\s*0\s*\)\s*\{(?P<body>.*?)\}", body, re.S)
    if not close_match:
        raise AssertionError("linux_read must explicitly handle peer close from recv rc == 0")
    if "bytes = -1" not in close_match.group("body"):
        raise AssertionError("recv rc == 0 after select-readable is peer close and must force reconnect")

    print("MQTT yield idle timeout checks passed")


if __name__ == "__main__":
    try:
        main()
    except AssertionError as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        sys.exit(1)
