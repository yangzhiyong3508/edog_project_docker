<div align="center">

# EDOG Firmware · edog_project

**软通动力通晓开发板（RK2206）** · OpenHarmony LiteOS · 完整机载工程

[![Board](https://img.shields.io/badge/Board-通晓%20RK2206-0ea5e9?style=for-the-badge)](https://github.com/yangzhiyong3508/edog_project_docker)
[![Modules](https://img.shields.io/badge/Modules-WiFi%20%7C%20MQTT%20%7C%20IoT%20%7C%2012DOF-8b5cf6?style=for-the-badge)](https://github.com/yangzhiyong3508/edog_project_docker)

</div>

---

## 本仓库包含什么（完整模块，不只是步态）

| 模块 | 路径 | 说明 |
|------|------|------|
| 入口 | `src/main.c` | 启动、任务创建 |
| WiFi + MQTT 任务 | `src/wifi_mqtt_task.c` · `include/wifi_mqtt_task.h` | 联网与云端会话任务 |
| WiFi 工具 | `utils/src/wifi_tool.c` · `utils/include/wifi_tool.h` | STA/配网等 |
| WiFi Portal 页 | `utils/src/wifi_portal_page.h` | 配网页面资源 |
| MQTT | `utils/src/mqtt_connect.c` · `utils/include/mqtt_connect.h` | MQTT 连接封装 |
| IoT 协议/上报 | `utils/src/iot.c` · `utils/include/iot.h` | 华为云 IoTDA 消息与属性 |
| IoT 命令处理 | `utils/src/iot_control.c` · `utils/include/iot_control.h` | `motion_control` 等命令解析与入队 |
| 舵机 | `utils/src/servo_control.c` · `utils/include/servo_control.h` | PWM/舵机输出 |
| I2C | `utils/src/i2c_bus_guard.c` | 总线保护 |
| IMU | `utils/src/mpu6050_motion_light.c` | MPU6050 轻量姿态 |
| 任务工具 | `utils/src/task_util.c` | 任务辅助 |
| 8DOF 遗留步态 | `utils/src/gait_generate.c` · `motion_utils.c` | 兼容/参考实现 |
| **12DOF 步态** | `12_DOF_Version/src/*` | 当前主用步态与逆解 |
| 配置 | `include/edog_config.h` | 编译期配置 |
| 本地密钥模板 | `include/edog_config.local.example.h` | WiFi/MQTT 密钥（复制为 `.local.h`，勿提交） |
| 构建 | `BUILD.gn` | 静态库 `edog_project` 源文件列表 |
| 测试脚本 | `tools/` | 步态/MQTT/WiFi 等契约测试 |

`BUILD.gn` 已编入的源文件：

```text
src/main.c
src/wifi_mqtt_task.c
utils/src/servo_control.c
utils/src/i2c_bus_guard.c
utils/src/mpu6050_motion_light.c
utils/src/task_util.c
utils/src/wifi_tool.c
utils/src/iot.c
utils/src/iot_control.c
utils/src/mqtt_connect.c
12_DOF_Version/src/gait_generate_12dof.c
12_DOF_Version/src/motion_utils_12dof.c
```

---

## 目录树

```text
edog_project/
├── BUILD.gn
├── include/
│   ├── edog_config.h
│   ├── edog_config.local.example.h
│   ├── utils.h
│   └── wifi_mqtt_task.h
├── src/
│   ├── main.c
│   └── wifi_mqtt_task.c
├── utils/
│   ├── include/          # iot · iot_control · mqtt · wifi · servo · ...
│   └── src/              # 同上对应 .c
├── 12_DOF_Version/
│   ├── include/
│   └── src/
└── tools/                # 主机侧测试脚本
```

---

## 本地密钥

```bash
cp include/edog_config.local.example.h include/edog_config.local.h
# 填写 WiFi / IoTDA 设备三元组与接入地址
```

`edog_config.local.h` **禁止提交**。

---

## 编译

放入 OpenHarmony 工程：

`vendor/isoftstone/rk2206/samples/edog_project`

按通晓（RK2206）开发板文档整编烧录。

---

## 相关仓库

- 主仓：https://github.com/yangzhiyong3508/Edog_powered_by_rk2206  
- 后端：https://github.com/yangzhiyong3508/SpringBoot  
- App：https://github.com/yangzhiyong3508/Application  
