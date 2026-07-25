# EDOG 狗端固件（edog_project）

小凌派 **RK2206** + OpenHarmony LiteOS 上的 EDOG 四足控制工程（12DOF 步态、华为云 IoTDA MQTT、舵机与运行时调参）。

主仓库：[Edog_powered_by_rk2206](https://github.com/yangzhiyong3508/Edog_powered_by_rk2206)

> 源码来自 OpenHarmony 编译 Docker / 本地 export 的 `edog_project` 模块。

## 功能概览

- Wi‑Fi STA + MQTT 连接华为云 IoTDA  
- 标准命令 `motion_control`（trot / turn / stop / `*_coze` 有限步等）  
- 设备属性：机身高度差、髋收展、腿长、`speed_level` 等 runtime 调参  
- 12DOF 步态生成与逆解（`12_DOF_Version/`）  
- 工具脚本与契约测试（`tools/test_12dof_*.py`）  

## 目录结构

```
Docker_Edog/
├── BUILD.gn
├── include/                 # edog_config.h、本地配置 example
├── src/                     # main、wifi_mqtt_task
├── utils/                   # iot、motion、servo、mqtt、wifi
├── 12_DOF_Version/          # 12 自由度步态与运动学
└── tools/                   # 校验脚本与单元风格测试
```

## 本地密钥配置（必读）

**不要提交** `edog_config.local.h`。

```bash
cp include/edog_config.local.example.h include/edog_config.local.h
# 编辑填入：
# - WiFi SSID/密码
# - IoTDA 设备 ID / MQTT 用户名 / 设备密钥
# - IoTDA 设备侧接入地址
```

`.gitignore` 已忽略 `edog_config.local.h`。

## 编译环境

推荐使用 OpenHarmony LiteOS 官方/课程 Docker 镜像，在完整源码树中编译：

```text
vendor/isoftstone/rk2206/samples/edog_project
```

将本仓库内容同步到上述路径后，按小凌派/RK2206 文档执行整编，产出：

- `liteos.bin` / 烧录镜像  
- 可用桌面目录 `EDOG_12_DOF_Image` 中的打包脚本作参考  

### 从 Docker 更新源码（示例）

```bash
docker start <your_openharmony_container>
docker cp <container>:/home/openharmony/.../samples/edog_project/. ./
```

## 与云端/App 约定

- 服务 ID：`Edog`  
- 命令：`motion_control`，paras 含 `command`、`step_length_m`、`step_height_m` 等  
- 属性：`front/rear_body_height_delta_mm`、`speed_level`、髋角、腿长等  
- 后端下发见 [SpringBoot](https://github.com/yangzhiyong3508/SpringBoot) 的 `RobotMotionService` / `DogDebugService`  

## 测试

```bash
# 在已安装 Python 的环境中
python tools/test_12dof_gait_ik.py
# 更多 test_12dof_*.py 见 tools/
```

## 安全

- 禁止提交 MQTT 设备密钥、WiFi 密码、证书  
- 仅提交 `edog_config.local.example.h`  
- 烧录用 `.img` / `.bin` 默认忽略  

## 相关仓库

- 后端：[SpringBoot](https://github.com/yangzhiyong3508/SpringBoot)  
- App：[Application](https://github.com/yangzhiyong3508/Application)  
- 图传：[ESP32](https://github.com/yangzhiyong3508/ESP32)  
- 视觉：[DeepLearning](https://github.com/yangzhiyong3508/DeepLearning)  
