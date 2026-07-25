# Edog Project

## 1. 项目概述

`edog_project` 是基于 OpenHarmony LiteOS-M 和 RK2206 开发板的机器狗控制样例。当前工程包含三条主线能力：

- 运动控制：本地任务循环执行步态和转向动作。
- Wi-Fi 配网：支持 AP 配网、保存家庭路由器凭据、开机自动重连、失败回退 AP。
- MQTT 上云：在 Wi-Fi 连通后连接华为云 IoTDA，通过 MQTT 接收控制指令并上报消息。

当前代码不包含麦克风采集、ADC 音频链路或 TCP 音频任务，README 中不再保留这些旧描述。

## 2. 当前功能

### 2.1 支持的运动指令

`utils/src/iot_control.c` 当前支持以下动作命令：

- `trot`
- `trot_back`
- `turn_left`
- `turn_right`
- `left_front`
- `right_front`
- `left_back`
- `right_back`
- `stop`

兼容输入还包括：

- 数字字符串 `1`
- 数值命令 `1`
- 局部同义词，如 `lf`、`rf`、`lb`、`rb`
- 带 `_coze` 后缀的命令，当前按固定次数执行
- 可选速度字段 `speed_level` / `speedLevel` / `level`，范围 `0~4`

### 2.2 Wi-Fi / MQTT 行为

- 开机先检查是否存在已保存的路由器 Wi-Fi 凭据。
- 有凭据时优先以 `STA` 模式连接家庭网络。
- 无凭据，或多次连接失败时，进入 `AP` 配网模式。
- 只有 Wi-Fi 连通后才开始 MQTT 连接。
- MQTT 断开时优先重连 MQTT；Wi-Fi 丢失时先重连 Wi-Fi，重试失败再回到 AP 配网。
- 长按按键可清除已保存的 Wi-Fi 信息并重启。

## 3. 代码结构

### 3.1 应用入口

- `src/main.c`
  创建舵机初始化任务、运动控制任务、长按清网任务、Wi-Fi/MQTT 状态机任务。

- `src/wifi_mqtt_task.c`
  负责网络状态机。串行管理 STA 连接、AP 配网、MQTT 建链、断线重连和回退逻辑。

### 3.2 网络与平台接入

- `utils/src/wifi_tool.c`
  封装 AP 启停、HTTP 配网页、凭据保存/清除、当前 IP 查询、长按清网。

- `utils/src/iot.c`
  封装 MQTT 连接、订阅、收包处理、心跳发送和上行消息发布。

- `utils/src/wifi_portal_page.h`
  内嵌的中文配网页 HTML 内容。

### 3.3 运动控制

- `utils/src/iot_control.c`
  将云端命令映射为动作状态。

- `utils/src/motion_utils.c`
  实现动作切换、平滑过渡、停止和站立回正。

- `utils/src/gait_generate.c`
  实现步态轨迹和逆运动学计算。

- `utils/src/servo_control.c`
  通过 I2C 控制 PCA9685，进而驱动舵机。

### 3.4 配置入口

- `include/edog_config.h`
  当前工程的统一配置头文件。修改这里即可影响全局默认行为。

已集中到该头文件的关键参数包括：

- AP 名称、密码、HTTP 端口、配网页缓冲大小
- Wi-Fi 重试次数、MQTT 重试周期、心跳周期
- MQTT 主机、设备 ID、clientId、用户名、密码
- 长按清网 GPIO、按下电平、长按判定时长
- 各任务栈大小和优先级

## 4. 默认配网信息

当前默认 AP 配网参数如下：

- SSID：`eDog_Setup`
- Password：`edog1234`
- Portal 地址：`http://192.168.2.1/`

如果手机未自动弹出页面，请手动在浏览器中输入上述地址。

## 5. 运行流程

1. 系统启动后初始化舵机和运动任务。
2. Wi-Fi 任务检查是否存在已保存凭据。
3. 若存在凭据，则尝试连接家庭路由器。
4. 若没有凭据或重连多次失败，则启动 AP 和本地 HTTP 配网页。
5. 用户提交家庭 Wi-Fi 名称和密码后，设备保存配置并切回 `STA`。
6. Wi-Fi 连通后开始连接 MQTT。
7. 云端命令通过 MQTT 下发，本地解析后驱动动作任务执行。

## 6. 编译

1. 在 OpenHarmony 根目录执行 `hb set`，选择 `isoftstone-rk2206`。
2. 执行 `hb build -f` 完成全量编译。
3. 将生成固件烧录到开发板。

## 7. 硬件与软件依赖

### 7.1 硬件依赖

- RK2206 开发板
- Wi-Fi 模组
- PCA9685 舵机驱动板
- PWM 舵机
- 一个可用于长按清网的按键输入 GPIO

### 7.2 软件依赖

- OpenHarmony LiteOS-M
- LwIP
- Paho MQTT
- cJSON
- HDF / IoT 硬件接口

## 8. 说明

- `utils/src/mqtt_connect.c` 当前主要保留兼容接口，主网络流程已由 `wifi_mqtt_task.c + iot.c` 串行管理。
- 板级默认 AP/Wi-Fi 配置仍存在于底层 `config_network` 模块中，但 `edog_project` 启动后会覆盖为本项目配置。

## 9. 舵机校准 MQTT 接口

当前版本新增了两个用于校准的设备侧接口：

- 单舵机调角
- 四腿伸直姿态

这两个接口都会先停止当前步态动作，再执行校准命令，避免运动任务继续覆盖手动设定的舵机角度。

### 9.1 单舵机调角

作用：让指定通道的舵机转到目标角度，便于做机械校正。

参数约束：

- `servo_id` / `servoId` / `channel`：`0~15`
- `angle` / `degree`：`-90~90`

推荐使用的活动腿部舵机通道：

- 左前腿：`0`、`2`
- 左后腿：`4`、`6`
- 右后腿：`8`、`10`
- 右前腿：`12`、`14`

#### 自定义消息下发示例

下发 Topic：

```text
$oc/devices/{deviceId}/sys/messages/down
```

消息体：

```json
{
  "content": "servo_set",
  "servo_id": 0,
  "angle": 15
}
```

也兼容下面这种字段名：

```json
{
  "action": "servo_set",
  "channel": 14,
  "degree": -20
}
```

#### IoTDA 系统命令示例

消息体：

```json
{
  "command_name": "servo_set",
  "paras": {
    "servo_id": 0,
    "angle": 15
  }
}
```

### 9.2 四腿伸直

作用：将四条腿对应的 8 个活动舵机全部打到 `0°`，进入便于校准的直腿姿态。

注意：这里的“伸直”是校准姿态，不是步态站姿；它会把活动腿部舵机统一设置为原始 `0°`。

#### 自定义消息下发示例

```json
{
  "content": "legs_straight"
}
```

## 10. 运动调速 MQTT 接口

当前版本支持两种调速方式：

- 在动作消息里直接携带速度档位
- 使用独立的调速命令单独设置速度

速度档位范围：

- `0`：最慢
- `1`：较慢
- `2`：标准
- `3`：较快
- `4`：最快

### 10.1 动作消息携带速度

适合和 App 当前这类消息格式直接对接：

```json
{
  "content": "trot",
  "speed_level": 3,
  "speedLevel": 3
}
```

设备收到后会先把速度档位切到 `3`，再执行 `trot`。

也兼容 IoTDA 系统命令格式：

```json
{
  "command_name": "turn_left",
  "paras": {
    "speed_level": 2
  }
}
```

### 10.2 独立调速命令

支持以下命令名：

- `speed_set`
- `motion_speed`
- `set_speed_level`
- `set_motion_speed`

自定义消息示例：

```json
{
  "content": "speed_set",
  "speed_level": 4
}
```

IoTDA 系统命令示例：

```json
{
  "command_name": "speed_set",
  "paras": {
    "speedLevel": 1
  }
}
```

#### IoTDA 系统命令示例

```json
{
  "command_name": "legs_straight",
  "paras": {}
}
```

### 9.3 已兼容的命令别名

- 单舵机调角：`servo_set`、`servo_calibrate`、`servo_move`
- 四腿伸直：`legs_straight`、`straight_legs`、`dog_legs_straight`

### 9.4 系统命令返回

如果通过 IoTDA 系统命令下发，设备会返回标准命令响应：

- 成功：`result_code = 0`
- 失败：`result_code = 1`

失败时通常表示：

- 命令名不支持
- 缺少 `servo_id/channel` 或 `angle/degree`
- 舵机编号超出 `0~15`
- 角度超出 `-90~90`
