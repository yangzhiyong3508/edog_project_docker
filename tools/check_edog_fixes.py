#!/usr/bin/env python3
import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read(relpath):
    path = ROOT / relpath
    if not path.exists():
        raise FileNotFoundError(relpath)
    return path.read_text(encoding="utf-8")


def read_optional(relpath):
    path = ROOT / relpath
    if not path.exists():
        return ""
    return path.read_text(encoding="utf-8")


def fail(message):
    print(f"FAIL: {message}")
    return 1


def check_public_config_has_no_live_secret():
    config = read("include/edog_config.h")
    if "edog_config.local.h" not in config:
        return fail("edog_config.h must load ignored local IoTDA credentials")
    if re.search(r'EDOG_MQTT_DEVICE_PASSWORD\s+"[0-9a-fA-F]{32,}"', config):
        return fail("edog_config.h still contains a live-looking device password")
    return 0


def check_secret_files_are_ignored_and_removed():
    try:
        ignore = read(".gitignore")
    except FileNotFoundError:
        return fail(".gitignore missing")
    required = ["include/edog_config.local.h", "docs/Application/tmp-*"]
    for pattern in required:
        if pattern not in ignore:
            return fail(f".gitignore missing {pattern}")
    app_ignore_path = ROOT / "docs" / "Application" / ".gitignore"
    if app_ignore_path.exists():
        app_ignore = app_ignore_path.read_text(encoding="utf-8")
        if "/entry/src/main/ets/utils/IotSecrets.ets" not in app_ignore:
            return fail("docs/Application/.gitignore missing IotSecrets.ets")
    leaked = sorted((ROOT / "docs" / "Application").glob("tmp-*"))
    if leaked:
        names = ", ".join(path.name for path in leaked[:5])
        return fail(f"temporary cloud credential files still exist: {names}")
    return 0


def check_public_files_have_no_cloud_credentials():
    roots = [
        ROOT / "include",
        ROOT / "utils",
        ROOT / "src",
        ROOT / "docs" / "Application" / "entry" / "src" / "main" / "ets",
    ]
    ignored_names = {"edog_config.local.h", "IotSecrets.ets", "IotSecrets.example.ets"}
    for root in roots:
        if not root.exists():
            continue
        for path in root.rglob("*"):
            if not path.is_file() or path.name in ignored_names:
                continue
            text = path.read_text(encoding="utf-8", errors="ignore")
            if re.search(r"IAM_PASSWORD\s*:\s*string\s*=\s*['\"][^'\"]+['\"]", text):
                return fail(f"public file still hardcodes an IAM password: {path.relative_to(ROOT)}")
    return 0


def check_mqtt_callbacks_only_enqueue():
    iot_c = read("utils/src/iot.c")
    for function in ("mqtt_edog_message_arrived", "mqtt_message_arrived"):
        match = re.search(
            rf"void\s+{function}\s*\([^)]*\)\s*\{{(?P<body>.*?)\n\}}",
            iot_c,
            re.S,
        )
        if not match:
            return fail(f"{function} not found")
        body = match.group("body")
        forbidden = [
            "IotControl_HandleCommandString",
            "IotControl_HandleCommandNumber",
            "IotControl_SetSingleServoAngle",
            "IotControl_SetMotionSpeedLevel",
            "IotControl_StraightenLegs",
        ]
        for token in forbidden:
            if token in body:
                return fail(f"{function} still directly calls {token}")
    if "IotControl_EnqueueCommand" not in iot_c:
        return fail("MQTT parser must enqueue commands through IotControl_EnqueueCommand")
    return 0


def check_http_portal_limits():
    wifi_tool = read("utils/src/wifi_tool.c")
    required = [
        "SO_RCVTIMEO",
        "SO_SNDTIMEO",
        "EDOG_WIFI_HTTP_MAX_CONTENT_LENGTH",
        "413 Payload Too Large",
    ]
    for token in required:
        if token not in wifi_tool:
            return fail(f"wifi_tool.c missing {token}")
    return 0


def check_fixed_wifi_mode_without_provisioning():
    config = read("include/edog_config.h")
    wifi_task = read("src/wifi_mqtt_task.c")

    for token in [
        "EDOG_WIFI_PROVISIONING_ENABLED",
        "EDOG_WIFI_FIXED_ROUTE_SSID",
        "EDOG_WIFI_FIXED_ROUTE_PASSWORD",
    ]:
        if token not in config:
            return fail(f"edog_config.h missing fixed Wi-Fi token {token}")

    for token in [
        "ConnectFixedNetwork",
        "#if !EDOG_WIFI_PROVISIONING_ENABLED",
        "WifiTool_SaveCredentials(EDOG_WIFI_FIXED_ROUTE_SSID, EDOG_WIFI_FIXED_ROUTE_PASSWORD)",
        "fixed WiFi reconnect failed, retry later",
    ]:
        if token not in wifi_task:
            return fail(f"wifi_mqtt_task.c missing fixed Wi-Fi behavior token {token}")
    return 0


def check_motion_state_locking():
    control = read("utils/src/iot_control.c")
    required = [
        "LOS_MuxCreate",
        "LOS_MuxPend",
        "LOS_MuxPost",
        "LOS_QueueCreate",
        "LOS_QueueWriteCopy",
        "LOS_QueueReadCopy",
    ]
    for token in required:
        if token not in control:
            return fail(f"iot_control.c missing {token}")
    return 0


def check_servo_i2c_errors_are_handled():
    servo = read("utils/src/servo_control.c")
    servo_header = read("utils/include/servo_control.h")
    motion = read("12_DOF_Version/src/motion_utils_12dof.c")
    motion_header = read("utils/include/motion_utils.h")
    main_c = read("src/main.c")
    control = read("utils/src/iot_control.c")

    required_servo_tokens = [
        "#include \"iot_errno.h\"",
        "static int i2cWrite",
        "int initPCA9685(void)",
        "EdogI2cBusGuard_EnsureBusInit",
        "IOT_SUCCESS",
        "g_pca9685Initialized = 1",
        "static int setPWM",
        "int setServo(int channel, int angle)",
        "channel < 0 || channel >= EDOG_SERVO_CHANNEL_COUNT",
        "int setZero(void)",
    ]
    for token in required_servo_tokens:
        if token not in servo:
            return fail(f"servo_control.c missing {token}")

    if servo.find("g_pca9685Initialized = 1") < servo.find("i2cWrite(MODE1_REG, 0xA1)"):
        return fail("PCA9685 must be marked initialized only after all init writes succeed")

    for token in [
        "int initPCA9685(void)",
        "int setServo(int channel, int angle)",
        "int setZero(void)",
    ]:
        if token not in servo_header:
            return fail(f"servo_control.h missing {token}")

    for token in [
        "static int motionSetServo",
        "int ret = setServoCentiDeg(channel, limitedCenti)",
        "int setDogServoAngleTracked",
        "int setDogLegsStraightPose",
    ]:
        if token not in motion:
            return fail(f"motion_utils.c missing {token}")

    if "#include \"../../12_DOF_Version/include/motion_utils_12dof.h\"" not in motion_header:
        return fail("motion_utils.h must forward to the 12DOF compatibility header")

    for token in [
        "if (initPCA9685() != 0)",
        "[ServoInitTask] PCA9685 init FAILED",
    ]:
        if token not in main_c:
            return fail(f"main.c missing {token}")

    for token in [
        "if (initPCA9685() != 0)",
        "setDogServoAngleTracked(channel, angle) != 0",
        "setDogLegsStraightPose() != 0",
    ]:
        if token not in control:
            return fail(f"iot_control.c missing {token}")

    return 0


def check_app_artifacts_are_removed_and_ignored():
    ignored = read(".gitignore")
    required_patterns = [
        "docs/Application/.git/",
        "docs/Application/.idea/",
        "docs/Application/.hvigor/",
        "docs/Application/**/build/",
        "docs/Application/oh_modules/",
        "docs/Application/**/oh_modules/",
        "docs/Application/node_modules/",
        "docs/Application/**/node_modules/",
    ]
    for pattern in required_patterns:
        if pattern not in ignored:
            return fail(f".gitignore missing {pattern}")

    app_root = ROOT / "docs" / "Application"
    forbidden_names = {
        ".git",
        ".idea",
        ".hvigor",
        ".cxx",
        ".appanalyzer",
        "build",
        "oh_modules",
        "node_modules",
    }
    for path in app_root.rglob("*"):
        if path.name in forbidden_names:
            return fail(
                "generated or nested project artifact still exists: "
                f"{path.relative_to(ROOT)}"
            )

    return 0


def check_mpu6050_motion_rgb_feature():
    build = read("BUILD.gn")
    config = read("include/edog_config.h")
    main_c = read("src/main.c")
    servo = read("utils/src/servo_control.c")
    required_files = [
        "utils/include/mpu6050_motion_light.h",
        "utils/src/mpu6050_motion_light.c",
        "utils/include/i2c_bus_guard.h",
        "utils/src/i2c_bus_guard.c",
    ]
    for relpath in required_files:
        if not (ROOT / relpath).exists():
            return fail(f"missing MPU/RGB support file: {relpath}")

    mpu_header = read("utils/include/mpu6050_motion_light.h")
    mpu_impl = read("utils/src/mpu6050_motion_light.c")
    guard_header = read("utils/include/i2c_bus_guard.h")
    guard_impl = read("utils/src/i2c_bus_guard.c")

    for token in [
        "./utils/src/mpu6050_motion_light.c",
        "./utils/src/i2c_bus_guard.c",
    ]:
        if token not in build:
            return fail(f"BUILD.gn missing {token}")

    for token in [
        "EDOG_MPU_FORWARD_AXIS",
        "EDOG_MPU_FORWARD_DIRECTION",
        "EDOG_MPU_MOTION_THRESHOLD_MG",
        "EDOG_MPU_MOTION_RELEASE_MG",
        "EDOG_MPU_TILT_THRESHOLD_MG",
        "EDOG_MPU_SAMPLE_INTERVAL_MS",
        "EDOG_MPU_INIT_RETRY_MS",
        "EDOG_RGB_ROTATE_INTERVAL_MS",
        "EDOG_RGB_ACTIVE_HIGH",
        "EDOG_TASK_MPU_LIGHT_STACK_SIZE",
        "EDOG_TASK_MPU_LIGHT_PRIORITY",
    ]:
        if token not in config:
            return fail(f"edog_config.h missing {token}")

    for token in [
        "EDOG_MPU_ACCEL_DELTA_THRESHOLD_MG",
    ]:
        if token in config:
            return fail(f"edog_config.h still keeps any-motion MPU token {token}")

    for token in [
        "#include \"../utils/include/mpu6050_motion_light.h\"",
        "#include \"../utils/include/i2c_bus_guard.h\"",
        "EdogI2cBusGuard_Init()",
        "MpuMotionLightTask",
        "mpu_motion_light_task",
    ]:
        if token not in main_c:
            return fail(f"main.c missing {token}")

    for token in [
        "EdogI2cBusGuard_Init",
        "EdogI2cBusGuard_Lock",
        "EdogI2cBusGuard_Unlock",
    ]:
        if token not in guard_header or token not in guard_impl:
            return fail(f"i2c_bus_guard missing {token}")

    if "EdogI2cBusGuard_Lock" not in servo or "EdogI2cBusGuard_Unlock" not in servo:
        return fail("servo_control.c must use the shared I2C bus guard")

    for token in [
        "void MpuMotionLightTask(void)",
        "MPU6050_I2C_ADDRESS    0x68",
        "MPU6050_REG_ACCEL_XOUT_H",
        "MPU6050_REG_WHO_AM_I",
        "SelectForwardAxisMg",
        "SelectForwardBaselineMg",
        "IsForwardMotionDetected",
        "IsForwardMotionReleased",
        "IsBodyTilted",
        "Rgb_ShowTiltState",
        "Rgb_ShowBalanceState",
        "EDOG_MPU_FORWARD_AXIS",
        "EDOG_MPU_FORWARD_DIRECTION",
        "EDOG_MPU_MOTION_THRESHOLD_MG",
        "EDOG_MPU_MOTION_RELEASE_MG",
        "EDOG_MPU_TILT_THRESHOLD_MG",
        "EDOG_RGB_ACTIVE_HIGH ? IOT_GPIO_VALUE1 : IOT_GPIO_VALUE0",
        "EDOG_RGB_ACTIVE_HIGH ? IOT_GPIO_VALUE0 : IOT_GPIO_VALUE1",
        "EdogI2cBusGuard_EnsureBusInit",
        "while (MpuMotionLight_Init() != 0 || MpuMotionLight_CalibrateBaseline() != 0)",
        "motion RGB task unavailable, RGB off, retry later",
        "LOS_Msleep(EDOG_MPU_INIT_RETRY_MS)",
        "static const RgbColor g_rainbow",
        "{1, 0, 0}",
        "{0, 1, 0}",
        "{0, 0, 1}",
        "Rgb_SetColor",
        "forwardMotionArmed",
        "forwardMotionArmed = 0",
        "forwardMotionArmed = 1",
    ]:
        if token not in mpu_header + mpu_impl:
            return fail(f"mpu6050 motion light feature missing {token}")

    if "if (IsForwardMotionDetected(x, y, z))" in mpu_impl:
        return fail("forward motion must be edge-triggered, not continuously rearmed")

    active_priority_index = mpu_impl.find("if (activeLoops > 0)")
    tilt_status_index = mpu_impl.find("Rgb_ShowTiltState();")
    balance_status_index = mpu_impl.find("Rgb_ShowBalanceState();")
    if active_priority_index < 0 or tilt_status_index < 0 or balance_status_index < 0:
        return fail("MPU task must call tilt/balance RGB status helpers after active tricolor handling")
    if active_priority_index > tilt_status_index or active_priority_index > balance_status_index:
        return fail("forward tricolor effect must take priority over tilt/balance status light")

    if "LOS_TaskDelete(LOS_CurTaskIDGet())" in mpu_impl:
        return fail("MPU motion RGB task must keep retrying and holding RGB off instead of deleting itself")

    for token in [
        "IsAccelerationChangeDetected",
        "EDOG_MPU_ACCEL_DELTA_THRESHOLD_MG",
        "Acceleration change detected",
        "{1, 1, 0}",
        "{0, 1, 1}",
        "{1, 0, 1}",
        "{1, 1, 1}",
    ]:
        if token in mpu_impl:
            return fail(f"mpu6050 motion light still keeps any-motion or non-tricolor logic {token}")

    return 0


def check_12_dof_version_layout_and_servo_mapping():
    build = read("BUILD.gn")
    config = read("include/edog_config.h")
    servo = read("utils/src/servo_control.c")
    servo_header = read("utils/include/servo_control.h")
    motion = read("12_DOF_Version/src/motion_utils_12dof.c")
    motion_header = read("12_DOF_Version/include/motion_utils_12dof.h")
    gait = read("12_DOF_Version/src/gait_generate_12dof.c")
    gait_header = read("12_DOF_Version/include/gait_generate_12dof.h")

    if not (ROOT / "8_DOF_Version").is_dir():
        return fail("8_DOF_Version archive directory missing")
    if "12_DOF_Version/src/motion_utils_12dof.c" not in build:
        return fail("BUILD.gn must compile the 12_DOF_Version motion implementation")
    if "12_DOF_Version/src/gait_generate_12dof.c" not in build:
        return fail("BUILD.gn must compile the 12_DOF_Version gait implementation")
    if "./utils/src/motion_utils.c" in build or "./utils/src/gait_generate.c" in build:
        return fail("BUILD.gn must not compile the old 8DOF motion/gait files for 12DOF build")

    required_config_tokens = [
        "EDOG_SERVO_ANGLE_RANGE_DEG             180",
        "EDOG_SERVO_MIN_ANGLE                   0",
        "EDOG_SERVO_MAX_ANGLE                   180",
        "EDOG_SERVO_PULSE_MIN_US                500",
        "EDOG_SERVO_PULSE_MAX_US                2500",
        "EDOG_SERVO_CENTER_ANGLE                90",
    ]
    for token in required_config_tokens:
        if token not in config:
            return fail(f"edog_config.h missing 180-degree servo safety constant: {token}")

    required_servo_tokens = [
        "EDOG_SERVO_ANGLE_RANGE_DEG",
        "EDOG_SERVO_PULSE_MIN_US",
        "EDOG_SERVO_PULSE_MAX_US",
        "EDOG_SERVO_PWM_FREQUENCY_HZ",
        "static int centiDegToPwmCount(int centiDeg)",
        "pulseNumeratorUs =",
        "return setServoPwmCount(channel, count)",
        "int setServoPulseUs(int channel, int pulseUs)",
    ]
    for token in required_servo_tokens:
        if token not in servo:
            return fail(f"servo_control.c missing 180-degree pulse mapping token: {token}")
    if "int setServoPulseUs(int channel, int pulseUs)" not in servo_header:
        return fail("servo_control.h must expose setServoPulseUs for calibration")

    expected_channels = {
        "LF_HIP": 0,
        "LF_THIGH": 1,
        "LF_CALF": 2,
        "RF_HIP": 4,
        "RF_THIGH": 5,
        "RF_CALF": 6,
        "LB_HIP": 8,
        "LB_THIGH": 9,
        "LB_CALF": 10,
        "RB_HIP": 12,
        "RB_THIGH": 13,
        "RB_CALF": 14,
    }
    for name, value in expected_channels.items():
        if not re.search(rf"#define\s+{name}\s+{value}\b", motion_header):
            return fail(f"12DOF channel mapping missing or wrong: {name}={value}")
    for token in [
        "EDOG_12DOF_ACTIVE_SERVO_COUNT 12",
        "EDOG_12DOF_LEG_COUNT 4",
        "EDOG_12DOF_JOINTS_PER_LEG 3",
        "LF_RESERVED 3",
        "RF_RESERVED 7",
        "LB_RESERVED 11",
        "RB_RESERVED 15",
    ]:
        name, value = token.rsplit(" ", 1)
        if not re.search(rf"#define\s+{name}\s+{value}\b", motion_header):
            return fail(f"12DOF header missing {token}")

    for name, value in {
        "SPOTMICRO_THIGH_LENGTH_MM": "107.0",
        "SPOTMICRO_CALF_LENGTH_MM": "135.0",
        "SPOTMICRO_PRECISE_THIGH_LENGTH_MM": "111.2",
        "SPOTMICRO_PRECISE_CALF_LENGTH_MM": "118.5",
    }.items():
        if not re.search(rf"#define\s+{name}\s+{re.escape(value)}\b", gait_header):
            return fail(f"12DOF IK/gait missing {name} {value}")
    for token in [
        "EDOG_12DOF_USE_PRECISE_SPOTMICRO_LINKS",
        "Edog12Dof_IK",
        "Edog12Dof_GenerateTrotTable",
        "Edog12Dof_GenerateDirectionalTrotTable",
        "hipAngleDeg",
        "femurAngleDeg",
        "tibiaAngleDeg",
    ]:
        if token not in gait + gait_header + motion:
            return fail(f"12DOF IK/gait missing {token}")
    if "#define EDOG_12DOF_TROT_FRAME_COUNT 30" not in gait_header:
        return fail("12DOF gait table must use 30 samples")
    if "Edog12Dof_GenerateTrotTable20" in gait + gait_header + motion:
        return fail("12DOF gait table must not use the old 20-sample API")
    if not re.search(r"#define\s+EDOG_12DOF_GAIT_FRAME_FPS\s+50\b", config):
        return fail("edog_config.h missing 50FPS gait frame config EDOG_12DOF_GAIT_FRAME_FPS")
    if not re.search(r"#define\s+EDOG_12DOF_GAIT_FRAME_PERIOD_US\s+\(1000000\s*/\s*EDOG_12DOF_GAIT_FRAME_FPS\)", config):
        return fail("edog_config.h missing gait frame period config EDOG_12DOF_GAIT_FRAME_PERIOD_US")
    for name, value in {
        "EDOG_SERVO_TRAPEZOID_MAX_SPEED_CENTI_PER_FRAME": "420",
        "EDOG_SERVO_TRAPEZOID_MAX_ACCEL_CENTI_PER_FRAME2": "140",
        "EDOG_SERVO_TRAPEZOID_TARGET_EPS_CENTI": "2",
    }.items():
        if not re.search(rf"#define\s+{name}\s+{value}\b", config):
            return fail(f"edog_config.h missing gait servo trapezoid config {name} {value}")
    for token in [
        "static void sleepUntilStopOrTimeout(uint64_t microseconds)",
        "static uint64_t getSpeedAdjustedFramePeriodUs(void)",
        "static const uint8_t framePeriodPercent[7] = {140, 100, 75, 45, 35, 30, 30}",
        "return ((uint64_t)EDOG_12DOF_GAIT_FRAME_PERIOD_US * percent + 50) / 100",
        "static void sleepRemainingFrameTime(uint64_t usedUs)",
        "static void addFrameUsedTime(uint64_t *usedUs, int elapsedUs)",
        "const uint64_t framePeriodUs = getSpeedAdjustedFramePeriodUs()",
        "sleepUntilStopOrTimeout(framePeriodUs - usedUs)",
        "addFrameUsedTime(&usedUs, setLegAnglesStaggered",
        "sleepRemainingFrameTime(usedUs)",
    ]:
        if token not in motion:
            return fail(f"12DOF gait loop missing speed-adjusted frame pacing token {token}")
    for token in [
        "#define EDOG_12DOF_TURN_CYCLE_SLOWDOWN_NUM     8",
        "#define EDOG_12DOF_TURN_CYCLE_SLOWDOWN_DEN     5",
    ]:
        if token not in config:
            return fail(f"edog_config.h missing turn cycle slowdown token {token}")
    for token in [
        "static int isTurnTableMode(int mode)",
        "if (isTurnTableMode(mode))",
        "requestedCycleMs * EDOG_12DOF_TURN_CYCLE_SLOWDOWN_NUM",
        "EDOG_12DOF_TURN_CYCLE_SLOWDOWN_DEN - 1",
    ]:
        if token not in motion:
            return fail(f"12DOF turn table missing slowdown token {token}")
    if "safeSleep(10000)" in motion or "safeSleep(20000)" in motion:
        return fail("12DOF trot frame delay must use speed-adjusted frame pacing, not fixed old safeSleep values")
    for token in [
        "static int centiDegToPwmCount(int centiDeg)",
        "return setServoPwmCount(channel, count)",
        "motionSetServoSmoothCenti(leg->hip, angles->hipAngleCentiDeg)",
        "motionSetServoSmoothCenti(leg->thigh, angles->femurAngleCentiDeg)",
        "motionSetServoSmoothCenti(leg->calf, angles->tibiaAngleCentiDeg)",
    ]:
        if token not in servo + motion:
            return fail(f"12DOF centi-degree precision path missing token {token}")
    if "pulseUs = EDOG_SERVO_PULSE_MIN_US +" in servo:
        return fail("setServoCentiDeg must not truncate centi-degree angles to integer microseconds")
    for token in [
        "static int servoVelocityCentiPerFrame[EDOG_SERVO_CHANNEL_COUNT]",
        "static void resetServoMotionProfile(int channel, int limited)",
        "resetServoMotionProfile(channel, limited)",
        "static int writeServoProfileAngleCenti(int channel, int centiAngle)",
        "static int planServoTrapezoidStepCenti(int channel, int targetCenti)",
        "brakeDistance = (long long)velocityAbs * velocityAbs /",
        "if ((long long)distanceAbs <= brakeDistance)",
        "nextVelocity = velocity + direction * EDOG_SERVO_TRAPEZOID_MAX_ACCEL_CENTI_PER_FRAME2",
        "EDOG_SERVO_TRAPEZOID_MAX_SPEED_CENTI_PER_FRAME",
        "servoVelocityCentiPerFrame[channel] = next - current",
        "static int writeServoPhysicalAngle(int channel, int angle)",
        "static int motionSetServoPhysicalSmooth(int channel, int angle)",
        "static int setLegAnglesStaggeredSmooth",
        "motionSetServoPhysicalSmooth(channel, jointDeltaToServoAngle(channel, jointDeltaDeg))",
        "return writeServoPhysicalAngle(channel, applyServoCenterTrim(channel, clampServoAngle(angle)))",
        "return writeServoProfileAngleCenti(channel, planServoTrapezoidStepCenti(channel, target))",
        "return setLegAnglesStaggeredSmooth(&g_legs[legIndex], &outputAngles, delayUs)",
        "setGaitLegAnglesStaggeredSmooth(0",
        "setLegAnglesStaggeredSmooth(&g_legs[leg]",
    ]:
        if token not in motion:
            return fail(f"12DOF gait servo output must use trapezoid profile token {token}")
    if "step = delta * EDOG_SERVO_GAIT_SMOOTHING_NUM / EDOG_SERVO_GAIT_SMOOTHING_DEN" in motion:
        return fail("12DOF gait servo output must not use old proportional slew smoothing")

    for token in [
        "static const Edog12DofJoint g_activeJoints[EDOG_12DOF_ACTIVE_SERVO_COUNT]",
        "static const int g_servoDirection[EDOG_SERVO_CHANNEL_COUNT]",
        "EDOG_SERVO_CENTER_ANGLE",
        "setDogServoAngleTracked",
        "setDogLegsStraightPose",
        "init_dog",
        "trot_cycle",
    ]:
        if token not in motion:
            return fail(f"12DOF motion implementation missing {token}")

    expected_directions = {
        0: 1,
        1: -1,
        2: -1,
        4: -1,
        5: 1,
        6: 1,
        8: 1,
        9: -1,
        10: -1,
        12: -1,
        13: 1,
        14: 1,
    }
    direction_match = re.search(
        r"static\s+const\s+int\s+g_servoDirection\s*\[\s*EDOG_SERVO_CHANNEL_COUNT\s*\]\s*=\s*\{(?P<body>.*?)\};",
        motion,
        re.S,
    )
    if not direction_match:
        return fail("12DOF motion implementation must define g_servoDirection[EDOG_SERVO_CHANNEL_COUNT]")
    body = re.sub(r"/\*.*?\*/", "", direction_match.group("body"), flags=re.S)
    values = [int(value) for value in re.findall(r"[-+]?\d+", body)]
    if len(values) != 16:
        return fail(f"g_servoDirection must contain 16 entries, got {len(values)}")
    for channel, expected in expected_directions.items():
        if values[channel] != expected:
            return fail(f"g_servoDirection[{channel}] must be {expected}, got {values[channel]}")
    for reserved in (3, 7, 11, 15):
        if values[reserved] != 0:
            return fail(f"reserved channel {reserved} direction must be 0")
    forbidden_motion_tokens = [
        "mirrorFemur",
        "mirrorTibia",
        "mirrorAroundCenter",
    ]
    for token in forbidden_motion_tokens:
        if token in motion:
            return fail(f"12DOF direction mapping must not use old mirror token {token}")
    if "EDOG_SERVO_CENTER_ANGLE + g_servoDirection[channel] * jointDeltaDeg" not in motion:
        return fail("servo output must use center + g_servoDirection[channel] * jointDeltaDeg")
    for name, value in {
        "EDOG_SERVO_STARTUP_STEP_DELAY_US": "120000",
        "EDOG_SERVO_MOTION_STEP_DELAY_US": "0",
        "EDOG_SERVO_STOP_STEP_DELAY_US": "6000",
    }.items():
        if not re.search(rf"#define\s+{name}\s+{value}\b", config):
            return fail(f"edog_config.h missing staggered servo delay {name} {value}")
    for token in [
        "static const int g_startupLegOrder[EDOG_12DOF_LEG_COUNT] = {2, 3, 0, 1}",
        "static int setLegAnglesStaggered",
        "static void apply8DofTableFrameStaggered",
        "static void apply8DofTurnFrameStaggered",
        "static int g_servoCenterTrim[EDOG_SERVO_CHANNEL_COUNT]",
        "static int applyServoCenterTrim",
        "angle + g_servoCenterTrim[channel]",
        "int setDogServoCenterTrim",
        "EDOG_SERVO_STARTUP_STEP_DELAY_US",
        "EDOG_SERVO_MOTION_STEP_DELAY_US",
        "EDOG_SERVO_STOP_STEP_DELAY_US",
    ]:
        if token not in motion:
            return fail(f"12DOF staggered servo output missing {token}")
    for token in [
        "int setDogServoCenterTrim(int channel, int trimDeg)",
        "int loadDogServoCenterTrims(void)",
        "int saveDogServoCenterTrim(int channel, int trimDeg)",
        "int getDogServoCenterTrim(int channel)",
        "int single_leg_gait_cycle(int leg_index, double step_length, double step_height)",
        "int leg_group_gait_cycle(int leg_mask, double step_length, double step_height)",
    ]:
        if token not in motion_header:
            return fail(f"12DOF motion header missing {token}")
    for token in [
        "int single_leg_gait_cycle(int leg_index, double step_length, double step_height)",
        "return leg_group_gait_cycle(1 << leg_index, step_length, step_height);",
        "int leg_group_gait_cycle(int leg_mask, double step_length, double step_height)",
        "setGaitLegAnglesStaggeredSmooth(leg, &gait[i]",
        "#include \"kv_store.h\"",
        "UtilsGetValue(key, value, sizeof(value))",
        "UtilsSetValue(key, value)",
        "edog.trim.%02d",
    ]:
        if token not in motion:
            return fail(f"12DOF motion implementation missing leg gait token {token}")
    control = read("utils/src/iot_control.c")
    control_header = read("utils/include/iot_control.h")
    iot = read("utils/src/iot.c")
    for token in [
        "IOT_CONTROL_COMMAND_SERVO_BATCH_SET",
        "IOT_CONTROL_COMMAND_SERVO_TRIM_SET",
        "IOT_CONTROL_COMMAND_SERVO_CALIBRATION_REPORT",
        "IOT_CONTROL_COMMAND_LEG_GAIT",
        "bool IotControl_SetServoBatchAngles",
        "bool IotControl_SetServoCenterTrim",
        "bool IotControl_ReportServoCalibration",
        "bool IotControl_RunSingleLegGait",
        "bool IotControl_RunLegGait",
        "MOTION_CMD_SINGLE_LEG_LF",
        "MOTION_CMD_LEG_GROUP",
    ]:
        if token not in control + control_header:
            return fail(f"IoT control layer missing servo calibration/batch token {token}")
    for token in [
        "servo_batch",
        "servo_trim",
        "servo_calibration_read",
        "single_leg_gait",
        "leg_gait",
        "leg_mask",
        "ParseServoBatchCommand",
        "EnqueueServoBatchCommand",
        "EnqueueServoTrimCommand",
        "EnqueueServoCalibrationReportCommand",
        "EnqueueLegGaitCommand",
        "LegMaskFromString",
        "ApplySingleLegGait",
    ]:
        if token not in iot:
            return fail(f"MQTT parser missing servo calibration/batch token {token}")
    if "//utils/native/lite/kv_store:kv_store" not in read("BUILD.gn"):
        return fail("edog_project build must depend on kv_store for persistent servo calibration")
    if "loadDogServoCenterTrims()" not in read("src/main.c"):
        return fail("servo init must load persisted servo trims before standing pose")
    init_match_for_power = re.search(r"void\s+init_dog\s*\([^)]*\)\s*\{(?P<body>.*?)\n\}", motion, re.S)
    if not init_match_for_power:
        return fail("12DOF motion implementation missing init_dog body")
    init_power_body = init_match_for_power.group("body")
    if "for (int i = 0; i < EDOG_12DOF_LEG_COUNT; i++)" not in init_power_body:
        return fail("init_dog must iterate startup leg order for staggered startup")
    if "int legIndex = g_startupLegOrder[i]" not in init_power_body:
        return fail("init_dog must use g_startupLegOrder")
    if ("setLegAnglesStaggered(&g_legs[legIndex], &g_standJointAngles, EDOG_SERVO_STARTUP_STEP_DELAY_US)" not in init_power_body and
        "setLegAnglesStaggered(&g_legs[legIndex], &standFrame[legIndex], EDOG_SERVO_STARTUP_STEP_DELAY_US)" not in init_power_body):
        return fail("init_dog must stagger each startup servo write")
    if "setLegAngles(&g_legs[0]" in init_power_body:
        return fail("init_dog must not use direct setLegAngles during startup")
    for function_name in ("apply8DofTableFrameStaggered", "apply8DofTurnFrameStaggered"):
        apply_match = re.search(
            rf"static\s+void\s+{function_name}\s*\([^)]*\)\s*\{{(?P<body>.*?)\n\}}",
            motion,
            re.S,
        )
        if not apply_match:
            return fail(f"12DOF motion implementation missing {function_name} body")
        apply_body = apply_match.group("body")
        if "setGaitLegAnglesStaggeredSmooth" not in apply_body:
            return fail(f"{function_name} must use the gait-layer smooth leg servo writer")
        if "EDOG_SERVO_MOTION_STEP_DELAY_US" not in apply_body:
            return fail(f"{function_name} must use EDOG_SERVO_MOTION_STEP_DELAY_US")
        if "sleepRemainingFrameTime(usedUs)" not in apply_body:
            return fail(f"{function_name} must preserve speed-adjusted frame pacing")
    for token in [
        "#define EDOG_12DOF_TROT_SWING_PORTION 0.38",
        "#define EDOG_12DOF_CRAWL_SWING_PORTION 0.20",
        "#define EDOG_12DOF_CRAWL_PHASE_SPACING 0.25",
        "#define EDOG_12DOF_CRAWL_SUPPORT_CALF_BIAS_DEG -4",
        "#define EDOG_12DOF_CRAWL_BALANCE_HIP_BIAS_DEG 2",
    ]:
        compact_config = re.sub(r"\s+", " ", config)
        if token not in compact_config:
            return fail(f"12DOF crawl gait config missing {token}")
    for token in [
        "static const int g_crawlSwingOrder[EDOG_12DOF_LEG_COUNT] = {0, 3, 1, 2}",
        "static void applyCrawlBalanceBias",
        "static int staticCrawlNextSwingLeg",
        "static int spotMicroCrawlCycleInternal",
        "static double pyAppleCrawlCycloidProgress",
        "static double pyAppleCrawlCycloidLift",
        "pyAppleCrawlCycloidProgress(t)",
        "pyAppleCrawlCycloidLift(swingT)",
        "static double legacyPupperSwingLift",
        "legacyPupperSwingLift(swingT)",
        "applyHipWeightShiftBias(frame, swingState, preShiftLeg",
        "applyRealtimeBalanceBias(frame, swingState)",
        "typedef Edog12DofJointAngles Edog12DofGaitSet",
        "buildDirectionalGaitSet",
        "static void buildScaledGaitSet",
        "double sideStep",
        "double yawStep",
        "gaitSet[0]",
        "gaitSet[1]",
        "gaitSet[2]",
        "gaitSet[3]",
    ]:
        if token not in motion:
            return fail(f"12DOF crawl gait stabilization missing {token}")
    for token in [
        "spotMicroCrawlSwingLift",
        "spotMicroCrawlBezier",
    ]:
        if token in motion:
            return fail(f"12DOF crawl gait must not use old SpotMicro swing helper {token}")
    for token in [
        "#define EDOG_12DOF_HARDWARE_FORWARD_SIGN -1.0",
        "#define EDOG_12DOF_REALTIME_SERVO_STEP_DELAY_US 100",
        "#define EDOG_STATIC_CRAWL_NUM_PHASES 8",
        "#define EDOG_12DOF_SWING_FORWARD_SCALE_PERCENT",
        "#define EDOG_12DOF_STANCE_PUSH_SCALE_PERCENT",
        "#define EDOG_12DOF_SWING_FORWARD_DELAY_PERCENT",
        "#define EDOG_12DOF_SWING_FORWARD_COMPLETE_PERCENT",
    ]:
        compact_config = re.sub(r"\s+", " ", config)
        if token not in compact_config:
            return fail(f"12DOF realtime gait config missing {token}")
    for token in [
        "typedef struct {\n    double vxMps;",
        "typedef struct {\n    Edog12DofFootPoint foot",
        "static void initRealtimeGaitState",
        "static void updateRealtimeStanceFoot",
        "static void updateRealtimeSwingFoot",
        "static int runRealtimeGaitCycle",
        "static double pupperCommandDeltaMm",
        "static double pupperSwingHeight",
        "touchdown.xMm",
        "yawStepMm",
        "swingLift",
        "hardwareStrideMm = EDOG_12DOF_HARDWARE_FORWARD_SIGN * deltaXmm",
        "hardwareDx = EDOG_12DOF_HARDWARE_FORWARD_SIGN * dx",
        "static const unsigned char g_staticCrawlContactPhases[EDOG_12DOF_LEG_COUNT][EDOG_STATIC_CRAWL_NUM_PHASES]",
        "static int realtimePhaseIndex",
        "static int realtimeLegIsContact",
        "static int spotMicroCrawlCycleInternal",
        "return trotCycleInternal(step_length, step_height, 0, 100, 100);",
        "return trotCycleInternal(step_length, step_height, 1, 100, 100);",
        "return runServoGaitTableCycle(EDOG_TABLE_MODE_TURN_RIGHT, step_length, step_height, 0x0F);",
        "return runServoGaitTableCycle(EDOG_TABLE_MODE_TURN_LEFT, step_length, step_height, 0x0F);",
        "return trotCycleInternal(0.0, step_height, 0, 100, 100);",
        "arc gait not supported",
        "Edog12Dof_IK(&state->leg[leg].foot",
    ]:
        if token not in motion:
            return fail(f"12DOF realtime gait controller missing {token}")
    for token in [
        "#define EDOG_12DOF_STAND_HIP_DELTA_DEG 0",
        "#define EDOG_12DOF_STAND_THIGH_DELTA_DEG -61",
        "#define EDOG_12DOF_STAND_CALF_DELTA_DEG 20",
    ]:
        if token not in gait_header:
            return fail(f"12DOF standing constants missing from gait header: {token}")
    if "static const Edog12DofJointAngles g_standJointAngles" not in motion:
        return fail("12DOF standing initialization missing static const Edog12DofJointAngles g_standJointAngles")
    for token in [
        "static double bezierQuintic",
        "static double bezierSwingLift",
        "static Edog12DofFootPoint quinticBezierSwingFoot",
        "static double pyAppleCycloidProgress",
        "static double pyAppleCycloidLift",
        "static Edog12DofFootPoint pyAppleCycloidSwingFoot",
        "#if EDOG_12DOF_TROT_TRAJECTORY_CYCLOID",
        "static int clampJointStepCenti",
        "static void limitGaitTableJointStepCenti",
        "EDOG_12DOF_GAIT_MAX_JOINT_STEP_CENTI",
        "static double signedStepMmWithMinimum",
        "static double clampSignedStepMm",
        "EDOG_12DOF_GAIT_FRAME_COUNT",
        "Edog12Dof_ReferencePhaseForFrame",
        "Edog12Dof_ReferenceSwingEnvelopeForPhase",
        "Edog12Dof_SampleReferenceTrotFootPoint",
        "double strideMm = signedStepMmWithMinimum",
        "double sideMm = clampSignedStepMm",
        "double yawStep = clampSignedStepMm",
        "double lateralTargetMm",
        "double liftMm = minimumLiftMm",
        "Edog12DofFootPoint foot",
        "Edog12Dof_FootYOnHipPlaneForLeg",
        "foot.yMm = Edog12Dof_FootYOnHipPlaneForLeg(legIndex, foot.zMm) + yOffset",
        "double swingT = phase / swingPortion",
        "foot = defaultSwingFoot",
        "effectiveLiftMm += EDOG_12DOF_REAR_SWING_LIFT_EXTRA_MM",
        "foot.xMm -= bodyShiftMm",
        "double stanceT = (phase - swingPortion) / stancePortion",
        "double hardwareStrideMm = EDOG_12DOF_HARDWARE_FORWARD_SIGN * strideMm",
        "foot.xMm += xOffset",
        "Edog12Dof_IK(&foot, isRightLeg, &gaitTable[k])",
    ]:
        if token not in gait:
            return fail(f"12DOF foot-space IK gait generation missing {token}")
    if "foot.yMm += yOffset" in gait:
        return fail("12DOF gait generator must recompute foot.yMm from the hip plane after z changes")
    if "foot.zMm -= lift" in gait or "foot.zMm += press" in gait:
        return fail("12DOF gait generator must set z first, then project y from the hip plane")
    for token in [
        "#define EDOG_12DOF_COMMAND_STEP_LENGTH_M       0.01",
        "#define EDOG_12DOF_COMMAND_STEP_HEIGHT_M       0.003",
        "#define EDOG_12DOF_TROT_TRAJECTORY_CYCLOID     1",
        "#define EDOG_12DOF_TROT_BODY_X_SHIFT_MM        5.0",
        "#define EDOG_12DOF_DEFAULT_FOOT_X_MM           -150.0",
        "#define EDOG_12DOF_REAR_FOOT_BACK_OFFSET_MM    0.0",
        "#define EDOG_12DOF_DEFAULT_FOOT_Y_MM           0.0",
        "#define EDOG_12DOF_DEFAULT_FOOT_Z_MM           140.0",
        "#define EDOG_12DOF_REAR_FOOT_UP_OFFSET_MM      0.0",
        "#define EDOG_12DOF_BODY_HEIGHT_DELTA_MIN_MM    -40.0",
        "#define EDOG_12DOF_BODY_HEIGHT_DELTA_MAX_MM    40.0",
        "#define EDOG_12DOF_HARDWARE_FORWARD_SIGN       -1.0",
        "#define EDOG_12DOF_MIN_STRIDE_MM               1.0",
        "#define EDOG_12DOF_MIN_LIFT_MM                 0.0",
        "#define EDOG_12DOF_MAX_SIDE_MM                 30.0",
        "#define EDOG_12DOF_MAX_YAW_MM                  28.0",
        "#define EDOG_12DOF_STANCE_PRESS_MM             3.0",
        "#define EDOG_12DOF_GAIT_MAX_JOINT_STEP_CENTI   360",
        "#define EDOG_PUPPER_MIN_LIFT_MM                0.0",
    ]:
        if token not in config:
            return fail(f"12DOF gait config missing from edog_config.h: {token}")
    for token in [
        "double Edog12Dof_DefaultFootXForLeg(int isFrontLeg)",
        "return isFrontLeg ? EDOG_12DOF_DEFAULT_FOOT_X_MM :",
        "EDOG_12DOF_DEFAULT_FOOT_X_MM - EDOG_12DOF_REAR_FOOT_BACK_OFFSET_MM",
    ]:
        if token not in gait:
            return fail(f"12DOF rear-foot default-X compensation missing {token}")
    if "Edog12Dof_DefaultFootXForLeg(isFrontLeg)" not in gait:
        return fail("12DOF table gait must use rear-foot default-X compensation")
    if "Edog12Dof_DefaultFootXForLeg(leg == 0 || leg == 1)" not in motion:
        return fail("12DOF runtime default foot point must use rear-foot default-X compensation")
    for token in [
        "double Edog12Dof_DefaultFootZForLeg(int isFrontLeg)",
        "baseZMm = isFrontLeg ? EDOG_12DOF_DEFAULT_FOOT_Z_MM :",
        "EDOG_12DOF_DEFAULT_FOOT_Z_MM - EDOG_12DOF_REAR_FOOT_UP_OFFSET_MM",
        "return baseZMm + g_runtimeFrontFootZDeltaMm",
        "return baseZMm + g_runtimeRearFootZDeltaMm",
    ]:
        if token not in gait:
            return fail(f"12DOF rear-foot default-Z compensation missing {token}")
    for token in [
        "Edog12Dof_SetRuntimeFootZDeltas",
        "Edog12Dof_GetRuntimeFootZDeltas",
        "g_runtimeFrontFootZDeltaMm",
        "g_runtimeRearFootZDeltaMm",
        "front_body_height_delta_mm",
        "rear_body_height_delta_mm",
    ]:
        combined = gait + gait_header + motion + iot + iot_control + iot_header
        if token not in combined:
            return fail(f"12DOF runtime front/rear body-height delta support missing {token}")
    if "Edog12Dof_DefaultFootZForLeg(isFrontLeg)" not in gait:
        return fail("12DOF table gait must use rear-foot default-Z compensation")
    if "Edog12Dof_DefaultFootZForLeg(leg == 0 || leg == 1)" not in motion:
        return fail("12DOF runtime default foot point must use rear-foot default-Z compensation")
    if "double Edog12Dof_FootYOnHipPlane(int isRightLeg, double zMm)" not in gait_header:
        return fail("12DOF gait header must expose hip-plane Y projection helper")
    if "EDOG_12DOF_8DOF_REFERENCE_KEY_FRAME_COUNT" not in gait_header:
        return fail("12DOF gait header must retain 8DOF reference key-frame compatibility macro")
    if "double Edog12Dof_FootYOnHipPlane(int isRightLeg, double zMm)" not in gait:
        return fail("12DOF gait source must implement hip-plane Y projection helper")
    if "double Edog12Dof_FootYOnHipPlaneForLeg(int legIndex, double zMm)" not in gait_header:
        return fail("12DOF gait header must expose per-leg hip-plane Y projection helper")
    if "double Edog12Dof_FootYOnHipPlaneForLeg(int legIndex, double zMm)" not in gait:
        return fail("12DOF gait source must implement per-leg hip-plane Y projection helper")
    if "Edog12Dof_DefaultFootZForLeg(legIndex == 0 || legIndex == 1)" not in gait:
        return fail("12DOF default foot-Y must use each leg's own default Z")
    if "Edog12Dof_FootYOnHipPlaneForLeg(leg, foot->zMm)" not in motion:
        return fail("12DOF IMU Z compensation must reproject foot Y onto the hip plane")
    if "legState->foot.yMm = Edog12Dof_FootYOnHipPlaneForLeg(leg, legState->foot.zMm)" not in motion:
        return fail("12DOF realtime swing must reproject foot Y after per-leg default foot-Z changes")
    for token in [
        "#define EDOG_12DOF_MAX_STRIDE_MM",
        "#define EDOG_12DOF_MAX_LIFT_MM",
        "#define EDOG_12DOF_REALTIME_MAX_FORWARD_MM",
        "#define EDOG_12DOF_REALTIME_LEGACY_MAX_LIFT_MM",
        "#define EDOG_PUPPER_MAX_LIFT_MM",
    ]:
        if token in config:
            return fail(f"12DOF gait upper-limit config should be removed: {token}")
    for token in [
        "EDOG_12DOF_MAX_STRIDE_MM",
        "EDOG_12DOF_MAX_LIFT_MM",
    ]:
        if token in gait:
            return fail(f"12DOF gait generator must not cap stride/lift with {token}")
    for token in [
        "EDOG_12DOF_REALTIME_MAX_FORWARD_MM",
        "EDOG_12DOF_REALTIME_LEGACY_MAX_LIFT_MM",
        "EDOG_PUPPER_MAX_LIFT_MM",
    ]:
        if token in motion:
            return fail(f"12DOF motion controller must not cap step length/height with {token}")
    if "GetMotionStepSnapshot(&stepLengthM, &stepHeightM)" not in control:
        return fail("IoT motion loop must use runtime step length/height snapshot")
    semantic_motion_patterns = [
        r"case\s+MOTION_CMD_TROT:\s*result\s*=\s*trot_cycle\s*\(\s*stepLengthM\s*,\s*stepHeightM\s*\)\s*;",
        r"case\s+MOTION_CMD_TROT_BACK:\s*result\s*=\s*trot_back_cycle\s*\(\s*stepLengthM\s*,\s*stepHeightM\s*\)\s*;",
        r"case\s+MOTION_CMD_TURN_LEFT:\s*result\s*=\s*diversion_left_cycle\s*\(\s*stepLengthM\s*,\s*stepHeightM\s*\)\s*;",
        r"case\s+MOTION_CMD_TURN_RIGHT:\s*result\s*=\s*diversion_right_cycle\s*\(\s*stepLengthM\s*,\s*stepHeightM\s*\)\s*;",
    ]
    for pattern in semantic_motion_patterns:
        if not re.search(pattern, control, re.S):
            return fail(f"IoT motion command mapping must stay semantic and config-driven: {pattern}")
    semantic_motion_patterns = [
        r"int\s+trot_cycle\s*\([^)]*\)\s*\{[^{}]*return\s+spotMicroCrawlCycleInternal\s*\(\s*step_length\s*,\s*0\.0\s*,\s*0\.0\s*,\s*step_height\s*\)\s*;",
        r"int\s+crawl_cycle\s*\([^)]*\)\s*\{[^{}]*return\s+spotMicroCrawlCycleInternal\s*\(\s*step_length\s*,\s*0\.0\s*,\s*0\.0\s*,\s*step_height\s*\)\s*;",
        r"int\s+trot_in_place_cycle\s*\([^)]*\)\s*\{[^{}]*return\s+spotMicroCrawlCycleInternal\s*\(\s*0\.0\s*,\s*0\.0\s*,\s*0\.0\s*,\s*step_height\s*\)\s*;",
        r"int\s+trot_back_cycle\s*\([^)]*\)\s*\{[^{}]*return\s+spotMicroCrawlCycleInternal\s*\(\s*-step_length\s*,\s*0\.0\s*,\s*0\.0\s*,\s*step_height\s*\)\s*;",
        r"int\s+diversion_right_cycle\s*\([^)]*\)\s*\{[^{}]*return\s+spotMicroCrawlCycleInternal\s*\(\s*step_length\s*\*\s*0\.25\s*,\s*0\.0\s*,\s*-step_length\s*\*\s*0\.75\s*,\s*step_height\s*\)\s*;",
        r"int\s+diversion_left_cycle\s*\([^)]*\)\s*\{[^{}]*return\s+spotMicroCrawlCycleInternal\s*\(\s*step_length\s*\*\s*0\.25\s*,\s*0\.0\s*,\s*step_length\s*\*\s*0\.75\s*,\s*step_height\s*\)\s*;",
    ]
    for pattern in semantic_motion_patterns:
        if not re.search(pattern, motion, re.S):
            return fail(f"12DOF gait functions must keep semantic directional mapping: {pattern}")
    for name in [
        "trot_left_front_cycle",
        "trot_right_front_cycle",
        "trot_left_back_cycle",
        "trot_right_back_cycle",
    ]:
        match = re.search(rf"int\s+{name}\s*\([^)]*\)\s*\{{(?P<body>.*?)\n\}}", motion, re.S)
        if not match:
            return fail(f"12DOF missing compatibility arc function {name}")
        body = match.group("body")
        if "return 0;" not in body or "not supported" not in body:
            return fail(f"{name} must reject unsupported arc gait")
        if "trotArcCycleInternal" in body or "runServoGaitTableCycle" in body:
            return fail(f"{name} must not plan active arc gait")
    if "trotArcCycleInternal" in motion:
        return fail("12DOF active arc gait planner must be removed")
    if "EDOG_TABLE_MODE_ARC" in motion or "innerScalePercent" in motion or "outerScalePercent" in motion:
        return fail("12DOF table gait must not keep active arc planning cases")
    gait_generator = gait[gait.find("void Edog12Dof_GenerateTrotTable"):]
    if "Edog12Dof_IK(" not in gait_generator:
        return fail("12DOF gait table must be generated from foot trajectory through IK")
    init_match = re.search(r"void\s+init_dog\s*\([^)]*\)\s*\{(?P<body>.*?)\n\}", motion, re.S)
    if not init_match:
        return fail("12DOF motion implementation missing init_dog body")
    init_body = init_match.group("body")
    if "Edog12Dof_GenerateTrotTable" in init_body:
        return fail("init_dog must initialize the explicit standing pose, not gait frame 0")
    for leg_index in range(4):
        if f"setLegAngles(&g_legs[{leg_index}], &g_standJointAngles)" not in init_body and "g_startupLegOrder[i]" not in init_body:
            return fail(f"init_dog must apply g_standJointAngles to leg {leg_index}")
    build_match = re.search(
        r"static\s+void\s+buildStandPose\s*\([^)]*\)\s*\{(?P<body>.*?)\n\}",
        motion,
        re.S,
    )
    if not build_match:
        return fail("12DOF motion implementation missing buildStandPose body")
    build_body = build_match.group("body")
    if "Edog12Dof_GenerateTrotTable" in build_body:
        return fail("buildStandPose must use explicit standing pose, not gait frame 0")
    for channel in [
        "LF_HIP", "LF_THIGH", "LF_CALF",
        "RF_HIP", "RF_THIGH", "RF_CALF",
        "LB_HIP", "LB_THIGH", "LB_CALF",
        "RB_HIP", "RB_THIGH", "RB_CALF",
    ]:
        if f"target[{channel}] = jointDeltaToServoAngle({channel}" not in build_body:
            return fail(f"buildStandPose missing standing target for {channel}")
    return 0


def check_12_dof_table_driven_motion():
    config = read("include/edog_config.h")
    gait = read("12_DOF_Version/src/gait_generate_12dof.c")
    gait_header = read("12_DOF_Version/include/gait_generate_12dof.h")
    motion = read("12_DOF_Version/src/motion_utils_12dof.c")
    motion_header = read("12_DOF_Version/include/motion_utils_12dof.h")
    control = read("utils/src/iot_control.c")
    servo = read("utils/src/servo_control.c")

    for name, value in {
        "EDOG_12DOF_L_BODY_MM": "190.0",
        "EDOG_12DOF_W_BODY_MM": "87.0",
        "EDOG_12DOF_L1_MM": "50.0",
        "EDOG_12DOF_L2_MM": "107.0",
        "EDOG_12DOF_L3_MM": "135.0",
        "EDOG_12DOF_GAIT_FRAME_COUNT": "30",
    }.items():
        if not re.search(rf"#define\s+{name}\s+{re.escape(value)}\b", gait_header):
            return fail(f"12DOF table gait header missing {name} {value}")
    if "#define EDOG_12DOF_TROT_FRAME_COUNT EDOG_12DOF_GAIT_FRAME_COUNT" not in gait_header:
        return fail("12DOF old trot frame count must alias the 30-frame gait target table count")

    for name, value in {
        "EDOG_SERVO_HIP_SPEED_60_DEG_MS": "330",
        "EDOG_SERVO_LEG_SPEED_60_DEG_MS": "250",
        "EDOG_SERVO_SPEED_SAFETY_NUM": "8",
        "EDOG_SERVO_SPEED_SAFETY_DEN": "10",
        "EDOG_12DOF_GAIT_FRAME_FPS": "50",
        "EDOG_12DOF_REAR_SWING_LIFT_EXTRA_MM": "6.0",
        "EDOG_12DOF_DEFAULT_FOOT_Z_MM": "140.0",
        "EDOG_12DOF_REAR_FOOT_UP_OFFSET_MM": "0.0",
        "EDOG_12DOF_TROT_TRAJECTORY_CYCLOID": "1",
        "EDOG_12DOF_TROT_BODY_X_SHIFT_MM": "5.0",
        "EDOG_SERVO_TRAPEZOID_MAX_SPEED_CENTI_PER_FRAME": "420",
        "EDOG_SERVO_TRAPEZOID_MAX_ACCEL_CENTI_PER_FRAME2": "140",
        "EDOG_SERVO_TRAPEZOID_TARGET_EPS_CENTI": "2",
        "EDOG_12DOF_GAIT_MAX_JOINT_STEP_CENTI": "360",
    }.items():
        if not re.search(rf"#define\s+{name}\s+{re.escape(value)}\b", config):
            return fail(f"edog_config.h missing {name} {value}")

    for token in [
        "static double bezierQuintic",
        "return u5 * p0",
        "static double bezierSwingLift",
        "return bezierQuintic(0.0, 0.0, 1.6, 1.6, 0.0, 0.0, t)",
        "static Edog12DofFootPoint quinticBezierSwingFoot",
        "static double pyAppleCycloidProgress",
        "static double pyAppleCycloidLift",
        "static Edog12DofFootPoint pyAppleCycloidSwingFoot",
        "#if EDOG_12DOF_TROT_TRAJECTORY_CYCLOID",
        "return pyAppleCycloidLift(phase / swingPortion)",
        "return bezierSwingLift(phase / swingPortion)",
        "foot = defaultSwingFoot",
        "effectiveLiftMm += EDOG_12DOF_REAR_SWING_LIFT_EXTRA_MM",
        "foot.xMm -= bodyShiftMm",
        "phase = (double)frameIndex / (double)EDOG_12DOF_GAIT_FRAME_COUNT",
        "Edog12Dof_ReferencePhaseForFrame",
        "Edog12Dof_ReferenceSwingEnvelopeForPhase",
        "Edog12Dof_SampleReferenceTrotFootPoint",
        "rearSwingBoostEnvelope",
        "static int clampJointStepCenti",
        "static void limitGaitTableJointStepCenti",
        "limitGaitTableJointStepCenti(gaitTable)",
        "EDOG_12DOF_GAIT_MAX_JOINT_STEP_CENTI",
        "double xOffset = hardwareStrideMm / 2.0 - hardwareStrideMm * stanceT",
        "Edog12Dof_FootYOnHipPlaneForLeg",
        "foot.yMm = Edog12Dof_FootYOnHipPlaneForLeg(legIndex, foot.zMm) + yOffset",
        "Edog12Dof_IK(&foot, isRightLeg, &gaitTable[k])",
    ]:
        if token not in gait:
            return fail(f"12DOF gait generator missing quadratic-bezier/table token {token}")
    if "foot.yMm += yOffset" in gait:
        return fail("12DOF gait generator must recompute foot.yMm from the hip plane after z changes")
    if "foot.zMm -= lift" in gait or "foot.zMm += press" in gait:
        return fail("12DOF gait generator must set z first, then project y from the hip plane")
    if "EDOG_12DOF_STANCE_PRESS_MM * sin" in gait:
        return fail("12DOF 8DOF-reference stance must keep neutral Z without sinusoidal press")
    if "double Edog12Dof_FootYOnHipPlane(int isRightLeg, double zMm)" not in gait_header:
        return fail("12DOF gait header must expose hip-plane Y projection helper")
    if "double Edog12Dof_FootYOnHipPlane(int isRightLeg, double zMm)" not in gait:
        return fail("12DOF gait source must implement hip-plane Y projection helper")
    if "double Edog12Dof_FootYOnHipPlaneForLeg(int legIndex, double zMm)" not in gait_header:
        return fail("12DOF gait header must expose per-leg hip-plane Y projection helper")
    if "double Edog12Dof_FootYOnHipPlaneForLeg(int legIndex, double zMm)" not in gait:
        return fail("12DOF gait source must implement per-leg hip-plane Y projection helper")
    if "Edog12Dof_DefaultFootZForLeg(legIndex == 0 || legIndex == 1)" not in gait:
        return fail("12DOF default foot-Y must use each leg's own default Z")
    if "Edog12Dof_FootYOnHipPlaneForLeg(leg, foot->zMm)" not in motion:
        return fail("12DOF IMU Z compensation must reproject foot Y onto the hip plane")
    if "legState->foot.yMm = Edog12Dof_FootYOnHipPlaneForLeg(leg, legState->foot.zMm)" not in motion:
        return fail("12DOF realtime swing must reproject foot Y after per-leg default foot-Z changes")

    for token in [
        "typedef struct {\n    int centiDeg[EDOG_SERVO_CHANNEL_COUNT];",
        "typedef struct {\n    Edog12DofServoFrame frames[EDOG_12DOF_GAIT_FRAME_COUNT];",
        "typedef struct {\n    int mode;",
        "static Edog12DofServoGaitTable g_servoGaitTable",
        "static Edog12DofTableRuntime g_servoTableRuntime",
        "static void buildServoGaitTable",
        "static int computeServoTableMinFrameMs",
        "static int getSafeServoCentiDegPerMs",
        "static int isHipServoChannel",
        "static int getServoSpeed60DegMsForChannel",
        "getSafeServoCentiDegPerMs(channel)",
        "static int approachServoTargetCenti",
        "static int runServoGaitTableCycle",
        "servoTableCentiToTrimmedTarget(",
        "table->frames[frameIndex].centiDeg[channel]",
        "g_servoTableRuntime.cycleMs = requestedCycleMs > g_servoGaitTable.minCycleMs",
        "table->minCycleMs = table->minFrameMs * table->frameCount",
        "return completedCycle ? 1 : -1",
        "typedef struct {\n    int active;\n    int mode;\n    int frameIndex;",
        "static Edog12DofContinuousTrotRuntime g_continuousTrotRuntime",
        "g_continuousTrotRuntime.frameIndex",
        "apply8DofTableFrameStaggered(*gaitSet, frameIndex, EDOG_12DOF_TROT_FRAME_COUNT / 2)",
    ]:
        if token not in motion:
            return fail(f"12DOF table scheduler missing token {token}")

    for name, expected in {
        "trot_cycle": "trotCycleInternal(step_length, step_height, 0, 100, 100)",
        "crawl_cycle": "spotMicroCrawlCycleInternal(step_length, 0.0, 0.0, step_height)",
        "trot_in_place_cycle": "trotCycleInternal(0.0, step_height, 0, 100, 100)",
        "trot_back_cycle": "trotCycleInternal(step_length, step_height, 1, 100, 100)",
        "diversion_right_cycle": "runServoGaitTableCycle(EDOG_TABLE_MODE_TURN_RIGHT, step_length, step_height, 0x0F)",
        "diversion_left_cycle": "runServoGaitTableCycle(EDOG_TABLE_MODE_TURN_LEFT, step_length, step_height, 0x0F)",
    }.items():
        match = re.search(rf"int\s+{name}\s*\([^)]*\)\s*\{{(?P<body>.*?)\n\}}", motion, re.S)
        if not match:
            return fail(f"12DOF missing public motion function {name}")
        body = match.group("body")
        if expected not in body:
            return fail(f"{name} must use expected whole-body gait path {expected}")
        if name not in ("diversion_right_cycle", "diversion_left_cycle") and "runServoGaitTableCycle" in body:
            return fail(f"{name} must not use servo gait table scheduler")
        if name in ("diversion_right_cycle", "diversion_left_cycle") and "trotTurnCycleInternal" in body:
            return fail(f"{name} must not use legacy forward/reverse turn mixer")
        if name != "crawl_cycle" and "spotMicroCrawlCycleInternal" in body:
            return fail(f"{name} must not call SpotMicro crawl as default gait")
    for name in [
        "trot_left_front_cycle",
        "trot_right_front_cycle",
        "trot_left_back_cycle",
        "trot_right_back_cycle",
    ]:
        match = re.search(rf"int\s+{name}\s*\([^)]*\)\s*\{{(?P<body>.*?)\n\}}", motion, re.S)
        if not match:
            return fail(f"12DOF missing public motion function {name}")
        body = match.group("body")
        if "return 0;" not in body or "not supported" not in body:
            return fail(f"{name} must reject unsupported arc gait")
        if "trotArcCycleInternal" in body or "runServoGaitTableCycle" in body:
            return fail(f"{name} must not call an active arc gait path")

    match = re.search(r"int\s+single_leg_gait_cycle\s*\([^)]*\)\s*\{(?P<body>.*?)\n\}", motion, re.S)
    if not match or "return leg_group_gait_cycle(1 << leg_index, step_length, step_height);" not in match.group("body"):
        return fail("single_leg_gait_cycle must delegate to leg_group_gait_cycle")
    match = re.search(r"int\s+leg_group_gait_cycle\s*\([^)]*\)\s*\{(?P<body>.*?)\n\}", motion, re.S)
    if not match or "runServoGaitTableCycle(EDOG_TABLE_MODE_FORWARD, step_length, step_height, leg_mask)" not in match.group("body"):
        return fail("leg_group_gait_cycle must keep servo gait table scheduler for leg-mask debug")

    for token in [
        "phaseOffset[0] = 0",
        "phaseOffset[3] = 0",
        "phaseOffset[1] = EDOG_12DOF_GAIT_FRAME_COUNT / 2",
        "phaseOffset[2] = EDOG_12DOF_GAIT_FRAME_COUNT / 2",
    ]:
        if token not in motion:
            return fail(f"12DOF table trot phase mapping missing {token}")

    direction_match = re.search(
        r"static\s+const\s+int\s+g_servoDirection\s*\[\s*EDOG_SERVO_CHANNEL_COUNT\s*\]\s*=\s*\{(?P<body>.*?)\};",
        motion,
        re.S,
    )
    if not direction_match:
        return fail("12DOF motion implementation must define g_servoDirection")
    values = [int(value) for value in re.findall(r"[-+]?\d+", re.sub(r"/\*.*?\*/", "", direction_match.group("body"), flags=re.S))]
    if values != [1, -1, -1, 0, -1, 1, 1, 0, 1, -1, -1, 0, -1, 1, 1, 0]:
        return fail(f"g_servoDirection changed unexpectedly: {values}")

    for token in [
        "static int applyServoCenterTrimCenti",
        "g_servoCenterTrim[channel] * EDOG_12DOF_CENTI_PER_DEG",
        "static int servoTableCentiToTrimmedTarget",
        "90deg + trim",
        "motionSetServoPhysicalNoTrimCenti(channel, calibratedCenterCenti)",
        "approachServoTargetCenti(current, finalTarget, maxStep)",
    ]:
        if token not in motion:
            return fail(f"12DOF trim/stop safety missing token {token}")

    for token in [
        "static int centiDegToPwmCount(int centiDeg)",
        "return setServoPwmCount(channel, count)",
        "const long long rangeCenti = (long long)EDOG_SERVO_ANGLE_RANGE_DEG * 100",
    ]:
        if token not in servo:
            return fail(f"servo centi-degree PWM path missing {token}")

    if "result > 0 && FinishOneMotionCycle()" not in control:
        return fail("motion task must only finish repeat cycles when the motion entry reports a completed cycle")
    if "int balance_stand_frame(void);" not in motion_header:
        return fail("motion header must export balance_stand_frame")
    return 0


def check_motion_task_stack_safety():
    config = read("include/edog_config.h")
    motion = read("12_DOF_Version/src/motion_utils_12dof.c")

    match = re.search(r"#define\s+EDOG_TASK_MOTION_STACK_SIZE\s+(\d+)", config)
    if not match:
        return fail("edog_config.h missing EDOG_TASK_MOTION_STACK_SIZE")
    if int(match.group(1)) < 8192:
        return fail("motion_control_task stack must be at least 8192 bytes")

    build_match = re.search(
        r"static\s+void\s+buildServoGaitTable\s*\([^)]*\)\s*\{(?P<body>.*?)\n\}",
        motion,
        re.S,
    )
    if not build_match:
        return fail("buildServoGaitTable not found")
    if re.search(r"\bEdog12DofGaitSet\s+gaitSet\s*;", build_match.group("body")):
        return fail("buildServoGaitTable must not allocate Edog12DofGaitSet on motion task stack")
    if "static Edog12DofGaitSet g_tableBuildGaitSet" not in motion:
        return fail("motion_utils_12dof.c missing static gait table build buffer")
    return 0


def check_mqtt_task_preempts_motion_task():
    config = read("include/edog_config.h")

    priorities = {}
    for name in [
        "EDOG_TASK_WIFI_MQTT_PRIORITY",
        "EDOG_TASK_MOTION_PRIORITY",
    ]:
        match = re.search(rf"#define\s+{name}\s+(\d+)\b", config)
        if not match:
            return fail(f"edog_config.h missing {name}")
        priorities[name] = int(match.group(1))

    if priorities["EDOG_TASK_WIFI_MQTT_PRIORITY"] >= priorities["EDOG_TASK_MOTION_PRIORITY"]:
        return fail("WiFi/MQTT task must have higher priority than motion task to avoid command-time disconnects")
    return 0


def check_imu_stop_settling_pd_contract():
    config = read("include/edog_config.h")
    motion = read("12_DOF_Version/src/motion_utils_12dof.c")

    required_macros = {
        "EDOG_12DOF_IMU_BALANCE_DEADBAND_CENTI_DEG": 80,
        "EDOG_12DOF_IMU_BALANCE_STOP_SETTLE_MS": 800,
        "EDOG_12DOF_IMU_BALANCE_STOP_MAX_SETTLE_MS": 2500,
        "EDOG_12DOF_IMU_BALANCE_STOP_STABLE_FRAMES": 15,
        "EDOG_12DOF_IMU_BALANCE_STOP_STABLE_TILT_CENTI_DEG": 400,
        "EDOG_12DOF_IMU_BALANCE_STOP_EXIT_TILT_CENTI_DEG": 600,
        "EDOG_12DOF_IMU_BALANCE_STOP_STABLE_RATE_CENTI_DPS": 8000,
        "EDOG_12DOF_IMU_BALANCE_STOP_EXIT_RATE_CENTI_DPS": 12000,
        "EDOG_12DOF_IMU_BALANCE_RAMP_IN_MS": 1200,
        "EDOG_12DOF_IMU_BALANCE_ROLL_STEP_LIMIT_CENTI_DEG": 167,
        "EDOG_12DOF_IMU_BALANCE_PITCH_STEP_LIMIT_CENTI_DEG": 167,
        "EDOG_12DOF_IMU_BALANCE_INTEGRAL_KI_PERCENT": 0,
    }
    for name, value in required_macros.items():
        if not re.search(rf"#define\s+{name}\s+{value}\b", config):
            return fail(f"edog_config.h missing stop IMU damping macro {name} {value}")

    for token in [
        "typedef enum {\n    EDOG_IMU_BALANCE_MODE_NORMAL",
        "EDOG_IMU_BALANCE_MODE_STOP_SETTLING",
        "EDOG_IMU_BALANCE_MODE_RAMP_IN",
        "static EdogImuBalanceMode g_balanceMode",
        "static void clearImuBalanceControlState(void)",
        "static void enterImuBalanceMode(EdogImuBalanceMode mode)",
        "static int imuBalanceMotionIsStable",
        "static int imuBalanceMotionExceedsExit",
        "static void applyImuBalanceRampScale",
        "enterImuBalanceMode(EDOG_IMU_BALANCE_MODE_STOP_SETTLING)",
        "enterImuBalanceMode(EDOG_IMU_BALANCE_MODE_NORMAL)",
        "g_balanceMode == EDOG_IMU_BALANCE_MODE_STOP_SETTLING",
        "g_balanceMode == EDOG_IMU_BALANCE_MODE_RAMP_IN",
        "limitImuControlStep(g_balanceLastRollOutputCentiDeg",
        "EDOG_12DOF_IMU_BALANCE_INTEGRAL_KI_PERCENT != 0",
    ]:
        if token not in motion:
            return fail(f"motion_utils_12dof.c missing stop IMU damping token {token}")
    return 0


def check_real_servo_angle_calibration_contract():
    iot = read("utils/src/iot.c")
    control = read("utils/src/iot_control.c")
    motion = read("12_DOF_Version/src/motion_utils_12dof.c")

    if "setDogServoPhysicalAngleTracked" not in motion:
        return fail("12DOF motion must expose a no-trim tracked physical angle setter")
    if "int setDogServoAngleTracked(int channel, int angle)\n{\n    return setDogServoPhysicalAngleTracked(channel, angle);\n}" not in motion:
        return fail("manual servo_set must write the requested real physical angle without adding trim")
    if "return motionSetServoPhysical(channel, angle);" in motion:
        return fail("manual servo_set must not use the trim-applying motionSetServoPhysical path")
    if "EDOG_SERVO_TRIM_RESET_MIGRATION_KEY" not in motion:
        return fail("firmware must include one-time servo trim reset migration key")
    if "clearDogServoCenterTrimsOnceForRealAngleCalibration" not in motion:
        return fail("firmware must clear persisted trims once for real-angle recalibration")
    if "clearDogServoCenterTrimsOnceForRealAngleCalibration" not in read("src/main.c"):
        return fail("servo init must run the one-time real-angle trim clear before loading trims")

    for token in [
        "\"physical_angle\"",
        "\"physicalAngle\"",
        "trim = angle - EDOG_SERVO_CENTER_ANGLE",
    ]:
        if token not in iot:
            return fail(f"MQTT servo_trim parser missing real physical angle token {token}")

    if "Angle_real=Slider_offset+Value_calib" in control or "Angle_real=Slider_offset+Value_calib" in iot:
        return fail("device reports must not expose the old angle plus calibration formula")
    if "trim=physical_center_angle-90" not in control:
        return fail("servo status report must expose the new physical-angle calibration formula")
    return 0


def check_stanford_pupper_gait_controller():
    motion = read("12_DOF_Version/src/motion_utils_12dof.c")
    header = read("12_DOF_Version/include/motion_utils_12dof.h")
    build = read("BUILD.gn")
    control = read("utils/src/iot_control.c")

    if "NOTICE_StanfordPupper.md" in build:
        return fail("BUILD.gn should not include Stanford Pupper notice after reverting to local gait")
    for token in [
        "EdogOpenGaitState",
        "EdogOpenGaitCommand",
        "EDOG_OPENGAIT",
        "openGaitCycleInternal",
        "openGaitStanceStep",
        "openGaitSwingStep",
        "Stanford Pupper",
    ]:
        if token in motion + header:
            return fail(f"Stanford/open gait code must be removed from active 12DOF firmware: {token}")
    if "open_gait_cycle" in header:
        return fail("12DOF public header should not expose open_gait_cycle after reverting")
    if "case MOTION_CMD_TROT:" not in control or not re.search(
        r"result\s*=\s*trot_cycle\s*\(\s*stepLengthM\s*,\s*stepHeightM\s*\)\s*;",
        control,
        re.S,
    ):
        return fail("IoT motion command mapping should keep local gait with runtime step parameters")
    return 0


def check_firmware_log_noise_contract():
    config = read("include/edog_config.h")
    sources = {
        "src/main.c": read("src/main.c"),
        "src/wifi_mqtt_task.c": read("src/wifi_mqtt_task.c"),
        "utils/src/iot.c": read("utils/src/iot.c"),
        "utils/src/iot_control.c": read("utils/src/iot_control.c"),
        "utils/src/mpu6050_motion_light.c": read("utils/src/mpu6050_motion_light.c"),
        "utils/src/servo_control.c": read("utils/src/servo_control.c"),
        "utils/src/task_util.c": read("utils/src/task_util.c"),
        "12_DOF_Version/src/motion_utils_12dof.c": read("12_DOF_Version/src/motion_utils_12dof.c"),
    }
    joined = "\n".join(sources.values())

    if not re.search(r"#define\s+EDOG_12DOF_IMU_BALANCE_DEBUG_ENABLED\s+0\b", config):
        return fail("IMU balance periodic debug logging must be disabled by default")

    forbidden_tokens = [
        "[IMU-BALANCE] status=",
        "gyroCentiDps=",
        "runtime strength=",
        "initialized for self balance",
        "[MotionTask] start motion command loop",
        "[MotionTask] requested repeat count finished",
        "[INFO] request stop current 12DOF motion",
        "[INFO] 12DOF speed level=",
        "[12DOF] stand hip=",
        "EDOG_12DOF_BUILD_PARAMS",
        "runtime tuning applied",
        "runtime geometry applied",
        "speed level=%d applied",
        "status report published",
        "calibration report published",
        "property report success",
        "Edog published to",
        "mqtt publish success",
        "Starting MQTT",
        "NetworkConnect ...",
        "MQTTClientInit ...",
        "MQTTConnect ...",
        "MQTTSubscribe (",
        "Subscribe %s success",
        "connected successfully",
        "Task [%s] created successfully",
        "[PCA9685] initialized at",
        "all channels reset to center",
        "Body balanced, RGB blue",
        "motion RGB task start",
        "[AppInit] Start",
        "[ServoInitTask] Start",
        "[ServoInitTask] Done",
        "[AppInit] MpuMotionLightTask disabled",
    ]
    for token in forbidden_tokens:
        if token in joined:
            return fail(f"normal/debug log must be quieted: {token}")

    for path, text in sources.items():
        if re.search(r'printf\s*\(\s*"\[Payload\]', text):
            return fail(f"{path} must not print raw MQTT payloads")
        if re.search(r'printf\s*\(\s*"\[Topic\]', text):
            return fail(f"{path} must not print verbose MQTT topic banners")
        if re.search(r'printf\s*\(\s*"Topic:', text):
            return fail(f"{path} must not print verbose MQTT topic lines")

    required_tokens = [
        "[MQTT] command=%s",
        "[Edog] content=%s",
        "[网络] WiFi连接开始",
        "[网络] WiFi连接成功",
        "[网络] WiFi连接失败",
        "[MQTT] TCP连接开始",
        "[MQTT] TCP连接成功",
        "[MQTT] 登录成功",
        "[MQTT] 订阅成功",
        "[MQTT] connected",
        "invalid",
        "failed",
        "FAILED",
        "[Error]",
    ]
    for token in required_tokens:
        if token not in joined:
            return fail(f"log contract missing allowed command/error token {token}")
    return 0


def check_fast_wifi_mqtt_startup_contract():
    config = read("include/edog_config.h")
    wifi_task = read("src/wifi_mqtt_task.c")
    wifi_tool = read("utils/src/wifi_tool.c")
    iot = read("utils/src/iot.c")
    mqtt_liteos_path = ROOT.parents[4] / "third_party/paho_mqtt/MQTTClient-C/src/liteOS/MQTTLiteOS.c"
    mqtt_client_path = ROOT.parents[4] / "third_party/paho_mqtt/MQTTClient-C/src/MQTTClient.c"
    mqtt_liteos = mqtt_liteos_path.read_text(encoding="utf-8")
    mqtt_client = mqtt_client_path.read_text(encoding="utf-8")

    required_macros = {
        "EDOG_WIFI_CONNECT_POLL_INTERVAL_MS": 100,
        "EDOG_WIFI_CONNECT_FAST_TIMEOUT_MS": 5000,
        "EDOG_WIFI_FAST_RETRY_DELAY_MS": 200,
        "EDOG_WIFI_MQTT_HEARTBEAT_MS": 30000,
        "EDOG_WIFI_MQTT_FAST_RETRY_DELAY_MS": 500,
        "EDOG_WIFI_MQTT_FAST_RETRY_COUNT": 3,
    }
    for name, value in required_macros.items():
        if not re.search(rf"#define\s+{name}\s+{value}\b", config):
            return fail(f"edog_config.h missing fast startup macro {name} {value}")
    timeout_match = re.search(r"#define\s+EDOG_MQTT_COMMAND_TIMEOUT_MS\s+(\d+)\b", config)
    if not timeout_match or int(timeout_match.group(1)) != 1000:
        return fail("MQTT client command timeout must match the known-good 6M22D value: 1000ms")

    required_wifi_tokens = [
        "TryConnectSavedNetworkFast",
        "WaitForStaConnected",
        "LOS_Msleep(EDOG_WIFI_CONNECT_POLL_INTERVAL_MS)",
        "EDOG_WIFI_CONNECT_FAST_TIMEOUT_MS",
        "EDOG_WIFI_FAST_RETRY_DELAY_MS",
        "GetMqttRetryDelayMs",
        "mqttFailureCount",
        "EDOG_WIFI_MQTT_FAST_RETRY_COUNT",
        "EDOG_WIFI_MQTT_FAST_RETRY_DELAY_MS",
    ]
    for token in required_wifi_tokens:
        if token not in wifi_task:
            return fail(f"wifi_mqtt_task.c missing fast startup token {token}")

    if "LOS_Msleep(2000)" in wifi_task:
        return fail("WiFi startup must not use fixed 2s sleeps between connect attempts")
    for forbidden in [
        "#define EDOG_WIFI_NETWORK_STABILIZE_MS",
        "WaitForNetworkStabilized",
        "LOS_Msleep(EDOG_WIFI_NETWORK_STABILIZE_MS)",
    ]:
        if forbidden in config or forbidden in wifi_task:
            return fail(f"WiFi/MQTT task must match 6M22D connection timing, found: {forbidden}")
    fixed_connect_body = re.search(
        r"static\s+int\s+ConnectFixedNetwork\s*\([^)]*\)\s*\{(?P<body>.*?)\n\}",
        wifi_task,
        re.S,
    )
    if not fixed_connect_body:
        return fail("wifi_mqtt_task.c missing ConnectFixedNetwork")
    body = fixed_connect_body.group("body")
    for token in [
        "WifiTool_LoadSavedCredentials",
        "strcmp(savedSsid, EDOG_WIFI_FIXED_ROUTE_SSID) != 0",
        "strcmp(savedPassword, EDOG_WIFI_FIXED_ROUTE_PASSWORD) != 0",
    ]:
        if token not in body:
            return fail("fixed WiFi startup must overwrite stale saved credentials before connecting")
    if "WifiTool_SaveCredentials(EDOG_WIFI_FIXED_ROUTE_SSID, EDOG_WIFI_FIXED_ROUTE_PASSWORD)" not in body:
        return fail("fixed WiFi startup must still seed fixed credentials when needed")

    if "SetWifiModeOn" not in wifi_tool or "WifiTool_IsStaConnected" not in wifi_tool:
        return fail("wifi_tool.c must start STA and let the task poll for connected/IP")
    if "WifiTool_GetCurrentIPv4" in wifi_tool.split("int WifiTool_ConnectSavedNetwork(void)", 1)[1].split("int WifiTool_StartProvisioningAp", 1)[0]:
        return fail("WifiTool_ConnectSavedNetwork must not do extra IP read before fast polling")

    if "MQTTClientInit(&client, &network, EDOG_MQTT_COMMAND_TIMEOUT_MS" not in iot:
        return fail("MQTT client command timeout must use EDOG_MQTT_COMMAND_TIMEOUT_MS")
    if not re.search(r"#define\s+EDOG_MQTT_PACKET_BUFFER_LENGTH\s+2048\b", config):
        return fail("MQTT packet buffer must be at least 2048 bytes for IoTDA system packets")
    for token in [
        "static unsigned char sendBuf[EDOG_MQTT_PACKET_BUFFER_LENGTH];",
        "static unsigned char readBuf[EDOG_MQTT_PACKET_BUFFER_LENGTH];",
    ]:
        if token not in iot:
            return fail("Paho MQTT send/read buffers must use EDOG_MQTT_PACKET_BUFFER_LENGTH")
    if "MQTTConnect(&client, &data)" not in iot:
        return fail("MQTT login must use the known-good MQTTConnect call")
    for token in [
        "fd_set read_set",
        "select(n->my_socket + 1, &read_set, NULL, NULL, &interval)",
        "total_timeout_ms",
        "select_timeout_ms",
        "bytes += rc;",
        "continue;",
        "errno != EAGAIN && errno != EWOULDBLOCK && errno != ETIMEDOUT",
        "errno != EINTR && errno != ECONNABORTED",
        "[MQTT] liteos select failed errno=%d socket=%d",
        "[MQTT] liteos recv failed errno=%d socket=%d",
        "[MQTT] liteos recv peer closed socket=%d",
    ]:
        if token not in mqtt_liteos:
            return fail("MQTT idle recv timeout must be handled by select without breaking the connection")
    if not re.search(r"int\s+linux_read\s*\([^)]*\)\s*\{", mqtt_liteos):
        return fail("MQTTLiteOS.c missing linux_read")
    if "SO_RCVTIMEO" in mqtt_liteos:
        return fail("MQTT linux_read must not rely on SO_RCVTIMEO for MQTTYield idle timeout")
    for token in [
        "static unsigned int mqttYieldFailureCount = 0;",
        "static unsigned int mqttPublishFailureCount = 0;",
        "EDOG_MQTT_YIELD_FAILURE_RECONNECT_THRESHOLD",
        "EDOG_MQTT_PUBLISH_FAILURE_RECONNECT_THRESHOLD",
        "static void MarkMqttPublishFailure(const char *source, int rc)",
        "MarkMqttPublishFailure(\"heartbeat\", rc)",
        "MarkMqttPublishSuccess();",
        "yield failed rc=%d consecutive=%u",
        "[MQTT] readPacket header read failed rc=%d left_ms=%d",
        "[MQTT] readPacket overflow rem_len=%d readbuf_size=%u",
        "[MQTT] cycle fatal packet_type=%d connected=%d ping_out=%d",
        "[MQTT] keepalive failed connected=%d ping_out=%d",
        "[MQTT] yield cycle failed cycle_rc=%d left_ms=%d",
        "static int MqttReadTimeoutMs(MQTTClient* c, Timer* timer)",
        "return left_ms > 0 ? left_ms : (int)c->command_timeout_ms;",
    ]:
        if token not in (iot + mqtt_client + mqtt_liteos):
            return fail(f"MQTT yield failures must include layered diagnostics, missing: {token}")
    if "cmd_ack" in iot or "mqtt_send_command_ack" in iot:
        return fail("custom edog commands must not publish extra MQTT ack packets")
    if "EDOG_WIFI_MQTT_FIRST_SESSION_REFRESH" in config + wifi_task:
        return fail("MQTT first-session refresh workaround must be removed; fix packet parsing instead")
    if "MQTT 首次会话刷新" in wifi_task:
        return fail("MQTT first-session refresh log must be removed")
    if "readPacket body read failed rc=%d rem_len=%d left_ms=%d" not in mqtt_client:
        return fail("MQTT packet body read failure diagnostic missing")
    if "int decoded_len = decodePacket(c, &rem_len, MqttReadTimeoutMs(c, timer));" not in mqtt_client:
        return fail("MQTT remaining length read must use full command timeout and check result")
    if "readPacket remaining length read failed rc=%d left_ms=%d" not in mqtt_client:
        return fail("MQTT remaining length read failure diagnostic missing")
    if "rc = MQTTPACKET_READ_ERROR;" not in mqtt_client:
        return fail("incomplete MQTT packet body must be fatal after linux_read exhausts retries")
    if "rc = 0;\n            goto exit;" in mqtt_client:
        return fail("partial MQTT packet body read must not be treated as idle timeout")
    if "decodePacket(c, &rem_len, TimerLeftMS(timer));" in mqtt_client:
        return fail("MQTT remaining length read must not ignore decodePacket result")
    init_match = re.search(r"int\s+mqtt_init\s*\([^)]*\)\s*\{(?P<body>.*?)\n\}", iot, re.S)
    if not init_match:
        return fail("iot.c missing mqtt_init")
    if "mqtt_send_heartbeat();" in init_match.group("body"):
        return fail("mqtt_init must not publish an immediate heartbeat after connected")
    for forbidden in [
        "MQTTConnectWithResults(&client, &data, &connack)",
        "[MQTT] CONNACK rc=%d session_present=%d",
        "[MQTT] 登录失败 rc=%d connack=%d",
    ]:
        if forbidden in iot:
            return fail(f"MQTT login diagnostics must not change the proven connect path: {forbidden}")
    return 0


def main():
    checks = [
        check_public_config_has_no_live_secret,
        check_secret_files_are_ignored_and_removed,
        check_public_files_have_no_cloud_credentials,
        check_mqtt_callbacks_only_enqueue,
        check_http_portal_limits,
        check_fixed_wifi_mode_without_provisioning,
        check_motion_state_locking,
        check_servo_i2c_errors_are_handled,
        check_app_artifacts_are_removed_and_ignored,
        check_mpu6050_motion_rgb_feature,
        check_12_dof_table_driven_motion,
        check_motion_task_stack_safety,
        check_imu_stop_settling_pd_contract,
        check_real_servo_angle_calibration_contract,
        check_stanford_pupper_gait_controller,
        check_firmware_log_noise_contract,
        check_fast_wifi_mqtt_startup_contract,
        check_mqtt_task_preempts_motion_task,
    ]
    failures = sum(check() for check in checks)
    if failures:
        print(f"{failures} check(s) failed")
        return 1
    print("all edog fix checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
