#ifndef EDOG_CONFIG_H
#define EDOG_CONFIG_H

#include "iot_gpio.h"

/*
 * Centralized runtime configuration for edog_project.
 * Update values here to change the project-wide defaults.
 */

/* Wi-Fi provisioning */
#define EDOG_WIFI_STA_CONNECT_RETRIES          3
#define EDOG_WIFI_MQTT_RETRY_DELAY_MS          3000
#define EDOG_WIFI_MQTT_FAST_RETRY_DELAY_MS     500
#define EDOG_WIFI_MQTT_FAST_RETRY_COUNT        3
#define EDOG_WIFI_MQTT_YIELD_MS                100
#define EDOG_WIFI_MQTT_HEARTBEAT_MS            30000
#define EDOG_WIFI_PROVISIONING_RESPONSE_MS     1500
#define EDOG_WIFI_CONNECT_POLL_INTERVAL_MS     100
#define EDOG_WIFI_CONNECT_FAST_TIMEOUT_MS      5000
#define EDOG_WIFI_FAST_RETRY_DELAY_MS          200
#define EDOG_WIFI_STARTUP_STABILIZE_MS         2000

#define EDOG_WIFI_AP_SSID                      "EDOG_WiFi"
#define EDOG_WIFI_AP_PASSWORD                  "88888888"
#define EDOG_WIFI_AP_FALLBACK_IP               "192.168.2.1"
#define EDOG_WIFI_FACTORY_ROUTE_SSID           "YZY"
#define EDOG_WIFI_FACTORY_ROUTE_PASSWORD       ""
#define EDOG_WIFI_HTTP_PORT                    80
#define EDOG_WIFI_HTTP_BUFFER_LEN              2048
#define EDOG_WIFI_HTTP_MAX_CONTENT_LENGTH      512
#define EDOG_WIFI_HTTP_TIMEOUT_MS              5000

/* Reset key */
#define EDOG_WIFI_RESET_GPIO                   GPIO0_PC7
#define EDOG_WIFI_RESET_PRESSED                0
#define EDOG_WIFI_RESET_LONG_PRESS_COUNT       100

/*
 * MQTT / IoTDA
 *
 * Put real device credentials in include/edog_config.local.h. That file is
 * intentionally ignored by git so production secrets do not live in source.
 */
#ifdef __has_include
#if __has_include("edog_config.local.h")
#include "edog_config.local.h"
#endif
#endif

#ifndef EDOG_MQTT_DEVICE_PASSWORD
#define EDOG_MQTT_DEVICE_PASSWORD              "CHANGE_ME"
#endif

#ifndef EDOG_MQTT_HOST_ADDR
#define EDOG_MQTT_HOST_ADDR                    "587a77885a.st1.iotda-device.cn-east-3.myhuaweicloud.com"
#endif

#ifndef EDOG_MQTT_DEVICE_ID
#define EDOG_MQTT_DEVICE_ID                    "CHANGE_ME_DEVICE_ID"
#endif

#ifndef EDOG_MQTT_CLIENT_ID
#define EDOG_MQTT_CLIENT_ID                    "CHANGE_ME_CLIENT_ID"
#endif

#ifndef EDOG_MQTT_USERNAME
#define EDOG_MQTT_USERNAME                     EDOG_MQTT_DEVICE_ID
#endif

/*
 * Wi-Fi startup mode.
 * EDOG_WIFI_PROVISIONING_ENABLED=0 时跳过 AP 配网，开机直接使用固定路由器。
 * 固定路由器 SSID/密码建议在 include/edog_config.local.h 中覆盖，避免把密码提交到公共代码。
 */
#ifndef EDOG_WIFI_PROVISIONING_ENABLED
#define EDOG_WIFI_PROVISIONING_ENABLED         0
#endif

#ifndef EDOG_WIFI_FIXED_ROUTE_SSID
#define EDOG_WIFI_FIXED_ROUTE_SSID             "CHANGE_ME_WIFI_SSID"
#endif

#ifndef EDOG_WIFI_FIXED_ROUTE_PASSWORD
#define EDOG_WIFI_FIXED_ROUTE_PASSWORD         "CHANGE_ME_WIFI_PASSWORD"
#endif

#define EDOG_MQTT_KEEPALIVE_SECONDS            300
#define EDOG_MQTT_COMMAND_TIMEOUT_MS           1000
#define EDOG_MQTT_BUFFER_LENGTH                512
#define EDOG_MQTT_PACKET_BUFFER_LENGTH         2048
#define EDOG_MQTT_STRING_LENGTH                64
#define EDOG_MOTION_COMMAND_QUEUE_LENGTH       12
#define EDOG_MOTION_COMMAND_TEXT_LENGTH        64

/* Temporary power-on servo test: keep disabled for normal startup. */
#define EDOG_POWER_ON_LF_SERVO_TEST_ENABLED    0
#define EDOG_POWER_ON_LF_SERVO_TEST_MIN_PULSE_US 1351
#define EDOG_POWER_ON_LF_SERVO_TEST_MAX_PULSE_US 1648
#define EDOG_POWER_ON_LF_SERVO_TEST_FRAME_COUNT 96
#define EDOG_POWER_ON_LF_SERVO_TEST_STEP_MS    20

/* Servo calibration */
#define EDOG_SERVO_CHANNEL_COUNT               16
/* 12DOF 版本使用 180 度舵机：软件角度 0~180，90 为机械中位。 */
#define EDOG_SERVO_ANGLE_RANGE_DEG             180
#define EDOG_SERVO_MIN_ANGLE                   0
#define EDOG_SERVO_MAX_ANGLE                   180
#define EDOG_SERVO_CENTER_ANGLE                90
/* PCA9685 以 50Hz 输出舵机 PWM；0.5ms~2.5ms 对应 0~180 度。 */
#define EDOG_SERVO_PWM_FREQUENCY_HZ            50
#define EDOG_SERVO_PULSE_MIN_US                500
#define EDOG_SERVO_PULSE_MAX_US                2500
#define EDOG_SERVO_MANUAL_SETTLE_US            50000
/* 分时输出，降低 12 个 25kg 舵机同时启动造成的瞬时电流冲击。 */
#define EDOG_SERVO_STARTUP_STEP_DELAY_US       120000
#define EDOG_SERVO_MOTION_STEP_DELAY_US        0
#define EDOG_SERVO_STOP_STEP_DELAY_US          6000
/* Rated servo speed: hip=0.33s/60deg, thigh/calf=0.25s/60deg. Use 80% loaded safety limit. */
#define EDOG_SERVO_HIP_SPEED_60_DEG_MS        330
#define EDOG_SERVO_LEG_SPEED_60_DEG_MS        250
#define EDOG_SERVO_SPEED_SAFETY_NUM            8
#define EDOG_SERVO_SPEED_SAFETY_DEN            10
/* 12DOF 步态目标帧率：50 FPS，即每帧 20ms；帧内舵机尽快同批写出。 */
#define EDOG_12DOF_GAIT_FRAME_FPS              50
#define EDOG_12DOF_GAIT_FRAME_PERIOD_US        (1000000 / EDOG_12DOF_GAIT_FRAME_FPS)
/* 12DOF gait geometry and command defaults. */
#define EDOG_12DOF_COMMAND_STEP_LENGTH_M       0.017
#define EDOG_12DOF_COMMAND_STEP_HEIGHT_M       0.012
#define EDOG_12DOF_TROT_TRAJECTORY_CYCLOID     1
#define EDOG_12DOF_TROT_BODY_X_SHIFT_MM        5.0
#define EDOG_12DOF_REAR_SWING_LIFT_EXTRA_MM    6.0
/* Turn gait draws higher transient current; stretch only yaw-turn table cycles. */
#define EDOG_12DOF_TURN_CYCLE_SLOWDOWN_NUM     8
#define EDOG_12DOF_TURN_CYCLE_SLOWDOWN_DEN     5
#define EDOG_12DOF_THIGH_LENGTH_DEFAULT_M       0.107
#define EDOG_12DOF_CALF_LENGTH_DEFAULT_M        0.135
#define EDOG_12DOF_LINK_LENGTH_MIN_M           0.001
#define EDOG_12DOF_LINK_LENGTH_MAX_M           0.220
#define EDOG_12DOF_DEFAULT_FOOT_X_MM           -150.0
#define EDOG_12DOF_REAR_FOOT_BACK_OFFSET_MM    0.0
#define EDOG_12DOF_DEFAULT_FOOT_Y_MM           0.0
#define EDOG_12DOF_DEFAULT_FOOT_Z_MM           140.0
#define EDOG_12DOF_REAR_FOOT_UP_OFFSET_MM      0.0
#define EDOG_12DOF_BODY_HEIGHT_DELTA_MIN_MM    -40.0
#define EDOG_12DOF_BODY_HEIGHT_DELTA_MAX_MM    40.0
#define EDOG_12DOF_HARDWARE_FORWARD_SIGN       -1.0
#define EDOG_12DOF_MIN_STRIDE_MM               1.0
#define EDOG_12DOF_MIN_LIFT_MM                 0.0
#define EDOG_12DOF_MAX_SIDE_MM                 30.0
#define EDOG_12DOF_MAX_YAW_MM                  28.0
#define EDOG_12DOF_STANCE_PRESS_MM             3.0
/* Upper bound for adjacent generated gait-table targets; servo output is still smoothed later. */
#define EDOG_12DOF_GAIT_MAX_JOINT_STEP_CENTI   360
/*
 * 后腿关节补偿：后腿因 REAR_FOOT_UP_OFFSET 伸直后，IK 对抬腿的灵敏度下降。
 * 在摆动相对站立姿态的关节增量上乘以补偿倍数，使后腿实际抬腿幅度与前腿一致。
 * femur(大腿)补偿约 1.65x，tibia(小腿)补偿约 1.23x（由 IK 几何计算得出）。
 */
#define EDOG_12DOF_REAR_FEMUR_BOOST_NUM        165
#define EDOG_12DOF_REAR_FEMUR_BOOST_DEN        100
#define EDOG_12DOF_REAR_TIBIA_BOOST_NUM        123
#define EDOG_12DOF_REAR_TIBIA_BOOST_DEN        100
#define EDOG_12DOF_TROT_SWING_PORTION          0.38
#define EDOG_12DOF_CRAWL_SWING_PORTION         0.20
#define EDOG_12DOF_CRAWL_PHASE_SPACING         0.25
#define EDOG_12DOF_CRAWL_SUPPORT_CALF_BIAS_DEG -4
#define EDOG_12DOF_CRAWL_PRE_SHIFT_HIP_BIAS_DEG 3
#define EDOG_12DOF_CRAWL_BALANCE_HIP_BIAS_DEG  2
#define EDOG_12DOF_REALTIME_SERVO_STEP_DELAY_US 100
#define EDOG_12DOF_REALTIME_MAX_SIDE_MM        28.0
#define EDOG_12DOF_REALTIME_MAX_YAW_MM         26.0
#define EDOG_12DOF_REALTIME_STANCE_PRESS_MM    3.0
#define EDOG_12DOF_REALTIME_TROT_SWING_PORTION 0.50
#define EDOG_12DOF_REALTIME_LEGACY_MIN_LIFT_MM 0.0
/* Realtime crawl foot path tuning: swing first lifts, then visibly reaches forward. */
#define EDOG_12DOF_SWING_FORWARD_SCALE_PERCENT 100
#define EDOG_12DOF_STANCE_PUSH_SCALE_PERCENT   100
#define EDOG_12DOF_SWING_FORWARD_DELAY_PERCENT 20
#define EDOG_12DOF_SWING_FORWARD_COMPLETE_PERCENT 75
#define EDOG_12DOF_STANCE_RETURN_TO_NEUTRAL_PERCENT 3
#define EDOG_PUPPER_NUM_PHASES                 4
#define EDOG_PUPPER_OVERLAP_TICKS              3
#define EDOG_PUPPER_SWING_TICKS                5
#define EDOG_PUPPER_STANCE_TICKS               (2 * EDOG_PUPPER_OVERLAP_TICKS + EDOG_PUPPER_SWING_TICKS)
#define EDOG_PUPPER_PHASE_LENGTH               (2 * EDOG_PUPPER_OVERLAP_TICKS + 2 * EDOG_PUPPER_SWING_TICKS)
#define EDOG_PUPPER_DT_SEC                     ((double)EDOG_12DOF_GAIT_FRAME_PERIOD_US / 1000000.0)
#define EDOG_PUPPER_ALPHA                      0.5
#define EDOG_PUPPER_BETA                       0.5
#define EDOG_PUPPER_COMMAND_VELOCITY_SCALE     5.5
#define EDOG_PUPPER_MIN_LIFT_MM                0.0
#define EDOG_PUPPER_YAW_RADIUS_MM              90.0
#define EDOG_PUPPER_YAW_ARM_MM                 61.0  /* was 45.0; body +32mm front-to-rear, arm +16mm */
#define EDOG_STATIC_CRAWL_NUM_PHASES           8
#define EDOG_STATIC_CRAWL_OVERLAP_TICKS        5
#define EDOG_STATIC_CRAWL_SWING_TICKS          5
#define EDOG_STATIC_CRAWL_STANCE_TICKS         \
    (3 * EDOG_STATIC_CRAWL_OVERLAP_TICKS + EDOG_STATIC_CRAWL_SWING_TICKS)
#define EDOG_STATIC_CRAWL_PHASE_LENGTH         \
    (4 * EDOG_STATIC_CRAWL_OVERLAP_TICKS + 4 * EDOG_STATIC_CRAWL_SWING_TICKS)
/*
 * 步态舵机平滑输出：
 * next = current + clamp((target-current) * NUM / DEN, +/- MAX_STEP)
 * 数字舵机仍然按目标位置闭环，但设备端不再每帧阶跃跳到新目标。
 */
#define EDOG_SERVO_TRAPEZOID_MAX_SPEED_CENTI_PER_FRAME   420
#define EDOG_SERVO_TRAPEZOID_MAX_ACCEL_CENTI_PER_FRAME2  140
#define EDOG_SERVO_TRAPEZOID_TARGET_EPS_CENTI            2

/*
 * 12DOF IMU 姿态自稳。
 * MPU6050 安装坐标：X 向前，Y 向机身左侧，Z 向上。
 * 自稳只使用 roll/pitch，不闭环 yaw，避免正常转向被拉回。
 */
#define EDOG_12DOF_IMU_BALANCE_ENABLED          1
#define EDOG_12DOF_IMU_BALANCE_ROLL_TARGET_CENTI_DEG   0
#define EDOG_12DOF_IMU_BALANCE_PITCH_TARGET_CENTI_DEG  0
#define EDOG_12DOF_IMU_BALANCE_DEADBAND_CENTI_DEG      80
#define EDOG_12DOF_IMU_BALANCE_MAX_TILT_CENTI_DEG      1800
#define EDOG_12DOF_IMU_BALANCE_MIN_ACCEL_MG            650
#define EDOG_12DOF_IMU_BALANCE_MAX_ACCEL_MG            1350
#define EDOG_IMU_FUSION_SAMPLE_INTERVAL_MS             5
#define EDOG_IMU_FUSION_COMPLEMENTARY_ALPHA_PERCENT    97
#define EDOG_12DOF_IMU_BALANCE_D_TERM_PERCENT          15
#define EDOG_12DOF_IMU_BALANCE_ROLL_KP_MM_PER_DEG      1.2
#define EDOG_12DOF_IMU_BALANCE_PITCH_KP_MM_PER_DEG     0.6
#define EDOG_12DOF_IMU_BALANCE_ROLL_KD_MM_PER_DEG_PER_SEC   0.050
#define EDOG_12DOF_IMU_BALANCE_PITCH_KD_MM_PER_DEG_PER_SEC  0.025
#define EDOG_12DOF_IMU_BALANCE_MAX_FOOT_Z_MM           16.0
#define EDOG_12DOF_IMU_BALANCE_SWING_SCALE_PERCENT     50
#define EDOG_12DOF_IMU_BALANCE_FAIL_DECAY_PERCENT      70
#define EDOG_12DOF_IMU_BALANCE_STOP_SETTLE_MS          800
#define EDOG_12DOF_IMU_BALANCE_STOP_MAX_SETTLE_MS      2500
#define EDOG_12DOF_IMU_BALANCE_STOP_STABLE_FRAMES      15
#define EDOG_12DOF_IMU_BALANCE_STOP_STABLE_TILT_CENTI_DEG 400
#define EDOG_12DOF_IMU_BALANCE_STOP_EXIT_TILT_CENTI_DEG   600
#define EDOG_12DOF_IMU_BALANCE_STOP_STABLE_RATE_CENTI_DPS 8000
#define EDOG_12DOF_IMU_BALANCE_STOP_EXIT_RATE_CENTI_DPS   12000
#define EDOG_12DOF_IMU_BALANCE_RAMP_IN_MS              1200
#define EDOG_12DOF_IMU_BALANCE_ROLL_STEP_LIMIT_CENTI_DEG  167
#define EDOG_12DOF_IMU_BALANCE_PITCH_STEP_LIMIT_CENTI_DEG 167
#define EDOG_12DOF_IMU_BALANCE_INTEGRAL_KI_PERCENT     0
#define EDOG_12DOF_IDLE_BALANCE_MANUAL_PAUSE_FRAMES    90
#define EDOG_12DOF_IMU_BALANCE_DEBUG_ENABLED    0
#define EDOG_12DOF_IMU_BALANCE_DEBUG_INTERVAL_FRAMES   30

/* MPU6050 forward motion -> RGB tricolor rotating light */
#define EDOG_MPU_RGB_LIGHT_TASK_ENABLED          0
#define EDOG_MPU_FORWARD_AXIS                  0
#define EDOG_MPU_FORWARD_DIRECTION             1
#define EDOG_MPU_MOTION_THRESHOLD_MG           180
#define EDOG_MPU_MOTION_RELEASE_MG             80
#define EDOG_MPU_TILT_THRESHOLD_MG             220
#define EDOG_MPU_SAMPLE_INTERVAL_MS            10
#define EDOG_MPU_INIT_RETRY_MS                 3000
#define EDOG_MPU_BASELINE_SAMPLES              24
#define EDOG_RGB_ACTIVE_HOLD_MS                1600
#define EDOG_RGB_ROTATE_INTERVAL_MS            120
#define EDOG_RGB_ACTIVE_HIGH                   0
#define EDOG_RGB_LED_R_GPIO                    GPIO0_PB5
#define EDOG_RGB_LED_G_GPIO                    GPIO0_PB4
#define EDOG_RGB_LED_B_GPIO                    GPIO1_PD0

/* Task layout */
#define EDOG_TASK_SERVO_INIT_STACK_SIZE        4096
#define EDOG_TASK_SERVO_INIT_PRIORITY          26
#define EDOG_TASK_MOTION_STACK_SIZE            8192
#define EDOG_TASK_MOTION_PRIORITY              25
#define EDOG_TASK_WIFI_RESET_STACK_SIZE        2048
#define EDOG_TASK_WIFI_RESET_PRIORITY          26
#define EDOG_TASK_WIFI_MQTT_STACK_SIZE         8192
#define EDOG_TASK_WIFI_MQTT_PRIORITY           20
#define EDOG_TASK_MPU_LIGHT_STACK_SIZE         4096
#define EDOG_TASK_MPU_LIGHT_PRIORITY           27

#endif
