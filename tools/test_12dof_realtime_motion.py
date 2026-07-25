#!/usr/bin/env python3
import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "include/edog_config.h"
MOTION = ROOT / "12_DOF_Version/src/motion_utils_12dof.c"


def body_of(name, text):
    match = re.search(rf"static\s+int(?:\s+EDOG_UNUSED)?\s+{name}\s*\([^)]*\)\s*\{{(?P<body>.*?)\n\}}", text, re.S)
    if not match:
        raise AssertionError(f"missing {name}")
    return match.group("body")


def public_body_of(name, text):
    match = re.search(rf"int\s+{name}\s*\([^)]*\)\s*\{{(?P<body>.*?)\n\}}", text, re.S)
    if not match:
        raise AssertionError(f"missing {name}")
    return match.group("body")


def main():
    config = CONFIG.read_text(encoding="utf-8")
    text = MOTION.read_text(encoding="utf-8")

    for token in [
        "#define EDOG_12DOF_HARDWARE_FORWARD_SIGN       -1.0",
        "#define EDOG_12DOF_REALTIME_SERVO_STEP_DELAY_US 100",
        "#define EDOG_12DOF_SWING_FORWARD_SCALE_PERCENT",
        "#define EDOG_12DOF_STANCE_PUSH_SCALE_PERCENT",
        "#define EDOG_12DOF_SWING_FORWARD_DELAY_PERCENT",
        "#define EDOG_12DOF_SWING_FORWARD_COMPLETE_PERCENT",
        "#define EDOG_SERVO_HIP_SPEED_60_DEG_MS",
        "#define EDOG_SERVO_LEG_SPEED_60_DEG_MS",
        "#define EDOG_SERVO_SPEED_SAFETY_NUM",
        "#define EDOG_SERVO_SPEED_SAFETY_DEN",
    ]:
        if token not in config:
            raise AssertionError(f"missing realtime gait config token: {token}")

    required = [
        "typedef struct {\n    int centiDeg[EDOG_SERVO_CHANNEL_COUNT];",
        "typedef struct {\n    Edog12DofServoFrame frames[EDOG_12DOF_GAIT_FRAME_COUNT];",
        "typedef struct {\n    int mode;",
        "static void buildServoGaitTable",
        "static int runServoGaitTableCycle",
        "static int computeServoTableMinFrameMs",
        "static int getSafeServoCentiDegPerMs",
        "static int approachServoTargetCenti",
        "static double pupperCommandDeltaMm",
        "static double pyAppleCrawlCycloidProgress",
        "static double pyAppleCrawlCycloidLift",
        "pyAppleCrawlCycloidProgress(t)",
        "pyAppleCrawlCycloidLift(swingT)",
        "static double legacyPupperSwingLift",
        "legacyPupperSwingLift(swingT)",
        "static double pupperSwingHeight",
        "touchdown.xMm",
        "yawStepMm",
        "swingLift",
        "hardwareStrideMm = EDOG_12DOF_HARDWARE_FORWARD_SIGN * deltaXmm",
        "hardwareDx = EDOG_12DOF_HARDWARE_FORWARD_SIGN * dx",
        "Edog12Dof_IK(&state->leg[leg].foot",
        "int balance_stand_frame(void)",
        "getImuBalanceControlCentiDeg(&rollControlCentiDeg, &pitchControlCentiDeg)",
        "static int spotMicroCrawlCycleInternal",
        "staticCrawlNextSwingLeg",
        "Edog12DofContinuousTrotRuntime",
        "g_continuousTrotRuntime.frameIndex",
        "trotCycleInternal",
        "trotTurnCycleInternal",
    ]
    for token in required:
        if token not in text:
            raise AssertionError(f"missing table gait token: {token}")
    for token in [
        "spotMicroCrawlSwingLift",
        "spotMicroCrawlBezier",
    ]:
        if token in text:
            raise AssertionError(f"realtime crawl must not use old SpotMicro swing helper: {token}")

    for name, expected in {
        "trot_cycle": "trotCycleInternal(step_length, step_height, 0, 100, 100)",
        "trot_back_cycle": "trotCycleInternal(step_length, step_height, 1, 100, 100)",
    }.items():
        body = public_body_of(name, text)
        if expected not in body:
            raise AssertionError(f"{name} must use default trot path {expected}")
        if "spotMicroCrawlCycleInternal" in body:
            raise AssertionError(f"{name} must not use SpotMicro crawl path")
        if "runServoGaitTableCycle" in body:
            raise AssertionError(f"{name} must not use the servo gait table scheduler")

    for name, expected in {
        "diversion_left_cycle": "runServoGaitTableCycle(EDOG_TABLE_MODE_TURN_LEFT, step_length, step_height, 0x0F)",
        "diversion_right_cycle": "runServoGaitTableCycle(EDOG_TABLE_MODE_TURN_RIGHT, step_length, step_height, 0x0F)",
    }.items():
        body = public_body_of(name, text)
        if expected not in body:
            raise AssertionError(f"{name} must use yaw turn table path {expected}")
        if "spotMicroCrawlCycleInternal" in body:
            raise AssertionError(f"{name} must not use SpotMicro crawl path")
        if "trotTurnCycleInternal" in body:
            raise AssertionError(f"{name} must not use legacy forward/reverse turn mixer")

    if "trotCycleInternal(0.0, step_height, 0, 100, 100)" not in public_body_of("trot_in_place_cycle", text):
        raise AssertionError("trot_in_place_cycle must use the continuous trot in-place path")
    if "spotMicroCrawlCycleInternal" in public_body_of("trot_in_place_cycle", text):
        raise AssertionError("trot_in_place_cycle must not use SpotMicro crawl path")
    if "spotMicroCrawlCycleInternal(step_length, 0.0, 0.0, step_height)" not in public_body_of("crawl_cycle", text):
        raise AssertionError("crawl_cycle must remain the explicit SpotMicro crawl path")

    for name in [
        "trot_left_front_cycle",
        "trot_right_front_cycle",
        "trot_left_back_cycle",
        "trot_right_back_cycle",
    ]:
        body = public_body_of(name, text)
        if "return 0;" not in body or "not supported" not in body:
            raise AssertionError(f"{name} must reject unsupported arc gait")
        if "trotArcCycleInternal" in body or "runServoGaitTableCycle" in body:
            raise AssertionError(f"{name} must not plan active arc gait")

    if "trotArcCycleInternal" in text:
        raise AssertionError("arc gait planner must not remain in active 12DOF motion code")
    if "EDOG_TABLE_MODE_ARC" in text:
        raise AssertionError("arc table modes must not remain in active 12DOF table gait")

    single_leg_body = public_body_of("single_leg_gait_cycle", text)
    leg_group_body = public_body_of("leg_group_gait_cycle", text)
    if "return leg_group_gait_cycle(1 << leg_index, step_length, step_height);" not in single_leg_body:
        raise AssertionError("single_leg_gait_cycle must delegate to leg_group_gait_cycle")
    if "runServoGaitTableCycle(EDOG_TABLE_MODE_FORWARD, step_length, step_height, leg_mask)" not in leg_group_body:
        raise AssertionError("leg_group_gait_cycle must keep the servo gait table scheduler")

    if re.search(r"int\s+(?:crawlDirectionalCycleInternal|trotDirectionalCycleInternal)\s*\(", text):
        raise AssertionError("old directional realtime entry points must be removed or made non-entry dead code")
    if "double deltaXmm = pupperCommandDeltaMm(cmd, cmd->vxMps);" not in text:
        raise AssertionError("touchdown planning must use the Pupper command delta helper")

    contact_match = re.search(
        r"g_staticCrawlContactPhases\s*\[[^\]]+\]\s*\[[^\]]+\]\s*=\s*\{(?P<body>.*?)\n\};",
        text,
        re.S,
    )
    if not contact_match:
        raise AssertionError("missing static crawl contact phase table")
    rows = re.findall(r"\{([^{}]+)\}", contact_match.group("body"))
    if len(rows) != 4:
        raise AssertionError("static crawl contact table must have four leg rows")
    contacts = []
    for row in rows:
        values = [int(value) for value in re.findall(r"\b[01]\b", row)]
        if len(values) != 8:
            raise AssertionError("each static crawl contact row must have eight phases")
        contacts.append(values)
    for phase in range(8):
        contact_count = sum(row[phase] for row in contacts)
        swing_count = 4 - contact_count
        if swing_count > 1:
            raise AssertionError(f"phase {phase} swings more than one leg")
        if contact_count < 3:
            raise AssertionError(f"phase {phase} has fewer than three support legs")
        if phase % 2 == 0 and swing_count != 0:
            raise AssertionError(f"body shift phase {phase} must keep all legs on the ground")
        if phase % 2 == 1 and swing_count != 1:
            raise AssertionError(f"swing phase {phase} must lift exactly one leg")

    stance_body = re.search(
        r"static\s+void\s+updatePupperStanceFoot\s*\([^)]*\)\s*\{(?P<body>.*?)\n\}",
        text,
        re.S,
    )
    if not stance_body:
        raise AssertionError("missing updatePupperStanceFoot")
    if "EDOG_12DOF_REALTIME_STANCE_PRESS_MM" in stance_body.group("body"):
        raise AssertionError("realtime/crawl stance must keep neutral foot Z")

    print("12DOF realtime support remains; default whole-body entries use trot")


if __name__ == "__main__":
    try:
        main()
    except AssertionError as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        sys.exit(1)
