#!/usr/bin/env python3
import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MOTION = ROOT / "12_DOF_Version/src/motion_utils_12dof.c"


def public_body(name, text):
    match = re.search(rf"int\s+{name}\s*\([^)]*\)\s*\{{(?P<body>.*?)\n\}}", text, re.S)
    if not match:
        raise AssertionError(f"missing public function {name}")
    return match.group("body")


def static_body(name, text):
    match = re.search(
        rf"static\s+int(?:\s+EDOG_UNUSED)?\s+{name}\s*\([^)]*\)\s*\{{(?P<body>.*?)\n\}}\n\nint\s+single_leg_gait_cycle",
        text,
        re.S,
    )
    if not match:
        raise AssertionError(f"missing static function {name}")
    return match.group("body")


def check_static_crawl_contact_table(text):
    match = re.search(
        r"g_staticCrawlContactPhases\s*\[[^\]]+\]\s*\[[^\]]+\]\s*=\s*\{(?P<body>.*?)\n\};",
        text,
        re.S,
    )
    if not match:
        raise AssertionError("missing static crawl contact phase table")
    rows = re.findall(r"\{([^{}]+)\}", match.group("body"))
    if len(rows) != 4:
        raise AssertionError("static crawl contact table must have four leg rows")
    contacts = []
    for row in rows:
        values = [int(value) for value in re.findall(r"\b[01]\b", row)]
        if len(values) != 8:
            raise AssertionError("each static crawl contact row must have eight phases")
        contacts.append(values)

    expected_swing_by_phase = {1: 0, 3: 3, 5: 1, 7: 2}
    for phase in range(8):
        swing_legs = [leg for leg, row in enumerate(contacts) if row[phase] == 0]
        if phase in expected_swing_by_phase:
            if swing_legs != [expected_swing_by_phase[phase]]:
                raise AssertionError(f"phase {phase} must swing only leg {expected_swing_by_phase[phase]}")
        elif swing_legs:
            raise AssertionError(f"body shift phase {phase} must keep all four legs in contact")


def require_trot_body(name, body, expected):
    if expected not in body:
        raise AssertionError(f"{name} must route to {expected}")
    if "spotMicroCrawlCycleInternal" in body:
        raise AssertionError(f"{name} must not route to SpotMicro crawl")
    if "runServoGaitTableCycle" in body:
        raise AssertionError(f"{name} must not use servo gait table scheduler")


def require_turn_body(name, body, expected):
    if expected not in body:
        raise AssertionError(f"{name} must route to {expected}")
    if "trotTurnCycleInternal" in body:
        raise AssertionError(f"{name} must not use legacy forward/reverse turn mixer")
    if "spotMicroCrawlCycleInternal" in body:
        raise AssertionError(f"{name} must not route to SpotMicro crawl")


def main():
    text = MOTION.read_text(encoding="utf-8")

    if "static const int g_crawlSwingOrder[EDOG_12DOF_LEG_COUNT] = {0, 3, 1, 2}" not in text:
        raise AssertionError("crawl swing order must stay available for crawl_cycle")
    check_static_crawl_contact_table(text)

    for token in [
        "static int spotMicroCrawlCycleInternal",
        "staticCrawlNextSwingLeg",
        "static double pyAppleCrawlCycloidProgress",
        "static double pyAppleCrawlCycloidLift",
        "pyAppleCrawlCycloidProgress(t)",
        "pyAppleCrawlCycloidLift(swingT)",
        "applyHipWeightShiftBias(frame, swingState, preShiftLeg",
        "applyRealtimeBalanceBias(frame, swingState)",
    ]:
        if token not in text:
            raise AssertionError(f"crawl implementation missing backup token: {token}")
    for token in [
        "spotMicroCrawlSwingLift",
        "spotMicroCrawlBezier",
    ]:
        if token in text:
            raise AssertionError(f"crawl implementation must not use old SpotMicro swing helper: {token}")

    require_trot_body(
        "trot_cycle",
        public_body("trot_cycle", text),
        "trotCycleInternal(step_length, step_height, 0, 100, 100)",
    )
    require_trot_body(
        "trot_back_cycle",
        public_body("trot_back_cycle", text),
        "trotCycleInternal(step_length, step_height, 1, 100, 100)",
    )
    require_turn_body(
        "diversion_right_cycle",
        public_body("diversion_right_cycle", text),
        "runServoGaitTableCycle(EDOG_TABLE_MODE_TURN_RIGHT, step_length, step_height, 0x0F)",
    )
    require_turn_body(
        "diversion_left_cycle",
        public_body("diversion_left_cycle", text),
        "runServoGaitTableCycle(EDOG_TABLE_MODE_TURN_LEFT, step_length, step_height, 0x0F)",
    )

    in_place = public_body("trot_in_place_cycle", text)
    if "spotMicroCrawlCycleInternal" in in_place:
        raise AssertionError("trot_in_place_cycle must not route to SpotMicro crawl")
    if "trotCycleInternal(0.0, step_height, 0, 100, 100)" not in in_place:
        raise AssertionError("trot_in_place_cycle must use the continuous trot in-place path")

    crawl_body = public_body("crawl_cycle", text)
    if "spotMicroCrawlCycleInternal(step_length, 0.0, 0.0, step_height)" not in crawl_body:
        raise AssertionError("crawl_cycle should remain the explicit SpotMicro crawl backup entry")

    leg_group = public_body("leg_group_gait_cycle", text)
    if "runServoGaitTableCycle(EDOG_TABLE_MODE_FORWARD, step_length, step_height, leg_mask)" not in leg_group:
        raise AssertionError("leg_group_gait_cycle must keep table scheduler for leg-mask debug output")

    crawl_kernel = static_body("spotMicroCrawlCycleInternal", text)
    if "crawlMode" not in crawl_kernel or "runRealtimeGaitCycle(&cmd)" not in crawl_kernel:
        raise AssertionError("SpotMicro crawl kernel must stay available for crawl_cycle")

    print("12DOF default whole-body motion entries use trot; crawl remains explicit backup")


if __name__ == "__main__":
    try:
        main()
    except AssertionError as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        sys.exit(1)
