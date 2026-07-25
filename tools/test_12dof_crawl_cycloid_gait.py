#!/usr/bin/env python3
import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MOTION = ROOT / "12_DOF_Version/src/motion_utils_12dof.c"


def function_body(signature_prefix, name, text):
    match = re.search(
        rf"{signature_prefix}\s+{name}\s*\([^)]*\)\s*\{{(?P<body>.*?)\n\}}",
        text,
        re.S,
    )
    if not match:
        raise AssertionError(f"missing function {name}")
    return match.group("body")


def require(token, text, context):
    if token not in text:
        raise AssertionError(f"{context} missing {token}")


def forbid(token, text, context):
    if token in text:
        raise AssertionError(f"{context} must not contain {token}")


def main():
    motion = MOTION.read_text(encoding="utf-8")

    require("static double pyAppleCrawlCycloidProgress(double t)", motion, "crawl cycloid progress helper")
    require("static double pyAppleCrawlCycloidLift(double t)", motion, "crawl cycloid lift helper")
    require("(sigma - sin(sigma)) / (2.0 * M_PI)", motion, "crawl cycloid progress formula")
    require("(1.0 - cos(sigma)) / 2.0", motion, "crawl cycloid lift formula")

    swing_body = function_body("static void", "updatePupperSwingFoot", motion)
    height_body = function_body("static double", "pupperSwingHeight", motion)
    require("pyAppleCrawlCycloidProgress(t)", swing_body, "crawl swing trajectory")
    require("cmd != NULL && cmd->crawlMode", swing_body, "crawl swing trajectory mode guard")
    require("static double legacyPupperSwingLift(double swingT)", motion, "legacy non-crawl swing lift")
    require("cmd->crawlMode", height_body, "crawl swing lift mode guard")
    require("pyAppleCrawlCycloidLift(swingT)", height_body, "crawl swing lift")
    require("legacyPupperSwingLift(swingT)", height_body, "legacy non-crawl swing lift")
    require("defaultFootZMm - swingHeightMm", swing_body, "crawl swing Z-down lift")
    require("Edog12Dof_FootYOnHipPlaneForLeg(leg, legState->foot.zMm)", swing_body, "crawl swing hip-plane projection")
    forbid("spotMicroCrawlBezier", swing_body, "crawl swing trajectory")
    forbid("controlX", swing_body, "crawl swing trajectory")
    forbid("controlY", swing_body, "crawl swing trajectory")

    stance_body = function_body("static void", "updatePupperStanceFoot", motion)
    require("cmd != NULL && cmd->crawlMode", stance_body, "crawl stance mode branch")
    require("legState->foot.zMm = neutral.zMm;", stance_body, "crawl stance neutral Z")

    for token in [
        "static const int g_crawlSwingOrder[EDOG_12DOF_LEG_COUNT] = {0, 3, 1, 2}",
        "staticCrawlNextSwingLeg",
        "applyHipWeightShiftBias(frame, swingState, preShiftLeg",
        "applyRealtimeBalanceBias(frame, swingState)",
        "spotMicroCrawlCycleInternal(step_length, 0.0, 0.0, step_height)",
    ]:
        require(token, motion, "crawl backup behavior")

    print("12DOF crawl/walk Py-Apple cycloid trajectory checks passed")


if __name__ == "__main__":
    try:
        main()
    except AssertionError as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        sys.exit(1)
