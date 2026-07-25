#!/usr/bin/env python3
import subprocess
import tempfile
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def main():
    source = r'''
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "include/edog_config.h"
#include "12_DOF_Version/include/gait_generate_12dof.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static void require_int_close(const char *name, int got, int expected, int tolerance)
{
    int diff = got - expected;
    if (diff < 0) {
        diff = -diff;
    }
    if (diff > tolerance) {
        fprintf(stderr, "%s expected %d +/- %d, got %d\n", name, expected, tolerance, got);
        exit(1);
    }
}

int main(void)
{
    Edog12DofJointAngles gait[EDOG_12DOF_TROT_FRAME_COUNT];
    Edog12DofJointAngles reverse[EDOG_12DOF_TROT_FRAME_COUNT];
    Edog12DofJointAngles legacy[EDOG_12DOF_TROT_FRAME_COUNT];
    Edog12DofJointAngles directional[EDOG_12DOF_TROT_FRAME_COUNT];
    Edog12DofJointAngles leftSide[EDOG_12DOF_TROT_FRAME_COUNT];
    Edog12DofJointAngles rightSide[EDOG_12DOF_TROT_FRAME_COUNT];
    Edog12DofJointAngles yawFront[EDOG_12DOF_TROT_FRAME_COUNT];
    Edog12DofJointAngles yawBack[EDOG_12DOF_TROT_FRAME_COUNT];
    Edog12DofJointAngles hipPlaneLeft[EDOG_12DOF_TROT_FRAME_COUNT];
    Edog12DofJointAngles hipPlaneRight[EDOG_12DOF_TROT_FRAME_COUNT];
    Edog12DofFootPoint standFoot = {-150.0, 0.0, 160.0};
    Edog12DofJointAngles standIk;
    int minFemur;
    int maxFemur;
    int minTibia;
    int maxTibia;
    int maxLeftSideHip;
    int minRightSideHip;
    int maxYawFrontHip;
    int minYawBackHip;
    int minHipPlaneLeft;
    int maxHipPlaneLeft;
    int minHipPlaneRight;
    int maxHipPlaneRight;
    int subDegreeFrames = 0;
    double runtimeThigh;
    double runtimeCalf;
    double runtimeHip;
    double runtimeFrontHip;
    double runtimeRearHip;
    Edog12DofJointAngles longLinkIk;
    Edog12DofFootPoint frontDefaultFoot;
    Edog12DofFootPoint rearDefaultFoot;
    Edog12DofJointAngles frontDefaultIk;
    Edog12DofJointAngles rearDefaultIk;

    if (EDOG_12DOF_GAIT_FRAME_COUNT != 30) {
        fprintf(stderr, "servo gait table must use 30 target frames by default, got %d\n",
                EDOG_12DOF_GAIT_FRAME_COUNT);
        return 1;
    }
    if (fabs(EDOG_12DOF_L_BODY_MM - 190.0) > 0.001 ||
        fabs(EDOG_12DOF_W_BODY_MM - 87.0) > 0.001 ||
        fabs(EDOG_12DOF_L1_MM - 50.0) > 0.001 ||
        fabs(EDOG_12DOF_L2_MM - 107.0) > 0.001 ||
        fabs(EDOG_12DOF_L3_MM - 135.0) > 0.001) {
        fprintf(stderr, "mechanical constants must match measured hardware L_body=190 W_body=87 L1=50 L2=107 L3=135\n");
        return 1;
    }
    if (EDOG_SERVO_HIP_SPEED_60_DEG_MS != 330 ||
        EDOG_SERVO_LEG_SPEED_60_DEG_MS != 250 ||
        EDOG_SERVO_SPEED_SAFETY_NUM != 8 ||
        EDOG_SERVO_SPEED_SAFETY_DEN != 10) {
        fprintf(stderr, "servo speed constants must encode hip=0.33s/60deg, thigh/calf=0.25s/60deg with 80%% safety\n");
        return 1;
    }
    if (fabs(SPOTMICRO_THIGH_LENGTH_MM - 107.0) > 0.001 ||
        fabs(SPOTMICRO_CALF_LENGTH_MM - 135.0) > 0.001) {
        fprintf(stderr, "IK link lengths must match measured hardware: thigh=107mm calf=135mm\n");
        return 1;
    }

    if (Edog12Dof_IK(&standFoot, 0, &standIk) != 0) {
        fprintf(stderr, "standing-foot IK failed\n");
        return 1;
    }

    require_int_close("stand hip", standIk.hipAngleDeg, EDOG_12DOF_STAND_HIP_DELTA_DEG, 1);
    if (standIk.hipAngleDeg != 0) {
        fprintf(stderr, "standing hip should stay centered at 0 deg, got %d\n",
                standIk.hipAngleDeg);
        return 1;
    }
    {
        Edog12DofFootPoint rightStandFoot = {-150.0, 0.0, 160.0};
        Edog12DofJointAngles rightStandIk;
        if (Edog12Dof_IK(&rightStandFoot, 1, &rightStandIk) != 0) {
            fprintf(stderr, "right standing-foot IK failed\n");
            return 1;
        }
        require_int_close("right stand hip", rightStandIk.hipAngleDeg,
                          EDOG_12DOF_STAND_HIP_DELTA_DEG, 1);
        if (rightStandIk.hipAngleDeg != 0) {
            fprintf(stderr, "right standing hip should also stay centered at 0 deg, got %d\n",
                    rightStandIk.hipAngleDeg);
            return 1;
        }
    }
    require_int_close("stand femur", standIk.femurAngleDeg, EDOG_12DOF_STAND_THIGH_DELTA_DEG, 2);
    require_int_close("stand tibia", standIk.tibiaAngleDeg, EDOG_12DOF_STAND_CALF_DELTA_DEG, 2);
    Edog12Dof_GetRuntimeGeometryForFrontRear(&runtimeFrontHip, &runtimeRearHip, &runtimeThigh, &runtimeCalf);
    if (fabs(runtimeFrontHip - EDOG_12DOF_FRONT_HIP_ADDUCTION_DEFAULT_DEG) > 0.001 ||
        fabs(runtimeRearHip - EDOG_12DOF_REAR_HIP_ADDUCTION_DEFAULT_DEG) > 0.001 ||
        fabs(runtimeThigh - 107.0) > 0.001 ||
        fabs(runtimeCalf - 135.0) > 0.001) {
        fprintf(stderr, "runtime geometry defaults must use configured front/rear hip adduction\n");
        return 1;
    }
    {
        double defaultFrontPlaneY = tan(EDOG_12DOF_FRONT_HIP_ADDUCTION_DEFAULT_DEG * M_PI / 180.0) *
            Edog12Dof_DefaultFootZForLeg(1);
        double defaultRearPlaneY = tan(EDOG_12DOF_REAR_HIP_ADDUCTION_DEFAULT_DEG * M_PI / 180.0) *
            Edog12Dof_DefaultFootZForLeg(0);
        if (fabs(Edog12Dof_DefaultFootYForLeg(0) - defaultFrontPlaneY) > 0.01 ||
            fabs(Edog12Dof_DefaultFootYForLeg(1) + defaultFrontPlaneY) > 0.01 ||
            fabs(Edog12Dof_DefaultFootYForLeg(2) - defaultRearPlaneY) > 0.01 ||
            fabs(Edog12Dof_DefaultFootYForLeg(3) + defaultRearPlaneY) > 0.01) {
            fprintf(stderr, "default foot Y must use independent front/rear hip adduction\n");
            return 1;
        }
    }
    Edog12Dof_SetRuntimeGeometry(-8.0, 120.0, 150.0);
    Edog12Dof_GetRuntimeGeometry(&runtimeHip, &runtimeThigh, &runtimeCalf);
    if (fabs(runtimeHip + 8.0) > 0.001 ||
        fabs(runtimeThigh - 120.0) > 0.001 ||
        fabs(runtimeCalf - 150.0) > 0.001) {
        fprintf(stderr, "runtime geometry setter/getter did not preserve values\n");
        return 1;
    }
    Edog12Dof_SetRuntimeGeometryForFrontRear(-20.0, -5.0, 107.0, 135.0);
    Edog12Dof_GetRuntimeGeometryForFrontRear(&runtimeFrontHip, &runtimeRearHip, &runtimeThigh, &runtimeCalf);
    if (fabs(runtimeFrontHip + 20.0) > 0.001 ||
        fabs(runtimeRearHip + 5.0) > 0.001 ||
        fabs(runtimeThigh - 107.0) > 0.001 ||
        fabs(runtimeCalf - 135.0) > 0.001) {
        fprintf(stderr, "front/rear runtime geometry setter/getter did not preserve values\n");
        return 1;
    }
    Edog12Dof_SetRuntimeGeometry(-8.0, 120.0, 150.0);
    {
        double hipPlaneFrontY = tan(-8.0 * M_PI / 180.0) *
            Edog12Dof_DefaultFootZForLeg(1);
        double hipPlaneRearY = tan(-8.0 * M_PI / 180.0) *
            Edog12Dof_DefaultFootZForLeg(0);
        if (fabs(Edog12Dof_DefaultFootYForLeg(0) - hipPlaneFrontY) > 0.01 ||
            fabs(Edog12Dof_DefaultFootYForLeg(1) + hipPlaneFrontY) > 0.01 ||
            fabs(Edog12Dof_DefaultFootYForLeg(2) - hipPlaneRearY) > 0.01 ||
            fabs(Edog12Dof_DefaultFootYForLeg(3) + hipPlaneRearY) > 0.01) {
            fprintf(stderr,
                    "runtime hip adduction must project mirrored default foot Y from each leg's own default Z\n");
            return 1;
        }
    }
    if (fabs(Edog12Dof_DefaultFootXForLeg(1) - EDOG_12DOF_DEFAULT_FOOT_X_MM) > 0.001 ||
        fabs(Edog12Dof_DefaultFootXForLeg(0) -
             (EDOG_12DOF_DEFAULT_FOOT_X_MM - EDOG_12DOF_REAR_FOOT_BACK_OFFSET_MM)) > 0.001) {
        fprintf(stderr, "front legs must keep default foot-X and rear legs must move backward by the configured offset\n");
        return 1;
    }
    if (fabs(Edog12Dof_DefaultFootZForLeg(1) - EDOG_12DOF_DEFAULT_FOOT_Z_MM) > 0.001 ||
        fabs(Edog12Dof_DefaultFootZForLeg(0) -
             (EDOG_12DOF_DEFAULT_FOOT_Z_MM - EDOG_12DOF_REAR_FOOT_UP_OFFSET_MM)) > 0.001) {
        fprintf(stderr, "front legs must keep default foot-Z and rear legs must use the configured rear Z offset\n");
        return 1;
    }
    if (fabs(Edog12Dof_DefaultFootZForLeg(1) - 140.0) > 0.001 ||
        fabs(Edog12Dof_DefaultFootZForLeg(0) - 140.0) > 0.001 ||
        fabs(Edog12Dof_DefaultFootZForLeg(1) - Edog12Dof_DefaultFootZForLeg(0)) > 0.001) {
        fprintf(stderr, "front/rear default foot-Z must both be 140mm: front=%.1f rear=%.1f\n",
                Edog12Dof_DefaultFootZForLeg(1), Edog12Dof_DefaultFootZForLeg(0));
        return 1;
    }
    Edog12Dof_SetRuntimeFootZDeltas(10.0, -5.0);
    if (fabs(Edog12Dof_DefaultFootZForLeg(1) - 150.0) > 0.001 ||
        fabs(Edog12Dof_DefaultFootZForLeg(0) - 135.0) > 0.001) {
        fprintf(stderr,
                "runtime front/rear body-height deltas must change default foot-Z independently: front=%.1f rear=%.1f\n",
                Edog12Dof_DefaultFootZForLeg(1), Edog12Dof_DefaultFootZForLeg(0));
        return 1;
    }
    Edog12Dof_SetRuntimeFootZDeltas(0.0, 0.0);
    if (Edog12Dof_IK(&standFoot, 0, &longLinkIk) != 0) {
        fprintf(stderr, "long-link standing-foot IK failed\n");
        return 1;
    }
    if (longLinkIk.femurAngleDeg == standIk.femurAngleDeg &&
        longLinkIk.tibiaAngleDeg == standIk.tibiaAngleDeg) {
        fprintf(stderr, "changing runtime link lengths must affect femur/tibia IK\n");
        return 1;
    }
    Edog12Dof_ResetRuntimeGeometry();
    frontDefaultFoot.xMm = Edog12Dof_DefaultFootXForLeg(1);
    frontDefaultFoot.yMm = Edog12Dof_DefaultFootYForLeg(0);
    frontDefaultFoot.zMm = Edog12Dof_DefaultFootZForLeg(1);
    rearDefaultFoot.xMm = Edog12Dof_DefaultFootXForLeg(0);
    rearDefaultFoot.yMm = Edog12Dof_DefaultFootYForLeg(2);
    rearDefaultFoot.zMm = Edog12Dof_DefaultFootZForLeg(0);
    if (Edog12Dof_IK(&frontDefaultFoot, 0, &frontDefaultIk) != 0 ||
        Edog12Dof_IK(&rearDefaultFoot, 0, &rearDefaultIk) != 0) {
        fprintf(stderr, "default front/rear IK failed\n");
        return 1;
    }
    if (fabs(rearDefaultFoot.zMm - frontDefaultFoot.zMm) > 0.001) {
        fprintf(stderr,
                "front/rear default-foot IK must start from equal Z before hip-plane differences: frontZ=%.1f rearZ=%.1f\n",
                frontDefaultFoot.zMm, rearDefaultFoot.zMm);
        return 1;
    }
    if (standIk.hipAngleCentiDeg == standIk.hipAngleDeg * EDOG_12DOF_CENTI_PER_DEG &&
        standIk.femurAngleCentiDeg == standIk.femurAngleDeg * EDOG_12DOF_CENTI_PER_DEG &&
        standIk.tibiaAngleCentiDeg == standIk.tibiaAngleDeg * EDOG_12DOF_CENTI_PER_DEG) {
        fprintf(stderr, "standing IK should retain sub-degree centi precision, got only integer angles\n");
        return 1;
    }

    Edog12Dof_SetRuntimeGeometry(-13.0, 107.0, 135.0);
    Edog12Dof_GenerateDirectionalTrotTable(hipPlaneLeft, 0.01, 0.0, 0.0, 0.006, 0, 1, 0);
    Edog12Dof_GenerateDirectionalTrotTable(hipPlaneRight, 0.01, 0.0, 0.0, 0.006, 1, 1, 0);
    minHipPlaneLeft = maxHipPlaneLeft = hipPlaneLeft[0].hipAngleCentiDeg;
    minHipPlaneRight = maxHipPlaneRight = hipPlaneRight[0].hipAngleCentiDeg;
    for (int i = 1; i < EDOG_12DOF_TROT_FRAME_COUNT; i++) {
        if (hipPlaneLeft[i].hipAngleCentiDeg < minHipPlaneLeft) {
            minHipPlaneLeft = hipPlaneLeft[i].hipAngleCentiDeg;
        }
        if (hipPlaneLeft[i].hipAngleCentiDeg > maxHipPlaneLeft) {
            maxHipPlaneLeft = hipPlaneLeft[i].hipAngleCentiDeg;
        }
        if (hipPlaneRight[i].hipAngleCentiDeg < minHipPlaneRight) {
            minHipPlaneRight = hipPlaneRight[i].hipAngleCentiDeg;
        }
        if (hipPlaneRight[i].hipAngleCentiDeg > maxHipPlaneRight) {
            maxHipPlaneRight = hipPlaneRight[i].hipAngleCentiDeg;
        }
    }
    if (abs(maxHipPlaneLeft - minHipPlaneLeft) > 6 ||
        abs(maxHipPlaneRight - minHipPlaneRight) > 6 ||
        abs(hipPlaneLeft[EDOG_12DOF_TROT_FRAME_COUNT / 4].hipAngleCentiDeg + 1300) > 6 ||
        abs(hipPlaneRight[EDOG_12DOF_TROT_FRAME_COUNT / 4].hipAngleCentiDeg + 1300) > 6) {
        fprintf(stderr,
                "zero-side/yaw gait must keep swing and stance feet on the inward hip plane: left span=%d..%d right span=%d..%d\n",
                minHipPlaneLeft, maxHipPlaneLeft, minHipPlaneRight, maxHipPlaneRight);
        return 1;
    }
    Edog12Dof_ResetRuntimeGeometry();

    Edog12Dof_GenerateTrotTable(gait, 0.01, 0.003, 0, 0);
    Edog12Dof_GenerateTrotTable(reverse, 0.01, 0.003, 0, 1);
    int wrapHipDelta = abs(gait[0].hipAngleCentiDeg -
                           gait[EDOG_12DOF_TROT_FRAME_COUNT - 1].hipAngleCentiDeg);
    int wrapFemurDelta = abs(gait[0].femurAngleCentiDeg -
                             gait[EDOG_12DOF_TROT_FRAME_COUNT - 1].femurAngleCentiDeg);
    int wrapTibiaDelta = abs(gait[0].tibiaAngleCentiDeg -
                             gait[EDOG_12DOF_TROT_FRAME_COUNT - 1].tibiaAngleCentiDeg);
    int wrapDelta = wrapHipDelta > wrapFemurDelta ? wrapHipDelta : wrapFemurDelta;
    if (wrapTibiaDelta > wrapDelta) {
        wrapDelta = wrapTibiaDelta;
    }
    if (gait[0].hipAngleCentiDeg == gait[EDOG_12DOF_TROT_FRAME_COUNT - 1].hipAngleCentiDeg &&
        gait[0].femurAngleCentiDeg == gait[EDOG_12DOF_TROT_FRAME_COUNT - 1].femurAngleCentiDeg &&
        gait[0].tibiaAngleCentiDeg == gait[EDOG_12DOF_TROT_FRAME_COUNT - 1].tibiaAngleCentiDeg) {
        fprintf(stderr, "continuous gait table must not duplicate first frame at the end\n");
        return 1;
    }
    if (wrapDelta > EDOG_12DOF_GAIT_MAX_JOINT_STEP_CENTI) {
        fprintf(stderr, "continuous gait wrap must stay smooth, got %d centi-deg\n", wrapDelta);
        return 1;
    }

    minFemur = maxFemur = gait[0].femurAngleDeg;
    minTibia = maxTibia = gait[0].tibiaAngleDeg;
    for (int i = 0; i < EDOG_12DOF_TROT_FRAME_COUNT; i++) {
        if (gait[i].femurAngleDeg < minFemur) {
            minFemur = gait[i].femurAngleDeg;
        }
        if (gait[i].femurAngleDeg > maxFemur) {
            maxFemur = gait[i].femurAngleDeg;
        }
        if (gait[i].tibiaAngleDeg < minTibia) {
            minTibia = gait[i].tibiaAngleDeg;
        }
        if (gait[i].tibiaAngleDeg > maxTibia) {
            maxTibia = gait[i].tibiaAngleDeg;
        }
        if (gait[i].hipAngleCentiDeg != gait[i].hipAngleDeg * EDOG_12DOF_CENTI_PER_DEG ||
            gait[i].femurAngleCentiDeg != gait[i].femurAngleDeg * EDOG_12DOF_CENTI_PER_DEG ||
            gait[i].tibiaAngleCentiDeg != gait[i].tibiaAngleDeg * EDOG_12DOF_CENTI_PER_DEG) {
            subDegreeFrames++;
        }
    }
    if (subDegreeFrames < EDOG_12DOF_TROT_FRAME_COUNT / 2) {
        fprintf(stderr, "gait IK should preserve centi-degree precision across frames, got %d sub-degree frames\n",
                subDegreeFrames);
        return 1;
    }

    if (maxFemur - minFemur < 2) {
        fprintf(stderr, "foot trajectory should create meaningful fore/aft femur travel, got span %d\n",
                maxFemur - minFemur);
        return 1;
    }
    if (standIk.femurAngleDeg < minFemur - 1 || standIk.femurAngleDeg > maxFemur + 1) {
        fprintf(stderr, "foot trajectory should stay centered around stand femur=%d, range=[%d,%d]\n",
                standIk.femurAngleDeg, minFemur, maxFemur);
        return 1;
    }
    if (maxTibia - minTibia < 1) {
        fprintf(stderr, "foot trajectory should create meaningful lift/stance tibia travel, got span %d\n",
                maxTibia - minTibia);
        return 1;
    }
    int reversedDiffCount = 0;
    for (int i = 0; i < EDOG_12DOF_TROT_FRAME_COUNT; i++) {
        if (reverse[i].femurAngleDeg != gait[i].femurAngleDeg ||
            reverse[i].tibiaAngleDeg != gait[i].tibiaAngleDeg) {
            reversedDiffCount++;
        }
    }
    if (reversedDiffCount < EDOG_12DOF_TROT_FRAME_COUNT / 2) {
        fprintf(stderr, "reversed gait should invert moving phase order\n");
        return 1;
    }

    Edog12Dof_GenerateTrotTable(legacy, 0.01, 0.003, 0, 0);
    Edog12Dof_GenerateDirectionalTrotTable(directional, 0.01, 0.0, 0.0, 0.003, 0, 1, 0);
    for (int i = 0; i < EDOG_12DOF_TROT_FRAME_COUNT; i++) {
        if (legacy[i].hipAngleDeg != directional[i].hipAngleDeg ||
            legacy[i].femurAngleDeg != directional[i].femurAngleDeg ||
            legacy[i].tibiaAngleDeg != directional[i].tibiaAngleDeg) {
            fprintf(stderr, "zero-side directional gait must match legacy forward gait at frame %d\n", i);
            return 1;
        }
    }

    Edog12Dof_GenerateDirectionalTrotTable(leftSide, 0.01, 0.02, 0.0, 0.003, 0, 1, 0);
    Edog12Dof_GenerateDirectionalTrotTable(rightSide, 0.01, 0.02, 0.0, 0.003, 1, 1, 0);
    maxLeftSideHip = leftSide[0].hipAngleDeg;
    minRightSideHip = rightSide[0].hipAngleDeg;
    for (int i = 1; i < EDOG_12DOF_TROT_FRAME_COUNT; i++) {
        if (leftSide[i].hipAngleDeg > maxLeftSideHip) {
            maxLeftSideHip = leftSide[i].hipAngleDeg;
        }
        if (rightSide[i].hipAngleDeg < minRightSideHip) {
            minRightSideHip = rightSide[i].hipAngleDeg;
        }
    }
    if (maxLeftSideHip <= EDOG_12DOF_FRONT_HIP_ADDUCTION_DEFAULT_DEG + 2) {
        fprintf(stderr, "left-side command should open the left front hip, frontBase=%.1f max=%d\n",
                EDOG_12DOF_FRONT_HIP_ADDUCTION_DEFAULT_DEG, maxLeftSideHip);
        return 1;
    }
    if (minRightSideHip >= EDOG_12DOF_FRONT_HIP_ADDUCTION_DEFAULT_DEG - 2) {
        fprintf(stderr, "left-side command should close the right front hip, frontBase=%.1f min=%d\n",
                EDOG_12DOF_FRONT_HIP_ADDUCTION_DEFAULT_DEG, minRightSideHip);
        return 1;
    }

    Edog12Dof_GenerateDirectionalTrotTable(yawFront, 0.01, 0.0, 0.018, 0.003, 0, 1, 0);
    Edog12Dof_GenerateDirectionalTrotTable(yawBack, 0.01, 0.0, 0.018, 0.003, 0, 0, 0);
    maxYawFrontHip = yawFront[0].hipAngleDeg;
    minYawBackHip = yawBack[0].hipAngleDeg;
    for (int i = 1; i < EDOG_12DOF_TROT_FRAME_COUNT; i++) {
        if (yawFront[i].hipAngleDeg > maxYawFrontHip) {
            maxYawFrontHip = yawFront[i].hipAngleDeg;
        }
        if (yawBack[i].hipAngleDeg < minYawBackHip) {
            minYawBackHip = yawBack[i].hipAngleDeg;
        }
    }
    if (maxYawFrontHip <= EDOG_12DOF_FRONT_HIP_ADDUCTION_DEFAULT_DEG + 1) {
        fprintf(stderr, "positive yaw should bias front-left hip outward, frontBase=%.1f max=%d\n",
                EDOG_12DOF_FRONT_HIP_ADDUCTION_DEFAULT_DEG, maxYawFrontHip);
        return 1;
    }
    if (minYawBackHip >= EDOG_12DOF_REAR_HIP_ADDUCTION_DEFAULT_DEG - 1) {
        fprintf(stderr, "positive yaw should bias back-left hip inward, rearBase=%.1f min=%d\n",
                EDOG_12DOF_REAR_HIP_ADDUCTION_DEFAULT_DEG, minYawBackHip);
        return 1;
    }

    return 0;
}
'''

    with tempfile.TemporaryDirectory() as tmpdir:
        stub_include = Path(tmpdir) / "stubs"
        test_c = Path(tmpdir) / "test_12dof_gait_ik.c"
        binary = Path(tmpdir) / "test_12dof_gait_ik"
        stub_include.mkdir()
        (stub_include / "iot_gpio.h").write_text(
            "#ifndef IOT_GPIO_H\n"
            "#define IOT_GPIO_H\n"
            "#define GPIO0_PC7 0\n"
            "#define GPIO0_PB5 0\n"
            "#define GPIO0_PB4 0\n"
            "#define GPIO1_PD0 0\n"
            "#endif\n",
            encoding="utf-8",
        )
        test_c.write_text(source, encoding="utf-8")
        cmd = [
            "gcc",
            "-std=c99",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-I",
            str(stub_include),
            "-I",
            str(ROOT),
            str(test_c),
            str(ROOT / "12_DOF_Version/src/gait_generate_12dof.c"),
            "-lm",
            "-o",
            str(binary),
        ]
        subprocess.run(cmd, check=True)
        subprocess.run([str(binary)], check=True)


if __name__ == "__main__":
    main()
