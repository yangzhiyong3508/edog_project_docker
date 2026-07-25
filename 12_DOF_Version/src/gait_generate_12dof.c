#include "../include/gait_generate_12dof.h"
#include "../../include/edog_config.h"

#include <math.h>
#include <stddef.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static double g_runtimeFrontHipAdductionDeg = EDOG_12DOF_FRONT_HIP_ADDUCTION_DEFAULT_DEG;
static double g_runtimeRearHipAdductionDeg = EDOG_12DOF_REAR_HIP_ADDUCTION_DEFAULT_DEG;
static double g_runtimeThighLengthMm = SPOTMICRO_THIGH_LENGTH_MM;
static double g_runtimeCalfLengthMm = SPOTMICRO_CALF_LENGTH_MM;
static double g_runtimeFrontFootZDeltaMm = 0.0;
static double g_runtimeRearFootZDeltaMm = 0.0;

static double clampDouble(double value, double minValue, double maxValue)
{
    if (value < minValue) {
        return minValue;
    }
    if (value > maxValue) {
        return maxValue;
    }
    return value;
}

static double bezierQuintic(double p0, double p1, double p2,
                            double p3, double p4, double p5, double t)
{
    double u;
    double u2;
    double u3;
    double u4;
    double u5;
    double t2;
    double t3;
    double t4;
    double t5;

    t = clampDouble(t, 0.0, 1.0);
    u = 1.0 - t;
    u2 = u * u;
    u3 = u2 * u;
    u4 = u3 * u;
    u5 = u4 * u;
    t2 = t * t;
    t3 = t2 * t;
    t4 = t3 * t;
    t5 = t4 * t;
    return u5 * p0 +
        5.0 * u4 * t * p1 +
        10.0 * u3 * t2 * p2 +
        10.0 * u2 * t3 * p3 +
        5.0 * u * t4 * p4 +
        t5 * p5;
}

static double bezierSwingLift(double t)
{
    return bezierQuintic(0.0, 0.0, 1.6, 1.6, 0.0, 0.0, t);
}

static double pyAppleCycloidProgress(double t)
{
    double sigma;

    t = clampDouble(t, 0.0, 1.0);
    sigma = 2.0 * M_PI * t;
    return (sigma - sin(sigma)) / (2.0 * M_PI);
}

static double pyAppleCycloidLift(double t)
{
    double sigma;

    t = clampDouble(t, 0.0, 1.0);
    sigma = 2.0 * M_PI * t;
    return (1.0 - cos(sigma)) / 2.0;
}

static Edog12DofFootPoint quinticBezierSwingFoot(const Edog12DofFootPoint *start,
                                                 const Edog12DofFootPoint *end,
                                                 double liftMm,
                                                 int legIndex,
                                                 double t)
{
    Edog12DofFootPoint foot = *start;
    double startYOffset;
    double endYOffset;
    double controlZ2;
    double controlZ3;

    t = clampDouble(t, 0.0, 1.0);
    startYOffset = start->yMm - Edog12Dof_FootYOnHipPlaneForLeg(legIndex, start->zMm);
    endYOffset = end->yMm - Edog12Dof_FootYOnHipPlaneForLeg(legIndex, end->zMm);
    controlZ2 = start->zMm - 1.6 * liftMm;
    controlZ3 = end->zMm - 1.6 * liftMm;

    foot.xMm = bezierQuintic(start->xMm, start->xMm, start->xMm,
                             end->xMm, end->xMm, end->xMm, t);
    foot.zMm = bezierQuintic(start->zMm, start->zMm, controlZ2,
                             controlZ3, end->zMm, end->zMm, t);
    foot.yMm = Edog12Dof_FootYOnHipPlaneForLeg(legIndex, foot.zMm) +
        bezierQuintic(startYOffset, startYOffset, startYOffset,
                      endYOffset, endYOffset, endYOffset, t);

    return foot;
}

static Edog12DofFootPoint pyAppleCycloidSwingFoot(const Edog12DofFootPoint *start,
                                                  const Edog12DofFootPoint *end,
                                                  double liftMm,
                                                  int legIndex,
                                                  double t)
{
    Edog12DofFootPoint foot = *start;
    double progress;
    double liftEnvelope;
    double startYOffset;
    double endYOffset;
    double lateralOffset;

    t = clampDouble(t, 0.0, 1.0);
    progress = pyAppleCycloidProgress(t);
    liftEnvelope = pyAppleCycloidLift(t);
    startYOffset = start->yMm - Edog12Dof_FootYOnHipPlaneForLeg(legIndex, start->zMm);
    endYOffset = end->yMm - Edog12Dof_FootYOnHipPlaneForLeg(legIndex, end->zMm);
    lateralOffset = startYOffset + (endYOffset - startYOffset) * progress;

    foot.xMm = start->xMm + (end->xMm - start->xMm) * progress;
    foot.zMm = start->zMm + (end->zMm - start->zMm) * progress -
        liftMm * liftEnvelope;
    foot.yMm = Edog12Dof_FootYOnHipPlaneForLeg(legIndex, foot.zMm) +
        lateralOffset;

    return foot;
}

#if EDOG_12DOF_TROT_TRAJECTORY_CYCLOID
#define EDOG_12DOF_TROT_DEFAULT_TRAJECTORY_IS_CYCLOID 1
#else
#define EDOG_12DOF_TROT_DEFAULT_TRAJECTORY_IS_CYCLOID 0
#endif

static Edog12DofFootPoint defaultSwingFoot(const Edog12DofFootPoint *start,
                                           const Edog12DofFootPoint *end,
                                           double liftMm,
                                           int legIndex,
                                           double t)
{
    if (EDOG_12DOF_TROT_DEFAULT_TRAJECTORY_IS_CYCLOID) {
        return pyAppleCycloidSwingFoot(start, end, liftMm, legIndex, t);
    }
    return quinticBezierSwingFoot(start, end, liftMm, legIndex, t);
}

static double signedStepMmWithMinimum(double stepM, double minAbsMm)
{
    double absMm = fabs(stepM) * 1000.0;
    double sign = stepM < 0.0 ? -1.0 : 1.0;

    if (absMm < 0.001) {
        return 0.0;
    }
    return sign * (absMm < minAbsMm ? minAbsMm : absMm);
}

static double clampSignedStepMm(double stepM, double minAbsMm, double maxAbsMm)
{
    double absMm = fabs(stepM) * 1000.0;
    double sign = stepM < 0.0 ? -1.0 : 1.0;

    if (absMm < 0.001) {
        return 0.0;
    }
    return sign * clampDouble(absMm, minAbsMm, maxAbsMm);
}

static double minimumLiftMm(double liftMm, double minMm)
{
    return liftMm < minMm ? minMm : liftMm;
}

static int clampServoAngleInt(int angle)
{
    if (angle < -90) {
        return -90;
    }
    if (angle > 90) {
        return 90;
    }
    return angle;
}

static int clampServoAngleCenti(int centiAngle)
{
    if (centiAngle < -9000) {
        return -9000;
    }
    if (centiAngle > 9000) {
        return 9000;
    }
    return centiAngle;
}

static int degToJointDelta(double jointDeg)
{
    int deltaDeg = (int)(jointDeg + (jointDeg >= 0.0 ? 0.5 : -0.5));
    return clampServoAngleInt(deltaDeg);
}

/* 厘度版本：保留 IK 原始亚度精度，供运动输出层使用。 */
static int degToJointDeltaCenti(double jointDeg)
{
    double centi = jointDeg * 100.0;
    int deltaCenti = (int)(centi + (centi >= 0.0 ? 0.5 : -0.5));
    return clampServoAngleCenti(deltaCenti);
}

static int clampJointStepCenti(int previous, int target)
{
    int delta = target - previous;

    if (delta > EDOG_12DOF_GAIT_MAX_JOINT_STEP_CENTI) {
        return previous + EDOG_12DOF_GAIT_MAX_JOINT_STEP_CENTI;
    }
    if (delta < -EDOG_12DOF_GAIT_MAX_JOINT_STEP_CENTI) {
        return previous - EDOG_12DOF_GAIT_MAX_JOINT_STEP_CENTI;
    }
    return target;
}

static void syncJointDegFromCenti(Edog12DofJointAngles *angles)
{
    if (angles == NULL) {
        return;
    }
    angles->hipAngleDeg = degToJointDelta((double)angles->hipAngleCentiDeg /
                                          (double)EDOG_12DOF_CENTI_PER_DEG);
    angles->femurAngleDeg = degToJointDelta((double)angles->femurAngleCentiDeg /
                                            (double)EDOG_12DOF_CENTI_PER_DEG);
    angles->tibiaAngleDeg = degToJointDelta((double)angles->tibiaAngleCentiDeg /
                                            (double)EDOG_12DOF_CENTI_PER_DEG);
}

static void limitGaitTableJointStepCenti(Edog12DofJointAngles gaitTable[EDOG_12DOF_TROT_FRAME_COUNT])
{
    if (gaitTable == NULL) {
        return;
    }

    for (int pass = 0; pass < 8; pass++) {
        for (int i = 1; i < EDOG_12DOF_TROT_FRAME_COUNT; i++) {
            gaitTable[i].hipAngleCentiDeg = clampJointStepCenti(
                gaitTable[i - 1].hipAngleCentiDeg, gaitTable[i].hipAngleCentiDeg);
            gaitTable[i].femurAngleCentiDeg = clampJointStepCenti(
                gaitTable[i - 1].femurAngleCentiDeg, gaitTable[i].femurAngleCentiDeg);
            gaitTable[i].tibiaAngleCentiDeg = clampJointStepCenti(
                gaitTable[i - 1].tibiaAngleCentiDeg, gaitTable[i].tibiaAngleCentiDeg);
            syncJointDegFromCenti(&gaitTable[i]);
        }

        gaitTable[0].hipAngleCentiDeg = clampJointStepCenti(
            gaitTable[EDOG_12DOF_TROT_FRAME_COUNT - 1].hipAngleCentiDeg,
            gaitTable[0].hipAngleCentiDeg);
        gaitTable[0].femurAngleCentiDeg = clampJointStepCenti(
            gaitTable[EDOG_12DOF_TROT_FRAME_COUNT - 1].femurAngleCentiDeg,
            gaitTable[0].femurAngleCentiDeg);
        gaitTable[0].tibiaAngleCentiDeg = clampJointStepCenti(
            gaitTable[EDOG_12DOF_TROT_FRAME_COUNT - 1].tibiaAngleCentiDeg,
            gaitTable[0].tibiaAngleCentiDeg);
        syncJointDegFromCenti(&gaitTable[0]);

        for (int i = EDOG_12DOF_TROT_FRAME_COUNT - 2; i > 0; i--) {
            gaitTable[i].hipAngleCentiDeg = clampJointStepCenti(
                gaitTable[i + 1].hipAngleCentiDeg, gaitTable[i].hipAngleCentiDeg);
            gaitTable[i].femurAngleCentiDeg = clampJointStepCenti(
                gaitTable[i + 1].femurAngleCentiDeg, gaitTable[i].femurAngleCentiDeg);
            gaitTable[i].tibiaAngleCentiDeg = clampJointStepCenti(
                gaitTable[i + 1].tibiaAngleCentiDeg, gaitTable[i].tibiaAngleCentiDeg);
            syncJointDegFromCenti(&gaitTable[i]);
        }
    }
}

void Edog12Dof_ResetRuntimeGeometry(void)
{
    g_runtimeFrontHipAdductionDeg = EDOG_12DOF_FRONT_HIP_ADDUCTION_DEFAULT_DEG;
    g_runtimeRearHipAdductionDeg = EDOG_12DOF_REAR_HIP_ADDUCTION_DEFAULT_DEG;
    g_runtimeThighLengthMm = SPOTMICRO_THIGH_LENGTH_MM;
    g_runtimeCalfLengthMm = SPOTMICRO_CALF_LENGTH_MM;
    g_runtimeFrontFootZDeltaMm = 0.0;
    g_runtimeRearFootZDeltaMm = 0.0;
}

void Edog12Dof_SetRuntimeGeometry(double hipAdductionDeg,
                                  double thighLengthMm,
                                  double calfLengthMm)
{
    Edog12Dof_SetRuntimeGeometryForFrontRear(hipAdductionDeg, hipAdductionDeg,
                                             thighLengthMm, calfLengthMm);
}

void Edog12Dof_SetRuntimeGeometryForFrontRear(double frontHipAdductionDeg,
                                              double rearHipAdductionDeg,
                                              double thighLengthMm,
                                              double calfLengthMm)
{
    g_runtimeFrontHipAdductionDeg = clampDouble(frontHipAdductionDeg, -45.0, 45.0);
    g_runtimeRearHipAdductionDeg = clampDouble(rearHipAdductionDeg, -45.0, 45.0);
    g_runtimeThighLengthMm = clampDouble(thighLengthMm,
                                         EDOG_12DOF_LINK_LENGTH_MIN_M * 1000.0,
                                         EDOG_12DOF_LINK_LENGTH_MAX_M * 1000.0);
    g_runtimeCalfLengthMm = clampDouble(calfLengthMm,
                                        EDOG_12DOF_LINK_LENGTH_MIN_M * 1000.0,
                                        EDOG_12DOF_LINK_LENGTH_MAX_M * 1000.0);
}

void Edog12Dof_GetRuntimeGeometry(double *hipAdductionDeg,
                                  double *thighLengthMm,
                                  double *calfLengthMm)
{
    if (hipAdductionDeg != NULL) {
        *hipAdductionDeg = g_runtimeFrontHipAdductionDeg;
    }
    if (thighLengthMm != NULL) {
        *thighLengthMm = g_runtimeThighLengthMm;
    }
    if (calfLengthMm != NULL) {
        *calfLengthMm = g_runtimeCalfLengthMm;
    }
}

void Edog12Dof_GetRuntimeGeometryForFrontRear(double *frontHipAdductionDeg,
                                              double *rearHipAdductionDeg,
                                              double *thighLengthMm,
                                              double *calfLengthMm)
{
    if (frontHipAdductionDeg != NULL) {
        *frontHipAdductionDeg = g_runtimeFrontHipAdductionDeg;
    }
    if (rearHipAdductionDeg != NULL) {
        *rearHipAdductionDeg = g_runtimeRearHipAdductionDeg;
    }
    if (thighLengthMm != NULL) {
        *thighLengthMm = g_runtimeThighLengthMm;
    }
    if (calfLengthMm != NULL) {
        *calfLengthMm = g_runtimeCalfLengthMm;
    }
}

void Edog12Dof_SetRuntimeFootZDeltas(double frontFootZDeltaMm,
                                     double rearFootZDeltaMm)
{
    g_runtimeFrontFootZDeltaMm = clampDouble(frontFootZDeltaMm,
                                             EDOG_12DOF_BODY_HEIGHT_DELTA_MIN_MM,
                                             EDOG_12DOF_BODY_HEIGHT_DELTA_MAX_MM);
    g_runtimeRearFootZDeltaMm = clampDouble(rearFootZDeltaMm,
                                            EDOG_12DOF_BODY_HEIGHT_DELTA_MIN_MM,
                                            EDOG_12DOF_BODY_HEIGHT_DELTA_MAX_MM);
}

void Edog12Dof_GetRuntimeFootZDeltas(double *frontFootZDeltaMm,
                                     double *rearFootZDeltaMm)
{
    if (frontFootZDeltaMm != NULL) {
        *frontFootZDeltaMm = g_runtimeFrontFootZDeltaMm;
    }
    if (rearFootZDeltaMm != NULL) {
        *rearFootZDeltaMm = g_runtimeRearFootZDeltaMm;
    }
}

void Edog12Dof_SetLinkLengthsM(double thighLengthM, double calfLengthM)
{
    Edog12Dof_SetRuntimeGeometryForFrontRear(g_runtimeFrontHipAdductionDeg,
                                             g_runtimeRearHipAdductionDeg,
                                             thighLengthM * 1000.0,
                                             calfLengthM * 1000.0);
}

double Edog12Dof_GetThighLengthMm(void)
{
    return g_runtimeThighLengthMm;
}

double Edog12Dof_GetCalfLengthMm(void)
{
    return g_runtimeCalfLengthMm;
}

double Edog12Dof_DefaultFootXForLeg(int isFrontLeg)
{
    return isFrontLeg ? EDOG_12DOF_DEFAULT_FOOT_X_MM :
        EDOG_12DOF_DEFAULT_FOOT_X_MM - EDOG_12DOF_REAR_FOOT_BACK_OFFSET_MM;
}

double Edog12Dof_DefaultFootZForLeg(int isFrontLeg)
{
    double baseZMm = isFrontLeg ? EDOG_12DOF_DEFAULT_FOOT_Z_MM :
        EDOG_12DOF_DEFAULT_FOOT_Z_MM - EDOG_12DOF_REAR_FOOT_UP_OFFSET_MM;
    if (isFrontLeg) {
        return baseZMm + g_runtimeFrontFootZDeltaMm;
    }
    return baseZMm + g_runtimeRearFootZDeltaMm;
}

double Edog12Dof_FootYOnHipPlane(int isRightLeg, double zMm)
{
    double leftFootY = tan(g_runtimeFrontHipAdductionDeg * M_PI / 180.0) * zMm;
    return isRightLeg ? -leftFootY : leftFootY;
}

double Edog12Dof_HipAdductionDegForLeg(int legIndex)
{
    return (legIndex == 0 || legIndex == 1) ?
        g_runtimeFrontHipAdductionDeg : g_runtimeRearHipAdductionDeg;
}

double Edog12Dof_FootYOnHipPlaneForLeg(int legIndex, double zMm)
{
    int isRightLeg = (legIndex == 1 || legIndex == 3);
    double leftFootY = tan(Edog12Dof_HipAdductionDegForLeg(legIndex) *
                           M_PI / 180.0) * zMm;
    return isRightLeg ? -leftFootY : leftFootY;
}

double Edog12Dof_DefaultFootYForLeg(int legIndex)
{
    return Edog12Dof_FootYOnHipPlaneForLeg(
        legIndex, Edog12Dof_DefaultFootZForLeg(legIndex == 0 || legIndex == 1));
}

double Edog12Dof_ReferencePhaseForFrame(int frameIndex, int reversed)
{
    double phase;

    if (frameIndex < 0) {
        frameIndex = 0;
    }
    if (frameIndex >= EDOG_12DOF_GAIT_FRAME_COUNT) {
        frameIndex = EDOG_12DOF_GAIT_FRAME_COUNT - 1;
    }

    phase = (double)frameIndex / (double)EDOG_12DOF_GAIT_FRAME_COUNT;
    if (reversed && frameIndex > 0) {
        phase = 1.0 - phase;
    }
    return phase;
}

double Edog12Dof_ReferenceSwingEnvelopeForPhase(double phase)
{
    double swingPortion = EDOG_12DOF_TROT_SWING_PORTION;

    if (phase < 0.0 || phase > swingPortion) {
        return 0.0;
    }
    if (EDOG_12DOF_TROT_DEFAULT_TRAJECTORY_IS_CYCLOID) {
        return pyAppleCycloidLift(phase / swingPortion);
    }
    return bezierSwingLift(phase / swingPortion);
}

Edog12DofFootPoint Edog12Dof_SampleReferenceTrotFootPoint(
    double forwardStepM, double sideStepM, double yawStepM, double stepHeightM,
    int isRightLeg, int isFrontLeg, int legIndex, int reversed,
    int frameIndex, int *isSwingOut)
{
    double phase = Edog12Dof_ReferencePhaseForFrame(frameIndex, reversed);
    double yawStep = clampSignedStepMm(yawStepM, 0.0, EDOG_12DOF_MAX_YAW_MM);
    double clampedYawStepM = yawStep / 1000.0;
    double effectiveForwardM = forwardStepM + (isRightLeg ? -clampedYawStepM : clampedYawStepM);
    double strideMm = signedStepMmWithMinimum(effectiveForwardM,
                                            EDOG_12DOF_MIN_STRIDE_MM);
    double hardwareStrideMm = EDOG_12DOF_HARDWARE_FORWARD_SIGN * strideMm;
    double sideMm = clampSignedStepMm(sideStepM, 0.0, EDOG_12DOF_MAX_SIDE_MM);
    double liftMm = minimumLiftMm(fabs(stepHeightM) * 1000.0,
                                EDOG_12DOF_MIN_LIFT_MM);
    double effectiveLiftMm = liftMm;
    double legSideSign = isRightLeg ? -1.0 : 1.0;
    double yawSideSign = (isFrontLeg ? 1.0 : -1.0) * legSideSign;
    double lateralTargetMm = legSideSign * sideMm + yawSideSign * yawStep;
    double baseZMm = Edog12Dof_DefaultFootZForLeg(isFrontLeg);
    double swingPortion = EDOG_12DOF_TROT_SWING_PORTION;
    double stancePortion = 1.0 - swingPortion;
    double bodyShiftMm = 0.0;
    Edog12DofFootPoint foot = {
        Edog12Dof_DefaultFootXForLeg(isFrontLeg),
        Edog12Dof_FootYOnHipPlaneForLeg(legIndex, baseZMm),
        baseZMm
    };

    if (fabs(forwardStepM) > 0.001) {
        double bodyShiftSign = forwardStepM < 0.0 ? -1.0 : 1.0;
        bodyShiftMm = bodyShiftSign * EDOG_12DOF_TROT_BODY_X_SHIFT_MM;
    }
    foot.xMm -= bodyShiftMm;

    if (phase < swingPortion) {
        double swingT = phase / swingPortion;
        Edog12DofFootPoint swingStart = foot;
        Edog12DofFootPoint swingEnd = foot;

        if (isSwingOut != NULL) {
            *isSwingOut = 1;
        }
        if (!isFrontLeg) {
            effectiveLiftMm += EDOG_12DOF_REAR_SWING_LIFT_EXTRA_MM;
        }
        swingStart.xMm += -hardwareStrideMm / 2.0;
        swingStart.yMm = Edog12Dof_FootYOnHipPlaneForLeg(legIndex, swingStart.zMm) -
            lateralTargetMm / 2.0;
        swingEnd.xMm += hardwareStrideMm / 2.0;
        swingEnd.yMm = Edog12Dof_FootYOnHipPlaneForLeg(legIndex, swingEnd.zMm) +
            lateralTargetMm / 2.0;
        foot = defaultSwingFoot(&swingStart, &swingEnd, effectiveLiftMm, legIndex, swingT);
    } else {
        double stanceT = (phase - swingPortion) / stancePortion;
        double xOffset = hardwareStrideMm / 2.0 - hardwareStrideMm * stanceT;
        double yOffset = lateralTargetMm / 2.0 - lateralTargetMm * stanceT;

        if (isSwingOut != NULL) {
            *isSwingOut = 0;
        }
        foot.xMm += xOffset;
        foot.zMm = baseZMm;
        foot.yMm = Edog12Dof_FootYOnHipPlaneForLeg(legIndex, foot.zMm) + yOffset;
    }
    return foot;
}

int Edog12Dof_IK(const Edog12DofFootPoint *foot, int isRightLeg,
                 Edog12DofJointAngles *outAngles)
{
    double femur = Edog12Dof_GetThighLengthMm();
    double tibia = Edog12Dof_GetCalfLengthMm();
    double x;
    double y;
    double z;
    double hipRad;
    double sagittal;
    double dist;
    double cosKnee;
    double kneeRad;
    double femurRad;
    double tibiaRad;

    if (foot == NULL || outAngles == NULL) {
        return -1;
    }

    x = foot->xMm;
    y = isRightLeg ? -foot->yMm : foot->yMm;
    z = foot->zMm;

    if (!isfinite(x) || !isfinite(y) || !isfinite(z)) {
        x = EDOG_12DOF_DEFAULT_FOOT_X_MM;
        y = isRightLeg ? -Edog12Dof_DefaultFootYForLeg(1) : Edog12Dof_DefaultFootYForLeg(0);
        z = EDOG_12DOF_DEFAULT_FOOT_Z_MM;
    }

    /* Hip angle is positive when the leg opens outward in local leg coordinates. */
    hipRad = atan2(y, z);

    sagittal = sqrt(y * y + z * z);
    dist = sqrt(x * x + sagittal * sagittal);
    if (dist < 0.001) {
        dist = 0.001;
    }
    dist = clampDouble(dist, fabs(femur - tibia) + 1.0, femur + tibia - 1.0);

    cosKnee = (femur * femur + tibia * tibia - dist * dist) / (2.0 * femur * tibia);
    cosKnee = clampDouble(cosKnee, -1.0, 1.0);
    kneeRad = M_PI - acos(cosKnee);

    femurRad = atan2(x, sagittal) +
               acos(clampDouble((femur * femur + dist * dist - tibia * tibia) /
                                (2.0 * femur * dist), -1.0, 1.0));
    tibiaRad = -kneeRad;

    {
        double hipDeg = hipRad * 180.0 / M_PI;
        double femurDeg = femurRad * 180.0 / M_PI - 45.0;
        double tibiaDeg = tibiaRad * 180.0 / M_PI + 70.0;

        outAngles->hipAngleDeg = degToJointDelta(hipDeg);
        outAngles->femurAngleDeg = degToJointDelta(femurDeg);
        outAngles->tibiaAngleDeg = degToJointDelta(tibiaDeg);
        /* 同步厘度字段，保留未取整的亚度精度（消除支撑相阶梯抖动）。 */
        outAngles->hipAngleCentiDeg = degToJointDeltaCenti(hipDeg);
        outAngles->femurAngleCentiDeg = degToJointDeltaCenti(femurDeg);
        outAngles->tibiaAngleCentiDeg = degToJointDeltaCenti(tibiaDeg);
    }
    return 0;
}

void Edog12Dof_GenerateTrotTable(Edog12DofJointAngles gaitTable[EDOG_12DOF_TROT_FRAME_COUNT],
                                 double stepLengthM, double stepHeightM,
                                 int isRightLeg, int reversed)
{
    Edog12Dof_GenerateDirectionalTrotTable(gaitTable, stepLengthM, 0.0, 0.0,
                                           stepHeightM, isRightLeg, 1, reversed);
}

void Edog12Dof_GenerateDirectionalTrotTable(
    Edog12DofJointAngles gaitTable[EDOG_12DOF_TROT_FRAME_COUNT],
    double forwardStepM, double sideStepM, double yawStepM, double stepHeightM,
    int isRightLeg, int isFrontLeg, int reversed)
{
    const int total = EDOG_12DOF_TROT_FRAME_COUNT;
    int legIndex = isFrontLeg ? (isRightLeg ? 1 : 0) : (isRightLeg ? 3 : 2);

    if (gaitTable == NULL) {
        return;
    }
    if (reversed) {
        Edog12DofJointAngles forwardTable[EDOG_12DOF_TROT_FRAME_COUNT];

        Edog12Dof_GenerateDirectionalTrotTable(forwardTable, forwardStepM, sideStepM,
                                               yawStepM, stepHeightM, isRightLeg,
                                               isFrontLeg, 0);
        for (int k = 0; k < total; k++) {
            int sourceIndex = (total - k) % total;
            gaitTable[k] = forwardTable[sourceIndex];
        }
        return;
    }

    for (int k = 0; k < total; k++) {
        double phase = Edog12Dof_ReferencePhaseForFrame(k, reversed);
        double baseZMm = Edog12Dof_DefaultFootZForLeg(isFrontLeg);
        int isSwing = 0;
        Edog12DofFootPoint foot = Edog12Dof_SampleReferenceTrotFootPoint(
            forwardStepM, sideStepM, yawStepM, stepHeightM,
            isRightLeg, isFrontLeg, legIndex, reversed, k, &isSwing);

        if (Edog12Dof_IK(&foot, isRightLeg, &gaitTable[k]) != 0) {
            gaitTable[k].hipAngleDeg = degToJointDelta(Edog12Dof_HipAdductionDegForLeg(legIndex));
            gaitTable[k].femurAngleDeg = EDOG_12DOF_STAND_THIGH_DELTA_DEG;
            gaitTable[k].tibiaAngleDeg = EDOG_12DOF_STAND_CALF_DELTA_DEG;
            Edog12Dof_SyncCentiFromDeg(&gaitTable[k]);
        }

        /* 后腿伸直后 IK 对抬腿灵敏度低，在关节空间对摆动相增量做补偿，
         * 使后腿实际抬腿幅度与前腿一致。补偿倍数由 IK 几何计算得出。 */
        if (!isFrontLeg && isSwing) {
            Edog12DofFootPoint stanceFoot;
            Edog12DofJointAngles stanceAngles;
            double rearSwingBoostEnvelope = Edog12Dof_ReferenceSwingEnvelopeForPhase(phase);
            int femurBoostNum = EDOG_12DOF_REAR_FEMUR_BOOST_DEN +
                (EDOG_12DOF_REAR_FEMUR_BOOST_NUM - EDOG_12DOF_REAR_FEMUR_BOOST_DEN) *
                rearSwingBoostEnvelope;
            int tibiaBoostNum = EDOG_12DOF_REAR_TIBIA_BOOST_DEN +
                (EDOG_12DOF_REAR_TIBIA_BOOST_NUM - EDOG_12DOF_REAR_TIBIA_BOOST_DEN) *
                rearSwingBoostEnvelope;
            stanceFoot.xMm = Edog12Dof_DefaultFootXForLeg(isFrontLeg);
            stanceFoot.zMm = baseZMm;
            stanceFoot.yMm = Edog12Dof_FootYOnHipPlaneForLeg(legIndex, stanceFoot.zMm);
            if (Edog12Dof_IK(&stanceFoot, isRightLeg, &stanceAngles) == 0) {
                int deltaFemurCenti = gaitTable[k].femurAngleCentiDeg - stanceAngles.femurAngleCentiDeg;
                int deltaTibiaCenti = gaitTable[k].tibiaAngleCentiDeg - stanceAngles.tibiaAngleCentiDeg;
                gaitTable[k].femurAngleCentiDeg = stanceAngles.femurAngleCentiDeg +
                    deltaFemurCenti * femurBoostNum / EDOG_12DOF_REAR_FEMUR_BOOST_DEN;
                gaitTable[k].tibiaAngleCentiDeg = stanceAngles.tibiaAngleCentiDeg +
                    deltaTibiaCenti * tibiaBoostNum / EDOG_12DOF_REAR_TIBIA_BOOST_DEN;
                gaitTable[k].femurAngleDeg = degToJointDelta(
                    (double)gaitTable[k].femurAngleCentiDeg / 100.0);
                gaitTable[k].tibiaAngleDeg = degToJointDelta(
                    (double)gaitTable[k].tibiaAngleCentiDeg / 100.0);
            }
        }
    }

    limitGaitTableJointStepCenti(gaitTable);
}
