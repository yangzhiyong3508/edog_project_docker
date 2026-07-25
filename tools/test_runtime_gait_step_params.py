from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read(path):
    return (ROOT / path).read_text(encoding="utf-8")


def require(text, token, label):
    if token not in text:
        raise AssertionError(f"{label} missing token: {token}")


def test_iot_command_struct_carries_runtime_gait_step_parameters():
    header = read("utils/include/iot_control.h")

    require(header, "double stepLengthM;", "iot_control.h")
    require(header, "double stepHeightM;", "iot_control.h")
    require(header, "double hipAdductionDeg;", "iot_control.h")
    require(header, "double thighLengthM;", "iot_control.h")
    require(header, "double calfLengthM;", "iot_control.h")


def test_mqtt_parser_accepts_step_length_and_height_meter_fields():
    source = read("utils/src/iot.c")

    require(source, '"step_length_m"', "iot.c")
    require(source, '"step_height_m"', "iot.c")
    require(source, "ParseGaitStepParameters", "iot.c")
    require(source, '"hip_adduction_deg"', "iot.c")
    require(source, '"thigh_length_m"', "iot.c")
    require(source, '"calf_length_m"', "iot.c")
    require(source, "ParseGaitGeometryParameters", "iot.c")
    require(source, "EnqueueTextCommand(commandText, stepLengthM, stepHeightM", "iot.c")
    require(source, "ClampGaitStepMinimum", "iot.c")
    if "EDOG_12DOF_MAX_STRIDE_MM / 1000.0" in source:
        raise AssertionError("iot.c must not cap runtime step length")
    if "0.012" in source:
        raise AssertionError("iot.c must not cap runtime step height at 12mm")


def test_device_runtime_keeps_step_parameters_unbounded_above_zero():
    config = read("include/edog_config.h")
    control = read("utils/src/iot_control.c")
    gait = read("12_DOF_Version/src/gait_generate_12dof.c")
    motion = read("12_DOF_Version/src/motion_utils_12dof.c")

    for token in [
        "EDOG_12DOF_MAX_STRIDE_MM",
        "EDOG_12DOF_MAX_LIFT_MM",
        "EDOG_12DOF_REALTIME_MAX_FORWARD_MM",
        "EDOG_12DOF_REALTIME_LEGACY_MAX_LIFT_MM",
        "EDOG_PUPPER_MAX_LIFT_MM",
    ]:
        if token in config:
            raise AssertionError(f"config must not define gait upper-limit token {token}")
    require(control, "NormalizeStepMinimumM", "iot_control.c")
    require(control, "IotControl_SetRuntimeGaitGeometry", "iot_control.c")
    require(gait, "Edog12Dof_SetRuntimeGeometry", "gait_generate_12dof.c")
    require(gait, "signedStepMmWithMinimum", "gait_generate_12dof.c")
    require(motion, "minimumLiftMm", "motion_utils_12dof.c")
    for text, label in [(control, "iot_control.c"), (gait, "gait_generate_12dof.c"), (motion, "motion_utils_12dof.c")]:
        for token in [
            "EDOG_12DOF_MAX_STRIDE_MM",
            "EDOG_12DOF_MAX_LIFT_MM",
            "EDOG_12DOF_REALTIME_MAX_FORWARD_MM",
            "EDOG_12DOF_REALTIME_LEGACY_MAX_LIFT_MM",
            "EDOG_PUPPER_MAX_LIFT_MM",
        ]:
            if token in text:
                raise AssertionError(f"{label} must not use gait upper-limit token {token}")


def test_motion_task_uses_runtime_steps_and_supports_in_place_trot():
    control = read("utils/src/iot_control.c")
    motion_header = read("12_DOF_Version/include/motion_utils_12dof.h")
    motion_source = read("12_DOF_Version/src/motion_utils_12dof.c")

    require(control, "MOTION_CMD_TROT_IN_PLACE", "iot_control.c")
    require(control, 'strcmp(content, "trot_in_place")', "iot_control.c")
    require(control, "GetMotionStepSnapshot(&stepLengthM, &stepHeightM)", "iot_control.c")
    require(control, "IotControl_HandleCommandStringWithSteps", "iot_control.c")
    require(control, "trot_in_place_cycle(stepHeightM)", "iot_control.c")
    require(motion_header, "int trot_in_place_cycle(double step_height);", "motion_utils_12dof.h")
    require(motion_source, "int trot_in_place_cycle(double step_height)", "motion_utils_12dof.c")
    require(motion_source, "0.0,", "motion_utils_12dof.c")


def test_config_defaults_match_low_step_debug_values():
    config = read("include/edog_config.h")

    require(config, "#define EDOG_12DOF_COMMAND_STEP_LENGTH_M       0.01", "edog_config.h")
    require(config, "#define EDOG_12DOF_COMMAND_STEP_HEIGHT_M       0.003", "edog_config.h")


def main():
    test_iot_command_struct_carries_runtime_gait_step_parameters()
    test_mqtt_parser_accepts_step_length_and_height_meter_fields()
    test_device_runtime_keeps_step_parameters_unbounded_above_zero()
    test_motion_task_uses_runtime_steps_and_supports_in_place_trot()
    test_config_defaults_match_low_step_debug_values()


if __name__ == "__main__":
    main()
