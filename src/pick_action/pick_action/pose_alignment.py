"""Pose correction and projection helpers for Odin/sensor alignment."""

from __future__ import annotations

import math
from dataclasses import dataclass


YAW_OFFSET_RAD = -1.6144295581
M_SENSOR_X = -0.3739821743
M_SENSOR_Y = 0.0124127122
N_SENSOR_X = 0.0443469277
N_SENSOR_Y = -0.3087514918


@dataclass(frozen=True)
class ProjectionAlignment:
    gripper_x_m: float
    gripper_y_m: float
    gripper_yaw_rad: float
    target_x_m: float
    target_y_m: float
    projection_x_m: float
    projection_y_m: float
    along_offset_m: float
    lateral_error_m: float


def yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def odin_to_field(
    odin_x_m: float,
    odin_y_m: float,
    field_origin_x_m: float,
    field_origin_y_m: float,
) -> tuple[float, float]:
    return (
        float(odin_x_m) - float(field_origin_x_m),
        float(odin_y_m) - float(field_origin_y_m),
    )


def correct_pose(
    sensor_3_mm: float,
    sensor_5_mm: float,
    odin_yaw_rad: float,
) -> tuple[float, float, float]:
    m = float(sensor_3_mm) / 1000.0
    n = float(sensor_5_mm) / 1000.0
    theta = float(odin_yaw_rad) + YAW_OFFSET_RAD

    corrected_x_m = (
        m * math.cos(theta)
        - M_SENSOR_X * math.cos(theta)
        + M_SENSOR_Y * math.sin(theta)
    )
    corrected_y_m = (
        n * math.cos(theta)
        - N_SENSOR_X * math.sin(theta)
        - N_SENSOR_Y * math.cos(theta)
    )
    return corrected_x_m, corrected_y_m, float(odin_yaw_rad)


def robot_to_gripper_pose(
    robot_x_m: float,
    robot_y_m: float,
    robot_yaw_rad: float,
    gripper_forward_m: float,
    gripper_left_m: float,
    gripper_yaw_offset_rad: float,
) -> tuple[float, float, float]:
    yaw = float(robot_yaw_rad)
    gripper_x_m = (
        float(robot_x_m)
        + float(gripper_forward_m) * math.cos(yaw)
        - float(gripper_left_m) * math.sin(yaw)
    )
    gripper_y_m = (
        float(robot_y_m)
        + float(gripper_forward_m) * math.sin(yaw)
        + float(gripper_left_m) * math.cos(yaw)
    )
    gripper_yaw_rad = yaw + float(gripper_yaw_offset_rad)
    return gripper_x_m, gripper_y_m, gripper_yaw_rad


def correct_pose_from_odin(
    sensor_3_mm: float,
    sensor_5_mm: float,
    odin_x_m: float,
    odin_y_m: float,
    odin_yaw_rad: float,
    field_origin_x_m: float,
    field_origin_y_m: float,
    gripper_forward_m: float,
    gripper_left_m: float,
    gripper_yaw_offset_rad: float,
    target_x_m: float,
    target_y_m: float,
    direct: float,
) -> dict[str, float]:
    input_field_x_m, input_field_y_m = odin_to_field(
        odin_x_m,
        odin_y_m,
        field_origin_x_m,
        field_origin_y_m,
    )
    corrected_robot_x_m, corrected_robot_y_m, corrected_robot_yaw_rad = correct_pose(
        sensor_3_mm,
        sensor_5_mm,
        odin_yaw_rad,
    )
    gripper_x_m, gripper_y_m, gripper_yaw_rad = robot_to_gripper_pose(
        corrected_robot_x_m,
        corrected_robot_y_m,
        corrected_robot_yaw_rad,
        gripper_forward_m,
        gripper_left_m,
        gripper_yaw_offset_rad,
    )
    projection = project_target_to_gripper_line(
        gripper_x_m,
        gripper_y_m,
        gripper_yaw_rad,
        target_x_m,
        target_y_m,
    )
    directed_move_m = float(direct) * projection.along_offset_m
    return {
        'input_field_x_m': input_field_x_m,
        'input_field_y_m': input_field_y_m,
        'input_field_yaw_rad': float(odin_yaw_rad),
        'corrected_robot_x_m': corrected_robot_x_m,
        'corrected_robot_y_m': corrected_robot_y_m,
        'corrected_robot_yaw_rad': corrected_robot_yaw_rad,
        'corrected_gripper_x_m': gripper_x_m,
        'corrected_gripper_y_m': gripper_y_m,
        'corrected_gripper_yaw_rad': gripper_yaw_rad,
        'target_x_m': float(target_x_m),
        'target_y_m': float(target_y_m),
        'target_projection_x_m': projection.projection_x_m,
        'target_projection_y_m': projection.projection_y_m,
        'raw_gripper_forward_move_m': projection.along_offset_m,
        'gripper_forward_move_m': directed_move_m,
        'direct': float(direct),
        'gripper_lateral_error_m': projection.lateral_error_m,
        'robot_delta_x_m': corrected_robot_x_m - input_field_x_m,
        'robot_delta_y_m': corrected_robot_y_m - input_field_y_m,
    }


def project_target_to_gripper_line(
    gripper_x_m: float,
    gripper_y_m: float,
    gripper_yaw_rad: float,
    target_x_m: float,
    target_y_m: float,
) -> ProjectionAlignment:
    ux = math.cos(float(gripper_yaw_rad))
    uy = math.sin(float(gripper_yaw_rad))
    dx = float(target_x_m) - float(gripper_x_m)
    dy = float(target_y_m) - float(gripper_y_m)

    along = dx * ux + dy * uy
    projection_x = float(gripper_x_m) + along * ux
    projection_y = float(gripper_y_m) + along * uy

    # Positive lateral error means the target is left of the gripper yaw line.
    lateral = dx * (-uy) + dy * ux
    return ProjectionAlignment(
        gripper_x_m=float(gripper_x_m),
        gripper_y_m=float(gripper_y_m),
        gripper_yaw_rad=float(gripper_yaw_rad),
        target_x_m=float(target_x_m),
        target_y_m=float(target_y_m),
        projection_x_m=projection_x,
        projection_y_m=projection_y,
        along_offset_m=along,
        lateral_error_m=lateral,
    )
