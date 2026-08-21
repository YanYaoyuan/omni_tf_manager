#!/usr/bin/env python3
"""Validate the Matrix XGW profile against simulator config and MuJoCo model."""

import argparse
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import yaml


TOLERANCE = 1.0e-8


def _vector(parameters, key):
    value = parameters.get(key)
    if not isinstance(value, list) or len(value) != 3:
        raise ValueError(f"{key} must contain three values")
    return [float(item) for item in value]


def _assert_close(label, actual, expected):
    if len(actual) != len(expected) or any(
        abs(lhs - rhs) > TOLERANCE for lhs, rhs in zip(actual, expected)
    ):
        raise ValueError(f"{label}: actual={actual}, expected={expected}")


def validate(matrix_root: Path, profile_path: Path, fast_lio_config: Path) -> None:
    config_path = matrix_root / "config" / "config.json"
    model_path = (
        matrix_root
        / "src"
        / "robot_mujoco"
        / "zsibot_robots"
        / "xgw"
        / "xgw.xml"
    )
    config = json.loads(config_path.read_text(encoding="utf-8"))
    robot = config["robot"]
    if robot["robot_type"] != "xgw":
        raise ValueError(f"active Matrix robot is {robot['robot_type']!r}, expected 'xgw'")
    sensors = robot["sensors"]

    model = ET.parse(model_path)
    livox_site = model.find(".//site[@name='livox_imu']")
    camera_site = model.find(".//site[@name='camera_imu']")
    if livox_site is None or camera_site is None:
        raise ValueError("xgw.xml is missing livox_imu or camera_imu site")
    model_livox = [float(item) for item in livox_site.attrib["pos"].split()]
    model_camera = [float(item) for item in camera_site.attrib["pos"].split()]

    lidar = sensors["lidar"]
    config_livox = [
        float(lidar["position"][axis]) / 100.0 for axis in ("x", "y", "z")
    ]
    _assert_close("config LiDAR vs xgw.xml livox_imu", config_livox, model_livox)
    _assert_close(
        "config LiDAR rotation",
        [float(lidar["rotation"][axis]) for axis in ("roll", "pitch", "yaw")],
        [0.0, 0.0, 0.0],
    )

    profile = yaml.safe_load(profile_path.read_text(encoding="utf-8"))
    parameters = profile["omni_tf_manager"]["ros__parameters"]
    _assert_close(
        "profile base->IMU translation",
        _vector(parameters, "static_transform.imu_mount.translation"),
        model_livox,
    )
    _assert_close(
        "profile base->IMU RPY",
        _vector(parameters, "static_transform.imu_mount.rotation_rpy"),
        [0.0, 0.0, 0.0],
    )
    _assert_close(
        "profile IMU->LiDAR translation",
        _vector(parameters, "static_transform.lidar_mount.translation"),
        [0.0, 0.0, 0.0],
    )
    _assert_close(
        "profile IMU->LiDAR RPY",
        _vector(parameters, "static_transform.lidar_mount.rotation_rpy"),
        [0.0, 0.0, 0.0],
    )

    expected_camera_rpy = [0.0, math.radians(15.0), 0.0]
    for sensor_name, transform_id in (
        ("camera", "rgb_camera_link"),
        ("depth_sensor", "depth_camera_link"),
    ):
        sensor = sensors[sensor_name]
        config_camera = [
            float(sensor["position"][axis]) / 100.0 for axis in ("x", "y", "z")
        ]
        _assert_close(f"{sensor_name} config vs camera_imu", config_camera, model_camera)
        _assert_close(
            f"profile {transform_id} translation",
            _vector(parameters, f"static_transform.{transform_id}.translation"),
            model_camera,
        )
        _assert_close(
            f"profile {transform_id} RPY",
            _vector(parameters, f"static_transform.{transform_id}.rotation_rpy"),
            expected_camera_rpy,
        )

    fast_lio = yaml.safe_load(fast_lio_config.read_text(encoding="utf-8"))
    fast_parameters = fast_lio["/**"]["ros__parameters"]
    _assert_close(
        "FAST-LIO Matrix T_imu_lidar",
        [float(item) for item in fast_parameters["mapping"]["extrinsic_T"]],
        [0.0, 0.0, 0.0],
    )
    _assert_close(
        "FAST-LIO Matrix R_imu_lidar",
        [float(item) for item in fast_parameters["mapping"]["extrinsic_R"]],
        [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--matrix-root", default="/data/robot/sim/matrix", type=Path)
    parser.add_argument(
        "--profile",
        default=Path(__file__).resolve().parents[1] / "config" / "matrix_xgw.yaml",
        type=Path,
    )
    parser.add_argument(
        "--fast-lio-config",
        default=(
            Path(__file__).resolve().parents[2]
            / "omni_slam"
            / "FAST_LIO"
            / "config"
            / "matrix_sim.yaml"
        ),
        type=Path,
    )
    arguments = parser.parse_args()
    try:
        validate(arguments.matrix_root, arguments.profile, arguments.fast_lio_config)
    except (KeyError, OSError, ValueError, ET.ParseError, yaml.YAMLError) as error:
        print(f"Matrix XGW profile validation failed: {error}", file=sys.stderr)
        return 1
    print("Matrix XGW 6DoF profile matches config.json, xgw.xml and FAST-LIO")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
