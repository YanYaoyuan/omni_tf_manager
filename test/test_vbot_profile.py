from pathlib import Path

import yaml


PROFILE_PATH = Path(__file__).resolve().parents[1] / "config" / "omni_vbot_dog.yaml"


def test_vbot_live_sensor_alias_contract():
    profile = yaml.safe_load(PROFILE_PATH.read_text(encoding="utf-8"))
    parameters = profile["omni_tf_manager"]["ros__parameters"]

    assert parameters["mode"] == "authority"
    assert parameters["profile_name"] == "omni_vbot_dog"
    assert parameters["calibration_id"] != "unverified"
    assert parameters["sensor_relays"] == ["lidar", "imu"]

    expected = {
        "lidar": {
            "type": "pointcloud2",
            "input_topic": "/lidar_points",
            "output_topic": "/omni/sensors/lidar/points",
            "output_frame": "omni_lidar_link",
        },
        "imu": {
            "type": "imu",
            "input_topic": "/lidar_imu",
            "output_topic": "/omni/sensors/imu/data",
            "output_frame": "omni_imu_link",
        },
    }
    for relay, contract in expected.items():
        prefix = f"sensor_relay.{relay}."
        assert parameters[prefix + "operation"] == "identity_frame_alias"
        assert parameters[prefix + "alias_verified"] is True
        assert parameters[prefix + "input_frame"] == "vita_lidar"
        for key, value in contract.items():
            assert parameters[prefix + key] == value
