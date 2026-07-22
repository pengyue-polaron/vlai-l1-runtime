from __future__ import annotations

from pathlib import Path

import pytest

from vlai_l1_runtime.configuration import MOTOR_NAMES, ConfigError, load_system_config

ROOT = Path(__file__).resolve().parents[1]
SYSTEM_CONFIG = ROOT / "configs/system/vlai_l1.toml"


def test_system_config_maps_the_complete_tracked_contract() -> None:
    config = load_system_config(SYSTEM_CONFIG)

    assert config.robot_id == "vlai_l1"
    assert config.position_unit == "degree"
    assert [endpoint.interface for endpoint in config.can.endpoints] == [
        "can0",
        "can1",
        "can2",
        "can3",
    ]
    assert [endpoint.parentdev for endpoint in config.can.endpoints] == [
        "1-1.4.1:1.0",
        "1-1.4.2:1.0",
        "1-1.4.3:1.0",
        "1-1.4.4:1.0",
    ]
    assert tuple(motor.name for motor in config.motors) == MOTOR_NAMES
    assert tuple(motor.send_id for motor in config.motors) == tuple(range(1, 9))
    assert tuple(motor.receive_id for motor in config.motors) == tuple(range(0x11, 0x19))
    assert config.control["follower"].kp == (
        240.0,
        240.0,
        240.0,
        240.0,
        24.0,
        31.0,
        25.0,
        16.0,
    )
    assert config.joint_limits["left"]["joint_2"].maximum_deg == 9.0
    assert config.joint_limits["right"]["joint_2"].minimum_deg == -9.0
    assert config.safety.command_ready is False
    assert config.command_blockers == (
        "command_transport_unimplemented",
        "production_source_unavailable",
        "j2_coordinate_unverified",
        "follower_right_bus_stability_unverified",
        "joint_limits_unverified",
    )
    assert config.cameras.collection_ready is False
    assert [(camera.role, camera.device_id) for camera in config.cameras.streams] == [
        ("agent", None),
        ("wrist", None),
    ]


@pytest.mark.parametrize(
    ("old", "new", "match"),
    [
        (
            'position_unit = "degree"',
            'position_unit = "degree"\nunknown = true',
            "unknown keys",
        ),
        ('interface = "can1"', 'interface = "can0"', "interfaces must be unique"),
        ("command_ready = false", "command_ready = true", "readiness gates"),
        ("maximum_deg = 9.0", "maximum_deg = -91.0", "minimum must be less"),
        (
            "enabled = false\nwidth = 640",
            'enabled = false\ndevice_id = "/dev/video0"\nwidth = 640',
            "must be absent",
        ),
        ("enabled = false", "enabled = true", "device_id is required"),
        (
            "kp = [240.0, 240.0, 240.0, 240.0, 24.0, 31.0, 25.0, 16.0]",
            "kp = [240.0]",
            "exactly 8",
        ),
    ],
)
def test_system_config_rejects_unsafe_or_ambiguous_edits(
    tmp_path: Path, old: str, new: str, match: str
) -> None:
    content = SYSTEM_CONFIG.read_text()
    assert old in content
    candidate = tmp_path / "system.toml"
    candidate.write_text(content.replace(old, new, 1))

    with pytest.raises(ConfigError, match=match):
        load_system_config(candidate)


def test_system_config_rejects_special_files_before_opening(tmp_path: Path) -> None:
    with pytest.raises(ConfigError, match="not a regular file"):
        load_system_config(Path("/dev/null"))

    with pytest.raises(ConfigError, match="trusted local filesystem"):
        load_system_config(Path("/proc/version"))

    symlink = tmp_path / "system-link.toml"
    symlink.symlink_to(SYSTEM_CONFIG)
    with pytest.raises(ConfigError, match="symbolic link"):
        load_system_config(symlink)

    config_directory = tmp_path / "config-directory"
    config_directory.mkdir()
    (config_directory / "system.toml").write_text(SYSTEM_CONFIG.read_text())
    directory_symlink = tmp_path / "config-link"
    directory_symlink.symlink_to(config_directory, target_is_directory=True)
    with pytest.raises(ConfigError, match="symbolic link"):
        load_system_config(directory_symlink / "system.toml")


def test_tracked_config_is_the_physical_identity_authority(tmp_path: Path) -> None:
    content = SYSTEM_CONFIG.read_text().replace("1-1.4.1:1.0", "1-1.4.9:1.0", 1)
    candidate = tmp_path / "system.toml"
    candidate.write_text(content)

    config = load_system_config(candidate)
    assert config.can.endpoints[0].parentdev == "1-1.4.9:1.0"
