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
        "1-2.2.1:1.0",
        "1-2.2.2:1.0",
        "1-2.2.3:1.0",
        "1-2.2.4:1.0",
    ]
    assert config.can.tx_queue_length == 1000
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
    assert config.control["follower"].fc[5] == 0.093
    assert config.teleoperation.source_revision == ("bf300508e179f652b23f0efaf3b6c9048f1f12e9")
    assert config.teleoperation.state_timeout_s == 0.1
    assert config.teleoperation.rt_priority == 20
    assert config.teleoperation.can_health_poll_s == 0.1
    assert config.teleoperation.startup_timeout_s == 45.0
    assert config.teleoperation.shutdown_timeout_s == 8.0
    assert config.teleoperation.blockers == ()
    assert config.schema_version == 3
    assert config.safety.command_ready is False
    assert config.command_blockers == (
        "command_transport_unimplemented",
        "production_source_unavailable",
        "j2_coordinate_unverified",
        "follower_right_bus_stability_unverified",
    )
    assert config.cameras.collection_ready is True
    assert config.cameras.startup_timeout_s == 2.0
    assert [(camera.role, camera.device_id) for camera in config.cameras.streams] == [
        ("wrist_left", "255323074436"),
        ("wrist_right", "255323074499"),
        ("agent", "251643060089"),
    ]
    assert config.operator_panel.port == 8765
    assert config.camera_preview.port == 8088
    assert config.camera_preview.fps == 10
    assert config.camera_preview.jpeg_quality == 80
    assert config.camera_preview.max_age_s == 0.5
    assert config.camera_preview.bridge_socket_path == Path("/run/vlai-l1/camera-bridge.sock")
    assert config.camera_preview.startup_timeout_s == 5.0
    assert config.camera_preview.shutdown_timeout_s == 5.0


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
        (
            'enabled = true\ndevice_id = "255323074436"',
            'enabled = false\ndevice_id = "255323074436"',
            "must be absent",
        ),
        (
            'enabled = true\ndevice_id = "255323074436"',
            "enabled = true",
            "device_id is required",
        ),
        (
            "kp = [240.0, 240.0, 240.0, 240.0, 24.0, 31.0, 25.0, 16.0]",
            "kp = [240.0]",
            "exactly 8",
        ),
        (
            'source_revision = "bf300508e179f652b23f0efaf3b6c9048f1f12e9"',
            'source_revision = "main"',
            "full Git commit",
        ),
        ("rt_priority = 20", "rt_priority = 100", "outside the allowed range"),
        ("tx_queue_length = 1000", "tx_queue_length = 0", "outside the allowed range"),
        ("startup_timeout_s = 2.0", "startup_timeout_s = 0.0", "must be positive"),
        ("port = 8088", "port = 8765", "must differ"),
        (
            'bridge_socket_path = "/run/vlai-l1/camera-bridge.sock"',
            'bridge_socket_path = "camera-bridge.sock"',
            "must be absolute",
        ),
        (
            "can_health_poll_s = 0.1",
            "can_health_poll_s = 0.2",
            "must not exceed state_timeout_s",
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
    content = SYSTEM_CONFIG.read_text().replace("1-2.2.1:1.0", "1-2.2.9:1.0", 1)
    candidate = tmp_path / "system.toml"
    candidate.write_text(content)

    config = load_system_config(candidate)
    assert config.can.endpoints[0].parentdev == "1-2.2.9:1.0"
