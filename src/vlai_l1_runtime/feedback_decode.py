"""Motor-reported range decoding and saved-capture replay, without hardware IO."""

from __future__ import annotations

import csv
import hashlib
import json
import struct
from dataclasses import asdict, dataclass
from pathlib import Path

from .configuration import MOTOR_NAMES, SystemConfig, _is_loader_validated, _read_local_regular_file
from .feedback import MotorFeedback, RawMotorFrame


@dataclass(frozen=True)
class DecodedFollowerFeedback:
    feedback: MotorFeedback
    status_nibbles: tuple[int, ...]
    mos_temperatures_raw: tuple[int, ...]
    rotor_temperatures_raw: tuple[int, ...]
    at_protocol_endpoint: tuple[bool, ...]


def decode_follower_feedback(
    config: SystemConfig, side: str, sdk_feedback: MotorFeedback
) -> DecodedFollowerFeedback:
    """Decode paired raw bytes using tracked evidence; infer no status/FT validity.

    The SDK values are not rescaled or modified. Only raw payloads are decoded.
    Host timestamps keep their original meanings; this is not live freshness
    validation and must not be used as a substitute for a telemetry session.
    """
    if not _is_loader_validated(config) or config.follower_feedback is None:
        raise ValueError("decoding requires validated config with Follower range evidence")
    ranges = config.follower_feedback.for_side(side)
    if sdk_feedback.raw_frames is None:
        raise ValueError("decoding requires paired raw frames")
    values, statuses, mos, rotor, endpoints = [], [], [], [], []
    for motor, limits, raw in zip(config.motors, ranges, sdk_feedback.raw_frames, strict=True):
        b = bytes.fromhex(raw.payload_hex)
        if (
            raw.name != motor.name
            or raw.response_id != motor.receive_id
            or not 1 <= motor.send_id <= 15
            or b[0] & 15 != motor.send_id
            or raw.can_fd != config.can.fd
        ):
            raise ValueError(f"raw identity/format mismatch for {side}/{motor.name}")
        integers = (b[1] << 8 | b[2], b[3] << 4 | b[4] >> 4, (b[4] & 15) << 8 | b[5])
        maxima = (limits.position_max_rad, limits.velocity_max_rad_s, limits.torque_max_nm)
        values.append(
            tuple(
                struct.unpack("<f", struct.pack("<f", n * (2.0 * maximum) / top - maximum))[0]
                for n, maximum, top in zip(integers, maxima, (65535, 4095, 4095), strict=True)
            )
        )
        statuses.append(b[0] >> 4)
        mos.append(b[6])
        rotor.append(b[7])
        endpoints.append(
            any(n in (0, top) for n, top in zip(integers, (65535, 4095, 4095), strict=True))
        )
    feedback = MotorFeedback(
        MOTOR_NAMES,
        sdk_feedback.monotonic_ns,
        *(tuple(row[index] for row in values) for index in range(3)),
        raw_frames=sdk_feedback.raw_frames,
    )
    return DecodedFollowerFeedback(
        feedback, tuple(statuses), tuple(mos), tuple(rotor), tuple(endpoints)
    )


def _snapshot(raw: dict) -> MotorFeedback:
    if set(raw) != set(MotorFeedback.__dataclass_fields__):
        raise ValueError("invalid saved feedback fields")
    return MotorFeedback(
        names=tuple(raw["names"]),
        monotonic_ns=raw["monotonic_ns"],
        position_rad=tuple(raw["position_rad"]),
        velocity_rad_s=tuple(raw["velocity_rad_s"]),
        torque_nm=tuple(raw["torque_nm"]),
        raw_frames=tuple(RawMotorFrame(**frame) for frame in raw["raw_frames"]),
    )


def redecode_capture(config: SystemConfig, source: Path, output: Path) -> Path:
    """Create a separate offline comparison; never overwrite source or an output."""
    if not _is_loader_validated(config) or config.follower_feedback is None:
        raise ValueError("replay requires validated config with Follower range evidence")
    source = source.absolute()
    # Same regular-local-file restriction as configuration: never open /dev or FIFOs.
    config_bytes = _read_local_regular_file(config.path)
    from .configuration import load_system_config

    if load_system_config(config.path)._validation_fingerprint != config._validation_fingerprint:
        raise ValueError("configuration changed after loading")
    source_bytes = {
        name: _read_local_regular_file(source / name)
        for name in ("report.json", "raw_feedback.jsonl")
    }
    report = json.loads(source_bytes["report.json"])
    config_digest = hashlib.sha256(config_bytes).hexdigest()
    if report.get("schema_version") not in (2, 3) or report.get("status") != "passed":
        raise ValueError("replay requires a passed schema-2/3 raw capture")
    if report.get("config_sha256") not in (
        config_digest,
        config.follower_feedback.source_config_sha256,
    ):
        raise ValueError("capture configuration does not match current or surveyed configuration")
    expected = {}
    for entry in report.get("endpoints", []):
        side = entry.get("side")
        endpoint = next(
            (e for e in config.can.endpoints if e.side == side and e.role == "follower"), None
        )
        if (
            endpoint is None
            or side in expected
            or entry.get("interface") != endpoint.interface
            or entry.get("parentdev") != endpoint.parentdev
            or type(entry.get("completed_rounds")) is not int
            or entry["completed_rounds"] <= 0
        ):
            raise ValueError("invalid capture endpoint identity/round count")
        expected[side] = entry
    if not expected:
        raise ValueError("capture has no endpoints")
    records, previous, counts = [], {}, dict.fromkeys(expected, 0)
    try:
        for line in source_bytes["raw_feedback.jsonl"].splitlines():
            record = json.loads(line)
            side = record["side"]
            if (
                side not in expected
                or record["role"] != "follower"
                or record["interface"] != expected[side]["interface"]
                or type(record["round"]) is not int
                or record["round"] != counts[side] + 1
            ):
                raise ValueError("invalid capture record identity/round sequence")
            sdk = _snapshot(record["feedback"])
            decoded = decode_follower_feedback(config, side, sdk)
            times = (sdk.monotonic_ns, *(r.host_receive_monotonic_ns for r in sdk.raw_frames))
            if side in previous and any(a <= b for a, b in zip(times, previous[side], strict=True)):
                raise ValueError("non-monotonic saved feedback timestamps")
            previous[side] = times
            counts[side] += 1
            records.append((side, record["round"], sdk, decoded))
    except (KeyError, TypeError) as exc:
        raise ValueError(f"malformed saved feedback: {exc}") from exc
    if any(counts[side] != entry["completed_rounds"] for side, entry in expected.items()):
        raise ValueError("saved raw feedback is incomplete")

    output.mkdir(parents=True, exist_ok=False)
    result = {
        "schema_version": 1,
        "status": "writing",
        "source": str(source.resolve()),
        "source_sha256": {
            name: hashlib.sha256(data).hexdigest() for name, data in source_bytes.items()
        },
        "source_config_sha256": report["config_sha256"],
        "decoding_config_sha256": config_digest,
        "range_evidence": asdict(config.follower_feedback),
        "decoder": "motor_reported_ranges_float32",
        "timestamp_kind": report.get("raw_timestamp_kind"),
        "ft_validated": False,
        "physical_torque_calibrated": False,
        "status_semantics_verified": False,
        "bound_violations": {
            side: entry.get("bound_violations", []) for side, entry in expected.items()
        },
        "rounds": counts,
    }
    result_path = output / "report.json"
    result_path.write_text(json.dumps(result, indent=2) + "\n")
    try:
        with (output / "decoded.csv").open("x", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(
                (
                    "side",
                    "round",
                    "motor",
                    "host_receive_monotonic_ns",
                    "raw_payload_hex",
                    "reported_position_rad",
                    "reported_velocity_rad_s",
                    "reported_torque_nm",
                    "sdk_position_rad",
                    "sdk_velocity_rad_s",
                    "sdk_torque_nm",
                    "raw_status",
                    "mos_temperature_raw",
                    "rotor_temperature_raw",
                    "at_protocol_endpoint",
                )
            )
            for side, round_index, sdk, decoded in records:
                for i, raw in enumerate(sdk.raw_frames):
                    writer.writerow(
                        (
                            side,
                            round_index,
                            raw.name,
                            raw.host_receive_monotonic_ns,
                            raw.payload_hex,
                            decoded.feedback.position_rad[i],
                            decoded.feedback.velocity_rad_s[i],
                            decoded.feedback.torque_nm[i],
                            sdk.position_rad[i],
                            sdk.velocity_rad_s[i],
                            sdk.torque_nm[i],
                            decoded.status_nibbles[i],
                            decoded.mos_temperatures_raw[i],
                            decoded.rotor_temperatures_raw[i],
                            decoded.at_protocol_endpoint[i],
                        )
                    )
        result["status"] = "passed"
    except BaseException:
        result["status"] = "failed"
        raise
    finally:
        result_path.write_text(json.dumps(result, indent=2) + "\n")
    return result_path
