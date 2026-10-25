"""Hardware-free tests for the HKVL streaming-frame decoder."""

import math
import os
import struct
import threading
from pathlib import Path

from vlai_l1_runtime.collection.ft import HKVLCollectionSource
from vlai_l1_runtime.configuration import FTSensorConfig
from vlai_l1_runtime.hkvl_serial import FrameDecoder, crc16_modbus


def frame(values: tuple[float, ...]) -> bytes:
    payload = b"\x53\x54" + struct.pack("<6f", *values)
    return payload + crc16_modbus(payload).to_bytes(2, "little")


def test_crc_reference_and_fragmented_frames():
    assert crc16_modbus(b"123456789") == 0x4B37
    first = frame((1.0, -2.0, 3.5, -0.25, 0.0, 7.0))
    second = frame((-1.0, 2.0, -3.5, 0.25, 0.0, -7.0))
    decoder = FrameDecoder()
    assert decoder.feed(b"noise\x53") == []
    assert decoder.feed(first[1:17]) == []
    samples = decoder.feed(first[17:] + second)
    assert [item.frame for item in samples] == [first, second]
    assert samples[0].values == (1.0, -2.0, 3.5, -0.25, 0.0, 7.0)
    assert decoder.discarded_bytes == 5


def test_corrupt_and_nonfinite_frames_do_not_become_force_samples():
    valid = frame((1.0, 2.0, 3.0, 4.0, 5.0, 6.0))
    corrupt = bytearray(valid)
    corrupt[10] ^= 0x80
    nan_frame = frame((math.nan, 0.0, 0.0, 0.0, 0.0, 0.0))
    decoder = FrameDecoder()
    assert [sample.frame for sample in decoder.feed(bytes(corrupt) + nan_frame + valid)] == [valid]
    assert decoder.crc_failures == 1
    assert decoder.nonfinite_frames == 1


def test_collection_ft_reader_decodes_a_pipe_without_opening_devices():
    source = HKVLCollectionSource(
        FTSensorConfig(Path("/dev/serial/by-id/test"), 1000000, 8, "none", 1, 1.0)
    )
    read_fd, write_fd = os.pipe()
    source._fd = read_fd
    thread = threading.Thread(target=source._read_loop)
    try:
        thread.start()
        first_frame = frame((1.0, -2.0, 3.0, 0.1, -0.2, 0.3))
        os.write(write_fd, first_frame + first_frame)
        source.wait_first(timeout_s=1, max_age_s=0.25)
        sample = source.closest(target_monotonic_ns=0, timeout_s=1)
        assert sample.sequence == 1
        assert sample.values[0:3] == (1.0, -2.0, 3.0)
        assert sample.monotonic_ns > 0
    finally:
        source._stop.set()
        thread.join(timeout=1)
        os.close(read_fd)
        os.close(write_fd)
