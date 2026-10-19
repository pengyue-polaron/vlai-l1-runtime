"""Hardware-free tests for the HKVL streaming-frame decoder."""

import math
import struct

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
