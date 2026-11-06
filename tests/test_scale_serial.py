"""Hardware-free checks for the observed RS232 scale stream."""

import os
import threading
from pathlib import Path

from vlai_l1_runtime.collection.scale import ScaleCollectionSource, ScaleLineDecoder
from vlai_l1_runtime.configuration import ScaleConfig


def test_scale_lines_decode_units_status_and_split_reads() -> None:
    decoder = ScaleLineDecoder()
    assert decoder.feed(b"\x00ST,NT,+     12") == []
    assert decoder.feed(b" g\r\nUS,GS,- 0.012 kg\r\nST,TR,+ 0 g\n\n") == [
        (12.0, True, False, True),
        (-12.0, False, False, False),
        (0.0, True, True, False),
    ]
    assert decoder.feed(b"OL,NT,+ 1 g\r\nST,NT,+ 15 lb\r\n") == []
    assert decoder.feed(b"ST,NT,+ 13 g\r\n") == [(13.0, True, False, True)]


def test_scale_reader_buffers_real_lines_from_a_pipe() -> None:
    source = ScaleCollectionSource(ScaleConfig(Path("/dev/vlai-test"), 9600, 8, "none", 1, 1.0))
    read_fd, write_fd = os.pipe()
    source._fd = read_fd
    thread = threading.Thread(target=source._read_loop)
    try:
        thread.start()
        os.write(write_fd, b"ST,TR,+     12 g\r\nUS,TR,+     13 g\r\n")
        source.wait_first(timeout_s=1, max_age_s=0.6)
        newest = source.latest_before(target_monotonic_ns=10**20)
        assert newest.sequence == 2
        assert newest.weight_g == 13.0
        assert newest.stable is False
        assert newest.tared is True
        assert newest.net is False
    finally:
        source._stop.set()
        thread.join(timeout=1)
        os.close(read_fd)
        os.close(write_fd)
