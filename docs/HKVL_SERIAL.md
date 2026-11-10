# HKVL serial capture

The parser reads continuous 28-byte frames: `53 54`, six little-endian float32
values ordered Fx/Fy/Fz/Mx/My/Mz, then CRC16/MODBUS (low byte first).
Current defaults are 1,000,000 baud and 8N1; verify device settings if CRC decoding
fails. Use the dedicated USB-RS485 adapter, preferably its stable
`/dev/serial/by-id/` path, and close other serial clients before capture.

```bash
PYTHONPATH=src .venv/bin/python -m vlai_l1_runtime.hkvl_serial \
  --port /dev/ttyUSB0 --baudrate 1000000 \
  --output-prefix data/temporary/hkvl_trial --duration 10
```

The command prints decoded readings and writes a CSV of valid frames plus a BIN
of received bytes. Choose a new output prefix; existing files are not overwritten.
Without a duration, Ctrl+C stops capture and closes the port.

Units are N and Nm in the sensor frame. Timestamps are host receive times; several
frames may share one timestamp. No zeroing, filtering, tool-weight compensation
or coordinate transform is applied. These observations are not a motor controller.

Managed collection owns this serial port exclusively, so stop standalone capture
before collection. The dataset saves one matched FT sample per robot/video frame,
rather than the entire high-rate byte stream. See [Datasets](DATASETS.md).
