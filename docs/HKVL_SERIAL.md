# HKVL-75B USB-RS485 read-only capture

The operator's toolbox screenshot shows **Serial**, **1,000,000 baud**, and a
device-initiated continuous stream that needs no data command. The displayed
frame format is `53 54` + six little-endian IEEE-754 float32 values in
`Fx, Fy, Fz, Mx, My, Mz` order + CRC16/MODBUS low byte first (28 bytes total).
The screenshot does not show data bits, parity or stop bits; the current script
uses **8N1 as an explicit assumption**. Check the device's serial settings if
no valid CRC frames are seen.

From the repository directory, close the browser's serial connection first
and identify the dedicated USB-RS485 adapter:

```bash
ls -l /dev/serial/by-id/
```

If that directory does not exist, check the current kernel-assigned serial
ports instead:

```bash
ls -l /dev/ttyUSB* /dev/ttyACM*
```

An existing `/dev/ttyUSB0` or `/dev/ttyACM0` can be used directly. If none
exist, check that the USB-RS485 adapter is plugged into this host and recognized
before attempting capture. Run a short capture, replacing the example port with
the verified adapter path (prefer a stable `/dev/serial/by-id/...` path when
available):

```bash
PYTHONPATH=src .venv/bin/python -m vlai_l1_runtime.hkvl_serial \
  --port /dev/ttyUSB0 --baudrate 1000000 \
  --output-prefix data/temporary/hkvl_75b_trial --duration 10
```

The command reads only the selected serial port and prints decoded six-axis
values at 10 Hz. It creates `hkvl_75b_trial.csv` with every valid frame,
host receive timestamps and raw frame hex, plus `hkvl_75b_trial.bin` with all
received bytes including malformed frames. A different prefix is required for
each run; existing data are never overwritten. Without `--duration`, Ctrl+C
stops the capture and closes the port.

The column names use the manufacturer's force and moment units (N and Nm).
Host receive timestamps are assigned when a serial read returns; several frames
can share a timestamp and the file contains no sensor timestamp. The script
does not apply zeroing, tool weight compensation, coordinate transforms or
filtering. Values are the sensor-frame stream and are not yet a validated
end-effector external wrench. Keep the adapter separate from the robot motor
CAN endpoints, and do not treat this observer as a control or safety stop.

The HKVL-75 series product page describes the 75B force range as 200 N per axis
and moment range as 8 Nm per axis, but the actual delivered interface and
calibration remain device-specific:
https://www.hkvtech.cn/archives/137/
