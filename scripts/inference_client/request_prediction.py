"""One bounded prediction request over a loopback SSH tunnel; never controls hardware."""

import argparse
import json
import time
from pathlib import Path

import msgpack_numpy
import numpy as np
from websockets.sync.client import connect


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8808)
    parser.add_argument(
        "--timeout",
        type=float,
        default=360,
        help="Bounded response wait, including first JAX compilation",
    )
    parser.add_argument(
        "--request", type=Path, help="Native observation encoded with msgpack_numpy"
    )
    parser.add_argument(
        "--response", type=Path, help="Create-only prediction output, never a command"
    )
    parser.add_argument("--transport-probe", action="store_true")
    args = parser.parse_args()
    if args.transport_probe:
        if args.request or args.response:
            parser.error("Transport probe cannot take observation/output files")
        request = {"probe": np.arange(256, dtype=np.float32).reshape(32, 8)}
    else:
        if not args.request or not args.response:
            parser.error("Both --request and --response are required")
        if args.response.exists():
            parser.error("Response file already exists")
        request = msgpack_numpy.unpackb(args.request.read_bytes())
    started = time.monotonic()
    with connect(
        f"ws://127.0.0.1:{args.port}",
        compression=None,
        max_size=64 * 1024 * 1024,
        open_timeout=10,
        close_timeout=5,
        # The pinned server runs synchronous inference in its event loop.
        # First JAX compilation can delay ping replies; recv stays bounded.
        ping_interval=None,
    ) as ws:
        metadata = msgpack_numpy.unpackb(ws.recv(timeout=10))
        if args.transport_probe:
            if metadata.get("transport_only") is not True:
                raise ValueError("Probe requires an explicitly marked echo server")
        elif (
            metadata.get("prediction_only") is not True
            or metadata.get("native_joint_degrees") is not True
            or metadata.get("kind") not in ("pi0", "unified")
            or metadata.get("transport_only", False)
        ):
            raise ValueError("Not the expected native prediction-only server")
        ws.send(msgpack_numpy.packb(request))
        raw = ws.recv(timeout=args.timeout)
        if isinstance(raw, str):
            raise RuntimeError(f"Server rejected prediction request: {raw}")
        response = msgpack_numpy.unpackb(raw)
        actions = np.asarray(response["actions"])
        if actions.shape != (32, 8) or not np.isfinite(actions).all():
            raise ValueError("Expected finite 32x8 joint predictions")
        if args.transport_probe:
            if actions.dtype != np.float32 or not np.array_equal(actions, request["probe"]):
                raise ValueError("Transport array did not roundtrip exactly")
        else:
            with args.response.open("xb") as output:
                output.write(raw)
    print(
        json.dumps(
            {
                "passed": True,
                "transport_only": args.transport_probe,
                "model_inference": not args.transport_probe,
                "robot_connected": False,
                "shape": list(actions.shape),
                "roundtrip_ms": (time.monotonic() - started) * 1000,
            }
        )
    )


if __name__ == "__main__":
    main()
