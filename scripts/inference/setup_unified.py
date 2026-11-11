"""Start/reuse the Unified prediction service and tunnel, then warm it without motion."""

import shlex
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
REMOTE = "/data/research/pengyue/vlai-distance-inference-20261108"
SSH = ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=8", "work-pc"]


def main():
    state = subprocess.run(
        [*SSH, "systemctl --user is-active vlai-unified-live.service"],
        capture_output=True,
        text=True,
    )
    if state.returncode != 0:
        if state.returncode not in (3, 4):
            raise RuntimeError(f"cannot inspect Unified service: {state.stderr.strip()}")
        subprocess.run(
            [*SSH, "systemctl --user reset-failed vlai-unified-live.service"], capture_output=True
        )
        launch = [
            "systemd-run",
            "--user",
            "--unit=vlai-unified-live",
            "--property=MemoryMax=30G",
            "--property=MemorySwapMax=0",
            f"--working-directory={REMOTE}",
            "/bin/bash",
            "-lc",
            'source bin/env.sh; exec "$VLAI_PYTHON" bin/serve_native.py --kind unified '
            '--checkpoint "$VLAI_INFERENCE_ROOT/checkpoints/unified-29999" --port 8809',
        ]
        subprocess.run([*SSH, shlex.join(launch)], check=True)
    if subprocess.run(
        ["tmux", "has-session", "-t", "vlai-unified-tunnel"], capture_output=True
    ).returncode:
        tunnel = [
            *SSH[:-1],
            "-NT",
            "-o",
            "ExitOnForwardFailure=yes",
            "-o",
            "ServerAliveInterval=15",
            "-L",
            "8809:127.0.0.1:8809",
            SSH[-1],
        ]
        subprocess.run(
            [
                "tmux",
                "new-session",
                "-d",
                "-s",
                "vlai-unified-tunnel",
                "exec " + shlex.join(tunnel),
            ],
            check=True,
        )
    subprocess.run([sys.executable, str(ROOT / "scripts/inference/warmup_unified.py")], check=True)


if __name__ == "__main__":
    main()
