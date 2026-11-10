"""Prediction-only server for the pinned native VLAI joint-space checkpoints.

No device or CAN code. The client must supply the native observation contract;
Unified additionally needs the real causal histories prepared by its adapter.
"""

import argparse
import dataclasses
import hashlib
import json
import logging
import time
from pathlib import Path


def check_bundle(root, kind):
    root = Path(root).resolve()
    for name in (
        "_CHECKPOINT_METADATA",
        "params/_METADATA",
        "assets/vlai_native_joint_v1/norm_stats.json",
        "deployment.json",
    ):
        if not (root / name).is_file():
            raise ValueError(f"Missing completed checkpoint artifact: {root / name}")
    meta = json.loads((root / "deployment.json").read_text())
    completed = json.loads((root / "_CHECKPOINT_METADATA").read_text())
    if not completed.get("commit_timestamp_nsecs"):
        raise ValueError("Checkpoint has not finished committing")
    if meta.get("kind") != kind or meta.get("revision") != "vlai_native_joint_v1":
        raise ValueError("Checkpoint kind/revision does not match this server")
    expected = "d2997d32704496ff28a65008d53ffb82be992a3bd497c3fd519799cc8e7b49f3"
    if meta.get("source_manifest_sha256") != expected:
        raise ValueError("Checkpoint training source is not the prepared source-v2")
    return root


def configuration(kind):
    from forcevla_vlai.inputs import NativeJointOutputs, NativePi0Inputs, NativeUnifiedInputs
    from forcevla_vlai.models import NativePi0Config, NativeUnifiedConfig
    from openpi import transforms
    from openpi.training import config

    @dataclasses.dataclass(frozen=True)
    class ServingData(config.DataConfigFactory):
        def create(self, assets_dirs, model_config):
            adapter = NativePi0Inputs() if kind == "pi0" else NativeUnifiedInputs()
            return config.DataConfig(
                repo_id="vlai_native_joint_v1",
                asset_id="vlai_native_joint_v1",
                data_transforms=transforms.Group(inputs=[adapter], outputs=[NativeJointOutputs()]),
                model_transforms=config.ModelTransformFactory()(model_config),
            )

    cls = NativePi0Config if kind == "pi0" else NativeUnifiedConfig
    return config.TrainConfig(
        name="unified_vlai_" + kind,
        exp_name="serve",
        data=ServingData(),
        model=cls(
            paligemma_variant="gemma_2b_lora",
            action_expert_variant="gemma_300m_lora",
            action_horizon=32,
        ),
        policy_metadata={
            "native_joint_degrees": True,
            "kind": kind,
            "fps": 30,
            "prediction_horizon": 32,
            "deployment_closed_loop_validated": False,
            "prediction_only": True,
            "history_delivery": "full_causal_prefix"
            if kind == "unified"
            else "current_observation",
        },
    )


def configure_history_delivery(policy, kind):
    """Pass native full prefixes into the frozen model, without cross-request cache.

    The pinned generic Policy defaults to a one-row online memory update. Native
    Unified supplies the same padded full causal prefix used in training instead.
    FAVLA.interaction_residual encodes that prefix using its unchanged learned
    memory when interaction_state is absent. Only the wrapper's online cache is
    bypassed; the model's memory, weights and config remain enabled and unchanged.
    """
    if kind == "unified":
        if not hasattr(policy, "_interaction_update"):
            raise ValueError("Pinned policy history interface changed")
        policy._interaction_update = None
        policy._interaction_hidden = None
    return policy


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--kind", choices=["pi0", "unified"], required=True)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--port", type=int, default=8808)
    parser.add_argument("--check-only", action="store_true")
    args = parser.parse_args()
    root = check_bundle(args.checkpoint, args.kind)
    if args.check_only:
        print(json.dumps({"bundle_present": True, "weights_loaded": False, "kind": args.kind}))
        return
    from openpi.policies.policy_config import create_trained_policy
    from openpi.serving.websocket_policy_server import WebsocketPolicyServer

    logging.basicConfig(level=logging.INFO, force=True)
    started = time.monotonic()
    policy = create_trained_policy(configuration(args.kind), root)
    configure_history_delivery(policy, args.kind)
    logging.info(
        "Loaded %s checkpoint %s in %.2f seconds", args.kind, root, time.monotonic() - started
    )
    metadata = dict(policy.metadata)
    metadata["checkpoint"] = {
        "kind": args.kind,
        "path": str(root),
        "deployment_sha256": hashlib.sha256((root / "deployment.json").read_bytes()).hexdigest(),
    }
    WebsocketPolicyServer(
        policy, host="127.0.0.1", port=args.port, metadata=metadata
    ).serve_forever()


if __name__ == "__main__":
    main()
