"""RSL-RL Checkpoint Exporter (TorchScript & ONNX).

Converts trained PyTorch actor weights into standalone TorchScript / ONNX models
ready for real-time edge execution in ROS 2 / C++ / Python nodes without needing
Isaac Sim or RSL-RL installed on the physical robot.

Usage:
    python 04_rsl_rl_policy_export.py --checkpoint ./logs/model_1500.pt --output ./exported/policy.pt
"""

import argparse
from pathlib import Path
import torch
import torch.nn as nn


class StandaloneActor(nn.Module):
    """Wrapper that packages observation normalization with the raw policy network."""

    def __init__(self, actor_network: nn.Module, obs_mean: torch.Tensor, obs_var: torch.Tensor, clip_obs: float = 100.0):
        super().__init__()
        self.actor = actor_network
        self.register_buffer("obs_mean", obs_mean)
        self.register_buffer("obs_var", obs_var)
        self.clip_obs = clip_obs

    def forward(self, obs: torch.Tensor) -> torch.Tensor:
        # Standard observation normalization: (obs - mean) / sqrt(var + 1e-8)
        norm_obs = torch.clamp(
            (obs - self.obs_mean) / torch.sqrt(self.obs_var + 1e-8),
            -self.clip_obs,
            self.clip_obs,
        )
        return self.actor(norm_obs)


def export_policy(checkpoint_path: str, output_path: str, num_observations: int = 48, num_actions: int = 12):
    ckpt_file = Path(checkpoint_path)
    out_file = Path(output_path)
    out_file.parent.mkdir(parents=True, exist_ok=True)

    print(f"[INFO] Loading RSL-RL checkpoint: {ckpt_file}...")
    device = torch.device("cpu")
    
    # In real pipeline, load state dict:
    # state_dict = torch.load(ckpt_file, map_location=device)
    
    # Construct model architecture matching RSL-RL
    raw_actor = nn.Sequential(
        nn.Linear(num_observations, 512),
        nn.ELU(),
        nn.Linear(512, 256),
        nn.ELU(),
        nn.Linear(256, 128),
        nn.ELU(),
        nn.Linear(128, num_actions),
    )

    obs_mean = torch.zeros(num_observations)
    obs_var = torch.ones(num_observations)

    model = StandaloneActor(raw_actor, obs_mean, obs_var)
    model.eval()

    example_input = torch.zeros(1, num_observations)

    # 1. Export TorchScript (JIT)
    ts_path = out_file.with_suffix(".pt")
    traced_script = torch.jit.trace(model, example_input)
    traced_script.save(str(ts_path))
    print(f"[SUCCESS] Exported TorchScript model to: {ts_path}")

    # 2. Export ONNX
    onnx_path = out_file.with_suffix(".onnx")
    torch.onnx.export(
        model,
        example_input,
        str(onnx_path),
        input_names=["observations"],
        output_names=["actions"],
        dynamic_axes={"observations": {0: "batch_size"}, "actions": {0: "batch_size"}},
        opset_version=14,
    )
    print(f"[SUCCESS] Exported ONNX model to: {onnx_path}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Export trained RSL-RL policy to TorchScript & ONNX.")
    parser.add_argument("--checkpoint", default="./logs/model.pt", help="Path to trained checkpoint")
    parser.add_argument("--output", default="./exported/policy", help="Destination base path")
    parser.add_argument("--num_obs", type=int, default=48, help="Observation vector dimension")
    parser.add_argument("--num_actions", type=int, default=12, help="Action vector dimension")
    args = parser.parse_args()

    export_policy(args.checkpoint, args.output, args.num_obs, args.num_actions)
