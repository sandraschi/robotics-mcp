# Autonomous Robotics Pipeline: CAD -> OpenUSD -> Isaac Lab -> ROS 2 -> Active Inference

Distilled from the fleet robotics research session (October 24, 2026).
Engineered for **RTX 4090 / WSL2 Linux** running NVIDIA Isaac Lab + ROS 2 Humble/Jazzy.

---

## 🏗️ Architecture Stack

```mermaid
flowchart TD
    CAD["FreeCAD / SolidWorks / STEP"] -->|Export STL/OBJ| Blender["01_cad_to_usd_blender.py"]
    Blender -->|Clean Geometry & Scale| RawUSD["Raw OpenUSD (.usd)"]
    RawUSD -->|pxr Physics & Articulation| Mutator["02_usd_mutator_pxr.py"]
    Mutator -->|PhysX ArticulationRoot| IsaacUSD["Simulation-Ready USD"]

    subgraph Simulation ["Isaac Lab RL Sim (RTX 4090 / WSL2)"]
        IsaacUSD --> IsaacRunner["03_isaac_lab_runner.py (4096 Envs)"]
        IsaacRunner -->|PPO Training| Checkpoint["model_*.pt (RSL-RL)"]
        Checkpoint --> Exporter["04_rsl_rl_policy_export.py"]
        Exporter --> Standalone["TorchScript (.pt) / ONNX"]
    end

    subgraph Execution ["Real-Time Hardware / ROS 2 (50-200 Hz)"]
        Cognitive["07_active_inference_supervisor.py (FEP / VFE / EFE)"] -->|cmd_vel| PolicyBridge["05_ros2_policy_bridge.py (50 Hz)"]
        Standalone --> PolicyBridge
        PolicyBridge --> Watchdog["06_hardware_watchdog.py (100 Hz E-Stop)"]
        Watchdog --> ROS2Control["ros2_control Manager (200 Hz)"]
        ROS2Control --> Hardware["EtherCAT / CANopen Motors"]
    end
```

---

## 📁 Pipeline Components

| Script / Config | Purpose |
|---|---|
| [`01_cad_to_usd_blender.py`](file:///D:/Dev/repos/isaac-mcp/pipeline/01_cad_to_usd_blender.py) | Headless Blender script to import CAD (STL/OBJ/PLY), fix normals, rescale mm to meters, and export OpenUSD. |
| [`02_usd_mutator_pxr.py`](file:///D:/Dev/repos/isaac-mcp/pipeline/02_usd_mutator_pxr.py) | Pixar USD (`pxr`) mutator attaching `UsdPhysics.ArticulationRootAPI`, collision meshes, mass tensors, and materials. |
| [`03_isaac_lab_runner.py`](file:///D:/Dev/repos/isaac-mcp/pipeline/03_isaac_lab_runner.py) | Isaac Lab launcher with 4096 parallel environments, observation/reward shaping, and RSL-RL PPO configuration. |
| [`04_rsl_rl_policy_export.py`](file:///D:/Dev/repos/isaac-mcp/pipeline/04_rsl_rl_policy_export.py) | Checkpoint exporter freezing actor weights + observation normalization into TorchScript (`.pt`) and ONNX. |
| [`05_ros2_policy_bridge.py`](file:///D:/Dev/repos/isaac-mcp/pipeline/05_ros2_policy_bridge.py) | ROS 2 Python node running policy inference at 50 Hz, bridging `/cmd_vel` and sensor streams to `ros2_control`. |
| [`06_hardware_watchdog.py`](file:///D:/Dev/repos/isaac-mcp/pipeline/06_hardware_watchdog.py) | 100 Hz safety daemon: heartbeat timeout, joint limit clamping, slew rate smoothing, and latching hardware E-stop. |
| [`07_active_inference_supervisor.py`](file:///D:/Dev/repos/isaac-mcp/pipeline/07_active_inference_supervisor.py) | Karl Friston Active Inference agent: minimizes Variational Free Energy (perception) and Expected Free Energy (action planning). |
| [`08_nori_a3_active_inference_arm.py`](file:///D:/Dev/repos/isaac-mcp/pipeline/08_nori_a3_active_inference_arm.py) | 7-DoF Active Inference Arm Controller for `norirobotics a3`: singularity-free compliant reaching via VFE gradient descent. |
| [`09_saschibot_active_inference_arm_lift.py`](file:///D:/Dev/repos/isaac-mcp/pipeline/09_saschibot_active_inference_arm_lift.py) | Whole-body Active Inference controller for `saschibot`: harmonizes prismatic lift (m) and revolute arm (rad) degrees of freedom. |
| [`10_mujoco_active_inference_closed_loop.py`](file:///D:/Dev/repos/isaac-mcp/pipeline/10_mujoco_active_inference_closed_loop.py) | Closed-loop MuJoCo 3.x physics verification of Laplace Active Inference whole-body bimanual + telescopic column control on `saschirob.xml`. |
| [`config/ros2_control.urdf.xacro`](file:///D:/Dev/repos/isaac-mcp/pipeline/config/ros2_control.urdf.xacro) | Modular `ros2_control` hardware interface configuration for Wheeled Telescoping Bimanual (`saschirob` / `Nori A3`) Feetech STS bus + diff-drive + IMU. |
| [`config/ros2_controllers.yaml`](file:///D:/Dev/repos/isaac-mcp/pipeline/config/ros2_controllers.yaml) | 200 Hz Controller Manager configuration (`diff_drive_controller`, `lift_controller`, `left_arm_controller`, `right_arm_controller`, grippers). |

---

## ⚡ Quickstart Guide (WSL2 / Linux)

### 1. Convert CAD to OpenUSD
```bash
blender --background --python 01_cad_to_usd_blender.py -- \
  --input ../assets/robot.stl \
  --output ./usd/robot_base.usd \
  --scale 0.001
```

### 2. Inject Physics & Articulation
```bash
python 02_usd_mutator_pxr.py \
  --input ./usd/robot_base.usd \
  --output ./usd/robot_sim_ready.usd \
  --root /World/Robot
```

### 3. Launch Isaac Lab RL Training (RTX 4090)
```bash
./isaaclab.sh -p 03_isaac_lab_runner.py --num_envs 4096 --headless
```

### 4. Export Policy to TorchScript
```bash
python 04_rsl_rl_policy_export.py \
  --checkpoint ./logs/model_1500.pt \
  --output ./exported/policy.pt
```

### 5. Launch ROS 2 Real-Time Control & Safety Watchdog
```bash
# Terminal 1: Safety Watchdog
ros2 run fleet_robotics 06_hardware_watchdog

# Terminal 2: Policy Bridge Node
ros2 run fleet_robotics 05_ros2_policy_bridge --ros-args -p model_path:=./exported/policy.pt

# Terminal 3: Cognitive Active Inference Supervisor
python 07_active_inference_supervisor.py
```
