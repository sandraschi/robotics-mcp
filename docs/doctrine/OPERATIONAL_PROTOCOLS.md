# Operational Protocols: Fleet Orchestration

Standardized behaviors for the 2026 Android Fleet.

## 🏗️ Layered Architecture (Host-Controller)
The fleet follows a decoupled **Host-Controller Separation** pattern:
- **Base Controller (E1/RosMaster)**: A dedicated micro-controller (STM32/ESP32-class) running reactive firmware for motion control, joint torques, and sensor feedback.
- **Autonomous Host / Bridge (Jetson/Pi)**: The **Minimum Functional Bridge**. A local Linux node required to bridge the E1's hard-wired interfaces (Ethernet/Serial) to the wireless network (WiFi/5G). The base E1 controller board does not support native wireless slave operation.
- **Mothership (Remote PC)**: The high-level **Brain**. Communicates with the robot via the local bridge node.

## 🚀 Swarm Scalability: The "Zero-Host" Pattern
For multi-robot deployments, the fleet supports a **Zero-Host Configuration**:
- **Design**: Removes the local Linux host (Pi/Jetson) from each agent.
- **Connectivity**: Uses a low-cost network bridge (WiFi-to-Ethernet/UART) to expose the base controller directly to the network.
- **Orchestration**: A single powerful Mothership (RTX 4090+) manages 3+ agents simultaneously as remote Micro-ROS nodes.
- **Benefit**: Reduces per-unit cost by ~60%, enabling industrial-scale swarm research.

## 📡 Middleware Standards (DDS / Micro-ROS)
The fleet utilizes **DDS (Data Distribution Service)** as the common language:
- **Protocol**: Micro-ROS (DDS-XRCE) runs natively on E1-class controllers.
- **Telemetry**: Exposes `/joint_states`, `/imu/data`, and `/battery/status`.
- **Commanding**: Accepts `/cmd_vel` and high-rate `/joint_commands`.

## 🚶 Bipedal Control (Noetix Bumi)
- **Hard Real-Time Balancing**: The E1 controller performs high-frequency (~1kHz) IMU/PID integration. This "Reflex Layer" ensures the robot maintains dynamic balance even during high-latency Mothership inference rounds.
- **Gait**: High-stability bipedal walking is the default for indoor navigation.
- **Gesture Expressivity**: Use 21-DOF capability to signal intent (e.g., pointing direction of travel).
- **Compute Latency**: Real-time inference (Orin) is supported on Research SKUs; base consumer models focus on reactive motion control.
- **Virtual Twin Sync**: All physical telemetry must mirror to the Unity/VRChat virtual twins with <50ms latency.

## 🏎️ Holonomic Navigation (Yahboom Boomy)
- **Mecanum Optimization**: Utilize sideways strafing for tight corridor navigation.
- **Speed Caps**: Indoor speed is capped at 0.5m/s for safety.
- **Sensor Sweeps**: Perform periodic 360° ultrasonic/camera sweeps when in "Idle/Sentry" mode.

## 🤝 Task Handover
- **Cross-Robot Communication**: Use `robotics-mcp` to coordinate task transfers (e.g., Boomy scouts a room, Bumi enters to interact).
- **Protocol**: `handover_request` -> `capability_check` -> `active_transfer`.

---
*Federated Intelligence. Local Reliability.*
