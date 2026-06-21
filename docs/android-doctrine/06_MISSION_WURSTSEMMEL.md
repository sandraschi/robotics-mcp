# Android Doctrine: The "Wurstsemmel Protocol"

This mission storyboard illustrates the high-fidelity synergy between a **Stationary Chef** (e.g., Yahboom DOFBOT) and a **Mobile Carrier** (e.g., Noetix Android), representing a SOTA v16.15 benchmark for agentic federated robotics.

## 🍱 Mission Goal
Deliver a freshly plated "Wurstsemmel" (Austrian sausage roll) from the preparation area to the Client (Sandra) using zero-manual intervention.

---

## 🤖 The Robot Cast

### 🏗️ 1. The Chef (Stationary/Mobile Arm)
- **Role**: Precision Plating & Preparation.
- **Hardware**: 6-DOF Mechanical Arm.
- **Logic**: Uses Inverse Kinematics (MoveIt!) to pick the Wurstsemmel and place it perfectly onto a dockable tray. 

### 🚶 2. The Carrier (Mobile Android)
- **Role**: Stabilized Delivery.
- **Hardware**: Bipedal/Mobile Android (e.g., Noetix Bumi).
- **Logic**: 
    - Navigates to the Chef station using shared fleet SLAM data.
    - Slides into the Tray's tablet-style holder.
    - Executes a stabilized "Delivery Walk" to the user's coordinates.

---

## 🔄 Workflow Execution

1.  **Orchestration**: The user triggers the `wurstsemmel_request` workflow via the Dashboard.
2.  **Staging**: The Chef confirms the tray is clear and uses its arm to plate the roll.
3.  **Coupling**: The Android moves into position (e.g., "Deep Crouch") and slides into the tray mount.
4.  **Transit**: The Android stands up and navigates to the destination, using its mass to provide a stable, non-tipping gait.
5.  **Completion**: Navigation ends at the target coordinates for handoff.

---

## 🛠️ Infrastructure Requirements

- **Unified DDS Bridge**: Enables real-time transform sharing between the Chef coordinates and the Android navigation base.
- **MCP Federated Search**: Allows the AI assistant to "find" which unit is currently equipped with the gripper arm and which is free for delivery.
- **Virtual Twin (VT) Simulation**: Modeling the passive hock attachment in simulation (e.g., Isaac Gym) to ensure stability during locomotion.

---
*Document Version: SOTA v16.16*
