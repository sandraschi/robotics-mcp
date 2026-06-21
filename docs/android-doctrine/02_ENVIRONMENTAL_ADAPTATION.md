# Android Doctrine: Environmental Adaptation

This document codifies the "Android-First" approach to household architecture. Instead of over-engineering robot torque, we modify the environment to accommodate autonomous agents.

## 🏗️ 1. The Amenability Principle
The most successful domestic Androids do not fight the environment; they collaborate with it. We prioritize **Low-Obstacle Infrastructure** to reduce the mechanical requirements of the fleet.

## 🛠️ 2. Key Household Modifications

### 🧊 A. The "Easy-Open" Fridge Standard
Standard fridge seals require high pull-force (~15-30N). 
- **The Mod**: Retrofitting magnetic seals or push-to-release mechanisms that allow a Level 1 (Mini-Claw) Android to initiate opening.
- **The Android Rule**: A fridge is an "Object Hub," not a "Fortress."

### 🚪 B. Unlocked/Push-to-Open Cupboards
Androids struggle with traditional door handles.
- **The Mod**: Replacing complex handles with standardized **Magnetic Push-Latches**.
- **The Android Rule**: Any cupboard containing Android-accessible tools (Cleaning supplies, Battery spares) must be "Handle-Neutral."

### 🧽 C. Open Tool Accessibility
- **The Mod**: Storing "Utility Objects" (Feather dusters, Trays, Tablets) in open **Reception Docks** rather than enclosed spaces.
- **The Android Rule**: If the Android cannot see the tool's interaction handle, the tool does not exist for the autonomous agent.

## 🗺️ 3. Physical Mapping: The Niantic Standard

To achieve **Tier 2 Intelligence**, the household must be converted into a high-fidelity digital twin.
- **Niantic Mapping**: We utilize the **Niantic/Scaniverse** ecosystem for 3D Gaussian Splatting. 
- **The Workflow**:
    1. A human operator performs the initial apartment scan using Niantic Lightship.
    2. The resulting SPLAT/PLY data is exported to the fleet knowledge base.
    3. The Android uses the 3D Splat as a reference for semantic object location (Spatial RAG).

## 🛒 4. External Mobility: The Passive Trolley

The Android's interaction with the outside world (e.g., the SPAR grocery mission) is governed by the **Passive Traction Standard**:
- **Mechanical Hook**: A standardized, low-cost hitch for standard "Cheapo" push/pull trolleys or shoppers.
- **Passive Tractive Effort**: The Android acts as the primary motor; the trolley is passive (no independent drive-wheels).
- **Control Law**: The Android must utilize stability-aware braking to account for trolley inertia during descent or emergency stops.

## 🌐 5. Interaction Standards
- **Digital Handshakes**: High-torque appliances should inform the Android of their state (Open/Closed/Locked) via MQTT/Matter to prevent "Dead-Lock" scenarios.
- **Path Clearance**: Maintaining a 50cm "Standard Corridor" in living spaces to ensure 100% navigation uptime.

---
*Document Version: SOTA v16.16*
