# Android Doctrine: Specialized Utility Bots

This document codifies the operational standards for non-bipedal utility platforms (Class U), focusing on assistive and domestic missions.

## 🐾 1. The Dogwalker Bot (The "Smart Tether" Protocol)

Walking a pet (e.g., a Chihuahua) requires balancing autonomous exploration with strict physical safety.

### 🛡️ A. Physical Guardrails
- **Hi-Vis Harness Mandatory**: To prevent accidental entanglement or strangulation (no neck-collars allowed), the bot requires a standardized **Hi-Visibility Harness** connection.
- **Tension-Release Failsafe**: The winch assembly must include a magnetic or mechanical release that triggers if a sudden "High-Torque" event (e.g., pet snagged) is detected.
- **Low-Center-of-Gravity (LCG)**: The bot chassis must be weighted to prevent being toppled by sudden lateral pulls.

### 🧠 B. Behavioral Logic (The "Benny" Filter)
The bot must prevent the pet from entering high-hazard zones (Traffic) while allowing social interaction.
- **Traffic Perimeter Enforcement**: Utilizing the **Niantic 3D Twin**, the bot maintains a hard digital fence at curbs. Even if the pet pulls toward "Benny" across the road, the bot initiates a **Soft-Stop Braking** event to prevent road entry.
- **Dynamic Leash Control**: Active winch adjustment to maintain "Heel" position in crowded areas and "Exploration" length (max 3m) in parks.
- **Social Distance Buffer**: Logic to maintain a 1.5m gap from other humans/bots unless a "Friendly Handshake" is authorized by the owner.

## 🛒 2. The Intelligent Shopper (Active Follow)

Transitioning from passive towing to active, autonomous grocery assistance.
- **Active Follow-Me**: Using UWB (Ultra-Wideband) or Visual Tags to follow the user at a 1.0m offset.
- **Obstacle Anticipation**: Using forward-facing LiDAR to navigate supermarket isles without clipping displays.
- **Mass-Aware Braking**: The drive system must actively compensate for the increasing weight of groceries to maintain consistent braking distances.

## 🛴 3. The Mobility Scooter Bot (Assistive Transit)

Providing safe, autonomous mobility for elderly or disabled users.
- **Hard-Coded Speed Cap**: Legally locked at **6km/h** (standard sidewalk speed) to prevent liability and ensure safe integration with pedestrians.
- **Stability Monitoring**: Real-time IMU sensing to detect "Tip-Risk" on uneven terrain, initiating an immediate tilt-correction or safety stop.
- **Obstacle Avoidance**: 360-degree ultrasonic/LiDAR coverage to prevent collisions with humans or infrastructure.

## 👁️ 4. The Guideway Bot (Visual Assistance)

Dedicated navigation for differently sighted users (Class U-Assist).
- **Physical Guidance**: Utilizing a handle-based tactile feedback system (e.g., Glide-style) to steer users around obstacles.
- **Semantic Narration**: Real-time LLM-based description of the environment via bone-conduction audio (e.g., "Step coming up in 2 meters," "Door to your left").
- **Traffic Light Integration**: Direct sync with pedestrian signals (via V2X or Visual recognition) to ensure safe crossing timing.

---
*Document Version: SOTA v16.17 (Extension Authorized)*
