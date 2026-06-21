# Safety Standards: Hazard Mitigation & Mitigation

The Android Robotics Doctrine prioritizes human safety and environment preservation above all mission objectives.

## 🛑 Critical Protocols

### 1. Mandatory E-Stop Federation
- All agents (physical and virtual) must listen on the `/fleet/emergency_stop` topic.
- A single E-stop triggers a fleet-wide freeze of all joint torques and motor velocities.

### 2. Hazard Mitigation Zone (HMZ)
- Agents must maintain a minimum 0.5m buffer from moving human entities.
- Bipedal agents (Bumi) must enter "Stability Mode" (lower center of gravity) when humans are within 1.0m.

### 3. Niantic 3D Semantic SLAM Integration
- Use semantic labeling to identify fragile objects (glass, electronics).
- Navigation paths must avoid "Fragile Zones" by at least 0.3m.

### 4. Assistive Physical Interaction
- Bipedal agents must utilize "Compliant Control" when interacting with humans.
- Force sensors must trigger immediate release if resistance exceeds 5N during non-industrial tasks.

---
*Safety First. Automation Second.*
