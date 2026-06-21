# Android Robotics Doctrine: Chapter 11
## Nonmobile Robot Protocols (Bumi Tier-1)

While mobile androids represent the peak of the fleet, the foundation of a robust autonomous household relies on **Nonmobile Robotics (Tier-1)**. These are stationary units, embedded sensors, and smart hubs that provide the "nervous system" and localized physical agency required for a seamless Mothership environment.

### 1. Definition and Scope
Nonmobile robots are categorised as:
- **Stationary Manipulators**: Fixed-base robot arms (e.g., kitchen-mounted COBOTs) for specialized tasks like meal prep or object sorting.
- **Smart Hubs**: Physical touchpoints that interact with mobile bots (e.g., auto-charging docks, mechanized storage).
- **Embedded Actuators**: Mechanized furniture, automated door openers, and motorized racks.

### 2. Operational Protocols
- **Anchor Integrity**: All Tier-1 units must be bolted to structural elements or weighted sufficiently to handle maximum torque without shifting.
- **Reachability Mapping**: Tier-1 units must report their "End-Effector Reach Volume" to the central registry (Bumi MCP) so mobile units know where to place objects for hand-off.
- **Power Redundancy**: Stationary units should have UPS backup to maintain "Safe Mode" (releasing grips/locking joints) during power failure.

### 3. Environment Retrofitting Checklist
To ensure the home is "Robot-First," the following modifications are standard for Tier-1 industrialization:

| Component | Retrofit Requirement | Rationale |
| :--- | :--- | :--- |
| **Doors** | High-torque lever handles or smart sliders | Easier for claw-type grippers than round knobs. |
| **Cabinets** | Push-to-open magnetic latches | Eliminates the need for precise handle grasping. |
| **Lighting** | 850nm IR Beacons in blind spots | Aids SLAM navigation in zero-light conditions. |
| **Storage** | Sloped "Gravity Feed" shelving | Ensures items are always at the front for easy bot retrieval. |
| **Fridge** | Mechanized door assist / Spring-loaded seals | Reduces required torque for breaking vacuum seals. |
| **Tools** | 3D-printed "Uni-Grip" handles | Standardizes the interface for all manual tools (hammers, screwdrivers). |

### 4. Integration with Mobile Units
1. **The Handshake Protocol**: A mobile unit (Tier-2/3) approaches a Tier-1 unit.
2. **Telemetry Sync**: Tier-1 reports its precise workspace coordinates.
3. **Task Hand-off**: Tier-1 takes over fine-grained manipulation (e.g., Tier-2 brings a potato, Tier-1 peels it).

---
*Propagated via Robotics MCP - April 2026*
