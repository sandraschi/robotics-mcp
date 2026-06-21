# Android Robotics Doctrine: Chapter 12
## Legal and Self-Defense Vectors (The Aegis Protocol)

As autonomous systems integrate into the primary living space, the necessity for **Self-Defense and Deterrence** arises. This chapter defines the legal, ethical, and operational framework for bots acting as "Protectors of the Mothership" while maintaining strict adherence to non-lethal and human-in-the-loop (HITL) principles.

### 1. Legal Framework: The Castle Doctrine for Bots
Autonomous protectants are legally classified as "Advanced Home Security Actuators."
- **Entity Identification**: All deterrence actions must be preceded by a clear audio-visual warning: *"UNAUTHORIZED ENTRY DETECTED. RETREAT TO THE PERIMETER OR NON-LETHAL DETERRENCE WILL BE DEPLOYED."*
- **The Aegis Defense**: Bots are authorized to defend the primary user and the physical property using "Proportionate Force."
- **Liability Stacking**: The Operator (User) remains legally responsible for the bot's actions. Bots must maintain a "Black Box" log of all deterrence events (Visual, LIDAR, Audio) for legal evidence.

### 2. Deterrence Hierarchy (The Escalation Ladder)
Bots must follow a strict escalation path to minimize permanent harm:

| Level | Action | Tooling Requirement |
| :--- | :--- | :--- |
| **0: Presence** | Patrol Mode / Active Scanning | Visible LED indicators (Blue), motor noise. |
| **1: Audio-Visual** | Strobe Lights / High-DB Alarm | High-intensity RGB strobes + 110dB sirens. |
| **2: Physical Obstruction** | Passive Blocking | Bot positions itself between intruder and user. |
| **3: Non-Lethal Denial** | Chemical/Liquid Diverter | Deployment of pepper spray or marking dye. |
| **4: Physical Engagement** | Joint-Locking/Entrapment | Bot uses manipulators to pin (not crush) limbs. |

### 3. Self-Defense for the Bot (Anti-Tamper)
To prevent "Unboxing" or theft of the hardware:
- **Electro-Static Discharge (ESD) Pulse**: Low-amperage surface charge on chassis to deter unauthorized touching.
- **Panic Geo-Fence**: If the bot is lifted/moved outside the Mothership Perimeter, it enters "Lock-Down Mode" (Internal GPS beacon enabled, motors locked).
- **Silent Alarm**: Immediate notification to the User's mobile device via Advanced Memory MCP.

### 4. Human-In-The-Loop (HITL) Override
Self-defense vectors (Level 3+) **MANDATE** a user override:
1. **Detection**: Bot identifies a threat.
2. **Notification**: Bot sends a "Prompt for Force" to the User.
3. **Authorization**: User must provide biometric/secret-key approval via the MCP Interface.
4. **Exception**: Level 2 (Blocking) is allowed autonomously if the User is incapacitated or "Comatose Mode" is detected.

---
*Propagated via Robotics MCP - April 2026*
