# Android Doctrine: Hardware Research

This document tracks the research into lightweight, affordable hardware components for Android prototypes (e.g., small-scale bipedal platforms).

## 🎯 1. Requirements for Mobile Manipulation
- **Weight**: <100g (Total with servo) to minimize gait disturbance for small units.
- **Actuator**: SG90 or MG90S (9g Micro Servo) for low-power ESP32 integration.
- **Payload**: target ~300g (sufficient for tablets, trays, or specialized sensors).

## 🛠️ 2. Recommended Gripper Options

| Option | Pros | Cons | Ideal Use |
| :--- | :--- | :--- | :--- |
| **Aluminum Mini-Claw** | Durable, Premium Look (~70g). | Slightly heavier than plastic. | **Tablet Holding** / Tray Service. |
| **Stopgap Toy Arms** | Ultra-Cheap ($50-$80), Fast-Ship. | Flimsy, Low-Payload (~150g). | **Software Prototyping** / Basic Handoff. |
| **3D-Printed Soft-Grip** | Ultralight (<50g), customizable. | Less durable, requires printing. | **Fruit/Delicate Object** handling. |
| **Plywood Laser-Cut** | Cheap, DIY friendly. | Aesthetics don't match premium VT. | Rapid prototyping. |

## 📦 3. Procurement Realism: The "3-Day Standard"

A critical lesson of the April 2026 fleet expansion is **Procurement Velocity**. 

- **The Yahboom Standard**: Hardware that is available via **Amazon.de (3-day delivery)** is prioritized. This ensures that the fleet can be repaired, replaced, or expanded within a single work-week.
- **The "False Trail" (Moorebot Lesson)**: Avoid "Import-Trap" platforms. Rebranded Chinese toy-bots (like the Moorebot Scout) with high acquisition friction and proprietary barriers are logically and physically archived. 
- **Rule**: If the hardware cannot be procured in <7 days without complex international shipping hurdles, it is not SOTA-compliant for a rapid-response fleet.

## 📜 3. Historical Precedent: The Dalek Principle

The **Dalek Plunger Arm** (BBC, 1963) remains one of the most effective "Affordable Industrialization" examples in robotics history.
- **The £5 Prop**: By using a domestic sink plunger, the BBC demonstrated that **High-Adhesion Vacuum** is a viable alternative to complex (and expensive) multi-fingered hands.
- **Modern SOTA (2026)**: We see this lineage in the **Universal Jamming Gripper** and professional vacuum suction hubs used in logistics. 
- **Application for Androids**: A passive suction cup (The "Dalek Standard") offers a zero-power, fail-safe method for transporting smooth trays or tablets, perfectly aligning with lightweight requirements.

## 📐 4. Integration Path
1. **Mounting**: Use existing passive claw mount points.
2. **Power**: External 5V rail (Common Ground with Controller). Do NOT power directly from micro-controller signal pins.
3. **Control**: PWM-based servo control libraries.

---
*Document Version: SOTA v16.16*
