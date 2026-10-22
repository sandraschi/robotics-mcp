# Grippers / Hands

**Collected:** 2026-10-22
**Sources:** `SO_ARM_GRIPPER_HISTOLOGY.md` (this folder), `norirobotics-mcp/docs/DIY_A3_EQUIVALENT.md` (Grippers section), `HOME_ROBOT_PARADIGM.md` (Tiered End Effectors)
**Status:** hw knowledge collection — fingertips are swappable, jaw mechanism stays

## Spectrum

1. **A3 stock (match):** geared 2-finger claw — one servo finger + idler on mimic joint (URDF-verified, `arm.xacro`). Cheap, simple, software-protectable. Two-point grip, weak on round/irregular.
2. **SO-ARM parallel jaw (lab-upgrade path):** Robonine 84mm parallel gripper, STS3215-driven, 120N / 0.5mm repeatability — see `SO_ARM_GRIPPER_HISTOLOGY.md`. Parallel jaws beat claw on slides, vials, PCBs, any flat/thin object. Domain fingertips (histology clamps) bolt onto the same base.
3. **Underactuated 3-finger (exceed):** one motor, linkage-driven, 3-point security near claw cost:
   - Yale OpenHand Model O (3-finger 4-DOF, longest-standing open reference)
   - RAMEL-ESPOL Three-Fingered Gripper (2024–25, PLA + TPU membranes, maintained STLs — best current start)
   - Alaris 2014 (four-bar + worm gear, single servo, IEEE 6935605)
4. **Nori soft-finger trick (steal regardless):** TPU fingers + sensorless force from servo current (Present_Current polling). Software substituting for fingertip sensors — same push-cost-into-software pattern as their actuator protection. Applies to any of the above.

## Decision for saschirob

- v1: A3-style 2-finger claw OR SO-ARM parallel jaw depending on first task set (household irregular → claw; lab/flat → parallel). Both cheap, both printable, both STS3215-class.
- Keep fingertip interface bolted, not glued: histology clamps today, pipette/vial/PCB fingers tomorrow.
- 3-finger underactuated is the deliberate upgrade when 2-point grip demonstrably fails — not v1 scope.
- Force: tune + current-limit for delicate objects (slides!) — 120N base claim vs. glass needs software cap, test on junk first.
