# Head, Sensors, Screen

**Collected:** 2026-10-22
**Reference:** Nori A3 sensor head — 4x 720p, 2D LiDAR, dual-mic + speaker
**Sources:** `norirobotics-mcp/docs/HOME_ROBOT_PARADIGM.md` ("A Head, Not a Face"), `DIY_A3_EQUIVALENT.md`, `REVIEW.md`
**Status:** hw knowledge collection

## Doctrine: a head, not a face

Cameras/mics need the highest clear vantage — top of the lift column. No
expressive human-mimicking face needed; coverage of (a) what the arms are
doing and (b) what's happening in the room. Uncanny valley is a cost, not a feature.

## A3 baseline (match this)

- **Cameras 4x 720p:** gripper x2 + head + neck. Commodity USB webcams, ~$60–120 total. Gripper views are task cameras (see SO_ARM_GRIPPER_HISTOLOGY.md Arducam OV9281 jaw-aimed holder pattern); head/neck are room cameras.
- **LiDAR 2D, 12m, 8–12Hz:** RPLidar A1 is near-exact match (360°, 12m, up to 10Hz, <0.5mm @1.5m), $180–220 retail, ~$300 with controller/adapter. Single most expensive sensor line item.
- **Audio:** commodity USB dual-mic array + onboard speaker, ~$40–80. Confirm full-duplex (simultaneous record+playback), not record-only.
- **Compute split:** Pi 5 (4GB matches A3, 8GB headroom) for I/O + control loop; policy/AI off-board. Jetson Orin Nano (~$250+) is Exceed only if onboard inference headroom wanted.

## Screen (saschirob decision)

A3 has no meaningful face-screen in sourced spec — status LEDs + cloud app.
For saschirob, screen is a product question, not a sensing one:

- **No screen (A3-like):** cheapest, all UI in phone/webapp + voice. Keeps head small, power low.
- **Small status screen (recommended v1):** 3.5–5" TFT/IPS showing state, battery, Wi-Fi, current task, e-stop hint. Huge debuggability win in the lab, negligible cost (~$20–35), no face mimicry.
- **Full face display:** explicitly against paradigm doc — reads as humanoid, triggers the 2AM-kitchen problem. Skip unless social-use case demands it.

Recommendation: status screen + physical e-stop + mute button on head, nothing anthropomorphic.

## Wiring / integration notes

- Head rides the lift top — cable chain / coiled USB + power loom must survive 700mm travel; budget the 1.121x wiring-mass factor from DIY doc.
- USB bandwidth: 4x 720p + LiDAR on Pi 5 needs a powered hub + loom separation (motor 12V away from USB data). Known Pi failure mode, plan it now.
- Mic placement away from lift actuator + wheel noise; speaker facing forward-down, not into mics.
