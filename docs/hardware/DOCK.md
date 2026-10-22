# Autonomous Charging Dock (the not-in-A3 part)

**Collected:** 2026-10-22
**Why:** Nori A3's most conspicuous gap (REVIEW.md) — 432 Wh, 6–8 h runtime,
no dock anywhere in spec/SDK/launch. A chore robot that creates a daily
plug-in chore. Saschirob plans the dock from day one; A3 owners get a
retrofit pattern.
**Live in:** `rust-saschirob-mcp` MJCF (`dock_station` body), glTF dojo corner,
`dock.status` Demo oracle. BOM rows in `models/BOM_SOURCING.md`.

## Station (passive, cheap, ~EUR 30-60)

- Floor pad 0.6x0.6 m (laser-cut plywood/acrylic at techLAB) + back pylon
  with spring/pogo charge contacts at base-rear height (~0.16 m).
- AprilTag 36h11 (printed on paper — the cheapest precision part on the robot)
  on the pylon face for near-field visual servo.
- 38 kHz IR LED beacon on the pylon + TSOP receiver on the robot for
  far-field homing (Roomba's trick, two decades proven).

## Robot side

- Charge contact nubs on the base rear, wired to the 12 V bus behind the
  fuse + e-stop (never bypass the safety chain for charging).
- Battery voltage sense into the control loop; `dock.status` reports
  `docked / battery_pct / charge_w` (Demo values today).

## Homing strategy (planned, not built)

1. Far-field: rotate-toward-IR-maximum (single TSOP, bang-bang, then gradient).
2. Mid-field: drive the AprilTag bearing/range from the head camera.
3. Near-field: slow forward onto the pad until contact voltage reads charge
   current, then latch `docked=true` and sleep the arms.

## What A3 would need to retrofit

Pylon + pad (weekend build), tag (printer), IR pair (~EUR 10), and the one
thing money can't buy used: a `docked` state in the control loop. Saschirob
has the state from day one — the metal can come later.
