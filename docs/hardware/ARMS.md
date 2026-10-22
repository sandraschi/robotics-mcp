# Arms

**Collected:** 2026-10-22
**Reference:** SO-ARM100/101 (LeRobot native) + Nori A3 2x 7+1-DOF, 55cm reach, 1.5kg/arm
**Sources:** `norirobotics-mcp/docs/DIY_A3_EQUIVALENT.md`, `ACTUATOR_ALTERNATIVES.md`, `HOME_ROBOT_PARADIGM.md`
**Status:** hw knowledge collection

## Base: SO-101 mechanical design (start here)

- 6-DOF, 3D-printed structure, native Hugging Face LeRobot support — the open lineage Nori itself builds on (XLeRobot/SO-ARM100 4.8k+ stars)
- Kit: SO-ARM101 ~$199/arm complete, ~$115–150/arm motor-only if printing own structure (Seeed Studio; TheRobotStudio/SO-ARM100)
- Stock servo: Feetech STS3215 (30 kg-cm / 2.9 N-m). Fine for wrist/gripper. NOT enough for shoulder at A3 spec.
- Lighter predecessor warning: XLeRobot arms are 5-DOF, ~40cm reach, 600–1000g/arm, no telescoping lift — good reference, not a spec match.

## The load-bearing math (why shoulder needs upgrade)

Holding 1.5kg statically at 55cm:

```
tau = F x r = (1.5 x 9.81) x 0.55 ≈ 8.1 N-m
```

Before arm self-weight, dynamics, or margin. A3's own shoulder STS3095 (95 kg-cm / 9.3 N-m) sits almost exactly on that number — 1.5kg @ 55cm IS the ceiling of a 9.3 N-m actuator. SO-101 stock STS3215 at shoulder cannot hit A3 payload. Proven, not opinion.

## Tiers

### Match (same actuator class as A3)

- Swap shoulder (+ ideally elbow) to Feetech STS3095 (9.3 N-m, A3's own part). Extrapolated $70–120/unit from STS3215→3250 curve ($16→$55 for 30→50 kg-cm) — confirm before budgeting.
- Keep STS3215/3250 everywhere else (wrist, gripper — no static payload load).
- 2x arms: ~$400 kits + 4x upgraded joints ≈ $680–880.

### Exceed (QDD, real force feedback)

- Damiao DM-J4340 (27 N-m, ~3x A3 shoulder) or RobStride 01/02 (17 N-m, ~$260-class). Native FOC current-sensing = real torque estimation, vs. Nori's software current-polling workaround on 1:345-geared Feetechs.
- Cost: 5–15x per Feetech joint for 2–3x torque + feedback. 2x arms ≈ $1,440 in joints alone.
- Integration price: round-body QDD needs new brackets (not Feetech horn pattern), 24–48V bus vs. 12V elsewhere, CAN/EtherCAT vs. TTL serial + firmware work.

## Software: fully local is the real exceed

Nori ships cloud-mediated (WebRTC + Supabase) — internet + third-party infra to run a robot in your own home. DIY runs LeRobot directly over USB/direct-wire, entirely local: no account, no telemetry question. That alone beats A3 regardless of tier.

Homework stock LeRobot does NOT include: actuator-protection layer (poll Present_Current, thermal/stall thresholds, cut on latch). Straightforward from Feetech register map, but genuinely new work — budget it.

## Saschirob guidance

- v1: SO-101 geometry + STS3095 shoulders/elbows, STS3215 wrists, 12V single bus, LeRobot local. Matches A3 reach/payload, stays in $1,400–2,100 total build band (Match tier rollup in DIY doc).
- Gripper/head/lift/base in sibling docs: `SO_ARM_GRIPPER_HISTOLOGY.md`, `TELESCOPIC_LIFT.md`, `ROLLING_BASE.md`, `HEAD_SENSORS_SCREEN.md`.
- Sourcing: actuators/electronics from Shenzhen-adjacent (CubeMars/MyActuator/Damiao/RobStride via RobotShop/Seeed/AliExpress at joint quantities); structure printed locally where tolerances can be inspected (A3 meshes are real-shape but non-watertight — blender-mcp/qcad-mcp repair pass, don't outsource fit-critical prints).
