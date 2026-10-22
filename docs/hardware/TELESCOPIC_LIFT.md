# Telescoping Lift

**Collected:** 2026-10-22
**Reference platform:** Nori A3 (3-stage steel column) — see `norirobotics-mcp/docs/DIY_A3_EQUIVALENT.md`, `HOME_ROBOT_PARADIGM.md`, `REVIEW.md`
**Status:** hw knowledge collection, not a BOM lock

## Why it matters

Wheeled base can't crouch. The lift is what turns "fetch at counter height"
into "floor to counter": A3 covers 69–145cm working range (76cm travel;
URDF-measured 700mm commanded / 720mm physical, middle stage 0.5x mimic).
No extra DOF in base or arms. This is the capability multiplier.

A3's own paper (arXiv:2605.16537): "A three-stage steel telescoping column,
of the type used for height-adjustable desks and comparable to YOR, replaces
the fixed-stroke ball screw of the prior version." Commodity furniture
mechanism, not bespoke robotics. Nori tried ball-screw first and moved away —
stowed-height vs. reach tradeoff decided it.

## Options

### 1. Standing-desk column (build this — what A3 ships)

- Single linear actuators 12–18V, 16–18" stroke: ~$80/unit on eBay, 220–600 lbs rated (massive margin, furniture economics)
- Full DIY dual-actuator build (tube + 2 actuators + controller): $175–400 parts, 5–6h
- Caveat: most off-shelf columns are 2-stage; matching A3's exact 3-stage / 0.5x mimic takes picking the right column, not any column
- Sources: Firgelli DIY column guide, LINAK / TiMOTION / Uplift / Progressive Desk / Firgelli catalogs. "YOR" in the paper doesn't match any found brand — treat as unverified lead.

### 2. Ballscrew / lead-screw rail + NEMA17 (the road not taken)

- 200mm SFU1605 rail + NEMA17: ~$78/unit — 3D-printer/CNC commodity, precise, quiet, abundant tooling
- Why A3 dropped it: fixed-stroke screw ties stowed height to screw length; nested telescoping decouples stowed height from reach. Choose this only if optimizing purely for precision, knowing the packaging penalty.

## Saschirob guidance

- Match tier: 2-stage desk column, differential control, 12V bus shared with arms
- Exceed: true 3-stage with 0.5x belt/chain mimic between stages (verify ratio on bench, don't trust the datasheet)
- Safety: pinch points at stage interfaces need covers (home with pets/kids); add current-limit + top/bottom limit switches regardless of vendor claims; actuator-protection layer (poll Present_Current, thermal/stall cut) is homework, not included in stock LeRobot
- Total moving mass budget matters: 20.5kg-class robot (A3 measured: 18.3kg links x1.121 wiring factor) — size column for arm assembly + head, not just static load
- Gap A3 leaves open (REVIEW.md): column is vertical-only, no torso pitch/lean — back-of-cabinet and front-loader reach stays limited. Note for saschirob v2, not v1 scope.
