# Rolling Base (keep it simple)

**Collected:** 2026-10-22
**Reference:** Nori A3 45x45cm differential-drive + casters; XLeRobot Lekiwi holonomic option
**Sources:** `norirobotics-mcp/docs/DIY_A3_EQUIVALENT.md`, `HOME_ROBOT_PARADIGM.md`
**Status:** hw knowledge collection — deliberately minimal

## Doctrine

Wheels, not legs. Home is flat, cluttered, human-scaled — legs buy stairs at
the cost of balance stack, fall risk, and 10+ extra actuators. Low center of
mass, can't fall over, budget goes to arms. A3 proves the shape.

## Match tier (build this)

- Differential drive + 2 passive casters, 45x45cm footprint
- 2x commodity 12V geared DC motor + wheel, ~$15–30/side
- Total base: ~$60–100
- Handles thresholds/ramps with enough ground clearance; simpler kinematics, more robust than omni

## Exceed option (only if aisles demand it)

- Lekiwi-style holonomic 3x omni-wheels — strafe without 3-point turn in cluttered aisles
- Cost: ~$150–250, see XLeRobot material list for reference
- Trade: maneuverability for complexity/robustness. A3 itself ships differential — no need to exceed on v1.

## Saschirob notes

- Keep v1 differential. Holonomic is a bolt-on later.
- Missing on A3 (conspicuous, per REVIEW.md): no autonomous charging dock anywhere in spec/SDK/launch. 432Wh / 6–8h means a human-plug chore. Plan dock + auto-charge from day one — every successful home robot won this.
- Power bus: 12V shared with arms/lift keeps PSU simple; isolate QDD 24–48V later only if Exceed arms happen.
