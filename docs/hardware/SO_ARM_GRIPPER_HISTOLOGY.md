# SO-ARM100/101 Parallel Gripper + Histology-Slide Mod

**Bookmarked:** 2026-10-22
**Source:** https://github.com/roboninecom/SO-ARM100-101-Parallel-Gripper
**Mod path:** `community/histology-slide-gripper/`
**Upstream:** roboninecom, 195* / 20 forks (2026-10-22), topics: gripper, so-arm100, so-arm101, lerobot
**Contributed by:** Michael Viacheslavov (histochemichael)
**Status:** reference only — do NOT vendor into fleet, link as upstream hardware

## Why kept

Different kind of repo (hardware CAD + sim + BOM, not an MCP server) but directly
relevant to robotics stack + future maker stack. Precision lab handling
(75x25mm slides, staining dipper/rack) is a better end-effector demo than
pick-and-place cubes, and the repo structure is the template to copy for
saschirob hardware docs.

## Base gripper (for context)

- 1-DOF parallel jaw, 84mm stroke, 120N max, 0.5mm repeatability, ~170g in PLA 30%
- 128x109x130.5mm, prints on 180x180mm bed (Bambu A1 mini / Prusa MINI+ class), 2-4h print
- Servo: Feetech STS3215 (12V, 30 kg-cm / ~2.9 N-m, 45 RPM, 12-bit absolute magnetic encoder, RS485/TTL)
- Cost ~$50: servo ~$29, MF106ZZ x2, D6x1x125mm tubes x2, ~100-150g PLA, M2/M4 fasteners
- Camera holders: IMX335 5MP, GC2093 2MP, Orbbec Gemini 2, RealSense D405 (close-range) / D435 / D455
- Assembly 30-45 min, 11-step guide with photos
- License: CERN-OHL-P-2.0 (CAD) / Apache-2.0 (code) / CC BY 4.0 (docs) + REUSE.toml — clean

## Histology mod — what changed

- Long flat clamps for direct pickup of standard 75x25mm microscope slides
- Clamp spacing/tips engage slide staining dippers/racks (ref target: Medicus Health plastic dipper with handle)
- Angled Arducam holder: Arducam 100fps Mono Global Shutter USB (OV9281) aimed at jaws for close-up grasp monitoring
- Full printable STL + editable STEP: `community/histology-slide-gripper/cad/the-parallel-microscope-slide-gripper.{stl,step}`
- Gallery: single-slide pickup, dipper/rack pickup, Arducam front view, front/side/iso CAD renders, demo gif
- Own warning: test on non-critical slides first, tune material/infill + gripper force

## Sim assets

- `community/histology-slide-gripper/mujoco/` — self-contained MuJoCo 3.4.0 model: full SO-ARM101 + gripper meshes, standalone testbench, build scripts, headless grasp-and-lift check. Run: `conda activate lerobot; cd community/histology-slide-gripper/mujoco; pip install -r requirements.txt; python view.py` (`sg_grip` control)
- Base repo also ships `simulation/{gazebo,mujoco,webots,coppeliasim,isaac_sim,so_arm_101_description/}` — ROS2 URDF + Docker + multi-sim guides
- Demo: https://www.linkedin.com/posts/michael-viacheslavov_histology-histotechnology-robotics-ugcPost-7482570199859240960-Gqpw/

## Fleet relevance

- `vla-mcp` / `teleoperator-mcp`: SO-ARM100/101 is the LeRobot reference arm — native LeRobot support, same servo bus family as DIY_A3 arms in norirobotics-mcp
- `mujoco-mcp`: histology `mujoco/` folder is a ready-made test asset (plus base multi-sim)
- `universal-actuator-mcp` / `ros-mcp` / `gazebo-mcp` / `isaac-mcp`: STS3215 control, URDF, Gazebo/Isaac guides
- `freecad-mcp` / `codecad-mcp` / `blender-mcp`: STEP (editable) + STL (printable) pair is the pattern to demand for every saschirob part; non-watertight mesh repair workflow from norirobotics-mcp DIY_A3_EQUIVALENT.md applies if meshes need cleanup
- `bench-camera-mcp`: jaw-aimed global-shutter holder is the pattern for fingertip-proximate observation
- Torque ceiling note: STS3215 (2.9 N-m) cannot do A3-class 1.5kg @ 55cm (~8.1 N-m static, needs STS3095 9.3 N-m at shoulder/elbow) — gripper/wrist servo is fine, shoulder needs upgrade per norirobotics-mcp `DIY_A3_EQUIVALENT.md`

## For saschirob (rolling telescoping bimanual, Nori A3-inspired)

- Reuse: SO-101 arm mechanical base + LeRobot stack (fully local, no cloud), standing-desk-column lift (what A3 itself uses), differential-drive base to start
- Gripper decision: match A3 2-finger claw vs. exceed with Yale Model O / RAMEL-ESPOL 3-finger vs. domain fingertips like this histology mod — fingertips are swappable, base jaw mechanism stays
- Maker-stack template from this repo: `models/parts/*.stl` + editable source, assembly guide with photos, BOM with buy links, multi-sim in parallel, `community/` for domain variants, triple license + REUSE.toml, i18n docs
- Next: decide `saschirob` (hardware CAD/docs, NO -mcp suffix) vs. `saschirob-mcp` (fleet control server) — recommendation is both, hardware first. See saschirob proposal (to be filed under `norirobotics-mcp/docs/` or new repo).
