# Robotics MCP — Technical documentation

[Project README](../README.md) · [Usage and API examples](README_USAGE.md)

Setup, hardware and software prerequisites, installation, packaging, IDE integration, configuration, testing, and troubleshooting.

## Crash recovery (watchfiles)

- Restart on failure with exponential backoff
- HTTP health checks (default interval 30s)
- JSON crash logs for debugging
- Example systemd units (Linux); PowerShell helpers (Windows)

See [Crash Protection Guide](../WATCHFILES_README.md) for setup.

## Critical requirements

### Hardware (recommended)

- **Physical robot**: Dreame D20 Pro (LIDAR vacuum), Moorebot Scout, Yahboom ROSMASTER, PX4/ArduPilot drones, Nori A3 (bimanual mobile manipulator, bridged via `norirobotics-mcp`)
- **Philips Hue Bridge Pro**: HomeAware RF movement detection (optional)
- **Without hardware**: Virtual robotics only (Unity3D + VRChat)

### Software (for virtual / VR workflows)

If you use Unity, VRChat, and the composed MCP fleet described in the docs:

- **Unity 3D** (6000.2.14f1+) — [Setup Prerequisites](guides/SETUP_PREREQUISITES.md#unity-3d-required-for-virtual-robotics)
- **VRChat** — [Setup Prerequisites](guides/SETUP_PREREQUISITES.md#vrchat-required-for-social-vr-robotics)
- **Peer MCP servers** — [Setup Prerequisites](guides/SETUP_PREREQUISITES.md#-required-mcp-servers)

Physical-only or HTTP-only use may not need the full virtual stack; see [Setup Prerequisites](guides/SETUP_PREREQUISITES.md) for what applies to your setup.

## AI and hardware

**Physical robotics (ROS2/SLAM):**

- **Minimum**: Raspberry Pi 5 / Jetson Orin Nano (on-board)
- **Recommended**: Discrete GPU workstation (RTX 3060+) for mapping/processing

**Virtual robotics (Unity/VRChat):**

- **VR-ready PC**: RTX 3060 or better recommended for smooth framerates

**Local intelligence (LLM):**

- **Auto-discovery**: Detects Ollama or LM Studio when running.
- **Large models (e.g. 70B)**: Typically need 24GB+ VRAM (e.g. RTX 3090/4090 class) for comfortable local inference.
- **Smaller models (e.g. 8B)**: Usable on RTX 3060+ or Apple Silicon with sufficient RAM.

> [!NOTE]
> If no local LLM or high-end GPU is detected, AI features (conversation, advanced analysis) are disabled, but core control loops remain functional.

## Prerequisites and dependencies

### Physical robots (recommended)

#### Yahboom ROSMASTER (ROS 2)

- **Yahboom ROSMASTER M1/X3/X3 Plus** — Camera, navigation, optional arm/gripper
- LiDAR can be added for mapping/SLAM
- Multiple sizes and add-ons
- **Typical use:** manipulation labs, pick/place experiments, ROS2 learning

#### Dreame D20 Pro as a mapping / control platform

Consumer vacuum robots with LDS mapping are a practical way to get floor plans and programmatic control without a full custom robot build.

| Factor | Details |
|--------|---------|
| **Cost** | Often ~$200–300 retail; compare with research/education kits at higher price points |
| **Availability** | Widely available through consumer channels |
| **Daily use** | Runs scheduled cleaning; dock, charging, and maintenance are productized |
| **LIDAR / mapping** | Room-mapping LDS; maps exportable to OBJ, PLY, Unity NavMesh, Blender-oriented workflows |
| **Software** | `python-miio` for control; MiIO map data; `vacuum-map-parser-dreame` for parsing |
| **Autonomy** | Scheduled runs, return-to-dock, self-empty (model-dependent) |
| **Limitation** | No onboard RGB camera on many units; add a fixed camera (e.g. Tapo C200) if you need vision |

#### Supported platforms

**Dreame D20 Pro (common starter for mapping):**

- **Dreame D20 Pro** — LIDAR vacuum with auto-mapping, zone cleaning, mop, and auto-empty station
- No Android device required; use `discover_dreame.py` and `get_dreame_token.py` in `scripts/`
- LIDAR maps exportable to OBJ, PLY, Unity NavMesh JSON, Blender Python scripts

**Also referenced in this project:**

- HTTP proxy bridging to simulators (e.g. Gazebo, VRChat) where configured
- FastMCP 3.4.4+
- Optional LLM sampling (`robotics_agentic_workflow`)
- **Dreame D20 Pro** — Vacuum API (MiIO protocols)
- **Yahboom ROSMASTER M1/X3/X3 Plus** — ROS2; example config on Raspberry Pi 5
- **Tdrone Mini** — PX4/ArduPilot educational drone with FPV camera
- **Philips Hue Bridge Pro** — Smart home hub with HomeAware RF movement detection
- **Moorebot Scout** — Legacy ROS1 wheeled robot (compatibility mode)
- **Unitree Go2/G1** — Quadrupeds (future hardware)
- **Nori A3** — 19-DOF bimanual mobile manipulator (2x 7+1 DOF arms, telescoping lift,
  differential-drive base); bridged via HTTP to a standalone `norirobotics-mcp` server
  (mirrors the Yahboom bridge pattern), mock-first by default. Live end-to-end verified
  2026-09-03 (connect, status, episode recording, e-stop, disconnect against a real running
  `norirobotics-mcp` instance).

#### Example budgets

**Dreame-first (~$200–300):**

- **Dreame D20 Pro** — LIDAR vacuum with mapping, zones, mop, auto-empty (model-dependent)
- Optional **Tapo C200** (~$25) for fixed camera coverage
- See [Dreame Setup Guide](hardware/DREAME_SETUP_GUIDE.md) for token and IP setup
- **Typical use:** floor plans, export pipelines, basic autonomy experiments

**ROS2 desktop + M1 (~$500–700):**

- **Yahboom ROSMASTER M1** (~$300) — Compact ROS2 base
- **Raspberry Pi 5 (4GB)** — Often bundled; otherwise ~$60–80
- **Typical use:** ROS2 coursework, arm/gripper add-ons

**Larger stack (~$800–1200):**

- **Dreame D20 Pro** + **Yahboom ROSMASTER X3/X3 Plus** + **Philips Hue Bridge Pro**
- **Typical use:** multiple agents, home/room context via Hue where used

### Required software

#### MCPB CLI (packaging `.mcpb` bundles)

```bash
npm install -g @anthropic-ai/mcpb
```

#### Unity 3D (virtual robotics)

```text
# Unity Hub: https://unity.com/download
# Editor 6000.2.14f1 or later — Installs tab → Add → Official releases
# Optional: Android Build Support
```

#### VRChat

```text
# Steam: https://store.steampowered.com/app/438100/VRChat/
# Or: https://hello.vrchat.com/
```

### Peer MCP servers (virtual / composed workflows)

Install and configure these when you follow the full virtual stack.

**1. Unity3D-MCP** (virtual robot control)

```bash
git clone https://github.com/sandraschi/unity3d-mcp.git
cd unity3d-mcp
uv pip install -e .
```

**2. OSC-MCP**

```bash
git clone https://github.com/sandraschi/osc-mcp.git
cd osc-mcp
uv pip install -e .
```

**3. VRChat-MCP**

```bash
git clone https://github.com/sandraschi/vrchat-mcp.git
cd vrchat-mcp
uv pip install -e .
```

**4. Blender-MCP** (requires Blender 4.0+)

```bash
git clone https://github.com/sandraschi/blender-mcp.git
cd blender-mcp
uv pip install -e .
```

**5. Avatar-MCP**

```bash
git clone https://github.com/sandraschi/avatar-mcp.git
cd avatar-mcp
uv pip install -e .
```

**6. Dreame D20 Pro** (vacuum; no Android required for token flow)

```bash
pip install python-miio
cd scripts
python discover_dreame.py
python get_dreame_token.py
# Optional:
pip install "python-miio[cli]"
miiocli discover
```

---

## Current State (2026-02-08, v0.2.0)

**Alpha with real hardware integration. FastMCP 2.14+ compliant.**

### What actually works

- **Dreame D20 Pro** — Vacuum control + LIDAR map retrieval + 3D export (OBJ, PLY, Unity, Blender)
- **Yahboom ROSMASTER** — roslibpy rosbridge client (connect, move, arm, gripper)
- **Elegoo** — Serial protocol communication
- **Gazebo Fuel** — Model browser: search, download, spawn via ROS services
- **MCP transport** — Dual stdio + HTTP, FastMCP 2.14+ with `ctx: Context` on tools
- **Webapp** — Live MCP data, sidebar navigation, 25+ pages, dark mode
- **13 portmanteau tools** — Honest simulation labels where applicable

### Mock / stub (labeled)

- Unitree Go2/G1 — `not_implemented`
- Drone flight — `simulated: True` in places
- LLM provider connection — placeholder behavior
- Physical-virtual sync — `simulated: True` where not wired

See [DEEP_ANALYSIS.md](../DEEP_ANALYSIS.md) for the mock audit.

## Overview

Unified control for **physical robots** (Dreame D20 Pro, Yahboom ROSMASTER, Elegoo), **simulated** (Gazebo), **virtual** (Unity/VRChat/Resonite), and **drones** (PX4/ArduPilot). Primary mapping path: Dreame D20 Pro with LIDAR export.

**Rough size:** ~12k lines Python, ~5k TypeScript/React, 13 portmanteau tools, 25+ webapp pages.

### Key features

- **Multi-robot coordination** — [MULTI_ROBOT_COORDINATION.md](project/MULTI_ROBOT_COORDINATION.md)
- **LIDAR map export** — Dreame maps for simulation and downstream processing
- **Collaborative mapping** — Multiple viewpoints where supported
- **Safety** — Status and planning hooks (verify timing on your hardware)
- **Physical and virtual** — Unity/VRChat when configured
- **Sensors** — LIDAR, cameras, IMUs, depth where drivers exist
- **Task routing** — By capability and environment model

#### Robot support

- **Physical**: Yahboom ROSMASTER (ROS2), Moorebot Scout (ROS1), Unitree Go2/G1/H1 (stubs), Dreame D20 Pro
- **Virtual**: Unity3D, VRChat, Resonite
- **Drones**: PX4/ArduPilot, MAVLink, streaming paths (implementation-dependent)
- **ROS**: Bridge for ROS1/ROS2 peers
- **World Labs Marble/Chisel**: Environment import where enabled
- **Transport**: stdio (MCP) + HTTP (FastAPI)
- **Composition**: `osc-mcp`, `unity3d-mcp`, `vrchat-mcp`, `avatar-mcp`, `blender-mcp`, `gimp-mcp` (some disabled by default)
- **Tools**: `robotics_system`, `robot_control`, `robot_behavior`, `robot_manufacturing`, `robot_virtual`, `robot_model_tools`, `vbot_crud`, `drone_control`, `dreame_control`, `gazebo_models`, `workflow_management`, `virtual_robotics`, `robot_navigation`

## Documentation (topic index)

- [Setup Prerequisites](guides/SETUP_PREREQUISITES.md) — Unity, VRChat, peer MCP servers
- [AI Research Workflow](guides/AI_RESEARCH_WORKFLOW.md)
- [Vienna Technical Museum Makerspace](project/VIENNA_TECHNICAL_MUSEUM_MAKERSPACE.md)
- [Progress Report](project/PROGRESS_REPORT.md)
- **Secondary app routing (`/apps`)** — Bundled web apps
- **Virtual robotics pipeline (`/vbot-ecosystem`)** — VRChat / Unity mapping
- [Dreame Setup Guide](hardware/DREAME_SETUP_GUIDE.md)
- [Hue Bridge Pro Setup](hardware/HUE_BRIDGE_PRO_SETUP.md)
- [Multi-Robot Coordination](project/MULTI_ROBOT_COORDINATION.md)
- [Unity Vbot Instantiation Guide](software/UNITY_VBOT_INSTANTIATION.md)

## Installation

Complete [Setup Prerequisites](guides/SETUP_PREREQUISITES.md) if you use Unity3D, VRChat, and the listed MCP servers.

### Prerequisites

- [uv](https://docs.astral.sh/uv/) (recommended)
- Python 3.12+

### Run with `uvx`

```bash
uvx robotics-mcp
```

### Claude Desktop integration

Add to `claude_desktop_config.json`:

```json
"mcpServers": {
  "robotics-mcp": {
    "command": "uv",
    "args": ["--directory", "D:/Dev/repos/robotics-mcp", "run", "robotics-mcp"]
  }
}
```

### From PyPI

```bash
pip install robotics-mcp
```

### From GitHub releases

```bash
pip install https://github.com/sandraschi/robotics-mcp/releases/download/v1.0.1b2/robotics_mcp-1.0.1b2-py3-none-any.whl
```

Or:

```bash
pip install git+https://github.com/sandraschi/robotics-mcp.git
```

### Development install

```bash
git clone https://github.com/sandraschi/robotics-mcp.git
cd robotics-mcp
pip install -e ".[dev]"
```

## Packaging and distribution

Package with `@anthropic-ai/mcpb`:

```bash
mcpb pack . dist/robotics-mcp.mcpb
```

### Cursor MCP integration

1. `pip install -e ".[dev]"`
2. Register the server via `mcpb.json` or your client’s MCP config
3. Enable in Cursor; server starts with the IDE when configured
4. For Unity/VRChat flows, finish [Setup Prerequisites](guides/SETUP_PREREQUISITES.md) first

**Exposed tools (13):**

- `robotics_system` — Help, status, list robots
- `robot_control` — Physical/virtual control (Dreame, Yahboom, Elegoo, Hue, virtual)
- `robot_behavior` — Behavior and manipulation
- `robot_manufacturing` — 3D printing, CNC, laser (where implemented)
- `robot_virtual` — Virtual environments
- `robot_model_tools` — Models: create, convert, optimize
- `vbot_crud` — Virtual robot CRUD
- `drone_control` — Flight primitives
- `dreame_control` — Dreame vacuum + map export
- `gazebo_models` — Gazebo Fuel
- `workflow_management` — Orchestration
- `virtual_robotics` — Legacy virtual ops
- `robot_navigation` — Planning / SLAM hooks

### Optional composed MCP servers

Default wiring (verify in code and config):

- `osc-mcp` — on
- `unity3d-mcp` — on (timeouts/fallbacks if Unity is down)
- `vrchat-mcp` — off
- `avatar-mcp` — off
- `blender-mcp` — off
- `gimp-mcp` — off

~30s load timeouts, retries with backoff, mock fallbacks when peers are missing.

### Configuration (optional)

Default: no config file. Advanced: `~/.robotics-mcp/config.yaml`:

```yaml
robotics:
  moorebot_scout:
    enabled: false
    robot_id: "scout_01"
    ip_address: "192.168.1.100"
    port: 9090
    mock_mode: true
  virtual:
    enabled: true
    platform: "unity"
server:
  enable_http: true
  http_port: 12230
  log_level: "INFO"
```

**MCP integration** (when re-enabling composed servers):

```yaml
mcp_integration:
  osc_mcp:
    enabled: true
    prefix: "osc"
  unity3d_mcp:
    enabled: true
    prefix: "unity"
  vrchat_mcp:
    enabled: true
    prefix: "vrchat"
  avatar_mcp:
    enabled: true
    prefix: "avatar"
  blender_mcp:
    enabled: true
    prefix: "blender"
  gimp_mcp:
    enabled: true
    prefix: "gimp"
```

### Running the server

Primary: MCP client config + `mcpb.json`.

**Manual / dev:**

```bash
python -m robotics_mcp --mode stdio
python -m robotics_mcp --mode http --port 12230
python -m robotics_mcp --mode dual --port 12230
```

Windows (watchfiles wrapper):

```text
.\scripts\run-with-watchfiles.ps1
```

## More documentation

- [ROS Fundamentals](software/ROS_FUNDAMENTALS.md)
- [LiDAR Guide](hardware/LIDAR_GUIDE.md)
- [Tiny Controllers Guide](hardware/TINY_CONTROLLERS_GUIDE.md)
- [Pyroelectric Sensors Guide](hardware/PYROELECTRIC_SENSORS_GUIDE.md)
- [Component Reuse Hacks](project/COMPONENT_REUSE_HACKS.md)
- [World Labs Unity Integration Fix](software/WORLDLABS_UNITY_INTEGRATION_FIX.md)
- [Import Nekomimi-chan VRM Guide](models/IMPORT_NEKOMIMI_VRM_GUIDE.md)
- [Blender VRM Workflow for Robotics](models/BLENDER_VRM_WORKFLOW_ROBOTICS.md)
- [VRM Tools Alternatives](models/VRM_TOOLS_ALTERNATIVES.md)
- [Comprehensive Project Notes](project/COMPREHENSIVE_NOTES.md)
- [VRM vs Robot Models](models/VRM_VS_ROBOT_MODELS.md)
- [Unity Vbot Instantiation Guide](software/UNITY_VBOT_INSTANTIATION.md)
- [Implementation Plan](../PLAN.md)
- [Quick Start: VRChat](guides/QUICK_START_VRCHAT.md)
- [ROS 1 local setup](software/ROS1_LOCAL_SETUP.md)
- [VRChat Integration Guide](guides/VRChat_INTEGRATION.md)
- [VRChat Scout Setup](guides/VRCHAT_SCOUT_SETUP.md)
- [Unity Setup Guide](software/UNITY_SETUP_GUIDE.md)

### Production and deployment

- [Watchfiles Crash Protection](../WATCHFILES_README.md)
- [Systemd service](../robotics-mcp-watchfiles.service)
- [PowerShell script](../scripts/run-with-watchfiles.ps1)

### Robot integration

- [Dreame D20 Pro Setup](hardware/DREAME_SETUP_GUIDE.md)
- [Hue Bridge Pro Setup](hardware/HUE_BRIDGE_PRO_SETUP.md)
- [Multi-Robot Coordination](project/MULTI_ROBOT_COORDINATION.md)
- [Yahboom Integration](hardware/YAHBOOM_INTEGRATION_SUMMARY.md)

### Development and testing

- [Setup Prerequisites](guides/SETUP_PREREQUISITES.md)
- [Quick Start VRChat](guides/QUICK_START_VRCHAT.md)
- [Unity Setup Guide](software/UNITY_SETUP_GUIDE.md)
- [Comprehensive Notes](project/COMPREHENSIVE_NOTES.md)

## Testing

About 21 test files and ~2,600 lines covering the main tool surface (counts approximate).

```bash
pytest
pytest tests/unit
pytest tests/integration
pytest --cov=robotics_mcp --cov-report=html
```

Or:

```text
.\scripts\run-tests.ps1
```

## Development

### Project structure

```text
robotics-mcp/
├── src/robotics_mcp/
│   ├── server.py
│   ├── clients/
│   ├── integrations/
│   ├── tools/
│   └── utils/
├── tests/
├── docs/
├── scripts/
└── mcpb/
```

### Code quality

```bash
black src/ tests/
ruff check src/ tests/
mypy src/
```

## Troubleshooting

### Cursor: server not listed

1. Cursor → Settings → Features → Model Context Protocol
2. Add server using `mcpb.json`
3. Confirm **Healthy**; restart Cursor if needed

**Smoke test:**

```bash
python -c "from robotics_mcp.server import RoboticsMCP; RoboticsMCP(); print('SUCCESS')"
```

### Server won’t start

- Python 3.10+
- `pip install -e ".[dev]"`
- Check client logs under `%AppData%\Cursor\logs\` on Windows

### Tools missing

- MCP server enabled in the IDE
- Inspect server stderr/logs

### Unity integration

- Unity Editor running
- Project path correct
- `unity3d-mcp` healthy

---

**Status:** Alpha v0.2.0 (2026-02-08) — Dreame, Yahboom, Gazebo Fuel, webapp; composed servers partially disabled. See [DEEP_ANALYSIS.md](../DEEP_ANALYSIS.md).
