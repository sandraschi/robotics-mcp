# Robotics-MCP Deep Analysis

**Date**: 2026-02-08
**Analyst**: Automated deep code review
**Scope**: Full repository - MCP server + webapp

---

## Table of Contents

1. [Executive Summary](#1-executive-summary)
2. [Architecture Analysis](#2-architecture-analysis)
3. [Mock & Gaslight Audit](#3-mock--gaslight-audit)
4. [Technical Debt Inventory](#4-technical-debt-inventory)
5. [Bug Report](#5-bug-report)
6. [Feature Analysis](#6-feature-analysis)
7. [SOTA Gap Analysis](#7-sota-gap-analysis)
8. [Improvement Plan](#8-improvement-plan)
9. [Implementation Priorities](#9-implementation-priorities)

---

## 1. Executive Summary

**robotics-mcp** is an ambitious FastMCP 2.14.3 server that aims to unify physical robot control (Dreame vacuums, Yahboom ROS robots, Elegoo cars, Unitree quadrupeds, drones) with virtual robot control (Unity3D, VRChat) through a single MCP interface. It includes a full React/Next.js webapp with real-time WebSocket communication, LLM integration, workflow management, and camera feeds.

### Verdict

The **architecture is sound** - portmanteau tool pattern, dual transport (stdio+HTTP), MCP server composition, structured logging. The **vision is world-class** - no other robotics MCP server attempts this scope. But the **implementation is 60-70% mock/stub**, with critical path integrations unimplemented. The webapp duplicates MCP server state management instead of being a thin client.

### Scores (1-10)

| Dimension | Score | Notes |
|---|---|---|
| Architecture Design | 8/10 | Portmanteau pattern, clean separation, good error handling |
| Code Quality | 6/10 | Consistent style, good docstrings, but lots of dead code |
| Feature Completeness | 3/10 | Most hardware integrations are stubs |
| Test Coverage | 4/10 | Tests exist but mostly test mock paths |
| Production Readiness | 2/10 | Cannot control any real robot without additional work |
| Documentation | 7/10 | Extensive docs, but many describe aspirational features |
| Webapp Quality | 5/10 | Full UI exists, but disconnected from real data |

---

## 2. Architecture Analysis

### 2.1 Strengths

**Portmanteau Tool Pattern**: Correctly consolidates 30+ potential tools into ~11 registered tools. This prevents tool explosion and follows FastMCP SOTA practices. Each tool class (`RobotControlTool`, `RobotVirtualTool`, etc.) owns its registration and handler dispatch.

**Dual Transport**: Proper stdio mode with binary mode on Windows (`msvcrt.setmode`), `DevNullStdout` to prevent protocol corruption, and HTTP mode via FastAPI. This is production-quality transport handling.

**MCP Server Composition**: Mounting external MCP servers (osc-mcp, unity3d-mcp) for internal use without exposing their tools. Correct pattern to prevent tool namespace pollution.

**Structured Logging**: Uses `structlog` with JSON rendering, stderr-only output. Proper for MCP protocol where stdout is reserved for JSON-RPC.

**Config-Driven Robot Registration**: Robots loaded from YAML config at startup, supporting both physical and virtual robot definitions.

**Error Handling Framework**: `format_error_response`, `format_success_response`, `handle_tool_error` provide consistent response structure across all tools.

### 2.2 Weaknesses

**God-Object robot_control.py**: At 1374 lines, this single tool handler manages Yahboom, Dreame, Hue, Elegoo, Gazebo, physical, and virtual robots. Each handler is 100-200 lines of if/elif chains. Should be decomposed into strategy pattern or separate handler classes.

**Missing `ctx: Context` Parameter**: Per FastMCP 2.14 standards, all `@mcp.tool()` functions MUST accept `ctx: Context` for progress reporting and structured logging. None of the tools use it. This is a compliance gap.

**Lifespan Disabled**: `server_lifespan` is a stub (`return None`). The comment says "TEMPORARILY DISABLE LIFESPAN FOR DEBUGGING" - this means startup/shutdown hooks are not functional.

**asyncio.run() in main()**: Calls `asyncio.run(server.initialize_async())` then `server.run()`. If run mode is stdio, `self.mcp.run()` creates its own event loop. This double-loop pattern may cause issues. FastMCP's run() should handle the full lifecycle.

**Webapp is a Parallel Universe**: The `web/backend/main.py` (1800+ lines) creates its own `RobotState`, `LLMState`, physics simulation, and robot management completely independent of the MCP server. It communicates with the MCP server via HTTP (localhost:12230) but also maintains its own mock robot state. This is architectural duplication.

**No Authentication**: Both the MCP server HTTP API (port 12230) and the webapp API (port 8354) have `allow_origins=["*"]` with no authentication. The webapp CORS comment says "Allow all tailnet members" but there's no actual tailnet enforcement.

---

## 3. Mock & Gaslight Audit

This is the critical section. The codebase has **systemic mock contamination** - functions that claim to do real operations but actually return fabricated data.

### 3.1 Explicit Mocks (Labeled)

These are honest about being mocks:

| Location | What It Mocks | Labeled? |
|---|---|---|
| `utils/simulated_data.py` | LiDAR scans, robot status, sensor data, maps | YES - "simulated" prefix |
| `utils/mock_data.py` | Re-exports simulated_data with mock_ prefix | YES - deprecated |
| `clients/yahboom_client.py` | All operations when `mock_mode=True` | YES - `"mock": True` in responses |
| `clients/gazebo_client.py` | All operations when `mock_mode=True` | YES - `"mock": True` in responses |
| `web/backend/main.py:243` | `RobotState.__init__` hardcodes vbot_scout_mini | YES - comment says "mock data" |
| `web/backend/main.py:1735` | Mock Dreame robot added on startup | YES - "Demo" in name |

### 3.2 Gaslighting Mocks (Unlabeled/Misleading)

These are the dangerous ones - they **claim success but do nothing real**:

| Location | Gaslight | Severity |
|---|---|---|
| `robot_control.py:349-357` | `_handle_physical_robot` returns success with `"mock": True` for ALL physical robot actions. Every Unitree Go2/G1, generic Scout command returns "command sent" when nothing happened. | **CRITICAL** |
| `robot_control.py:537-558` | `_handle_yahboom_robot` ai_query returns fake confidence (0.95), fake processing time (150ms), and canned text responses. No actual LLM or vision processing occurs. | **HIGH** |
| `robot_control.py:509-525` | `home_patrol` returns hardcoded waypoints (living room, kitchen, bedroom) with no actual navigation. The response implies the patrol has started. | **HIGH** |
| `robot_virtual.py:570-573` | `_handle_test_navigation` returns "Navigation test completed" when nothing was tested. | **MEDIUM** |
| `robot_virtual.py:576-581` | `_handle_sync_with_physical` returns "Virtual robot synced" when no sync occurred. | **MEDIUM** |
| `robot_virtual.py:476-484` | `_handle_load_environment` returns success "loaded (mock)" when Unity MCP unavailable, but the "success" status may mislead callers. | **MEDIUM** |
| `robot_virtual.py:531-534` | `_handle_get_lidar` returns empty `{"points": []}` labeled as "mock" but with success status. | **LOW** |
| `web/backend/main.py:678-716` | `simulate_robot_physics` generates fake IMU, odometry, battery data in an infinite loop and broadcasts to WebSocket clients as if real. | **CRITICAL** |
| `web/backend/main.py:975` | `connect_llm_provider` does `await asyncio.sleep(1.0)` then sets status to "connected" without actually connecting. | **HIGH** |
| `yahboom_client.py:78-79` | Non-mock `connect()` does `await asyncio.sleep(0.1)` and sets `self.connected = True` without actually connecting to anything. | **CRITICAL** |
| `yahboom_client.py:176` | Non-mock `move()` returns `{"error": "ROS integration not implemented"}` - at least honest, but dead code path. | **LOW** |

### 3.3 Mock Contamination Summary

- **Total tool handler methods**: ~25
- **Fully implemented (real hardware)**: 3 (Elegoo serial, partial Dreame via python-miio)
- **Mock-only**: ~18
- **Partially real**: ~4 (Dreame actions that call `get_dreame_client`)

**Conclusion**: ~72% of advertised functionality is mock/stub. The Dreame integration is the most real, but even that relies on `python-miio` which requires cloud token extraction. The Elegoo client is the only one with actual hardware protocol implementation (serial).

---

## 4. Technical Debt Inventory

### 4.1 Critical Debt

1. **No real ROS2 bridge integration**: Despite extensive ROS2 topic definitions in `yahboom_client.py`, there is zero actual `roslibpy` usage. Every ROS operation is a TODO stub. The existing open-source solutions (`wise-vision/ros2_mcp`, `robotmcp/ros-mcp-server`) already solve this.

2. **Webapp duplicates server state**: `web/backend/main.py` maintains its own robot registry, physics simulation, and LLM state instead of being a thin client to the MCP server. Changes to one don't propagate to the other.

3. **Disabled server lifespan**: The `@asynccontextmanager` lifespan is commented out, meaning no proper startup/shutdown lifecycle management.

4. **Missing `ctx: Context`**: Zero MCP tools accept the FastMCP Context parameter. This means no progress reporting, no structured tool-level logging, and non-compliance with FastMCP 2.14.

5. **`pyserial` is not async**: `ElegooClient` uses synchronous `serial.Serial` with a background thread for sensor reading. This blocks the event loop during `connect()` and `set_motors()` calls via `self.serial.write()`.

### 4.2 High Debt

6. **Duplicate route definitions**: `web/backend/main.py` defines `@app.get("/api/llm/models")` twice (lines 910 and 1126). The second one silently overwrites the first.

7. **`startup_event` defined inside a route handler**: Lines 1723-1780, `@app.on_event("startup")` is defined inside the `get_apps_status()` function body. This is a scoping bug - the event handler will never fire because it's defined too late.

8. **Dead import `call_mounted_server_tool`**: Imported in `robot_control.py` but many handlers don't use the mounted server pattern; they directly instantiate clients.

9. **No dependency injection**: Tool classes take raw `mcp`, `state_manager`, `mounted_servers` as constructor args. No DI container, no testable interfaces.

10. **`numpy` dependency for simulated data**: `simulated_data.py` imports `numpy` for generating fake LiDAR data. This pulls in a heavy dependency just for mocks.

### 4.3 Medium Debt

11. **Inconsistent response format**: Some handlers return `format_success_response()` dicts, others return raw `{"status": "success", ...}` dicts. The virtual robot handler (`_handle_virtual_robot`) mixes both.

12. **Unity path manipulation**: `_load_unity_server` does `sys.path.insert(0, ...)` to find the unity3d-mcp package. This is fragile and assumes a specific sibling directory layout.

13. **Hardcoded ports**: MCP server on 12230, webapp on 8354, webapp assumes MCP at 12230, etc. No centralized port configuration.

14. **`web/` directory structure confusion**: Contains both `script.js` (vanilla JS), `vbot.js`, `vbot.html` AND a full Next.js/React app in `src/`. Two different frontend architectures coexist.

15. **`package.json` in both root and web/**: Root has Node.js dependencies (`robotics-mcp.js` bin entry), web/ has React/Next.js deps. Unclear which is the canonical frontend.

---

## 5. Bug Report

### 5.1 Critical Bugs

**BUG-001**: `web/backend/main.py` line 1323 - `request.debug_mode` attribute access on `WorkflowExecuteRequest` which has no `debug_mode` field. This will crash with `AttributeError` when any workflow is executed.

```python
# Line 1323: WorkflowExecuteRequest only has 'variables' field
result = await workflow_service.execute_workflow(
    workflow_id, request.variables or {}, debug_mode=request.debug_mode  # BUG
)
```

**BUG-002**: `web/backend/main.py` lines 1723-1780 - `@app.on_event("startup")` and `@app.on_event("shutdown")` are defined inside the `get_apps_status()` route handler function. They will never register because they're local to the function scope. The physics simulation, robot loading, LLM init, and camera init never start.

**BUG-003**: `robot_control.py` line 136 - Duplicate line in docstring:
```python
# Line 136: appears twice
- Hue HomeAware: "hue_get_movement_events", "hue_get_sensor_status", "hue_get_movement_zones"
- Hue HomeAware: "hue_get_movement_events", "hue_get_sensor_status", "hue_get_movement_zones"
```

### 5.2 High Bugs

**BUG-004**: `robot_control.py` - The `action` Literal type includes `"ai_query"` nowhere in the list, but the Yahboom handler checks for `action == "ai_query"` at line 527. Since `"ai_query"` is not in the Literal union, FastMCP schema validation will reject it.

**BUG-005**: `server.py` line 412 - `list_tools` accesses `self.mcp._tools` which is a private internal of FastMCP. This will break on FastMCP version updates. Use `self.mcp.list_tools()` or the public API.

**BUG-006**: `server.py` line 680 - `_load_unity_server` uses `await asyncio.get_event_loop().run_in_executor(None, unity_server.app.list_tools)` - `get_event_loop()` is deprecated in Python 3.12+. Use `asyncio.get_running_loop()`.

### 5.3 Medium Bugs

**BUG-007**: `robot_control.py` - Dreame handler checks for `action == "go_to"` and `action == "start_cleaning"` etc. but these are NOT in the `action` Literal type. The Literal includes `"start_auto_empty"`, `"stop_auto_empty"` etc. but the handler checks different action strings. Dead code paths.

**BUG-008**: `elegoo_client.py` line 20 - Default serial port is `/dev/ttyUSB0` (Linux), but workspace rules state this is a Windows-primary project. Should default to `COM3` or auto-detect.

**BUG-009**: `web/backend/main.py` line 308 - `load_physical_robots` connects to `http://localhost:8888/api/robots` but the MCP server HTTP API runs on port 12230. Wrong port.

### 5.4 Fixed Since Deep Analysis (2026-02-08)

**BUG-OSC-001** (HIGH): All `osc-mcp` mounted server calls in `virtual_robotics.py`, `vbot_crud.py`, and `robot_animation.py` used `Client(self.mcp).call_tool("osc_send_osc", ...)` which targets the main robotics-mcp server, not the mounted OSC server. The OSC tools are never exposed on the main server, so these were dead code paths. **Fixed**: Replaced with `call_mounted_server_tool(self.mounted_servers, "osc", "osc_send_osc", {...})`.

**BUG-UNITY-001** (HIGH): Same pattern for all Unity3D mounted server calls. `virtual_robotics.py` had 4 broken blocks (robot spawn, environment loading, virtual LiDAR, set scale) and `robot_animation.py` had 1 block with 6 action branches (animate_wheels, animate_movement, set_pose, play_animation, stop_animation, get_animation_state). All used `Client(self.mcp).call_tool(...)` instead of the mounted server helper. **Fixed**: Replaced with `call_mounted_server_tool(self.mounted_servers, "unity", ...)`. Removed dead `from fastmcp import Client` imports from all 3 files.

---

## 6. Feature Analysis

### 6.1 Implemented (Real)

| Feature | Status | Notes |
|---|---|---|
| MCP stdio transport | WORKING | Binary mode, DevNullStdout, proper stderr logging |
| MCP HTTP transport | WORKING | FastAPI router with robot CRUD and tool proxy |
| Robot state management | WORKING | In-memory registry with register/unregister/list |
| Config YAML loading | WORKING | Physical and virtual robot config parsing |
| Elegoo serial protocol | WORKING | Real serial communication, sensor parsing |
| Dreame vacuum (partial) | PARTIAL | Uses python-miio, requires cloud token setup |
| Structured error responses | WORKING | Consistent error/success format across tools |
| MCP server composition | WORKING | Mount and call external MCP servers |
| Workflow storage (SQLite) | WORKING | CRUD for workflow definitions |

### 6.2 Implemented (Mock Only)

| Feature | Status | What's Missing |
|---|---|---|
| Yahboom ROS2 control | MOCK | No roslibpy, no actual ROS bridge connection |
| Unitree Go2/G1 control | MOCK | No Unitree SDK integration |
| Gazebo simulation | MOCK | No actual Gazebo connection via rosbridge |
| Virtual robot spawn (Unity) | MOCK | Depends on unity3d-mcp which may not be running |
| Virtual LiDAR | MOCK | Returns empty data when Unity unavailable |
| Robot navigation | MOCK | No path planning, no SLAM |
| Drone control | EXISTS | Tool registered but handler analysis needed |
| AI query (Yahboom vision) | MOCK | Canned responses, no real LLM/vision |
| Webapp physics sim | MOCK | Fake IMU/odometry in infinite loop |
| LLM provider connection | MOCK | sleep(1.0) + "connected" |

### 6.3 Not Implemented

| Feature | Promised In | Status |
|---|---|---|
| ROS2 topic pub/sub | README, docs | NOT STARTED |
| ROS2 service calling | docs | NOT STARTED |
| SLAM/mapping | docs | NOT STARTED |
| Object detection | docs | NOT STARTED |
| Multi-robot coordination | config schema | NOT STARTED |
| Physical-virtual sync | tool exists | STUB ONLY |
| Gaussian Splatting integration | scripts exist | SCRIPT ONLY |
| Raspberry Pi deployment | docs | NOT STARTED |
| Docker deployment (ROS) | docker/ dir | Dockerfiles exist, untested |

---

## 7. SOTA Gap Analysis

### 7.1 vs. Existing Robotics MCP Servers

| Feature | robotics-mcp | ros-mcp-server (robotmcp) | ros2_mcp (WiseVision) |
|---|---|---|---|
| ROS2 topic pub/sub | MOCK | REAL | REAL |
| ROS2 service calls | NO | REAL | REAL |
| Auto-discovery | NO | YES | YES |
| Action client | NO | PLANNED | YES |
| Multi-robot | DESIGNED | NO | NO |
| Virtual robots | DESIGNED | NO | NO |
| Non-ROS devices | YES (Dreame, Elegoo) | NO | NO |
| LLM integration | DESIGNED | NO | NO |
| Webapp | YES (full) | NO | NO |

**Key Insight**: robotics-mcp has the broadest *vision* but the narrowest *implementation*. The competitors have narrow scope but actually work with real ROS2 systems. The path forward is to **integrate** `roslibpy` or leverage `ros-mcp-server` patterns rather than reimplementing ROS2 bridge from scratch.

### 7.2 vs. python-miio SOTA for Dreame

The Dreame integration in `dreame_client.py` wraps `python-miio`, which is the correct library. However:
- No token management UI or CLI
- No map parsing (the `vacuum-map-parser-dreame` package exists)
- No event stream subscription for real-time status

### 7.3 vs. Yahboom Official ROS2 Packages

Yahboom provides official ROS2 packages (`YahboomTechnology/ROSMASTER-R2`, `automaticaddison/yahboom_rosmaster`) with launch files, URDF models, and hardware drivers. The robotics-mcp Yahboom client reimplements nothing of this - it defines ROS2 topics but never connects to rosbridge.

---

## 8. Improvement Plan

### 8.1 Phase 1: De-Mock (2-3 weeks)

**Goal**: Make at least ONE robot type work end-to-end with real hardware.

1. **Implement roslibpy bridge**: Add actual `roslibpy.Ros` connection in `yahboom_client.py`. Subscribe to `/odom`, `/scan`, `/battery`. Publish to `/cmd_vel`. This is ~100 lines of real code replacing ~200 lines of mocks.

2. **Fix Dreame token flow**: Add CLI command to extract Dreame cloud token (the `get_dreame_token.py` script exists but isn't integrated). Store token in config.yaml. Make `dreame_client.py` use real `DreameVacuum` from python-miio.

3. **Add `ctx: Context`**: Every `@mcp.tool()` must accept `ctx: Context` parameter. Use `ctx.info()` for logging and `ctx.report_progress()` for long operations.

4. **Fix startup bug**: Move `@app.on_event("startup")` out of the route handler in `web/backend/main.py`.

5. **Fix WorkflowExecuteRequest**: Add `debug_mode: bool = False` field or remove the reference.

### 8.2 Phase 2: Unify Architecture (2-3 weeks)

**Goal**: Make the webapp a proper thin client of the MCP server.

1. **Remove webapp robot state**: Delete `RobotState` class from `web/backend/main.py`. Replace all robot state queries with HTTP calls to the MCP server (port 12230).

2. **Remove webapp physics sim**: The MCP server should own robot state, including simulated physics when in mock mode. The webapp should only display data received via WebSocket.

3. **Add WebSocket to MCP server**: The MCP server's HTTP API needs a WebSocket endpoint for real-time state push. Currently only the webapp has WebSocket support.

4. **Consolidate LLM state**: The webapp has its own `LLMProviderManager` and `LLMState`. These should live in the MCP server as tools/resources.

### 8.3 Phase 3: World-Beating Features (4-6 weeks)

**Goal**: Differentiate from ros-mcp-server and ros2_mcp.

1. **Multi-robot coordination tool**: Implement the `coordination` config section. Allow sequential/parallel task execution across multiple robots. This is unique to robotics-mcp.

2. **Physical-virtual sync (real)**: When a Dreame vacuum moves, update its virtual twin in Unity. Use the existing MCP server composition to call unity3d-mcp with real position data.

3. **Dreame map visualization**: Use `vacuum-map-parser-dreame` to parse SLAM maps and display in the webapp. Overlay robot position, cleaning zones, restricted areas.

4. **Safety layer**: Implement velocity clamping, collision detection thresholds, emergency stop propagation across all robot types. This is absent from all competing solutions.

5. **ROS2 auto-discovery**: Mirror what WiseVision's ros2_mcp does - scan available topics/services and expose them dynamically.

### 8.4 Phase 4: Production Hardening (2-3 weeks)

1. **Authentication**: Add API key auth to both MCP HTTP API and webapp.
2. **Rate limiting**: Prevent command flooding to physical robots.
3. **Reconnection logic**: Auto-reconnect to ROS bridge, Dreame, serial ports on failure.
4. **Health monitoring**: Watchdog for each robot connection with automatic degradation.
5. **Logging to file**: Add file handler for audit trail of all robot commands.
6. **Tests with hardware mocking**: Use `pytest-mock` to mock `roslibpy.Ros` and `serial.Serial` at the transport level, not at the application logic level.

---

## 9. Implementation Priorities

### Immediate (This Week)

| # | Task | Impact | Effort |
|---|---|---|---|
| 1 | Fix BUG-001 (WorkflowExecuteRequest.debug_mode) | Crash prevention | 5 min |
| 2 | Fix BUG-002 (startup/shutdown event scoping) | Webapp won't initialize | 15 min |
| 3 | Fix BUG-009 (wrong port 8888 -> 12230) | Webapp can't reach MCP | 5 min |
| 4 | Add `ctx: Context` to all tools | FastMCP compliance | 2 hours |
| 5 | Re-enable server lifespan | Proper lifecycle | 30 min |

### Short Term (2 weeks)

| # | Task | Impact | Effort |
|---|---|---|---|
| 6 | Implement roslibpy in yahboom_client | First real ROS2 robot | 2 days |
| 7 | Integrate Dreame token management | Real vacuum control | 1 day |
| 8 | Remove webapp duplicate state | Architecture cleanup | 2 days |
| 9 | Add WebSocket to MCP HTTP API | Real-time updates | 1 day |
| 10 | Fix BUG-004 (missing ai_query in Literal) | Tool schema correctness | 15 min |

### Medium Term (1 month)

| # | Task | Impact | Effort |
|---|---|---|---|
| 11 | Multi-robot coordination | Unique differentiator | 1 week |
| 12 | Dreame map visualization | Killer feature | 3 days |
| 13 | Safety layer implementation | Production requirement | 3 days |
| 14 | Authentication + rate limiting | Security | 2 days |
| 15 | Replace asyncio.sleep() mocks with real connections | Credibility | 1 week |

---

## Appendix A: File-Level Audit

### MCP Server (`src/robotics_mcp/`)

| File | Lines | Mock % | Verdict |
|---|---|---|---|
| `server.py` | 875 | 10% | GOOD - mostly real infrastructure |
| `tools/robot_control.py` | 1374 | 70% | NEEDS WORK - too many mock handlers |
| `tools/robot_virtual.py` | 689 | 60% | NEEDS WORK - platform calls are mock fallbacks |
| `tools/robot_behavior.py` | ~500 | 80% | MOSTLY MOCK |
| `tools/robot_model_tools.py` | ~400 | 40% | PARTIAL - SPZ converter may work |
| `tools/vbot_crud.py` | ~300 | 50% | PARTIAL - state management works |
| `tools/dreame_control.py` | ~300 | 30% | BEST - uses python-miio |
| `tools/drone_control.py` | ~200 | 90% | MOSTLY MOCK |
| `clients/yahboom_client.py` | 295 | 95% | CRITICAL - all operations are mock/stub |
| `clients/elegoo_client.py` | 226 | 0% | REAL - actual serial protocol |
| `clients/gazebo_client.py` | ~200 | 90% | MOCK - no rosbridge connection |
| `utils/simulated_data.py` | 120 | 100% | BY DESIGN - labeled simulated data |
| `utils/state_manager.py` | ~200 | 0% | REAL - in-memory state management |
| `utils/error_handler.py` | ~100 | 0% | REAL - response formatting |
| `utils/config_loader.py` | ~100 | 0% | REAL - YAML config parsing |
| `services/workflow_*.py` | ~400 | 20% | MOSTLY REAL - SQLite storage |

### Webapp (`web/`)

| File | Lines | Mock % | Verdict |
|---|---|---|---|
| `backend/main.py` | 1804 | 40% | MIXED - real API + fake physics/state |
| `backend/mcp_client.py` | ~200 | 20% | MOSTLY REAL - HTTP client to MCP |
| `backend/llm_service.py` | ~300 | 30% | PARTIAL - Ollama/LMStudio integration |
| `backend/camera_integration.py` | ~200 | ? | NEEDS AUDIT |
| `backend/workflow_service.py` | ~300 | 20% | MOSTLY REAL |
| `src/services/mcpService.ts` | 305 | 0% | REAL - typed HTTP client |
| `src/components/*.tsx` | ~3000+ | 10% | REAL UI with mock data display |

### Total Repository

- **Python source**: ~8,000 lines in `src/`, ~3,000 in `web/backend/`
- **TypeScript/React**: ~5,000 lines in `web/src/`
- **Mock percentage overall**: ~55%
- **Documentation**: ~40 markdown files across `docs/`

---

## Appendix B: Competing Solutions Reference

| Solution | URL | Stars | Real ROS2 | MCP |
|---|---|---|---|---|
| ros-mcp-server | github.com/robotmcp/ros-mcp-server | 992 | YES | YES |
| ros2_mcp | github.com/wise-vision/ros2_mcp | ~100 | YES | YES |
| mcp_server_ros_2 | github.com/wise-vision/mcp_server_ros_2 | ~50 | YES | YES |
| python-miio | github.com/rytilahti/python-miio | 3600+ | NO | NO |
| yahboom_rosmaster | github.com/automaticaddison/yahboom_rosmaster | 33 | YES | NO |

**Strategy**: Don't compete with ros-mcp-server on pure ROS2 integration. Instead, **compose** it as a mounted server (like osc-mcp is mounted today) and focus on the unique value: multi-robot + virtual-physical sync + non-ROS devices + webapp.

---

## 10. Fix Log (2026-02-08)

All critical and high-priority bugs from this analysis have been fixed in a single session:

### Bugs Fixed

| ID | Severity | Description | File(s) |
|---|---|---|---|
| BUG-001 | CRITICAL | `WorkflowExecuteRequest.debug_mode` AttributeError | `web/backend/main.py` |
| BUG-002 | CRITICAL | startup/shutdown events scoped inside route handler, never registered | `web/backend/main.py` |
| BUG-003 | MEDIUM | Duplicate Hue docstring line | `robot_control.py` |
| BUG-004 | HIGH | `ai_query` missing from action Literal type | `robot_control.py` |
| BUG-005 | HIGH | Private `_tools` dict access in server.py | `server.py` |
| BUG-006 | HIGH | Deprecated `asyncio.get_event_loop()` | `server.py` |
| BUG-008 | MEDIUM | Linux-default serial port on Windows project | `elegoo_client.py` |
| BUG-009 | MEDIUM | Webapp connecting to wrong MCP port (8888 vs 12230) | `web/backend/main.py` |

### Structural Fixes

| Fix | Description | Files |
|---|---|---|
| ctx: Context | Added FastMCP 2.14 `ctx: Context` parameter to all 10 portmanteau tools | All tool files |
| Lifespan | Re-enabled `@asynccontextmanager` server lifespan (was stubbed) | `server.py` |
| roslibpy | Implemented real rosbridge WebSocket client in Yahboom (was 100% mock) | `yahboom_client.py` |
| De-gaslight | Replaced misleading mock responses with honest `simulated: True` labels | `robot_control.py`, `yahboom_client.py` |
| Missing actions | Added `start_cleaning`, `stop_cleaning`, `clean_room`, `go_to`, `get_map`, `start_fast_mapping`, Hue actions to Literal | `robot_control.py` |
| Generic handler | `_handle_physical_robot` now returns honest "not_implemented" instead of fake success | `robot_control.py` |
| Demo label | Webapp demo robot labeled with `is_demo: True` instead of pretending to be real | `web/backend/main.py` |

### Remaining Work (Next Session)

- Connect Yahboom to real Raspberry Pi hardware and test rosbridge
- Implement camera frame async fetch (stubbed with TODO)
- Mount ros-mcp-server as composed server for pure ROS2 tasks
- Add WebSocket push for real-time robot state to webapp
- Add authentication to HTTP API
- Refactor robot_control.py God-object into per-type strategy classes

---

## 11. Fix Log - Session 2 (2026-02-08)

Continued from Session 1. Focused on webapp improvements, Dreame integration, Gazebo Fuel models, and navigation completeness.

### Webapp - Live Data Integration

| Fix | Description | Files |
|---|---|---|
| useRobots hook | New React hook fetching live robot data from MCP server | `web/src/hooks/useRobots.ts` (new) |
| Dashboard live data | Replaced hardcoded status cards with live MCP data | `web/src/app/page.tsx` |
| Layout live status | Topbar shows real MCP Online/Offline + robot count | `web/src/components/layout/robotics-layout.tsx` |
| Dark mode fix | Replaced hardcoded `text-gray-900` with `text-foreground` theme vars | `web/src/app/globals.css` |
| MCPDirect client | Added direct MCP server client bypassing webapp proxy | `web/src/services/mcpService.ts` |

### Dreame D20 Pro - LIDAR Map to 3D Export

| Fix | Description | Files |
|---|---|---|
| Raw map retrieval | Implemented MiIO raw map fetch via siid=23/piid=1 | `src/robotics_mcp/tools/dreame_client.py` |
| Map parser integration | Integrated vacuum-map-parser-dreame for structured parsing | `src/robotics_mcp/tools/dreame_client.py` |
| 3D export pipeline | OBJ mesh, PLY point cloud, Unity NavMesh JSON, Blender Python script | `src/robotics_mcp/tools/dreame_map_export.py` (new) |
| export_map operation | Added export_map to dreame_control portmanteau tool | `src/robotics_mcp/tools/dreame_control.py` |
| Webapp export UI | Export buttons for all 3D formats in DreameControls component | `web/src/components/DreameControls.tsx` |
| Dependency | Added `vacuum-map-parser-dreame>=0.1.2` to pyproject.toml | `pyproject.toml` |

### Gazebo Fuel Model Browser

| Fix | Description | Files |
|---|---|---|
| Fuel API service | REST client for fuel.gazebosim.org - search, download, local management | `src/robotics_mcp/services/gazebo_fuel_service.py` (new) |
| Gazebo spawn/delete | ROS service calls for model spawning (Classic + Gz Sim) | `src/robotics_mcp/clients/gazebo_client.py` |
| gazebo_models tool | MCP portmanteau: search, download, spawn, list_local, delete_local | `src/robotics_mcp/tools/gazebo_models.py` (new) |
| Tool registration | Registered gazebo_models in server.py | `src/robotics_mcp/server.py` |
| Webapp API endpoints | /api/fuel/models, /api/fuel/download, /api/fuel/spawn, /api/fuel/local | `web/backend/main.py` |
| Model browser UI | Full React component with search, thumbnails, download, spawn controls | `web/src/components/GazeboModelBrowser.tsx` (new) |
| Gazebo page tabs | Restructured Gazebo page: Model Library + Robot Control + Setup tabs | `web/src/app/robot-control/gazebo/page.tsx` |

### Sidebar Navigation - Zero Invisible Pages

| Fix | Description | Files |
|---|---|---|
| Virtual Platforms section | Added collapsible section: Unity3D, VRChat, Resonite, VBot Control | `robotics-layout.tsx` |
| Gazebo in Robot Control | Added Gazebo Simulation to Robot Control children | `robotics-layout.tsx` |
| Workflows section | Added collapsible Workflows section (was completely invisible) | `robotics-layout.tsx` |
| Avatars & VRM group | Grouped VRM Avatars + VRoid Studio (VRoid was invisible) | `robotics-layout.tsx` |
| Documentation Browser | Added /documentation route + sidebar entry | `App.tsx`, `robotics-layout.tsx` |
| 7 missing doc pages | Added routes + sidebar for pilot-labs-scout, scenne-humanoid, unitree-go2, unitree-humanoids, hardware-requirements, planning-strategy, software-installation | `App.tsx`, `robotics-layout.tsx` |
| Unity page fixes | Added missing state vars (mcpConnected, appRunning, etc.) | `web/src/app/unity3d/page.tsx` |
| VRChat page fixes | Added missing imports (useEffect, mcpService, LaunchControls) | `web/src/app/vrchat/page.tsx` |
| Resonite import fix | Fixed relative imports to @/ aliases | `web/src/app/resonite/page.tsx` |

### Updated Tool Count

After Session 2, the server has **13 registered portmanteau tools**:

1. `robotics_system` - System management
2. `robot_control` - Unified physical/virtual control
3. `robot_behavior` - Animation, camera, navigation
4. `robot_manufacturing` - 3D printing, CNC
5. `robot_virtual` - Virtual robot CRUD + operations
6. `robot_model_tools` - 3D model tools
7. `vbot_crud` - Virtual robot lifecycle
8. `drone_control` - Drone flight operations
9. `dreame_control` - Dreame D20 Pro vacuum (with map export)
10. `gazebo_models` - Gazebo Fuel model browser + spawner
11. `workflow_management` - Workflow orchestration
12. `virtual_robotics` - Legacy virtual robotics
13. `robot_navigation` - (if registered separately)

### Remaining Work (Session 3)

- Connect Yahboom to real Raspberry Pi hardware and test rosbridge
- Migrate Unity/VRChat/Resonite pages from inline styles to Tailwind/shadcn
- Implement camera frame async fetch (stubbed with TODO)
- Mount ros-mcp-server as composed server for pure ROS2 tasks
- Add WebSocket push for real-time robot state to webapp
- Add authentication to HTTP API
- Refactor robot_control.py God-object into per-type strategy classes

---

*End of Analysis*
