#!/usr/bin/env python3
"""Robotics MCP Server - Unified control for physical and virtual robots.

FastMCP 3.4.4+ compliant server with dual transport (stdio/HTTP) and MCP server composition.
"""

# CRITICAL: Set stdio to binary mode on Windows for Antigravity IDE compatibility
# Antigravity IDE is strict about JSON-RPC protocol and interprets trailing \r as "invalid trailing data"
# This must happen BEFORE any imports that might write to stdout
# Import all necessary modules
import asyncio
import json
import logging
import os
import socket
import subprocess
import sys
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Any, Literal

from fastapi import APIRouter, FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, JSONResponse
from fastapi.staticfiles import StaticFiles
from fastmcp import FastMCP
from fastmcp.server import create_proxy
from pydantic import BaseModel, Field

from .sim_orchestrator.tools import register_sim_orchestrator
from .tools.agentic_support import AgenticSupportTool
from .tools.dreame_control import DreameControlTool
from .tools.drone_control import DroneControlTool
from .tools.environmental_logistics import EnvironmentalLogisticsTool
from .tools.fab_art_fleet import FabArtFleetTool
from .tools.gazebo_models import GazeboModelsTool
from .tools.noetix_bumi import NoetixBumiTool
from .tools.robot_control import RobotControlTool
from .tools.robot_manufacturing import RobotManufacturingTool
from .tools.robot_model_tools import RobotModelTools
from .tools.sim_art_fleet import SimArtFleetTool
from .tools.vbot_crud import VbotCrudTool
from .utils.auth import wurst_auth_middleware
from .utils.config_loader import ConfigLoader
from .utils.state_manager import RobotStateManager

# 1. Force Binary Mode (Prevent CRLF corruption on Windows)
if os.name == "nt":
    try:
        import msvcrt

        msvcrt.setmode(sys.stdin.fileno(), os.O_BINARY)
        msvcrt.setmode(sys.stdout.fileno(), os.O_BINARY)
    except (ImportError, OSError, AttributeError):
        pass

# 2. Industrial Logging (FastMCP 3.2 Standard)
from robotics_mcp.utils.logging_config import get_logger, setup_logging

setup_logging(log_level="INFO")

# Suppress noisy loggers for clean "Bashing" telemetry
for logger_name in ["mcp.server.lowlevel.server", "fastmcp", "uvicorn", "watchfiles"]:
    logging.getLogger(logger_name).setLevel(logging.WARNING)

logger = get_logger(__name__)


class RoboticsConfig(BaseModel):
    """Configuration for Robotics MCP server."""

    enable_http: bool = Field(default=True, description="Enable HTTP interface alongside stdio")
    http_port: int = Field(default=10707, description="HTTP server port")
    http_host: str = Field(default="0.0.0.0", description="HTTP server host")
    log_level: str = Field(default="INFO", description="Logging level")
    config_path: str | None = Field(default=None, description="Path to config YAML file")


@asynccontextmanager
async def server_lifespan(mcp_instance: FastMCP):
    """Server lifespan for startup and cleanup."""
    logger.info("Robotics MCP server starting up", version="0.1.0")
    yield
    logger.info("Robotics MCP server shutting down")


class RoboticsMCP:
    """Robotics MCP Server with unified bot + vbot control."""

    def __init__(self, config: RoboticsConfig | None = None):
        """Initialize Robotics MCP server.

        Args:
            config: Server configuration. If None, uses defaults.
        """
        self.config = config or RoboticsConfig()

        self.mcp = FastMCP(
            name="Robotics-MCP",
            version="0.2.0",
            lifespan=self.lifespan_context,
        )

        _bridge_proxies = []
        bridge_urls = os.getenv("MCP_BRIDGE_URLS", "")
        # --- Crossconnect real hardware peers (yahboom 10892, dreame 10894, nori 11970) ---
        _defaults = [
            "http://127.0.0.1:10892/mcp",
            "http://127.0.0.1:10894/mcp",
            "http://127.0.0.1:11970/mcp",
        ]
        for _u in _defaults:
            if _u not in bridge_urls:
                bridge_urls = _u if not bridge_urls else bridge_urls + "," + _u
                logger.info("Crossconnect auto-bridge peer", url=_u)
        if bridge_urls:
            for url in bridge_urls.split(","):
                url = url.strip()
                if url:
                    try:
                        self.mcp.add_provider(create_proxy(url))
                        _bridge_proxies.append(url)
                    except Exception as exc:
                        logger.warning("Bridge proxy registration failed for %s: %s", url, exc)

        # Initialize managers
        self.config_loader = ConfigLoader(Path(self.config.config_path) if self.config.config_path else None)
        self.config_data = self.config_loader.load()
        self.state_manager = RobotStateManager()

        # MCP server composition (will be mounted if available)
        self.mounted_servers: dict[str, Any] = {}
        self._unity_available = False  # Flag for Unity availability

        # NOTE: Server mounting is now done asynchronously in initialize_async()

        # SOTA: Initialize FastAPI here so it's available for ASGI export
        if self.config.enable_http:
            self.http_app = FastAPI(
                title="Robotics MCP API",
                description="Unified industrial control and telemetry for physical/virtual robots.",
                version="1.4.1",
            )
            # Add Wurst-Auth Security Middleware (The Benny Protocol)
            self.http_app.middleware("http")(wurst_auth_middleware)
            self.http_app.add_middleware(
                CORSMiddleware,
                allow_origins=["http://127.0.0.1:10706", "http://localhost:10706"],
                allow_origin_regex=r"https?://(localhost|127\.0\.0\.1|192\.168\.\d+\.\d+)(:\d+)?",
                allow_credentials=True,
                allow_methods=["*"],
                allow_headers=["*"],
            )

            # --- Fleet health probe: MUST be available before initialize_async ---
            @self.http_app.get("/api/v1/health")
            @self.http_app.get("/health")
            @self.http_app.get("/api/health")
            async def _fleet_health():
                return {"status": "healthy", "version": "0.2.1"}

            @self.http_app.get("/api/v1/capabilities")
            async def _fleet_capabilities():
                return {"name": "robotics-mcp", "version": "0.2.1", "transport": ["stdio", "http"]}

            @self.http_app.get("/api/v1/robots")
            async def _fleet_robots():
                robots = self.state_manager.list_robots()
                out = []
                for r in robots:
                    d = r.to_dict()
                    d["id"] = d.get("robot_id", d.get("id"))
                    out.append(d)
                return {"robots": out}

            @self.http_app.get("/api/v1/robots/{robot_id}/status")
            async def _fleet_robot_status(robot_id: str):
                try:
                    if hasattr(self, "robot_control") and getattr(self, "robot_control", None):
                        result = await self.robot_control.handle_action(robot_id, "get_status", {})
                        return result.get("data", result) if isinstance(result, dict) else result
                    robot = self.state_manager.get_robot(robot_id)
                    if not robot:
                        raise HTTPException(status_code=404, detail=f"Robot {robot_id} not found")
                    return robot.to_dict()
                except HTTPException:
                    raise
                except Exception as e:
                    raise HTTPException(status_code=500, detail=str(e)) from e

            @self.http_app.get("/api/v1/status")
            async def _fleet_status():
                robots = self.state_manager.list_robots()
                return {
                    "version": "0.2.1",
                    "status": "healthy",
                    "robots": [r.to_dict() for r in robots],
                    "mounted_servers": list(self.mounted_servers.keys()),
                    "http_enabled": self.config.enable_http,
                }

            @self.http_app.on_event("startup")
            async def _http_startup_load_robots():
                try:
                    await self._load_robots_from_config()
                except Exception:
                    pass

            # --- Fleet apps hub (git-github-mcp port, crossconnected hardware) ---
            def _load_registry():
                reg = Path(r"D:\Dev\repos\mcp-central-docs\operations\fleet-registry.json")
                if reg.is_file():
                    try:
                        data = json.loads(reg.read_text(encoding="utf-8"))
                        rows = data.get("fleet") or data.get("servers") or []
                        if isinstance(rows, list) and rows:
                            return rows
                    except Exception:
                        pass
                return [
                    {
                        "id": "yahboom-mcp",
                        "name": "Yahboom MCP",
                        "description": "Yahboom ROSMASTER car — real hardware on 10892",
                        "port": 10892,
                        "frontend_port": 10893,
                        "category": "robotics",
                        "github_owner": "sandraschi",
                        "github_repo": "yahboom-mcp",
                        "repo_path": r"D:\Dev\repos\yahboom-mcp",
                    },
                    {
                        "id": "dreame-mcp",
                        "name": "Dreame MCP",
                        "description": "Dreame D20 Pro vacuum — real hardware on 10894",
                        "port": 10894,
                        "frontend_port": 10895,
                        "category": "robotics",
                        "github_owner": "sandraschi",
                        "github_repo": "dreame-mcp",
                        "repo_path": r"D:\Dev\repos\dreame-mcp",
                    },
                    {
                        "id": "norirobotics-mcp",
                        "name": "Nori Robotics",
                        "description": "Nori A3 bimanual — real hardware on 11970",
                        "port": 11970,
                        "frontend_port": 11971,
                        "category": "robotics",
                        "github_owner": "sandraschi",
                        "github_repo": "norirobotics-mcp",
                        "repo_path": r"D:\Dev\repos\norirobotics-mcp",
                    },
                    {
                        "id": "robotics-mcp",
                        "name": "Robotics MCP",
                        "description": "Unified robotics control — current app",
                        "port": 10707,
                        "frontend_port": 10706,
                        "category": "robotics",
                        "github_owner": "sandraschi",
                        "github_repo": "robotics-mcp",
                        "repo_path": r"D:\Dev\repos\robotics-mcp",
                    },
                ]

            def _check_port_health_sync(port: int, timeout: float = 1.2):
                try:
                    with socket.create_connection(("127.0.0.1", port), timeout=timeout):
                        pass
                except Exception as e:
                    return {
                        "port": port,
                        "alive": False,
                        "reason": f"tcp refused: {e}",
                        "health_url": f"http://127.0.0.1:{port}/health",
                    }
                for path in ("/health", "/api/health", "/api/v1/health", "/api/status"):
                    try:
                        import httpx

                        with httpx.Client(timeout=timeout) as c:
                            r = c.get(f"http://127.0.0.1:{port}{path}")
                            if 200 <= r.status_code < 500:
                                return {
                                    "port": port,
                                    "alive": True,
                                    "status_code": r.status_code,
                                    "health_url": f"http://127.0.0.1:{port}{path}",
                                    "reason": "http ok",
                                }
                    except Exception:
                        continue
                return {
                    "port": port,
                    "alive": True,
                    "reason": "tcp open but health 404",
                    "health_url": f"http://127.0.0.1:{port}/health",
                }

            @self.http_app.get("/api/apps")
            async def _api_apps():
                rows = _load_registry()
                apps = []
                for row in rows:
                    if not isinstance(row, dict):
                        continue
                    rid = str(row.get("id") or "")
                    port = int(row.get("frontend_port") or row.get("port") or 0)
                    if port <= 0:
                        continue
                    repo_path = Path(str(row.get("repo_path") or f"D:/Dev/repos/{rid}"))
                    has_tauri = (repo_path / "native" / "tauri.conf.json").is_file() or (
                        repo_path / "src-tauri" / "tauri.conf.json"
                    ).is_file()
                    apps.append(
                        {
                            "id": rid,
                            "name": str(row.get("name") or rid),
                            "description": str(row.get("description") or ""),
                            "port": port,
                            "backend_port": int(row.get("port") or 0),
                            "category": str(row.get("category") or "robotics"),
                            "url": f"http://127.0.0.1:{port}",
                            "gh_url": f"https://github.com/{row.get('github_owner') or 'sandraschi'}/{row.get('github_repo') or rid}",
                            "repo_path": str(repo_path),
                            "has_tauri": has_tauri,
                            "has_tauri_installed": False,
                            "last_commit": None,
                        }
                    )
                apps.sort(key=lambda a: a["port"])
                return {"apps": apps, "fleet_total": len(rows)}

            @self.http_app.get("/api/apps/health")
            async def _api_apps_health(port: int):
                return await asyncio.to_thread(_check_port_health_sync, port)

            @self.http_app.post("/api/apps/ensure")
            async def _api_apps_ensure(payload: dict):
                app_id = str(payload.get("id") or payload.get("app_id") or "").strip()
                port = int(payload.get("port") or 0)
                if not port and app_id:
                    for row in _load_registry():
                        if str(row.get("id")) == app_id:
                            port = int(row.get("frontend_port") or row.get("port") or 0)
                            break
                if not port:
                    return {"success": False, "error": "port or id required", "alive": False}
                health = await asyncio.to_thread(_check_port_health_sync, port)
                if health.get("alive"):
                    return {
                        "success": True,
                        "status": "already_running",
                        "alive": True,
                        "url": f"http://127.0.0.1:{port}",
                        "port": port,
                        "id": app_id,
                    }
                candidates = []
                for cand in [app_id, app_id.replace("-mcp", "")] if app_id else []:
                    mcd = Path(r"D:\Dev\repos\mcp-central-docs\starts") / f"{cand}-start.bat"
                    if mcd.exists():
                        candidates.append(str(mcd))
                    repo_ps1 = Path(r"D:\Dev\repos") / cand / "start.ps1"
                    if repo_ps1.exists():
                        candidates.append(str(repo_ps1))
                if candidates:
                    try:
                        subprocess.Popen(
                            ["powershell.exe", "-NoProfile", "-ExecutionPolicy", "Bypass", "-File", candidates[0]],
                            creationflags=subprocess.DETACHED_PROCESS if os.name == "nt" else 0,
                        )
                        for _ in range(15):
                            await asyncio.sleep(1)
                            h = await asyncio.to_thread(_check_port_health_sync, port)
                            if h.get("alive"):
                                return {
                                    "success": True,
                                    "status": "started",
                                    "alive": True,
                                    "url": f"http://127.0.0.1:{port}",
                                    "port": port,
                                    "id": app_id,
                                }
                        return {"success": False, "error": "start initiated but port not yet alive", "alive": False}
                    except Exception as e:
                        return {"success": False, "error": str(e), "alive": False}
                return {"success": False, "error": "no start candidate found", "alive": False}

            # Mount static files
            web_dir = Path(__file__).parent.parent.parent / "web_sota" / "dist"
            if web_dir.exists():
                self.http_app.mount("/static", StaticFiles(directory=str(web_dir)), name="static")
        else:
            self.http_app = None

    @asynccontextmanager
    async def lifespan_context(self, mcp: FastMCP):
        """Server lifespan for startup and cleanup."""
        logger.info(":START: Robotics MCP server initializing...", version="1.4.1")
        await self.initialize_async()
        yield
        logger.info("Robotics MCP server shutting down")

    async def _load_robots_from_config(self):
        """Load and register robots from configuration."""
        try:
            robotics_config = self.config_data.get("robotics", {})

            # Load physical robots
            for robot_key, robot_config in robotics_config.items():
                if robot_key == "virtual" or robot_key == "coordination" or robot_key == "mcp_integration":
                    continue

                if isinstance(robot_config, dict) and robot_config.get("enabled", False):
                    robot_id = robot_config.get("robot_id")
                    if not robot_id:
                        continue

                    # Determine robot type from key
                    if robot_key.startswith("yahboom"):
                        robot_type = "yahboom"
                    elif robot_key.startswith("moorebot"):
                        robot_type = "scout"
                    elif robot_key.startswith("unitree_go2"):
                        robot_type = "go2"
                    elif robot_key.startswith("unitree_g1"):
                        robot_type = "g1"
                    elif robot_key.startswith("dreame"):
                        robot_type = "dreame"
                    elif robot_key.startswith("elegoo"):
                        robot_type = "elegoo"
                    elif robot_key.startswith("noetix") or robot_key.startswith("bumi"):
                        robot_type = "noetix_bumi"
                    elif robot_key.startswith("hue"):
                        robot_type = "hue"
                    elif robot_key.startswith("gazebo"):
                        robot_type = "gazebo"
                    elif robot_key.startswith("nori"):
                        robot_type = "nori_a3"
                    else:
                        robot_type = robot_key

                    # Register robot
                    try:
                        self.state_manager.register_robot(
                            robot_id=robot_id,
                            robot_type=robot_type,
                            platform=None,  # Physical robot
                            metadata=robot_config,
                        )
                        logger.info(
                            "Registered robot from config",
                            robot_id=robot_id,
                            robot_type=robot_type,
                            config_key=robot_key,
                        )
                    except ValueError as e:
                        logger.warning("Failed to register robot", robot_id=robot_id, error=str(e))

            # Load virtual robots
            virtual_config = robotics_config.get("virtual", {})
            if virtual_config.get("enabled", False):
                robots_config = virtual_config.get("robots", {})
                for robot_id, robot_info in robots_config.items():
                    try:
                        self.state_manager.register_robot(
                            robot_id=robot_id,
                            robot_type=robot_info.get("type", "unknown"),
                            platform=robot_info.get("platform", "unity"),
                            metadata=robot_info,
                        )
                        logger.info(
                            "Registered virtual robot from config",
                            robot_id=robot_id,
                            platform=robot_info.get("platform"),
                        )
                    except ValueError as e:
                        logger.warning(
                            "Failed to register virtual robot",
                            robot_id=robot_id,
                            error=str(e),
                        )

        except Exception as e:
            logger.error("Failed to load robots from config", error=str(e))

    async def initialize_async(self):
        """Async initialization of MCP servers with proper error handling."""
        # Mount external MCP servers first (needed by tools)
        await self._mount_mcp_servers()

        # Load robots from configuration
        await self._load_robots_from_config()

        # Initialize tool handlers (after MCP is created and servers are mounted)
        try:
            from robotics_mcp.tools.robot_behavior import RobotBehaviorTool
            from robotics_mcp.tools.robot_virtual import RobotVirtualTool
            from robotics_mcp.tools.robotics_system import RoboticsSystemTool
            from robotics_mcp.tools.workflow_management import WorkflowManagementTool

            try:
                from robotics_mcp.tools.robot_sampling import RobotSamplingTool
            except ModuleNotFoundError:
                RobotSamplingTool = None

            # Consolidated portmanteau tools (SOTA: max 15 tools)
            # Note: RobotControlTool and RobotModelTools are imported at module level
            self.robotics_system = RoboticsSystemTool(
                self.mcp,
                self.state_manager,
                self.config,
                self.config_loader,
                self.mounted_servers,
            )
            self.robot_control = RobotControlTool(self.mcp, self.state_manager, self.mounted_servers)
            self.robot_behavior = RobotBehaviorTool(self.mcp, self.state_manager, self.mounted_servers)
            self.robot_manufacturing = RobotManufacturingTool(self.mcp, self.state_manager, self.mounted_servers)
            self.robot_virtual = RobotVirtualTool(self.mcp, self.state_manager, self.mounted_servers)
            self.robot_model_tools = RobotModelTools(self.mcp, self.state_manager, self.mounted_servers)
            self.vbot_crud = VbotCrudTool(
                self.mcp,
                self.state_manager,
                self.mounted_servers,
                self._unity_available,
            )
            if RobotSamplingTool:
                self.robot_sampling = RobotSamplingTool(self.mcp, self.state_manager, self.mounted_servers)
            else:
                self.robot_sampling = None

            # Workflow management tool
            from robotics_mcp.utils.mcp_client_helper import call_mounted_server_tool

            self.workflow_management = WorkflowManagementTool(
                self.mcp,
                mounted_servers=self.mounted_servers,
                mcp_client_helper=lambda server, tool, args: call_mounted_server_tool(
                    self.mounted_servers, server, tool, args
                ),
            )

            # Drone control tools
            logger.debug("Creating drone_control tool instance")
            self.drone_control = DroneControlTool(self.mcp, self.state_manager, self.mounted_servers)
            logger.debug("Drone control tool instance created")

            # Dreame vacuum control tool (DreameHome cloud)
            self.dreame_control = DreameControlTool(
                self.mcp,
                self.state_manager,
                self.config_loader,
                self.mounted_servers,
            )

            # Noetix Bumi humanoid - info and SDK links
            self.noetix_bumi = NoetixBumiTool(self.mcp)

            # Gazebo Fuel model browser + spawner
            self.gazebo_models = GazeboModelsTool(self.mcp, self.state_manager, self.mounted_servers)
            self.sim_art_fleet = SimArtFleetTool(self.mcp, self.state_manager, self.mounted_servers)
            self.fab_art_fleet = FabArtFleetTool(self.mcp, self.state_manager, self.mounted_servers)

            # Agentic sampling workflow
            self.agentic_support = AgenticSupportTool(self.mcp)

            # Environmental Logistics tool
            self.environmental_logistics = EnvironmentalLogisticsTool(
                self.mcp, self.state_manager, self.mounted_servers
            )

            # Register all tools
            self._register_tools()
        except Exception as e:
            logger.error("Failed to initialize tools", error=str(e), exc_info=True)
            raise

        # Setup HTTP routes after tools are registered
        if self.config.enable_http:
            self._setup_http_routes()

        logger.info("Robotics MCP server initialized", http_enabled=self.config.enable_http)

    def _setup_http_routes(self):
        """Set up FastAPI HTTP routes."""
        router = APIRouter(prefix="/api/v1")

        @router.get("/health")
        async def health():
            """Health check endpoint."""
            return {"status": "healthy", "version": "0.1.0"}

        @router.get("/robots")
        async def list_robots():
            """List all registered robots."""
            robots = self.state_manager.list_robots()
            out = []
            for r in robots:
                d = r.to_dict()
                d["id"] = d.get("robot_id", d.get("id"))
                out.append(d)
            return {"robots": out}

        @router.get("/robots/{robot_id}")
        async def get_robot(robot_id: str):
            """Get robot information."""
            robot = self.state_manager.get_robot(robot_id)
            if not robot:
                raise HTTPException(status_code=404, detail=f"Robot {robot_id} not found")
            return robot.to_dict()

        @router.get("/robots/{robot_id}/status")
        async def get_robot_status(robot_id: str):
            """Get robot status via get_status action."""
            try:
                result = await self.robot_control.handle_action(robot_id, "get_status", {})
                return result.get("data", result) if isinstance(result, dict) else result
            except Exception as e:
                raise HTTPException(status_code=500, detail=str(e)) from e

        @router.post("/robots/{robot_id}/control")
        async def control_robot(robot_id: str, request: dict[str, Any] | None = None):
            """Control a robot via HTTP."""
            if request is None:
                request = {}
            try:
                action = request.get("action", "get_status")
                params = {k: v for k, v in request.items() if k != "action"}
                # Use the robot_control tool
                result = await self.robot_control.handle_action(robot_id, action, params)
                return result
            except Exception as e:
                raise HTTPException(status_code=500, detail=str(e)) from e

        @router.get("/tools")
        async def list_tools():
            """List all available MCP tools."""
            tools = []
            try:
                # Use FastMCP public API to list tools
                tool_list = await self.mcp.get_tools()
                for tool_info in tool_list:
                    # FastMCP Tool objects have parameters (JSON Schema)
                    schema = getattr(tool_info, "parameters", {})
                    # If it's a pydantic model (FastMCP style), we might need to get the schema
                    if hasattr(schema, "model_json_schema"):
                        schema = schema.model_json_schema()

                    tools.append(
                        {
                            "name": getattr(tool_info, "name", str(tool_info)),
                            "description": getattr(tool_info, "description", ""),
                            "inputSchema": schema,
                        }
                    )
            except Exception as e:
                logger.error(f"Error listing tools: {e}")
                # Fall back to the public async API (private attrs removed in 3.4.x)
                mcp_tools = await self.mcp.list_tools()
                for tool in mcp_tools:
                    description = getattr(tool, "description", "") or ""
                    schema = getattr(tool, "parameters", {})
                    if hasattr(schema, "model_json_schema"):
                        schema = schema.model_json_schema()

                    tools.append(
                        {
                            "name": tool.name,
                            "description": description,
                            "inputSchema": schema,
                        }
                    )
            return {"tools": tools}

        @router.post("/tools/{tool_name}")
        async def call_tool(tool_name: str, params: dict[str, Any] | None = None):
            """Call an MCP tool via HTTP."""
            if params is None:
                params = {}
            try:
                # Method 1: Public sync API - get_tool (private attrs removed in 3.4.x)
                tool_func = None
                try:
                    tool = self.mcp.get_tool(tool_name)
                except (ValueError, KeyError, TypeError):
                    tool = None
                if tool is not None:
                    tool_func = getattr(tool, "fn", tool)
                # Method 2: Check if tool_name is a method on mcp
                if tool_func is None and hasattr(self.mcp, tool_name):
                    attr = getattr(self.mcp, tool_name)
                    if callable(attr):
                        tool_func = attr
                # Method 3: Use FastMCP's call_tool method if available
                if tool_func is None and hasattr(self.mcp, "call_tool"):
                    try:
                        result = await self.mcp.call_tool(tool_name, arguments=params)
                        return {"result": result}
                    except Exception as e:
                        logger.error(f"FastMCP call_tool failed: {e}")
                        raise HTTPException(
                            status_code=404,
                            detail=f"Tool '{tool_name}' not found via call_tool",
                        ) from e

                if tool_func is None:
                    # Debug: log available attributes
                    available_attrs = [attr for attr in dir(self.mcp) if not attr.startswith("_")]
                    logger.error(f"Tool '{tool_name}' not found. Available: {available_attrs}")
                    raise HTTPException(status_code=404, detail=f"Tool '{tool_name}' not found")

                # Call the tool function with params as keyword arguments
                result = await tool_func(**params)
                return {"result": result}
            except HTTPException:
                raise
            except Exception as e:
                import traceback

                error_detail = f"{e!s}\n{traceback.format_exc()}"
                logger.error(f"Error calling tool {tool_name}: {error_detail}")
                raise HTTPException(status_code=500, detail=str(e)) from e

        @router.get("/status")
        async def get_status():
            """Get server status."""
            robots = self.state_manager.list_robots()
            return {
                "version": "0.1.0",
                "status": "healthy",
                "robots": [r.to_dict() for r in robots],
                "mounted_servers": list(self.mounted_servers.keys()),
                "http_enabled": self.config.enable_http,
            }

        @router.post("/robots")
        async def register_robot(request: dict[str, Any]):
            """Register a new robot."""
            try:
                robot_id = request.get("robot_id")
                robot_type = request.get("robot_type")
                platform = request.get("platform")
                metadata = request.get("metadata", {})

                if not robot_id or not robot_type:
                    raise HTTPException(status_code=400, detail="robot_id and robot_type required")

                robot = self.state_manager.register_robot(robot_id, robot_type, platform=platform, metadata=metadata)
                if platform == "resonite":
                    await self.robot_control.handle_action(robot_id, "spawn", {})
                return robot.to_dict()
            except ValueError as e:
                raise HTTPException(status_code=400, detail=str(e)) from e
            except Exception as e:
                raise HTTPException(status_code=500, detail=str(e)) from e

        @router.delete("/robots/{robot_id}")
        async def unregister_robot(robot_id: str):
            """Unregister a robot."""
            try:
                self.state_manager.unregister_robot(robot_id)
                return {
                    "status": "success",
                    "message": f"Robot {robot_id} unregistered",
                }
            except Exception as e:
                raise HTTPException(status_code=500, detail=str(e)) from e

        # Add web interface route
        @self.http_app.get("/")
        async def serve_web_interface():
            """Serve the main web interface."""
            web_dir = Path(__file__).parent.parent.parent / "web_sota" / "dist"
            index_file = web_dir / "index.html"

            if index_file.exists():
                return FileResponse(str(index_file), media_type="text/html")
            else:
                return JSONResponse(
                    {
                        "error": "Web interface not compiled",
                        "detail": "index.html not found",
                    },
                    status_code=404,
                )

        @self.http_app.get("/vbot")
        async def serve_vbot_interface():
            """Serve the VBot web interface."""
            web_dir = Path(__file__).parent.parent.parent / "web"
            vbot_file = web_dir / "vbot.html"
            if vbot_file.exists():
                return FileResponse(str(vbot_file), media_type="text/html")
            else:
                return JSONResponse(
                    {
                        "error": "VBot interface not available",
                        "detail": "vbot.html not found",
                    },
                    status_code=404,
                )

        @router.get("/capabilities")
        async def get_capabilities():
            """Fleet capabilities endpoint."""
            return {
                "name": "robotics-mcp",
                "version": "0.2.1",
                "transport": ["stdio", "http"],
                "tools": [
                    "robot_control",
                    "robot_behavior",
                    "vbot_crud",
                    "dreame_control",
                    "gazebo_models",
                    "robotics_system",
                ],
                "features": {"webapp": True, "tauri": True, "ros": True},
            }

        @router.get("/diagnostics")
        async def get_diagnostics():
            """Diagnostics for CUA-NSIS smoke testing."""
            robots = self.state_manager.list_robots()
            return {
                "status": "healthy",
                "version": "0.2.1",
                "robots": [r.to_dict() for r in robots],
                "mounted_servers": list(self.mounted_servers.keys()),
                "tool_count": 13,
                "system": {"platform": "win32", "python": "3.12"},
            }

        @router.get("/llm/discover")
        async def llm_discover():
            """Auto-discover local LLM providers."""
            import httpx

            providers = []
            for name, url in [
                ("ollama", "http://127.0.0.1:11434/api/tags"),
                ("lmstudio", "http://127.0.0.1:1234/v1/models"),
            ]:
                try:
                    async with httpx.AsyncClient(timeout=1.0) as client:
                        r = await client.get(url)
                        providers.append({"name": name, "url": url, "available": r.status_code == 200})
                except Exception:
                    providers.append({"name": name, "url": url, "available": False})
            return {"providers": providers}

        @router.get("/skills")
        async def list_skills():
            """List available skills."""
            return {"skills": [{"name": "robotics-control", "description": "Robotics control skills"}]}

        @router.post("/chat")
        async def chat_completion(request: dict):
            """Chat completion endpoint."""
            return {"response": "Chat endpoint — connect local LLM via /api/llm/discover", "echo": request}

        @router.post("/shutdown")
        async def shutdown():
            """Graceful shutdown endpoint."""
            return {"status": "shutting_down"}

        self.http_app.include_router(router)

    async def _mount_mcp_servers(self):
        """Load external MCP servers for internal use (NOT exposed as tools).

        These servers are kept in self.mounted_servers for internal use via Client.call_tool(),
        but their tools are NOT exposed to avoid tool explosion. Only robotics-mcp's own
        portmanteau tools are exposed.
        """
        try:
            # Load osc-mcp (for internal use only) - starts in Cursor and works with MCP protocol
            try:
                from oscmcp.mcp_server import server as osc_mcp_server

                self.mounted_servers["osc"] = osc_mcp_server
                logger.info("Loaded osc-mcp server (internal use only)")
            except ImportError:
                logger.warning("osc-mcp not available, skipping")

            # Load Unity3D-MCP with robust error handling and timeout protection
            await self._mount_unity_server_safely()
            await self._mount_resonite_server_safely()

            # DISABLED stdio mounts (protocol hangs):
            # - vrchat-mcp, avatar-mcp, blender-mcp, gimp-mcp
            #
            # Fleet HTTP bridges (no stdio mount):
            # - gimp-mcp: robotics_sim_art -> POST :10773/api/v1/tool
            # - avatar-mcp: robotics_sim_art avatar_thumbnail -> POST :10793/api/v1/tools/execute
            #
            # ENABLED stdio mounts with safety:
            # - unity3d-mcp: timeout protection and fallbacks

        except Exception as e:
            logger.error("Error loading MCP servers", error=str(e), exc_info=True)

    async def _mount_unity_server_safely(self):
        """Safely mount Unity3D MCP server with timeout and error handling."""
        import asyncio

        unity_load_timeout = 30.0  # 30 second timeout
        max_retry_attempts = 3
        retry_delay = 2.0

        logger.info(
            "Attempting to load Unity3D MCP server with safety measures",
            timeout=unity_load_timeout,
            max_retries=max_retry_attempts,
        )

        for attempt in range(max_retry_attempts):
            try:
                # Create a task with timeout for Unity server loading
                load_task = asyncio.create_task(self._load_unity_server())

                try:
                    # Wait for Unity server to load with timeout
                    await asyncio.wait_for(load_task, timeout=unity_load_timeout)
                    logger.info(
                        "Successfully loaded Unity3D MCP server",
                        attempt=attempt + 1,
                        server_count=len(self.mounted_servers),
                    )
                    return  # Success - exit retry loop

                except TimeoutError:
                    logger.warning(
                        f"Unity server load timeout (attempt {attempt + 1}/{max_retry_attempts})",
                        timeout=unity_load_timeout,
                    )
                    load_task.cancel()  # Cancel the hanging task

                    if attempt < max_retry_attempts - 1:
                        logger.info(f"Retrying Unity server load in {retry_delay}s...")
                        await asyncio.sleep(retry_delay)
                        continue
                    else:
                        logger.error(
                            "Unity server load failed after all retry attempts",
                            total_attempts=max_retry_attempts,
                        )
                        break

            except Exception as e:
                logger.warning(
                    f"Unity server load failed (attempt {attempt + 1}/{max_retry_attempts})",
                    error=str(e),
                    error_type=type(e).__name__,
                )

                if attempt < max_retry_attempts - 1:
                    logger.info(f"Retrying Unity server load in {retry_delay}s...")
                    await asyncio.sleep(retry_delay)
                else:
                    logger.error(
                        "Unity server load failed after all retry attempts",
                        total_attempts=max_retry_attempts,
                        final_error=str(e),
                    )
                    break

        # If we get here, Unity loading failed - log state exception
        logger.warning(
            "Unity3D MCP server unresponsive - virtual robot integration halted",
            available_servers=list(self.mounted_servers.keys()),
        )

        # Set a flag for tools to know Unity is not available
        self._unity_available = False

    async def _load_unity_server(self):
        """Load Unity3D MCP server with proper error isolation."""
        try:
            logger.debug("Importing unity3d_mcp server module...")

            # Import with timeout protection
            import sys
            from pathlib import Path

            # Add unity3d-mcp to path if not already there
            unity_mcp_path = Path(__file__).parent.parent.parent.parent / "unity3d-mcp" / "src"
            if str(unity_mcp_path) not in sys.path:
                sys.path.insert(0, str(unity_mcp_path))
                logger.debug("Added unity3d-mcp to Python path", path=str(unity_mcp_path))

            # Import the server module
            from unity3d_mcp.server import Unity3DMCP

            logger.debug("Creating Unity3D MCP server instance...")

            # Create server instance with default configuration
            # Note: Unity3DMCP doesn't accept enable_http parameter
            unity_server = Unity3DMCP()

            # Test that server is responsive (quick health check)
            logger.debug("Testing Unity server responsiveness...")
            if hasattr(unity_server, "app") and hasattr(unity_server.app, "list_tools"):
                # Quick tool listing to verify server is working
                loop = asyncio.get_running_loop()
                tools = await loop.run_in_executor(None, unity_server.app.list_tools)
                logger.debug(
                    "Unity server health check passed",
                    tool_count=len(tools) if tools else 0,
                )

            # Store the server
            self.mounted_servers["unity"] = unity_server
            self._unity_available = True

            logger.info(
                "Unity3D MCP server loaded successfully",
                tools_available=len(tools) if "tools" in locals() else "unknown",
            )

        except ImportError as e:
            logger.warning(
                "Unity3D MCP not available (not installed)",
                error=str(e),
                import_path=str(unity_mcp_path),
            )
            raise  # Re-raise to trigger retry logic

        except Exception as e:
            logger.error(
                "Failed to load Unity3D MCP server",
                error=str(e),
                error_type=type(e).__name__,
                exc_info=True,
            )
            raise  # Re-raise to trigger retry logic

    async def _mount_resonite_server_safely(self):
        """Safely mount Resonite MCP server (ResoniteLink) with timeout and error handling.

        Mirrors _mount_unity_server_safely - same timeout/retry safety net, since resonite-mcp
        (like unity3d-mcp) can be slow or absent and must never block robotics-mcp startup.
        """
        import asyncio

        resonite_load_timeout = 30.0
        max_retry_attempts = 3
        retry_delay = 2.0

        logger.info(
            "Attempting to load Resonite MCP server with safety measures",
            timeout=resonite_load_timeout,
            max_retries=max_retry_attempts,
        )

        for attempt in range(max_retry_attempts):
            try:
                load_task = asyncio.create_task(self._load_resonite_server())
                try:
                    await asyncio.wait_for(load_task, timeout=resonite_load_timeout)
                    logger.info(
                        "Successfully loaded Resonite MCP server",
                        attempt=attempt + 1,
                        server_count=len(self.mounted_servers),
                    )
                    return
                except TimeoutError:
                    logger.warning(
                        f"Resonite server load timeout (attempt {attempt + 1}/{max_retry_attempts})",
                        timeout=resonite_load_timeout,
                    )
                    load_task.cancel()
                    if attempt < max_retry_attempts - 1:
                        await asyncio.sleep(retry_delay)
                        continue
                    logger.error("Resonite server load failed after all retry attempts")
                    break
            except Exception as e:
                logger.warning(
                    f"Resonite server load failed (attempt {attempt + 1}/{max_retry_attempts})",
                    error=str(e),
                    error_type=type(e).__name__,
                )
                if attempt < max_retry_attempts - 1:
                    await asyncio.sleep(retry_delay)
                else:
                    logger.error("Resonite server load failed after all retry attempts")
                    break

        logger.warning(
            "Resonite MCP server unresponsive - Resonite vbot spawning halted",
            available_servers=list(self.mounted_servers.keys()),
        )
        self._resonite_available = False

    async def _load_resonite_server(self):
        """Load Resonite MCP server with proper error isolation.

        Unlike unity3d_mcp.server.Unity3DMCP (a wrapper class), resonite_mcp.server exposes a
        module-level `server` FastMCP instance directly - no class to instantiate.
        """
        try:
            logger.debug("Importing resonite_mcp server module...")
            import sys
            from pathlib import Path

            resonite_mcp_path = Path(__file__).parent.parent.parent.parent / "resonite-mcp" / "src"
            if str(resonite_mcp_path) not in sys.path:
                sys.path.insert(0, str(resonite_mcp_path))
                logger.debug("Added resonite-mcp to Python path", path=str(resonite_mcp_path))

            from resonite_mcp.server import server as resonite_server

            logger.debug("Testing Resonite server responsiveness...")
            if hasattr(resonite_server, "list_tools"):
                loop = asyncio.get_running_loop()
                tools = await loop.run_in_executor(None, resonite_server.list_tools)
                logger.debug("Resonite server health check passed", tool_count=len(tools) if tools else 0)

            self.mounted_servers["resonite"] = resonite_server
            self._resonite_available = True
            logger.info("Resonite MCP server loaded successfully")

        except ImportError as e:
            logger.warning("Resonite MCP not available (not installed)", error=str(e))
            raise
        except Exception as e:
            logger.error("Failed to load Resonite MCP server", error=str(e), error_type=type(e).__name__, exc_info=True)
            raise

    def _register_tools(self):
        """Register all MCP tools."""
        # Note: MCP servers are already mounted in __init__

        try:
            # Register consolidated portmanteau tools (SOTA: 5 tools total)
            self.robotics_system.register()  # System: help, status, list_robots
            logger.debug("Registered robotics_system tool")

            self.robot_control.register()  # Control: movement, status, control
            logger.debug("Registered robot_control tool")

            self.robot_behavior.register()  # Behavior: animation, camera, navigation, manipulation
            logger.debug("Registered robot_behavior tool")

            self.robot_manufacturing.register()  # Manufacturing: 3D printers, CNC, laser cutters
            logger.debug("Registered robot_manufacturing tool")
            logger.debug("Registered robot_behavior tool")

            self.robot_virtual.register()  # Virtual: CRUD + virtual robot operations
            logger.debug("Registered robot_virtual tool")

            self.robot_model_tools.register()  # Model: create, import, export, convert, spz operations
            logger.debug("Registered robot_model_tools tool")

            self.vbot_crud.register()  # Virtual robot CRUD operations
            logger.debug("Registered vbot_crud tool")

            self.workflow_management.register()  # Workflow management operations
            logger.debug("Registered workflow_management tool")

            if self.robot_sampling:
                self.robot_sampling.register()  # AI Sampling operations
                logger.debug("Registered robot_sampling tool")

            logger.debug("Calling drone_control.register()")
            self.drone_control.register()  # Drone control operations
            logger.debug("Registered drone_control tool - register() completed")

            self.dreame_control.register()  # Dreame D20 Pro vacuum (DreameHome cloud)
            logger.debug("Registered dreame_control tool")

            self.noetix_bumi.register()  # Noetix Bumi humanoid info/SDK
            logger.debug("Registered noetix_bumi tool")

            self.gazebo_models.register()  # Gazebo Fuel model browser + spawner
            self.sim_art_fleet.register()  # gimp-mcp HTTP sim-art bridge
            self.fab_art_fleet.register()  # inkscape-mcp HTTP fab-art bridge
            logger.debug("Registered gazebo_models tool")

            self.agentic_support.register()  # SEP-1577 Agentic Sampling tool
            logger.debug("Registered agentic_support tool")

            register_sim_orchestrator(self.mcp)  # Sim fleet orchestrator (mujoco/gazebo/isaac/limx)
            logger.debug("Registered sim_orchestrator tools")

            self.environmental_logistics.register()  # Logistics: textiles, pet, mural, health
            logger.debug("Registered robot_textile tool")

            import asyncio

            async def _list_tools_sync() -> list:
                return await self.mcp.list_tools()

            loop = asyncio.get_event_loop()
            tools = asyncio.run(_list_tools_sync()) if not loop.is_running() else []
            logger.info(
                "All tools registered",
                tool_count=len(tools),
                tool_names=[t.name for t in tools],
            )
        except Exception as e:
            logger.error("Failed to register tools", error=str(e), exc_info=True)
            raise

    def run(
        self,
        mode: Literal["stdio", "http", "dual"] = "dual",
        host: str | None = None,
        port: int | None = None,
    ):
        """Run the robotics MCP server.

        Args:
            mode: Server mode - "stdio" (MCP only), "http" (HTTP only), or "dual" (both).
            host: HTTP server host (defaults to config).
            port: HTTP server port (defaults to config).
        """
        host = host or self.config.http_host
        port = port or self.config.http_port

        if mode == "stdio":
            logger.info("Starting MCP stdio server")
            import argparse

            from robotics_mcp.transport import run_server

            args = argparse.Namespace(
                stdio=True,
                http=False,
                sse=False,
                host=host,
                port=port,
                path="/mcp",
                debug=False,
            )
            run_server(self.mcp, args=args, server_name="robotics-mcp", show_banner=False)
        elif mode == "http":
            if not self.config.enable_http:
                raise ValueError("HTTP mode not enabled in configuration")
            logger.info(f"Starting HTTP server on {host}:{port}")
            import uvicorn

            uvicorn.run(self.http_app, host=host, port=port)
        elif mode == "dual":
            logger.info(f"Starting dual-mode server (stdio + HTTP on {host}:{port})")
            # Run HTTP server in background thread
            import threading

            def run_http():
                import uvicorn

                uvicorn.run(self.http_app, host=host, port=port, log_level="info")

            http_thread = threading.Thread(target=run_http, daemon=True)
            http_thread.start()

            # Run stdio in main thread via transport
            import argparse

            from robotics_mcp.transport import run_server

            args = argparse.Namespace(
                stdio=True,
                http=False,
                sse=False,
                host=host,
                port=port,
                path="/mcp",
                debug=False,
            )
            run_server(self.mcp, args=args, server_name="robotics-mcp", show_banner=False)
        else:
            raise ValueError(f"Unknown mode: {mode}")


def main():
    """Entry point for robotics-mcp server."""
    import argparse
    import sys

    parser = argparse.ArgumentParser(description="Robotics MCP Server")
    parser.add_argument(
        "--mode",
        choices=["stdio", "http", "dual"],
        default="stdio",  # Default to stdio for MCP protocol
        help="Server mode (default: stdio)",
    )
    parser.add_argument("--host", default="0.0.0.0", help="HTTP server host")
    parser.add_argument("--port", type=int, default=10707, help="HTTP server port")
    parser.add_argument("--config", help="Path to config YAML file")
    parser.add_argument("--stdio", action="store_true", help="FastMCP stdio mode")
    parser.add_argument("--http", action="store_true", help="FastMCP http mode")
    parser.add_argument("--sse", action="store_true", help="FastMCP sse mode")
    parser.add_argument("--path", default="/mcp", help="FastMCP path")
    parser.add_argument("--debug", action="store_true", help="Enable debug logging")

    args, _unknown = parser.parse_known_args()

    import os as _os

    if _os.getenv("ROBOTICS_TAURI") == "1":
        args.mode = "http"
        args.port = int(_os.getenv("PORT", args.port))
        args.host = _os.getenv("HOST", args.host)

    # Consolidate mode
    mode_intent = "stdio"
    if args.http or getattr(args, "mode", None) == "http":
        mode_intent = "http"
    elif args.sse:
        mode_intent = "sse"
    elif getattr(args, "mode", None) == "dual":
        mode_intent = "dual"

    config = RoboticsConfig(
        enable_http=mode_intent in ["http", "dual", "sse"],
        http_port=args.port,
        http_host=args.host,
        config_path=args.config,
    )

    try:
        server = RoboticsMCP(config)

        # Async initialization of MCP servers
        import asyncio

        asyncio.run(server.initialize_async())

        # CRITICAL: After server initialization, restore stdout for stdio mode
        # This allows the server to communicate via JSON-RPC while preventing initialization logging
        if mode_intent == "stdio":
            if hasattr(sys.stdout, "restore"):
                sys.stdout.restore()
                # Now we can safely write to stdout for JSON-RPC communication

            # Set up proper logging to stderr only (not stdout)
            import logging

            logging.basicConfig(
                level=logging.INFO,
                format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
                stream=sys.stderr,  # Critical: log to stderr, not stdout
            )

        server.run(mode=mode_intent, host=args.host, port=args.port)
    except Exception as e:
        logger.critical("Failed to start server", error=str(e), exc_info=True)
        sys.exit(1)


# Export ASGI app for uvicorn/deployment
# SOTA: This instance will use default config which is expected for most use cases
# If custom args are needed, use a factory or environment variables
_default_config = RoboticsConfig()
_server_instance = RoboticsMCP(_default_config)
app = _server_instance.http_app


if __name__ == "__main__":
    main()
