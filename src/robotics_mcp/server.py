#!/usr/bin/env python3
"""Robotics MCP Server - Unified control for physical and virtual robots.

FastMCP 2.13+ compliant server with dual transport (stdio/HTTP) and MCP server composition.
"""

# CRITICAL: Set stdio to binary mode on Windows for Antigravity IDE compatibility
# Antigravity IDE is strict about JSON-RPC protocol and interprets trailing \r as "invalid trailing data"
# This must happen BEFORE any imports that might write to stdout
import os
import sys

if os.name == "nt":  # Windows only
    try:
        # Force binary mode for stdin/stdout to prevent CRLF conversion
        import msvcrt

        msvcrt.setmode(sys.stdin.fileno(), os.O_BINARY)
        msvcrt.setmode(sys.stdout.fileno(), os.O_BINARY)
    except (OSError, AttributeError):
        # Fallback: just ensure no CRLF conversion
        pass


# DevNullStdout class for stdio mode to prevent any console output during initialization
class DevNullStdout:
    """Suppress all stdout writes during stdio mode to prevent JSON-RPC protocol corruption."""

    def __init__(self, original_stdout):
        self.original_stdout = original_stdout
        self.buffer = []

    def write(self, text):
        # Buffer output instead of writing to stdout
        self.buffer.append(text)

    def flush(self):
        # Do nothing - prevent any stdout writes
        pass

    def get_buffered_output(self):
        """Get all buffered output for debugging if needed."""
        return "".join(self.buffer)

    def restore(self):
        """Restore original stdout."""
        sys.stdout = self.original_stdout


# CRITICAL: Detect stdio mode BEFORE importing logger
# This must be done before ANY logging imports
_is_stdio_mode = not sys.stdout.isatty()

# Import all necessary modules
import asyncio
import logging
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Any, Dict, List, Literal, Optional

import structlog
from pydantic import BaseModel, Field

# TEMPORARILY DISABLE FastMCP IMPORT FOR DEBUGGING
# Import FastMCP BEFORE doing logging replacement
from fastmcp import FastMCP, Client

from fastapi import APIRouter, FastAPI, HTTPException
from fastapi.responses import JSONResponse
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse

from .utils.config_loader import ConfigLoader
from .utils.error_handler import (
    format_error_response,
    format_success_response,
    handle_tool_error,
)
from .utils.state_manager import RobotStateManager
from .tools.robot_control import RobotControlTool
from .tools.robot_manufacturing import RobotManufacturingTool
from .tools.robot_model_tools import RobotModelTools
from .tools.vbot_crud import VbotCrudTool
from .tools.drone_control import DroneControlTool

# Configure structured logging
structlog.configure(
    processors=[
        structlog.stdlib.filter_by_level,
        structlog.stdlib.add_logger_name,
        structlog.stdlib.add_log_level,
        structlog.stdlib.PositionalArgumentsFormatter(),
        structlog.processors.TimeStamper(fmt="iso"),
        structlog.processors.StackInfoRenderer(),
        structlog.processors.format_exc_info,
        structlog.processors.UnicodeDecoder(),
        structlog.processors.JSONRenderer(),
    ],
    wrapper_class=structlog.make_filtering_bound_logger(logging.INFO),
    logger_factory=structlog.stdlib.LoggerFactory(),
    cache_logger_on_first_use=True,
)

# Setup stderr handler (stdout is reserved for MCP protocol!)
stderr_handler = logging.StreamHandler(sys.stderr)
stderr_handler.setFormatter(logging.Formatter("%(message)s"))

root_logger = logging.getLogger()
root_logger.setLevel(logging.INFO)
root_logger.addHandler(stderr_handler)

logger = structlog.get_logger(__name__)


class RoboticsConfig(BaseModel):
    """Configuration for Robotics MCP server."""

    enable_http: bool = Field(
        default=True, description="Enable HTTP interface alongside stdio"
    )
    http_port: int = Field(default=8080, description="HTTP server port")
    http_host: str = Field(default="0.0.0.0", description="HTTP server host")
    log_level: str = Field(default="INFO", description="Logging level")
    config_path: Optional[str] = Field(
        default=None, description="Path to config YAML file"
    )


# TEMPORARILY DISABLE LIFESPAN FOR DEBUGGING
# @asynccontextmanager
# async def server_lifespan(mcp_instance: FastMCP):
#     """Server lifespan for startup and cleanup."""
#     logger.info("Robotics MCP server starting up", version="0.1.0")
#     yield
#     logger.info("Robotics MCP server shutting down")


def server_lifespan(mcp_instance):
    """Stub lifespan function."""
    return None


class RoboticsMCP:
    """Robotics MCP Server with unified bot + vbot control."""

    def __init__(self, config: Optional[RoboticsConfig] = None):
        """Initialize Robotics MCP server.

        Args:
            config: Server configuration. If None, uses defaults.
        """
        self.config = config or RoboticsConfig()

        # Initialize FastMCP without lifespan to avoid context manager issues
        self.mcp = FastMCP(
            name="Robotics-MCP",
            version="0.1.0",
        )

        # Initialize managers
        self.config_loader = ConfigLoader(
            Path(self.config.config_path) if self.config.config_path else None
        )
        self.config_data = self.config_loader.load()
        self.state_manager = RobotStateManager()

        # MCP server composition (will be mounted if available)
        self.mounted_servers: Dict[str, Any] = {}
        self._unity_available = False  # Flag for Unity availability

        # NOTE: Server mounting is now done asynchronously in initialize_async()

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
                    else:
                        robot_type = robot_key

                    # Register robot
                    try:
                        self.state_manager.register_robot(
                            robot_id=robot_id,
                            robot_type=robot_type,
                            platform=None,  # Physical robot
                            metadata=robot_config
                        )
                        logger.info("Registered robot from config",
                                  robot_id=robot_id,
                                  robot_type=robot_type,
                                  config_key=robot_key)
                    except ValueError as e:
                        logger.warning("Failed to register robot",
                                     robot_id=robot_id,
                                     error=str(e))

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
                            metadata=robot_info
                        )
                        logger.info("Registered virtual robot from config",
                                  robot_id=robot_id,
                                  platform=robot_info.get("platform"))
                    except ValueError as e:
                        logger.warning("Failed to register virtual robot",
                                     robot_id=robot_id,
                                     error=str(e))

        except Exception as e:
            logger.error("Failed to load robots from config", error=str(e))

    async def initialize_async(self):
        """Async initialization of MCP servers with proper error handling."""
        # Mount external MCP servers first (needed by tools)
        await self._mount_mcp_servers()

        # Initialize FastAPI for HTTP endpoints
        if self.config.enable_http:
            self.http_app = FastAPI(
                title="Robotics MCP API",
                description="HTTP API for Robotics MCP Server",
                version="0.1.0",
            )

            # Mount static files
            import os
            web_dir = Path(__file__).parent.parent.parent / "web"
            if web_dir.exists():
                self.http_app.mount("/static", StaticFiles(directory=str(web_dir)), name="static")

        else:
            self.http_app = None

        # Load robots from configuration
        await self._load_robots_from_config()

        # Initialize tool handlers (after MCP is created and servers are mounted)
        try:
            from robotics_mcp.tools.robotics_system import RoboticsSystemTool
            from robotics_mcp.tools.robot_behavior import RobotBehaviorTool
            from robotics_mcp.tools.robot_virtual import RobotVirtualTool
            from robotics_mcp.tools.workflow_management import WorkflowManagementTool

            # Consolidated portmanteau tools (SOTA: max 15 tools)
            # Note: RobotControlTool and RobotModelTools are imported at module level
            self.robotics_system = RoboticsSystemTool(
                self.mcp,
                self.state_manager,
                self.config,
                self.config_loader,
                self.mounted_servers,
            )
            self.robot_control = RobotControlTool(
                self.mcp, self.state_manager, self.mounted_servers
            )
            self.robot_behavior = RobotBehaviorTool(
                self.mcp, self.state_manager, self.mounted_servers
            )
            self.robot_manufacturing = RobotManufacturingTool(
                self.mcp, self.state_manager, self.mounted_servers
            )
            self.robot_virtual = RobotVirtualTool(
                self.mcp, self.state_manager, self.mounted_servers
            )
            self.robot_model_tools = RobotModelTools(
                self.mcp, self.state_manager, self.mounted_servers
            )
            self.vbot_crud = VbotCrudTool(
                self.mcp,
                self.state_manager,
                self.mounted_servers,
                self._unity_available,
            )

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
            self.drone_control = DroneControlTool(
                self.mcp, self.state_manager, self.mounted_servers
            )
            logger.debug("Drone control tool instance created")

            # Register all tools
            self._register_tools()
        except Exception as e:
            error_msg = f"Failed to initialize tools: {e}"
            logger.error("Failed to initialize tools", error=str(e), exc_info=True)
            raise

        # Setup HTTP routes after tools are registered
        if self.config.enable_http:
            self._setup_http_routes()

        logger.info(
            "Robotics MCP server initialized", http_enabled=self.config.enable_http
        )

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
            return {"robots": [r.to_dict() for r in robots]}

        @router.get("/robots/{robot_id}")
        async def get_robot(robot_id: str):
            """Get robot information."""
            robot = self.state_manager.get_robot(robot_id)
            if not robot:
                raise HTTPException(
                    status_code=404, detail=f"Robot {robot_id} not found"
                )
            return robot.to_dict()

        @router.post("/robots/{robot_id}/control")
        async def control_robot(robot_id: str, request: Dict[str, Any] = None):
            """Control a robot via HTTP."""
            if request is None:
                request = {}
            try:
                action = request.get("action", "get_status")
                params = {k: v for k, v in request.items() if k != "action"}
                # Use the robot_control tool
                result = await self.robot_control.handle_action(
                    robot_id, action, params
                )
                return result
            except Exception as e:
                raise HTTPException(status_code=500, detail=str(e))

        @router.get("/tools")
        async def list_tools():
            """List all available MCP tools."""
            tools = []
            # FastMCP stores tools in _tools dict - get info from function
            for tool_name, tool_func in getattr(self.mcp, "_tools", {}).items():
                description = ""
                if hasattr(tool_func, "__doc__") and tool_func.__doc__:
                    description = tool_func.__doc__.split("\n")[0].strip()
                tools.append(
                    {
                        "name": tool_name,
                        "description": description,
                        "inputSchema": {},  # Schema not easily accessible from function
                    }
                )
            return {"tools": tools}

        @router.post("/tools/{tool_name}")
        async def call_tool(tool_name: str, params: Dict[str, Any] = None):
            """Call an MCP tool via HTTP."""
            if params is None:
                params = {}
            try:
                # Try multiple ways to access FastMCP tools
                tool_func = None

                # Method 1: Check _tools dict
                if hasattr(self.mcp, "_tools") and tool_name in self.mcp._tools:
                    tool_func = self.mcp._tools[tool_name]
                # Method 2: Check if tool_name is a method on mcp
                elif hasattr(self.mcp, tool_name):
                    attr = getattr(self.mcp, tool_name)
                    if callable(attr):
                        tool_func = attr
                # Method 3: Use FastMCP's call_tool method if available
                elif hasattr(self.mcp, "call_tool"):
                    try:
                        result = await self.mcp.call_tool(tool_name, arguments=params)
                        return {"result": result}
                    except Exception as e:
                        logger.error(f"FastMCP call_tool failed: {e}")
                        raise HTTPException(
                            status_code=404,
                            detail=f"Tool '{tool_name}' not found via call_tool",
                        )

                if tool_func is None:
                    # Debug: log available attributes
                    available_attrs = [
                        attr for attr in dir(self.mcp) if not attr.startswith("_")
                    ]
                    logger.error(
                        f"Tool '{tool_name}' not found. Available: {available_attrs}"
                    )
                    raise HTTPException(
                        status_code=404, detail=f"Tool '{tool_name}' not found"
                    )

                # Call the tool function with params as keyword arguments
                result = await tool_func(**params)
                return {"result": result}
            except HTTPException:
                raise
            except Exception as e:
                import traceback

                error_detail = f"{str(e)}\n{traceback.format_exc()}"
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
        async def register_robot(request: Dict[str, Any]):
            """Register a new robot."""
            try:
                robot_id = request.get("robot_id")
                robot_type = request.get("robot_type")
                platform = request.get("platform")
                metadata = request.get("metadata", {})

                if not robot_id or not robot_type:
                    raise HTTPException(
                        status_code=400, detail="robot_id and robot_type required"
                    )

                robot = self.state_manager.register_robot(
                    robot_id, robot_type, platform=platform, metadata=metadata
                )
                return robot.to_dict()
            except ValueError as e:
                raise HTTPException(status_code=400, detail=str(e))
            except Exception as e:
                raise HTTPException(status_code=500, detail=str(e))

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
                raise HTTPException(status_code=500, detail=str(e))

        # Add web interface route
        @self.http_app.get("/")
        async def serve_web_interface():
            """Serve the main web interface."""
            web_dir = Path(__file__).parent.parent.parent / "web"
            index_file = web_dir / "index.html"
            if index_file.exists():
                return FileResponse(str(index_file), media_type="text/html")
            else:
                return JSONResponse(
                    {"error": "Web interface not available", "detail": "index.html not found"},
                    status_code=404
                )

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

            # DISABLED SERVERS - cause MCP protocol hangs or conflicts:
            # - vrchat-mcp: causes MCP protocol hangs (may re-enable after Unity works)
            # - avatar-mcp: causes timeseries conflicts
            # - blender-mcp: causes MCP protocol hangs
            # - gimp-mcp: causes MCP protocol hangs
            #
            # ENABLED SERVERS - working with proper error handling:
            # - unity3d-mcp: enabled with timeout protection and fallbacks

        except Exception as e:
            logger.error("Error loading MCP servers", error=str(e), exc_info=True)

    async def _mount_unity_server_safely(self):
        """Safely mount Unity3D MCP server with timeout and error handling."""
        import asyncio

        UNITY_LOAD_TIMEOUT = 30.0  # 30 second timeout
        MAX_RETRY_ATTEMPTS = 3
        RETRY_DELAY = 2.0

        logger.info(
            "Attempting to load Unity3D MCP server with safety measures",
            timeout=UNITY_LOAD_TIMEOUT,
            max_retries=MAX_RETRY_ATTEMPTS,
        )

        for attempt in range(MAX_RETRY_ATTEMPTS):
            try:
                # Create a task with timeout for Unity server loading
                load_task = asyncio.create_task(self._load_unity_server())

                try:
                    # Wait for Unity server to load with timeout
                    await asyncio.wait_for(load_task, timeout=UNITY_LOAD_TIMEOUT)
                    logger.info(
                        "Successfully loaded Unity3D MCP server",
                        attempt=attempt + 1,
                        server_count=len(self.mounted_servers),
                    )
                    return  # Success - exit retry loop

                except asyncio.TimeoutError:
                    logger.warning(
                        f"Unity server load timeout (attempt {attempt + 1}/{MAX_RETRY_ATTEMPTS})",
                        timeout=UNITY_LOAD_TIMEOUT,
                    )
                    load_task.cancel()  # Cancel the hanging task

                    if attempt < MAX_RETRY_ATTEMPTS - 1:
                        logger.info(f"Retrying Unity server load in {RETRY_DELAY}s...")
                        await asyncio.sleep(RETRY_DELAY)
                        continue
                    else:
                        logger.error(
                            "Unity server load failed after all retry attempts",
                            total_attempts=MAX_RETRY_ATTEMPTS,
                        )
                        break

            except Exception as e:
                logger.warning(
                    f"Unity server load failed (attempt {attempt + 1}/{MAX_RETRY_ATTEMPTS})",
                    error=str(e),
                    error_type=type(e).__name__,
                )

                if attempt < MAX_RETRY_ATTEMPTS - 1:
                    logger.info(f"Retrying Unity server load in {RETRY_DELAY}s...")
                    await asyncio.sleep(RETRY_DELAY)
                else:
                    logger.error(
                        "Unity server load failed after all retry attempts",
                        total_attempts=MAX_RETRY_ATTEMPTS,
                        final_error=str(e),
                    )
                    break

        # If we get here, Unity loading failed - log graceful degradation
        logger.warning(
            "Unity3D MCP server not available - virtual robot Unity integration disabled",
            fallback_mode="local_fallbacks_only",
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
            unity_mcp_path = (
                Path(__file__).parent.parent.parent.parent / "unity3d-mcp" / "src"
            )
            if str(unity_mcp_path) not in sys.path:
                sys.path.insert(0, str(unity_mcp_path))
                logger.debug(
                    "Added unity3d-mcp to Python path", path=str(unity_mcp_path)
                )

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
                tools = await asyncio.get_event_loop().run_in_executor(
                    None, unity_server.app.list_tools
                )
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

            logger.debug("Calling drone_control.register()")
            self.drone_control.register()  # Drone control operations
            logger.debug("Registered drone_control tool - register() completed")

            tools = getattr(self.mcp, "_tools", {})
            logger.info(
                "All tools registered",
                tool_count=len(tools),
                tool_names=list(tools.keys()),
            )
        except Exception as e:
            error_msg = f"Failed to register tools: {e}"
            logger.error("Failed to register tools", error=str(e), exc_info=True)
            raise

    def run(
        self,
        mode: Literal["stdio", "http", "dual"] = "dual",
        host: Optional[str] = None,
        port: Optional[int] = None,
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
            self.mcp.run()
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

            # Run stdio server in main thread
            self.mcp.run()
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
    parser.add_argument("--port", type=int, default=8080, help="HTTP server port")
    parser.add_argument("--config", help="Path to config YAML file")
    args = parser.parse_args()

    config = RoboticsConfig(
        enable_http=args.mode in ["http", "dual"],
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
        if _is_stdio_mode:
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

        server.run(mode=args.mode, host=args.host, port=args.port)
    except Exception as e:
        logger.critical("Failed to start server", error=str(e), exc_info=True)
        sys.exit(1)


# NOW DO THE LOGGING REPLACEMENT AFTER ALL IMPORTS ARE COMPLETE
if _is_stdio_mode:
    # Replace stdout with our devnull version to catch any accidental writes
    original_stdout = sys.stdout
    sys.stdout = DevNullStdout(original_stdout)


if __name__ == "__main__":
    main()
