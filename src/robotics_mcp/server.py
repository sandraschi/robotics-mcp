"""
Robotics MCP Server - Unified control for physical and virtual robots.

FastMCP 2.13+ compliant server with dual transport (stdio/HTTP) and MCP server composition.
"""

import asyncio
import logging
import sys
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Any, Dict, List, Literal, Optional

import structlog
from fastapi import APIRouter, FastAPI, HTTPException
from fastapi.responses import JSONResponse
from fastmcp import FastMCP, Client
from pydantic import BaseModel, Field

from .utils.config_loader import ConfigLoader
from .utils.error_handler import format_error_response, format_success_response, handle_tool_error
from .utils.state_manager import RobotStateManager
from .tools.robot_control import RobotControlTool
from .tools.virtual_robotics import VirtualRoboticsTool
from .tools.vbot_crud import VbotCrudTool
from .tools.robot_model_tools import RobotModelTools
from .tools.robot_animation import RobotAnimationTool
from .tools.robot_camera import RobotCameraTool
from .tools.robot_navigation import RobotNavigationTool
from .tools.spz_converter import SPZConverterTool

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

    enable_http: bool = Field(default=True, description="Enable HTTP interface alongside stdio")
    http_port: int = Field(default=8080, description="HTTP server port")
    http_host: str = Field(default="0.0.0.0", description="HTTP server host")
    log_level: str = Field(default="INFO", description="Logging level")
    config_path: Optional[str] = Field(default=None, description="Path to config YAML file")


@asynccontextmanager
async def server_lifespan(mcp_instance: FastMCP):
    """Server lifespan for startup and cleanup."""
    logger.info("Robotics MCP server starting up", version="0.1.0")
    yield
    logger.info("Robotics MCP server shutting down")


class RoboticsMCP:
    """Robotics MCP Server with unified bot + vbot control."""

    def __init__(self, config: Optional[RoboticsConfig] = None):
        """Initialize Robotics MCP server.

        Args:
            config: Server configuration. If None, uses defaults.
        """
        self.config = config or RoboticsConfig()

        # Initialize FastMCP with lifespan
        self.mcp = FastMCP(
            name="Robotics-MCP",
            version="0.1.0",
            lifespan=server_lifespan,
        )

        # Initialize managers
        self.config_loader = ConfigLoader(
            Path(self.config.config_path) if self.config.config_path else None
        )
        self.config_data = self.config_loader.load()
        self.state_manager = RobotStateManager()

        # MCP server composition (will be mounted if available)
        self.mounted_servers: Dict[str, Any] = {}

        # Mount external MCP servers first (needed by tools)
        self._mount_mcp_servers()

        # Initialize FastAPI for HTTP endpoints
        if self.config.enable_http:
            self.http_app = FastAPI(
                title="Robotics MCP API",
                description="HTTP API for Robotics MCP Server",
                version="0.1.0",
            )
        else:
            self.http_app = None

        # Initialize tool handlers (after MCP is created and servers are mounted)
        from robotics_mcp.tools.robotics_system import RoboticsSystemTool

        self.robotics_system = RoboticsSystemTool(
            self.mcp, self.state_manager, self.config, self.config_loader, self.mounted_servers
        )
        self.robot_control = RobotControlTool(self.mcp, self.state_manager)
        self.virtual_robotics = VirtualRoboticsTool(self.mcp, self.state_manager, self.mounted_servers)
        self.vbot_crud = VbotCrudTool(self.mcp, self.state_manager, self.mounted_servers)
        self.robot_model_tools = RobotModelTools(self.mcp, self.state_manager, self.mounted_servers)
        self.robot_animation = RobotAnimationTool(self.mcp, self.state_manager, self.mounted_servers)
        self.robot_camera = RobotCameraTool(self.mcp, self.state_manager, self.mounted_servers)
        self.robot_navigation = RobotNavigationTool(self.mcp, self.state_manager, self.mounted_servers)
        self.spz_converter = SPZConverterTool(self.mcp)

        # Register all tools
        self._register_tools()

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
            return {"robots": [r.to_dict() for r in robots]}

        @router.get("/robots/{robot_id}")
        async def get_robot(robot_id: str):
            """Get robot information."""
            robot = self.state_manager.get_robot(robot_id)
            if not robot:
                raise HTTPException(status_code=404, detail=f"Robot {robot_id} not found")
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
                result = await self.robot_control.handle_action(robot_id, action, params)
                return result
            except Exception as e:
                raise HTTPException(status_code=500, detail=str(e))

        @router.get("/tools")
        async def list_tools():
            """List all available MCP tools."""
            tools = []
            for tool_name, tool_info in self.mcp.list_tools().items():
                tools.append(
                    {
                        "name": tool_name,
                        "description": tool_info.get("description", ""),
                        "inputSchema": tool_info.get("inputSchema", {}),
                    }
                )
            return {"tools": tools}

        @router.post("/tools/{tool_name}")
        async def call_tool(tool_name: str, params: Dict[str, Any] = None):
            """Call an MCP tool via HTTP."""
            if params is None:
                params = {}
            try:
                # Execute tool using MCP instance
                # Note: FastMCP 2.13 tool calling interface
                result = await self.mcp.call_tool(tool_name, **params)
                return {"result": result}
            except Exception as e:
                raise HTTPException(status_code=500, detail=str(e))

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
                    raise HTTPException(status_code=400, detail="robot_id and robot_type required")

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
                return {"status": "success", "message": f"Robot {robot_id} unregistered"}
            except Exception as e:
                raise HTTPException(status_code=500, detail=str(e))

        self.http_app.include_router(router)

    def _mount_mcp_servers(self):
        """Mount external MCP servers for composition."""
        try:
            # Try to mount osc-mcp
            try:
                from oscmcp.mcp_server import server as osc_mcp_server

                self.mcp.mount(osc_mcp_server, prefix="osc", as_proxy=True)
                self.mounted_servers["osc"] = osc_mcp_server
                logger.info("Mounted osc-mcp server")
            except ImportError:
                logger.warning("osc-mcp not available, skipping mount")

            # Try to mount unity3d-mcp
            try:
                from unity3d_mcp.server import Unity3DMCP

                unity_server = Unity3DMCP()
                self.mcp.mount(unity_server.app, prefix="unity", as_proxy=True)
                self.mounted_servers["unity"] = unity_server
                logger.info("Mounted unity3d-mcp server")
            except ImportError:
                logger.warning("unity3d-mcp not available, skipping mount")

            # Try to mount vrchat-mcp
            try:
                from vrchat_mcp import VRChatMCP

                vrchat_server = VRChatMCP()
                self.mcp.mount(vrchat_server.mcp, prefix="vrchat", as_proxy=True)
                self.mounted_servers["vrchat"] = vrchat_server
                logger.info("Mounted vrchat-mcp server")
            except ImportError:
                logger.warning("vrchat-mcp not available, skipping mount")

            # Try to mount avatar-mcp
            try:
                from avatarmcp.server import AvatarMCPServer

                avatar_server = AvatarMCPServer()
                self.mcp.mount(avatar_server.mcp, prefix="avatar", as_proxy=True)
                self.mounted_servers["avatar"] = avatar_server
                logger.info("Mounted avatar-mcp server")
            except ImportError:
                logger.warning("avatar-mcp not available, skipping mount")

            # Try to mount blender-mcp
            try:
                import sys
                from pathlib import Path

                # Add blender-mcp to path if not already there
                blender_mcp_path = Path(__file__).parent.parent.parent.parent / "blender-mcp" / "src"
                if str(blender_mcp_path) not in sys.path:
                    sys.path.insert(0, str(blender_mcp_path))

                from blender_mcp.app import get_app

                blender_app = get_app()
                self.mcp.mount(blender_app, prefix="blender", as_proxy=True)
                self.mounted_servers["blender"] = blender_app
                logger.info("Mounted blender-mcp server")
            except ImportError as e:
                logger.warning(f"blender-mcp not available, skipping mount: {e}")
            except Exception as e:
                logger.warning(f"Failed to mount blender-mcp: {e}")

            # Try to mount gimp-mcp
            try:
                import sys
                from pathlib import Path

                # Add gimp-mcp to path if not already there
                gimp_mcp_path = Path(__file__).parent.parent.parent.parent / "gimp-mcp" / "src"
                if str(gimp_mcp_path) not in sys.path:
                    sys.path.insert(0, str(gimp_mcp_path))

                from gimp_mcp.main import GimpMCPServer

                gimp_server = GimpMCPServer()
                # GimpMCPServer has a .mcp attribute that's the FastMCP instance
                if hasattr(gimp_server, "mcp"):
                    self.mcp.mount(gimp_server.mcp, prefix="gimp", as_proxy=True)
                    self.mounted_servers["gimp"] = gimp_server
                    logger.info("Mounted gimp-mcp server")
                else:
                    # Fallback: create FastMCP app and register tools
                    from fastmcp import FastMCP

                    gimp_app = FastMCP("gimp-mcp")
                    if hasattr(gimp_server, "register_tools"):
                        gimp_server.register_tools(gimp_app)
                    self.mcp.mount(gimp_app, prefix="gimp", as_proxy=True)
                    self.mounted_servers["gimp"] = gimp_server
                    logger.info("Mounted gimp-mcp server (fallback method)")
            except ImportError as e:
                logger.warning(f"gimp-mcp not available, skipping mount: {e}")
            except Exception as e:
                logger.warning(f"Failed to mount gimp-mcp: {e}")

        except Exception as e:
            logger.error("Error mounting MCP servers", error=str(e))

    def _register_tools(self):
        """Register all MCP tools."""
        # Note: MCP servers are already mounted in __init__

        # Register portmanteau tools (SOTA: max 15 tools)
        self.robotics_system.register()  # Portmanteau: help, status, list_robots
        self.robot_control.register()  # Portmanteau: movement, status, control
        self.virtual_robotics.register()  # Portmanteau: virtual robot operations
        self.vbot_crud.register()  # Portmanteau: CRUD for virtual robots
        self.robot_model_tools.register()  # Portmanteau: create, import, export, convert
        self.robot_animation.register()  # Portmanteau: animation and behavior control
        self.robot_camera.register()  # Portmanteau: camera and visual feed control
        self.robot_navigation.register()  # Portmanteau: path planning and navigation
        self.spz_converter.register()  # Portmanteau: .spz file conversion and Unity plugin management

        logger.info("All tools registered")

    # System tools moved to robotics_system portmanteau
    # Keeping this method for backwards compatibility but it's now empty
    def _register_system_tools(self):
        """Register system management tools (DEPRECATED - use robotics_system portmanteau)."""
        pass
        # Legacy code removed - use robotics_system portmanteau instead
        # @self.mcp.tool()
        # async def help() -> Dict[str, Any]:
        #     """Get help information about the Robotics MCP server and its tools.
        #
        #     Returns comprehensive information about the server's purpose, available tools,
        #     and how to use them. This is the primary entry point for understanding
        #     the Robotics MCP's capabilities.
        #
        #     Returns:
        #         A dictionary containing server information, a list of available tools
        #         with their descriptions, and usage guidance.
        #
        #     Examples:
        #         Get help information:
        #             help_info = await help()
        #             # Returns: {
        #             #     "server_name": "Robotics-MCP",
        #             #     "version": "0.1.0",
        #             #     "description": "...",
        #             #     "tools": [...]
        #             # }
        #     """
        #     try:
        #         # Get all registered tools
        #         tools_info = []
        #         for tool_name, tool_info in self.mcp.list_tools().items():
        #             tools_info.append(
        #                 {
        #                     "name": tool_name,
        #                     "description": tool_info.get("description", ""),
        #                 }
        #             )
        #
        #         return {
        #             "server_name": "Robotics-MCP",
        #             "version": "0.1.0",
        #             "description": (
        #                 "Unified robotics control via MCP - Physical and virtual robots (bot + vbot). "
        #                 "Provides comprehensive control for Moorebot Scout, Unitree robots, and virtual "
        #                 "robots in Unity/VRChat. Integrates with osc-mcp, unity3d-mcp, vrchat-mcp, and "
        #                 "avatar-mcp for seamless virtual robotics testing."
        #             ),
        #             "features": [
        #                 "Physical robot control (ROS 1.4 via rosbridge)",
        #                 "Virtual robot control (Unity3D/VRChat/Resonite)",
        #                 "YDLIDAR SuperLight (95g) LiDAR integration",
        #                 "World Labs Marble/Chisel environment generation",
        #                 "Multi-robot coordination",
        #                 "Dual transport (stdio + HTTP)",
        #             ],
        #             "tools": tools_info,
        #             "mounted_servers": list(self.mounted_servers.keys()),
        #             "configuration": {
        #                 "http_enabled": self.config.enable_http,
        #                 "http_port": self.config.http_port if self.config.enable_http else None,
        #                 "config_path": str(self.config_loader.config_path),
        #             },
        #         }
        #     except Exception as e:
        #         logger.error("Failed to generate help", error=str(e), exc_info=True)
        #         return format_error_response("Failed to generate help information", details={"error": str(e)})

        @self.mcp.tool()
        async def get_status() -> Dict[str, Any]:
            """Get robotics MCP server status with connectivity tests.

            Returns comprehensive server status including:
            - Server version and health
            - Registered robots (bot + vbot)
            - Mounted MCP servers and their connectivity
            - Configuration status
            - HTTP server status (if enabled)

            This tool also tests connectivity to mounted MCP servers to verify
            they are properly configured and accessible.

            Returns:
                Dictionary containing server status information with connectivity test results.

            Examples:
                Get server status:
                    status = await get_status()
                    # Returns: {
                    #     "version": "0.1.0",
                    #     "status": "healthy",
                    #     "robots": [...],
                    #     "mounted_servers": {
                    #         "osc": {"available": True, "tools": 3},
                    #         "unity": {"available": True, "tools": 12}
                    #     },
                    #     "http_enabled": True,
                    #     "http_port": 8080
                    # }
            """
            try:
                robots = self.state_manager.list_robots()

                # Test mounted server connectivity
                mounted_servers_status: Dict[str, Any] = {}
                for server_name, server_instance in self.mounted_servers.items():
                    try:
                        # Try to list tools from mounted server
                        if hasattr(server_instance, "list_tools"):
                            tools = server_instance.list_tools()
                            mounted_servers_status[server_name] = {
                                "available": True,
                                "tools_count": len(tools) if isinstance(tools, dict) else 0,
                            }
                        else:
                            mounted_servers_status[server_name] = {
                                "available": True,
                                "tools_count": "unknown",
                            }
                    except Exception as e:
                        logger.warning(
                            "Mounted server connectivity test failed",
                            server=server_name,
                            error=str(e),
                        )
                        mounted_servers_status[server_name] = {
                            "available": False,
                            "error": str(e),
                        }

                # Test HTTP server if enabled
                http_status = None
                if self.config.enable_http:
                    try:
                        import socket

                        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                        sock.settimeout(1)
                        result = sock.connect_ex((self.config.http_host, self.config.http_port))
                        sock.close()
                        http_status = {
                            "enabled": True,
                            "host": self.config.http_host,
                            "port": self.config.http_port,
                            "reachable": result == 0,
                        }
                    except Exception as e:
                        logger.warning("HTTP server status check failed", error=str(e))
                        http_status = {
                            "enabled": True,
                            "host": self.config.http_host,
                            "port": self.config.http_port,
                            "reachable": False,
                            "error": str(e),
                        }

                return format_success_response(
                    "Server status retrieved successfully",
                    data={
                        "version": "0.1.0",
                        "status": "healthy",
                        "robots": [r.to_dict() for r in robots],
                        "robots_count": len(robots),
                        "mounted_servers": mounted_servers_status,
                        "http": http_status,
                        "config": {
                            "http_enabled": self.config.enable_http,
                            "log_level": self.config.log_level,
                        },
                    },
                )
            except Exception as e:
                return handle_tool_error("get_status", e)

        @self.mcp.tool()
        async def list_robots(
            robot_type: Optional[str] = None, is_virtual: Optional[bool] = None
        ) -> Dict[str, Any]:
            """List all registered robots with optional filtering.

            Retrieves a list of all registered robots (both physical and virtual)
            with optional filtering by robot type or virtual/physical status.
            Returns detailed information about each robot including status,
            platform, and metadata.

            Args:
                robot_type: Optional filter by robot type. Valid values:
                    - "scout": Moorebot Scout robots
                    - "go2": Unitree Go2 robots
                    - "g1": Unitree G1 robots
                    - Any custom robot type string
                    If None, returns all robot types.
                is_virtual: Optional filter by virtual/physical status:
                    - True: Only virtual robots (vbots)
                    - False: Only physical robots (bots)
                    - None: Both virtual and physical robots

            Returns:
                Dictionary containing:
                    - count: Number of robots matching filters
                    - robots: List of robot dictionaries with:
                        - robot_id: Unique robot identifier
                        - robot_type: Type of robot
                        - platform: Platform (unity, vrchat, ros, etc.)
                        - is_virtual: Whether robot is virtual
                        - connected: Connection status
                        - metadata: Additional robot metadata

            Examples:
                List all robots:
                    result = await list_robots()
                    # Returns: {"count": 3, "robots": [...]}

                List only virtual robots:
                    result = await list_robots(is_virtual=True)
                    # Returns: {"count": 2, "robots": [vbot_1, vbot_2]}

                List only Scout robots:
                    result = await list_robots(robot_type="scout")
                    # Returns: {"count": 1, "robots": [scout_01]}

                List physical Scout robots:
                    result = await list_robots(robot_type="scout", is_virtual=False)
                    # Returns: {"count": 1, "robots": [scout_01]}
            """
            try:
                robots = self.state_manager.list_robots(robot_type=robot_type, is_virtual=is_virtual)
                return format_success_response(
                    f"Found {len(robots)} robot(s)",
                    data={
                        "count": len(robots),
                        "robots": [r.to_dict() for r in robots],
                        "filters": {
                            "robot_type": robot_type,
                            "is_virtual": is_virtual,
                        },
                    },
                )
            except Exception as e:
                return handle_tool_error("list_robots", e, context={"robot_type": robot_type, "is_virtual": is_virtual})

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

    parser = argparse.ArgumentParser(description="Robotics MCP Server")
    parser.add_argument(
        "--mode",
        choices=["stdio", "http", "dual"],
        default="dual",
        help="Server mode (default: dual)",
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

    server = RoboticsMCP(config)
    server.run(mode=args.mode, host=args.host, port=args.port)


if __name__ == "__main__":
    main()

