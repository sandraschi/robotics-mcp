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
from .utils.state_manager import RobotStateManager
from .tools.robot_control import RobotControlTool
from .tools.virtual_robotics import VirtualRoboticsTool

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

        # Initialize tool handlers
        self.robot_control = RobotControlTool(self.mcp, self.state_manager)
        self.virtual_robotics = VirtualRoboticsTool(self.mcp, self.state_manager)

        # MCP server composition (will be mounted if available)
        self.mounted_servers: Dict[str, Any] = {}

        # Initialize FastAPI for HTTP endpoints
        if self.config.enable_http:
            self.http_app = FastAPI(
                title="Robotics MCP API",
                description="HTTP API for Robotics MCP Server",
                version="0.1.0",
            )
            self._setup_http_routes()

        # Register all tools
        self._register_tools()

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
        async def control_robot(robot_id: str, action: str, params: Dict[str, Any] = None):
            """Control a robot via HTTP."""
            try:
                # Use the robot_control tool
                result = await self.robot_control.handle_action(robot_id, action, params or {})
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
        async def call_tool(tool_name: str, params: Dict[str, Any]):
            """Call an MCP tool via HTTP."""
            try:
                # Execute tool using MCP instance
                result = await self.mcp.call_tool(tool_name, params)
                return {"result": result}
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

        except Exception as e:
            logger.error("Error mounting MCP servers", error=str(e))

    def _register_tools(self):
        """Register all MCP tools."""
        # Mount external MCP servers first
        self._mount_mcp_servers()

        # Register portmanteau tools
        self.robot_control.register()
        self.virtual_robotics.register()

        # Register system tools
        self._register_system_tools()

        logger.info("All tools registered")

    def _register_system_tools(self):
        """Register system management tools."""

        @self.mcp.tool()
        async def get_status() -> Dict[str, Any]:
            """Get robotics MCP server status.

            Returns comprehensive server status including:
            - Server version and health
            - Registered robots (bot + vbot)
            - Mounted MCP servers
            - Configuration status

            Returns:
                Dictionary containing server status information.

            Examples:
                Get server status:
                    status = await get_status()
                    # Returns: {
                    #     "version": "0.1.0",
                    #     "status": "healthy",
                    #     "robots": [...],
                    #     "mounted_servers": [...]
                    # }
            """
            robots = self.state_manager.list_robots()
            return {
                "version": "0.1.0",
                "status": "healthy",
                "robots": [r.to_dict() for r in robots],
                "mounted_servers": list(self.mounted_servers.keys()),
                "http_enabled": self.config.enable_http,
                "http_port": self.config.http_port if self.config.enable_http else None,
            }

        @self.mcp.tool()
        async def list_robots(
            robot_type: Optional[str] = None, is_virtual: Optional[bool] = None
        ) -> Dict[str, Any]:
            """List all registered robots with optional filtering.

            Args:
                robot_type: Filter by robot type (e.g., "scout", "go2", "g1").
                is_virtual: Filter by virtual/physical (True=vbot, False=bot).

            Returns:
                Dictionary containing list of robots.

            Examples:
                List all robots:
                    robots = await list_robots()

                List only virtual robots:
                    vbots = await list_robots(is_virtual=True)

                List only Scout robots:
                    scouts = await list_robots(robot_type="scout")
            """
            robots = self.state_manager.list_robots(robot_type=robot_type, is_virtual=is_virtual)
            return {
                "count": len(robots),
                "robots": [r.to_dict() for r in robots],
            }

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

