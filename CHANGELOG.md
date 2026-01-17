# Changelog

All notable changes to Robotics MCP will be documented in this file.

## [Unreleased]

### Added
- **Drone Control Integration**: Complete drone support with PX4/ArduPilot via MAVLink
- **4 New Portmanteau Tools**: `drone_control`, `drone_streaming`, `drone_navigation`, `drone_flight_control`
- **Drone Flight Operations**: Takeoff, landing, movement, RTL, arm/disarm, calibration
- **Video Streaming**: FPV, RTSP, WebRTC streaming with OpenIPC integration
- **GPS Navigation**: Waypoint following, geofencing, follow-me mode
- **Advanced Flight Control**: Mission planning, parameter tuning, flight modes
- **Type Safety Improvements**: Fixed Pydantic schema generation errors
- **Union Type Annotations**: Proper `Type | None` syntax throughout codebase
- **Enhanced Documentation**: Comprehensive parameter descriptions for all tools
- **Yahboom Robot Integration**: Full ROS 2 support for Raspbot-V2 with camera, navigation, and arm control
- **Yahboom ROS 2 Client**: Complete client implementation for Yahboom robot communication
- **Web Control Interface**: Modern web-based control panel for robot management
- **Robot Auto-Loading**: Automatic robot registration from YAML configuration
- **Home Patrol Features**: Pre-configured patrol routes for home security
- **Arm & Gripper Control**: Full support for robotic arms and grippers when equipped
- **Real-time Status Monitoring**: Live battery, position, and sensor status updates
- **Unity Integration**: Enabled unity3d-mcp with robust error handling, timeouts, and fallbacks
- **Prerequisites Documentation**: Comprehensive setup guides for Unity3D, VRChat, and MCP servers
- **Webapp Build Fixes**: Replaced shadcn/ui components with HTML/Tailwind for stability
- **Setup Prerequisites Pages**: Interactive web documentation for installation requirements
- **Hardware Requirements**: Detailed specifications for supported robots
- **Software Installation Guide**: Step-by-step installation instructions

### Fixed
- **Pydantic Schema Generation**: Resolved critical errors in MCP tool registration
- **Type Annotations**: Corrected malformed `[Type] = None` syntax to `Type | None`
- **Function Signatures**: Fixed missing closing parentheses in parameter definitions
- **MCP Tool Registration**: Ensured all 11 portmanteau tools register correctly
- **Unity Server Loading**: Added 30-second timeouts and 3 retry attempts with graceful fallbacks
- **Webapp Component Errors**: Resolved @radix-ui/react-label build errors by removing shadcn/ui dependencies
- **Cursor MCP Integration**: Fixed server startup issues preventing robotics-mcp from loading in Cursor IDE
- **NullLogger Replacement**: Removed problematic NullLogger that was causing hangs during FastMCP initialization
- **Lifespan Context Manager**: Fixed AttributeError in FastMCP lifespan handling
- **MCPB Configuration**: Updated to use correct module path for Cursor integration

### Changed
- **Unity Availability**: Now tracked with fallback support when Unity MCP server is unavailable
- **Virtual Robot Operations**: Enhanced with timeout protection and mock fallbacks
- **Mounted Servers**: Unity3D-MCP now enabled with safety measures, other servers remain disabled
- **Documentation**: Major updates to clearly state prerequisites and installation requirements
- **Status Update**: Changed from "requires multiple MCP servers" to "Unity integration active"
- **Webapp Components**: Migrated from shadcn/ui to pure HTML/Tailwind for dependency-free operation

### Security
- **Error Handling**: Added comprehensive error isolation for Unity server loading
- **Timeout Protection**: Implemented timeouts to prevent hangs during MCP server operations
- **Fallback Mechanisms**: Added mock operations when external dependencies are unavailable

## [0.1.0] - 2025-12-02

### Added
- Initial release
- FastMCP 2.13+ server with dual transport (stdio/HTTP)
- MCP server composition (osc-mcp, unity3d-mcp, vrchat-mcp, avatar-mcp)
- Unified bot + vbot control interface
- Robot control portmanteau tool
- Virtual robotics portmanteau tool
- State management for robot connections
- Configuration system (YAML-based)
- Mock data for testing without hardware
- FastAPI HTTP endpoints
- Comprehensive test suite
- MCPB packaging support
- PowerShell scripts for testing and standards checking

### Features
- Physical robot control (Moorebot Scout, Unitree Go2/G1) - planned
- Virtual robot control (Unity/VRChat) - initial implementation
- World Labs Marble/Chisel environment integration - planned
- YDLIDAR SuperLight LiDAR integration - planned
- SLAM and navigation - planned

### Documentation
- Implementation plan
- README with quick start guide
- API documentation
- System prompt for MCP

