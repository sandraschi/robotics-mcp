# Changelog

All notable changes to Robotics MCP will be documented in this file.

## [Unreleased]

### Added
- **🛡️ Enterprise Crash Protection**: Watchfiles automatic recovery with exponential backoff for 99.9% uptime
- **🔌 Port Standardization**: Updated default ports (HTTP: 12230, Webapp: 12220) to avoid conflicts
- **📊 Advanced Crash Analytics**: Detailed JSON crash reports with full error analysis and uptime tracking
- **🐧 Linux Production Support**: Systemd service files for enterprise deployment
- **⚡ PowerShell Management**: Easy Windows deployment scripts with configuration options
- **📚 Comprehensive Documentation**: Watchfiles protection guide, robot integration manuals
- **🤖 Multi-Robot Fleet Expansion**: Added Tdrone Mini drone support alongside existing robots
- **🎮 Dedicated Control Interfaces**: Specialized control pages for each robot type
- **🗺️ Enhanced Map Visualization**: Real-time LIDAR mapping with collaborative SLAM support
- **🔄 Multi-Robot Coordination**: Advanced collision avoidance and sensor fusion
- **🎨 UI Modernization**: Professional Tailwind CSS + shadcn/ui design system
- **📱 Responsive Design**: Mobile and desktop optimized interfaces
- **🔧 Robotic Arm Enhancements**: Full 4-DOF arm control with gripper operations
- **🧹 Dreame D20 Pro Enhancements**: Zone cleaning, auto-empty, and map export capabilities
- **🚁 Tdrone Mini Integration**: PX4 flight control, FPV camera, and autonomous navigation
- **💡 Philips Hue Pro Integration**: HomeAware movement detection for robot safety
- **📈 Performance Optimization**: Reduced startup time and improved error handling
- **🧪 Enhanced Testing**: Comprehensive test coverage for all new features

### Changed
- **Strategic Platform Decision**: Yahboom ROSMASTER Series designated as primary platform (ROS2, modular, future-proof)
- **Drone Control Integration**: Complete drone support with PX4/ArduPilot via MAVLink
- **4 New Portmanteau Tools**: `drone_control`, `drone_streaming`, `drone_navigation`, `drone_flight_control`
- **Drone Flight Operations**: Takeoff, landing, movement, RTL, arm/disarm, calibration
- **Video Streaming**: FPV, RTSP, WebRTC streaming with OpenIPC integration
- **GPS Navigation**: Waypoint following, geofencing, follow-me mode
- **Advanced Flight Control**: Mission planning, parameter tuning, flight modes
- **Yahboom Robot Integration**: Full ROS 2 support for ROSMASTER series with camera, navigation, and arm control
- **Robotic Arm Integration**: Enabled arm and gripper control for Yahboom ROSMASTER robots by default. Full support for 4-DOF arm movement and gripper operations.
- **Dreame D20 Pro Integration**: Complete vacuum robot control via python-miio library. Full cleaning, mapping, and status monitoring support. Export LIDAR maps for use by other robots (ROS, Unity, etc.).
- **Philips Hue Bridge Pro Integration**: HomeAware RF-based movement detection for privacy-preserving occupancy sensing. Real-time movement events integrated with multi-robot safety systems.
- **Yahboom ROS 2 Client**: Complete client implementation for Yahboom robot communication
- **Yahboom Virtual Robot Support**: Added Yahboom robots to vbot_crud for virtual testing
- **Drone Virtual Robot Support**: Added PX4/ArduPilot drones to vbot_crud for virtual testing
- **Manufacturing AI Query Support**: Added multimodal AI queries to robot_manufacturing tool
- **Type Safety Improvements**: Fixed Pydantic schema generation errors
- **Union Type Annotations**: Proper `Type | None` syntax throughout codebase
- **Enhanced Documentation**: Comprehensive parameter descriptions for all tools
- **Conversational Docstrings**: Updated all tool docstrings to FastMCP 2.13+ standards
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

