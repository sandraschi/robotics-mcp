# Changelog

All notable changes to Robotics MCP will be documented in this file.

## [Unreleased]

### Changed
- **Status clarification**: Updated from Beta to Alpha across all documentation
- **Documentation**: Added prominent warnings about alpha status and required composited MCP servers
- **README**: Added dedicated "MCP Server Dependencies" section with setup instructions
- **pyproject.toml**: Changed classifier to "Development Status :: 3 - Alpha"
- **Status badges**: Added ALPHA status badge to README

### Documentation
- Clarified that this server requires 6+ composited MCP servers to function
- Removed misleading "production ready" language
- Added warnings about ongoing development and incomplete features
- Updated GLAMA_INTEGRATION.md with alpha status warnings
- Created GLAMA_AI_RESCAN_GUIDE.md for triggering platform updates

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

