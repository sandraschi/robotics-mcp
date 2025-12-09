"""SPZ file converter tool - Convert .spz files to Unity-compatible formats.

Handles Adobe's compressed Gaussian splat format (.spz) conversion.
"""

import json
import subprocess
import zipfile
from pathlib import Path
from typing import Any, Dict, Optional

import structlog

from ..utils.error_handler import format_error_response, format_success_response, handle_tool_error

logger = structlog.get_logger(__name__)


class SPZConverterTool:
    """Tool for converting .spz files to Unity-compatible formats."""

    def __init__(self, mcp: Any):
        """Initialize SPZ converter tool.

        Args:
            mcp: FastMCP server instance.
        """
        self.mcp = mcp

    def register(self):
        """Register SPZ converter tool with MCP server."""

        @self.mcp.tool()
        async def spz_converter(
            operation: str,
            spz_path: Optional[str] = None,
            output_path: Optional[str] = None,
            output_format: str = "ply",
            unity_project_path: Optional[str] = None,
        ) -> Dict[str, Any]:
            """SPZ file converter and Unity package installer.

            PORTMANTEAU PATTERN: Consolidates .spz handling operations.

            SUPPORTED OPERATIONS:
            - check_spz_support: Check if .spz conversion tools are available
            - convert_spz: Convert .spz file to .ply or other format
            - install_unity_spz_plugin: Install Unity package for .spz support (if available)
            - extract_spz_info: Extract metadata from .spz file

            NOTE: .spz is Adobe's compressed format. There is NO official Unity plugin.
            This tool provides conversion options and workarounds.

            Args:
                operation: Operation to perform:
                    - "check_spz_support": Check available conversion tools
                    - "convert_spz": Convert .spz to .ply/.splat
                    - "install_unity_spz_plugin": Attempt to install Unity support (may not exist)
                    - "extract_spz_info": Get metadata from .spz file
                spz_path: Path to .spz file (required for convert/extract operations).
                output_path: Output file path (optional, auto-generated if not provided).
                output_format: Output format - "ply" (default) or "splat".
                unity_project_path: Unity project path for plugin installation.

            Returns:
                Dictionary containing operation result.

            Examples:
                Check support:
                    result = await spz_converter(operation="check_spz_support")

                Convert .spz to .ply:
                    result = await spz_converter(
                        operation="convert_spz",
                        spz_path="C:/Downloads/file.spz",
                        output_path="C:/Output/file.ply",
                        output_format="ply"
                    )

                Install Unity plugin (if available):
                    result = await spz_converter(
                        operation="install_unity_spz_plugin",
                        unity_project_path="C:/Users/sandr/My project"
                    )
            """
            try:
                if operation == "check_spz_support":
                    return await self._check_spz_support()
                elif operation == "convert_spz":
                    if not spz_path:
                        return format_error_response("spz_path required for convert_spz", error_type="validation_error")
                    return await self._convert_spz(spz_path, output_path, output_format)
                elif operation == "install_unity_spz_plugin":
                    if not unity_project_path:
                        return format_error_response(
                            "unity_project_path required for install_unity_spz_plugin", error_type="validation_error"
                        )
                    return await self._install_unity_spz_plugin(unity_project_path)
                elif operation == "extract_spz_info":
                    if not spz_path:
                        return format_error_response("spz_path required for extract_spz_info", error_type="validation_error")
                    return await self._extract_spz_info(spz_path)
                else:
                    return format_error_response(
                        f"Unknown operation: {operation}",
                        error_type="validation_error",
                        details={"valid_operations": ["check_spz_support", "convert_spz", "install_unity_spz_plugin", "extract_spz_info"]},
                    )

            except Exception as e:
                return handle_tool_error("spz_converter", e, operation=operation)

    async def _check_spz_support(self) -> Dict[str, Any]:
        """Check available .spz conversion tools."""
        tools_available = {
            "adobe_spz_tools": False,
            "python_spz_lib": False,
            "manual_conversion": True,  # Always available (re-export from Marble)
        }

        # Check for Adobe spz-tools (would need to be installed separately)
        try:
            result = subprocess.run(["spz-decompress", "--version"], capture_output=True, timeout=5)
            tools_available["adobe_spz_tools"] = result.returncode == 0
        except (FileNotFoundError, subprocess.TimeoutExpired):
            pass

        # Check for Python spz library
        try:
            import spz  # type: ignore

            tools_available["python_spz_lib"] = True
        except ImportError:
            pass

        recommendations = []
        if not tools_available["adobe_spz_tools"] and not tools_available["python_spz_lib"]:
            recommendations.append(
                "No .spz conversion tools found. Recommended: Re-export from Marble as .ply or .fbx/.glb"
            )
            recommendations.append("Alternative: Build converter using Adobe's spz library: https://github.com/adobe/spz")

        return format_success_response(
            "SPZ support check completed",
            data={
                "tools_available": tools_available,
                "recommendations": recommendations,
                "note": "There is NO official Unity plugin for .spz files. Conversion or re-export is required.",
            },
        )

    async def _convert_spz(self, spz_path: str, output_path: Optional[str], output_format: str) -> Dict[str, Any]:
        """Convert .spz file to .ply or other format."""
        spz_file = Path(spz_path)
        if not spz_file.exists():
            return format_error_response(f".spz file not found: {spz_path}", error_type="file_not_found")

        # Auto-generate output path if not provided
        if not output_path:
            output_path = str(spz_file.with_suffix(f".{output_format}"))

        output_file = Path(output_path)

        # Try different conversion methods
        conversion_method = None

        # Method 1: Try Adobe spz-tools (if available)
        try:
            result = subprocess.run(
                ["spz-decompress", str(spz_file), str(output_file)],
                capture_output=True,
                timeout=60,
                text=True,
            )
            if result.returncode == 0:
                conversion_method = "adobe_spz_tools"
                return format_success_response(
                    f"Converted .spz to .{output_format}",
                    data={
                        "input_file": str(spz_file),
                        "output_file": str(output_file),
                        "format": output_format,
                        "method": conversion_method,
                        "file_size": output_file.stat().st_size if output_file.exists() else 0,
                    },
                )
        except (FileNotFoundError, subprocess.TimeoutExpired):
            pass

        # Method 2: Try Python spz library (if available)
        try:
            import spz  # type: ignore

            # This would require the actual spz Python library implementation
            # Placeholder for future implementation
            conversion_method = "python_spz_lib"
        except ImportError:
            pass

        # If no conversion tools available, provide guidance
        return format_error_response(
            "No .spz conversion tools available",
            error_type="conversion_unavailable",
            details={
                "input_file": str(spz_file),
                "recommendations": [
                    "Re-export from Marble as .ply (for splats) or .fbx/.glb (for meshes)",
                    "Install Adobe spz-tools: https://github.com/adobe/spz",
                    "Build custom converter using Adobe's C++ library",
                ],
                "note": "There is no official Unity plugin for .spz. Re-exporting from Marble is the recommended solution.",
            },
        )

    async def _install_unity_spz_plugin(self, unity_project_path: str) -> Dict[str, Any]:
        """Install Unity plugin for .spz support (installs Gaussian Splatting as alternative).

        NOTE: There is NO official Unity plugin for .spz files.
        This function installs Gaussian Splatting plugin for .ply file support as an alternative.
        """
        project_path = Path(unity_project_path)
        manifest_path = project_path / "Packages" / "manifest.json"

        if not manifest_path.exists():
            return format_error_response(
                f"Not a valid Unity project: {manifest_path} not found", error_type="invalid_project"
            )

        # Install Gaussian Splatting plugin (supports .ply files, alternative to .spz)
        try:
            with open(manifest_path, "r") as f:
                manifest = json.load(f)

            dependencies = manifest.get("dependencies", {})

            # Check if already installed
            gs_package = "com.aras-p.gaussian-splatting"
            if gs_package in dependencies:
                return format_success_response(
                    "Gaussian Splatting plugin already installed",
                    data={
                        "unity_project": str(project_path),
                        "package": gs_package,
                        "version": dependencies[gs_package],
                        "note": "This plugin supports .ply files. Re-export from Marble as .ply to use it.",
                    },
                )

            # Add Gaussian Splatting package (package.json is in /package subdirectory)
            dependencies[gs_package] = "https://github.com/aras-p/UnityGaussianSplatting.git?path=/package"
            manifest["dependencies"] = dependencies

            with open(manifest_path, "w") as f:
                json.dump(manifest, f, indent=2)

            return format_success_response(
                "Gaussian Splatting plugin installed (alternative to .spz)",
                data={
                    "unity_project": str(project_path),
                    "package": gs_package,
                    "source": "https://github.com/aras-p/UnityGaussianSplatting.git",
                    "next_steps": [
                        "Open Unity Editor - package will auto-download",
                        "Re-export from Marble as .ply (not .spz) to use this plugin",
                        "Or export as .fbx/.glb for native Unity support (better for navigation)",
                    ],
                    "note": "There is no Unity plugin for .spz files. This installs Gaussian Splatting for .ply files as an alternative.",
                },
            )

        except Exception as e:
            logger.error(f"Failed to install Gaussian Splatting: {e}")
            return format_error_response(
                f"Failed to install plugin: {e}",
                error_type="installation_error",
                details={
                    "unity_project": str(project_path),
                    "note": "There is no official Unity plugin for .spz files. Re-export from Marble as .ply or .fbx/.glb.",
                },
            )

    async def _extract_spz_info(self, spz_path: str) -> Dict[str, Any]:
        """Extract metadata from .spz file."""
        spz_file = Path(spz_path)
        if not spz_file.exists():
            return format_error_response(f".spz file not found: {spz_path}", error_type="file_not_found")

        file_size = spz_file.stat().st_size

        # Try to read file header
        try:
            with open(spz_file, "rb") as f:
                header = f.read(16)
                header_hex = header.hex()
                header_ascii = "".join(chr(b) if 32 <= b < 127 else "." for b in header[:8])
        except Exception as e:
            return format_error_response(f"Failed to read .spz file: {e}", error_type="read_error")

        return format_success_response(
            "SPZ file info extracted",
            data={
                "file_path": str(spz_file),
                "file_size": file_size,
                "file_size_mb": round(file_size / (1024 * 1024), 2),
                "header_hex": header_hex,
                "header_ascii": header_ascii,
                "format": "Adobe compressed Gaussian splat (.spz)",
                "note": "This is Adobe's compressed format. Unity does not support it natively. Re-export from Marble as .ply or .fbx/.glb.",
            },
        )

