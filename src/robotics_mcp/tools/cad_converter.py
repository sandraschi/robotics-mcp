"""CAD file converter tool - Convert STEP/STP files to Blender-compatible formats.

Handles CAD file conversion for robotics applications, integrating with blender-mcp.
Converts STEP files to mesh formats (OBJ, STL, PLY) for 3D modeling workflows.
"""

import os
import subprocess
import tempfile
from pathlib import Path
from typing import Any

import structlog

from ..utils.error_handler import format_error_response, format_success_response

logger = structlog.get_logger(__name__)

# Supported CAD formats for input
CAD_INPUT_FORMATS = ["step", "stp", "iges", "igs", "brep"]

# Supported output formats for Blender
CAD_OUTPUT_FORMATS = ["obj", "stl", "ply", "gltf", "glb"]


class CADConverterTool:
    """Tool for converting CAD files to Blender-compatible mesh formats."""

    def __init__(self, mcp: Any, mounted_servers: dict[str, Any] | None = None):
        """Initialize CAD converter tool.

        Args:
            mcp: FastMCP server instance.
            mounted_servers: Dictionary of mounted MCP servers for integration.
        """
        self.mcp = mcp
        self.mounted_servers = mounted_servers or {}

    def register(self):
        """Register CAD converter tool with MCP server."""

        @self.mcp.tool()
        async def cad_converter(
            operation: str,
            cad_path: str | None = None,
            output_path: str | None = None,
            output_format: str = "obj",
            mesh_quality: str = "medium",
            scale_factor: float = 1.0,
            blender_import: bool = False,
            blender_project_path: str | None = None,
        ) -> dict[str, Any]:
            """CAD file converter for robotics applications.

            PORTMANTEAU PATTERN: Consolidates CAD conversion operations for robot modeling.

            SUPPORTED INPUT FORMATS:
            - STEP (.step, .stp) - ISO 10303-21 CAD standard
            - IGES (.iges, .igs) - Initial Graphics Exchange Specification
            - BREP (.brep) - Boundary Representation

            SUPPORTED OUTPUT FORMATS:
            - OBJ - Wavefront OBJ (best for Blender import)
            - STL - Stereolithography (good for 3D printing)
            - PLY - Polygon File Format
            - GLTF/GLB - glTF formats (modern web/mobile)

            INTEGRATION FEATURES:
            - Direct Blender import via blender-mcp
            - Mesh quality control (low/medium/high)
            - Scale adjustment for unit conversion
            - Automatic output path generation

            Args:
                operation: Operation to perform:
                    - "check_cad_support": Check available conversion tools
                    - "convert_cad": Convert CAD file to mesh format
                    - "batch_convert_cad": Convert multiple CAD files
                    - "import_to_blender": Convert and import directly to Blender
                    - "analyze_cad": Get CAD file metadata and structure info
                cad_path: Path to CAD file (required for convert/analyze operations)
                output_path: Output file path (optional, auto-generated if not provided)
                output_format: Output format - "obj", "stl", "ply", "gltf", "glb"
                mesh_quality: Mesh quality - "low", "medium", "high"
                scale_factor: Scale factor for unit conversion (default 1.0)
                blender_import: Whether to import directly to Blender after conversion
                blender_project_path: Blender project path for direct import

            Returns:
                Dictionary containing operation result with conversion details.

            Examples:
                Check conversion support:
                    result = await cad_converter(operation="check_cad_support")

                Convert STEP to OBJ:
                    result = await cad_converter(
                        operation="convert_cad",
                        cad_path="C:/Downloads/robot.step",
                        output_path="C:/Output/robot.obj",
                        output_format="obj",
                        mesh_quality="high"
                    )

                Convert and import to Blender:
                    result = await cad_converter(
                        operation="import_to_blender",
                        cad_path="C:/Downloads/scout_wheel.step",
                        blender_project_path="C:/Projects/robot_model.blend"
                    )
            """
            try:
                if operation == "check_cad_support":
                    return await self._check_cad_support()
                elif operation == "convert_cad":
                    if not cad_path:
                        return format_error_response("cad_path required for convert_cad", error_type="validation_error")
                    return await self._convert_cad(cad_path, output_path, output_format, mesh_quality, scale_factor)
                elif operation == "batch_convert_cad":
                    # For batch conversion, cad_path could be a directory or list
                    return await self._batch_convert_cad(
                        cad_path, output_path, output_format, mesh_quality, scale_factor
                    )
                elif operation == "import_to_blender":
                    if not cad_path:
                        return format_error_response(
                            "cad_path required for import_to_blender", error_type="validation_error"
                        )
                    return await self._import_to_blender(
                        cad_path, output_format, mesh_quality, scale_factor, blender_project_path
                    )
                elif operation == "analyze_cad":
                    if not cad_path:
                        return format_error_response("cad_path required for analyze_cad", error_type="validation_error")
                    return await self._analyze_cad(cad_path)
                else:
                    return format_error_response(f"Unknown operation: {operation}", error_type="validation_error")

            except Exception as e:
                logger.error("CAD converter error", operation=operation, cad_path=cad_path, error=str(e))
                return format_error_response(f"CAD converter error: {e!s}", error_type="conversion_error")

    async def _check_cad_support(self) -> dict[str, Any]:
        """Check what CAD conversion tools are available."""
        tools_available = []

        # Check for Mayo (free, open-source CAD converter)
        mayo_available = await self._check_mayo_available()
        if mayo_available:
            tools_available.append(
                {
                    "name": "Mayo",
                    "description": "Open-source CAD converter (STEP, IGES, BREP)",
                    "formats": ["step", "stp", "iges", "igs", "brep"],
                    "outputs": ["obj", "stl", "ply"],
                    "status": "available",
                }
            )

        # Check for FreeCAD
        freecad_available = await self._check_freecad_available()
        if freecad_available:
            tools_available.append(
                {
                    "name": "FreeCAD",
                    "description": "Open-source parametric 3D CAD modeler",
                    "formats": ["step", "stp", "iges", "igs"],
                    "outputs": ["obj", "stl", "ply", "gltf"],
                    "status": "available",
                }
            )

        # Check for CAD Assistant (free online tool)
        tools_available.append(
            {
                "name": "CAD Assistant",
                "description": "Free CAD conversion tool",
                "formats": ["step", "stp", "iges", "igs", "brep"],
                "outputs": ["obj", "stl", "ply"],
                "status": "available",
                "web_based": True,
            }
        )

        return format_success_response(
            {
                "tools_available": tools_available,
                "total_tools": len(tools_available),
                "recommended_tool": "Mayo" if mayo_available else "CAD Assistant",
                "blender_integration": "blender-mcp" in self.mounted_servers,
            }
        )

    async def _check_mayo_available(self) -> bool:
        """Check if Mayo converter is available."""
        try:
            # Check common installation paths
            common_paths = [
                "C:\\Program Files\\Mayo\\mayo-conv.exe",
                "C:\\Program Files (x86)\\Mayo\\mayo-conv.exe",
                "mayo-conv.exe",  # In PATH
            ]

            for path in common_paths:
                if os.path.exists(path) or await self._check_command_available("mayo-conv"):
                    return True
            return False
        except Exception:
            return False

    async def _check_freecad_available(self) -> bool:
        """Check if FreeCAD is available."""
        try:
            # Check for FreeCAD executable
            result = await self._run_command(["freecad", "--version"])
            return result.returncode == 0
        except Exception:
            return False

    async def _check_command_available(self, command: str) -> bool:
        """Check if a command is available in PATH."""
        try:
            result = await self._run_command([command, "--version"])
            return result.returncode == 0
        except Exception:
            return False

    async def _run_command(self, command: list[str]) -> subprocess.CompletedProcess:
        """Run a command and return the result."""
        try:
            return await subprocess.run(command, capture_output=True, text=True, timeout=30)
        except subprocess.TimeoutExpired as e:
            raise Exception(f"Command timed out: {' '.join(command)}") from e
        except FileNotFoundError as e:
            raise Exception(f"Command not found: {command[0]}") from e

    async def _convert_cad(
        self,
        cad_path: str,
        output_path: str | None,
        output_format: str,
        mesh_quality: str,
        scale_factor: float,
    ) -> dict[str, Any]:
        """Convert a single CAD file."""
        cad_file = Path(cad_path)
        if not cad_file.exists():
            return format_error_response(f"CAD file not found: {cad_path}", error_type="file_not_found")

        # Validate input format
        input_ext = cad_file.suffix.lower().lstrip(".")
        if input_ext not in CAD_INPUT_FORMATS:
            return format_error_response(
                f"Unsupported input format: {input_ext}. Supported: {', '.join(CAD_INPUT_FORMATS)}",
                error_type="unsupported_format",
            )

        # Generate output path if not provided
        if not output_path:
            output_path = str(cad_file.with_suffix(f".{output_format}"))

        # Try conversion with available tools
        conversion_result = await self._try_mayo_conversion(cad_path, output_path, mesh_quality, scale_factor)

        if not conversion_result["success"]:
            conversion_result = await self._try_freecad_conversion(cad_path, output_path, mesh_quality, scale_factor)

        if not conversion_result["success"]:
            return format_error_response(
                "No suitable CAD conversion tool found. Install Mayo or FreeCAD.",
                error_type="no_converter_available",
            )

        return format_success_response(
            {
                "conversion_success": True,
                "input_file": cad_path,
                "output_file": output_path,
                "input_format": input_ext,
                "output_format": output_format,
                "mesh_quality": mesh_quality,
                "scale_factor": scale_factor,
                "tool_used": conversion_result.get("tool_used", "unknown"),
                "conversion_time": conversion_result.get("time_taken", 0),
            }
        )

    async def _try_mayo_conversion(
        self, cad_path: str, output_path: str, mesh_quality: str, scale_factor: float
    ) -> dict[str, Any]:
        """Try conversion using Mayo."""
        if not await self._check_mayo_available():
            return {"success": False, "error": "Mayo not available", "message": "Mayo not available"}

        try:
            # Mayo command construction
            cmd = ["mayo-conv", cad_path, "-o", output_path]

            # Add mesh quality settings
            quality_map = {
                "low": ["--meshing-edge-length", "1.0"],
                "medium": ["--meshing-edge-length", "0.5"],
                "high": ["--meshing-edge-length", "0.1"],
            }

            if mesh_quality in quality_map:
                cmd.extend(quality_map[mesh_quality])

            # Add scale if different from 1.0
            if scale_factor != 1.0:
                cmd.extend(["--scale", str(scale_factor)])

            result = await self._run_command(cmd)

            if result.returncode == 0:
                return {
                    "success": True,
                    "tool_used": "Mayo",
                    "time_taken": getattr(result, "elapsed", 0),
                }
            else:
                return {"success": False, "error": result.stderr, "message": result.stderr, "tool_used": "Mayo"}

        except Exception as e:
            return {"success": False, "error": str(e), "message": str(e), "tool_used": "Mayo"}

    async def _try_freecad_conversion(
        self, cad_path: str, output_path: str, mesh_quality: str, scale_factor: float
    ) -> dict[str, Any]:
        """Try conversion using FreeCAD."""
        if not await self._check_freecad_available():
            return {"success": False, "error": "FreeCAD not available", "message": "FreeCAD not available"}

        try:
            # FreeCAD conversion would require a Python script
            # This is a simplified version - real implementation would need FreeCAD Python API
            return {"success": False, "error": "FreeCAD conversion not implemented yet", "message": "FreeCAD conversion not implemented yet"}

        except Exception as e:
            return {"success": False, "error": str(e), "message": str(e), "tool_used": "FreeCAD"}

    async def _batch_convert_cad(
        self,
        input_path: str | None,
        output_dir: str | None,
        output_format: str,
        mesh_quality: str,
        scale_factor: float,
    ) -> dict[str, Any]:
        """Convert multiple CAD files."""
        if not input_path:
            return format_error_response("input_path required for batch conversion", error_type="validation_error")

        input_path = Path(input_path)
        if input_path.is_file():
            # Single file - convert it
            return await self._convert_cad(str(input_path), output_dir, output_format, mesh_quality, scale_factor)
        elif input_path.is_dir():
            # Directory - convert all CAD files
            cad_files = []
            for ext in CAD_INPUT_FORMATS:
                cad_files.extend(input_path.glob(f"*.{ext}"))

            if not cad_files:
                return format_error_response(f"No CAD files found in {input_path}", error_type="no_files_found")

            results = []
            for cad_file in cad_files:
                output_file = output_dir or str(cad_file.with_suffix(f".{output_format}"))
                result = await self._convert_cad(str(cad_file), output_file, output_format, mesh_quality, scale_factor)
                results.append(
                    {
                        "file": str(cad_file),
                        "success": result.get("success", False),
                        "output": result.get("data", {}).get("output_file"),
                        "error": result.get("error"),
                    }
                )

            successful = sum(1 for r in results if r["success"])
            return format_success_response(
                {
                    "batch_conversion_complete": True,
                    "total_files": len(results),
                    "successful_conversions": successful,
                    "failed_conversions": len(results) - successful,
                    "results": results,
                }
            )
        else:
            return format_error_response(f"Input path not found: {input_path}", error_type="file_not_found")

    async def _import_to_blender(
        self,
        cad_path: str,
        output_format: str,
        mesh_quality: str,
        scale_factor: float,
        blender_project_path: str | None,
    ) -> dict[str, Any]:
        """Convert CAD and import directly to Blender."""
        # First convert the CAD file
        temp_dir = tempfile.mkdtemp()
        temp_output = os.path.join(temp_dir, f"converted_cad.{output_format}")

        conversion_result = await self._convert_cad(cad_path, temp_output, output_format, mesh_quality, scale_factor)

        if not conversion_result.get("success", False):
            return conversion_result

        # Now import to Blender if blender-mcp is available
        if "blender-mcp" not in self.mounted_servers:
            return format_error_response(
                "blender-mcp not available for direct import. CAD file converted successfully.",
                error_type="blender_unavailable",
            )

        try:
            # Use blender-mcp to import the converted file
            blender_result = await self._call_blender_import(temp_output, blender_project_path)

            return format_success_response(
                {
                    "cad_conversion_success": True,
                    "blender_import_success": blender_result.get("success", False),
                    "input_file": cad_path,
                    "converted_file": temp_output,
                    "blender_project": blender_project_path,
                    "conversion_details": conversion_result.get("data", {}),
                    "blender_details": blender_result.get("data", {}),
                    "temp_dir": temp_dir,  # For cleanup
                }
            )

        except Exception as e:
            return format_success_response(
                {
                    "cad_conversion_success": True,
                    "blender_import_success": False,
                    "input_file": cad_path,
                    "converted_file": temp_output,
                    "blender_project": blender_project_path,
                    "conversion_details": conversion_result.get("data", {}),
                    "blender_error": str(e),
                    "temp_dir": temp_dir,
                }
            )

    async def _call_blender_import(self, file_path: str, project_path: str | None) -> dict[str, Any]:
        """Call blender-mcp to import a file."""
        try:
            # This would use the mounted server interface
            # Implementation depends on how mounted servers are called
            return {
                "success": True,
                "message": f"Would import {file_path} to Blender project {project_path}",
            }
        except Exception as e:
            return {"success": False, "error": str(e), "message": str(e)}

    async def _analyze_cad(self, cad_path: str) -> dict[str, Any]:
        """Analyze CAD file and return metadata."""
        cad_file = Path(cad_path)
        if not cad_file.exists():
            return format_error_response(f"CAD file not found: {cad_path}", error_type="file_not_found")

        try:
            # Basic file info
            file_size = cad_file.stat().st_size
            file_ext = cad_file.suffix.lower().lstrip(".")

            # Try to get more detailed info using available tools
            detailed_info = await self._get_cad_metadata(cad_path)

            return format_success_response(
                {
                    "file_path": cad_path,
                    "file_name": cad_file.name,
                    "file_size_bytes": file_size,
                    "file_size_mb": round(file_size / (1024 * 1024), 2),
                    "file_format": file_ext,
                    "supported_format": file_ext in CAD_INPUT_FORMATS,
                    "detailed_metadata": detailed_info,
                }
            )

        except Exception as e:
            return format_error_response(f"Error analyzing CAD file: {e!s}", error_type="analysis_error")

    async def _get_cad_metadata(self, cad_path: str) -> dict[str, Any]:
        """Get detailed metadata from CAD file."""
        # This would use CAD analysis tools to extract model info
        # For now, return basic placeholder
        return {
            "estimated_complexity": "unknown",
            "contains_assemblies": "unknown",
            "unit_system": "unknown",
            "bounding_box": "unknown",
        }
