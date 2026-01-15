"""Workflow management portmanteau tool for robotics-mcp."""

import json
from typing import Any, Literal
from uuid import uuid4

import structlog

from ..services.workflow_executor import WorkflowExecutor
from ..services.workflow_models import Workflow, WorkflowCategory
from ..services.workflow_storage import WorkflowStorage
from ..utils.error_handler import format_error_response, format_success_response

logger = structlog.get_logger(__name__)


class WorkflowManagementTool:
    """Workflow management portmanteau tool."""

    def __init__(
        self,
        mcp: Any,
        mounted_servers: dict[str, Any] | None = None,
        mcp_client_helper: Any | None = None,
        app_launcher: Any | None = None,
    ):
        """Initialize workflow management tool.

        Args:
            mcp: FastMCP server instance.
            mounted_servers: Dictionary of mounted MCP servers.
            mcp_client_helper: Helper for calling MCP tools.
            app_launcher: Application launcher service.
        """
        self.mcp = mcp
        self.mounted_servers = mounted_servers or {}
        self.storage = WorkflowStorage()
        self.executor = WorkflowExecutor(
            self.storage,
            mcp_client_helper=mcp_client_helper,
            app_launcher=app_launcher,
        )

    def register(self):
        """Register workflow management tool with MCP server."""

        @self.mcp.tool()
        async def workflow_management(
            operation: Literal[
                "create",
                "read",
                "update",
                "delete",
                "list",
                "execute",
                "status",
                "templates",
                "import",
                "export",
                "pause",
                "resume",
                "step",
                "continue",
            ],
            workflow_id: str | None = None,
            workflow_data: dict[str, Any] | None = None,
            variables: dict[str, Any] | None = None,
            execution_id: str | None = None,
            category: str | None = None,
            tags: list[str] | None = None,
            search: str | None = None,
            debug_mode: bool = False,
        ) -> dict[str, Any]:
            """Comprehensive workflow management operations.

            OPERATIONS:
            - create: Create new workflow (requires workflow_data)
            - read: Get workflow details (requires workflow_id)
            - update: Update workflow (requires workflow_id, workflow_data)
            - delete: Delete workflow (requires workflow_id)
            - list: List all workflows (filterable by category, tags, search)
            - execute: Execute workflow (requires workflow_id, variables)
            - status: Get execution status (requires execution_id)
            - templates: List available workflow templates
            - import: Import workflow from JSON (requires workflow_data)
            - export: Export workflow to JSON (requires workflow_id)

            Args:
                operation: Operation to perform
                workflow_id: Workflow identifier
                workflow_data: Workflow definition (for create/update/import)
                variables: Variables for workflow execution
                execution_id: Execution identifier (for status operations)
                category: Filter by category (for list operation)
                tags: Filter by tags (for list operation)
                search: Search query (for list operation)

            Returns:
                Operation-specific result with workflow data or execution status

            Examples:
                # Create workflow
                workflow_management(
                    operation="create",
                    workflow_data={
                        "name": "VRoid to VRChat",
                        "category": "avatar",
                        "steps": [...]
                    }
                )

                # List workflows
                workflow_management(operation="list", category="avatar")

                # Execute workflow
                workflow_management(
                    operation="execute",
                    workflow_id="workflow-123",
                    variables={"vroid_file_path": "/path/to/vroid.vroid"}
                )

                # Get execution status
                workflow_management(operation="status", execution_id="exec-456")
            """
            try:
                if operation == "create":
                    if not workflow_data:
                        return format_error_response("workflow_data required for create operation")

                    # Generate ID if not provided
                    if "id" not in workflow_data:
                        workflow_data["id"] = str(uuid4())

                    workflow = Workflow(**workflow_data)
                    created = self.storage.create_workflow(workflow)

                    return format_success_response(
                        {
                            "workflow": created.model_dump(),
                            "message": f"Workflow '{created.name}' created successfully",
                        }
                    )

                elif operation == "read":
                    if not workflow_id:
                        return format_error_response("workflow_id required for read operation")

                    workflow = self.storage.get_workflow(workflow_id)
                    if not workflow:
                        return format_error_response(f"Workflow not found: {workflow_id}")

                    return format_success_response({"workflow": workflow.model_dump()})

                elif operation == "update":
                    if not workflow_id or not workflow_data:
                        return format_error_response(
                            "workflow_id and workflow_data required for update operation"
                        )

                    workflow_data["id"] = workflow_id
                    workflow = Workflow(**workflow_data)
                    updated = self.storage.update_workflow(workflow_id, workflow)

                    return format_success_response(
                        {
                            "workflow": updated.model_dump(),
                            "message": f"Workflow '{updated.name}' updated successfully",
                        }
                    )

                elif operation == "delete":
                    if not workflow_id:
                        return format_error_response("workflow_id required for delete operation")

                    deleted = self.storage.delete_workflow(workflow_id)
                    if not deleted:
                        return format_error_response(f"Workflow not found: {workflow_id}")

                    return format_success_response(
                        {"message": f"Workflow {workflow_id} deleted successfully"}
                    )

                elif operation == "list":
                    workflows = self.storage.list_workflows(
                        category=category, tags=tags, search=search
                    )
                    return format_success_response(
                        {
                            "workflows": [w.model_dump() for w in workflows],
                            "count": len(workflows),
                        }
                    )

                elif operation == "execute":
                    if not workflow_id:
                        return format_error_response("workflow_id required for execute operation")

                    if not variables:
                        variables = {}

                    execution_id = await self.executor.execute_workflow(
                        workflow_id, variables, debug_mode=debug_mode
                    )

                    return format_success_response(
                        {
                            "execution_id": execution_id,
                            "message": "Workflow execution started",
                            "debug_mode": debug_mode,
                        }
                    )

                elif operation == "pause":
                    if not execution_id:
                        return format_error_response("execution_id required for pause operation")

                    await self.executor.pause_execution(execution_id)
                    return format_success_response({"message": "Execution paused"})

                elif operation == "resume":
                    if not execution_id:
                        return format_error_response("execution_id required for resume operation")

                    await self.executor.resume_execution(execution_id)
                    return format_success_response({"message": "Execution resumed"})

                elif operation == "step":
                    if not execution_id:
                        return format_error_response("execution_id required for step operation")

                    await self.executor.step_execution(execution_id)
                    return format_success_response({"message": "Stepped to next instruction"})

                elif operation == "continue":
                    if not execution_id:
                        return format_error_response("execution_id required for continue operation")

                    await self.executor.continue_execution(execution_id)
                    return format_success_response({"message": "Execution continued"})

                elif operation == "status":
                    if not execution_id:
                        return format_error_response("execution_id required for status operation")

                    execution = self.storage.get_execution(execution_id)
                    if not execution:
                        return format_error_response(f"Execution not found: {execution_id}")

                    return format_success_response({"execution": execution.model_dump(mode="json")})

                elif operation == "templates":
                    templates = self._get_templates()
                    return format_success_response({"templates": templates})

                elif operation == "import":
                    if not workflow_data:
                        return format_error_response("workflow_data required for import operation")

                    workflow = Workflow(**workflow_data)
                    imported = self.storage.create_workflow(workflow)

                    return format_success_response(
                        {
                            "workflow": imported.model_dump(),
                            "message": f"Workflow '{imported.name}' imported successfully",
                        }
                    )

                elif operation == "export":
                    if not workflow_id:
                        return format_error_response("workflow_id required for export operation")

                    workflow = self.storage.get_workflow(workflow_id)
                    if not workflow:
                        return format_error_response(f"Workflow not found: {workflow_id}")

                    return format_success_response(
                        {
                            "workflow_json": workflow.model_dump_json(),
                            "workflow": workflow.model_dump(),
                        }
                    )

                else:
                    return format_error_response(f"Unknown operation: {operation}")

            except Exception as e:
                logger.error(
                    "Workflow management operation failed",
                    operation=operation,
                    error=str(e),
                    exc_info=True,
                )
                return format_error_response(f"Operation failed: {str(e)}")

    def _get_templates(self) -> list[dict[str, Any]]:
        """Get built-in workflow templates.

        Returns:
            List of workflow templates.
        """
        return [
            {
                "id": "mixed_reality_launch",
                "name": "Mixed Reality Launch (Scout)",
                "description": "Launch Unity, Resonite, and Blender with Pilot Labs Scout model",
                "category": "demo",
                "steps": [
                    {
                        "id": "check_model",
                        "order": 1,
                        "name": "Check Model Exists",
                        "type": "condition",
                        "condition": {
                            "expression": "exists('${model_path}')",
                            "true_branch": ["launch_unity"],
                            "false_branch": ["create_model"],
                        },
                    },
                    {
                        "id": "create_model",
                        "order": 2,
                        "name": "Create Scout Model",
                        "type": "mcp_tool",
                        "mcp_server": "robotics",
                        "tool_name": "robot_model",
                        "arguments": {
                            "operation": "create",
                            "robot_type": "scout",
                            "output_path": "${model_path}",
                            "format": "fbx",
                        },
                    },
                    {
                        "id": "launch_unity",
                        "order": 3,
                        "name": "Launch Unity3D",
                        "type": "app_launch",
                        "app_id": "unity3d",
                        "app_config": {"desktop": 1, "monitor": 1},
                    },
                    {
                        "id": "launch_resonite",
                        "order": 4,
                        "name": "Launch Resonite",
                        "type": "app_launch",
                        "app_id": "resonite",
                        "app_config": {"desktop": 2, "monitor": 2},
                    },
                    {
                        "id": "launch_blender",
                        "order": 5,
                        "name": "Launch Blender",
                        "type": "app_launch",
                        "app_id": "blender",
                    },
                    {
                        "id": "load_model_blender",
                        "order": 6,
                        "name": "Load Model in Blender",
                        "type": "mcp_tool",
                        "mcp_server": "blender",
                        "tool_name": "open_file",
                        "arguments": {"file_path": "${model_path}"},
                    },
                    {
                        "id": "zoom_model_blender",
                        "order": 7,
                        "name": "Zoom to Model",
                        "type": "mcp_tool",
                        "mcp_server": "blender",
                        "tool_name": "execute_script",
                        "arguments": {"script": "import bpy; bpy.ops.view3d.view_all(center=True)"},
                    },
                    {
                        "id": "color_pink_blender",
                        "order": 8,
                        "name": "Color it Pink",
                        "type": "mcp_tool",
                        "mcp_server": "blender",
                        "tool_name": "execute_script",
                        "arguments": {
                            "script": "import bpy; obj = bpy.context.active_object; mat = obj.data.materials[0] if obj.data.materials else bpy.data.materials.new(name='Pink'); obj.data.materials.append(mat) if not obj.data.materials else None; mat.use_nodes = True; bsdf = mat.node_tree.nodes.get('Principled BSDF'); bsdf.inputs['Base Color'].default_value = (1.0, 0.4, 0.7, 1.0) if bsdf else None; mat.diffuse_color = (1.0, 0.4, 0.7, 1.0)"
                        },
                    },
                    {
                        "id": "create_racetrack_blender",
                        "order": 9,
                        "name": "Create Racetrack",
                        "type": "mcp_tool",
                        "mcp_server": "blender",
                        "tool_name": "execute_script",
                        "arguments": {
                            "script": "import bpy; bpy.ops.mesh.primitive_torus_add(location=(0,0,-0.2), major_radius=8, minor_radius=2); track = bpy.context.active_object; track.name = 'Racetrack'; track.scale = (1, 1, 0.05); mat = bpy.data.materials.new(name='Asphalt'); mat.use_nodes = True; bsdf = mat.node_tree.nodes.get('Principled BSDF'); bsdf.inputs['Base Color'].default_value = (0.1, 0.1, 0.1, 1.0) if bsdf else None; track.data.materials.append(mat)"
                        },
                    },
                    {
                        "id": "create_clown_nose",
                        "order": 10,
                        "name": "Clown Nose",
                        "type": "mcp_tool",
                        "mcp_server": "blender",
                        "tool_name": "execute_script",
                        "arguments": {
                            "script": "import bpy; bpy.ops.mesh.primitive_uv_sphere_add(radius=0.015, location=(0, -0.06, 0.04)); nose = bpy.context.active_object; nose.name = 'Clown Nose'; mat = bpy.data.materials.new(name='Red Nose'); mat.use_nodes = True; bsdf = mat.node_tree.nodes.get('Principled BSDF'); bsdf.inputs['Base Color'].default_value = (1.0, 0.0, 0.0, 1.0) if bsdf else None; nose.data.materials.append(mat)"
                        },
                    },
                    {
                        "id": "save_blend_file",
                        "order": 11,
                        "name": "Save as scout2.blend",
                        "type": "mcp_tool",
                        "mcp_server": "blender",
                        "arguments": {
                            "script": "import bpy; bpy.ops.wm.save_as_mainfile(filepath=r'D:\\Models\\scout2.blend')"
                        },
                    },
                    {
                        "id": "import_scout_model",
                        "order": 12,
                        "name": "Import Scout Model to Unity",
                        "type": "mcp_tool",
                        "mcp_server": "unity3d",
                        "tool_name": "import_3d_model",
                        "arguments": {"model_path": r"D:\Models\scout2.blend"},
                    },
                    {
                        "id": "create_light",
                        "order": 13,
                        "name": "Create Pretty Spotlight",
                        "type": "mcp_tool",
                        "mcp_server": "unity3d",
                        "tool_name": "create_unity_light",
                        "arguments": {
                            "light_name": "PrettySpotlight",
                            "light_type": "Spot",
                            "color": [1.0, 1.0, 0.8, 1.0],
                            "intensity": 2.0,
                            "position": {"x": 2.0, "y": 5.0, "z": -2.0},
                        },
                    },
                ],
                "variables": [
                    {
                        "name": "model_path",
                        "type": "file_path",
                        "default_value": "D:/Models/test_scout.blend",
                        "description": "Path to Pilot Labs Scout model",
                        "required": True,
                    }
                ],
            },
            {
                "id": "vroid_to_vrchat",
                "name": "VRoid to VRChat Avatar",
                "description": "Complete pipeline from VRoid creation to VRChat upload",
                "category": "avatar",
                "steps": [
                    {
                        "id": "step1",
                        "order": 1,
                        "name": "Select VRoid Source",
                        "type": "condition",
                        "condition": {
                            "expression": "${vroid_source}",
                            "true_branch": ["step2a"],
                            "false_branch": ["step2b"],
                        },
                    },
                    {
                        "id": "step2a",
                        "order": 2,
                        "name": "Export VRM from VRoid Studio",
                        "type": "mcp_tool",
                        "mcp_server": "vroidstudio",
                        "tool_name": "vroid_export",
                        "arguments": {"operation": "export_vrm", "file_path": "${vroid_file_path}"},
                    },
                    {
                        "id": "step2b",
                        "order": 2,
                        "name": "Launch VRoid Studio",
                        "type": "app_launch",
                        "app_id": "vroid",
                    },
                    {
                        "id": "step3",
                        "order": 3,
                        "name": "Import to Unity3D",
                        "type": "mcp_tool",
                        "mcp_server": "unity3d",
                        "tool_name": "unity_vrm",
                        "arguments": {"operation": "import_vrm", "vrm_path": "${vrm_file_path}"},
                    },
                    {
                        "id": "step4",
                        "order": 4,
                        "name": "Upload to VRChat",
                        "type": "mcp_tool",
                        "mcp_server": "vrchat",
                        "tool_name": "vrchat_avatar",
                        "arguments": {
                            "operation": "upload_avatar",
                            "avatar_path": "${avatar_build_path}",
                        },
                    },
                ],
                "variables": [
                    {
                        "name": "vroid_source",
                        "type": "string",
                        "required": True,
                        "description": "Source type: 'file' or 'create'",
                    },
                    {
                        "name": "vroid_file_path",
                        "type": "file_path",
                        "required": False,
                        "description": "Path to VRoid file",
                    },
                ],
            },
            {
                "id": "create_virtual_robot",
                "name": "Create Virtual Robot",
                "description": "Create vbot with 3D model and deploy to environments",
                "category": "vbot",
                "steps": [
                    {
                        "id": "step1",
                        "order": 1,
                        "name": "Convert to VRM",
                        "type": "mcp_tool",
                        "mcp_server": "avatar",
                        "tool_name": "avatar_convert",
                        "arguments": {
                            "operation": "convert_to_vrm",
                            "input_file": "${model_file_path}",
                        },
                    },
                    {
                        "id": "step2",
                        "order": 2,
                        "name": "Create Virtual Robot",
                        "type": "mcp_tool",
                        "mcp_server": "robotics",
                        "tool_name": "vbot_crud",
                        "arguments": {
                            "operation": "create",
                            "name": "${robot_name}",
                            "robot_type": "${robot_type}",
                        },
                    },
                ],
                "variables": [
                    {
                        "name": "robot_name",
                        "type": "string",
                        "required": True,
                        "description": "Name of the virtual robot",
                    },
                    {
                        "name": "robot_type",
                        "type": "string",
                        "required": True,
                        "description": "Robot type: scout, go2, g1, custom",
                    },
                    {
                        "name": "model_file_path",
                        "type": "file_path",
                        "required": True,
                        "description": "Path to 3D model file",
                    },
                ],
            },
            {
                "id": "vrm_to_vrchat_with_osc",
                "name": "VRM to VRChat with OSC Control",
                "description": "Import VRM from avatar-mcp, process in Unity3D, upload to VRChat, and connect via OSC",
                "category": "avatar",
                "version": "1.0.0",
                "author": "Robotics Workflow System",
                "tags": ["avatar", "vrchat", "unity3d", "osc", "vrm"],
                "steps": [
                    {
                        "id": "step1",
                        "order": 1,
                        "name": "List Available VRMs",
                        "type": "mcp_tool",
                        "mcp_server": "avatar",
                        "tool_name": "avatar_management",
                        "arguments": {
                            "operation": "list_avatars",
                            "source_path": "D:\\Dev\\repos\\avatar-mcp\\models",
                        },
                        "required": True,
                        "output_variable": "available_vrms",
                    },
                    {
                        "id": "step2",
                        "order": 2,
                        "name": "Select VRM File",
                        "type": "mcp_tool",
                        "mcp_server": "avatar",
                        "tool_name": "avatar_management",
                        "arguments": {
                            "operation": "get_avatar",
                            "avatar_id": "${vrm_name}",
                            "source_path": "D:\\Dev\\repos\\avatar-mcp\\models",
                        },
                        "required": True,
                        "output_variable": "selected_vrm_path",
                    },
                    {
                        "id": "step3",
                        "order": 3,
                        "name": "Launch Unity3D Editor",
                        "type": "app_launch",
                        "app_id": "unity3d",
                        "app_config": {
                            "desktop": 2,
                            "monitor": 1,
                            "project_path": "${unity_project_path}",
                            "fullscreen": False,
                        },
                        "required": True,
                    },
                    {
                        "id": "step4",
                        "order": 4,
                        "name": "Wait for Unity3D",
                        "type": "delay",
                        "arguments": {"delay": 5},
                        "required": False,
                    },
                    {
                        "id": "step5",
                        "order": 5,
                        "name": "Import VRM to Unity3D",
                        "type": "mcp_tool",
                        "mcp_server": "unity3d",
                        "tool_name": "unity_vrm_import",
                        "arguments": {
                            "operation": "import_vrm",
                            "vrm_path": "${selected_vrm_path}",
                            "project_path": "${unity_project_path}",
                            "create_prefab": True,
                        },
                        "required": True,
                        "output_variable": "unity_prefab_path",
                    },
                    {
                        "id": "step6",
                        "order": 6,
                        "name": "Configure VRChat Avatar",
                        "type": "mcp_tool",
                        "mcp_server": "unity3d",
                        "tool_name": "unity_vrchat_sdk",
                        "arguments": {
                            "operation": "setup_vrchat_avatar",
                            "prefab_path": "${unity_prefab_path}",
                            "avatar_name": "${avatar_name}",
                            "add_descriptor": True,
                            "configure_expression_menu": True,
                            "configure_gesture_controller": True,
                        },
                        "required": True,
                        "output_variable": "vrchat_avatar_path",
                    },
                    {
                        "id": "step7",
                        "order": 7,
                        "name": "Build VRChat Avatar",
                        "type": "mcp_tool",
                        "mcp_server": "unity3d",
                        "tool_name": "unity_build",
                        "arguments": {
                            "operation": "build_avatar",
                            "avatar_path": "${vrchat_avatar_path}",
                            "output_path": "${build_output_path}",
                            "platform": "Windows",
                        },
                        "required": True,
                        "output_variable": "built_avatar_path",
                    },
                    {
                        "id": "step8",
                        "order": 8,
                        "name": "Upload to VRChat",
                        "type": "mcp_tool",
                        "mcp_server": "vrchat",
                        "tool_name": "vrchat_avatar_upload",
                        "arguments": {
                            "operation": "upload_avatar",
                            "avatar_file": "${built_avatar_path}",
                            "avatar_name": "${avatar_name}",
                            "description": "${avatar_description}",
                        },
                        "required": True,
                        "output_variable": "vrchat_avatar_id",
                    },
                    {
                        "id": "step9",
                        "order": 9,
                        "name": "Launch VRChat",
                        "type": "app_launch",
                        "app_id": "vrchat",
                        "app_config": {"desktop": 2, "monitor": 2, "fullscreen": False},
                        "required": False,
                    },
                    {
                        "id": "step10",
                        "order": 10,
                        "name": "Connect OSC to VRChat",
                        "type": "mcp_tool",
                        "mcp_server": "osc",
                        "tool_name": "osc_connect",
                        "arguments": {
                            "operation": "connect",
                            "host": "127.0.0.1",
                            "port": 9000,
                            "protocol": "udp",
                        },
                        "required": True,
                        "output_variable": "osc_connection_id",
                    },
                    {
                        "id": "step11",
                        "order": 11,
                        "name": "Send Avatar Selection via OSC",
                        "type": "mcp_tool",
                        "mcp_server": "osc",
                        "tool_name": "osc_send",
                        "arguments": {
                            "operation": "send_message",
                            "connection_id": "${osc_connection_id}",
                            "address": "/avatar/change",
                            "args": ["${vrchat_avatar_id}"],
                        },
                        "required": False,
                    },
                ],
                "variables": [
                    {
                        "name": "vrm_name",
                        "type": "string",
                        "required": True,
                        "description": "Name of the VRM file to import (without .vrm extension)",
                        "source": "user_input",
                    },
                    {
                        "name": "unity_project_path",
                        "type": "file_path",
                        "required": True,
                        "default_value": "C:\\UnityProjects\\VRChatAvatars",
                        "description": "Path to Unity3D project for VRChat avatars",
                        "source": "user_input",
                    },
                    {
                        "name": "avatar_name",
                        "type": "string",
                        "required": True,
                        "description": "Name for the VRChat avatar",
                        "source": "user_input",
                    },
                    {
                        "name": "avatar_description",
                        "type": "string",
                        "required": False,
                        "default_value": "Avatar imported via workflow",
                        "description": "Description for the VRChat avatar",
                        "source": "user_input",
                    },
                    {
                        "name": "build_output_path",
                        "type": "file_path",
                        "required": False,
                        "default_value": "C:\\VRChatAvatars\\Builds",
                        "description": "Output path for built avatar files",
                        "source": "user_input",
                    },
                ],
                "error_handling": {
                    "on_error": "stop",
                    "retry_count": 2,
                    "rollback_steps": [],
                    "error_notification": True,
                },
                "metadata": {
                    "estimated_duration": "15-20 minutes",
                    "complexity": "high",
                    "requires": ["Unity3D", "VRChat SDK", "OSC-MCP"],
                },
            },
            {
                "id": "vrm_to_resonite_with_osc",
                "name": "VRM to Resonite with OSC Control",
                "description": "Import VRM from avatar-mcp, process in Unity3D, upload to Resonite, and connect via OSC",
                "category": "avatar",
                "version": "1.0.0",
                "author": "Robotics Workflow System",
                "tags": ["avatar", "resonite", "unity3d", "osc", "vrm"],
                "steps": [
                    {
                        "id": "step1",
                        "order": 1,
                        "name": "Get VRM from Avatar-MCP",
                        "type": "mcp_tool",
                        "mcp_server": "avatar",
                        "tool_name": "avatar_management",
                        "arguments": {
                            "operation": "get_avatar",
                            "avatar_id": "${vrm_name}",
                            "source_path": "D:\\Dev\\repos\\avatar-mcp\\models",
                        },
                        "required": True,
                        "output_variable": "vrm_file_path",
                    },
                    {
                        "id": "step2",
                        "order": 2,
                        "name": "Launch Unity3D Editor",
                        "type": "app_launch",
                        "app_id": "unity3d",
                        "app_config": {
                            "desktop": 2,
                            "monitor": 1,
                            "project_path": "${unity_project_path}",
                            "fullscreen": False,
                        },
                        "required": True,
                    },
                    {
                        "id": "step3",
                        "order": 3,
                        "name": "Wait for Unity3D",
                        "type": "delay",
                        "arguments": {"delay": 5},
                        "required": False,
                    },
                    {
                        "id": "step4",
                        "order": 4,
                        "name": "Import VRM to Unity3D",
                        "type": "mcp_tool",
                        "mcp_server": "unity3d",
                        "tool_name": "unity_vrm_import",
                        "arguments": {
                            "operation": "import_vrm",
                            "vrm_path": "${vrm_file_path}",
                            "project_path": "${unity_project_path}",
                            "create_prefab": True,
                        },
                        "required": True,
                        "output_variable": "unity_prefab_path",
                    },
                    {
                        "id": "step5",
                        "order": 5,
                        "name": "Configure Resonite Avatar",
                        "type": "mcp_tool",
                        "mcp_server": "unity3d",
                        "tool_name": "unity_resonite_sdk",
                        "arguments": {
                            "operation": "setup_resonite_avatar",
                            "prefab_path": "${unity_prefab_path}",
                            "avatar_name": "${avatar_name}",
                            "configure_rig": True,
                            "add_resonite_components": True,
                        },
                        "required": True,
                        "output_variable": "resonite_avatar_path",
                    },
                    {
                        "id": "step6",
                        "order": 6,
                        "name": "Build Resonite Avatar",
                        "type": "mcp_tool",
                        "mcp_server": "unity3d",
                        "tool_name": "unity_build",
                        "arguments": {
                            "operation": "build_avatar",
                            "avatar_path": "${resonite_avatar_path}",
                            "output_path": "${build_output_path}",
                            "platform": "Windows",
                        },
                        "required": True,
                        "output_variable": "built_avatar_path",
                    },
                    {
                        "id": "step7",
                        "order": 7,
                        "name": "Upload to Resonite",
                        "type": "mcp_tool",
                        "mcp_server": "resonite",
                        "tool_name": "resonite_avatar_upload",
                        "arguments": {
                            "operation": "upload_avatar",
                            "avatar_file": "${built_avatar_path}",
                            "avatar_name": "${avatar_name}",
                        },
                        "required": True,
                        "output_variable": "resonite_avatar_id",
                    },
                    {
                        "id": "step8",
                        "order": 8,
                        "name": "Launch Resonite",
                        "type": "app_launch",
                        "app_id": "resonite",
                        "app_config": {"desktop": 2, "monitor": 2, "fullscreen": False},
                        "required": False,
                    },
                    {
                        "id": "step9",
                        "order": 9,
                        "name": "Connect OSC to Resonite",
                        "type": "mcp_tool",
                        "mcp_server": "osc",
                        "tool_name": "osc_connect",
                        "arguments": {
                            "operation": "connect",
                            "host": "127.0.0.1",
                            "port": 9001,
                            "protocol": "udp",
                        },
                        "required": True,
                        "output_variable": "osc_connection_id",
                    },
                    {
                        "id": "step10",
                        "order": 10,
                        "name": "Send Avatar Selection via OSC",
                        "type": "mcp_tool",
                        "mcp_server": "osc",
                        "tool_name": "osc_send",
                        "arguments": {
                            "operation": "send_message",
                            "connection_id": "${osc_connection_id}",
                            "address": "/avatar/load",
                            "args": ["${resonite_avatar_id}"],
                        },
                        "required": False,
                    },
                ],
                "variables": [
                    {
                        "name": "vrm_name",
                        "type": "string",
                        "required": True,
                        "description": "Name of the VRM file to import (without .vrm extension)",
                        "source": "user_input",
                    },
                    {
                        "name": "unity_project_path",
                        "type": "file_path",
                        "required": True,
                        "default_value": "C:\\UnityProjects\\ResoniteAvatars",
                        "description": "Path to Unity3D project for Resonite avatars",
                        "source": "user_input",
                    },
                    {
                        "name": "avatar_name",
                        "type": "string",
                        "required": True,
                        "description": "Name for the Resonite avatar",
                        "source": "user_input",
                    },
                    {
                        "name": "build_output_path",
                        "type": "file_path",
                        "required": False,
                        "default_value": "C:\\ResoniteAvatars\\Builds",
                        "description": "Output path for built avatar files",
                        "source": "user_input",
                    },
                ],
                "error_handling": {
                    "on_error": "stop",
                    "retry_count": 2,
                    "rollback_steps": [],
                    "error_notification": True,
                },
                "metadata": {
                    "estimated_duration": "15-20 minutes",
                    "complexity": "high",
                    "requires": ["Unity3D", "Resonite SDK", "OSC-MCP"],
                },
            },
            {
                "id": "dual_platform_avatar_deploy",
                "name": "Dual Platform Avatar Deployment",
                "description": "Import VRM, deploy to both VRChat and Resonite simultaneously, then connect OSC to both",
                "category": "avatar",
                "version": "1.0.0",
                "author": "Robotics Workflow System",
                "tags": ["avatar", "vrchat", "resonite", "unity3d", "osc", "parallel"],
                "steps": [
                    {
                        "id": "step1",
                        "order": 1,
                        "name": "Get VRM from Avatar-MCP",
                        "type": "mcp_tool",
                        "mcp_server": "avatar",
                        "tool_name": "avatar_management",
                        "arguments": {
                            "operation": "get_avatar",
                            "avatar_id": "${vrm_name}",
                            "source_path": "D:\\Dev\\repos\\avatar-mcp\\models",
                        },
                        "required": True,
                        "output_variable": "vrm_file_path",
                    },
                    {
                        "id": "step2",
                        "order": 2,
                        "name": "Launch Unity3D Editor",
                        "type": "app_launch",
                        "app_id": "unity3d",
                        "app_config": {
                            "desktop": 2,
                            "monitor": 1,
                            "project_path": "${unity_project_path}",
                            "fullscreen": False,
                        },
                        "required": True,
                    },
                    {
                        "id": "step3",
                        "order": 3,
                        "name": "Wait for Unity3D",
                        "type": "delay",
                        "arguments": {"delay": 5},
                        "required": False,
                    },
                    {
                        "id": "step4",
                        "order": 4,
                        "name": "Import VRM to Unity3D",
                        "type": "mcp_tool",
                        "mcp_server": "unity3d",
                        "tool_name": "unity_vrm_import",
                        "arguments": {
                            "operation": "import_vrm",
                            "vrm_path": "${vrm_file_path}",
                            "project_path": "${unity_project_path}",
                            "create_prefab": True,
                        },
                        "required": True,
                        "output_variable": "unity_prefab_path",
                    },
                    {
                        "id": "step5",
                        "order": 5,
                        "name": "Configure VRChat Avatar",
                        "type": "mcp_tool",
                        "mcp_server": "unity3d",
                        "tool_name": "unity_vrchat_sdk",
                        "arguments": {
                            "operation": "setup_vrchat_avatar",
                            "prefab_path": "${unity_prefab_path}",
                            "avatar_name": "${avatar_name}_vrchat",
                            "add_descriptor": True,
                        },
                        "required": True,
                        "output_variable": "vrchat_avatar_path",
                    },
                    {
                        "id": "step6",
                        "order": 6,
                        "name": "Configure Resonite Avatar",
                        "type": "mcp_tool",
                        "mcp_server": "unity3d",
                        "tool_name": "unity_resonite_sdk",
                        "arguments": {
                            "operation": "setup_resonite_avatar",
                            "prefab_path": "${unity_prefab_path}",
                            "avatar_name": "${avatar_name}_resonite",
                            "configure_rig": True,
                        },
                        "required": True,
                        "output_variable": "resonite_avatar_path",
                    },
                    {
                        "id": "step7",
                        "order": 7,
                        "name": "Build VRChat Avatar",
                        "type": "mcp_tool",
                        "mcp_server": "unity3d",
                        "tool_name": "unity_build",
                        "arguments": {
                            "operation": "build_avatar",
                            "avatar_path": "${vrchat_avatar_path}",
                            "output_path": "${build_output_path}/vrchat",
                            "platform": "Windows",
                        },
                        "required": True,
                        "output_variable": "built_vrchat_avatar",
                    },
                    {
                        "id": "step8",
                        "order": 8,
                        "name": "Build Resonite Avatar",
                        "type": "mcp_tool",
                        "mcp_server": "unity3d",
                        "tool_name": "unity_build",
                        "arguments": {
                            "operation": "build_avatar",
                            "avatar_path": "${resonite_avatar_path}",
                            "output_path": "${build_output_path}/resonite",
                            "platform": "Windows",
                        },
                        "required": True,
                        "output_variable": "built_resonite_avatar",
                    },
                    {
                        "id": "step9",
                        "order": 9,
                        "name": "Upload to VRChat",
                        "type": "mcp_tool",
                        "mcp_server": "vrchat",
                        "tool_name": "vrchat_avatar_upload",
                        "arguments": {
                            "operation": "upload_avatar",
                            "avatar_file": "${built_vrchat_avatar}",
                            "avatar_name": "${avatar_name}_vrchat",
                        },
                        "required": True,
                        "output_variable": "vrchat_avatar_id",
                    },
                    {
                        "id": "step10",
                        "order": 10,
                        "name": "Upload to Resonite",
                        "type": "mcp_tool",
                        "mcp_server": "resonite",
                        "tool_name": "resonite_avatar_upload",
                        "arguments": {
                            "operation": "upload_avatar",
                            "avatar_file": "${built_resonite_avatar}",
                            "avatar_name": "${avatar_name}_resonite",
                        },
                        "required": True,
                        "output_variable": "resonite_avatar_id",
                    },
                    {
                        "id": "step11",
                        "order": 11,
                        "name": "Launch VRChat",
                        "type": "app_launch",
                        "app_id": "vrchat",
                        "app_config": {"desktop": 2, "monitor": 2, "fullscreen": False},
                        "required": False,
                    },
                    {
                        "id": "step12",
                        "order": 12,
                        "name": "Launch Resonite",
                        "type": "app_launch",
                        "app_id": "resonite",
                        "app_config": {"desktop": 2, "monitor": 3, "fullscreen": False},
                        "required": False,
                    },
                    {
                        "id": "step13",
                        "order": 13,
                        "name": "Connect OSC to VRChat",
                        "type": "mcp_tool",
                        "mcp_server": "osc",
                        "tool_name": "osc_connect",
                        "arguments": {
                            "operation": "connect",
                            "host": "127.0.0.1",
                            "port": 9000,
                            "protocol": "udp",
                            "name": "vrchat_connection",
                        },
                        "required": True,
                        "output_variable": "osc_vrchat_connection",
                    },
                    {
                        "id": "step14",
                        "order": 14,
                        "name": "Connect OSC to Resonite",
                        "type": "mcp_tool",
                        "mcp_server": "osc",
                        "tool_name": "osc_connect",
                        "arguments": {
                            "operation": "connect",
                            "host": "127.0.0.1",
                            "port": 9001,
                            "protocol": "udp",
                            "name": "resonite_connection",
                        },
                        "required": True,
                        "output_variable": "osc_resonite_connection",
                    },
                    {
                        "id": "step15",
                        "order": 15,
                        "name": "Send VRChat Avatar via OSC",
                        "type": "mcp_tool",
                        "mcp_server": "osc",
                        "tool_name": "osc_send",
                        "arguments": {
                            "operation": "send_message",
                            "connection_id": "${osc_vrchat_connection}",
                            "address": "/avatar/change",
                            "args": ["${vrchat_avatar_id}"],
                        },
                        "required": False,
                    },
                    {
                        "id": "step16",
                        "order": 16,
                        "name": "Send Resonite Avatar via OSC",
                        "type": "mcp_tool",
                        "mcp_server": "osc",
                        "tool_name": "osc_send",
                        "arguments": {
                            "operation": "send_message",
                            "connection_id": "${osc_resonite_connection}",
                            "address": "/avatar/load",
                            "args": ["${resonite_avatar_id}"],
                        },
                        "required": False,
                    },
                ],
                "variables": [
                    {
                        "name": "vrm_name",
                        "type": "string",
                        "required": True,
                        "description": "Name of the VRM file to import (without .vrm extension)",
                        "source": "user_input",
                    },
                    {
                        "name": "unity_project_path",
                        "type": "file_path",
                        "required": True,
                        "default_value": "C:\\UnityProjects\\MultiPlatformAvatars",
                        "description": "Path to Unity3D project",
                        "source": "user_input",
                    },
                    {
                        "name": "avatar_name",
                        "type": "string",
                        "required": True,
                        "description": "Base name for the avatar (will be suffixed with _vrchat and _resonite)",
                        "source": "user_input",
                    },
                    {
                        "name": "build_output_path",
                        "type": "file_path",
                        "required": False,
                        "default_value": "C:\\Avatars\\Builds",
                        "description": "Output path for built avatar files",
                        "source": "user_input",
                    },
                ],
                "error_handling": {
                    "on_error": "continue",
                    "retry_count": 2,
                    "rollback_steps": ["step4"],
                    "error_notification": True,
                },
                "metadata": {
                    "estimated_duration": "20-25 minutes",
                    "complexity": "very_high",
                    "requires": ["Unity3D", "VRChat SDK", "Resonite SDK", "OSC-MCP"],
                    "parallel_steps": ["step7", "step8", "step9", "step10"],
                },
            },
            {
                "id": "worldlabs_splat_to_resonite",
                "name": "WorldLabs.ai Splat to Resonite",
                "description": "Download Gaussian splat from worldlabs.ai, import into Unity3D, and export to Resonite",
                "category": "avatar",
                "version": "1.0.0",
                "author": "Robotics Workflow System",
                "tags": [
                    "splat",
                    "worldlabs",
                    "resonite",
                    "unity3d",
                    "gaussian",
                    "neural_radiance",
                ],
                "steps": [
                    {
                        "id": "step1",
                        "order": 1,
                        "name": "Search WorldLabs.ai for Splat",
                        "type": "mcp_tool",
                        "mcp_server": "web",
                        "tool_name": "web_scraper",
                        "arguments": {
                            "operation": "search",
                            "url": "https://worldlabs.ai",
                            "query": "${splat_search_query}",
                            "content_type": "splat",
                        },
                        "required": True,
                        "output_variable": "splat_search_results",
                    },
                    {
                        "id": "step2",
                        "order": 2,
                        "name": "Select Splat from Results",
                        "type": "mcp_tool",
                        "mcp_server": "web",
                        "tool_name": "web_scraper",
                        "arguments": {
                            "operation": "extract",
                            "url": "${splat_url}",
                            "extract_type": "splat_metadata",
                        },
                        "required": True,
                        "output_variable": "splat_metadata",
                    },
                    {
                        "id": "step3",
                        "order": 3,
                        "name": "Download Splat File",
                        "type": "mcp_tool",
                        "mcp_server": "web",
                        "tool_name": "web_downloader",
                        "arguments": {
                            "operation": "download",
                            "url": "${splat_download_url}",
                            "output_path": "${download_output_path}",
                            "file_type": "splat",
                        },
                        "required": True,
                        "output_variable": "splat_file_path",
                    },
                    {
                        "id": "step4",
                        "order": 4,
                        "name": "Extract Splat Archive",
                        "type": "mcp_tool",
                        "mcp_server": "file",
                        "tool_name": "file_operations",
                        "arguments": {
                            "operation": "extract",
                            "archive_path": "${splat_file_path}",
                            "output_path": "${extracted_splat_path}",
                            "format": "zip",
                        },
                        "required": True,
                        "output_variable": "extracted_splat_path",
                    },
                    {
                        "id": "step5",
                        "order": 5,
                        "name": "Launch Unity3D Editor",
                        "type": "app_launch",
                        "app_id": "unity3d",
                        "app_config": {
                            "desktop": 2,
                            "monitor": 1,
                            "project_path": "${unity_project_path}",
                            "fullscreen": False,
                        },
                        "required": True,
                    },
                    {
                        "id": "step6",
                        "order": 6,
                        "name": "Wait for Unity3D",
                        "type": "delay",
                        "arguments": {"delay": 5},
                        "required": False,
                    },
                    {
                        "id": "step7",
                        "order": 7,
                        "name": "Import Splat to Unity3D",
                        "type": "mcp_tool",
                        "mcp_server": "unity3d",
                        "tool_name": "unity_splat_import",
                        "arguments": {
                            "operation": "import_splat",
                            "splat_path": "${extracted_splat_path}",
                            "project_path": "${unity_project_path}",
                            "splat_name": "${splat_name}",
                            "import_settings": {
                                "scale": "${splat_scale}",
                                "position": "${splat_position}",
                                "rotation": "${splat_rotation}",
                            },
                        },
                        "required": True,
                        "output_variable": "unity_splat_object",
                    },
                    {
                        "id": "step8",
                        "order": 8,
                        "name": "Configure Splat for Resonite",
                        "type": "mcp_tool",
                        "mcp_server": "unity3d",
                        "tool_name": "unity_resonite_sdk",
                        "arguments": {
                            "operation": "setup_resonite_splat",
                            "splat_object": "${unity_splat_object}",
                            "splat_name": "${splat_name}",
                            "configure_lighting": True,
                            "configure_collision": True,
                            "optimize_for_resonite": True,
                        },
                        "required": True,
                        "output_variable": "resonite_splat_path",
                    },
                    {
                        "id": "step9",
                        "order": 9,
                        "name": "Build Splat for Resonite",
                        "type": "mcp_tool",
                        "mcp_server": "unity3d",
                        "tool_name": "unity_build",
                        "arguments": {
                            "operation": "build_splat",
                            "splat_path": "${resonite_splat_path}",
                            "output_path": "${build_output_path}",
                            "platform": "Windows",
                            "build_settings": {"compression": "high", "optimize": True},
                        },
                        "required": True,
                        "output_variable": "built_splat_path",
                    },
                    {
                        "id": "step10",
                        "order": 10,
                        "name": "Upload Splat to Resonite",
                        "type": "mcp_tool",
                        "mcp_server": "resonite",
                        "tool_name": "resonite_world_upload",
                        "arguments": {
                            "operation": "upload_splat",
                            "splat_file": "${built_splat_path}",
                            "world_name": "${world_name}",
                            "splat_name": "${splat_name}",
                            "description": "${splat_description}",
                            "tags": "${splat_tags}",
                        },
                        "required": True,
                        "output_variable": "resonite_splat_id",
                    },
                    {
                        "id": "step11",
                        "order": 11,
                        "name": "Launch Resonite",
                        "type": "app_launch",
                        "app_id": "resonite",
                        "app_config": {"desktop": 2, "monitor": 2, "fullscreen": False},
                        "required": False,
                    },
                    {
                        "id": "step12",
                        "order": 12,
                        "name": "Load Splat in Resonite",
                        "type": "mcp_tool",
                        "mcp_server": "resonite",
                        "tool_name": "resonite_world_management",
                        "arguments": {
                            "operation": "load_splat",
                            "splat_id": "${resonite_splat_id}",
                            "world_name": "${world_name}",
                        },
                        "required": False,
                    },
                ],
                "variables": [
                    {
                        "name": "splat_search_query",
                        "type": "string",
                        "required": True,
                        "description": "Search query for splat on worldlabs.ai",
                        "source": "user_input",
                    },
                    {
                        "name": "splat_url",
                        "type": "string",
                        "required": False,
                        "description": "Direct URL to splat page on worldlabs.ai (if known)",
                        "source": "user_input",
                    },
                    {
                        "name": "splat_download_url",
                        "type": "string",
                        "required": False,
                        "description": "Direct download URL for splat file (auto-detected if not provided)",
                        "source": "step_output",
                    },
                    {
                        "name": "download_output_path",
                        "type": "file_path",
                        "required": False,
                        "default_value": "C:\\Splats\\Downloads",
                        "description": "Path to download splat files",
                        "source": "user_input",
                    },
                    {
                        "name": "extracted_splat_path",
                        "type": "file_path",
                        "required": False,
                        "default_value": "C:\\Splats\\Extracted",
                        "description": "Path for extracted splat files",
                        "source": "user_input",
                    },
                    {
                        "name": "unity_project_path",
                        "type": "file_path",
                        "required": True,
                        "default_value": "C:\\UnityProjects\\ResoniteSplats",
                        "description": "Path to Unity3D project for Resonite splats",
                        "source": "user_input",
                    },
                    {
                        "name": "splat_name",
                        "type": "string",
                        "required": True,
                        "description": "Name for the splat in Unity3D and Resonite",
                        "source": "user_input",
                    },
                    {
                        "name": "splat_scale",
                        "type": "number",
                        "required": False,
                        "default_value": 1.0,
                        "description": "Scale factor for splat import (default: 1.0)",
                        "source": "user_input",
                    },
                    {
                        "name": "splat_position",
                        "type": "string",
                        "required": False,
                        "default_value": "0,0,0",
                        "description": "Position for splat in Unity3D (format: x,y,z)",
                        "source": "user_input",
                    },
                    {
                        "name": "splat_rotation",
                        "type": "string",
                        "required": False,
                        "default_value": "0,0,0",
                        "description": "Rotation for splat in Unity3D (format: x,y,z)",
                        "source": "user_input",
                    },
                    {
                        "name": "build_output_path",
                        "type": "file_path",
                        "required": False,
                        "default_value": "C:\\Splats\\Builds",
                        "description": "Output path for built splat files",
                        "source": "user_input",
                    },
                    {
                        "name": "world_name",
                        "type": "string",
                        "required": False,
                        "default_value": "My Splat World",
                        "description": "Name of the Resonite world to upload splat to",
                        "source": "user_input",
                    },
                    {
                        "name": "splat_description",
                        "type": "string",
                        "required": False,
                        "default_value": "Gaussian splat imported from worldlabs.ai",
                        "description": "Description for the splat in Resonite",
                        "source": "user_input",
                    },
                    {
                        "name": "splat_tags",
                        "type": "string",
                        "required": False,
                        "default_value": "splat,gaussian,worldlabs",
                        "description": "Comma-separated tags for the splat",
                        "source": "user_input",
                    },
                ],
                "error_handling": {
                    "on_error": "stop",
                    "retry_count": 3,
                    "retry_delay": 2.0,
                    "rollback_steps": ["step3", "step4"],
                    "error_notification": True,
                },
                "metadata": {
                    "estimated_duration": "25-30 minutes",
                    "complexity": "very_high",
                    "requires": [
                        "Unity3D",
                        "Resonite SDK",
                        "Web Scraper MCP",
                        "File Operations MCP",
                    ],
                    "notes": "Requires Gaussian Splat importer plugin for Unity3D and Resonite SDK",
                },
            },
        ]
