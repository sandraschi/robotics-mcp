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
            ],
            workflow_id: str | None = None,
            workflow_data: dict[str, Any] | None = None,
            variables: dict[str, Any] | None = None,
            execution_id: str | None = None,
            category: str | None = None,
            tags: list[str] | None = None,
            search: str | None = None,
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
                        return format_error_response("workflow_id and workflow_data required for update operation")

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

                    return format_success_response({"message": f"Workflow {workflow_id} deleted successfully"})

                elif operation == "list":
                    workflows = self.storage.list_workflows(category=category, tags=tags, search=search)
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

                    execution_id = await self.executor.execute_workflow(workflow_id, variables)

                    return format_success_response(
                        {
                            "execution_id": execution_id,
                            "message": "Workflow execution started",
                        }
                    )

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
                logger.error("Workflow management operation failed", operation=operation, error=str(e), exc_info=True)
                return format_error_response(f"Operation failed: {str(e)}")

    def _get_templates(self) -> list[dict[str, Any]]:
        """Get built-in workflow templates.

        Returns:
            List of workflow templates.
        """
        return [
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
                        "arguments": {"operation": "upload_avatar", "avatar_path": "${avatar_build_path}"},
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
                        "arguments": {"operation": "convert_to_vrm", "input_file": "${model_file_path}"},
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
        ]
