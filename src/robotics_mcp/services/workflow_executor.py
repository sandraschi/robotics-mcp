"""Workflow execution engine for robotics-mcp."""

import asyncio
import re
from datetime import datetime
from typing import Any

import structlog

from .workflow_models import (
    ExecutionStatus,
    StepExecutionResult,
    StepExecutionStatus,
    StepType,
    Workflow,
    WorkflowExecution,
)
from .workflow_storage import WorkflowStorage

logger = structlog.get_logger(__name__)


class WorkflowExecutor:
    """Workflow execution engine."""

    def __init__(
        self,
        storage: WorkflowStorage,
        mcp_client_helper: Any | None = None,
        app_launcher: Any | None = None,
    ):
        """Initialize workflow executor.

        Args:
            storage: Workflow storage service.
            mcp_client_helper: Helper for calling MCP tools.
            app_launcher: Application launcher service.
        """
        self.storage = storage
        self.mcp_client_helper = mcp_client_helper
        self.app_launcher = app_launcher
        self.running_executions: dict[str, asyncio.Task] = {}

    def _substitute_variables(self, text: str, variables: dict[str, Any], step_outputs: dict[str, Any]) -> str:
        """Substitute variables in text.

        Args:
            text: Text with variable references.
            variables: User-provided variables.
            step_outputs: Step output variables.

        Returns:
            Text with variables substituted.
        """
        # Pattern: ${variable_name} or ${step_id.output.field}
        pattern = r"\$\{([^}]+)\}"

        def replace_var(match: re.Match) -> str:
            var_expr = match.group(1)

            # Handle step output references: step_id.output.field
            if "." in var_expr:
                parts = var_expr.split(".")
                if len(parts) == 3 and parts[1] == "output":
                    step_id, _, field = parts
                    if step_id in step_outputs:
                        output = step_outputs[step_id]
                        if isinstance(output, dict) and field in output:
                            return str(output[field])
                return match.group(0)  # Return original if not found

            # Handle built-in functions
            if var_expr == "timestamp":
                return datetime.now().isoformat()

            # Handle regular variables
            if var_expr in variables:
                return str(variables[var_expr])
            if var_expr in step_outputs:
                return str(step_outputs[var_expr])

            # Try environment variable
            import os

            if var_expr.startswith("env."):
                env_var = var_expr[4:]
                return str(os.getenv(env_var, match.group(0)))

            return match.group(0)  # Return original if not found

        return re.sub(pattern, replace_var, text)

    def _substitute_dict_variables(
        self, data: dict[str, Any], variables: dict[str, Any], step_outputs: dict[str, Any]
    ) -> dict[str, Any]:
        """Recursively substitute variables in dictionary.

        Args:
            data: Dictionary with potential variable references.
            variables: User-provided variables.
            step_outputs: Step output variables.

        Returns:
            Dictionary with variables substituted.
        """
        result: dict[str, Any] = {}
        for key, value in data.items():
            if isinstance(value, str):
                result[key] = self._substitute_variables(value, variables, step_outputs)
            elif isinstance(value, dict):
                result[key] = self._substitute_dict_variables(value, variables, step_outputs)
            elif isinstance(value, list):
                result[key] = [
                    self._substitute_dict_variables(item, variables, step_outputs) if isinstance(item, dict) else self._substitute_variables(item, variables, step_outputs) if isinstance(item, str) else item
                    for item in value
                ]
            else:
                result[key] = value
        return result

    async def _execute_mcp_tool_step(
        self, step: Any, variables: dict[str, Any], step_outputs: dict[str, Any]
    ) -> dict[str, Any]:
        """Execute MCP tool step.

        Args:
            step: Workflow step.
            variables: Execution variables.
            step_outputs: Previous step outputs.

        Returns:
            Step execution result.
        """
        if not self.mcp_client_helper:
            raise ValueError("MCP client helper not available")

        if not step.mcp_server or not step.tool_name:
            raise ValueError("MCP tool step requires mcp_server and tool_name")

        # Substitute variables in arguments
        arguments = step.arguments or {}
        substituted_args = self._substitute_dict_variables(arguments, variables, step_outputs)

        # Call MCP tool
        result = await self.mcp_client_helper(step.mcp_server, step.tool_name, substituted_args)

        return {"success": True, "data": result}

    async def _execute_app_launch_step(
        self, step: Any, variables: dict[str, Any], step_outputs: dict[str, Any]
    ) -> dict[str, Any]:
        """Execute application launch step.

        Args:
            step: Workflow step.
            variables: Execution variables.
            step_outputs: Previous step outputs.

        Returns:
            Step execution result.
        """
        if not self.app_launcher:
            raise ValueError("App launcher not available")

        if not step.app_id:
            raise ValueError("App launch step requires app_id")

        # Substitute variables in app config
        app_config = step.app_config or {}
        substituted_config = self._substitute_dict_variables(app_config, variables, step_outputs)

        # Launch application
        result = await self.app_launcher.launch_app(
            step.app_id,
            desktop_number=substituted_config.get("desktop"),
            project_path=substituted_config.get("project_path"),
            fullscreen=substituted_config.get("fullscreen", False),
            monitor=substituted_config.get("monitor"),
        )

        return {"success": result.get("success", False), "data": result}

    async def _execute_delay_step(self, step: Any) -> dict[str, Any]:
        """Execute delay step.

        Args:
            step: Workflow step.

        Returns:
            Step execution result.
        """
        delay_seconds = step.arguments.get("seconds", 1.0) if step.arguments else 1.0
        await asyncio.sleep(delay_seconds)
        return {"success": True, "data": {"delayed": delay_seconds}}

    async def _execute_step(
        self,
        step: Any,
        variables: dict[str, Any],
        step_outputs: dict[str, Any],
        execution_id: str,
    ) -> StepExecutionResult:
        """Execute a single workflow step.

        Args:
            step: Workflow step to execute.
            variables: Execution variables.
            step_outputs: Previous step outputs.
            execution_id: Execution identifier.

        Returns:
            Step execution result.
        """
        step_result = StepExecutionResult(
            step_id=step.id,
            step_name=step.name,
            status=StepExecutionStatus.RUNNING,
            started_at=datetime.now(),
        )

        try:
            # Execute based on step type
            if step.type == StepType.MCP_TOOL:
                result = await self._execute_mcp_tool_step(step, variables, step_outputs)
            elif step.type == StepType.APP_LAUNCH:
                result = await self._execute_app_launch_step(step, variables, step_outputs)
            elif step.type == StepType.DELAY:
                result = await self._execute_delay_step(step)
            else:
                raise ValueError(f"Unsupported step type: {step.type}")

            step_result.status = StepExecutionStatus.COMPLETED
            step_result.output_data = result
            step_result.completed_at = datetime.now()

            # Store output variable if specified
            if step.output_variable and result.get("success"):
                step_outputs[step.output_variable] = result.get("data", {})

        except Exception as e:
            logger.error("Step execution failed", step_id=step.id, error=str(e), exc_info=True)
            step_result.status = StepExecutionStatus.FAILED
            step_result.error_message = str(e)
            step_result.completed_at = datetime.now()

        # Save step result
        self.storage.add_step_result(execution_id, step_result)

        return step_result

    async def execute_workflow(
        self,
        workflow_id: str,
        variables: dict[str, Any],
        execution_id: str | None = None,
    ) -> str:
        """Execute workflow.

        Args:
            workflow_id: Workflow identifier.
            variables: Execution variables.
            execution_id: Optional execution ID (for resuming).

        Returns:
            Execution ID.
        """
        workflow = self.storage.get_workflow(workflow_id)
        if not workflow:
            raise ValueError(f"Workflow not found: {workflow_id}")

        # Create execution record if not provided
        if not execution_id:
            execution_id = self.storage.create_execution(workflow_id, variables)

        # Update status to running
        self.storage.update_execution_status(execution_id, ExecutionStatus.RUNNING)

        # Sort steps by order
        steps = sorted(workflow.steps, key=lambda s: s.order)
        step_outputs: dict[str, Any] = {}

        try:
            # Execute steps sequentially
            for step in steps:
                # Update current step
                self.storage.update_execution_status(execution_id, ExecutionStatus.RUNNING, current_step_id=step.id)

                # Execute step
                step_result = await self._execute_step(step, variables, step_outputs, execution_id)

                # Handle step failure
                if step_result.status == StepExecutionStatus.FAILED:
                    if step.required:
                        # Required step failed - stop workflow
                        self.storage.update_execution_status(
                            execution_id,
                            ExecutionStatus.FAILED,
                            error_message=f"Required step failed: {step.name} - {step_result.error_message}",
                        )
                        return execution_id
                    else:
                        # Optional step failed - continue
                        logger.warning("Optional step failed, continuing", step_id=step.id)

                # Handle conditional branching
                if step.condition:
                    # Evaluate condition
                    condition_result = self._evaluate_condition(step.condition, variables, step_outputs)
                    if condition_result:
                        # Follow true branch
                        next_step_ids = step.condition.true_branch
                    else:
                        # Follow false branch
                        next_step_ids = step.condition.false_branch or []

                    # Skip to next step in branch
                    if next_step_ids:
                        # Find next step by ID
                        next_step = next((s for s in steps if s.id in next_step_ids), None)
                        if next_step:
                            # Continue with next step
                            continue

            # All steps completed successfully
            self.storage.update_execution_status(execution_id, ExecutionStatus.COMPLETED)
            self.storage.increment_execution_count(workflow_id, success=True)

        except Exception as e:
            logger.error("Workflow execution failed", execution_id=execution_id, error=str(e), exc_info=True)
            self.storage.update_execution_status(execution_id, ExecutionStatus.FAILED, error_message=str(e))
            self.storage.increment_execution_count(workflow_id, success=False)

        return execution_id

    def _evaluate_condition(self, condition: Any, variables: dict[str, Any], step_outputs: dict[str, Any]) -> bool:
        """Evaluate condition expression.

        Args:
            condition: Condition to evaluate.
            variables: Execution variables.
            step_outputs: Step outputs.

        Returns:
            Condition result.
        """
        # Simple condition evaluation (can be enhanced with full expression parser)
        expr = condition.expression

        # Substitute variables
        expr = self._substitute_variables(expr, variables, step_outputs)

        # Simple comparison evaluation
        if "==" in expr:
            left, right = expr.split("==", 1)
            return left.strip() == right.strip().strip('"').strip("'")
        elif "!=" in expr:
            left, right = expr.split("!=", 1)
            return left.strip() != right.strip().strip('"').strip("'")

        # Default: evaluate as boolean
        try:
            return bool(eval(expr))
        except:
            return False

    async def pause_execution(self, execution_id: str):
        """Pause workflow execution.

        Args:
            execution_id: Execution identifier.
        """
        # TODO: Implement pause functionality
        self.storage.update_execution_status(execution_id, ExecutionStatus.PAUSED)

    async def resume_execution(self, execution_id: str):
        """Resume workflow execution.

        Args:
            execution_id: Execution identifier.
        """
        execution = self.storage.get_execution(execution_id)
        if not execution:
            raise ValueError(f"Execution not found: {execution_id}")

        workflow = self.storage.get_workflow(execution.workflow_id)
        if not workflow:
            raise ValueError(f"Workflow not found: {execution.workflow_id}")

        # Resume from current step
        # TODO: Implement resume functionality
        await self.execute_workflow(execution.workflow_id, execution.variables, execution_id)

    async def cancel_execution(self, execution_id: str):
        """Cancel workflow execution.

        Args:
            execution_id: Execution identifier.
        """
        self.storage.update_execution_status(execution_id, ExecutionStatus.CANCELLED)
