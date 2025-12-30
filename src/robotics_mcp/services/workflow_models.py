"""Workflow data models for robotics-mcp."""

import json
from datetime import datetime
from enum import Enum
from typing import Any

from pydantic import BaseModel, Field


class WorkflowCategory(str, Enum):
    """Workflow categories."""

    AVATAR = "avatar"
    VBOT = "vbot"
    SYNC = "sync"
    CUSTOM = "custom"


class StepType(str, Enum):
    """Workflow step types."""

    MCP_TOOL = "mcp_tool"
    APP_LAUNCH = "app_launch"
    CONDITION = "condition"
    LOOP = "loop"
    PARALLEL = "parallel"
    DELAY = "delay"
    USER_INPUT = "user_input"


class ErrorHandlingStrategy(str, Enum):
    """Error handling strategies."""

    STOP = "stop"
    CONTINUE = "continue"
    RETRY = "retry"
    ROLLBACK = "rollback"


class VariableType(str, Enum):
    """Variable types."""

    STRING = "string"
    NUMBER = "number"
    BOOLEAN = "boolean"
    FILE_PATH = "file_path"
    MCP_RESPONSE = "mcp_response"


class VariableSource(str, Enum):
    """Variable sources."""

    USER_INPUT = "user_input"
    STEP_OUTPUT = "step_output"
    ENVIRONMENT = "environment"
    FILE = "file"


class WorkflowVariable(BaseModel):
    """Workflow variable definition."""

    name: str
    type: VariableType
    default_value: Any | None = None
    description: str = ""
    required: bool = False
    source: VariableSource | None = None


class RetryConfig(BaseModel):
    """Retry configuration for workflow steps."""

    max_retries: int = 3
    retry_delay: float = 1.0
    exponential_backoff: bool = False


class StepCondition(BaseModel):
    """Condition for workflow step branching."""

    expression: str
    true_branch: list[str] = Field(default_factory=list)
    false_branch: list[str] = Field(default_factory=list)


class AppLaunchConfig(BaseModel):
    """Application launch configuration."""

    desktop: int | None = None
    monitor: int | None = None
    project_path: str | None = None
    fullscreen: bool = False


class WorkflowStep(BaseModel):
    """Workflow step definition."""

    id: str
    order: int
    name: str
    type: StepType
    mcp_server: str | None = None
    tool_name: str | None = None
    arguments: dict[str, Any] | None = None
    app_id: str | None = None
    app_config: AppLaunchConfig | None = None
    condition: StepCondition | None = None
    on_success: list[str] = Field(default_factory=list)
    on_failure: list[str] = Field(default_factory=list)
    retry: RetryConfig | None = None
    timeout: float | None = None
    required: bool = True
    output_variable: str | None = None


class ErrorHandlingConfig(BaseModel):
    """Error handling configuration."""

    on_error: ErrorHandlingStrategy = ErrorHandlingStrategy.STOP
    retry_count: int = 0
    rollback_steps: list[str] = Field(default_factory=list)
    error_notification: bool = True


class Workflow(BaseModel):
    """Workflow definition."""

    id: str
    name: str
    description: str = ""
    category: WorkflowCategory
    version: str = "1.0.0"
    author: str = ""
    tags: list[str] = Field(default_factory=list)
    steps: list[WorkflowStep] = Field(default_factory=list)
    variables: list[WorkflowVariable] = Field(default_factory=list)
    error_handling: ErrorHandlingConfig = Field(default_factory=ErrorHandlingConfig)
    metadata: dict[str, Any] = Field(default_factory=dict)

    def model_dump_json(self) -> str:
        """Export workflow to JSON string."""
        return json.dumps(self.model_dump(), indent=2, default=str)

    @classmethod
    def from_json(cls, json_str: str) -> "Workflow":
        """Create workflow from JSON string."""
        data = json.loads(json_str)
        return cls(**data)


class ExecutionStatus(str, Enum):
    """Workflow execution status."""

    PENDING = "pending"
    RUNNING = "running"
    PAUSED = "paused"
    COMPLETED = "completed"
    FAILED = "failed"
    CANCELLED = "cancelled"


class StepExecutionStatus(str, Enum):
    """Step execution status."""

    PENDING = "pending"
    RUNNING = "running"
    COMPLETED = "completed"
    FAILED = "failed"
    SKIPPED = "skipped"


class StepExecutionResult(BaseModel):
    """Step execution result."""

    step_id: str
    step_name: str
    status: StepExecutionStatus
    started_at: datetime | None = None
    completed_at: datetime | None = None
    output_data: dict[str, Any] | None = None
    error_message: str | None = None
    retry_count: int = 0


class WorkflowExecution(BaseModel):
    """Workflow execution record."""

    id: str
    workflow_id: str
    status: ExecutionStatus
    variables: dict[str, Any] = Field(default_factory=dict)
    step_results: list[StepExecutionResult] = Field(default_factory=list)
    started_at: datetime
    completed_at: datetime | None = None
    error_message: str | None = None
    current_step_id: str | None = None
