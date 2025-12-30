"""Workflow storage service for robotics-mcp."""

import json
import sqlite3
from datetime import datetime
from pathlib import Path
from typing import Any
from uuid import uuid4

import structlog

from .workflow_models import Workflow, WorkflowExecution, ExecutionStatus

logger = structlog.get_logger(__name__)


class WorkflowStorage:
    """Workflow storage service using SQLite."""

    def __init__(self, db_path: str | Path | None = None):
        """Initialize workflow storage.

        Args:
            db_path: Path to SQLite database file. Defaults to workflows.db in data directory.
        """
        if db_path is None:
            # Default to data/workflows.db in project root
            data_dir = Path(__file__).parent.parent.parent.parent / "data"
            data_dir.mkdir(exist_ok=True)
            db_path = data_dir / "workflows.db"
        self.db_path = Path(db_path)
        self._init_database()

    def _init_database(self):
        """Initialize database schema."""
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()

        # Workflows table
        cursor.execute(
            """
            CREATE TABLE IF NOT EXISTS workflows (
                id TEXT PRIMARY KEY,
                name TEXT NOT NULL,
                description TEXT,
                category TEXT NOT NULL,
                version TEXT NOT NULL,
                author TEXT,
                tags TEXT,
                workflow_data TEXT NOT NULL,
                created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                execution_count INTEGER DEFAULT 0,
                success_count INTEGER DEFAULT 0,
                last_executed_at TIMESTAMP
            )
        """
        )

        # Workflow executions table
        cursor.execute(
            """
            CREATE TABLE IF NOT EXISTS workflow_executions (
                id TEXT PRIMARY KEY,
                workflow_id TEXT NOT NULL,
                status TEXT NOT NULL,
                variables TEXT,
                started_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                completed_at TIMESTAMP,
                error_message TEXT,
                current_step_id TEXT,
                FOREIGN KEY (workflow_id) REFERENCES workflows(id)
            )
        """
        )

        # Execution steps table
        cursor.execute(
            """
            CREATE TABLE IF NOT EXISTS workflow_execution_steps (
                id TEXT PRIMARY KEY,
                execution_id TEXT NOT NULL,
                step_id TEXT NOT NULL,
                step_name TEXT NOT NULL,
                status TEXT NOT NULL,
                started_at TIMESTAMP,
                completed_at TIMESTAMP,
                output_data TEXT,
                error_message TEXT,
                retry_count INTEGER DEFAULT 0,
                FOREIGN KEY (execution_id) REFERENCES workflow_executions(id)
            )
        """
        )

        conn.commit()
        conn.close()
        logger.info("Workflow database initialized", db_path=str(self.db_path))

    def create_workflow(self, workflow: Workflow) -> Workflow:
        """Create a new workflow.

        Args:
            workflow: Workflow definition.

        Returns:
            Created workflow with generated ID if not provided.
        """
        if not workflow.id:
            workflow.id = str(uuid4())

        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()

        cursor.execute(
            """
            INSERT INTO workflows (id, name, description, category, version, author, tags, workflow_data)
            VALUES (?, ?, ?, ?, ?, ?, ?, ?)
        """,
            (
                workflow.id,
                workflow.name,
                workflow.description,
                workflow.category.value,
                workflow.version,
                workflow.author,
                json.dumps(workflow.tags),
                workflow.model_dump_json(),
            ),
        )

        conn.commit()
        conn.close()
        logger.info("Workflow created", workflow_id=workflow.id, name=workflow.name)
        return workflow

    def get_workflow(self, workflow_id: str) -> Workflow | None:
        """Get workflow by ID.

        Args:
            workflow_id: Workflow identifier.

        Returns:
            Workflow if found, None otherwise.
        """
        conn = sqlite3.connect(self.db_path)
        conn.row_factory = sqlite3.Row
        cursor = conn.cursor()

        cursor.execute("SELECT workflow_data FROM workflows WHERE id = ?", (workflow_id,))
        row = cursor.fetchone()
        conn.close()

        if row:
            return Workflow.from_json(row["workflow_data"])
        return None

    def update_workflow(self, workflow_id: str, workflow: Workflow) -> Workflow:
        """Update existing workflow.

        Args:
            workflow_id: Workflow identifier.
            workflow: Updated workflow definition.

        Returns:
            Updated workflow.
        """
        workflow.id = workflow_id  # Ensure ID matches

        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()

        cursor.execute(
            """
            UPDATE workflows
            SET name = ?, description = ?, category = ?, version = ?, author = ?, tags = ?, workflow_data = ?, updated_at = CURRENT_TIMESTAMP
            WHERE id = ?
        """,
            (
                workflow.name,
                workflow.description,
                workflow.category.value,
                workflow.version,
                workflow.author,
                json.dumps(workflow.tags),
                workflow.model_dump_json(),
                workflow_id,
            ),
        )

        conn.commit()
        conn.close()
        logger.info("Workflow updated", workflow_id=workflow_id)
        return workflow

    def delete_workflow(self, workflow_id: str) -> bool:
        """Delete workflow.

        Args:
            workflow_id: Workflow identifier.

        Returns:
            True if deleted, False if not found.
        """
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()

        cursor.execute("DELETE FROM workflows WHERE id = ?", (workflow_id,))
        deleted = cursor.rowcount > 0

        conn.commit()
        conn.close()

        if deleted:
            logger.info("Workflow deleted", workflow_id=workflow_id)
        return deleted

    def list_workflows(
        self,
        category: str | None = None,
        tags: list[str] | None = None,
        search: str | None = None,
    ) -> list[Workflow]:
        """List workflows with optional filtering.

        Args:
            category: Filter by category.
            tags: Filter by tags (workflow must have all tags).
            search: Search in name and description.

        Returns:
            List of matching workflows.
        """
        conn = sqlite3.connect(self.db_path)
        conn.row_factory = sqlite3.Row
        cursor = conn.cursor()

        query = "SELECT workflow_data FROM workflows WHERE 1=1"
        params: list[Any] = []

        if category:
            query += " AND category = ?"
            params.append(category)

        if search:
            query += " AND (name LIKE ? OR description LIKE ?)"
            search_pattern = f"%{search}%"
            params.extend([search_pattern, search_pattern])

        cursor.execute(query, params)
        rows = cursor.fetchall()
        conn.close()

        workflows: list[Workflow] = []
        for row in rows:
            workflow = Workflow.from_json(row["workflow_data"])
            # Filter by tags if specified
            if tags:
                if all(tag in workflow.tags for tag in tags):
                    workflows.append(workflow)
            else:
                workflows.append(workflow)

        return workflows

    def increment_execution_count(self, workflow_id: str, success: bool = True):
        """Increment workflow execution statistics.

        Args:
            workflow_id: Workflow identifier.
            success: Whether execution was successful.
        """
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()

        if success:
            cursor.execute(
                """
                UPDATE workflows
                SET execution_count = execution_count + 1,
                    success_count = success_count + 1,
                    last_executed_at = CURRENT_TIMESTAMP
                WHERE id = ?
            """,
                (workflow_id,),
            )
        else:
            cursor.execute(
                """
                UPDATE workflows
                SET execution_count = execution_count + 1,
                    last_executed_at = CURRENT_TIMESTAMP
                WHERE id = ?
            """,
                (workflow_id,),
            )

        conn.commit()
        conn.close()

    def create_execution(self, workflow_id: str, variables: dict[str, Any]) -> str:
        """Create workflow execution record.

        Args:
            workflow_id: Workflow identifier.
            variables: Execution variables.

        Returns:
            Execution ID.
        """
        execution_id = str(uuid4())

        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()

        cursor.execute(
            """
            INSERT INTO workflow_executions (id, workflow_id, status, variables)
            VALUES (?, ?, ?, ?)
        """,
            (execution_id, workflow_id, ExecutionStatus.RUNNING.value, json.dumps(variables)),
        )

        conn.commit()
        conn.close()
        logger.info("Workflow execution created", execution_id=execution_id, workflow_id=workflow_id)
        return execution_id

    def update_execution_status(
        self,
        execution_id: str,
        status: ExecutionStatus,
        error_message: str | None = None,
        current_step_id: str | None = None,
    ):
        """Update execution status.

        Args:
            execution_id: Execution identifier.
            status: New status.
            error_message: Error message if failed.
            current_step_id: Current step ID.
        """
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()

        completed_at = datetime.now().isoformat() if status in [ExecutionStatus.COMPLETED, ExecutionStatus.FAILED, ExecutionStatus.CANCELLED] else None

        cursor.execute(
            """
            UPDATE workflow_executions
            SET status = ?, error_message = ?, current_step_id = ?, completed_at = ?
            WHERE id = ?
        """,
            (status.value, error_message, current_step_id, completed_at, execution_id),
        )

        conn.commit()
        conn.close()

    def get_execution(self, execution_id: str) -> WorkflowExecution | None:
        """Get execution record.

        Args:
            execution_id: Execution identifier.

        Returns:
            Execution record if found, None otherwise.
        """
        conn = sqlite3.connect(self.db_path)
        conn.row_factory = sqlite3.Row
        cursor = conn.cursor()

        cursor.execute("SELECT * FROM workflow_executions WHERE id = ?", (execution_id,))
        row = cursor.fetchone()

        if not row:
            conn.close()
            return None

        # Get step results
        cursor.execute(
            "SELECT * FROM workflow_execution_steps WHERE execution_id = ? ORDER BY started_at",
            (execution_id,),
        )
        step_rows = cursor.fetchall()
        conn.close()

        from .workflow_models import StepExecutionResult, StepExecutionStatus

        step_results: list[StepExecutionResult] = []
        for step_row in step_rows:
            step_results.append(
                StepExecutionResult(
                    step_id=step_row["step_id"],
                    step_name=step_row["step_name"],
                    status=StepExecutionStatus(step_row["status"]),
                    started_at=datetime.fromisoformat(step_row["started_at"]) if step_row["started_at"] else None,
                    completed_at=datetime.fromisoformat(step_row["completed_at"]) if step_row["completed_at"] else None,
                    output_data=json.loads(step_row["output_data"]) if step_row["output_data"] else None,
                    error_message=step_row["error_message"],
                    retry_count=step_row["retry_count"],
                )
            )

        return WorkflowExecution(
            id=row["id"],
            workflow_id=row["workflow_id"],
            status=ExecutionStatus(row["status"]),
            variables=json.loads(row["variables"]) if row["variables"] else {},
            step_results=step_results,
            started_at=datetime.fromisoformat(row["started_at"]),
            completed_at=datetime.fromisoformat(row["completed_at"]) if row["completed_at"] else None,
            error_message=row["error_message"],
            current_step_id=row["current_step_id"],
        )

    def add_step_result(
        self,
        execution_id: str,
        step_result: "StepExecutionResult",
    ):
        """Add step execution result.

        Args:
            execution_id: Execution identifier.
            step_result: Step execution result.
        """
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()

        step_id = str(uuid4())

        cursor.execute(
            """
            INSERT INTO workflow_execution_steps (
                id, execution_id, step_id, step_name, status, started_at, completed_at, output_data, error_message, retry_count
            )
            VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
        """,
            (
                step_id,
                execution_id,
                step_result.step_id,
                step_result.step_name,
                step_result.status.value,
                step_result.started_at.isoformat() if step_result.started_at else None,
                step_result.completed_at.isoformat() if step_result.completed_at else None,
                json.dumps(step_result.output_data) if step_result.output_data else None,
                step_result.error_message,
                step_result.retry_count,
            ),
        )

        conn.commit()
        conn.close()
