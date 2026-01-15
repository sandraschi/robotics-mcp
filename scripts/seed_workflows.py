"""Seed database with workflows from templates."""

import asyncio
import sys
from pathlib import Path

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from robotics_mcp.services.workflow_storage import WorkflowStorage
from robotics_mcp.tools.workflow_management import WorkflowManagementTool
from robotics_mcp.services.workflow_models import Workflow


async def seed_workflows():
    """Create workflows from templates."""
    storage = WorkflowStorage()
    
    # Create a minimal tool instance just to get templates
    class MockMCP:
        pass
    
    tool = WorkflowManagementTool(MockMCP())
    templates = tool._get_templates()
    
    print(f"Found {len(templates)} templates")
    
    created_count = 0
    skipped_count = 0
    
    for template in templates:
        template_id = template.get("id")
        
        # Check if workflow already exists (by ID or name)
        existing_by_id = storage.get_workflow(template_id) if template_id else None
        if existing_by_id:
            print(f"  Skipping '{template['name']}' - already exists (ID: {template_id})")
            skipped_count += 1
            continue
        
        # Also check by name
        all_workflows = storage.list_workflows()
        existing_by_name = next((w for w in all_workflows if w.name == template.get("name")), None)
        if existing_by_name:
            print(f"  Skipping '{template['name']}' - already exists (ID: {existing_by_name.id})")
            skipped_count += 1
            continue
        
        # Create workflow from template
        try:
            workflow = Workflow(**template)
            created = storage.create_workflow(workflow)
            print(f"  Created '{created.name}' (ID: {created.id})")
            created_count += 1
        except Exception as e:
            print(f"  Failed to create '{template.get('name', 'unknown')}': {e}")
    
    print(f"\nSummary: Created {created_count}, Skipped {skipped_count}, Total templates: {len(templates)}")


if __name__ == "__main__":
    asyncio.run(seed_workflows())
