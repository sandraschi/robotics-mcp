"""MCP tools for sim fleet orchestration — dispatches to mujoco-mcp, gazebo-mcp, isaac-mcp, limx-robotics-mcp."""

from typing import Annotated, Any

from fastmcp import Context, FastMCP
from pydantic import Field

from .marketplace import sim_marketplace_info, sim_marketplace_search
from .registry import SIM_BACKENDS
from .router import dispatch_sim, probe_all


def register_sim_orchestrator(mcp: FastMCP) -> None:
    @mcp.tool()
    async def sim_fleet_status() -> dict[str, Any]:
        """Probe all registered simulation MCP backends and report availability.

        Checks: mujoco-mcp (MuJoCo), gazebo-mcp (Gazebo),
        isaac-mcp (Isaac Sim), limx-robotics-mcp (LimX).
        Returns health data, strengths, and ports for each.
        """
        results = await probe_all()
        available = sum(1 for v in results.values() if v.get("available"))
        return {
            "success": True,
            "message": f"{available}/4 simulation backends available.",
            "backends": results,
        }

    @mcp.tool()
    async def sim_fleet_route(
        task: Annotated[str, Field(description="Description of the simulation task.")],
        preferred_backend: Annotated[
            str | None,
            Field(description="Prefer a specific backend: 'mujoco', 'gazebo', 'isaac', 'limx'."),
        ] = None,
        ctx: Context | None = None,
    ) -> dict[str, Any]:
        """Route a simulation task to the best available backend.

        Probes all running sim MCPs, picks the optimal one based on task
        keywords (sensors → Gazebo, rendering → Isaac, locomotion → MuJoCo,
        TRON 1/Oli → LimX), and returns the backend URL and metadata.
        The caller then makes the actual API call to that backend.
        """
        return await dispatch_sim(task, preferred_backend or "", ctx)

    @mcp.tool()
    async def sim_fleet_backends() -> dict[str, Any]:
        """List all registered sim backends and their capabilities (offline registry)."""
        return {
            "success": True,
            "message": f"{len(SIM_BACKENDS)} backends in registry.",
            "backends": {k: {kk: vv for kk, vv in v.items() if kk != "health"} for k, v in SIM_BACKENDS.items()},
        }

    @mcp.tool()
    async def sim_marketplace_search_tool(
        query: Annotated[str | None, Field(description="Keyword to search model names, tags, and formats.")] = None,
        tags: Annotated[
            list[str] | None, Field(description="Filter by tags (e.g. 'humanoid', 'biped', 'quadruped').")
        ] = None,
        sim_backend: Annotated[
            str | None,
            Field(description="Filter by compatible sim backend (e.g. 'mujoco', 'gazebo', 'isaac', 'limx')."),
        ] = None,
        type: Annotated[
            str | None, Field(description="Filter by robot type (e.g. 'humanoid', 'quadruped', 'biped').")
        ] = None,
        ctx: Context | None = None,
    ) -> dict:
        """Search the robot model marketplace for available models.

        Filters by keyword, tags, sim backend, or robot type.  When exact
        keyword match returns nothing, falls back to a local LLM (Ollama)
        to suggest the closest catalog entry.

        ## Return Format
        {"success": bool, "message": str, "matches": [{
            "id", "name", "type", "tags", "sim_backends", "formats",
            "joint_count", "actuator_count"
        }], "total": int, "llm_fallback_used": bool}

        ## Examples
        sim_marketplace_search_tool(query="go2")
        sim_marketplace_search_tool(tags=["humanoid", "unitree"])
        sim_marketplace_search_tool(sim_backend="gazebo")
        """
        return await sim_marketplace_search(query, tags, sim_backend, type, ctx)

    @mcp.tool()
    async def sim_marketplace_info_tool(
        model_id: Annotated[str, Field(description="Model ID slug, e.g. 'unitree_go2', 'limx_tron1_pf'.")],
    ) -> dict:
        """Get full metadata for a specific robot model from the marketplace.

        Returns file paths (MJCF, URDF), compatible sim backends, joint/actuator
        counts, and source URLs.

        ## Return Format
        {"success": bool, "message": str, "model": {"id": str, "name": str, ...} | None}

        ## Examples
        sim_marketplace_info_tool(model_id="unitree_go2")
        sim_marketplace_info_tool(model_id="limx_oli_d04")
        """
        return await sim_marketplace_info(model_id)
