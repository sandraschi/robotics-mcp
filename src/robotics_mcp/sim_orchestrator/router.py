"""Sim MCP router - dispatches tasks to available simulation backends."""

import httpx
from fastmcp import Context

from .registry import SIM_BACKENDS


async def probe_backend(key: str) -> dict:
    """Quick health check for a sim backend."""
    info = SIM_BACKENDS[key]
    url = info["backend_url"] + info["health"]
    try:
        async with httpx.AsyncClient(timeout=5) as client:
            resp = await client.get(url)
            data = resp.json() if resp.status_code == 200 else {}
            return {"available": True, "status": "ok", "data": data, **info}
    except Exception as e:
        return {"available": False, "status": f"unreachable: {e}", **info}


async def probe_all() -> dict[str, dict]:
    """Probe all registered sim backends in parallel."""
    import asyncio

    results = await asyncio.gather(*[probe_backend(k) for k in SIM_BACKENDS])
    return dict(zip(SIM_BACKENDS.keys(), results, strict=True))


def _pick_backend(task: str, availability: dict[str, dict]) -> str:
    """Pick the best sim backend for a given task description."""
    task_lower = task.lower()
    if any(w in task_lower for w in ["photo", "render", "vision", "synthetic", "rtx"]):
        candidates = ["isaac"]
    elif any(w in task_lower for w in ["sensor", "lidar", "camera", "ros", "terrain", "outdoor"]):
        candidates = ["gazebo"]
    elif any(w in task_lower for w in ["contact", "grasp", "walk", "locomotion", "rl", "fast"]):
        candidates = ["mujoco", "isaac"]
    elif any(w in task_lower for w in ["limx", "tron1", "oli", "vla", "policy"]):
        candidates = ["limx", "mujoco"]
    elif any(w in task_lower for w in ["xr", "vr", "spatial", "avatar", "world", "resonite"]):
        candidates = ["resonite", "vrchat", "unity3d"]
    elif any(w in task_lower for w in ["unity", "game", "scene", "interactive", "3d viewer"]):
        candidates = ["unity3d", "resonite"]
    elif any(w in task_lower for w in ["vrchat", "social", "vrc", "user-generated"]):
        candidates = ["vrchat", "resonite"]
    elif any(w in task_lower for w in ["world", "landscape", "building", "environment", "scene gen"]):
        candidates = ["worldlabs", "blender", "unity3d"]
    elif any(w in task_lower for w in ["blender", "furniture", "modelling", "sculpt", "mesh", "asset"]):
        candidates = ["blender", "worldlabs"]
    else:
        candidates = ["mujoco", "gazebo", "isaac"]

    for c in candidates:
        if availability.get(c, {}).get("available"):
            return c
    return next((k for k, v in availability.items() if v.get("available")), "")


async def dispatch_sim(
    task: str,
    preferred_backend: str = "",
    ctx: Context | None = None,
) -> dict:
    """Route a simulation task to the best available backend.

    Probes all sim MCPs, picks the best one for the task, and returns
    the backend info so the caller can make the actual API call.
    """
    availability = await probe_all()
    available = {k: v for k, v in availability.items() if v.get("available")}

    if not available:
        # Fall back to Ollama for a planning response
        prompt = f"""The user wants to: {task}

No simulation backends are currently running. The available backends are:
- MuJoCo (port 11046, /health) - fast physics, differentiable, GPU parallel
- Gazebo (port 10991, /health) - sensors, ROS, terrains
- Isaac Sim (port 11049, /health) - photorealistic, NVIDIA GPU
- LimX Robotics (port 11044, /health) - TRON 1, Oli, VLA policies
- Resonite (port 10979, /health) - XR worlds, spatial, avatars
- VRChat (port 10712, /health) - social VR, user-generated worlds, avatars
- Unity 3D (port 10831, /health) - real-time 3D, interactive scenes
- World Labs (port 10865, /health) - 3D world generation, image/text-to-world
- Blender (port 10849, /health) - 3D modelling, furniture, assets, sculpting

Advise which backend to start and what port it runs on.
"""
        try:
            import httpx as hx

            resp = hx.post(
                "http://127.0.0.1:11434/api/generate",
                json={"model": "llama3.2:3b", "prompt": prompt, "stream": False},
                timeout=30,
            )
            advice = resp.json().get("response", "")
        except Exception:
            advice = "No LLM available to advise."
        return {
            "success": False,
            "message": "No simulation backends available.",
            "available_backends": {k: v["status"] for k, v in availability.items()},
            "advice": advice,
        }

    backend_key = preferred_backend if preferred_backend in available else _pick_backend(task, availability)
    backend = available[backend_key]

    return {
        "success": True,
        "message": f"Routing to {backend['name']}.",
        "selected_backend": backend_key,
        "backend_info": {
            "name": backend["name"],
            "backend_url": backend["backend_url"],
            "api_base": backend["backend_url"],
            "health_data": backend.get("data", {}),
            "strengths": backend["strengths"],
        },
        "all_available": list(available.keys()),
        "availability": {k: v["status"] for k, v in availability.items()},
    }
