"""Model marketplace — curated index + LLM search for robot models."""

from fastmcp import Context

MODEL_CATALOG = {
    "unitree_go2": {
        "name": "Unitree Go2",
        "type": "quadruped",
        "formats": ["mjcf", "urdf"],
        "source": "https://github.com/unitreerobotics/unitree_mujoco",
        "external_path": "D:/Dev/repos/external/unitree_mujoco/data/go2",
        "mjcf": "D:/Dev/repos/external/unitree_mujoco/data/go2/go2.xml",
        "sim_backends": ["mujoco", "gazebo"],
        "tags": ["quadruped", "unitree", "locomotion", "rl"],
        "joint_count": 12,
        "actuator_count": 12,
    },
    "unitree_h1": {
        "name": "Unitree H1",
        "type": "humanoid",
        "formats": ["mjcf", "urdf"],
        "source": "https://github.com/unitreerobotics/unitree_mujoco",
        "external_path": "D:/Dev/repos/external/unitree_mujoco/data/h1",
        "mjcf": "D:/Dev/repos/external/unitree_mujoco/data/h1/h1.xml",
        "sim_backends": ["mujoco", "isaac"],
        "tags": ["humanoid", "unitree", "biped", "rl"],
        "joint_count": 19,
        "actuator_count": 19,
    },
    "limx_tron1_pf": {
        "name": "LimX TRON 1 (PointFoot)",
        "type": "biped",
        "formats": ["urdf", "xml"],
        "source": "https://github.com/limxdynamics/humanoid-description",
        "mjcf": None,
        "sim_backends": ["mujoco", "limx"],
        "tags": ["biped", "limx", "tron1", "pointfoot"],
        "joint_count": 10,
        "actuator_count": 10,
    },
    "limx_oli_d04": {
        "name": "LimX Oli (D04)",
        "type": "humanoid",
        "formats": ["urdf", "xml"],
        "source": "https://github.com/limxdynamics/humanoid-description",
        "mjcf": None,
        "sim_backends": ["mujoco", "limx"],
        "tags": ["humanoid", "limx", "oli", "loco-manipulation"],
        "joint_count": 27,
        "actuator_count": 27,
    },
}


def _filter_catalog(
    query: str | None = None,
    tags: list[str] | None = None,
    sim_backend: str | None = None,
    robot_type: str | None = None,
) -> list[tuple[str, dict]]:
    """Apply filters and keyword search against MODEL_CATALOG."""
    candidates = list(MODEL_CATALOG.items())

    if robot_type:
        candidates = [(k, v) for k, v in candidates if v.get("type") == robot_type]

    if sim_backend:
        candidates = [
            (k, v)
            for k, v in candidates
            if sim_backend in [b.lower() for b in v.get("sim_backends", [])] or sim_backend in v.get("sim_backends", [])
        ]

    if tags:
        tag_set = {t.lower() for t in tags}
        candidates = [(k, v) for k, v in candidates if tag_set & {t.lower() for t in v.get("tags", [])}]

    if query:
        q = query.lower()
        candidates = [
            (k, v)
            for k, v in candidates
            if q in v.get("name", "").lower()
            or q in k.lower()
            or any(q in t.lower() for t in v.get("tags", []))
            or any(q in f.lower() for f in v.get("formats", []))
        ]

    return candidates


def _build_match(model_id: str, entry: dict) -> dict:
    return {
        "id": model_id,
        "name": entry["name"],
        "type": entry["type"],
        "tags": entry["tags"],
        "sim_backends": entry["sim_backends"],
        "formats": entry["formats"],
        "joint_count": entry["joint_count"],
        "actuator_count": entry["actuator_count"],
    }


async def sim_marketplace_search(
    query: str | None = None,
    tags: list[str] | None = None,
    sim_backend: str | None = None,
    type: str | None = None,
    ctx: Context | None = None,
) -> dict:
    """Search the model marketplace — curated index + LLM fallback for fuzzy matching.

    Filters by keyword, tags, sim backend, or robot type.
    When exact match returns nothing and ctx is available, falls back to
    an LLM to suggest the closest model from the catalog.

    ## Return Format
    {"success": bool, "message": str, "matches": [{
        "id": str, "name": str, "type": str,
        "tags": [str], "sim_backends": [str], "formats": [str],
        "joint_count": int, "actuator_count": int
    }], "total": int, "llm_fallback_used": bool}

    ## Examples
    sim_marketplace_search(query="go2")
    sim_marketplace_search(tags=["humanoid"])
    sim_marketplace_search(sim_backend="gazebo")
    sim_marketplace_search(type="biped")
    """
    matches = _filter_catalog(query, tags, sim_backend, type)
    llm_fallback_used = False

    if not matches and ctx is not None and query:
        catalog_text = "\n".join(
            f"  - {mid}: {e['name']} ({e['type']}) — tags: {', '.join(e['tags'])}" for mid, e in MODEL_CATALOG.items()
        )
        prompt = (
            f"User searched the robot model marketplace with query: '{query}'. "
            f"Available models:\n{catalog_text}\n"
            "Return ONLY the model ID (slug) of the closest match, or 'NONE' if nothing fits."
        )
        try:
            import httpx

            resp = httpx.post(
                "http://127.0.0.1:11434/api/generate",
                json={"model": "llama3.2:3b", "prompt": prompt, "stream": False},
                timeout=15,
            )
            suggestion = resp.json().get("response", "").strip().lower()
            if suggestion != "none" and suggestion in MODEL_CATALOG:
                entry = MODEL_CATALOG[suggestion]
                matches = [(suggestion, entry)]
                llm_fallback_used = True
        except Exception:
            ...

    match_list = [_build_match(mid, entry) for mid, entry in matches]
    return {
        "success": True,
        "message": f"Found {len(match_list)} model{'s' if len(match_list) != 1 else ''}"
        f"{' matching ' + repr(query) if query else ''}.",
        "matches": match_list,
        "total": len(match_list),
        "llm_fallback_used": llm_fallback_used,
    }


async def sim_marketplace_info(model_id: str) -> dict:
    """Get full metadata for a specific robot model, including file paths and compatible sim backends.

    ## Return Format
    {"success": bool, "message": str, "model": {"id": str, "name": str, ...} | None}

    ## Examples
    sim_marketplace_info("unitree_go2")
    sim_marketplace_info("limx_tron1_pf")
    """
    if model_id not in MODEL_CATALOG:
        available = list(MODEL_CATALOG.keys())
        return {
            "success": False,
            "message": f"Model '{model_id}' not found. Available: {', '.join(available)}.",
            "model": None,
        }

    entry = MODEL_CATALOG[model_id]
    return {
        "success": True,
        "message": f"Found model '{entry['name']}'.",
        "model": {
            "id": model_id,
            **entry,
        },
    }
