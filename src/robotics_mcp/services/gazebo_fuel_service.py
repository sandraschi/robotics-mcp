"""Gazebo Fuel model service - Browse, search, download models from fuel.gazebosim.org.

Provides a Python interface to the Gazebo Fuel REST API for:
- Searching and browsing the 3000+ model library
- Downloading models to local Gazebo model path
- Spawning downloaded models into running Gazebo simulations via ROS

Fuel API base: https://fuel.gazebosim.org/1.0
"""

import shutil
import zipfile
from io import BytesIO
from pathlib import Path
from typing import Any

import aiohttp
import structlog

logger = structlog.get_logger(__name__)

FUEL_API_BASE = "https://fuel.gazebosim.org/1.0"
FUEL_THUMBNAIL_BASE = "https://fuel.gazebosim.org"

# Default local model storage (Gazebo convention)
DEFAULT_MODEL_DIR = Path.home() / ".gz" / "fuel" / "fuel.gazebosim.org"

# Popular categories for quick browsing
CATEGORIES = [
    "Robots",
    "Vehicles",
    "Buildings",
    "Furniture",
    "Electronics",
    "Animals",
    "People",
    "Nature",
    "Signs",
    "Tools",
    "Warehouse",
]

# Curated popular models for featured section
FEATURED_OWNERS = ["OpenRobotics", "chapulina", "OpenAI"]


async def search_models(
    query: str = "",
    owner: str = "",
    category: str = "",
    page: int = 1,
    per_page: int = 20,
    sort: str = "",
) -> dict[str, Any]:
    """Search Gazebo Fuel models.

    Args:
        query: Search text (name, description, tags)
        owner: Filter by owner (e.g., "OpenRobotics")
        category: Filter by category (e.g., "Robots")
        page: Page number (1-based)
        per_page: Results per page (max 100)
        sort: Sort order (empty = newest first)

    Returns:
        Dict with models list and pagination info.
    """
    params: dict[str, str | int] = {
        "page": page,
        "per_page": min(per_page, 100),
    }
    if query:
        params["q"] = query
    if category:
        params["category"] = category

    url = FUEL_API_BASE
    url = f"{FUEL_API_BASE}/{owner}/models" if owner else f"{FUEL_API_BASE}/models"

    try:
        async with (
            aiohttp.ClientSession() as session,
            session.get(url, params=params, timeout=aiohttp.ClientTimeout(total=15)) as resp,
        ):
            if resp.status != 200:
                return {"error": f"Fuel API returned {resp.status}", "models": []}

            raw_models = await resp.json()
            # Normalize thumbnail URLs
            models = []
            for m in raw_models:
                thumb = m.get("thumbnail_url", "")
                if thumb and not thumb.startswith("http"):
                    thumb = f"{FUEL_THUMBNAIL_BASE}{thumb}"
                models.append(
                    {
                        "name": m.get("name", ""),
                        "owner": m.get("owner", ""),
                        "description": m.get("description", ""),
                        "downloads": m.get("downloads", 0),
                        "likes": m.get("likes", 0),
                        "filesize": m.get("filesize", 0),
                        "filesize_mb": round(m.get("filesize", 0) / (1024 * 1024), 2),
                        "thumbnail_url": thumb,
                        "license_name": m.get("license_name", ""),
                        "categories": m.get("categories", []),
                        "tags": m.get("tags", []),
                        "created_at": m.get("createdAt", ""),
                        "updated_at": m.get("updatedAt", ""),
                        "fuel_url": f"https://app.gazebosim.org/{m.get('owner', '')}/fuel/models/{m.get('name', '')}",
                        "download_url": f"{FUEL_API_BASE}/{m.get('owner', '')}/models/{m.get('name', '')}/tip/files",
                    }
                )

            return {
                "models": models,
                "page": page,
                "per_page": per_page,
                "total_returned": len(models),
                "query": query,
                "owner": owner,
                "category": category,
            }

    except TimeoutError:
        return {"error": "Fuel API timeout", "models": []}
    except Exception as e:
        logger.error("Fuel API search failed", error=str(e))
        return {"error": str(e), "models": []}


async def get_model_details(owner: str, model_name: str) -> dict[str, Any]:
    """Get detailed model info from Fuel.

    Args:
        owner: Model owner (e.g., "OpenRobotics")
        model_name: Model name (e.g., "TurtleBot")

    Returns:
        Model details dict.
    """
    url = f"{FUEL_API_BASE}/{owner}/models/{model_name}"
    try:
        async with (
            aiohttp.ClientSession() as session,
            session.get(url, timeout=aiohttp.ClientTimeout(total=10)) as resp,
        ):
            if resp.status != 200:
                return {"error": f"Model not found ({resp.status})"}
            data = await resp.json()
            thumb = data.get("thumbnail_url", "")
            if thumb and not thumb.startswith("http"):
                thumb = f"{FUEL_THUMBNAIL_BASE}{thumb}"
            return {
                "name": data.get("name", ""),
                "owner": data.get("owner", ""),
                "description": data.get("description", ""),
                "downloads": data.get("downloads", 0),
                "likes": data.get("likes", 0),
                "filesize": data.get("filesize", 0),
                "filesize_mb": round(data.get("filesize", 0) / (1024 * 1024), 2),
                "thumbnail_url": thumb,
                "license_name": data.get("license_name", ""),
                "categories": data.get("categories", []),
                "tags": data.get("tags", []),
                "fuel_url": f"https://app.gazebosim.org/{owner}/fuel/models/{model_name}",
            }
    except Exception as e:
        return {"error": str(e)}


async def download_model(
    owner: str,
    model_name: str,
    model_dir: str | None = None,
) -> dict[str, Any]:
    """Download a model from Fuel to local Gazebo model path.

    Downloads the model zip and extracts it to the standard Gazebo fuel
    directory (~/.gz/fuel/fuel.gazebosim.org/<owner>/models/<name>/).

    Args:
        owner: Model owner
        model_name: Model name
        model_dir: Override model directory (default: ~/.gz/fuel/...)

    Returns:
        Download result with local path.
    """
    base_dir = Path(model_dir) if model_dir else DEFAULT_MODEL_DIR
    target_dir = base_dir / owner / "models" / model_name

    # Check if already downloaded
    if target_dir.exists() and any(target_dir.rglob("*.sdf")):
        return {
            "success": True,
            "already_exists": True,
            "local_path": str(target_dir),
            "owner": owner,
            "model_name": model_name,
        }

    download_url = f"{FUEL_API_BASE}/{owner}/models/{model_name}/tip/{model_name}.zip"

    try:
        async with (
            aiohttp.ClientSession() as session,
            session.get(download_url, timeout=aiohttp.ClientTimeout(total=120)) as resp,
        ):
            if resp.status != 200:
                # Try alternative URL format
                alt_url = f"{FUEL_API_BASE}/{owner}/models/{model_name}/tip/files"
                async with session.get(alt_url, timeout=aiohttp.ClientTimeout(total=120)) as resp2:
                    if resp2.status != 200:
                        return {
                            "success": False,
                            "error": f"Download failed: HTTP {resp.status} / {resp2.status}",
                            "message": f"Download failed: HTTP {resp.status} / {resp2.status}",
                            "owner": owner,
                            "model_name": model_name,
                        }
                    content = await resp2.read()
            else:
                content = await resp.read()

        # Extract zip
        target_dir.mkdir(parents=True, exist_ok=True)
        try:
            with zipfile.ZipFile(BytesIO(content)) as zf:
                zf.extractall(target_dir)
        except zipfile.BadZipFile:
            # Not a zip - might be raw SDF, write as model.sdf
            sdf_path = target_dir / "model.sdf"
            sdf_path.write_bytes(content)

        logger.info("Model downloaded", owner=owner, model=model_name, path=str(target_dir))

        return {
            "success": True,
            "already_exists": False,
            "local_path": str(target_dir),
            "owner": owner,
            "model_name": model_name,
            "size_bytes": len(content),
        }

    except TimeoutError:
        return {"success": False, "error": "Download timeout (120s)", "message": "Download timeout (120s)"}
    except Exception as e:
        logger.error("Model download failed", owner=owner, model=model_name, error=str(e))
        return {"success": False, "error": str(e), "message": str(e)}


def list_local_models(model_dir: str | None = None) -> list[dict[str, Any]]:
    """List locally downloaded Gazebo Fuel models.

    Args:
        model_dir: Override model directory

    Returns:
        List of local model info dicts.
    """
    base_dir = Path(model_dir) if model_dir else DEFAULT_MODEL_DIR
    models = []

    if not base_dir.exists():
        return models

    for owner_dir in base_dir.iterdir():
        if not owner_dir.is_dir():
            continue
        models_dir = owner_dir / "models"
        if not models_dir.exists():
            continue
        for model_path in models_dir.iterdir():
            if not model_path.is_dir():
                continue
            sdf_files = list(model_path.rglob("*.sdf"))
            models.append(
                {
                    "name": model_path.name,
                    "owner": owner_dir.name,
                    "local_path": str(model_path),
                    "has_sdf": len(sdf_files) > 0,
                    "sdf_files": [str(f) for f in sdf_files],
                    "size_bytes": sum(f.stat().st_size for f in model_path.rglob("*") if f.is_file()),
                }
            )

    return models


def delete_local_model(owner: str, model_name: str, model_dir: str | None = None) -> dict[str, Any]:
    """Delete a locally downloaded model.

    Args:
        owner: Model owner
        model_name: Model name
        model_dir: Override model directory

    Returns:
        Deletion result.
    """
    base_dir = Path(model_dir) if model_dir else DEFAULT_MODEL_DIR
    target_dir = base_dir / owner / "models" / model_name

    if not target_dir.exists():
        return {"success": False, "error": "Model not found locally", "message": "Model not found locally"}

    try:
        shutil.rmtree(target_dir)
        return {"success": True, "deleted": str(target_dir)}
    except Exception as e:
        return {"success": False, "error": str(e), "message": str(e)}


def get_spawn_sdf(owner: str, model_name: str, model_dir: str | None = None) -> str | None:
    """Get the SDF content for spawning a model in Gazebo.

    Args:
        owner: Model owner
        model_name: Model name
        model_dir: Override model directory

    Returns:
        SDF XML string or None if not found.
    """
    base_dir = Path(model_dir) if model_dir else DEFAULT_MODEL_DIR
    target_dir = base_dir / owner / "models" / model_name

    if not target_dir.exists():
        return None

    # Try common SDF locations
    for candidate in [
        target_dir / "model.sdf",
        target_dir / f"{model_name}.sdf",
        target_dir / "1" / "model.sdf",
    ]:
        if candidate.exists():
            return candidate.read_text(encoding="utf-8", errors="replace")

    # Fallback: find any .sdf
    sdf_files = list(target_dir.rglob("*.sdf"))
    if sdf_files:
        return sdf_files[0].read_text(encoding="utf-8", errors="replace")

    return None
