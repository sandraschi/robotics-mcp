import os

import structlog
from fastapi import HTTPException, Request, Security
from fastapi.security import APIKeyHeader
from starlette.responses import JSONResponse
from starlette.status import HTTP_403_FORBIDDEN

logger = structlog.get_logger(__name__)

# Standard SOTA Header
WURST_AUTH_HEADER = "X-Wurst-Auth"
# Fallback token for local dev if environment is not set
DEFAULT_BENNY_TOKEN = "benny-handshake-2026"

api_key_header = APIKeyHeader(name=WURST_AUTH_HEADER, auto_error=False)


async def wurst_auth_middleware(request: Request, call_next):
    """
    Middleware to verify the Benny/Wurstsemmel security handshake.

    Protects destructive robot movement or high-privilege hardware operations.
    """
    # Define sensitive paths that REQUIRE auth (e.g., robot control, movement)
    protected_paths = [
        "/mcp/move_robot",
        "/mcp/execute_command",
        "/api/robot/control",
        "/api/osc/bridge/start",
    ]

    path = request.url.path
    requires_auth = (path.endswith("/control") and request.method == "POST") or any(
        path.startswith(p) for p in protected_paths
    )

    if requires_auth:
        token = request.headers.get(WURST_AUTH_HEADER)
        expected_token = os.getenv("WURST_AUTH_TOKEN", DEFAULT_BENNY_TOKEN)

        if not token or token != expected_token:
            logger.warning(
                ":SECURITY: Unauthorized robot control attempt blocked",
                path=request.url.path,
                client=request.client.host if request.client else "unknown",
            )
            return JSONResponse(
                status_code=HTTP_403_FORBIDDEN,
                content={
                    "detail": "Security Handshake Failed: Invalid or missing X-Wurst-Auth header. Robotics operation aborted."
                },
            )

    response = await call_next(request)
    return response


async def get_wurst_token(api_key: str = Security(api_key_header)) -> str:
    """Dependency injection version for specific robotics routes."""
    expected_token = os.getenv("WURST_AUTH_TOKEN", DEFAULT_BENNY_TOKEN)
    if not api_key or api_key != expected_token:
        raise HTTPException(status_code=HTTP_403_FORBIDDEN, detail="Robotics Security Handshake Failed")
    return api_key
