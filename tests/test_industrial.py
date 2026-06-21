def test_health_check(api_client):
    """Verify that the industrialized health endpoint is alive."""
    response = api_client.get("/api/v1/health")
    assert response.status_code == 200
    assert response.json()["status"] == "healthy"


def test_wurst_auth_security(api_client):
    """Verify that the Benny Protocol (Wurst-Auth) blocks unauthorized access."""
    # Attempt to control a robot without headers
    response = api_client.post("/api/v1/robots/mock-id/control", json={"action": "move"})
    assert response.status_code == 403
    assert "Security Handshake Failed" in response.json()["detail"]


def test_wurst_auth_success(api_client, auth_headers, mock_robot):
    """Verify that valid Wurst-Auth headers grant access."""
    response = api_client.post(
        f"/api/v1/robots/{mock_robot}/control", headers=auth_headers, json={"action": "get_status"}
    )
    # 200 OK (or 500 if hardware mock fails, but not 403)
    assert response.status_code != 403


def test_telemetry_logs_memory(mcp_server):
    """Verify that the MemoryLogHandler is capturing telemetry."""
    import structlog

    from robotics_mcp.utils.logging_config import get_recent_logs

    test_log = structlog.get_logger("test_telemetry")
    test_log.info(":START: Ground control to Major Tom")

    logs = get_recent_logs(limit=1)
    assert len(logs) > 0
    assert "Major Tom" in logs[0]["message"]
