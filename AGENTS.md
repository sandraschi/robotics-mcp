# robotics-mcp — Agent Guide

## Overview
Unified robotics control via MCP - Physical and virtual robots (bot + vbot)

## Entry Points
- `uv run robotics-mcp` → `robotics_mcp.server:main`
- `uv run raspbot-setup` → `robotics_mcp.scripts.raspbot_setup:main`

## Standards
- FastMCP 3.2+ portmanteau tool pattern — tools use `operation` enum param
- Responses: structured dicts with `success`, `message`, domain-specific fields
- Dual transport: stdio (Claude Desktop) + HTTP (`MCP_TRANSPORT=http`)
- See [mcp-central-docs](https://github.com/sandraschi/mcp-central-docs) for fleet-wide coding standards

## Key Files
- `README.md` — full documentation
- `pyproject.toml` — build config and entry points
- `CLAUDE.md` — Claude Code context (if present)

Install docs: follow mcp-central-docs/standards/AGENT_INSTALL_REFERENCE.md
