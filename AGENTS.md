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
- `pipeline/README.md` — 10-stage CAD -> OpenUSD -> Isaac Lab -> ROS 2 -> Active Inference pipeline (`01_cad_to_usd_blender.py` .. `10_mujoco_active_inference_closed_loop.py`)
- `docs/gemini_distillations/INDEX.md` — 17-part robotics & Active Inference knowledge distillation
- `docs/hardware/` — Subsystem hardware blueprints for `saschibot` (`rust-saschirob-mcp`) and `Nori A3` (`norirobotics-mcp`)

Install docs: follow mcp-central-docs/standards/AGENT_INSTALL_REFERENCE.md
