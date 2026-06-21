# Robotics MCP Server

<p align="center">
  <a href="https://github.com/casey/just"><img src="https://img.shields.io/badge/just-ready_to_go-7c5cfc?style=flat-square&logo=just&logoColor=white" alt="Just"></a>
  <a href="https://github.com/astral-sh/ruff"><img src="https://img.shields.io/endpoint?url=https://raw.githubusercontent.com/astral-sh/ruff/main/assets/badge/v2.json" alt="Ruff"></a>
  <a href="https://python.org"><img src="https://img.shields.io/badge/Python-3.13+-3776AB?style=flat-square&logo=python&logoColor=white" alt="Python"></a>
  <a href="https://biomejs.dev"><img src="https://img.shields.io/badge/Linted_with-Biome-60a5fa?style=flat-square&logo=biome&logoColor=white" alt="Biome"></a>
  <a href="https://github.com/PrefectHQ/fastmcp"><img src="https://img.shields.io/badge/FastMCP-3.2-7c5cfc?style=flat-square" alt="FastMCP"></a>
</p>


> 📖 **[Installation Guide](INSTALL.md)** — quick start, manual setup, and troubleshooting

**Author:** FlowEngineer sandraschi

One MCP server for physical robots (Dreame vacuums, Yahboom ROS, Elegoo, ), Gazebo, and virtual stacks (Unity, VRChat). FastMCP, **stdio** + **HTTP**. Optional **watchfiles** restarts for dev or unattended runs (not an SLA).

| Doc | Contents |
|-----|----------|
| **[Technical README](docs/README_TECHNICAL.md)** | Prerequisites, hardware, install, packaging, Cursor, config, composed MCPs, testing, troubleshooting |
| **[Usage and API](docs/README_USAGE.md)** | Tool examples, web UI, HTTP endpoints |
| **[Android Doctrine](docs/doctrine/README.md)** | Safety, legal, and operational standards for the android fleet |
| **[Setup Prerequisites](docs/guides/SETUP_PREREQUISITES.md)** | Unity, VRChat, peer MCP servers when you need the full stack |
| **[Crash protection (watchfiles)](WATCHFILES_README.md)** | Auto-restart and health checks |

**Status:** Industrial **v1.4.1** (2026-04-14) | SOTA v1.4.1 Compliance | Benny Protocol Secured.

[![SOTA](https://img.shields.io/badge/SOTA-v1.4.1-magenta)](https://github.com/FlowEngineer/sota)
[![Benny Protocol](https://img.shields.io/badge/Security-Wurst--Auth-red)]()
[![FastMCP](https://img.shields.io/badge/FastMCP-3.2+-blue)](https://goFastMCP 3.1.0com)
[![Biome](https://img.shields.io/badge/Lint-Biome-yellow)](https://biomejs.dev)
[![Python](https://img.shields.io/badge/Python-3.12+-green)](https://www.python.org)
[![License](https://img.shields.io/badge/License-MIT-gray)](LICENSE)

## Quick Start

```powershell
git clone https://github.com/sandraschi/robotics-mcp
cd robotics-mcp
just
```

This opens an interactive dashboard showing all available commands. Run `just bootstrap` to install dependencies, then `just serve` or `just dev` to start.

### Manual Setup

If you don't have `just` installed:


## Install

```bash
uvx robotics-mcp
```

```bash
pip install robotics-mcp
```

Development clone: see [Installation](docs/README_TECHNICAL.md#installation) in the technical README.

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md).


## 🛡️ Industrial Quality Stack

This project adheres to **SOTA 14.1** industrial standards for high-fidelity agentic orchestration:

- **Python (Core)**: [Ruff](https://astral.sh/ruff) for linting and formatting. Zero-tolerance for `print` statements in core handlers (`T201`).
- **Webapp (UI)**: [Biome](https://biomejs.dev/) for sub-millisecond linting. Strict `noConsoleLog` enforcement.
- **Protocol Compliance**: Hardened `stdout/stderr` isolation to ensure crash-resistant JSON-RPC communication.
- **Automation**: [Justfile](./justfile) recipes for all fleet operations (`just lint`, `just fix`, `just dev`).
- **Security**: Automated audits via `bandit` and `safety`.

## License

MIT  see [LICENSE](LICENSE).

## Acknowledgments

FastMCP; ROS community; Unity3D, VRChat, World Labs Marble/Chisel; MCP ecosystem contributors.
