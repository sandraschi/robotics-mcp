# Contributing to Robotics MCP

Thank you for your interest in contributing to Robotics MCP!

## Development Setup

```bash
# Clone repository
git clone https://github.com/sandraschi/robotics-mcp.git
cd robotics-mcp

# Create virtual environment
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install dependencies
pip install -e ".[dev]"

# Git hooks: Ruff + Biome (web_sota) on every commit
pre-commit install

# Run tests
pytest

# Check code quality
.\scripts\check-standards.ps1
```

With **uv** (recommended):

```bash
uv sync --extra dev
uv run pre-commit install
```

Manual run of all hooks (matches CI): `uv run pre-commit run --all-files`

## Code Standards

- **Python**: Ruff (lint + format); Black/isort listed for legacy scripts
- **Web (`web_sota/`)**: Biome (`biome ci` in pre-commit; same as `npm run lint`)
- **Type Checking**: mypy (non-strict)
- **Testing**: pytest with coverage

## Pull Request Process

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests
5. Run code quality checks
6. Submit pull request

## Project Structure

- `src/robotics_mcp/` - Main source code
- `tests/` - Test suite
- `docs/` - Documentation
- `scripts/` - Utility scripts
- `mcpb/` - MCPB packaging

## Questions?

Open an issue or contact the maintainers.

