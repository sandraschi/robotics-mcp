set windows-shell := ["powershell.exe", "-NoProfile", "-Command"]
import 'scripts/just/fleet.just'

REPO := justfile_directory()

# --- Dashboard — fleet standard (just --list = default) ---
default:
    @just --list

# Serve HTTP + MCP (dual) on fleet ports 10707/10706
serve:
    powershell.exe -NoProfile -File "{{justfile_directory()}}/scripts/just/serve.ps1" 2>$null; uv run robotics-mcp --http --port 10707

# Alias for format
fmt: fix

# Full CI gate: ruff + format --check + tsc + biome + pytest
ci:
    powershell.exe -NoProfile -File "{{justfile_directory()}}/scripts/just/ci.ps1" 2>$null; uv run ruff check .; uv run ruff format --check .; uv run pytest tests -q

# Fleet five-gate certification (ruff/biome style + pyright/tsc types + pytest)
gates-green:
    @just ci

certify: gates-green


# Synchronize deps, pre-commit hooks, and web frontend
bootstrap:
    uv sync --extra dev --group dev
    uv run pre-commit install
    Set-Location web_sota; npm ci; if ($LASTEXITCODE -ne 0) { npm install }
    Write-Host "Pre-commit hooks installed." -ForegroundColor Green
# --- Quality ---

# --- Execute Python  Ruff and Web  Biome linting sweep  use  biome ci matches npm run biome ci ---
lint:
    uv run ruff check .
    Set-Location web_sota; npx @biomejs/biome ci .

# Execute Python and Web formatting and automated fixes
fix:
    uv run ruff check . --fix --unsafe-fixes
    uv run ruff format .
    Set-Location web_sota; npx @biomejs/biome check --write .

# --- Testing ---

# Execute Pytest SOTA v1.4.1 test suite
test:
    uv run pytest --cov=src/robotics_mcp tests/

# --- Hardening ---

# Execute Bandit security audit
check-sec:
    uv run bandit -r src/

# Execute safety audit of dependencies
audit-deps:
    uv run safety check

# --- Rust Operations ---

# Execute Rust linting (Clippy)
lint-rs:
    cargo clippy --all-targets --all-features -- -D warnings

# Execute Rust formatting
fix-rs:
    cargo fmt --all

# --- Web UI ---

# Start frontend development server (Vite)
dev-ui:
    Set-Location web_sota; npm run dev

# Build production frontend bundle
build-ui:
    Set-Location web_sota; npm run build

# --- Tauri Native ---

# Build Tauri native desktop app (full pipeline: frontend + backend)
build-native:
    Set-Location '{{justfile_directory()}}\native'
    $env:Path = "$env:USERPROFILE\.cargo\bin;$env:Path"
    powershell.exe -NoProfile -File '{{justfile_directory()}}\native\build.ps1'

# --- Playwright E2E ---

# Install Playwright browsers (one-time)
e2e-install:
    cd {{REPO}}\web_sota
    npx playwright install chromium

# Run Playwright E2E smoke tests (start backend first: just serve)
e2e:
    cd {{REPO}}\web_sota
    npx playwright test


# Bootstrap: install dev deps + pre-commit hook
