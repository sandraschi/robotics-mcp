# PowerShell script to check code standards

$ErrorActionPreference = "Stop"

Write-Host "Checking code standards..." -ForegroundColor Cyan

# Check if tools are installed
$tools = @("black", "ruff", "mypy")
$missing = @()

foreach ($tool in $tools) {
    if (-not (Get-Command $tool -ErrorAction SilentlyContinue)) {
        $missing += $tool
    }
}

if ($missing.Count -gt 0) {
    Write-Host "ERROR: Missing tools: $($missing -join ', ')" -ForegroundColor Red
    Write-Host "Install with: pip install black ruff mypy" -ForegroundColor Yellow
    exit 1
}

# Format check
Write-Host "`nChecking code formatting (black)..." -ForegroundColor Yellow
black --check src/ tests/
if ($LASTEXITCODE -ne 0) {
    Write-Host "ERROR: Code formatting issues found. Run: black src/ tests/" -ForegroundColor Red
    exit 1
}

# Lint check
Write-Host "`nChecking code quality (ruff)..." -ForegroundColor Yellow
ruff check src/ tests/
if ($LASTEXITCODE -ne 0) {
    Write-Host "ERROR: Linting issues found. Run: ruff check --fix src/ tests/" -ForegroundColor Red
    exit 1
}

# Type check
Write-Host "`nChecking types (mypy)..." -ForegroundColor Yellow
mypy src/
if ($LASTEXITCODE -ne 0) {
    Write-Host "WARNING: Type checking issues found (non-blocking)" -ForegroundColor Yellow
}

Write-Host "`nAll checks passed!" -ForegroundColor Green

