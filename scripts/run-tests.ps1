# PowerShell script to run tests for robotics-mcp

param(
    [string]$Pattern = "*",
    [switch]$Unit,
    [switch]$Integration,
    [switch]$Coverage,
    [switch]$Verbose
)

$ErrorActionPreference = "Stop"

Write-Host "Running robotics-mcp tests..." -ForegroundColor Cyan

# Activate virtual environment if it exists
if (Test-Path "venv\Scripts\Activate.ps1") {
    Write-Host "Activating virtual environment..." -ForegroundColor Yellow
    & "venv\Scripts\Activate.ps1"
}

# Build pytest command
$pytestArgs = @()

if ($Unit) {
    $pytestArgs += "tests/unit"
} elseif ($Integration) {
    $pytestArgs += "tests/integration"
} else {
    $pytestArgs += "tests"
}

if ($Pattern -ne "*") {
    $pytestArgs += "-k", $Pattern
}

if ($Coverage) {
    $pytestArgs += "--cov=robotics_mcp", "--cov-report=html", "--cov-report=term-missing"
}

if ($Verbose) {
    $pytestArgs += "-v"
}

# Run pytest
Write-Host "Running: pytest $($pytestArgs -join ' ')" -ForegroundColor Green
pytest $pytestArgs

if ($LASTEXITCODE -eq 0) {
    Write-Host "Tests passed!" -ForegroundColor Green
    if ($Coverage) {
        Write-Host "Coverage report: htmlcov/index.html" -ForegroundColor Cyan
    }
} else {
    Write-Host "Tests failed!" -ForegroundColor Red
    exit $LASTEXITCODE
}

