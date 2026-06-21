# Ensure ~/.robotics-mcp/config.yaml has yahboom_raspbot_v2 enabled for Raspbot V2.
# Usage: .\Ensure-RaspbotConfig.ps1 [-RobotIp 192.168.1.11] [-Probe]
# Run from robotics-mcp repo root so uv run can find the project.

param(
    [string]$RobotIp = "192.168.0.250",
    [switch]$Probe
)

$ErrorActionPreference = "Stop"
$configDir = Join-Path $env:USERPROFILE ".robotics-mcp"
if (-not (Test-Path $configDir)) {
    New-Item -ItemType Directory -Force -Path $configDir | Out-Null
}

$scriptArgs = @("--ensure-config", "--ip", $RobotIp)
if ($Probe) { $scriptArgs += "--probe" }

# Prefer uv run from repo root (parent of scripts/); else python -m
$repoRoot = (Split-Path -Parent $PSScriptRoot)
if (-not $repoRoot) { $repoRoot = (Get-Location).Path }
$pyMod = "robotics_mcp.scripts.raspbot_setup"

$uvExe = Get-Command uv -ErrorAction SilentlyContinue
if ($uvExe) {
    Push-Location $repoRoot
    try {
        & uv run python -m $pyMod @scriptArgs
    } finally {
        Pop-Location
    }
} else {
    & python -m $pyMod @scriptArgs
}
