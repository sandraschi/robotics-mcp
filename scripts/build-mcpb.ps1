# PowerShell script to build MCPB package

$ErrorActionPreference = "Stop"

Write-Host "Building MCPB package..." -ForegroundColor Cyan

# Check if mcpb is installed
if (-not (Get-Command mcpb -ErrorAction SilentlyContinue)) {
    Write-Host "Installing @anthropic-ai/mcpb..." -ForegroundColor Yellow
    npm install -g @anthropic-ai/mcpb
    if ($LASTEXITCODE -ne 0) {
        Write-Host "ERROR: Failed to install mcpb" -ForegroundColor Red
        exit 1
    }
}

# Create dist directory if it doesn't exist
if (-not (Test-Path "dist")) {
    New-Item -ItemType Directory -Path "dist" | Out-Null
}

# Read version from manifest
$manifest = Get-Content "mcpb/manifest.json" | ConvertFrom-Json
$version = $manifest.version
$name = $manifest.name

$packageName = "$name-v$version.mcpb"
$packagePath = "dist/$packageName"

Write-Host "Building package: $packageName" -ForegroundColor Yellow

# Build MCPB package
mcpb pack mcpb $packagePath

if ($LASTEXITCODE -eq 0) {
    Write-Host "Package built successfully: $packagePath" -ForegroundColor Green
    Write-Host "Package size: $((Get-Item $packagePath).Length / 1KB) KB" -ForegroundColor Cyan
} else {
    Write-Host "ERROR: Package build failed" -ForegroundColor Red
    exit 1
}

