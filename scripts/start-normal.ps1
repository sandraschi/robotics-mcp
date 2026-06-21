# Robotics MCP Server Normal Start Script (without watchfiles)
# Cleans up zombie processes before starting fresh

param(
    [string]$Host = "0.0.0.0",
    [int]$Port = 12230
)

Write-Host "🤖 Starting Robotics MCP Server (Normal Mode)" -ForegroundColor Green
Write-Host "===============================================" -ForegroundColor Cyan
Write-Host "Host: $Host" -ForegroundColor Yellow
Write-Host "Port: $Port" -ForegroundColor Yellow
Write-Host "Watchfiles: Disabled" -ForegroundColor Yellow
Write-Host "===============================================" -ForegroundColor Cyan

# Clean up zombie processes
Write-Host "🧹 Cleaning up zombie processes..." -ForegroundColor Yellow

# Kill any existing MCP server processes on this port
$existingProcesses = Get-Process | Where-Object {
    ($_.CommandLine -like "*robotics_mcp*" -and $_.CommandLine -like "*$Port*") -or
    ($_.CommandLine -like "*python*" -and $_.CommandLine -like "*robotics_mcp*" -and $_.CommandLine -like "*$Port*")
}

if ($existingProcesses) {
    Write-Host "Found $($existingProcesses.Count) existing process(es)" -ForegroundColor Yellow
    foreach ($proc in $existingProcesses) {
        Write-Host "Stopping process $($proc.Id): $($proc.ProcessName)" -ForegroundColor Yellow
        Stop-Process -Id $proc.Id -Force -ErrorAction SilentlyContinue
    }
    Start-Sleep -Seconds 2  # Wait for processes to fully terminate
} else {
    Write-Host "No existing processes found" -ForegroundColor Green
}

# Clean up shutdown signal files
Remove-Item -Path "$PSScriptRoot\..\.shutdown_signal" -Force -ErrorAction SilentlyContinue

Write-Host "✅ Cleanup complete" -ForegroundColor Green
Write-Host ""

# Set environment variables
$env:ROBOTICS_MCP_HOST = $Host
$env:ROBOTICS_MCP_PORT = $Port

# Start the MCP server directly
Write-Host "🚀 Starting MCP server..." -ForegroundColor Green
try {
    Push-Location "$PSScriptRoot\.."
    & python -m robotics_mcp.server --mode http --host $Host --port $Port
} catch {
    Write-Host "❌ Failed to start MCP server: $_" -ForegroundColor Red
    exit 1
} finally {
    Pop-Location
}