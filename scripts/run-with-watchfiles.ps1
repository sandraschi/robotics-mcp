# Robotics MCP WebApp Watchfiles Runner
# Run the MCP server with crashproofing protection

param(
    [string]$ServerHost = "0.0.0.0",
    [int]$Port = 12230,
    [switch]$Debug,
    [int]$MaxRestarts = 10,
    [float]$RestartDelay = 1.0,
    [float]$BackoffMultiplier = 1.5,
    [int]$HealthCheckInterval = 60,  # Increase to reduce spam
    [switch]$NoNotifications = $true  # Disable notifications by default to reduce spam
)

# Set environment variables
$env:ROBOTICS_MCP_HOST = $ServerHost
$env:ROBOTICS_MCP_PORT = $Port
$env:ROBOTICS_MCP_DEBUG = if ($Debug) { "true" } else { "false" }
$env:WATCHFILES_MAX_RESTARTS = $MaxRestarts
$env:WATCHFILES_RESTART_DELAY = $RestartDelay
$env:WATCHFILES_BACKOFF_MULTIPLIER = $BackoffMultiplier
$env:WATCHFILES_HEALTH_CHECK_INTERVAL = $HealthCheckInterval
$env:WATCHFILES_NOTIFY_ON_CRASH = if ($NoNotifications) { "false" } else { "true" }

Write-Host "🚀 Starting Robotics MCP Server with Watchfiles Protection" -ForegroundColor Green
Write-Host "==================================================" -ForegroundColor Cyan
Write-Host "Host: $ServerHost" -ForegroundColor Yellow
Write-Host "Port: $Port" -ForegroundColor Yellow
Write-Host "Debug: $($Debug.ToString())" -ForegroundColor Yellow
Write-Host "Max Restarts: $MaxRestarts" -ForegroundColor Yellow
Write-Host "Health Check Interval: ${HealthCheckInterval}s" -ForegroundColor Yellow
Write-Host "Notifications: $(if ($NoNotifications) { 'Disabled' } else { 'Enabled' })" -ForegroundColor Yellow
Write-Host "==================================================" -ForegroundColor Cyan

# Clean up zombie processes before starting
Write-Host "🧹 Cleaning up zombie processes..." -ForegroundColor Yellow

$existingProcesses = Get-Process | Where-Object {
    ($_.CommandLine -like "*robotics_mcp*" -and $_.CommandLine -like "*$Port*") -or
    ($_.CommandLine -like "*python*" -and $_.CommandLine -like "*robotics_mcp*" -and $_.CommandLine -like "*$Port*") -or
    ($_.CommandLine -like "*watchfiles_runner.py*" -and $_.CommandLine -like "*$Port*")
}

if ($existingProcesses) {
    Write-Host "Found $($existingProcesses.Count) existing process(es)" -ForegroundColor Yellow
    foreach ($proc in $existingProcesses) {
        Write-Host "Stopping process $($proc.Id): $($proc.ProcessName)" -ForegroundColor Yellow
        Stop-Process -Id $proc.Id -Force -ErrorAction SilentlyContinue
    }
    Start-Sleep -Seconds 2
} else {
    Write-Host "No existing processes found" -ForegroundColor Green
}

# Clean up shutdown signal files
Remove-Item -Path "$PSScriptRoot\..\.shutdown_signal" -Force -ErrorAction SilentlyContinue

Write-Host "✅ Cleanup complete" -ForegroundColor Green
Write-Host ""

# Run the watchfiles runner
python watchfiles_runner.py