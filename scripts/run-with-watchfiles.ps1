# Robotics MCP WebApp Watchfiles Runner
# Run the MCP server with crashproofing protection

param(
    [string]$Host = "0.0.0.0",
    [int]$Port = 12230,
    [switch]$Debug,
    [int]$MaxRestarts = 10,
    [float]$RestartDelay = 1.0,
    [float]$BackoffMultiplier = 1.5,
    [int]$HealthCheckInterval = 30,
    [switch]$NoNotifications
)

# Set environment variables
$env:ROBOTICS_MCP_HOST = $Host
$env:ROBOTICS_MCP_PORT = $Port
$env:ROBOTICS_MCP_DEBUG = if ($Debug) { "true" } else { "false" }
$env:WATCHFILES_MAX_RESTARTS = $MaxRestarts
$env:WATCHFILES_RESTART_DELAY = $RestartDelay
$env:WATCHFILES_BACKOFF_MULTIPLIER = $BackoffMultiplier
$env:WATCHFILES_HEALTH_CHECK_INTERVAL = $HealthCheckInterval
$env:WATCHFILES_NOTIFY_ON_CRASH = if ($NoNotifications) { "false" } else { "true" }

Write-Host "🚀 Starting Robotics MCP WebApp with Watchfiles Protection" -ForegroundColor Green
Write-Host "==================================================" -ForegroundColor Cyan
Write-Host "Host: $Host" -ForegroundColor Yellow
Write-Host "Port: $Port" -ForegroundColor Yellow
Write-Host "Debug: $($Debug.ToString())" -ForegroundColor Yellow
Write-Host "Max Restarts: $MaxRestarts" -ForegroundColor Yellow
Write-Host "Health Check Interval: ${HealthCheckInterval}s" -ForegroundColor Yellow
Write-Host "Notifications: $(if ($NoNotifications) { 'Disabled' } else { 'Enabled' })" -ForegroundColor Yellow
Write-Host "==================================================" -ForegroundColor Cyan

# Run the watchfiles runner
python scripts\watchfiles_runner.py