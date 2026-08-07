# Robotics MCP Server Watchfiles Stopper
# Cleanly stops the watchfiles protection without spam

param(
    [int]$Port = 12230,
    [switch]$Force
)

Write-Host "🛑 Stopping Robotics MCP Server Watchfiles Protection" -ForegroundColor Red
Write-Host "==================================================" -ForegroundColor Cyan

# Create shutdown signal file to prevent restart spam
$shutdownSignal = "$PSScriptRoot\..\.shutdown_signal"
try {
    New-Item -ItemType File -Path $shutdownSignal -Force | Out-Null
    Write-Host "Created shutdown signal file" -ForegroundColor Green
} catch {
    Write-Host "Warning: Could not create shutdown signal file" -ForegroundColor Yellow
}

# Find and stop the watchfiles process
$watchfilesProcess = Get-Process | Where-Object {
    $_.CommandLine -like "*watchfiles_runner.py*" -and
    $_.CommandLine -like "*$Port*"
}

if ($watchfilesProcess) {
    Write-Host "Found watchfiles process (PID: $($watchfilesProcess.Id))" -ForegroundColor Yellow

    # Try graceful shutdown first
    if (-not $Force) {
        Write-Host "Attempting graceful shutdown..." -ForegroundColor Green
        try {
            Stop-Process -Id $watchfilesProcess.Id -ErrorAction Stop
            Start-Sleep -Seconds 2
        } catch {
            Write-Host "Graceful shutdown failed, using force..." -ForegroundColor Yellow
            $Force = $true
        }
    }

    # Force kill if graceful failed or force requested
    if ($Force) {
        Write-Host "Force stopping watchfiles process..." -ForegroundColor Red
        Stop-Process -Id $watchfilesProcess.Id -Force
    }

    Write-Host "OK Watchfiles protection stopped" -ForegroundColor Green
} else {
    Write-Host "No watchfiles process found for port $Port" -ForegroundColor Yellow
}

# Also stop any remaining MCP server processes on this port
$mcpProcesses = Get-Process | Where-Object {
    $_.CommandLine -like "*robotics_mcp*" -and
    $_.CommandLine -like "*$Port*"
}

if ($mcpProcesses) {
    Write-Host "Found $($mcpProcesses.Count) MCP server process(es)" -ForegroundColor Yellow
    foreach ($proc in $mcpProcesses) {
        Write-Host "Stopping MCP server process (PID: $($proc.Id))" -ForegroundColor Yellow
        Stop-Process -Id $proc.Id -Force
    }
    Write-Host "OK MCP server processes stopped" -ForegroundColor Green
} else {
    Write-Host "No MCP server processes found for port $Port" -ForegroundColor Yellow
}

Write-Host "==================================================" -ForegroundColor Cyan
Write-Host "✅ Robotics MCP Server and watchfiles protection stopped cleanly" -ForegroundColor Green