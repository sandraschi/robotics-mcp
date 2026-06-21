param(
    [switch]$Headless,
    [switch]$BackendOnly,
    [switch]$NoBrowser,
    [switch]$ReuseIfRunning)

$WebPort = 10706
$BackendPort = 10707
$ProjectRoot = Split-Path -Parent $PSScriptRoot

$FleetStartPath = Join-Path $ProjectRoot "scripts\FleetStartMode.ps1"
if (-not (Test-Path -LiteralPath $FleetStartPath)) {
    Write-Host "ERROR: Missing vendored launcher helper: $FleetStartPath" -ForegroundColor Red
    exit 1
}
. $FleetStartPath
$FleetStart = Initialize-FleetStartMode @PSBoundParameters
Enter-FleetHeadlessConsole -Headless:$Headless -BackendOnly:$BackendOnly

$portResolve = @{
    Ports      = @($WebPort, $BackendPort)
    Label      = "robotics-mcp"
    AllowReuse = $ReuseIfRunning
}
if ($ReuseIfRunning) {
    $portResolve.HealthChecks = @{
        $WebPort = "http://127.0.0.1:$WebPort/"
        $BackendPort = "http://127.0.0.1:$BackendPort/api/v1/health"
    }
}
$portState = Resolve-FleetPortConflict @portResolve
if ($portState.Action -eq 'Blocked') { exit 1 }
if ($portState.Reuse) { return }

Set-Location $PSScriptRoot
if (-not (Test-Path "node_modules")) { npm install }

Write-Host "Starting Python backend on port $BackendPort ..." -ForegroundColor Cyan
$backendCmd = "Set-Location '$ProjectRoot'; uv run --project '$ProjectRoot' python -m robotics_mcp.server --mode http --host 127.0.0.1 --port $BackendPort"
Start-Process powershell -ArgumentList "-NoProfile", "-WindowStyle", "Normal", "-Command", $backendCmd

$healthUrl = "http://127.0.0.1:$BackendPort/api/v1/health"
$attempt = 0
while ($attempt -lt 45) {
    try {
        $null = Invoke-WebRequest -Uri $healthUrl -UseBasicParsing -TimeoutSec 3 -ErrorAction Stop
        Write-Host "Backend ready at $healthUrl" -ForegroundColor Green
        break
    } catch {
        Start-Sleep -Seconds 2
        $attempt++
    }
}

if (-not $FleetStart.RunFrontend) {
    while ($true) { Start-Sleep -Seconds 60 }
}

if (-not $NoBrowser) {
    $frontendUrl = "http://127.0.0.1:$WebPort/"
    $pollAndOpen = "for (`$i = 0; `$i -lt 60; `$i++) { try { `$null = Invoke-WebRequest -Uri '$frontendUrl' -TimeoutSec 2 -UseBasicParsing -ErrorAction Stop; Start-Process '$frontendUrl'; exit } catch { Start-Sleep -Seconds 1 } }"
    Start-Process powershell -ArgumentList "-NoProfile", "-WindowStyle", "Hidden", "-Command", $pollAndOpen
}

Write-Host "Starting Vite frontend on port $WebPort ..." -ForegroundColor Green
npm run dev -- --port $WebPort --host


