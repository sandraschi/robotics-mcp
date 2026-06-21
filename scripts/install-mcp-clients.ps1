param([switch]$Interactive)
$ErrorActionPreference = "Stop"

$configs = @(
    @{ Name = "Cursor"; Path = "$env:USERPROFILE\.cursor\mcp.json" },
    @{ Name = "Claude Desktop"; Path = "$env:APPDATA\Claude\claude_desktop_config.json" }
)

$updated = 0
foreach ($cfg in $configs) {
    if (-not (Test-Path $cfg.Path)) { continue }
    try {
        $json = Get-Content $cfg.Path -Raw | ConvertFrom-Json
        if ($json.mcpServers -eq $null) { $json = @{ mcpServers = @{ } } }
        if ($json.mcpServers."robotics-mcp" -eq $null) {
            $json.mcpServers | Add-Member -Name "robotics-mcp" -Value @{ url = "http://127.0.0.1:10706/mcp" } -MemberType NoteProperty
            $json | ConvertTo-Json -Depth 10 | Set-Content $cfg.Path
            Write-Host "  Registered robotics-mcp in $($cfg.Name)" -ForegroundColor Green
            $updated++
        } else {
            Write-Host "  Already registered in $($cfg.Name)" -ForegroundColor Yellow
        }
    } catch {
        Write-Host "  Failed to update $($cfg.Name): $_" -ForegroundColor Red
    }
}

if ($updated -gt 0) {
    Write-Host "MCP client registration complete." -ForegroundColor Green
} elseif ($Interactive -and $updated -eq 0) {
    Write-Host "No new registrations needed." -ForegroundColor Yellow
}
