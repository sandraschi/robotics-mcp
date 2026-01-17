# Install Watchfiles for Robotics MCP Crashproofing
# This script installs the watchfiles library and required dependencies

Write-Host "🔧 Installing Watchfiles for Robotics MCP Crashproofing" -ForegroundColor Green
Write-Host "======================================================" -ForegroundColor Cyan

# Check if Python is available
try {
    $pythonVersion = python --version 2>&1
    Write-Host "✓ Python found: $pythonVersion" -ForegroundColor Green
} catch {
    Write-Host "✗ Python not found. Please install Python 3.8+ first." -ForegroundColor Red
    exit 1
}

# Check if pip is available
try {
    $pipVersion = python -m pip --version 2>&1
    Write-Host "✓ Pip found: $pipVersion" -ForegroundColor Green
} catch {
    Write-Host "✗ Pip not found. Please install pip first." -ForegroundColor Red
    exit 1
}

# Install watchfiles and aiohttp
Write-Host "📦 Installing watchfiles and dependencies..." -ForegroundColor Yellow
python -m pip install watchfiles>=0.24.0 aiohttp>=3.9.0

if ($LASTEXITCODE -eq 0) {
    Write-Host "✓ Watchfiles installation successful!" -ForegroundColor Green

    # Test the installation
    Write-Host "🧪 Testing watchfiles installation..." -ForegroundColor Yellow
    python -c "import watchfiles, aiohttp; print('✓ All dependencies installed successfully')"

    if ($LASTEXITCODE -eq 0) {
        Write-Host "" -ForegroundColor Green
        Write-Host "🎉 Watchfiles Crashproofing Setup Complete!" -ForegroundColor Green
        Write-Host "" -ForegroundColor Green
        Write-Host "Usage:" -ForegroundColor Cyan
        Write-Host "  .\scripts\run-with-watchfiles.ps1              # Run with default settings" -ForegroundColor White
        Write-Host "  .\scripts\run-with-watchfiles.ps1 -Debug       # Run in debug mode" -ForegroundColor White
        Write-Host "  .\scripts\run-with-watchfiles.ps1 -Port 12231  # Run on custom port" -ForegroundColor White
        Write-Host "" -ForegroundColor Green
        Write-Host "The MCP server will automatically restart on crashes with exponential backoff." -ForegroundColor Yellow
    } else {
        Write-Host "✗ Installation test failed. Please check your Python environment." -ForegroundColor Red
        exit 1
    }
} else {
    Write-Host "✗ Failed to install watchfiles. Please check your internet connection and try again." -ForegroundColor Red
    exit 1
}