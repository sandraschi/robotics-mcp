# Install Gaussian Splatting Plugin for Unity
# This enables .ply file support (alternative to .spz)

param(
    [string]$ProjectPath = "C:\Users\sandr\My project"
)

Write-Output "Installing Gaussian Splatting plugin for Unity..."
Write-Output "Project: $ProjectPath"
Write-Output ""

# Check if Unity project exists
$manifestPath = Join-Path $ProjectPath "Packages\manifest.json"
if (-not (Test-Path $manifestPath)) {
    Write-Output "ERROR: Not a valid Unity project (manifest.json not found)"
    Write-Output "Path: $manifestPath"
    exit 1
}

# Read manifest
$manifest = Get-Content $manifestPath | ConvertFrom-Json

# Check if already installed
$dependencies = $manifest.dependencies
$gsPackages = @(
    "com.aras-p.gaussian-splatting",
    "gaussian-splatting",
    "com.unity.gaussian-splatting"
)

$installed = $false
foreach ($pkg in $gsPackages) {
    if ($dependencies.PSObject.Properties.Name -contains $pkg) {
        Write-Output "✅ Gaussian Splatting already installed: $pkg"
        $installed = $true
        break
    }
}

if (-not $installed) {
    # Add package
    if (-not $dependencies) {
        $dependencies = @{}
    }
    
    # Note: package.json is in /package subdirectory, not at repo root
    $dependencies["com.aras-p.gaussian-splatting"] = "https://github.com/aras-p/UnityGaussianSplatting.git?path=/package"
    
    $manifest.dependencies = $dependencies
    
    # Write back
    $manifest | ConvertTo-Json -Depth 10 | Set-Content $manifestPath
    
    Write-Output "✅ Gaussian Splatting package added to manifest.json"
    Write-Output ""
    Write-Output "Next steps:"
    Write-Output "1. Open Unity Editor"
    Write-Output "2. Unity will automatically download and import the package"
    Write-Output "3. Wait for import to complete"
    Write-Output ""
    Write-Output "NOTE: This plugin supports .ply files, NOT .spz files."
    Write-Output "To use your Marble apartment:"
    Write-Output "  - Re-export from Marble as .ply (for splats)"
    Write-Output "  - Or export as .fbx/.glb (for meshes, better for navigation)"
} else {
    Write-Output ""
    Write-Output "Plugin already installed. Open Unity to use it."
}

