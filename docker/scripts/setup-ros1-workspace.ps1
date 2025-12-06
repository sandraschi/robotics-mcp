# PowerShell script to setup ROS 1.4 workspace in Docker container

param(
    [switch]$Build,
    [switch]$Clean
)

$ErrorActionPreference = "Stop"

Write-Host "🔧 Setting up ROS 1.4 (Melodic) workspace for Scout..." -ForegroundColor Cyan

# Check if container is running
$containerName = "robotics-ros1-dev"
$containerRunning = docker ps --filter "name=$containerName" --format "{{.Names}}"

if (-not $containerRunning) {
    Write-Host "❌ Container $containerName is not running!" -ForegroundColor Red
    Write-Host "Start it with: docker-compose -f docker/docker-compose.ros1.yml up -d" -ForegroundColor Yellow
    exit 1
}

if ($Clean) {
    Write-Host "🧹 Cleaning workspace..." -ForegroundColor Yellow
    docker exec $containerName bash -c "cd /ros_ws && rm -rf build devel"
}

Write-Host "📦 Setting up workspace..." -ForegroundColor Yellow
docker exec -it $containerName bash /ros_ws/setup-ros1-workspace.sh

if ($Build) {
    Write-Host "🔨 Building workspace..." -ForegroundColor Yellow
    docker exec $containerName bash -c "cd /ros_ws && source /opt/ros/melodic/setup.bash && catkin_make"
}

Write-Host "✅ Setup complete!" -ForegroundColor Green
Write-Host ""
Write-Host "To enter container:" -ForegroundColor Cyan
Write-Host "  docker exec -it $containerName bash" -ForegroundColor White
Write-Host ""
Write-Host "To start rosbridge:" -ForegroundColor Cyan
Write-Host "  docker exec -it $containerName bash -c 'source /ros_ws/devel/setup.bash && roslaunch rosbridge_server rosbridge_websocket.launch port:=9090'" -ForegroundColor White

