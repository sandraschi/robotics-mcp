// Robotics MCP Control Panel JavaScript

class RoboticsControlPanel {
    constructor() {
        this.apiBaseUrl = window.location.origin; // Use same host/port as the web server
        this.selectedRobot = null;
        this.commandLog = [];

        this.init();
    }

    init() {
        this.bindEvents();
        this.loadRobots();
        this.updateConnectionStatus();
    }

    bindEvents() {
        // Robot selection
        document.getElementById('refresh-btn').addEventListener('click', () => this.loadRobots());

        // Movement controls
        document.getElementById('move-forward').addEventListener('click', () => this.sendMovementCommand(0.2, 0.0));
        document.getElementById('move-backward').addEventListener('click', () => this.sendMovementCommand(-0.2, 0.0));
        document.getElementById('turn-left').addEventListener('click', () => this.sendMovementCommand(0.0, 0.5));
        document.getElementById('turn-right').addEventListener('click', () => this.sendMovementCommand(0.0, -0.5));
        document.getElementById('stop').addEventListener('click', () => this.sendStopCommand());

        // Speed control
        document.getElementById('linear-speed').addEventListener('input', (e) => {
            document.getElementById('speed-value').textContent = `${e.target.value} m/s`;
        });

        // Special actions
        document.getElementById('home-patrol').addEventListener('click', () => this.sendPatrolCommand());
        document.getElementById('camera-capture').addEventListener('click', () => this.sendCameraCommand());
        document.getElementById('get-status').addEventListener('click', () => this.refreshRobotStatus());

        // Arm controls
        document.getElementById('move-arm').addEventListener('click', () => this.sendArmCommand());
        document.getElementById('gripper-open').addEventListener('click', () => this.sendGripperCommand('open'));
        document.getElementById('gripper-close').addEventListener('click', () => this.sendGripperCommand('close'));

        // Vacuum controls
        document.getElementById('start-cleaning').addEventListener('click', () => this.sendVacuumCommand('start_cleaning'));
        document.getElementById('stop-cleaning').addEventListener('click', () => this.sendVacuumCommand('stop_cleaning'));
        document.getElementById('return-to-dock').addEventListener('click', () => this.sendVacuumCommand('return_to_dock'));
        document.getElementById('apply-settings').addEventListener('click', () => this.sendVacuumSettings());
        document.getElementById('clean-room').addEventListener('click', () => this.sendCleanRoomCommand());
        document.getElementById('clean-zone').addEventListener('click', () => this.sendCleanZoneCommand());
        document.getElementById('clean-spot').addEventListener('click', () => this.sendCleanSpotCommand());
        document.getElementById('start-mapping').addEventListener('click', () => this.sendVacuumCommand('start_mapping'));
        document.getElementById('get-map').addEventListener('click', () => this.sendGetMapCommand());
        document.getElementById('clear-map').addEventListener('click', () => this.clearMap());

        // Mode switching
        document.getElementById('clean-room').addEventListener('click', () => this.showCleaningMode('room'));
        document.getElementById('clean-zone').addEventListener('click', () => this.showCleaningMode('zone'));
        document.getElementById('clean-spot').addEventListener('click', () => this.showCleaningMode('spot'));
    }

    async loadRobots() {
        try {
            const response = await fetch(`${this.apiBaseUrl}/api/v1/robots`);
            if (!response.ok) throw new Error(`HTTP ${response.status}`);

            const data = await response.json();
            this.displayRobots(data.robots || []);
        } catch (error) {
            console.error('Failed to load robots:', error);
            this.displayRobots([]);
            this.addLogEntry('Failed to load robots', 'error');
        }
    }

    displayRobots(robots) {
        const robotList = document.getElementById('robot-list');

        if (robots.length === 0) {
            robotList.innerHTML = '<p>No robots found. Make sure the robotics MCP server is running.</p>';
            return;
        }

        robotList.innerHTML = robots.map(robot => `
            <div class="robot-card ${this.selectedRobot?.robot_id === robot.robot_id ? 'selected' : ''}"
                 data-robot-id="${robot.robot_id}"
                 onclick="controlPanel.selectRobot('${robot.robot_id}')">
                <h3>${robot.model || 'Unknown'} (${robot.robot_id})</h3>
                <div class="status">
                    <span>${robot.connected ? '🟢 Connected' : '🔴 Disconnected'}</span>
                    <span>${robot.mock ? 'Mock' : 'Real'}</span>
                </div>
            </div>
        `).join('');
    }

    selectRobot(robotId) {
        // Find robot data
        const robotCards = document.querySelectorAll('.robot-card');
        robotCards.forEach(card => {
            if (card.dataset.robotId === robotId) {
                card.classList.add('selected');
                const robotData = this.parseRobotFromCard(card);
                this.selectedRobot = robotData;
                this.showControlPanels(robotData);
            } else {
                card.classList.remove('selected');
            }
        });
    }

    parseRobotFromCard(card) {
        const robotId = card.dataset.robotId;
        const title = card.querySelector('h3').textContent;
        const connected = card.textContent.includes('🟢');

        // Parse model from title (format: "Model (robot_id)")
        const modelMatch = title.match(/^(.+)\s+\(.+\)$/);
        const model = modelMatch ? modelMatch[1] : 'Unknown';

        return { robot_id: robotId, model, connected };
    }

    showControlPanels(robot) {
        document.getElementById('control-panels').style.display = 'block';
        this.updateRobotStatus(robot);
        this.enableControls(robot.connected);

        // Show arm controls if robot has arm capability
        const armSection = document.getElementById('arm-section');
        armSection.style.display = robot.model.includes('Yahboom') ? 'block' : 'none';

        // Show vacuum controls if robot is Dreame vacuum
        const vacuumSection = document.getElementById('vacuum-section');
        const isDreame = robot.model.includes('Dreame') || robot.robot_type === 'dreame';
        vacuumSection.style.display = isDreame ? 'block' : 'none';

        // Show map display if robot supports mapping
        const mapSection = document.getElementById('map-section');
        mapSection.style.display = isDreame ? 'block' : 'none';

        if (robot.model.includes('Yahboom')) {
            this.setupArmControls();
        }

        if (isDreame) {
            this.setupVacuumControls();
        }
    }

    updateRobotStatus(robot) {
        document.getElementById('robot-model').textContent = robot.model;
        document.getElementById('robot-connected').textContent = robot.connected ? '🟢 Yes' : '🔴 No';
        document.getElementById('robot-battery').textContent = robot.battery || '-';
        document.getElementById('robot-position').textContent = robot.position || '-';
    }

    enableControls(enabled) {
        const buttons = document.querySelectorAll('.move-btn, .action-btn');
        buttons.forEach(btn => {
            btn.disabled = !enabled;
        });
    }

    setupArmControls() {
        const jointInputs = document.getElementById('joint-inputs');
        // Create joint angle inputs for typical 4-DOF arm
        const joints = ['Base', 'Shoulder', 'Elbow', 'Wrist'];

        jointInputs.innerHTML = joints.map((joint, index) => `
            <div class="joint-input">
                <label>${joint}:</label>
                <input type="range" min="-180" max="180" value="0" step="5"
                       data-joint="${index + 1}">
                <span>0°</span>
            </div>
        `).join('');

        // Add event listeners to update displayed values
        jointInputs.querySelectorAll('input[type="range"]').forEach(input => {
            input.addEventListener('input', (e) => {
                e.target.nextElementSibling.textContent = `${e.target.value}°`;
            });
        });
    }

    async sendMovementCommand(linear, angular) {
        if (!this.selectedRobot) return;

        const speed = parseFloat(document.getElementById('linear-speed').value);

        await this.sendCommand({
            action: 'move',
            robot_id: this.selectedRobot.robot_id,
            linear: linear * speed,
            angular: angular
        });
    }

    async sendStopCommand() {
        if (!this.selectedRobot) return;

        await this.sendCommand({
            action: 'stop',
            robot_id: this.selectedRobot.robot_id
        });
    }

    async sendPatrolCommand() {
        if (!this.selectedRobot) return;

        await this.sendCommand({
            action: 'home_patrol',
            robot_id: this.selectedRobot.robot_id
        });
    }

    async sendCameraCommand() {
        if (!this.selectedRobot) return;

        await this.sendCommand({
            action: 'camera_capture',
            robot_id: this.selectedRobot.robot_id
        });
    }

    async sendArmCommand() {
        if (!this.selectedRobot) return;

        const jointAngles = {};
        document.querySelectorAll('#joint-inputs input[type="range"]').forEach(input => {
            const jointIndex = input.dataset.joint;
            jointAngles[`joint${jointIndex}`] = parseFloat(input.value) * Math.PI / 180; // Convert to radians
        });

        await this.sendCommand({
            action: 'arm_move',
            robot_id: this.selectedRobot.robot_id,
            joint_angles: jointAngles
        });
    }

    async sendGripperCommand(action) {
        if (!this.selectedRobot) return;

        await this.sendCommand({
            action: 'gripper_control',
            robot_id: this.selectedRobot.robot_id,
            gripper_action: action
        });
    }

    async sendCommand(command) {
        try {
            const response = await fetch(`${this.apiBaseUrl}/api/v1/robots/${command.robot_id}/control`, {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify(command)
            });

            const result = await response.json();

            if (response.ok) {
                this.addLogEntry(`✅ ${command.action} command sent`, 'success');
                // Refresh status after successful command
                setTimeout(() => this.refreshRobotStatus(), 1000);
            } else {
                this.addLogEntry(`❌ ${command.action} failed: ${result.detail || 'Unknown error'}`, 'error');
            }

        } catch (error) {
            console.error('Command failed:', error);
            this.addLogEntry(`❌ ${command.action} failed: ${error.message}`, 'error');
        }
    }

    async refreshRobotStatus() {
        if (!this.selectedRobot) return;

        try {
            const response = await fetch(`${this.apiBaseUrl}/api/v1/robots/${this.selectedRobot.robot_id}/status`);
            if (response.ok) {
                const status = await response.json();
                this.updateRobotStatus({ ...this.selectedRobot, ...status });
                this.addLogEntry('📊 Status refreshed', 'success');
            }
        } catch (error) {
            console.error('Status refresh failed:', error);
            this.addLogEntry('❌ Status refresh failed', 'error');
        }
    }

    addLogEntry(message, type = 'info') {
        const logContainer = document.getElementById('command-log');
        const timestamp = new Date().toLocaleTimeString();
        const logEntry = document.createElement('div');
        logEntry.className = `log-entry ${type}`;
        logEntry.textContent = `[${timestamp}] ${message}`;

        logContainer.appendChild(logEntry);
        logContainer.scrollTop = logContainer.scrollHeight;

        // Keep only last 50 entries
        while (logContainer.children.length > 50) {
            logContainer.removeChild(logContainer.firstChild);
        }
    }

    updateConnectionStatus() {
        // Check server connectivity
        fetch(`${this.apiBaseUrl}/api/v1/health`)
            .then(response => {
                const status = document.getElementById('connection-status');
                if (response.ok) {
                    status.textContent = '🟢 Connected to Robotics MCP';
                    status.className = 'status-connected';
                } else {
                    status.textContent = '🟡 Server responding (robots unavailable)';
                    status.className = 'status-disconnected';
                }
            })
            .catch(() => {
                const status = document.getElementById('connection-status');
                status.textContent = '🔴 Cannot connect to Robotics MCP';
                status.className = 'status-disconnected';
            });
    }

    setupVacuumControls() {
        // Setup any dynamic vacuum controls if needed
        this.showCleaningMode('none');
    }

    showCleaningMode(mode) {
        // Hide all mode inputs
        document.getElementById('room-selection').style.display = 'none';
        document.getElementById('zone-input').style.display = 'none';
        document.getElementById('spot-input').style.display = 'none';

        // Show selected mode input
        if (mode === 'room') {
            document.getElementById('room-selection').style.display = 'block';
        } else if (mode === 'zone') {
            document.getElementById('zone-input').style.display = 'block';
        } else if (mode === 'spot') {
            document.getElementById('spot-input').style.display = 'block';
        }
    }

    async sendVacuumCommand(action, extraParams = {}) {
        if (!this.selectedRobot) return;

        await this.sendCommand({
            action: action,
            robot_id: this.selectedRobot.robot_id,
            ...extraParams
        });
    }

    async sendVacuumSettings() {
        if (!this.selectedRobot) return;

        const suctionLevel = parseInt(document.getElementById('suction-level').value);
        const waterVolume = parseInt(document.getElementById('water-volume').value);
        const mopHumidity = parseInt(document.getElementById('mop-humidity').value);

        // Send settings in sequence
        await this.sendVacuumCommand('set_suction_level', { suction_level: suctionLevel });
        await this.sendVacuumCommand('set_water_volume', { water_volume: waterVolume });
        await this.sendVacuumCommand('set_mop_humidity', { mop_humidity: mopHumidity });

        this.addLogEntry('⚙️ Vacuum settings applied', 'success');
    }

    async sendCleanRoomCommand() {
        if (!this.selectedRobot) return;

        const roomId = parseInt(document.getElementById('room-id').value) || 1;
        await this.sendVacuumCommand('clean_room', { room_id: roomId });
    }

    async sendCleanZoneCommand() {
        if (!this.selectedRobot) return;

        const zoneCoords = document.getElementById('zone-coords').value;
        const coords = zoneCoords.split(',').map(c => parseInt(c.trim()));

        if (coords.length !== 4) {
            this.addLogEntry('❌ Invalid zone coordinates format (need x1,y1,x2,y2)', 'error');
            return;
        }

        await this.sendVacuumCommand('clean_zone', { zones: [coords] });
    }

    async sendCleanSpotCommand() {
        if (!this.selectedRobot) return;

        const spotCoords = document.getElementById('spot-coords').value;
        const coords = spotCoords.split(',').map(c => parseInt(c.trim()));

        if (coords.length !== 2) {
            this.addLogEntry('❌ Invalid spot coordinates format (need x,y)', 'error');
            return;
        }

        await this.sendVacuumCommand('clean_spot', { spot_x: coords[0], spot_y: coords[1] });
    }

    async sendGetMapCommand() {
        if (!this.selectedRobot) return;

        try {
            const response = await fetch(`${this.apiBaseUrl}/api/v1/robots/${this.selectedRobot.robot_id}/control`, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({
                    action: 'get_map',
                    robot_id: this.selectedRobot.robot_id
                })
            });

            if (response.ok) {
                const result = await response.json();
                if (result.success && result.data) {
                    // Handle different map data formats (Dreame vs others)
                    const mapData = result.data.map_data || result.data.map;
                    if (mapData) {
                        this.renderMap(mapData);
                        this.addLogEntry('🗺️ Map data retrieved', 'success');
                    } else {
                        this.addLogEntry('⚠️ No map data available', 'warning');
                    }
                } else {
                    this.addLogEntry('⚠️ No map data available', 'warning');
                }
            } else {
                this.addLogEntry('❌ Failed to get map data', 'error');
            }
        } catch (error) {
            console.error('Map retrieval failed:', error);
            this.addLogEntry('❌ Map retrieval failed', 'error');
        }
    }

    renderMap(mapData) {
        const canvas = document.getElementById('map-canvas');
        const ctx = canvas.getContext('2d');

        // Clear canvas
        ctx.clearRect(0, 0, canvas.width, canvas.height);

        // Set background (floor)
        ctx.fillStyle = '#e8f5e8'; // Light green for floor
        ctx.fillRect(0, 0, canvas.width, canvas.height);

        // Scale factor for map coordinates (Dreame maps are usually in mm)
        const scale = canvas.width / 8000; // Assume 8m x 8m room, adjust as needed

        // Draw rooms (if available)
        if (mapData.rooms && Array.isArray(mapData.rooms)) {
            mapData.rooms.forEach((room, index) => {
                ctx.strokeStyle = '#2563eb'; // Blue for room boundaries
                ctx.lineWidth = 2;
                ctx.fillStyle = `hsl(${index * 60}, 70%, 90%)`; // Different colors for rooms

                if (room.coordinates && Array.isArray(room.coordinates)) {
                    // Draw room as polygon or rectangle
                    if (room.coordinates.length >= 4) {
                        ctx.beginPath();
                        ctx.moveTo(room.coordinates[0] * scale, room.coordinates[1] * scale);

                        for (let i = 2; i < room.coordinates.length; i += 2) {
                            ctx.lineTo(room.coordinates[i] * scale, room.coordinates[i + 1] * scale);
                        }

                        ctx.closePath();
                        ctx.fill();
                        ctx.stroke();

                        // Room label
                        if (room.name) {
                            ctx.fillStyle = '#000';
                            ctx.font = '12px Arial';
                            ctx.fillText(room.name, room.coordinates[0] * scale + 5, room.coordinates[1] * scale + 15);
                        }
                    }
                }
            });
        }

        // Draw obstacles (if available)
        if (mapData.obstacles && Array.isArray(mapData.obstacles)) {
            ctx.fillStyle = '#dc2626'; // Red for obstacles

            mapData.obstacles.forEach(obstacle => {
                if (obstacle.x !== undefined && obstacle.y !== undefined) {
                    const size = obstacle.size || 50; // Default 5cm obstacle
                    ctx.beginPath();
                    ctx.arc(obstacle.x * scale, obstacle.y * scale, size * scale, 0, 2 * Math.PI);
                    ctx.fill();
                }
            });
        }

        // Draw charging station (if available)
        if (mapData.charging_station) {
            const station = mapData.charging_station;
            if (station.x !== undefined && station.y !== undefined) {
                ctx.fillStyle = '#16a34a'; // Green for charging station
                ctx.strokeStyle = '#166534';
                ctx.lineWidth = 2;

                // Draw charging station icon
                const x = station.x * scale;
                const y = station.y * scale;
                const size = 30;

                // Draw a simple charging station symbol
                ctx.beginPath();
                ctx.rect(x - size/2, y - size/2, size, size);
                ctx.fill();
                ctx.stroke();

                // Charging symbol
                ctx.strokeStyle = '#fff';
                ctx.lineWidth = 2;
                ctx.beginPath();
                ctx.moveTo(x, y - 8);
                ctx.lineTo(x, y + 8);
                ctx.moveTo(x - 4, y - 4);
                ctx.lineTo(x, y);
                ctx.lineTo(x + 4, y - 4);
                ctx.stroke();
            }
        }

        // Draw robot position if available
        if (this.selectedRobot && this.selectedRobot.position) {
            ctx.fillStyle = '#ff0000'; // Red for robot
            ctx.strokeStyle = '#cc0000';
            ctx.lineWidth = 2;
            ctx.beginPath();
            ctx.arc(this.selectedRobot.position.x * scale, this.selectedRobot.position.y * scale, 8, 0, 2 * Math.PI);
            ctx.fill();
            ctx.stroke();
        }

        // Draw map info
        ctx.fillStyle = '#000';
        ctx.font = '10px Arial';
        let yPos = 15;
        ctx.fillText(`Map ID: ${mapData.map_id || 'Unknown'}`, 5, yPos);
        yPos += 12;
        ctx.fillText(`Rooms: ${mapData.rooms ? mapData.rooms.length : 0}`, 5, yPos);
        yPos += 12;
        ctx.fillText(`Obstacles: ${mapData.obstacles ? mapData.obstacles.length : 0}`, 5, yPos);

        // Legend
        const legendX = canvas.width - 80;
        let legendY = 15;

        ctx.fillStyle = '#2563eb';
        ctx.fillRect(legendX, legendY, 10, 10);
        ctx.fillStyle = '#000';
        ctx.fillText('Rooms', legendX + 15, legendY + 8);
        legendY += 15;

        ctx.fillStyle = '#dc2626';
        ctx.fillRect(legendX, legendY, 10, 10);
        ctx.fillStyle = '#000';
        ctx.fillText('Obstacles', legendX + 15, legendY + 8);
        legendY += 15;

        ctx.fillStyle = '#16a34a';
        ctx.fillRect(legendX, legendY, 10, 10);
        ctx.fillStyle = '#000';
        ctx.fillText('Charger', legendX + 15, legendY + 8);
        legendY += 15;

        ctx.fillStyle = '#ff0000';
        ctx.fillRect(legendX, legendY, 10, 10);
        ctx.fillStyle = '#000';
        ctx.fillText('Robot', legendX + 15, legendY + 8);
    }

    clearMap() {
        const canvas = document.getElementById('map-canvas');
        const ctx = canvas.getContext('2d');
        ctx.clearRect(0, 0, canvas.width, canvas.height);

        // Reset to default background
        ctx.fillStyle = '#f0f0f0';
        ctx.fillRect(0, 0, canvas.width, canvas.height);

        this.addLogEntry('🗑️ Map cleared', 'info');
    }
}

// Initialize the control panel when the page loads
const controlPanel = new RoboticsControlPanel();

// Auto-refresh robot list every 30 seconds
setInterval(() => {
    if (controlPanel.selectedRobot) {
        controlPanel.refreshRobotStatus();
    }
    controlPanel.updateConnectionStatus();
}, 30000);