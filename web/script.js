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
        // For now, assume Yahboom robots have arm capability when equipped
        armSection.style.display = robot.model.includes('Yahboom') ? 'block' : 'none';

        if (robot.model.includes('Yahboom')) {
            this.setupArmControls();
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