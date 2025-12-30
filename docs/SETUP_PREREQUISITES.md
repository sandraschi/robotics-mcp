# 🎯 Robotics MCP Setup Prerequisites

**Complete installation guide for all required software and MCP servers**

---

## 🔴 **PHYSICAL ROBOTS (RECOMMENDED)**

### Supported Robots
**You SHOULD own one of these supported robots for the complete experience:**

#### **Moorebot Scout** (Primary Focus)
- **Type**: Mecanum wheeled robot
- **Features**: LiDAR, camera, ROS integration
- **Where to Buy**: [Moorebot Store](https://moorebot.com) or authorized dealers
- **Price Range**: $2,500 - $5,000

#### **Unitree Go2**
- **Type**: Quadrupedal robot (dog-like)
- **Features**: Advanced locomotion, ROS 2 support
- **Where to Buy**: [Unitree Robotics](https://www.unitree.com/)
- **Price Range**: $2,700+

#### **Unitree G1/H1**
- **Type**: Humanoid robot
- **Features**: Bipedal walking, arm manipulation
- **Where to Buy**: [Unitree Robotics](https://www.unitree.com/)
- **Price Range**: $16,000+

> **Note**: Physical robots provide the authentic robotics experience. Virtual robots are for testing/simulation when you don't have hardware.

---

## 🟡 **REQUIRED SOFTWARE INSTALLATION**

### Unity 3D (Required for Virtual Robotics)

#### **Download Unity Hub**
```bash
# Official Unity Hub Download:
# https://unity.com/download
```

#### **Install Unity Editor 6000.2.14f1**
1. **Open Unity Hub**
2. **Go to "Installs" tab**
3. **Click "Add" → "Official releases"**
4. **Select version "6000.2.14f1" (LTS)**
5. **Required Components**:
   - ✅ Unity Editor
   - ✅ Android Build Support (recommended)
   - ✅ Windows Build Support (recommended)
   - ✅ Documentation
6. **Installation Size**: ~8GB
7. **Wait for download and installation to complete**

#### **Verify Installation**
```bash
# Check Unity version
& "C:\Program Files\Unity\Hub\Editor\6000.2.14f1\Editor\Unity.exe" --version
# Should output: 6000.2.14f1
```

### VRChat (Required for Social VR Robotics)

#### **Download from Steam**
```bash
# Steam Store Page:
# https://store.steampowered.com/app/438100/VRChat/
```

#### **Alternative Download**
```bash
# VRChat Official Website:
# https://hello.vrchat.com/
```

#### **Setup OSC (Required for Robotics Control)**
1. **Launch VRChat**
2. **Go to Settings → OSC**
3. **Enable OSC** (checkmark)
4. **Set OSC Port**: `9000` (default)
5. **Restart VRChat**

#### **Test OSC Connection**
```bash
# Use any OSC testing tool or our robotics-mcp server
# OSC messages should work between robotics-mcp and VRChat
```

---

## 🟢 **REQUIRED MCP SERVERS**

### 1. Unity3D-MCP (Virtual Robot Control)

#### **GitHub Repository**
```bash
# Repository: https://github.com/sandraschi/unity3d-mcp
```

#### **Installation**
```bash
# Clone repository
git clone https://github.com/sandraschi/unity3d-mcp.git
cd unity3d-mcp

# Install dependencies
pip install -e .

# Verify installation
python -c "import unity3d_mcp; print('Unity3D-MCP installed successfully')"
```

#### **Cursor MCP Configuration**
Add to your Cursor MCP configuration:
```json
{
  "mcpServers": {
    "unity3d-mcp": {
      "command": "python",
      "args": ["-m", "unity3d_mcp"],
      "env": {}
    }
  }
}
```

### 2. OSC-MCP (Real-time Communication)

#### **GitHub Repository**
```bash
# Repository: https://github.com/sandraschi/osc-mcp
```

#### **Installation**
```bash
# Clone repository
git clone https://github.com/sandraschi/osc-mcp.git
cd osc-mcp

# Install dependencies
pip install -e .

# Verify installation
python -c "import oscmcp; print('OSC-MCP installed successfully')"
```

#### **Cursor MCP Configuration**
```json
{
  "mcpServers": {
    "osc-mcp": {
      "command": "python",
      "args": ["-m", "oscmcp"],
      "env": {}
    }
  }
}
```

### 3. VRChat-MCP (Social VR Integration)

#### **GitHub Repository**
```bash
# Repository: https://github.com/sandraschi/vrchat-mcp
```

#### **Installation**
```bash
# Clone repository
git clone https://github.com/sandraschi/vrchat-mcp.git
cd vrchat-mcp

# Install dependencies
pip install -e .

# Verify installation
python -c "import vrchat_mcp; print('VRChat-MCP installed successfully')"
```

#### **Cursor MCP Configuration**
```json
{
  "mcpServers": {
    "vrchat-mcp": {
      "command": "python",
      "args": ["-m", "vrchat_mcp"],
      "env": {}
    }
  }
}
```

### 4. Blender-MCP (3D Model Creation)

#### **Prerequisites**
- **Blender 4.0+** must be installed first
- Download from: https://www.blender.org/download/

#### **GitHub Repository**
```bash
# Repository: https://github.com/sandraschi/blender-mcp
```

#### **Installation**
```bash
# Clone repository
git clone https://github.com/sandraschi/blender-mcp.git
cd blender-mcp

# Install dependencies
pip install -e .

# Verify installation
python -c "import blender_mcp; print('Blender-MCP installed successfully')"
```

#### **Cursor MCP Configuration**
```json
{
  "mcpServers": {
    "blender-mcp": {
      "command": "python",
      "args": ["-m", "blender_mcp"],
      "env": {}
    }
  }
}
```

### 5. Avatar-MCP (Avatar Management)

#### **GitHub Repository**
```bash
# Repository: https://github.com/sandraschi/avatar-mcp
```

#### **Installation**
```bash
# Clone repository
git clone https://github.com/sandraschi/avatar-mcp.git
cd avatar-mcp

# Install dependencies
pip install -e .

# Verify installation
python -c "import avatar_mcp; print('Avatar-MCP installed successfully')"
```

#### **Cursor MCP Configuration**
```json
{
  "mcpServers": {
    "avatar-mcp": {
      "command": "python",
      "args": ["-m", "avatar_mcp"],
      "env": {}
    }
  }
}
```

---

## 🔧 **ROBOTICS-MCP INSTALLATION**

### Main Repository
```bash
# Repository: https://github.com/sandraschi/robotics-mcp
```

### Installation
```bash
# Clone repository
git clone https://github.com/sandraschi/robotics-mcp.git
cd robotics-mcp

# Install with development dependencies
pip install -e ".[dev]"

# Verify installation
python -c "import robotics_mcp; print('Robotics-MCP installed successfully')"
```

### Cursor MCP Configuration
Add to your Cursor MCP configuration:
```json
{
  "mcpServers": {
    "robotics-mcp": {
      "command": "python",
      "args": ["-m", "robotics_mcp.server"],
      "env": {}
    }
  }
}
```

---

## ✅ **VERIFICATION CHECKLIST**

After installation, verify everything works:

### Unity 3D
```bash
& "C:\Program Files\Unity\Hub\Editor\6000.2.14f1\Editor\Unity.exe" --version
# Should show: 6000.2.14f1
```

### VRChat
- Launch VRChat
- Check Settings → OSC → Enabled
- OSC Port: 9000

### MCP Servers
```bash
# Test each MCP server
python -m unity3d_mcp --help
python -m oscmcp --help
python -m vrchat_mcp --help
python -m blender_mcp --help
python -m avatar_mcp --help
python -m robotics_mcp.server --help
```

### Cursor IDE
- Open Cursor
- Check MCP server status
- All 6 servers should show as "Connected"

---

## 🚨 **TROUBLESHOOTING**

### Unity Installation Issues
- **Hub not installing Unity**: Try downloading Unity directly from unity.com
- **Version not found**: Use Unity Hub → Installs → Add → All versions
- **Permission errors**: Run Unity Hub as Administrator

### VRChat OSC Issues
- **OSC not working**: Restart VRChat after enabling OSC
- **Port conflicts**: Check if port 9000 is available
- **Firewall**: Allow VRChat through Windows Firewall

### MCP Server Issues
- **Import errors**: Check Python path and virtual environment
- **Port conflicts**: Change default ports in server configs
- **Cursor not connecting**: Restart Cursor after adding MCP configs

---

## 🎯 **NEXT STEPS**

Once everything is installed:

1. **Start robotics-mcp server** in Cursor
2. **Test virtual robot spawning**:
   ```python
   # Spawn a virtual Scout in Unity
   await vbot_crud(
       operation="create",
       robot_type="scout",
       platform="unity",
       robot_id="test_scout"
   )
   ```
3. **Connect physical robot** (if available)
4. **Test VRChat integration** (if desired)

**Ready to build the future of robotics! 🤖🚀**