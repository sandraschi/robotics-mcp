# Tiny Controller CPU Boards: Pico & Micro for Small Robots

**Smallest microcontroller boards for robotics - pico-sized power for tiny bots**

[![Microcontrollers](https://img.shields.io/badge/Microcontrollers-RP2040/ESP32-blue)](README.md)
[![Size](https://img.shields.io/badge/Smallest-Boards-green)](README.md)
[![Power](https://img.shields.io/badge/Low_Power-Consumption-orange)](README.md)

---

## 🎯 **Why Tiny Controllers Matter for Small Robots**

**Small robots need small brains.** When adding sensors and effectors to palm-sized robots, you can't use full-sized Raspberry Pi or Arduino Uno boards. You need **pico-sized controllers** that fit in your 3D-printed enclosures.

### **Key Requirements for Small Robot Controllers:**
- **Size:** < 5cm × 3cm footprint (ideally < 3cm × 2cm)
- **Power:** < 100mA active, < 50µA sleep mode
- **Weight:** < 10g (including headers)
- **I/O:** UART, I2C, SPI, ADC, PWM, GPIO
- **Processing:** Enough for sensor fusion and basic control
- **Cost:** $5-15 for single units

**Perfect for:** LiDAR integration, motor control, sensor fusion, IMU processing, basic navigation

---

## 🏆 **Smallest "Pico" Controllers**

### **🥇 Raspberry Pi Pico (RP2040)** ⭐ **WINNER - Smallest Full-Featured**

- **Size:** 21mm × 51mm × 4mm (0.8" × 2" × 0.16")
- **Weight:** 3g (with headers: 5g)
- **MCU:** RP2040 dual-core Cortex-M0+ @ 133MHz
- **RAM:** 264KB SRAM
- **Flash:** 2MB onboard
- **Power:** 2.3-5.5V, 17mA active, 0.8mA sleep
- **GPIO:** 26 pins (23 digital, 3 ADC)
- **Interfaces:** 2× UART, 2× I2C, 2× SPI, 16× PWM
- **USB:** Micro-USB (USB 1.1 host/device)
- **Price:** $4-6
- **Programming:** MicroPython, C/C++, CircuitPython
- **ROS Support:** Micro-ROS, ros2arduino
- **Why for Robots:** Excellent I/O, fast processing, great community

### **🥈 Raspberry Pi Pico W** ⭐ **WiFi-Enabled Pico**

- **Size:** Same as Pico (21mm × 51mm)
- **Weight:** 4g (with headers: 6g)
- **Extra:** Integrated CYW43439 WiFi/Bluetooth
- **Power:** 18mA active (WiFi), 1.2mA sleep
- **Price:** $6-8
- **Use Case:** **Wireless robot control, sensor networks, remote monitoring**

### **🥉 Adafruit QT Py RP2040** ⭐ **Extremely Tiny RP2040**

- **Size:** 22mm × 18mm × 5mm (0.9" × 0.7" × 0.2")
- **Weight:** 2g
- **MCU:** RP2040 (same as Pico)
- **Pins:** STEMMA QT connector (I2C) + 5 GPIO
- **Power:** 2.3-5.5V, same low power as Pico
- **Price:** $6-8
- **Why Tiny:** **Extremely compact**, perfect for embedding in small robots
- **Use Case:** **Minimal footprint applications, sensor integration**

---

## 📏 **Other Tiny Controllers**

### **ESP32 Series** ⭐ **WiFi/BT Built-in**

#### **ESP32-PICO-D4** ⭐ **Extremely Small ESP32**
- **Size:** 7mm × 7mm (chip only, needs breakout board)
- **MCU:** Dual-core Xtensa LX6 @ 240MHz
- **RAM:** 520KB
- **Flash:** External (up to 16MB)
- **Power:** 5V, 80mA active, 5µA deep sleep
- **WiFi/BT:** Integrated
- **GPIO:** 36 pins (configurable)
- **Price:** $3-5 (chip), $8-12 (breakout)
- **Use Case:** **Ultra-small WiFi robots, IoT sensor nodes**

#### **ESP32-C3** ⭐ **RISC-V ESP32**
- **Size:** 5mm × 5mm (chip), various breakout sizes
- **MCU:** Single-core RISC-V @ 160MHz
- **RAM:** 400KB
- **WiFi/BT:** WiFi 4 + BT 5
- **Power:** Very low power (10µA deep sleep)
- **Price:** $2-4
- **Why Unique:** **RISC-V architecture, excellent for custom robotics firmware**

### **Arduino Nano Series** ⭐ **Classic Tiny Form Factor**

#### **Arduino Nano 33 BLE** ⭐ **Bluetooth + IMU**
- **Size:** 18mm × 45mm × 5mm
- **Weight:** 5g
- **MCU:** nRF52840 Cortex-M4 @ 64MHz
- **RAM:** 256KB
- **Flash:** 1MB
- **Bluetooth:** BLE 5.0
- **Sensors:** Built-in IMU (accelerometer + gyro)
- **Power:** 3.3V, 10mA active
- **Price:** $15-20
- **Use Case:** **Tiny robots with built-in motion sensing**

#### **Arduino Nano ESP32** ⭐ **WiFi + Bluetooth**
- **Size:** 18mm × 45mm (same as Nano form factor)
- **MCU:** ESP32-S3 @ 240MHz
- **WiFi/BT:** Full ESP32 capabilities
- **RAM:** 512KB
- **Price:** $18-25
- **Use Case:** **WiFi-enabled tiny robots, IoT robotics**

### **Teensy Series** ⭐ **High Performance Tiny**

#### **Teensy 4.0** ⭐ **Powerful in Tiny Package**
- **Size:** 35mm × 18mm × 4mm
- **Weight:** 2g
- **MCU:** iMXRT1062 Cortex-M7 @ 600MHz
- **RAM:** 1MB
- **Flash:** 2MB
- **USB:** High-speed USB 2.0
- **Power:** 3.6-6V, 100mA active
- **Price:** $20-25
- **Why Powerful:** **600MHz performance in tiny package**
- **Use Case:** **High-performance tiny robots, real-time control**

#### **Teensy LC** ⭐ **Ultra Low Power**
- **Size:** 35mm × 18mm × 4mm
- **MCU:** MKL26Z64 Cortex-M0+ @ 48MHz
- **RAM:** 8KB
- **Flash:** 62KB
- **Power:** **9µA sleep mode** (extremely low!)
- **Price:** $12-15
- **Use Case:** **Battery-powered tiny robots, long runtime**

### **Other Tiny Options**

#### **Seeeduino Xiao** ⭐ **Arduino-Compatible Tiny**
- **Size:** 20mm × 17.5mm × 3.5mm
- **MCU:** SAMD21 Cortex-M0+ @ 48MHz (or RP2040 variants)
- **Price:** $5-15
- **Use Case:** **Arduino ecosystem in tiny form factor**

#### **Circuit Playground Express** ⭐ **Educational Tiny**
- **Size:** 51mm diameter (circular)
- **MCU:** SAMD21 @ 48MHz
- **Built-in:** NeoPixels, speaker, sensors, buttons
- **Price:** $25
- **Use Case:** **Learning robotics, sensor experimentation**

---

## 🔋 **Power Consumption Comparison**

| Board | Active Current | Sleep Current | Battery Life (1000mAh) |
|-------|----------------|----------------|-------------------------|
| **RPi Pico** | 17mA | 0.8mA | **60+ hours** |
| **RPi Pico W** | 18mA (WiFi) | 1.2mA | **50+ hours** |
| **ESP32-PICO-D4** | 80mA | 5µA | **12+ hours** |
| **Teensy LC** | 10mA | **9µA** | **110+ hours** |
| **Arduino Nano 33** | 10mA | 10µA | **100+ hours** |

**Key Insight:** Modern micros can run for days/weeks on small batteries!

---

## 🤖 **Robotics Applications**

### **LiDAR Integration Examples**

#### **Raspberry Pi Pico + YDLidar TG15**
```python
# MicroPython on Pico - Forward LiDAR obstacle avoidance
import machine
import utime
from machine import UART, Pin

# UART for LiDAR communication
lidar = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1))

# Motor control pins
left_motor = machine.PWM(machine.Pin(2))
right_motor = machine.PWM(machine.Pin(3))

def get_distance():
    # Read LiDAR distance data
    if lidar.any():
        data = lidar.read(9)  # TG15 data packet
        distance = (data[3] << 8) + data[2]  # Parse distance
        return distance * 0.01  # Convert to meters
    return 10.0  # Default safe distance

while True:
    dist = get_distance()
    if dist < 0.5:  # Obstacle within 50cm
        left_motor.duty_u16(0)   # Stop
        right_motor.duty_u16(0)
    else:
        left_motor.duty_u16(32768)  # Forward
        right_motor.duty_u16(32768)
    utime.sleep(0.1)
```

#### **ESP32-PICO-D4 + RPLIDAR A1**
```cpp
// ESP32 Arduino framework - 360° LiDAR mapping
#include <HardwareSerial.h>
#include <ESP32Servo.h>

HardwareSerial lidarSerial(1); // UART1
Servo panServo, tiltServo;

void setup() {
    lidarSerial.begin(115200, SERIAL_8N1, 16, 17); // RX, TX pins
    panServo.attach(18);  // Pan servo for scanning
    tiltServo.attach(19); // Tilt servo for 3D scanning
}

void loop() {
    // Read RPLIDAR data
    while (lidarSerial.available()) {
        // Parse RPLIDAR protocol
        // Build 360° distance map
        // Control servos for 3D scanning
    }
}
```

### **Sensor Fusion Examples**

#### **Pico IMU + LiDAR Fusion**
```python
# IMU + LiDAR sensor fusion on Pico
import imu
import math

# MPU6050 IMU
imu = imu.MPU6050()

def get_robot_orientation():
    accel = imu.accel()
    gyro = imu.gyro()
    # Simple complementary filter
    return math.atan2(accel[1], accel[0])  # Simplified yaw

def navigate_to_goal(current_pos, goal_pos, orientation):
    # Use LiDAR for obstacle avoidance
    # Use IMU for orientation
    # Calculate navigation commands
    pass
```

### **Motor Control Examples**

#### **Tiny Servo Control (QT Py)**
```python
# Adafruit QT Py RP2040 - Servo control
import board
import pwmio
import servo

# Create PWM outputs for servos
pwm1 = pwmio.PWMOut(board.A1, frequency=50)
pwm2 = pwmio.PWMOut(board.A2, frequency=50)

servo1 = servo.Servo(pwm1)
servo2 = servo.Servo(pwm2)

# Control robot arm or gripper
servo1.angle = 90  # Center position
servo2.angle = 45  # Gripper open
```

---

## 🛠️ **Integration with ROS**

### **Micro-ROS on Pico**

**Micro-ROS** brings ROS 2 to microcontrollers:

```bash
# Install micro-ROS for RP2040
pip install micro_ros_setup
ros2 run micro_ros_setup create_firmware_ws.sh
ros2 run micro_ros_setup build_firmware.sh
```

**Pico Micro-ROS Example:**
```c
// Pico C/C++ with Micro-ROS
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <sensor_msgs/msg/laser_scan.h>

rcl_publisher_t publisher;
sensor_msgs__msg__LaserScan laser_msg;

void setup() {
    // Initialize Micro-ROS
    set_microros_transports();
    // Create publisher for LiDAR data
    // Publish sensor readings to ROS 2
}

void loop() {
    // Read LiDAR sensor
    // Fill laser_msg with data
    // Publish to ROS 2 topic
    rcl_publish(&publisher, &laser_msg, NULL);
}
```

### **ROS Serial on ESP32**

```cpp
// ESP32 with rosserial
#include <ros.h>
#include <sensor_msgs/LaserScan.h>

ros::NodeHandle nh;
sensor_msgs::LaserScan scan_msg;
ros::Publisher scan_pub("scan", &scan_msg);

void setup() {
    nh.initNode();
    nh.advertise(scan_pub);
}

void loop() {
    // Read LiDAR data
    // Fill scan_msg
    scan_pub.publish(&scan_msg);
    nh.spinOnce();
    delay(100);
}
```

---

## 📏 **Size Comparison**

| Board | Length | Width | Thickness | Volume | Weight | Power Active |
|-------|--------|-------|-----------|--------|--------|--------------|
| **QT Py RP2040** | 22mm | 18mm | 5mm | **2.0 cm³** | 2g | 17mA |
| **RPi Pico** | 51mm | 21mm | 4mm | **4.3 cm³** | 3g | 17mA |
| **Pico W** | 51mm | 21mm | 4mm | **4.3 cm³** | 4g | 18mA |
| **ESP32-PICO-D4** | 52mm | 20mm | 5mm | **5.2 cm³** | 3g | 80mA |
| **Teensy 4.0** | 35mm | 18mm | 4mm | **2.5 cm³** | 2g | 100mA |
| **Arduino Nano** | 45mm | 18mm | 5mm | **4.0 cm³** | 5g | 15mA |

**Smallest Winners:**
1. **QT Py RP2040** - 2.0 cm³ (extremely tiny!)
2. **Teensy 4.0** - 2.5 cm³ (powerful in small package)
3. **RPi Pico** - 4.3 cm³ (most capable general-purpose)

---

## 🏗️ **DIY Mounting Solutions**

### **3D-Printed Controller Enclosures**

**Pico-Sized Controller Box:**
```openscad
// Tiny controller enclosure
module controller_box() {
    // Main box for QT Py RP2040 (22x18mm)
    difference() {
        cube([30, 26, 10]); // External dimensions

        // Internal cavity
        translate([2, 2, 2])
        cube([26, 22, 8]); // 22x18mm board area

        // Cable access holes
        translate([15, 0, 5]) cylinder(d=6, h=3);
        translate([15, 26, 5]) cylinder(d=6, h=3);

        // Ventilation slots
        for(x=[5, 10, 15, 20, 25]) {
            translate([x, 0, 8]) cube([1, 26, 2]);
        }
    }
}
```

### **Mounting Tips**
- **Heat dissipation:** Small controllers can get warm - add ventilation
- **Cable management:** Use right-angle connectors to save space
- **Power stability:** Add capacitors for motor noise filtering
- **EMI shielding:** Metal enclosures for noisy environments
- **Weight:** Every gram matters for small robots

---

## 🔌 **Sensor Integration Examples**

### **LiDAR + IMU + Motor Control**

**Complete Tiny Robot Controller Setup:**

```
QT Py RP2040 (22x18mm)
├── LiDAR: YDLidar TG15 (UART)
├── IMU: MPU6050 (I2C)
├── Motors: 2× Micro servos (PWM)
├── Battery: 3.7V LiPo (charging circuit)
└── Total weight: < 20g
```

**Code Structure:**
```python
# sensor_fusion.py
def read_sensors():
    lidar_dist = read_lidar()
    accel, gyro = read_imu()
    return sensor_data

# control.py
def control_loop(sensor_data):
    if sensor_data.lidar < 0.3:  # Obstacle
        stop_motors()
        turn_random()
    else:
        move_forward()

# main.py
while True:
    data = read_sensors()
    control_loop(data)
    time.sleep(0.05)  # 20Hz control loop
```

---

## 💡 **Pro Tips for Tiny Controllers**

### **Power Management**
- **Sleep modes:** Use deep sleep for battery-powered robots
- **Voltage monitoring:** Track battery level
- **Current limiting:** Protect small power supplies
- **Regulators:** Efficient buck/boost converters

### **Processing Optimization**
- **Interrupt-driven:** Don't poll sensors in loops
- **DMA transfers:** For high-speed sensor data
- **Fixed-point math:** Avoid floating point when possible
- **Lookup tables:** Pre-compute complex functions

### **Debugging Tiny Systems**
- **LED indicators:** Status LEDs for debugging
- **Serial logging:** UART debug output
- **Watchdog timers:** Recover from hangs
- **Reset buttons:** Easy access for development

### **Environmental Considerations**
- **Temperature:** Many micros rated -40°C to 85°C
- **Humidity:** Conformal coating for wet environments
- **Vibration:** Shock-absorbing mounts
- **EMI:** Shielding for electrically noisy robots

---

## 📚 **Resources**

### **Official Documentation**
- **Raspberry Pi Pico:** https://www.raspberrypi.com/documentation/microcontrollers/pico-series/
- **ESP32:** https://docs.espressif.com/projects/esp32/
- **Arduino:** https://docs.arduino.cc/hardware/nano-33-ble/
- **Teensy:** https://www.pjrc.com/teensy/

### **Robotics Libraries**
- **Micro-ROS:** https://micro.ros.org/
- **rosserial:** http://wiki.ros.org/rosserial
- **CircuitPython:** https://circuitpython.org/

### **Communities**
- **Raspberry Pi Forums:** Pico-specific discussions
- **ESP32 Forum:** https://esp32.com/
- **Arduino Forum:** Nano and microcontroller help
- **Reddit r/embedded:** General microcontroller advice

---

## 🎯 **Choosing Your Tiny Controller**

### **For Beginners:**
- **Raspberry Pi Pico** - Easy to program, great community, all the features you need

### **For WiFi/BT Projects:**
- **Raspberry Pi Pico W** or **ESP32 series** - Built-in wireless capabilities

### **For Ultra-Tiny Projects:**
- **QT Py RP2040** or **ESP32-PICO-D4** - Smallest footprints available

### **For Low Power:**
- **Teensy LC** or **ESP32-C3** - Excellent sleep modes, long battery life

### **For ROS Integration:**
- **Any RP2040 board** with Micro-ROS support

**Key Insight:** Modern tiny controllers have more processing power than desktop computers from 20 years ago, but in packages smaller than a postage stamp. The bottleneck is often your imagination, not the hardware capabilities!

---

**Tiny controllers have revolutionized small robotics - you can now build sophisticated robots smaller than a deck of cards with capabilities that would have required room-sized computers just a decade ago!** 🤖🎮
