# Pyroelectric Sensors: Super Small Motion Detection for Tiny Robots

**Ultra-small, ultra-low-power PIR sensors for heat-based motion detection - perfect for tiny robots!**

[![PIR Sensors](https://img.shields.io/badge/Pyroelectric-Motion_Detection-red)](README.md)
[![Size](https://img.shields.io/badge/Micro_Sensors-Tiny-green)](README.md)
[![Power](https://img.shields.io/badge/Ultra_Low_Power-µA-orange)](README.md)

---

## 🔥 **What are Pyroelectric Sensors?**

**Pyroelectric sensors** detect infrared (IR) radiation changes caused by moving heat sources (like people, animals, or warm objects). They're the technology behind PIR (Passive Infrared) motion sensors used in security systems and automatic lights.

### **How They Work:**

```
Heat Source → IR Radiation → Pyroelectric Crystal → Electrical Signal → Motion Detected

1. Pyroelectric crystal generates voltage when IR radiation changes
2. Fresnel lens focuses IR radiation onto sensor
3. Microcontroller detects voltage changes
4. Motion triggers robot response
```

**Key Advantages for Small Robots:**
- **Ultra-small:** < 10mm × 10mm packages
- **Ultra-low power:** < 10µA standby current
- **Passive:** No emitted radiation (unlike ultrasonic)
- **Wide detection:** 5-12m range, 110° field of view
- **Cost-effective:** $1-3 per sensor

---

## 📏 **Smallest Pyroelectric Sensors Available**

### **🥇 AM312** ⭐ **ULTRA-MICRO PIR Sensor**
- **Size:** 6mm × 4.5mm × 3.5mm (0.024" × 0.177" × 0.138")
- **Weight:** < 1g
- **Detection Range:** 5-7m, 100° horizontal, 90° vertical
- **Power:** 2.7-12V, 15µA standby, 50µA active
- **Output:** Digital (high/low) or analog
- **Price:** $1.50-2.50
- **Perfect for:** **Extremely tiny robots, wearable tech, micro-drones**

### **🥈 HC-SR501** ⭐ **Popular Mini PIR Module**
- **Size:** 32mm × 24mm × 18mm (with lens)
- **Weight:** 5g
- **Detection Range:** 3-7m (adjustable)
- **Angle:** 110° cone
- **Power:** 4.5-20V, 50µA standby
- **Output:** TTL digital signal
- **Features:** Built-in lens, adjustable sensitivity/delay
- **Price:** $1-2
- **Use Case:** **Small robot motion detection, security monitoring**

### **🥉 EKMC1603111** ⭐ **Panasonic Ultra-Compact PIR**
- **Size:** 10mm × 8mm × 5mm (lens separate)
- **Weight:** 1g
- **Detection Range:** 5m, 94° horizontal
- **Power:** 3-6V, 1µA standby, 170µA active
- **Output:** Analog voltage
- **Features:** Extremely low power, high sensitivity
- **Price:** $3-5
- **Use Case:** **Battery-powered tiny robots, IoT sensors**

### **🏃 Other Tiny Options:**

#### **D205B** ⭐ **Basic Pyroelectric Element**
- **Size:** 2mm × 2mm × 0.5mm (bare element)
- **Power:** Passive (no power needed for sensing)
- **Output:** Microvolts (needs amplification)
- **Price:** $0.50-1.00
- **Use Case:** **Custom ultra-miniature sensors**

#### **LHI968** ⭐ **Dual Element PIR**
- **Size:** 9.5mm × 9.5mm × 3mm
- **Elements:** Dual pyroelectric for direction detection
- **Power:** 3-15V, 10µA standby
- **Features:** Can detect motion direction
- **Price:** $2-4
- **Use Case:** **Advanced motion tracking, gesture recognition**

#### **RE200B** ⭐ **Low Power PIR**
- **Size:** 13mm × 10mm × 6mm
- **Power:** 2.7-12V, 3µA standby (extremely low!)
- **Detection:** 5m range, 110° angle
- **Price:** $1.50-3.00
- **Use Case:** **Long-battery-life robots, solar-powered applications**

---

## 🔋 **Power Consumption - Extremely Low!**

| Sensor | Standby Current | Active Current | Battery Life (CR2032) | Battery Life (1000mAh LiPo) |
|--------|----------------|----------------|----------------------|-----------------------------|
| **AM312** | 15µA | 50µA | **6+ months** | **50+ years** |
| **EKMC1603111** | 1µA | 170µA | **100+ months** | **500+ years** |
| **RE200B** | 3µA | 50µA | **30+ months** | **200+ years** |
| **HC-SR501** | 50µA | 100µA | **2+ months** | **10+ years** |

**These sensors can run for YEARS on small batteries!**

---

## 🤖 **Robotics Applications**

### **Motion Detection & Security:**

#### **Perimeter Monitoring:**
```python
# Pico + AM312 - Security patrol robot
import machine
import utime

pir_pin = machine.Pin(16, machine.Pin.IN)

def motion_detected(pin):
    print("Motion detected! Investigating...")
    # Take photo, sound alarm, or navigate to location

pir_pin.irq(trigger=machine.Pin.IRQ_RISING, handler=motion_detected)

# Main loop - low power sleep
while True:
    machine.lightsleep(1000)  # Wake every second to check
```

#### **Intruder Alert System:**
```python
# ESP32 + HC-SR501 - Security robot
from machine import Pin
import network
import urequests

pir = Pin(14, Pin.IN)
wifi = network.WLAN(network.STA_IF)

def send_alert():
    wifi.connect('your_wifi', 'password')
    urequests.post('http://your-server/alert', json={'motion': True})

while True:
    if pir.value():
        send_alert()
        utime.sleep(30)  # Prevent spam alerts
    utime.sleep_ms(100)
```

### **Smart Wake-Up Systems:**

#### **Power Management:**
```python
# Teensy LC + RE200B - Wake from deep sleep
#include <avr/sleep.h>

const int pirPin = 2;

void wakeUp() {
    // Empty ISR - just wake up
}

void enterSleep() {
    attachInterrupt(digitalPinToInterrupt(pirPin), wakeUp, RISING);
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    sleep_mode();
    // Code resumes here after wake-up
    sleep_disable();
    detachInterrupt(digitalPinToInterrupt(pirPin));
}

void loop() {
    // Normal robot operation
    delay(5000);

    // Go to sleep if no activity
    enterSleep();
}
```

### **People/Animal Detection:**

#### **Human Presence Detection:**
```python
# QT Py + EKMC1603111 - Person following robot
import board
import digitalio
import time

pir = digitalio.DigitalInOut(board.A1)
pir.direction = digitalio.Direction.INPUT

def follow_person():
    # Use LiDAR for distance, PIR for presence detection
    if pir.value:
        print("Person detected - following...")
        # Move towards person using LiDAR data
    else:
        print("Person lost - searching...")
        # Rotate to find person

while True:
    follow_person()
    time.sleep(0.1)
```

### **Multi-Sensor Fusion:**

#### **PIR + LiDAR + IMU:**
```python
# Complete tiny robot sensor suite
# Pico + AM312 (PIR) + YDLidar TG15 (LiDAR) + MPU6050 (IMU)

def comprehensive_detection():
    motion = pir_sensor.value()      # PIR: Is there motion?
    distance = lidar.read_distance() # LiDAR: How far?
    orientation = imu.get_yaw()      # IMU: Robot orientation

    if motion and distance < 2.0:
        print(f"Moving object {distance}m away at {orientation}°")
        # Respond to detected object
```

---

## 🔧 **Integration with Tiny Controllers**

### **Pico RP2040 + AM312:**

**Ultra-Micro Motion Detection:**
```python
# Raspberry Pi Pico + AM312 PIR sensor
import machine
import utime

# AM312 connected to GPIO 16
pir_sensor = machine.Pin(16, machine.Pin.IN, machine.Pin.PULL_DOWN)

# LED for visual feedback
led = machine.Pin(25, machine.Pin.OUT)

motion_count = 0

def motion_callback(pin):
    global motion_count
    motion_count += 1
    led.value(1)  # Turn on LED
    print(f"Motion detected! Count: {motion_count}")
    # Trigger robot action here

# Set up interrupt on rising edge
pir_sensor.irq(trigger=machine.Pin.IRQ_RISING, handler=motion_callback)

print("PIR sensor active...")
while True:
    led.value(0)  # Turn off LED
    utime.sleep(0.1)
```

### **ESP32 + HC-SR501:**

**WiFi-Enabled Motion Sensor:**
```cpp
// ESP32 + HC-SR501 with WiFi alerts
#include <WiFi.h>
#include <HTTPClient.h>

const char* ssid = "your_wifi";
const char* password = "your_password";
const int pirPin = 14;

void setup() {
    Serial.begin(115200);
    pinMode(pirPin, INPUT);

    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
    }
    Serial.println("WiFi connected");
}

void sendMotionAlert() {
    if (WiFi.status() == WL_CONNECTED) {
        HTTPClient http;
        http.begin("http://your-server/motion-alert");
        http.addHeader("Content-Type", "application/json");

        String payload = "{\"robot_id\":\"tiny-bot\",\"motion\":true}";
        int httpResponseCode = http.POST(payload);

        if (httpResponseCode > 0) {
            Serial.println("Alert sent");
        }
        http.end();
    }
}

void loop() {
    if (digitalRead(pirPin) == HIGH) {
        Serial.println("Motion detected!");
        sendMotionAlert();
        delay(5000); // Prevent spam
    }
    delay(100);
}
```

### **Arduino Nano 33 BLE + EKMC1603111:**

**Bluetooth Motion Notifications:**
```cpp
// Arduino Nano 33 BLE + Panasonic EKMC1603111
#include <ArduinoBLE.h>

const int pirPin = 2;
BLEService motionService("180F"); // Custom service
BLEBoolCharacteristic motionCharacteristic("2A19", BLERead | BLENotify);

void setup() {
    pinMode(pirPin, INPUT);
    pinMode(LED_BUILTIN, OUTPUT);

    if (!BLE.begin()) {
        Serial.println("BLE failed!");
        while (1);
    }

    BLE.setLocalName("TinyRobot");
    BLE.setAdvertisedService(motionService);
    motionService.addCharacteristic(motionCharacteristic);
    BLE.addService(motionService);
    BLE.advertise();
}

void loop() {
    BLEDevice central = BLE.central();

    if (central) {
        while (central.connected()) {
            if (digitalRead(pirPin) == HIGH) {
                motionCharacteristic.writeValue(true);
                digitalWrite(LED_BUILTIN, HIGH);
                delay(1000);
                motionCharacteristic.writeValue(false);
                digitalWrite(LED_BUILTIN, LOW);
            }
            delay(100);
        }
    }
}
```

---

## 📏 **Size Comparison - Extremely Small!**

| Sensor | Length | Width | Height | Volume | Weight | Power Standby |
|--------|--------|-------|--------|--------|--------|---------------|
| **AM312** | 6mm | 4.5mm | 3.5mm | **0.094 cm³** | <1g | 15µA |
| **D205B** | 2mm | 2mm | 0.5mm | **0.002 cm³** | <0.1g | Passive |
| **EKMC1603111** | 10mm | 8mm | 5mm | **0.4 cm³** | 1g | 1µA |
| **RE200B** | 13mm | 10mm | 6mm | **0.78 cm³** | 2g | 3µA |
| **HC-SR501** | 32mm | 24mm | 18mm | **13.82 cm³** | 5g | 50µA |

**Smallest Winners:**
1. **AM312** - 0.094 cm³ (rice grain sized!)
2. **D205B** - 0.002 cm³ (grain of sand!)
3. **EKMC1603111** - 0.4 cm³ (pea sized)

---

## 🏗️ **DIY Mounting & Integration**

### **3D-Printed Sensor Mounts:**

**Micro PIR Bracket:**
```openscad
// Ultra-small PIR mount for AM312
module pir_mount() {
    difference() {
        // Base plate
        cube([15, 12, 3]);

        // PIR sensor cutout (6x4.5mm)
        translate([4.5, 3.75, 0])
        cube([6, 4.5, 4]);

        // Lens cutout (circular)
        translate([7.5, 6, 0])
        cylinder(d=8, h=4);

        // Mounting holes
        translate([2, 2, 0]) cylinder(d=2, h=3);
        translate([13, 2, 0]) cylinder(d=2, h=3);
        translate([2, 10, 0]) cylinder(d=2, h=3);
        translate([13, 10, 0]) cylinder(d=2, h=3);
    }
}
```

### **Integration Tips:**
- **Position:** Mount at robot "eye" level (10-50cm high)
- **Angle:** Point slightly downward for ground-level detection
- **Protection:** Add mesh screen to prevent dust/debris
- **Calibration:** Adjust sensitivity for environment (indoor vs outdoor)
- **Multiple sensors:** Use 2-3 for 360° coverage on larger robots

### **Sensor Fusion Examples:**

**PIR + LiDAR Robot:**
```
Tiny Robot Configuration:
├── Controller: QT Py RP2040 (22x18mm)
├── PIR: AM312 (6x4.5mm) - Motion detection wake-up
├── LiDAR: YDLidar TG15 (forward) - Precise obstacle avoidance
├── IMU: MPU6050 - Orientation
├── Battery: 3.7V 500mAh LiPo
└── Total size: 40mm × 30mm × 25mm
```

---

## 💡 **Pro Tips for Pyroelectric Sensors**

### **Sensitivity Tuning:**
- **Indoor use:** Lower sensitivity (avoid false triggers from temperature changes)
- **Outdoor use:** Higher sensitivity (compensate for wind/air movement)
- **Temperature compensation:** Many sensors have built-in temp compensation

### **False Trigger Prevention:**
- **Pet immunity:** Some sensors ignore small animals
- **White light immunity:** Avoid triggering from light sources
- **Timing:** Use delay circuits to prevent rapid retriggering
- **Direction:** Position sensors to avoid unwanted motion detection

### **Advanced Applications:**
- **Direction detection:** Use dual-element sensors for motion direction
- **Presence detection:** Continuous sensing for occupancy detection
- **Counting:** Track people entering/leaving areas
- **Gesture recognition:** Detect specific motion patterns

### **Environmental Considerations:**
- **Temperature:** Rated -20°C to 70°C typically
- **Humidity:** Avoid condensation on lens
- **Wind:** Outdoor sensors can false-trigger in wind
- **Lighting:** IR sources (heat lamps) can interfere

---

## 📊 **Performance Comparison**

### **Detection Capabilities:**

| Sensor | Range | Angle | Response Time | Hold Time | Reset Time |
|--------|-------|-------|---------------|-----------|------------|
| **AM312** | 5-7m | 100°×90° | <2s | 2-10s | 0.5s |
| **HC-SR501** | 3-7m | 110° | <0.2s | Adjustable | Adjustable |
| **EKMC1603111** | 5m | 94° | <0.6s | Continuous | 0.5s |
| **RE200B** | 5m | 110° | <1s | 2-200s | 0.3s |

### **Cost vs Performance:**

| Price Range | Best For | Example Sensor |
|-------------|----------|----------------|
| **$0.50-1.50** | Basic motion detection | D205B, basic elements |
| **$1.50-3.00** | Tiny robot applications | AM312, RE200B |
| **$3.00-5.00** | Advanced features | EKMC1603111, LHI968 |

---

## 📚 **Resources & Datasheets**

### **Manufacturer Datasheets:**
- **AM312:** https://www.dfrobot.com/product-1440.html
- **HC-SR501:** Search "HC-SR501 datasheet"
- **Panasonic EKMC:** https://www3.panasonic.biz/ac/e/download/control/sensor/ekmc/
- **RE200B:** Search "RE200B datasheet"

### **Tutorials & Guides:**
- **Adafruit PIR Guide:** https://learn.adafruit.com/pir-passive-infrared-proximity-motion-sensor
- **SparkFun PIR Tutorial:** https://learn.sparkfun.com/tutorials/pir-motion-sensor-hookup-guide
- **Raspberry Pi PIR:** https://projects.raspberrypi.org/en/projects/physical-computing/15

### **Communities:**
- **Arduino Forum:** PIR sensor discussions
- **Raspberry Pi Forums:** Motion detection projects
- **Reddit r/arduino:** PIR sensor help

---

## 🎯 **Perfect for Tiny Robots!**

**Pyroelectric sensors are incredibly small and consume almost no power - perfect for adding motion detection to your tiny robots!**

### **Key Benefits:**
- ✅ **Ultra-small:** AM312 is 6mm × 4.5mm (rice grain sized!)
- ✅ **Ultra-low power:** 1-50µA standby (years of battery life!)
- ✅ **Easy integration:** Simple digital output, works with any microcontroller
- ✅ **Wide detection:** 5-12m range, 100°+ field of view
- ✅ **Cost-effective:** $0.50-5.00 per sensor
- ✅ **Robust:** Works in various lighting conditions, temperature compensated

### **Tiny Robot Integration:**
- **QT Py RP2040 + AM312:** Complete palm-sized robot with motion detection
- **Battery life:** 100+ months on CR2032 coin cell
- **Size:** Entire sensor suite fits in 40mm × 30mm × 25mm
- **Cost:** $10-20 total for full sensor suite

**Add sophisticated motion detection to your tiny robots without significantly increasing size or power consumption!** 🔍🤖
