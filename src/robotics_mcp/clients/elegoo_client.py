"""Elegoo Smart Robot Car client - ROS-on-PC implementation."""

import asyncio
import threading
import time

try:
    import serial
except ModuleNotFoundError:
    serial = None  # type: ignore[assignment]

import structlog

logger = structlog.get_logger(__name__)

SERIAL_REQUIRED_MSG = (
    "pyserial is not available. Start robotics-mcp with: uv --directory D:/Dev/repos/robotics-mcp run robotics-mcp"
)


def _require_serial() -> None:
    if serial is None:
        raise RuntimeError(SERIAL_REQUIRED_MSG)


class ElegooRobotConfig:
    """Configuration for Elegoo robot cars."""

    def __init__(
        self,
        robot_id: str,
        serial_port: str = "COM3",  # Windows default; override for Linux: /dev/ttyUSB0
        baud_rate: int = 115200,
        timeout: float = 1.0,
    ):
        """Initialize Elegoo robot configuration.

        Args:
            robot_id: Unique robot identifier
            serial_port: Serial port (e.g., /dev/ttyUSB0, COM3)
            baud_rate: Serial baud rate
            timeout: Serial timeout in seconds
        """
        self.robot_id = robot_id
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.timeout = timeout


class ElegooClient:
    """ROS-on-PC client for Elegoo Smart Robot Car.

    This implements the serial protocol for Elegoo robot cars,
    allowing ROS2 to run on PC while robot handles basic motor control.
    """

    def __init__(self, config: ElegooRobotConfig):
        """Initialize Elegoo robot client.

        Args:
            config: Robot configuration
        """
        self.config = config
        self.serial = None
        self.connected = False
        self._sensor_data = {}
        self._sensor_thread: threading.Thread | None = None
        self._running = False

    async def connect(self) -> bool:
        """Connect to Elegoo robot via serial.

        Returns:
            True if connection successful, False otherwise
        """
        _require_serial()
        try:
            self.serial = serial.Serial(
                port=self.config.serial_port,
                baudrate=self.config.baud_rate,
                timeout=self.config.timeout,
            )

            # Test connection with ping
            self.serial.write(b"PING\n")
            await asyncio.sleep(0.1)  # Wait for response

            if self.serial.in_waiting:
                response = self.serial.readline().decode().strip()
                if response == "PONG":
                    self.connected = True
                    self._running = True
                    self._sensor_thread = threading.Thread(target=self._sensor_loop, daemon=True)
                    self._sensor_thread.start()
                    logger.info("Elegoo robot connected", robot_id=self.config.robot_id)
                    return True

            logger.error("Elegoo robot ping failed", robot_id=self.config.robot_id)
            return False

        except Exception as e:
            logger.error("Elegoo robot connection failed", robot_id=self.config.robot_id, error=str(e))
            return False

    async def disconnect(self):
        """Disconnect from Elegoo robot."""
        self._running = False
        if self._sensor_thread and self._sensor_thread.is_alive():
            self._sensor_thread.join(timeout=2.0)

        if self.serial:
            self.serial.close()
            self.serial = None

        self.connected = False
        logger.info("Elegoo robot disconnected", robot_id=self.config.robot_id)

    async def set_motors(self, left_speed: float, right_speed: float) -> bool:
        """Set motor speeds for differential drive.

        Args:
            left_speed: Left motor speed (-1.0 to 1.0)
            right_speed: Right motor speed (-1.0 to 1.0)

        Returns:
            True if command sent successfully
        """
        if not self.connected or not self.serial:
            return False

        try:
            # Clamp speeds to valid range
            left_speed = max(-1.0, min(1.0, left_speed))
            right_speed = max(-1.0, min(1.0, right_speed))

            cmd = f"MOTORS,{left_speed:.3f},{right_speed:.3f}\n"
            self.serial.write(cmd.encode())
            logger.debug(
                "Elegoo motors set",
                robot_id=self.config.robot_id,
                left=left_speed,
                right=right_speed,
            )
            return True

        except Exception as e:
            logger.error("Elegoo motor command failed", robot_id=self.config.robot_id, error=str(e))
            return False

    async def get_sensor_data(self) -> dict:
        """Get latest sensor data from robot.

        Returns:
            Dictionary with sensor readings
        """
        return self._sensor_data.copy()

    def _sensor_loop(self):
        """Background thread for reading sensor data."""
        while self._running and self.serial:
            try:
                if self.serial.in_waiting:
                    data = self.serial.readline().decode().strip()
                    self._parse_sensor_data(data)

                time.sleep(0.05)  # 20Hz sensor reading

            except Exception as e:
                logger.error("Elegoo sensor reading error", robot_id=self.config.robot_id, error=str(e))
                time.sleep(1.0)  # Back off on errors

    def _parse_sensor_data(self, data: str):
        """Parse sensor data from Arduino.

        Expected format: "SENSORS,ultra:25.5,ir:1,0,1"
        """
        try:
            if data.startswith("SENSORS,"):
                parts = data[8:].split(",")
                if len(parts) >= 4:
                    # Parse ultrasonic
                    ultra_part = parts[0]  # "ultra:25.5"
                    ultrasonic = float(ultra_part.split(":")[1]) if ":" in ultra_part else 0.0

                    # Parse IR sensors
                    ir_part = parts[1]  # "ir:1,0,1"
                    if ":" in ir_part:
                        ir_values = ir_part.split(":")[1].split(",")
                        if len(ir_values) >= 3:
                            ir_left = int(ir_values[0])
                            ir_center = int(ir_values[1])
                            ir_right = int(ir_values[2])
                        else:
                            ir_left = ir_center = ir_right = 0
                    else:
                        ir_left = ir_center = ir_right = 0

                    # Update sensor data
                    self._sensor_data = {
                        "ultrasonic": ultrasonic,  # cm
                        "ir_left": ir_left,  # 0=obstacle, 1=clear
                        "ir_center": ir_center,  # 0=obstacle, 1=clear
                        "ir_right": ir_right,  # 0=obstacle, 1=clear
                        "timestamp": time.time(),
                    }

        except Exception as e:
            logger.error(
                "Elegoo sensor parsing error",
                robot_id=self.config.robot_id,
                data=data,
                error=str(e),
            )

    async def emergency_stop(self) -> bool:
        """Emergency stop - set all motors to zero.

        Returns:
            True if stop command sent successfully
        """
        return await self.set_motors(0.0, 0.0)

    def get_status(self) -> dict:
        """Get robot status information.

        Returns:
            Status dictionary
        """
        return {
            "robot_id": self.config.robot_id,
            "connected": self.connected,
            "serial_port": self.config.serial_port,
            "sensor_data": self._sensor_data,
            "last_update": self._sensor_data.get("timestamp", 0),
        }
