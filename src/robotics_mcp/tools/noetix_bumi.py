"""Noetix Bumi humanoid robot — info and SDK links.

Noetix Bumi: 94 cm, 12 kg, 21 DOF consumer humanoid (~$1,370). ROS/ROS2, Python, C++.
See docs/hardware/NOETIX_BUMI.md for full details.
"""

from typing import Any

from ..utils.error_handler import format_success_response

NOETIX_INFO = {
    "name": "Noetix Bumi",
    "vendor": "Noetix Robotics (Beijing) / 诺提克斯",
    "type": "humanoid",
    "specs": {
        "height_cm": 94,
        "weight_kg": 12,
        "dof": 21,
        "battery": "48V 3.5Ah, 2-3 h runtime",
        "peak_torque_nm": 50,
        "speed_ms": 0.5,
    },
    "features": [
        "Walking, running, dancing, gymnastics",
        "Reactive motion control (Base) | NVIDIA Jetson AI (Research)",
        "ROS/ROS2, Python, C++, graphical programming",
        "Voice interaction, vision, encoders, env sensors",
        "USB, HDMI, Ethernet, WiFi",
    ],
    "github": [
        "https://github.com/Noetix-Robotics/noetix_sdk_e1 (E1 SDK, C++)",
        "https://github.com/Noetix-Robotics/noetix_n2_gym (N2 RL, Isaac Gym)",
    ],
    "docs": "docs/hardware/NOETIX_BUMI.md",
    "opensource": "https://noetixrobotics.com/opensource",
}


class NoetixBumiTool:
    """Noetix Bumi humanoid — info and SDK references."""

    def __init__(self, mcp: Any):
        self.mcp = mcp

    def register(self) -> None:
        @self.mcp.tool()
        async def noetix_info(operation: str = "info") -> dict[str, Any]:
            """Noetix Bumi humanoid robot — features, specs, and SDK links.

            Operations:
                info — return features, specs, GitHub and docs links (default)

            Returns:
                success, message, and data (specs, features, github, docs).
            """
            if operation != "info":
                return {
                    "success": False,
                    "error": f"Unknown operation: {operation}. Use operation='info'.",
                    "message": f"Unknown operation: {operation}. Use operation='info'.",
                }
            return format_success_response(
                "Noetix Bumi info",
                data=NOETIX_INFO,
            )
