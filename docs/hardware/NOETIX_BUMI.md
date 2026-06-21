# Noetix Bumi — Consumer humanoid robot

**Noetix Bumi** (诺提克斯 BUMI) is a compact consumer-grade humanoid from Noetix Robotics (Beijing) — 94 cm, 12 kg, 21 DOF, ~$1,370 (9,998 CNY). Often referred to as “Bumi Android” in listings.

## Features

- **Motion**: Walking, running, dancing, gymnastics; speed >0.5 m/s; force-controlled joints, 50 N·m peak torque.
- **Runtime**: 2–3 h on 48 V 3.5 Ah battery; quick-release, tool-free swap in &lt;5 s.
- **Programming**: Drag-and-drop graphical (education), ROS/ROS2, open APIs, Python, C++.
- **Hardware**: Voice mics, vision, encoders, env sensors; USB, HDMI, Ethernet, WiFi; &gt;90% internal wiring.
- **Ecosystem**: JD.com Joy Inside 2.0; multimodal LLM development; native mobile apps; cross-platform SDKs (Linux, ROS).

## GitHub / SDK

- **Noetix-Robotics/noetix_sdk_e1** — C++ SDK for E1 robot (high/low controller): [github.com/Noetix-Robotics/noetix_sdk_e1](https://github.com/Noetix-Robotics/noetix_sdk_e1)  
  BSD-3-Clause; Ubuntu 20.04/22.04, aarch64/x86_64; build: `./build_release.sh`
- **Noetix-Robotics/noetix_n2_gym** — N2 humanoid RL (Isaac Gym, PPO, sim2sim): [github.com/Noetix-Robotics/noetix_n2_gym](https://github.com/Noetix-Robotics/noetix_n2_gym)
- **Bumi**: Vendor states open SDK support for Linux/ROS (C++, Python, graphical). No separate “Bumi SDK” repo found; E1/N2 SDKs and noetix_n2_gym are the main public repos. For Bumi-specific APIs, check Noetix opensource page: [noetixrobotics.com/opensource](https://noetixrobotics.com/opensource).

## Integration with robotics-mcp

Use **noetix_info** (or **robot_control** with robot_type `noetix_bumi`) for links and setup hints. Full control would require a Bumi-specific SDK or ROS nodes; when available, a dedicated Noetix/Bumi client can be added alongside Dreame and Elegoo.
