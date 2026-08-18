---
title: LeKiwi Base & Arm Assembly
description: Hardware specifications, 3D printing details, servo IDs, and wiring topology for LeKiwi and the SO-100/SO-101 arm.
---

Instant Robot supports the **LeKiwi 3-omniwheel mobile platform** paired with an **SO-100 or SO-101 6-DoF robotic arm**.

---

## Servo Bus Configuration

All 9 servos share a single serial half-duplex UART bus at **1,000,000 baud**:

| Subsystem | Servo ID | Model | Mode | Purpose |
| :--- | :---: | :--- | :---: | :--- |
| **Arm** | **1** | Feetech STS3215 | Position | Shoulder Pan (Yaw) |
| **Arm** | **2** | Feetech STS3215 | Position | Shoulder Lift (Pitch) |
| **Arm** | **3** | Feetech STS3215 | Position | Elbow Flex (Pitch) |
| **Arm** | **4** | Feetech STS3215 | Position | Wrist Flex (Pitch) |
| **Arm** | **5** | Feetech STS3215 | Position | Wrist Roll |
| **Arm** | **6** | Feetech STS3215 / SCS0009 | Position | Gripper Jaw |
| **Base** | **7** | Feetech STS3215 / SCS0009 | Wheel | Front-Left Omniwheel |
| **Base** | **8** | Feetech STS3215 / SCS0009 | Wheel | Front-Right Omniwheel |
| **Base** | **9** | Feetech STS3215 / SCS0009 | Wheel | Rear Omniwheel |

---

## 3D Printing Guidelines

- **Material**: PETG or ABS recommended for structural arm brackets and wheel mounts; PLA+ acceptable for prototyping.
- **Infill**: 40% gyroid infill with 4 perimeters for arm links.
- **Hardware**: M3 socket head screws, brass threaded heat-set inserts (M3×4mm).
