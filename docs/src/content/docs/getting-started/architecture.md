---
title: System Architecture
description: Understand how the browser frontend, MuJoCo WASM, WebRTC transport, and the Raspberry Pi hardware bridge communicate.
---

Instant Robot bridges browser simulation and physical robotics through a unified protocol.

```
┌─────────────────────────────────────────────────────────────┐
│                    Web Browser (Frontend)                   │
│                                                             │
│  ┌────────────────────┐  ┌───────────────────────────────┐  │
│  │  Svelte UI & State │  │  MuJoCo Physics (WASM)        │  │
│  └─────────┬──────────┘  └───────────────┬───────────────┘  │
│            │                             │                  │
│  ┌─────────▼─────────────────────────────▼───────────────┐  │
│  │   Kinematics & Analytical IK Solver / Visual Servoing │  │
│  └───────────────────────┬───────────────────────────────┘  │
│                          │                                  │
│             ┌────────────┴────────────┐                     │
│             │   RobotHub Abstraction  │                     │
│             └─────┬─────────────┬─────┘                     │
└───────────────────┼─────────────┼───────────────────────────┘
                    │ (USB)       │ (WebRTC DataChannel + Video)
                    ▼             ▼
       ┌──────────────────┐  ┌────────────────────────────────┐
       │ WebSerial Driver │  │   Raspberry Pi Bridge Daemon   │
       │ (Direct Laptop)  │  │   (Python aiortc + V4L2)       │
       └──────────────────┘  └───────────────┬────────────────┘
                                             │
                              ┌──────────────▼──────────────┐
                              │ Feetech STS3215/SCS0009 Bus │
                              │ (Arm Servos 1-6, Wheels 7-9)│
                              └─────────────────────────────┘
```

---

## Key Components

### 1. Browser Application
- **MuJoCo WebAssembly**: Provides real-time rigid body dynamics, joint simulation, and forward kinematics.
- **Analytical IK Engine**: Solves 6-DoF arm angles in under **0.2 ms** with zero numerical iteration drift.
- **OpenCV.js Vision Worker**: Detects ArUco fiducials at 60 FPS and estimates 6-DoF camera poses via homography and `solvePnP`.

### 2. Signalling Worker (Cloudflare Durable Objects)
- Coordinates WebRTC connection setup between browsers and robots without storing or relaying any video data (zero bandwidth overhead).

### 3. Robot Bridge Daemon (`lekiwi_bridge`)
- Runs on Raspberry Pi (e.g. Pi Zero 2 W or Pi 4/5).
- Captures hardware-accelerated MJPEG frames from USB/CSI cameras (`/dev/video0` and `/dev/video1`) via Linux V4L2.
- Handles dual-camera switching without locking kernel video buffers.
- Streams high-frequency servo positions and wheel velocity commands over binary WebRTC DataChannels.
