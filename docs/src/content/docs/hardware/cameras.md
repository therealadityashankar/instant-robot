---
title: Dual-Camera Topology
description: Mounting, resolution, and switching architecture for the Base wide-angle camera and Arm wrist camera.
---

Instant Robot leverages two independent cameras to handle navigation and high-precision manipulation simultaneously.

---

## 1. Base Navigation Camera (`/dev/video0`)
- **Mount Location**: Front center of the LeKiwi chassis, angled slightly forward/down.
- **Lens / FOV**: Wide-angle lens ($90^\circ$–$120^\circ$ FOV).
- **Primary Function**:
  - Scanning the room for stationary docking tags (IDs 200 & 201).
  - Visual servoing during base navigation.
  - OCR text reading on station labels.

---

## 2. Arm Wrist Camera (`/dev/video1`)
- **Mount Location**: Mounted directly behind the gripper wrist on link 5/6.
- **Lens / FOV**: Narrower FOV ($60^\circ$–$75^\circ$) for sharp closeup detail.
- **Primary Function**:
  - Close-range tag detection on objects to pick up (IDs 1–199).
  - Visual depth verification and fine gripper jaw alignment.

---

## Clean Hardware Switching

The Raspberry Pi bridge software (`lekiwi_bridge`) implements **V4L2 clean buffer teardown** (`munmap` + `REQBUFS(0)`):
- Only one camera streams full-rate MJPEG at a time to minimize CPU and Wi-Fi congestion.
- Switching between Base and Arm feeds in the web interface takes under **300 ms** with zero kernel buffer lockups or device busy errors.
