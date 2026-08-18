---
title: Quickstart Guide
description: Get started with Instant Robot simulation, hardware connection, and basic navigation in 5 minutes.
---

Instant Robot gives you a full robotics simulator and real-hardware bridge running entirely in the browser with **zero local installation required**.

---

## 1. Try the In-Browser Simulator

1. Navigate to **[instant.river.berlin](https://instant.river.berlin)**.
2. Click **Explore** or use the **WASD / Q / E** on-screen controls to drive the 3-omniwheel LeKiwi base around the workspace.
3. Drag the target marker to test analytical Inverse Kinematics (IK) for the 6-DoF robotic arm.
4. Watch the simulated camera feeds detect the station fiducial tags in real-time.

---

## 2. Print Your Physical Tags

To navigate in the physical world, your robot uses **6×6 ArUco fiducials**:

1. Open the [**Printable Tag Generator**](/docs/tags-and-navigation/tag-generator/).
2. Generate your station cards (e.g. **DOCKING STATION**, **APPLE**, **SHELF**).
3. Print the cards at **100% scale (no fit-to-page)** and attach them to your stations or objects.

---

## 3. Connect to Physical Hardware

Instant Robot supports two connection modes:

### Local Mode (USB Serial)
- Plug your Feetech servo bus directly into your computer via a USB-to-UART adapter.
- Click **Connect robot** → **USB Serial** in the web app and select the serial port.

### Remote Mode (WebRTC via Raspberry Pi)
- Power on your Raspberry Pi running the `lekiwi-bridge` daemon.
- In the app, click **Connect robot** → **Remote (WebRTC)** and enter your room key.
- The web app establishes an end-to-end WebRTC DataChannel + Video stream, allowing full teleoperation and autonomous visual navigation over Wi-Fi.
