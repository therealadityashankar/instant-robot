---
title: Calibration & Auto-Tuning
description: Zero-touch visual auto-tuning and manual calibration tools for joint angles, wheel geometry, and camera intrinsics.
---

Instant Robot provides both **zero-touch automatic adaptation** and precision manual calibration tools.

---

## 1. Zero-Touch Visual Auto-Tuning (Recommended)

Thanks to closed-loop visual servoing with ArUco fiducials:
- **No wheel calibration needed**: The robot centers itself on tags using camera feedback directly, automatically compensating for floor friction differences (tile, wood, carpet).
- **Background Heading Snapping**: Every time the base camera sees a stationary station tag (200 & 201), the robot's global orientation snaps to ground truth without accumulating dead-reckoning drift.

---

## 2. Joint & Servo Zero Calibration

If an arm servo horn was installed slightly off-angle:
1. Open the **Calibrate** modal in the top navigation bar.
2. Select **Joint Calibration**.
3. Move the arm joints to their physical visual alignment positions (e.g. straight upright or 90° reference marks).
4. Click **Save Offsets**. The calibration is saved directly to your browser's `localStorage` and optionally exported as a `.json` profile.

---

## 3. Camera Intrinsic Calibration (ChArUco)

For sub-millimeter visual picking accuracy:
- Print the ChArUco calibration board from the **Calibrate** modal.
- Point the camera at the board from 8–12 different angles.
- The browser calculates the focal length ($f_x, f_y$), principal point ($c_x, c_y$), and lens distortion coefficients ($k_1, k_2, p_1, p_2$) in real time using OpenCV.js.
