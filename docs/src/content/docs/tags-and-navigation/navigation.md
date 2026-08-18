---
title: Visual Navigation & Servoing
description: How the robot navigates autonomously between stationary fiducials using closed-loop visual servoing.
---

Instant Robot relies on **Closed-Loop Visual Servoing** rather than open-loop wheel dead-reckoning.

---

## Why Visual Servoing?

Traditional small mobile robots suffer from wheel slip, varying floor friction (tile vs carpet), and battery voltage drop that cause dead-reckoning (odometry) to drift rapidly.

By using stationary ArUco fiducials as **active visual beacons**, the robot achieves repeatable millimeter-level positioning without needing expensive wheel encoders or LiDAR.

```
       Visual Camera View               Closed-Loop P Controller
   ┌─────────────────────────┐
   │                         │          e_yaw = θ_target - θ_observed
   │     [200]     [201]     │   ────►  Turn Speed = Kp * e_yaw
   │       ▲                 │
   │       │                 │          e_dist = z_target - z_observed
   │     Center              │   ────►  Forward Speed = Kp * e_dist
   └─────────────────────────┘
```

---

## Navigation Phases

### 1. Far-Range Approach
- The robot navigates toward the estimated world coordinates of the station tag.
- If the tag is outside the camera field of view, the robot executes a continuous scan rotation until the markers (IDs 200 & 201) enter the camera frame.

### 2. Heading Alignment
- Once the tag pair is visible, the angle error $e_{\theta}$ is computed from the horizontal centroid offset in the camera frame:
  $$\text{turn\_cmd} = K_p \cdot (\theta_{\text{target}} - \theta_{\text{tag}})$$
- The robot turns on the spot until the station is perfectly centered in the forward camera feed.

### 3. Precision Standoff & Squaring Up
- Using the known distance between markers 200 and 201 (or solving for 3D translation vector $\vec{t}$), the robot drives straight forward while maintaining square orientation.
- It settles at a precise standoff distance (e.g., $34\text{ cm}$ for scanning or $18\text{ cm}$ for picking).
