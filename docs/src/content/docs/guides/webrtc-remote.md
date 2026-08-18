---
title: WebRTC & Remote Control
description: Setting up low-latency WebRTC video and bidirectional command streaming between your Raspberry Pi and web browser.
---

Instant Robot uses peer-to-peer **WebRTC** to stream dual-camera video feeds and bidirectional motor telemetry between the robot and any web browser worldwide.

---

## 1. Raspberry Pi Setup

Ensure your Raspberry Pi has Python 3.10+ and the required packages installed:

```bash
cd instant-robot/python
pip install -e .
```

### Running the Bridge Daemon

```bash
# Run interactively
lekiwi-bridge --room my-lekiwi@yourpass

# Or manage via systemd
sudo systemctl enable --now lekiwi-bridge
```

---

## 2. Connecting from the Web App

1. Open **[instant.river.berlin](https://instant.river.berlin)**.
2. Click **Connect robot** → **Remote (WebRTC)**.
3. Enter your room identifier (e.g. `my-lekiwi@yourpass`).
4. Click **Connect**. The signaling worker exchanges SDP offers/answers and ICE candidates, establishing a direct peer-to-peer connection within 1–2 seconds.

---

## 3. Protocol Features

- **Binary DataChannel Transport**: Servo positions and wheel speed packets use compact binary schemas (< 64 bytes per frame) for sub-millisecond serialization.
- **SCTP Backpressure Control**: The bridge monitors `channel.bufferedAmount` to automatically drop non-critical video frames during network congestion, preventing latency buildup.
