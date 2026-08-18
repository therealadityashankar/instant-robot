# lekiwi-bridge

Minimal Python package for ARM64 single-board computers (Raspberry Pi Zero 2 W, Pi 4/5, Orange Pi, etc.) bolted to the LeKiwi robot. It is a
transport bridge, nothing else: reads and writes the Feetech servo bus,
forwards native MJPEG camera frames (optimized at 5 FPS to eliminate USB bus saturation), and speaks WebRTC to whichever
browser is remotely driving the robot at [instant.river.berlin](https://instant.river.berlin).
All policy, vision (ArUco), and IK run in that browser — this package never
imports anything that reasons about the world, only things that move bytes.

That split is why there's no PyTorch, no LeRobot, no OpenCV here: a 512 MB
board with no GPU can't run any of those usefully anyway, and the moment you
need them you're better off doing it where the browser already is. Runs smoothly across most Linux ARM64 SBCs.

## Install

**`scservo_sdk` must already be in the venv.** The build this expects is a
generic Feetech fork (Dynamixel-style API — `packet.ping(port, id)` rather than
an `sms_sts` class) that isn't on PyPI, so it can't be a declared dependency:
pip would resolve it to nothing and fail the whole install. Check it's there
before anything else:

```bash
source ~/lekiwi-env/bin/activate
python3 -c "from scservo_sdk import PortHandler, PacketHandler; print('scservo_sdk ok')"
```

If that fails, install the fork into this venv first — the rest of the package
is useless without it.

```bash
cd python
pip install -e .
```

`aiortc` is the one *declared* dependency worth checking before you trust this
to just work — it pulls in `av` (ffmpeg bindings) and `cryptography`, which are
heavy source builds if no arm64 wheels match your Python:

```bash
python3 -c "import aiortc; print('aiortc ok')"
python3 -c "import v4l2py; print('v4l2py ok')"
```

## Configure

Everything is environment variables, so the same install works for every
robot — nothing device-specific is hardcoded.

| Variable | Default | Meaning |
|---|---|---|
| `LEKIWI_SIGNAL_URL` | *required* | `wss://instant.river.berlin` — same origin as the app |
| `LEKIWI_ROOM` | *required* | Room code; must match what's typed into the browser's "Remote" connect panel |
| `LEKIWI_TOKEN` | *required* | Shared secret; must match the Worker's `LEKIWI_TOKEN` |
| `LEKIWI_SERIAL_PORT` | `/dev/ttyACM0` | The CH340 adapter |
| `LEKIWI_BAUD` | `1000000` | Feetech bus baud rate |
| `LEKIWI_ARM_IDS` | `1,2,3,4,5,6` | Servo IDs that are arm joints (torque-enabled on connect) |
| `LEKIWI_WHEEL_IDS` | `7,8,9` | Servo IDs that are wheels (put in wheel mode on connect) |
| `LEKIWI_CAMERAS` | *(autodetect by-path)* | Comma-separated `/dev/v4l/by-path/...` entries, in the order the browser should see them |
| `LEKIWI_WATCHDOG_MS` | `200` | No control packet for this long → wheels stop, arm holds |

## Run

```bash
LEKIWI_SIGNAL_URL=wss://instant.river.berlin \
LEKIWI_ROOM=my-room \
LEKIWI_TOKEN=... \
lekiwi-bridge
```

It connects to the signalling room, waits for the browser to join and offer,
answers, and from then on is just moving bytes: servo writes off the `stream`
channel, reads/torque/mode off the `rpc` channel, JPEG frames out the `video`
channel. `Ctrl-C` disables torque and stops the wheels on the way out.

## Wi-Fi power save

The board's onboard hop stalls 100 ms+ on any period of quiet if power saving
is left on, which reads as intermittent lag in teleop. `lekiwi-bridge` disables
it on startup (`iw dev wlan0 set power_save off`) and does not re-enable it —
that's a one-way trade a robot bridge should make, not something to toggle per
run.
