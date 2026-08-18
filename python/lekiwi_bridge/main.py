"""Entrypoint: wires the servo bus, cameras and WebRTC peer together and runs
until interrupted. `lekiwi-bridge` on the path after `pip install -e .`.
"""

from __future__ import annotations

import asyncio
import json
import logging
import subprocess

from .bus import ServoBus
from .camera import Camera
from .config import Config
from .protocol import (
    MAX_VIDEO_FRAME,
    PROTOCOL_VERSION,
    CH_RPC,
    CH_STREAM,
    CH_VIDEO,
    encode_video_frame,
    rpc_err,
    rpc_ok,
)
from .watchdog import Watchdog
from .webrtc import Peer

log = logging.getLogger("lekiwi_bridge.main")


def _disable_wifi_power_save() -> None:
    # Best-effort: periodic 100ms+ stalls otherwise, which read as lag in
    # teleop. Not fatal if this fails (e.g. no wlan0, no permission) — log and
    # keep going rather than block startup on it.
    try:
        subprocess.run(
            ["iw", "dev", "wlan0", "set", "power_save", "off"],
            check=True,
            capture_output=True,
            timeout=5,
        )
        log.info("wifi power save disabled")
    except Exception as e:
        log.warning("could not disable wifi power save: %s", e)


class CameraManager:
    def __init__(self, cameras: list[Camera]):
        self.cameras = cameras
        self.active_index = 0
        self.changed = asyncio.Event()

    def set_active(self, index: int) -> int:
        if 0 <= index < len(self.cameras):
            if index != self.active_index:
                if self.active_index < len(self.cameras):
                    self.cameras[self.active_index].close()
                self.active_index = index
                self.changed.set()
                log.info("active camera switched to %d", index)
        return self.active_index


async def handle_rpc(
    bus: ServoBus,
    cfg: Config,
    servos: list[int],
    cam_mgr: CameraManager,
    msg: dict,
) -> dict:
    op = msg.get("op")
    args = msg.get("args") or {}
    rid = msg["id"]
    try:
        if op == "hello":
            return rpc_ok(rid, {"version": PROTOCOL_VERSION, "servos": servos, "cameras": len(cam_mgr.cameras)})
        if op == "ping":
            return rpc_ok(rid, "pong")
        if op == "readPosition":
            pos = await bus.read_position(int(args["servoId"]))
            return rpc_ok(rid, pos)
        if op == "syncReadPositions":
            positions = await bus.sync_read_positions([int(i) for i in args["servoIds"]])
            return rpc_ok(rid, list(positions.items()))
        if op == "writeTorqueEnable":
            await bus.write_torque_enable([int(i) for i in args["servoIds"]], bool(args["enable"]))
            return rpc_ok(rid)
        if op == "setWheelMode":
            await bus.set_wheel_mode([int(i) for i in args["servoIds"]])
            return rpc_ok(rid)
        if op == "stopAll":
            await bus.stop_all(cfg.wheel_ids)
            return rpc_ok(rid)
        if op == "setActiveCamera":
            idx = cam_mgr.set_active(int(args.get("camera", 0)))
            return rpc_ok(rid, {"activeCamera": idx})
        return rpc_err(rid, f"unknown op {op!r}")
    except Exception as e:  # noqa: BLE001 - reported to the caller, not swallowed
        return rpc_err(rid, str(e))


async def run_async() -> None:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(levelname)s %(message)s")
    cfg = Config.from_env()
    _disable_wifi_power_save()

    bus = ServoBus(cfg.serial_port, cfg.baud)
    await bus.connect()
    log.info("bus connected on %s @ %d baud", cfg.serial_port, cfg.baud)

    servos = await bus.probe(range(1, 16))
    log.info("servos found: %s", servos)
    await bus.write_torque_enable(cfg.arm_ids, True)
    if cfg.wheel_ids:
        await bus.set_wheel_mode(cfg.wheel_ids)

    watchdog = Watchdog(bus, cfg.wheel_ids, cfg.watchdog_s)
    watchdog.start()

    cameras = []
    for i, path in enumerate(cfg.camera_paths):
        cam = Camera(path, i)
        try:
            cam.open()
            cam.close()
            cameras.append(cam)
            log.info("camera %d probed ok: %s", i, path)
        except Exception as e:
            log.warning("camera %s failed to open: %s", path, e)
    cam_mgr = CameraManager(cameras)

    peer: Peer | None = None

    def on_rpc_open(channel):
        @channel.on("message")
        def on_message(message):
            try:
                msg = json.loads(message)
            except Exception:
                return
            asyncio.ensure_future(_reply(channel, bus, cfg, servos, cam_mgr, msg))

        async def _reply(channel, bus, cfg, servos, cam_mgr, msg):
            reply = await handle_rpc(bus, cfg, servos, cam_mgr, msg)
            try:
                channel.send(json.dumps(reply))
            except Exception:
                pass

    def on_stream_open(channel):
        @channel.on("message")
        def on_message(message):
            try:
                msg = json.loads(message)
            except Exception:
                return
            watchdog.kick()
            op = msg.get("op")
            if op == "pos":
                targets = {int(sid): int(pos) for sid, pos in msg.get("targets", [])}
                asyncio.ensure_future(bus.sync_write_positions(targets))
            elif op == "wheel":
                speeds = {int(sid): int(spd) for sid, spd in msg.get("speeds", [])}
                asyncio.ensure_future(bus.sync_write_wheel_speed(speeds))

    cam_tasks: list[asyncio.Task] = []

    def on_video_open(channel):
        log.info("video channel open, streaming single active camera (%d total available)", len(cameras))
        cam_tasks.append(asyncio.ensure_future(_stream_camera(channel, cam_mgr)))

    async def _stream_camera(channel, cam_mgr: CameraManager):
        seq = 0
        sent = 0
        while channel.readyState == "open":
            idx = cam_mgr.active_index
            if not (0 <= idx < len(cam_mgr.cameras)):
                await asyncio.sleep(0.2)
                continue
            cam = cam_mgr.cameras[idx]
            cam_mgr.changed.clear()
            try:
                async for jpeg in cam.frames():
                    if channel.readyState != "open" or cam_mgr.changed.is_set():
                        break
                    if len(jpeg) > MAX_VIDEO_FRAME:
                        log.warning(
                            "camera %d: frame too large (%d bytes), dropping", cam.index, len(jpeg)
                        )
                        continue
                    if getattr(channel, "bufferedAmount", 0) > 4 * MAX_VIDEO_FRAME:
                        continue
                    channel.send(encode_video_frame(cam.index, seq, jpeg))
                    seq += 1
                    sent += 1
                    if sent in (1, 50) or sent % 250 == 0:
                        log.info("camera %d: %d frames sent", cam.index, sent)
            except asyncio.CancelledError:
                raise
            except Exception as e:
                log.warning("camera %d stream error: %s (retrying in 0.5s)", cam.index, e)
                await asyncio.sleep(0.5)

    try:
        # A robot bridge should sit there waiting to be driven, not exit because
        # nobody happened to be connected yet — or because a stale browser
        # session was still holding the room slot when it first tried.
        while True:
            peer = Peer(cfg.signal_url, cfg.room, cfg.token)
            peer.on_channel_open(CH_RPC, on_rpc_open)
            peer.on_channel_open(CH_STREAM, on_stream_open)
            peer.on_channel_open(CH_VIDEO, on_video_open)
            log.info("joining room %s at %s", cfg.room, cfg.signal_url)
            try:
                await peer.connect()
            except Exception as e:
                log.warning("pairing failed (%s); retrying in 5s", e)
                await peer.close()
                # Wheels must not keep their last commanded speed across a
                # dropped session.
                await bus.stop_all(cfg.wheel_ids)
                await asyncio.sleep(5)
                continue

            log.info("peer connected — bridge is live")
            await peer.wait_closed()
            log.info("peer disconnected; waiting for a new session")
            for t in cam_tasks:
                t.cancel()
            if cam_tasks:
                await asyncio.gather(*cam_tasks, return_exceptions=True)
            cam_tasks.clear()
            await bus.stop_all(cfg.wheel_ids)
            await peer.close()
    finally:
        watchdog.stop()
        # Stop the capture loops before closing the devices they're reading, or
        # their deferred generator cleanup runs ioctls on a closed fd.
        for t in cam_tasks:
            t.cancel()
        if cam_tasks:
            await asyncio.gather(*cam_tasks, return_exceptions=True)
        await bus.stop_all(cfg.wheel_ids)
        await bus.write_torque_enable(cfg.arm_ids, False)
        for cam in cameras:
            cam.close()
        if peer is not None:
            await peer.close()
        await bus.disconnect()


def run() -> None:
    try:
        asyncio.run(run_async())
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    run()
