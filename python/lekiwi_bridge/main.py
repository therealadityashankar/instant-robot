"""Entrypoint: wires the servo bus, cameras and WebRTC peer together and runs
until interrupted. `lekiwi-bridge` on the path after `pip install -e .`.
"""

from __future__ import annotations

import asyncio
import collections
import json
import logging
import pathlib
import subprocess
import time

from .bus import ServoBus
from .camera import Camera, default_camera_paths
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
    def __init__(self, fps: int = 5):
        self.fps = fps
        self.cameras: list[Camera] = []
        self.active_index = 0
        self.virtual_index = 0
        self.changed = asyncio.Event()
        self.rescan()

    def rescan(self) -> list[Camera]:
        paths = default_camera_paths()
        existing_paths = [c.path for c in self.cameras]
        if paths != existing_paths or not self.cameras:
            log.info("rescan found cameras: %s (was %s)", paths, existing_paths)
            for c in self.cameras:
                if c.path not in paths:
                    c.close()
            self.cameras = [Camera(path, i, fps=self.fps) for i, path in enumerate(paths)]
            if self.active_index >= len(self.cameras):
                self.active_index = max(0, len(self.cameras) - 1)
        return self.cameras

    def set_active(self, index: int) -> int:
        self.rescan()
        self.virtual_index = index
        if len(self.cameras) > 1:
            target_idx = min(index, len(self.cameras) - 1)
            if target_idx != self.active_index:
                if self.active_index < len(self.cameras):
                    self.cameras[self.active_index].close()
                self.active_index = target_idx
                self.changed.set()
                log.info("active camera switched to hardware %d (virtual %d)", target_idx, index)
        elif len(self.cameras) == 1:
            self.active_index = 0
            self.changed.set()
            log.info("camera feed routed to virtual camera %d using physical camera %s", index, self.cameras[0].path)
        else:
            log.warning("no camera hardware found on rescan")
        return self.virtual_index


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
            return rpc_ok(rid, {"version": PROTOCOL_VERSION, "servos": servos, "cameras": max(1, len(cam_mgr.cameras))})
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
            return rpc_ok(rid, {"activeCamera": idx, "hardwareCameras": len(cam_mgr.cameras)})
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

    cam_mgr = CameraManager(fps=cfg.camera_fps)
    for i, c in enumerate(cam_mgr.cameras):
        log.info("camera %d probed ok: %s", i, c.path)

    peer: Peer | None = None

    def on_rpc_open(channel):
        @channel.on("message")
        async def on_msg(raw):
            try:
                msg = json.loads(raw)
            except Exception as e:
                log.warning("bad rpc message: %s", e)
                return
            reply = await handle_rpc(bus, cfg, servos, cam_mgr, msg)
            channel.send(json.dumps(reply))

    def on_stream_open(channel):
        @channel.on("message")
        def on_msg(raw):
            try:
                msg = json.loads(raw)
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
        log.info("video channel open, streaming active camera (%d hardware cameras)", len(cam_mgr.cameras))
        cam_tasks.append(asyncio.ensure_future(_stream_camera(channel, cam_mgr)))

    async def _stream_camera(channel, cam_mgr: CameraManager):
        seq = 0
        sent = 0
        while channel.readyState == "open":
            if not cam_mgr.cameras:
                cam_mgr.rescan()
                if not cam_mgr.cameras:
                    await asyncio.sleep(0.5)
                    continue
            idx = cam_mgr.active_index
            if not (0 <= idx < len(cam_mgr.cameras)):
                idx = 0
            cam = cam_mgr.cameras[idx]
            cam_mgr.changed.clear()
            virt_idx = getattr(cam_mgr, "virtual_index", idx)
            try:
                async for jpeg in cam.frames():
                    if channel.readyState != "open" or cam_mgr.changed.is_set():
                        break

                    if len(jpeg) > MAX_VIDEO_FRAME:
                        log.warning(
                            "camera %d: frame too large (%d bytes > %d), dropping", virt_idx, len(jpeg), MAX_VIDEO_FRAME
                        )
                        continue

                    # Backpressure: never let the SCTP send queue swell
                    buf_amt = getattr(channel, "bufferedAmount", 0)
                    if buf_amt > 65536:
                        log.debug("video channel backpressure (buffered %d B), dropping frame cam=%d", buf_amt, virt_idx)
                        continue

                    try:
                        channel.send(encode_video_frame(virt_idx, seq, jpeg))
                        seq += 1
                        sent += 1
                        if sent == 1 or sent % 10 == 0:
                            log.info("Tx frame: cam=%d (hw=%d) seq=%d size=%d B (buffered=%d)", virt_idx, idx, seq - 1, len(jpeg), buf_amt)
                    except Exception as e:
                        log.error("channel.send failed on cam %d seq %d: %s", virt_idx, seq, e)
            except asyncio.CancelledError:
                raise
            except Exception as e:
                log.warning("camera %d stream error: %s (retrying in 0.5s)", idx, e)
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
