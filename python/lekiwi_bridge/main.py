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
        if op == "reconnectBus":
            ok = await bus.reconnect()
            found = await bus.probe(range(1, 16)) if ok else []
            if ok and found:
                await bus.write_torque_enable(cfg.arm_ids, True)
                if cfg.wheel_ids:
                    await bus.set_wheel_mode(cfg.wheel_ids)
                servos.clear()
                servos.extend(found)
            return rpc_ok(rid, {"ok": ok, "port": getattr(bus, "_port_name", "unknown"), "servos": servos})
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
    servos: list[int] = list(cfg.arm_ids + cfg.wheel_ids)

    async def _init_bus():
        nonlocal servos
        try:
            await bus.connect()
            log.info("bus connected on %s @ %d baud", bus._port_name, cfg.baud)
            found = await bus.probe(range(1, 16))
            if found:
                servos = found
                log.info("servos probed ok: %s", servos)
                await bus.write_torque_enable(cfg.arm_ids, True)
                if cfg.wheel_ids:
                    await bus.set_wheel_mode(cfg.wheel_ids)
        except Exception as e:
            log.warning("background bus init warning: %s", e)

    asyncio.create_task(_init_bus())

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

    def on_video_open(channel):
        log.info("video data channel open (primary streaming via Cloudflare Edge WebSocket)")

    async def _stream_camera_ws(cam_mgr: CameraManager, ws_url: str):
        import websockets
        seqs = [0, 0]
        while True:
            try:
                log.info("connecting video websocket to Cloudflare Edge: %s", ws_url)
                async with websockets.connect(
                    ws_url,
                    max_size=MAX_VIDEO_FRAME * 2,
                    ping_interval=15,
                    ping_timeout=20,
                    close_timeout=5,
                ) as ws:
                    log.info("video websocket connected to Cloudflare Edge!")

                    async def stream_single(cam_idx: int):
                        while True:
                            if cam_idx >= len(cam_mgr.cameras):
                                await asyncio.sleep(0.5)
                                continue
                            cam = cam_mgr.cameras[cam_idx]
                            virt_idx = getattr(cam_mgr, f"virtual_index_{cam_idx}", cam_idx)
                            try:
                                async for jpeg in cam.frames():
                                    if len(jpeg) > MAX_VIDEO_FRAME:
                                        continue
                                    await ws.send(encode_video_frame(virt_idx, seqs[cam_idx], jpeg))
                                    seqs[cam_idx] += 1
                                    if seqs[cam_idx] == 1 or seqs[cam_idx] % 20 == 0:
                                        log.info("Tx WS frame: cam=%d (hw=%d) seq=%d size=%d B", virt_idx, cam_idx, seqs[cam_idx] - 1, len(jpeg))
                            except asyncio.CancelledError:
                                raise
                            except Exception as e:
                                log.warning("camera %d frame stream error: %s (reconnecting)", cam_idx, e)
                                raise

                    tasks = [
                        asyncio.create_task(stream_single(0)),
                        asyncio.create_task(stream_single(1)),
                    ]
                    done, pending = await asyncio.wait(tasks, return_when=asyncio.FIRST_EXCEPTION)
                    for t in pending:
                        t.cancel()
                    for t in done:
                        if t.exception():
                            raise t.exception()
            except asyncio.CancelledError:
                raise
            except Exception as e:
                log.warning("video websocket reconnecting in 2s (%s)...", e)
                await asyncio.sleep(2.0)

    cf_ws_url = f"wss://lekiwi-video-relay.theadityashankar.workers.dev/?room={cfg.room}&role=robot"
    asyncio.ensure_future(_stream_camera_ws(cam_mgr, cf_ws_url))

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
                await asyncio.sleep(1.5)
                continue

            log.info("peer connected — bridge is live")
            await peer.wait_closed()
            log.info("peer disconnected; waiting for a new session")
            await bus.stop_all(cfg.wheel_ids)
            await peer.close()
    finally:
        watchdog.stop()
        await bus.stop_all(cfg.wheel_ids)
        await bus.write_torque_enable(cfg.arm_ids, False)
        for cam in cam_mgr.cameras:
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
