"""Camera capture that forwards native MJPEG frames untouched.

Both cameras on this board are UVC devices set to MJPEG mode (mandatory: the
servo adapter shares the same USB 2.0 bus, and YUYV at 640x480 would saturate
it). In MJPEG mode the V4L2 driver already hands back complete JPEG frames —
so capture here is "read a buffer", not "decode a frame and re-encode it",
which is the only reason this is affordable on a Zero 2 W. No OpenCV, no
PyAV frame handling: v4l2py talks V4L2 ioctls directly and gives us bytes.

Verified against v4l2py 3.0.0 on the actual Pi: `Device` has no `video_capture`
attribute; you construct `VideoCapture(device)` yourself, and it only produces
frames inside its context manager (`__enter__` runs open/arm/start — without
that its buffers are None and iterating raises "NoneType is not iterable").

Device paths are the `/dev/v4l/by-path/...` entries because the two cameras are
the same model with no distinguishing serial, so `/dev/video0` vs `/dev/video2`
swap on reboot depending on enumeration order — by-path is stable because it
encodes the physical USB hub port.
"""

from __future__ import annotations

import asyncio
import logging
from collections.abc import AsyncIterator

from v4l2py.device import Device, VideoCapture

log = logging.getLogger("lekiwi_bridge.camera")

DEFAULT_WIDTH = 640
DEFAULT_HEIGHT = 480
DEFAULT_FPS = 5


class Camera:
    def __init__(
        self,
        path: str,
        index: int,
        width: int = DEFAULT_WIDTH,
        height: int = DEFAULT_HEIGHT,
        fps: int = DEFAULT_FPS,
    ):
        self.path = path
        self.index = index
        self._width = width
        self._height = height
        self._fps = fps
        self._device: Device | None = None
        self._capture: VideoCapture | None = None

    def open(self) -> None:
        self.close()
        dev = Device(self.path)
        dev.open()
        cap = VideoCapture(dev)
        cap.set_format(self._width, self._height, "MJPG")
        try:
            cap.set_fps(self._fps)
        except Exception as e:
            log.warning("camera %s: could not set fps (%s), using driver default", self.path, e)
        fmt = cap.get_format()
        log.info(
            "camera %d: %dx%d %s @ %d fps", self.index, fmt.width, fmt.height, fmt.pixel_format, self._fps
        )
        self._device = dev
        self._capture = cap

    def close(self) -> None:
        if self._device is not None:
            try:
                self._device.close()
            except Exception:
                pass
            self._device = None
            self._capture = None

    async def frames(self) -> AsyncIterator[bytes]:
        """Yields raw JPEG bytes, one per captured frame, until cancelled."""
        if self._capture is None:
            self.open()
        cap = self._capture
        if cap is None:
            raise RuntimeError(f"camera {self.index} not open")
        loop = asyncio.get_running_loop()

        # Every one of these is a blocking ioctl/mmap, so each hops to a thread:
        # stalling the event loop here would stall the servo control stream too.
        def _enter():
            cap.__enter__()
            return iter(cap)

        it = await loop.run_in_executor(None, _enter)
        try:
            while True:
                frame = await loop.run_in_executor(None, next, it, None)
                if frame is None:
                    return
                yield bytes(frame)
        finally:
            # Generator cleanup is deferred, so this can land after close() has
            # already torn the device down — in which case stream_off ioctls on a
            # closed fd. Nothing left to release at that point, so it's ignorable.
            try:
                await loop.run_in_executor(None, cap.__exit__, None, None, None)
            except Exception as e:
                log.debug("camera %d: stream teardown after close (%s)", self.index, e)
            self.close()


def default_camera_paths() -> list[str]:
    """Best-effort by-path discovery, ordered by USB port number for stability.

    Falls back to empty (caller then runs with zero cameras) rather than
    guessing at /dev/videoN, which is exactly the ordering this exists to avoid.

    Deduplicated by the /dev/videoN each symlink resolves to: udev publishes more
    than one by-path alias for the same device (both `usb-` and `usbv2-` forms on
    this Pi), so a raw glob reports two cameras as four and then fails opening
    the phantoms. The `usb` filter also drops the onboard mailbox video node.
    """
    import glob
    import os

    seen: set[str] = set()
    out: list[str] = []
    for p in sorted(p for p in glob.glob("/dev/v4l/by-path/*-video-index0") if "usb" in p):
        real = os.path.realpath(p)
        if real in seen:
            continue
        seen.add(real)
        out.append(p)
    return out
