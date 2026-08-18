"""All configuration is environment variables — see the README table. Nothing
device-specific is hardcoded, so the same install works on any LeKiwi."""

from __future__ import annotations

import os
from dataclasses import dataclass, field

from .camera import default_camera_paths


def _ids(env_val: str | None, default: list[int]) -> list[int]:
    if not env_val:
        return default
    return [int(x) for x in env_val.split(",") if x.strip()]


@dataclass
class Config:
    signal_url: str
    room: str
    token: str
    serial_port: str = "/dev/ttyACM0"
    baud: int = 1_000_000
    arm_ids: list[int] = field(default_factory=lambda: [1, 2, 3, 4, 5, 6])
    wheel_ids: list[int] = field(default_factory=lambda: [7, 8, 9])
    camera_paths: list[str] = field(default_factory=list)
    watchdog_s: float = 0.2

    @staticmethod
    def from_env() -> "Config":
        signal_url = os.environ.get("LEKIWI_SIGNAL_URL")
        room = os.environ.get("LEKIWI_ROOM")
        token = os.environ.get("LEKIWI_TOKEN")
        if not signal_url or not room or not token:
            raise SystemExit(
                "LEKIWI_SIGNAL_URL, LEKIWI_ROOM and LEKIWI_TOKEN are all required "
                "(see python/README.md)"
            )
        cam_env = os.environ.get("LEKIWI_CAMERAS")
        cameras = [c.strip() for c in cam_env.split(",") if c.strip()] if cam_env else default_camera_paths()
        return Config(
            signal_url=signal_url,
            room=room,
            token=token,
            serial_port=os.environ.get("LEKIWI_SERIAL_PORT", "/dev/ttyACM0"),
            baud=int(os.environ.get("LEKIWI_BAUD", "1000000")),
            arm_ids=_ids(os.environ.get("LEKIWI_ARM_IDS"), [1, 2, 3, 4, 5, 6]),
            wheel_ids=_ids(os.environ.get("LEKIWI_WHEEL_IDS"), [7, 8, 9]),
            camera_paths=cameras,
            watchdog_s=int(os.environ.get("LEKIWI_WATCHDOG_MS", "200")) / 1000,
        )
