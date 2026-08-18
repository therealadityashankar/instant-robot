"""Halt wheels (and stop chasing new arm targets) if control packets stop.

A dropped WebRTC datachannel or a browser tab that silently died must not
leave the base rolling. The arm doesn't need active intervention here: once
the `stream` channel goes quiet, no new goal positions arrive, and a torqued
STS3215 already holds its last commanded position on its own — "hold the arm"
falls out of doing nothing, not from a separate action. Wheels are velocity
controlled, so they need the opposite: an explicit zero, because "do nothing"
for a wheel means "keep spinning at the last speed forever".
"""

from __future__ import annotations

import asyncio
import logging
import time

from .bus import ServoBus

log = logging.getLogger("lekiwi_bridge.watchdog")


class Watchdog:
    def __init__(self, bus: ServoBus, wheel_ids: list[int], timeout_s: float):
        self._bus = bus
        self._wheel_ids = wheel_ids
        self._timeout_s = timeout_s
        self._last_packet = time.monotonic()
        self._tripped = False
        self._armed = False
        self._task: asyncio.Task | None = None

    def kick(self) -> None:
        """Call on every stream message received."""
        self._last_packet = time.monotonic()
        self._tripped = False
        self._armed = True

    async def _loop(self) -> None:
        while True:
            await asyncio.sleep(self._timeout_s / 2)
            # Stays disarmed until the first control packet ever arrives —
            # otherwise it trips immediately at startup, before the browser has
            # even connected, and reports a stall that never happened.
            if not self._armed:
                continue
            idle = time.monotonic() - self._last_packet
            if idle > self._timeout_s and not self._tripped:
                self._tripped = True
                log.warning("watchdog: no control packet for %.0fms, stopping wheels", idle * 1000)
                try:
                    await self._bus.stop_all(self._wheel_ids)
                except Exception as e:
                    log.error("watchdog: failed to stop wheels: %s", e)

    def start(self) -> None:
        if self._task is None:
            self._task = asyncio.create_task(self._loop())

    def stop(self) -> None:
        if self._task is not None:
            self._task.cancel()
            self._task = None
