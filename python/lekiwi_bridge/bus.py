"""Feetech STS3215 servo bus, over the generic (Dynamixel-style) scservo_sdk fork.

Verified against the actual hardware during setup: 9x STS3215 (model 777), IDs
1-6 arm / 7-9 wheels, 1,000,000 baud, half-duplex TTL on /dev/ttyACM0 (a CH340
adapter — note ACM, not USB). This fork has no `sms_sts` convenience class, so
every call goes through PortHandler + PacketHandler directly, addressing
registers by number:

    40  torque enable        (1 byte)
    33  mode (0=servo,1=wheel) (1 byte) -- not in the original handoff notes;
                                           confirm against your firmware before
                                           trusting wheel mode blindly
    42  goal position         (2 bytes)
    46  goal speed            (2 bytes, sign-bit packed: bit15 = direction)
    56  present position      (2 bytes)
    58  present speed         (2 bytes)

Mirrors the shape of app/src/lib/robot.ts on the browser side: one queue
serializes every bus access, because the arm's position stream and the wheels'
speed stream would otherwise both try to own the port each control tick.
"""

import asyncio
import glob
import logging
import os

from scservo_sdk import PacketHandler, PortHandler

log = logging.getLogger("lekiwi_bridge.bus")

ADDR_MODE = 33
ADDR_TORQUE_ENABLE = 40
ADDR_GOAL_POSITION = 42
ADDR_GOAL_SPEED = 46
ADDR_PRESENT_POSITION = 56
ADDR_PRESENT_SPEED = 58

MODE_SERVO = 0
MODE_WHEEL = 1


class BusError(RuntimeError):
    pass


def find_serial_ports(preferred: str | None = None) -> list[str]:
    """Return all plausible serial port paths for the Feetech STS adapter in priority order."""
    candidates: list[str] = []
    if preferred and os.path.exists(preferred):
        candidates.append(preferred)
    # QinHeng CH340/CH343 / STS adapters identified by USB VID 1a86
    candidates.extend(glob.glob("/dev/serial/by-id/*1a86*"))
    candidates.extend(glob.glob("/dev/serial/by-id/*"))
    candidates.extend(sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*")))
    # Deduplicate preserving order
    seen: set[str] = set()
    out: list[str] = []
    for c in candidates:
        try:
            real = os.path.realpath(c)
        except Exception:
            real = c
        if real not in seen and os.path.exists(c):
            seen.add(real)
            out.append(c)
    return out or ([preferred] if preferred else ["/dev/ttyACM0"])


class ServoBus:
    def __init__(self, port: str = "/dev/ttyACM0", baud: int = 1_000_000):
        self._preferred_port = port
        self._port_name = port
        self._baud = baud
        self._port: PortHandler | None = None
        self._packet = PacketHandler(0)  # 0 = little-endian, correct for STS
        self._lock = asyncio.Lock()
        self.connected = False

    def _sync_connect(self) -> None:
        if self._port:
            try:
                self._port.closePort()
            except Exception:
                pass
            self._port = None
            self.connected = False

        candidates = find_serial_ports(self._preferred_port)
        last_err = None
        for p_name in candidates:
            log.info("trying servo serial port: %s @ %d baud", p_name, self._baud)
            ph = PortHandler(p_name)
            if not ph.openPort():
                last_err = f"could not open {p_name}"
                continue
            if not ph.setBaudRate(self._baud):
                ph.closePort()
                last_err = f"could not set baud {self._baud} on {p_name}"
                continue
            self._port = ph
            self._port_name = p_name
            self.connected = True
            log.info("successfully connected to servo bus on %s", p_name)
            return

        raise BusError(last_err or f"no valid serial port found (tried {candidates})")

    async def connect(self) -> None:
        async with self._lock:
            await asyncio.to_thread(self._sync_connect)

    async def reconnect(self) -> bool:
        """Attempt to re-discover and reopen the serial port."""
        try:
            await self.connect()
            return True
        except Exception as e:
            log.warning("servo bus reconnect failed: %s", e)
            return False

    async def disconnect(self) -> None:
        async with self._lock:
            if self._port:
                try:
                    await asyncio.to_thread(self._port.closePort)
                except Exception:
                    pass
                self._port = None
            self.connected = False

    async def ping(self, servo_id: int) -> int | None:
        """Returns the model number if the servo answers, else None."""
        async with self._lock:
            if not self._port:
                return None
            model, comm, err = await asyncio.to_thread(self._packet.ping, self._port, servo_id)
            if comm != 0 or err != 0:
                return None
            return model

    def _sync_probe(self, ids: range) -> list[int]:
        if not self._port:
            return []
        found = []
        for sid in ids:
            model, comm, err = self._packet.ping(self._port, sid)
            if comm == 0 and err == 0:
                found.append(sid)
        return found

    async def probe(self, ids: range) -> list[int]:
        """Which of `ids` answer a ping — used at startup to report what's on the bus."""
        async with self._lock:
            return await asyncio.to_thread(self._sync_probe, ids)

    def _read2(self, servo_id: int, addr: int) -> int:
        if not self._port:
            raise BusError("bus not connected")
        val, comm, err = self._packet.read2ByteTxRx(self._port, servo_id, addr)
        if comm != 0:
            raise BusError(f"servo {servo_id} read@{addr} comm error {comm}")
        if err != 0:
            raise BusError(f"servo {servo_id} read@{addr} status error {err}")
        return val

    def _write1(self, servo_id: int, addr: int, val: int) -> None:
        if not self._port:
            raise BusError("bus not connected")
        comm, err = self._packet.write1ByteTxRx(self._port, servo_id, addr, val)
        if comm != 0:
            raise BusError(f"servo {servo_id} write@{addr} comm error {comm}")
        if err != 0:
            raise BusError(f"servo {servo_id} write@{addr} status error {err}")

    def _write2(self, servo_id: int, addr: int, val: int) -> None:
        if not self._port:
            raise BusError("bus not connected")
        comm, err = self._packet.write2ByteTxRx(self._port, servo_id, addr, val & 0xFFFF)
        if comm != 0:
            raise BusError(f"servo {servo_id} write@{addr} comm error {comm}")
        if err != 0:
            raise BusError(f"servo {servo_id} write@{addr} status error {err}")

    async def read_position(self, servo_id: int) -> int:
        async with self._lock:
            return self._read2(servo_id, ADDR_PRESENT_POSITION)

    async def sync_read_positions(self, ids: list[int]) -> dict[int, int]:
        # The generic fork's GroupSyncRead is fussier to wire up correctly than
        # this loop is slow to run for a handful of servos (9, here) — read
        # them one at a time under the same lock so it's still one atomic bus
        # operation from every other caller's point of view.
        async with self._lock:
            out: dict[int, int] = {}
            for sid in ids:
                try:
                    out[sid] = self._read2(sid, ADDR_PRESENT_POSITION)
                except BusError as e:
                    log.warning("sync_read_positions: %s", e)
            return out

    async def write_torque_enable(self, ids: list[int], enable: bool) -> None:
        async with self._lock:
            for sid in ids:
                self._write1(sid, ADDR_TORQUE_ENABLE, 1 if enable else 0)

    async def set_wheel_mode(self, ids: list[int]) -> None:
        async with self._lock:
            for sid in ids:
                self._write1(sid, ADDR_MODE, MODE_WHEEL)

    async def sync_write_positions(self, targets: dict[int, int]) -> None:
        async with self._lock:
            for sid, pos in targets.items():
                try:
                    self._write2(sid, ADDR_GOAL_POSITION, pos)
                except BusError as e:
                    log.warning("sync_write_positions: %s", e)

    @staticmethod
    def _pack_speed(speed: int) -> int:
        """Feetech wheel-speed convention: 15-bit magnitude, bit15 = direction."""
        magnitude = min(abs(int(speed)), 0x7FFF)
        return magnitude | (0x8000 if speed < 0 else 0)

    async def sync_write_wheel_speed(self, speeds: dict[int, int]) -> None:
        async with self._lock:
            for sid, speed in speeds.items():
                try:
                    self._write2(sid, ADDR_GOAL_SPEED, self._pack_speed(speed))
                except BusError as e:
                    log.warning("sync_write_wheel_speed: %s", e)

    async def stop_all(self, wheel_ids: list[int]) -> None:
        await self.sync_write_wheel_speed({sid: 0 for sid in wheel_ids})
