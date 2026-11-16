"""The Pi's half of the WebRTC connection: joins the signalling room as
'robot', waits for the browser's offer, answers, and wires up the three
datachannels defined in protocol.py.

Signalling-only traffic goes through the Cloudflare Durable Object
(websockets library); once ICE completes, the RTCPeerConnection carries
everything else peer-to-peer and the signalling socket is idle until either
side hangs up.
"""

from __future__ import annotations

import asyncio
import json
import logging

import websockets
from aiortc import RTCConfiguration, RTCIceServer, RTCPeerConnection, RTCSessionDescription
from aiortc.sdp import candidate_from_sdp

from .protocol import CH_RPC, CH_STREAM, CH_VIDEO

log = logging.getLogger("lekiwi_bridge.webrtc")

ICE_SERVERS = [
    RTCIceServer(urls=["stun:stun.cloudflare.com:3478", "stun:stun.l.google.com:19302"])
]


def _parse_ice(cand: dict):
    """Browser-trickled ICE dict -> aiortc RTCIceCandidate.

    aiortc won't take the raw `{candidate, sdpMid, sdpMLineIndex}` dict that
    RTCIceCandidate.toJSON() produces in the browser — it wants an already-parsed
    object, and the leading "candidate:" prefix stripped. Getting this wrong
    fails ICE silently: the offer/answer still completes, then no datachannel
    ever opens and the connect times out.
    """
    sdp = (cand.get("candidate") or "").strip()
    if not sdp:
        return None  # end-of-candidates sentinel
    if sdp.startswith("candidate:"):
        sdp = sdp[len("candidate:") :]
    parsed = candidate_from_sdp(sdp)
    parsed.sdpMid = cand.get("sdpMid")
    parsed.sdpMLineIndex = cand.get("sdpMLineIndex")
    return parsed


class Peer:
    """One signalling session. Reconnect by constructing a new Peer."""

    def __init__(self, signal_url: str, room: str, token: str):
        self.signal_url = signal_url.rstrip("/")
        self.room = room
        self.token = token
        self.pc = RTCPeerConnection(RTCConfiguration(iceServers=ICE_SERVERS))
        self.channels: dict[str, object] = {}
        self._ws: websockets.WebSocketClientProtocol | None = None
        self._channel_ready = {name: asyncio.Event() for name in (CH_RPC, CH_STREAM, CH_VIDEO)}
        self._offer_seen = asyncio.Event()
        self._peer_left = asyncio.Event()
        self._pending_ice: list[object] = []
        self._recv_task: asyncio.Task | None = None

    def on_channel_open(self, name: str, cb) -> None:
        """Register a callback for when a named datachannel opens (post-connect)."""
        self._on_open = getattr(self, "_on_open", {})
        self._on_open[name] = cb

    async def connect(self, timeout_s: float = 30.0) -> None:
        url = f"{self.signal_url}/signal/{self.room}?role=robot&token={self.token}"
        log.info("connecting ws to %s", url)
        self._ws = await asyncio.wait_for(
            websockets.connect(
                url,
                open_timeout=min(10.0, timeout_s),
                ping_interval=20.0,
                ping_timeout=20.0,
            ),
            timeout=min(10.0, timeout_s),
        )
        log.info("ws connected ok to %s", url)

        @self.pc.on("datachannel")
        def on_datachannel(channel):
            log.info("datachannel open: %s", channel.label)
            self.channels[channel.label] = channel
            if channel.label in self._channel_ready:
                self._channel_ready[channel.label].set()
            on_open = getattr(self, "_on_open", {}).get(channel.label)
            if on_open:
                on_open(channel)

        # NB: aiortc has no "icecandidate" event — it does not trickle. It gathers
        # every candidate during setLocalDescription and ships them inside the
        # answer SDP, so there is nothing to forward from this side.

        @self.pc.on("connectionstatechange")
        def on_state():
            log.info("peer connection state: %s", self.pc.connectionState)

        self._recv_task = asyncio.create_task(self._recv_loop())
        self._heartbeat_task = asyncio.create_task(self._heartbeat_loop())

        # Two separate waits, because they mean different things. Waiting for a
        # browser to show up is open-ended — a robot sits idle for hours and that
        # is not an error. Timing out there would drop and re-open the room every
        # 30s, and since each re-open draws down the daily budget, an idle robot
        # would exhaust the cap in about an hour. Only once an offer arrives is
        # there a handshake that can legitimately be called stuck.
        offer = asyncio.ensure_future(self._offer_seen.wait())
        done, _ = await asyncio.wait({offer, self._recv_task}, return_when=asyncio.FIRST_COMPLETED)
        if offer not in done:
            offer.cancel()
            err = self._recv_task.exception() if self._recv_task.done() else None
            raise RuntimeError(f"signalling ended before any offer arrived: {err}")

        try:
            await asyncio.wait_for(
                asyncio.gather(
                    *(e.wait() for e in (self._channel_ready[CH_RPC], self._channel_ready[CH_STREAM]))
                ),
                timeout=timeout_s,
            )
        except asyncio.TimeoutError:
            if self._recv_task.done() and self._recv_task.exception():
                raise RuntimeError(f"signalling failed: {self._recv_task.exception()}") from self._recv_task.exception()
            raise RuntimeError(
                f"offer received but no datachannel opened within {timeout_s:.0f}s "
                f"(ICE state={self.pc.iceConnectionState}, connection={self.pc.connectionState}). "
                "Usually means ICE could not establish a peer-to-peer path."
            ) from None
        finally:
            if self._recv_task.done() and self._recv_task.exception():
                log.error("signalling loop died: %s", self._recv_task.exception())

    async def _heartbeat_loop(self) -> None:
        try:
            while self._ws and getattr(self._ws, "state", None) == 1:
                await asyncio.sleep(5.0)
                if self._ws and getattr(self._ws, "state", None) == 1:
                    await self._ws.send(json.dumps({"type": "ping"}))
        except Exception:
            pass

    async def _recv_loop(self) -> None:
        assert self._ws is not None
        try:
            async for raw in self._ws:
                m = json.loads(raw)
                t = m.get("type")
                if t == "offer":
                    log.info("offer received, answering")
                    self._offer_seen.set()
                    await self.pc.setRemoteDescription(RTCSessionDescription(sdp=m["sdp"], type="offer"))
                    for early_cand in self._pending_ice:
                        try:
                            await self.pc.addIceCandidate(early_cand)
                        except Exception as e:
                            log.warning("could not add early ICE candidate: %s", e)
                    self._pending_ice.clear()
                    answer = await self.pc.createAnswer()
                    await self.pc.setLocalDescription(answer)
                    await self._ws.send(
                        json.dumps({"type": "answer", "sdp": self.pc.localDescription.sdp})
                    )
                    log.info("answer sent")
                elif t == "ice":
                    cand = m.get("candidate")
                    if cand:
                        try:
                            parsed = _parse_ice(cand)
                            if parsed is not None:
                                if self.pc.remoteDescription is not None:
                                    await self.pc.addIceCandidate(parsed)
                                else:
                                    self._pending_ice.append(parsed)
                        except Exception as e:
                            log.warning("failed to process ICE candidate: %s", e)
                elif t == "error":
                    log.error("signalling error: %s", m.get("reason"))
                elif t == "peer-left":
                    log.info("signalling: peer-left (role=%s)", m.get("role"))
                    self._peer_left.set()
                elif t in ("ready", "peer-joined", "pong"):
                    if t != "pong":
                        log.info("signalling: %s", m)
        except Exception:
            log.exception("signalling loop failed")
            raise

    async def wait_closed(self) -> None:
        """Block until the peer connection drops, signaling drops, or peer leaves."""
        done = asyncio.Event()

        @self.pc.on("connectionstatechange")
        def _watch_pc():
            if self.pc.connectionState in ("failed", "closed", "disconnected"):
                done.set()

        @self.pc.on("iceconnectionstatechange")
        def _watch_ice():
            if self.pc.iceConnectionState in ("failed", "closed", "disconnected"):
                done.set()

        if self.pc.connectionState in ("failed", "closed", "disconnected"):
            return

        async def _watch_events():
            peer_left_fut = asyncio.ensure_future(self._peer_left.wait())
            recv_task_fut = self._recv_task if self._recv_task else asyncio.sleep(86400)
            d, _ = await asyncio.wait(
                {peer_left_fut, recv_task_fut},
                return_when=asyncio.FIRST_COMPLETED,
            )
            peer_left_fut.cancel()
            done.set()

        watcher = asyncio.create_task(_watch_events())
        try:
            await done.wait()
        finally:
            watcher.cancel()

    async def close(self) -> None:
        if hasattr(self, "_recv_task") and self._recv_task and not self._recv_task.done():
            self._recv_task.cancel()
        if hasattr(self, "_heartbeat_task") and self._heartbeat_task and not self._heartbeat_task.done():
            self._heartbeat_task.cancel()
        if self._ws:
            try:
                await self._ws.close()
            except Exception:
                pass
            self._ws = None
        try:
            await self.pc.close()
        except Exception:
            pass
