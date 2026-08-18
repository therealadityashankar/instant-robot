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

    def on_channel_open(self, name: str, cb) -> None:
        """Register a callback for when a named datachannel opens (post-connect)."""
        self._on_open = getattr(self, "_on_open", {})
        self._on_open[name] = cb

    async def connect(self, timeout_s: float = 30.0) -> None:
        url = f"{self.signal_url}/signal/{self.room}?role=robot&token={self.token}"
        self._ws = await websockets.connect(
            url,
            open_timeout=timeout_s,
            ping_interval=20.0,
            ping_timeout=20.0,
        )

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
        async def on_state():
            log.info("peer connection state: %s", self.pc.connectionState)

        recv_task = asyncio.create_task(self._recv_loop())

        # Two separate waits, because they mean different things. Waiting for a
        # browser to show up is open-ended — a robot sits idle for hours and that
        # is not an error. Timing out there would drop and re-open the room every
        # 30s, and since each re-open draws down the daily budget, an idle robot
        # would exhaust the cap in about an hour. Only once an offer arrives is
        # there a handshake that can legitimately be called stuck.
        offer = asyncio.ensure_future(self._offer_seen.wait())
        done, _ = await asyncio.wait({offer, recv_task}, return_when=asyncio.FIRST_COMPLETED)
        if offer not in done:
            offer.cancel()
            err = recv_task.exception() if recv_task.done() else None
            raise RuntimeError(f"signalling ended before any offer arrived: {err}")

        try:
            await asyncio.wait_for(
                asyncio.gather(
                    *(e.wait() for e in (self._channel_ready[CH_RPC], self._channel_ready[CH_STREAM]))
                ),
                timeout=timeout_s,
            )
        except asyncio.TimeoutError:
            # A dead _recv_loop is the usual cause; surface it rather than the
            # bare timeout, which says nothing about why.
            if recv_task.done() and recv_task.exception():
                raise RuntimeError(f"signalling failed: {recv_task.exception()}") from recv_task.exception()
            raise RuntimeError(
                f"offer received but no datachannel opened within {timeout_s:.0f}s "
                f"(ICE state={self.pc.iceConnectionState}, connection={self.pc.connectionState}). "
                "Usually means ICE could not establish a peer-to-peer path."
            ) from None
        finally:
            if recv_task.done() and recv_task.exception():
                log.error("signalling loop died: %s", recv_task.exception())

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
                    answer = await self.pc.createAnswer()
                    await self.pc.setLocalDescription(answer)
                    await self._ws.send(
                        json.dumps({"type": "answer", "sdp": self.pc.localDescription.sdp})
                    )
                    log.info("answer sent")
                elif t == "ice":
                    cand = m.get("candidate")
                    if cand:
                        parsed = _parse_ice(cand)
                        if parsed is not None:
                            await self.pc.addIceCandidate(parsed)
                elif t == "error":
                    log.error("signalling error: %s", m.get("reason"))
                elif t in ("ready", "peer-joined", "peer-left"):
                    log.info("signalling: %s", m)
        except Exception:
            # Without this the traceback vanishes into the task and all the
            # caller sees is a timeout with no cause.
            log.exception("signalling loop failed")
            raise

    async def wait_closed(self) -> None:
        """Block until the peer connection drops, so the caller can re-pair."""
        done = asyncio.Event()

        @self.pc.on("connectionstatechange")
        async def _watch():
            if self.pc.connectionState in ("failed", "closed", "disconnected"):
                done.set()

        # Catch a connection that died between connect() returning and this
        # handler being attached.
        if self.pc.connectionState in ("failed", "closed", "disconnected"):
            return
        await done.wait()

    async def close(self) -> None:
        if self._ws:
            try:
                await self._ws.close()
            except Exception:
                pass
        try:
            await self.pc.close()
        except Exception:
            pass
