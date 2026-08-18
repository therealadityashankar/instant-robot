"""Wire format shared with the browser.

Hand-mirrors app/src/lib/remoteProtocol.ts — if you change one, change the
other. Kept deliberately dumb (dataclasses over dicts, no framework) so the
mirror stays easy to eyeball against the TypeScript.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Literal

CH_RPC = "rpc"
CH_STREAM = "stream"
CH_VIDEO = "video"

PROTOCOL_VERSION = 1

RpcOp = Literal[
    "hello",
    "ping",
    "readPosition",
    "syncReadPositions",
    "writeTorqueEnable",
    "setWheelMode",
    "stopAll",
    "setActiveCamera",
]


@dataclass
class RpcRequest:
    id: int
    op: RpcOp
    args: dict[str, Any] = field(default_factory=dict)

    @staticmethod
    def from_json(d: dict[str, Any]) -> "RpcRequest":
        return RpcRequest(id=d["id"], op=d["op"], args=d.get("args") or {})


def rpc_ok(id: int, result: Any = None) -> dict[str, Any]:
    return {"id": id, "ok": True, "result": result}


def rpc_err(id: int, error: str) -> dict[str, Any]:
    return {"id": id, "ok": False, "error": error}


# ── stream (unreliable, fire-and-forget) ──────────────────────────────────────

# {"op": "pos", "seq": int, "targets": [[servoId, pos], ...]}
# {"op": "wheel", "seq": int, "speeds": [[servoId, speed], ...]}


# ── video ──────────────────────────────────────────────────────────────────

VIDEO_HEADER_BYTES = 5
MAX_VIDEO_FRAME = 250_000


def encode_video_frame(camera: int, seq: int, jpeg: bytes) -> bytes:
    return bytes([camera & 0xFF]) + (seq & 0xFFFFFFFF).to_bytes(4, "little") + jpeg


# ── signalling ───────────────────────────────────────────────────────────────

Role = Literal["browser", "robot"]
