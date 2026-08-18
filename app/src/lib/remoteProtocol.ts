// Wire format shared by the browser and the Pi.
//
// This file is the single source of truth for both ends. The Python side
// (`python/lekiwi_bridge/protocol.py`) mirrors it by hand — if you change a message
// here, change it there too. The shapes are deliberately dull (plain JSON, flat
// fields) so the mirror stays easy to eyeball.
//
// Three datachannels, because the traffic has three different tolerances:
//
//   rpc     ordered + reliable    request/reply. Reads, torque, wheel mode.
//                                 Rare, and a lost one would hang a caller.
//   stream  unordered + lossy     the per-frame joint/wheel writes. These are
//                                 re-sent every animation frame, so a retransmit
//                                 would only ever deliver a stale target late.
//   video   unordered + lossy     JPEG frames. Same reasoning: yesterday's frame
//                                 is worth less than nothing.

export const CH_RPC = 'rpc';
export const CH_STREAM = 'stream';
export const CH_VIDEO = 'video';

/** Bumped when a change would make an old Pi and a new browser misbehave. */
export const PROTOCOL_VERSION = 1;

// ── rpc ──────────────────────────────────────────────────────────────────────

/** Everything the browser can ask the Pi to do and wait on. */
export type RpcOp =
  | 'hello'
  | 'ping'
  | 'readPosition'
  | 'syncReadPositions'
  | 'writeTorqueEnable'
  | 'setWheelMode'
  | 'stopAll'
  | 'setActiveCamera';

export interface RpcRequest {
  /** Correlates reply to request; monotonic per connection. */
  id: number;
  op: RpcOp;
  args?: unknown;
}

export interface RpcReply {
  id: number;
  ok: boolean;
  /** Present when ok. */
  result?: unknown;
  /** Present when !ok — a human-readable reason, already stringified by the Pi. */
  error?: string;
}

export interface HelloResult {
  version: number;
  /** Servo IDs that answered a ping on the bus, so the UI can say what's there. */
  servos: number[];
  /** How many cameras the Pi opened; frames arrive tagged 0..n-1. */
  cameras: number;
}

// ── stream ───────────────────────────────────────────────────────────────────

/**
 * Fire-and-forget writes. No reply, no id — if one is dropped the next frame
 * supersedes it. `seq` exists only so the Pi can discard one that arrives out of
 * order (unordered delivery means that happens routinely).
 */
export type StreamMessage =
  | { op: 'pos'; seq: number; targets: [number, number][] }
  | { op: 'wheel'; seq: number; speeds: [number, number][] };

// ── video ────────────────────────────────────────────────────────────────────

/**
 * Binary, not JSON — base64 would cost a third more bytes for no gain.
 *
 *   byte 0      camera index
 *   bytes 1-4   frame sequence, little-endian uint32
 *   bytes 5+    JPEG payload
 *
 * One frame per message. The Pi drops any frame that would exceed
 * MAX_VIDEO_FRAME rather than chunking: at the resolutions this runs at a
 * frame is ~10-30 kB, and a frame big enough to need splitting means the
 * camera settings are wrong, which is worth noticing rather than papering over.
 */
export const VIDEO_HEADER_BYTES = 5;
export const MAX_VIDEO_FRAME = 60_000;

export function encodeVideoFrame(camera: number, seq: number, jpeg: Uint8Array): ArrayBuffer {
  const out = new Uint8Array(VIDEO_HEADER_BYTES + jpeg.length);
  const view = new DataView(out.buffer);
  out[0] = camera;
  view.setUint32(1, seq >>> 0, true);
  out.set(jpeg, VIDEO_HEADER_BYTES);
  return out.buffer;
}

export function decodeVideoFrame(buf: ArrayBuffer): {
  camera: number;
  seq: number;
  jpeg: Uint8Array;
} | null {
  if (buf.byteLength <= VIDEO_HEADER_BYTES) return null;
  const bytes = new Uint8Array(buf);
  const view = new DataView(buf);
  return {
    camera: bytes[0],
    seq: view.getUint32(1, true),
    jpeg: bytes.subarray(VIDEO_HEADER_BYTES),
  };
}

// ── signalling ───────────────────────────────────────────────────────────────

/** Which end of the connection a WebSocket belongs to. */
export type Role = 'browser' | 'robot';

/**
 * Sent over the Durable Object WebSocket. `offer`/`answer`/`ice` are relayed to
 * the other peer untouched; the rest are the DO talking to one peer directly.
 */
export type SignalMessage =
  | { type: 'offer'; sdp: string }
  | { type: 'answer'; sdp: string }
  | { type: 'ice'; candidate: unknown }
  | { type: 'peer-joined'; role: Role }
  | { type: 'peer-left'; role: Role }
  | { type: 'ready'; role: Role; peerPresent: boolean }
  | { type: 'error'; reason: string; fatal: boolean };

/** Relayed to the peer as-is; anything else is either DO-authored or rejected. */
export function isRelayable(m: SignalMessage): boolean {
  return m.type === 'offer' || m.type === 'answer' || m.type === 'ice';
}
