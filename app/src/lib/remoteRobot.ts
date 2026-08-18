// WebRTC counterpart to `Robot` (robot.ts) — same RobotLike surface, but the
// servo bus lives on a Pi somewhere else, reached through a Cloudflare Durable
// Object for signalling and then peer-to-peer for everything that matters.
//
// Three datachannels (see remoteProtocol.ts for why): `rpc` for request/reply
// calls that need an answer, `stream` for the per-frame position/wheel writes
// that supersede each other anyway, `video` for camera frames. Connect waits
// for `rpc` and `stream` to open — `video` is best-effort and its absence
// shouldn't block control.

import {
  CH_RPC,
  CH_STREAM,
  CH_VIDEO,
  PROTOCOL_VERSION,
  decodeVideoFrame,
  type HelloResult,
  type RpcOp,
  type RpcReply,
  type SignalMessage,
  type StreamMessage,
} from './remoteProtocol';

const ICE_SERVERS: RTCIceServer[] = [
  { urls: ['stun:stun.cloudflare.com:3478', 'stun:stun.l.google.com:19302'] },
];

const RPC_TIMEOUT_MS = 5000;
const CONNECT_TIMEOUT_MS = 15000;

export interface RemoteRobotOptions {
  /** e.g. "wss://instant.river.berlin" — same origin as the app in production. */
  signalOrigin: string;
  room: string;
  token: string;
}

export class RemoteRobot {
  connected = false;
  /** Set once `hello` comes back; lets the UI show what the Pi actually found. */
  info: HelloResult | null = null;
  /** Latest JPEG per camera index, if the caller wants to paint it somewhere. */
  onVideoFrame: ((camera: number, jpeg: Uint8Array) => void) | null = null;

  private opts: RemoteRobotOptions;
  private ws: WebSocket | null = null;
  private pc: RTCPeerConnection | null = null;
  private rpcChan: RTCDataChannel | null = null;
  private streamChan: RTCDataChannel | null = null;
  private videoChan: RTCDataChannel | null = null;
  private nextRpcId = 1;
  private nextStreamSeq = 0;
  private pending = new Map<
    number,
    { resolve: (v: unknown) => void; reject: (e: Error) => void; timer: ReturnType<typeof setTimeout> }
  >();

  constructor(opts: RemoteRobotOptions) {
    this.opts = opts;
  }

  async connect(): Promise<void> {
    try {
      await this.attempt();
    } catch (e) {
      // Critical: without this a failed attempt leaves its WebSocket in the
      // signalling room and its dead RTCPeerConnection behind. The room slot
      // stays occupied, so the next attempt (or the robot rejoining) pairs with
      // a corpse — offer/answer completes, ICE checks pass, and no datachannel
      // ever opens.
      await this.disconnect();
      throw e;
    }
  }

  private async attempt(): Promise<void> {
    const pc = new RTCPeerConnection({ iceServers: ICE_SERVERS });
    this.pc = pc;

    const rpcChan = pc.createDataChannel(CH_RPC, { ordered: true });
    const streamChan = pc.createDataChannel(CH_STREAM, { ordered: false, maxRetransmits: 0 });
    const videoChan = pc.createDataChannel(CH_VIDEO, { ordered: false, maxRetransmits: 0 });
    this.rpcChan = rpcChan;
    this.streamChan = streamChan;
    this.videoChan = videoChan;
    rpcChan.onmessage = (e) => this.handleRpcMessage(e.data);
    videoChan.binaryType = 'arraybuffer';
    videoChan.onmessage = (e) => this.handleVideoMessage(e.data);

    const wsUrl = `${this.opts.signalOrigin}/signal/${encodeURIComponent(this.opts.room)}?role=browser&token=${encodeURIComponent(this.opts.token)}`;
    const ws = new WebSocket(wsUrl);
    this.ws = ws;

    const iceQueue: RTCIceCandidateInit[] = [];
    pc.onicecandidate = (e) => {
      if (!e.candidate) return;
      const msg: SignalMessage = { type: 'ice', candidate: e.candidate.toJSON() };
      if (ws.readyState === WebSocket.OPEN) ws.send(JSON.stringify(msg));
      else iceQueue.push(e.candidate.toJSON());
    };

    await new Promise<void>((resolve, reject) => {
      const timeout = setTimeout(
        () => reject(new Error('timed out connecting to the robot (room not joined in time)')),
        CONNECT_TIMEOUT_MS,
      );
      const fail = (msg: string) => {
        clearTimeout(timeout);
        reject(new Error(msg));
      };

      ws.onerror = () => fail('signalling connection failed');
      ws.onclose = (e) => {
        if (!this.connected) fail(e.reason || 'signalling connection closed before pairing');
      };

      ws.onopen = () => {
        for (const c of iceQueue.splice(0)) ws.send(JSON.stringify({ type: 'ice', candidate: c }));
      };

      ws.onmessage = async (e) => {
        let m: SignalMessage;
        try {
          m = JSON.parse(e.data);
        } catch {
          return;
        }
        try {
          if (m.type === 'error') {
            fail(m.reason);
          } else if (m.type === 'ready') {
            if (m.peerPresent) await this.makeOffer(pc, ws);
          } else if (m.type === 'peer-joined') {
            await this.makeOffer(pc, ws);
          } else if (m.type === 'answer') {
            await pc.setRemoteDescription({ type: 'answer', sdp: m.sdp });
          } else if (m.type === 'ice') {
            await pc.addIceCandidate(m.candidate as RTCIceCandidateInit);
          }
        } catch (err) {
          fail(err instanceof Error ? err.message : String(err));
        }
      };

      // Resolve once both channels a caller actually needs are open. `video`
      // is nice-to-have and never gates connect().
      let rpcOpen = false;
      let streamOpen = false;
      const maybeDone = () => {
        if (rpcOpen && streamOpen) {
          clearTimeout(timeout);
          resolve();
        }
      };
      rpcChan.onopen = () => {
        rpcOpen = true;
        maybeDone();
      };
      streamChan.onopen = () => {
        streamOpen = true;
        maybeDone();
      };
    });

    this.connected = true;
    this.info = await this.rpc<HelloResult>('hello');
    if (this.info.version !== PROTOCOL_VERSION) {
      // Not fatal — best-effort forward compatibility — but worth surfacing.
      console.warn(
        `robot protocol mismatch: browser=${PROTOCOL_VERSION} pi=${this.info.version}`,
      );
    }
  }

  private async makeOffer(pc: RTCPeerConnection, ws: WebSocket) {
    const offer = await pc.createOffer();
    await pc.setLocalDescription(offer);
    ws.send(JSON.stringify({ type: 'offer', sdp: offer.sdp } satisfies SignalMessage));
  }

  private handleVideoMessage(data: ArrayBuffer) {
    const frame = decodeVideoFrame(data);
    if (frame) this.onVideoFrame?.(frame.camera, frame.jpeg);
  }

  private handleRpcMessage(data: string) {
    let m: RpcReply;
    try {
      m = JSON.parse(data);
    } catch {
      return;
    }
    const p = this.pending.get(m.id);
    if (!p) return;
    this.pending.delete(m.id);
    clearTimeout(p.timer);
    if (m.ok) p.resolve(m.result);
    else p.reject(new Error(m.error ?? 'robot rpc failed'));
  }

  private rpc<T>(op: RpcOp, args?: unknown): Promise<T> {
    const chan = this.rpcChan;
    if (!chan || chan.readyState !== 'open') return Promise.reject(new Error('not connected to robot'));
    const id = this.nextRpcId++;
    return new Promise<T>((resolve, reject) => {
      const timer = setTimeout(() => {
        this.pending.delete(id);
        reject(new Error(`robot rpc "${op}" timed out`));
      }, RPC_TIMEOUT_MS);
      this.pending.set(id, { resolve: resolve as (v: unknown) => void, reject, timer });
      chan.send(JSON.stringify({ id, op, args }));
    });
  }

  private sendStream(m: Omit<StreamMessage, 'seq'>) {
    const chan = this.streamChan;
    if (!chan || chan.readyState !== 'open') return; // best-effort — no queueing, next frame supersedes
    chan.send(JSON.stringify({ ...m, seq: this.nextStreamSeq++ } as StreamMessage));
  }

  async disconnect(): Promise<void> {
    this.connected = false;
    for (const p of this.pending.values()) {
      clearTimeout(p.timer);
      p.reject(new Error('disconnected'));
    }
    this.pending.clear();
    try {
      this.ws?.close();
    } catch {
      /* already closed */
    }
    try {
      this.pc?.close();
    } catch {
      /* already closed */
    }
    this.ws = null;
    this.pc = null;
    this.rpcChan = null;
    this.streamChan = null;
    this.videoChan = null;
    this.info = null;
  }

  readPosition(servoId: number): Promise<number> {
    return this.rpc('readPosition', { servoId });
  }

  async syncReadPositions(servoIds: number[]): Promise<Map<number, number>> {
    const entries = await this.rpc<[number, number][]>('syncReadPositions', { servoIds });
    return new Map(entries);
  }

  setTorque(servoIds: number[], enable: boolean): Promise<void> {
    return this.rpc('writeTorqueEnable', { servoIds, enable });
  }

  setWheelMode(servoIds: number[]): Promise<void> {
    return this.rpc('setWheelMode', { servoIds });
  }

  async syncWritePositions(targets: Map<number, number>): Promise<'success'> {
    this.sendStream({ op: 'pos', targets: [...targets.entries()] });
    return 'success';
  }

  async syncWriteWheelSpeed(speeds: Map<number, number>): Promise<'success'> {
    this.sendStream({ op: 'wheel', speeds: [...speeds.entries()] });
    return 'success';
  }

  async writeWheelSpeed(servoId: number, speed: number): Promise<'success'> {
    this.sendStream({ op: 'wheel', speeds: [[servoId, speed]] });
    return 'success';
  }

  setActiveCamera(camera: number): Promise<{ activeCamera: number }> {
    return this.rpc('setActiveCamera', { camera });
  }
}
