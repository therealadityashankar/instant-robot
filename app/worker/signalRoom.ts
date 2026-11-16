// One Durable Object per room code. Its only job is WebRTC signalling: relay an
// SDP offer/answer and ICE candidates between exactly two peers (browser, robot),
// then get out of the way — media flows peer-to-peer once ICE completes, so this
// object does no further work and costs nothing while the session runs.
//
// Uses the WebSocket Hibernation API (`acceptWebSocket`, not a held `addEventListener`
// loop) so an idle-but-connected room doesn't pin the DO in memory between messages.
// Role (browser vs robot) is stored as a hibernation tag, since hibernation drops
// everything that isn't storage or tags across an eviction.

import type { Role, SignalMessage } from '../src/lib/remoteProtocol';
import { isRelayable } from '../src/lib/remoteProtocol';

/** Signalling-only lifetime cap. Doesn't touch an already-established peer connection —
 * only closes this DO's sockets, so a session that's past the handshake is unaffected. */
const MAX_SIGNAL_LIFETIME_MS = 30 * 60 * 1000;

export class SignalRoom {
  private state: DurableObjectState;
  private env: { BUDGET: DurableObjectNamespace };
  private roles = new WeakMap<WebSocket, Role>();

  constructor(state: DurableObjectState, env: { BUDGET: DurableObjectNamespace }) {
    this.state = state;
    this.env = env;
  }

  private budget(): DurableObjectStub {
    return this.env.BUDGET.get(this.env.BUDGET.idFromName('global'));
  }

  // Whether this room currently holds a budget slot. MUST live in storage, not
  // on the instance: hibernation and eviction throw away instance fields, and a
  // forgotten `true` here means the slot is never released — the counter drifts
  // up until every future connection is refused.
  private async isOpened(): Promise<boolean> {
    return (await this.state.storage.get<boolean>('opened')) ?? false;
  }

  private setOpened(v: boolean): Promise<void> {
    return this.state.storage.put('opened', v);
  }

  /** The room name, stashed on first connect so teardown can name its slot. */
  private async roomName(): Promise<string> {
    return (await this.state.storage.get<string>('room')) ?? '';
  }

  private getRole(ws: WebSocket): Role | null {
    const mem = this.roles.get(ws);
    if (mem) return mem;
    try {
      const att = ws.deserializeAttachment() as WsAttachment | null;
      if (att?.role === 'robot' || att?.role === 'browser') {
        this.roles.set(ws, att.role);
        return att.role;
      }
    } catch {
      /* ignore */
    }
    const tags = this.state.getTags(ws);
    if (tags && tags.includes('robot')) {
      this.roles.set(ws, 'robot');
      return 'robot';
    }
    if (tags && tags.includes('browser')) {
      this.roles.set(ws, 'browser');
      return 'browser';
    }
    return null;
  }

  private peers(): { ws: WebSocket; role: Role }[] {
    const out: { ws: WebSocket; role: Role }[] = [];
    for (const ws of this.state.getWebSockets()) {
      if (ws.readyState === 1) {
        const r = this.getRole(ws);
        if (r) out.push({ ws, role: r });
      }
    }
    return out;
  }

  async fetch(request: Request): Promise<Response> {
    const url = new URL(request.url);
    const role: Role = url.searchParams.get('role') === 'robot' ? 'robot' : 'browser';
    const otherRole: Role = role === 'robot' ? 'browser' : 'robot';

    // Gracefully replace any stale socket of the same role (e.g. browser refresh or robot bridge reconnect)
    for (const ws of this.state.getWebSockets()) {
      if (ws.readyState === 1 && this.getRole(ws) === role) {
        try {
          ws.close(1000, `replaced by new ${role} connection`);
        } catch {
          /* ignore */
        }
      }
    }

    const otherPeer = this.state
      .getWebSockets()
      .find((ws) => ws.readyState === 1 && this.getRole(ws) === otherRole);

    // Only the very first socket into a fresh room draws down the daily/concurrent
    // budget — the second peer joining an already-open room is free.
    const room = url.pathname.startsWith('/signal/')
      ? url.pathname.slice('/signal/'.length)
      : (url.searchParams.get('room') ?? '');
    if (!otherPeer) {
      const res = await this.budget().fetch(
        `http://do/?action=try-open&room=${encodeURIComponent(room)}`,
      );
      const body = (await res.json()) as { ok: boolean; reason?: string };
      if (!body.ok) {
        return new Response(body.reason ?? 'budget exhausted', { status: 429 });
      }
      await this.state.storage.put('room', room);
      await this.setOpened(true);
      // First socket to actually land also arms the lifetime alarm.
      const existingAlarm = await this.state.storage.getAlarm();
      if (!existingAlarm) await this.state.storage.setAlarm(Date.now() + MAX_SIGNAL_LIFETIME_MS);
    }

    const pair = new WebSocketPair();
    const [client, server] = Object.values(pair) as [WebSocket, WebSocket];
    this.roles.set(server, role);
    this.state.acceptWebSocket(server, [role]);
    try {
      server.serializeAttachment({ role } satisfies WsAttachment);
    } catch {
      /* ignore */
    }

    if (otherPeer) {
      this.send(server, { type: 'ready', role, peerPresent: true });
      this.send(otherPeer, { type: 'peer-joined', role });
    } else {
      this.send(server, { type: 'ready', role, peerPresent: false });
    }

    return new Response(null, { status: 101, webSocket: client });
  }

  private send(ws: WebSocket, m: SignalMessage) {
    try {
      ws.send(JSON.stringify(m));
    } catch {
      // socket already gone; the close/error handler will clean it up
    }
  }

  async webSocketMessage(ws: WebSocket, message: string | ArrayBuffer) {
    if (typeof message !== 'string') return;
    let m: SignalMessage;
    try {
      m = JSON.parse(message);
    } catch {
      return;
    }
    if (m.type === 'ping') {
      this.send(ws, { type: 'pong' });
      return;
    }
    if (m.type === 'pong') {
      return;
    }
    if (!isRelayable(m)) return; // only offer/answer/ice are ever client-sent

    const other = this.state.getWebSockets().find((otherWs) => otherWs !== ws && otherWs.readyState === 1);
    if (other) this.send(other, m);
  }

  async webSocketClose(ws: WebSocket) {
    await this.teardown(ws);
  }

  async webSocketError(ws: WebSocket) {
    await this.teardown(ws);
  }

  private async teardown(ws: WebSocket) {
    const role = this.getRole(ws);
    const remaining = this.state.getWebSockets().filter((p) => p !== ws && p.readyState === 1);
    for (const p of remaining) this.send(p, { type: 'peer-left', role: role ?? 'browser' });
    if (remaining.length === 0 && (await this.isOpened())) {
      await this.setOpened(false);
      const room = await this.roomName();
      await this.budget().fetch(`http://do/?action=release&room=${encodeURIComponent(room)}`);
    }
  }

  private async releaseSlot() {
    await this.setOpened(false);
    const room = await this.roomName();
    await this.budget().fetch(`http://do/?action=close&room=${encodeURIComponent(room)}`);
  }

  /** Fires MAX_SIGNAL_LIFETIME_MS after the room opened: signalling window is over. */
  async alarm() {
    for (const { ws } of this.peers()) {
      this.send(ws, { type: 'error', reason: 'signalling window expired', fatal: true });
      ws.close(1000, 'signalling window expired');
    }
    if (await this.isOpened()) await this.releaseSlot();
  }
}
