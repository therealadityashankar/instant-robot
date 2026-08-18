// Entry point for instant-robot's Worker. Two jobs, kept deliberately separate:
//
//  - Serve the built app (unchanged from the assets-only Worker this replaces).
//  - Broker WebRTC signalling for "remote connection" mode: GET /signal/<room>
//    upgrades to a WebSocket and hands off to a per-room SignalRoom Durable
//    Object. Everything else — reading/writing servos, video, the actual
//    control loop — happens peer-to-peer once ICE completes; this file and the
//    DOs it owns never see that traffic.
//
// Auth is a single shared token (Worker secret LEKIWI_TOKEN), checked here
// before a room is ever touched, so an unauthenticated prober costs one fetch
// and nothing else — no DO invoked, no budget spent.

import { SignalRoom } from './signalRoom';
import { Budget } from './budget';

export { SignalRoom, Budget };

export interface Env {
  ASSETS: Fetcher;
  SIGNAL_ROOM: DurableObjectNamespace;
  BUDGET: DurableObjectNamespace;
  /** Shared secret both the browser and the Pi must present to open a room. */
  LEKIWI_TOKEN: string;
}

function timingSafeEqual(a: string, b: string): boolean {
  if (a.length !== b.length) return false;
  let diff = 0;
  for (let i = 0; i < a.length; i++) diff |= a.charCodeAt(i) ^ b.charCodeAt(i);
  return diff === 0;
}

export default {
  async fetch(request: Request, env: Env): Promise<Response> {
    const url = new URL(request.url);

    if (url.pathname.startsWith('/signal/')) {
      if (request.headers.get('Upgrade') !== 'websocket') {
        return new Response('expected websocket', { status: 426 });
      }
      const token = url.searchParams.get('token') ?? '';
      if (!env.LEKIWI_TOKEN || !timingSafeEqual(token, env.LEKIWI_TOKEN)) {
        return new Response('bad token', { status: 401 });
      }
      const room = url.pathname.slice('/signal/'.length);
      if (!room || room.length > 64) {
        return new Response('bad room', { status: 400 });
      }
      const stub = env.SIGNAL_ROOM.get(env.SIGNAL_ROOM.idFromName(room));
      return stub.fetch(request);
    }

    // `?reset=1` clears stuck concurrency slots without touching the daily count.
    if (url.pathname === '/budget-status') {
      const token = url.searchParams.get('token') ?? '';
      if (!env.LEKIWI_TOKEN || !timingSafeEqual(token, env.LEKIWI_TOKEN)) {
        return new Response('bad token', { status: 401 });
      }
      const stub = env.BUDGET.get(env.BUDGET.idFromName('global'));
      const action = url.searchParams.get('reset') === '1' ? 'reset' : 'status';
      return stub.fetch(`http://do/?action=${action}`);
    }

    return env.ASSETS.fetch(request);
  },
};
