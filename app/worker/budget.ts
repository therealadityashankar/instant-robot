// Global spend guard for the WebRTC signalling worker.
//
// This is the whole point of running signalling through Cloudflare instead of
// some other free relay: a single choke point that can say no. Rooms cost us
// nothing once WebRTC is up (media is peer-to-peer over STUN, no TURN), so the
// only thing worth capping is Durable Object *churn* — how many rooms open, and
// how many are open at once.
//
// Concurrency is tracked as a map of roomId -> expiry, NOT an incrementing
// counter. A counter is only correct if every increment is matched by a
// decrement, and that assumption fails the moment a DO is evicted mid-session
// or a process dies: the count drifts upward, and since it never recovers, the
// guard eventually refuses everything forever. Expiring entries are
// self-healing — a leaked room frees its slot on its own.
//
// One instance of this class ever runs (idFromName('global')), backed by
// SQLite storage so counts survive a restart.

export const DAILY_ROOM_CAP = 100;
export const CONCURRENT_ROOM_CAP = 2;
/** Slightly longer than SignalRoom's own lifetime cap, so the room normally
 *  releases its slot explicitly and this is only ever the backstop. */
export const ROOM_SLOT_TTL_MS = 35 * 60 * 1000;

interface BudgetState {
  day: string; // YYYY-MM-DD (UTC) the counters below apply to
  opened: number; // rooms created today
  /** roomId -> epoch ms after which the slot is considered abandoned. */
  live: Record<string, number>;
}

function today(): string {
  return new Date().toISOString().slice(0, 10);
}

export class Budget {
  private state: DurableObjectState;

  constructor(state: DurableObjectState) {
    this.state = state;
  }

  private async load(): Promise<BudgetState> {
    const s = (await this.state.storage.get<BudgetState>('s')) ?? {
      day: today(),
      opened: 0,
      live: {},
    };
    // Drop expired slots on every read — that is what makes a leak temporary.
    const now = Date.now();
    const live: Record<string, number> = {};
    for (const [room, expiry] of Object.entries(s.live ?? {})) {
      if (expiry > now) live[room] = expiry;
    }
    if (s.day !== today()) return { day: today(), opened: 0, live };
    return { ...s, live };
  }

  private save(s: BudgetState): Promise<void> {
    return this.state.storage.put('s', s);
  }

  async fetch(request: Request): Promise<Response> {
    const url = new URL(request.url);
    const action = url.searchParams.get('action');
    const room = url.searchParams.get('room') ?? '';
    const s = await this.load();

    if (action === 'try-open') {
      // Re-opening a room that already holds a slot is free — it's the same
      // session reconnecting, not a new one.
      const alreadyHeld = room in s.live;
      if (!alreadyHeld && s.opened >= DAILY_ROOM_CAP) {
        return Response.json({ ok: false, reason: `daily room cap (${DAILY_ROOM_CAP}) reached` });
      }
      if (!alreadyHeld && Object.keys(s.live).length >= CONCURRENT_ROOM_CAP) {
        return Response.json({
          ok: false,
          reason: `concurrent room cap (${CONCURRENT_ROOM_CAP}) reached`,
        });
      }
      await this.save({
        ...s,
        opened: alreadyHeld ? s.opened : s.opened + 1,
        live: { ...s.live, [room]: Date.now() + ROOM_SLOT_TTL_MS },
      });
      return Response.json({ ok: true });
    }

    if (action === 'close') {
      const live = { ...s.live };
      delete live[room];
      await this.save({ ...s, live });
      return Response.json({ ok: true });
    }

    if (action === 'status') {
      return Response.json({
        day: s.day,
        opened: s.opened,
        concurrent: Object.keys(s.live).length,
        liveRooms: Object.keys(s.live),
        dailyCap: DAILY_ROOM_CAP,
        concurrentCap: CONCURRENT_ROOM_CAP,
      });
    }

    // Manual escape hatch: clears every live slot without touching the daily
    // count. Only reachable with the shared token (checked in the Worker).
    if (action === 'reset') {
      await this.save({ ...s, live: {} });
      return Response.json({ ok: true, reset: true });
    }

    return new Response('unknown action', { status: 400 });
  }
}
