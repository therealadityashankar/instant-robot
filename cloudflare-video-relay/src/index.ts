export class VideoRoom implements DurableObject {
  state: DurableObjectState;
  sessions: Set<WebSocket> = new Set();
  robotWs: WebSocket | null = null;

  constructor(state: DurableObjectState) {
    this.state = state;
  }

  async fetch(request: Request): Promise<Response> {
    const url = new URL(request.url);
    const role = url.searchParams.get("role") || "browser";

    const pair = new WebSocketPair();
    const [client, server] = Object.values(pair);

    server.accept();

    if (role === "robot") {
      this.robotWs = server;
      server.addEventListener("message", async (event) => {
        let data = event.data;
        if (data instanceof Blob) {
          data = await data.arrayBuffer();
        }
        for (const ws of this.sessions) {
          if (ws.readyState === WebSocket.OPEN) {
            try {
              ws.send(data);
            } catch {}
          }
        }
      });
      server.addEventListener("close", () => {
        if (this.robotWs === server) this.robotWs = null;
      });
    } else {
      this.sessions.add(server);
      server.addEventListener("close", () => {
        this.sessions.delete(server);
      });
    }

    return new Response(null, { status: 101, webSocket: client });
  }
}

export default {
  async fetch(request: Request, env: any): Promise<Response> {
    const url = new URL(request.url);
    const roomId = url.searchParams.get("room") || "my-lekiwi";

    if (request.headers.get("Upgrade") !== "websocket") {
      return new Response("LeKiwi Video WebSocket Relay Live", { status: 200 });
    }

    const id = env.ROOMS.idFromName(roomId);
    const obj = env.ROOMS.get(id);
    return obj.fetch(request);
  }
};
