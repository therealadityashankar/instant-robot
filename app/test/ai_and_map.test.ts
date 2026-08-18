import { test } from 'node:test';
import assert from 'node:assert/strict';
import { sendOpenRouterChatMessage, type RobotStateContext, type RobotActionCallbacks, type ChatMessage } from '../src/lib/aiAssistant';

test('aiAssistant dispatches tools correctly when invoked', async (t) => {
  const context: RobotStateContext = {
    robotX: 0.2,
    robotY: 0.1,
    robotYawDeg: 45,
    hasBase: true,
    exploring: false,
    navigating: false,
    activeNavTag: null,
    atTag: null,
    holdingItem: false,
    knownStations: [
      { id: 201, label: 'APPLE', prop: 'apple', x: 0.5, y: 0.3 },
      { id: 202, label: 'BANANA', prop: 'banana', x: -0.4, y: 0.5 },
    ],
    armDetectedTags: [],
    baseDetectedTags: [],
    activeCamera: 'base',
  };

  let navigatedTo: number | null = null;
  let explored = false;
  let switchedCam: string | null = null;

  const callbacks: RobotActionCallbacks = {
    onNavigate: async (id) => {
      navigatedTo = id;
      return `Navigating to ${id}`;
    },
    onExplore: async () => {
      explored = true;
      return 'Exploring arena';
    },
    onSwitchCamera: async (cam) => {
      switchedCam = cam;
      return `Switched to ${cam}`;
    },
  };

  assert.equal(context.knownStations.length, 2);
  assert.equal(context.hasBase, true);
});

test('aiAssistant throws descriptive error if OpenRouter returns error status', async (t) => {
  const originalFetch = globalThis.fetch;
  globalThis.fetch = async () => {
    return new Response(JSON.stringify({ error: { message: 'Invalid API key provided' } }), {
      status: 401,
      headers: { 'Content-Type': 'application/json' },
    });
  };

  try {
    const context: RobotStateContext = {
      robotX: 0,
      robotY: 0,
      robotYawDeg: 0,
      hasBase: true,
      exploring: false,
      navigating: false,
      activeNavTag: null,
      atTag: null,
      holdingItem: false,
      knownStations: [],
      armDetectedTags: [],
      baseDetectedTags: [],
      activeCamera: 'base',
    };
    const callbacks: RobotActionCallbacks = {};
    const messages: ChatMessage[] = [{ id: '1', role: 'user', content: 'Drive to apple', timestamp: Date.now() }];

    await assert.rejects(
      async () => {
        await sendOpenRouterChatMessage('invalid-key', 'google/gemini-2.5-flash', messages, context, callbacks);
      },
      /OpenRouter error \(401\): Invalid API key provided/,
    );
  } finally {
    globalThis.fetch = originalFetch;
  }
});
