// OpenRouter Gemini AI Assistant with Direct Audio Input & Tool Calling

export interface RobotStateContext {
  robotX: number;
  robotY: number;
  robotYawDeg: number;
  hasBase: boolean;
  exploring: boolean;
  navigating: boolean;
  activeNavTag: number | null;
  atTag: number | null;
  holdingItem: boolean;
  knownStations: Array<{ id: number; label: string; prop?: string; x: number; y: number }>;
  armDetectedTags: number[];
  baseDetectedTags: number[];
  activeCamera: 'base' | 'arm';
}

export interface RobotActionCallbacks {
  onNavigate?: (stationId: number) => Promise<string> | string | void;
  onPickTag?: (tagId?: number) => Promise<string> | string | void;
  onExplore?: () => Promise<string> | string | void;
  onLookForItems?: () => Promise<string> | string | void;
  onPutInBasket?: () => Promise<string> | string | void;
  onOpenGripper?: () => Promise<string> | string | void;
  onCloseGripper?: () => Promise<string> | string | void;
  onResetToCenter?: () => Promise<string> | string | void;
  onSwitchCamera?: (cam: 'base' | 'arm') => Promise<string> | string | void;
  onStop?: () => Promise<string> | string | void;
}

export interface ChatMessage {
  id: string;
  role: 'user' | 'assistant' | 'system';
  content: string;
  timestamp: number;
  audioBase64?: string;
  audioFormat?: string;
  audioMimeType?: string;
  toolCalls?: Array<{ name: string; args: any; result?: string }>;
}

export const DEFAULT_MODEL = 'google/gemini-3.7-flash';

export const POPULAR_MODELS = [
  { id: 'google/gemini-3.7-flash', label: 'Gemini 3.7 Flash (Recommended - newest, fast multimodal audio & tools)' },
  { id: 'google/gemini-2.5-flash', label: 'Gemini 2.5 Flash' },
  { id: 'google/gemini-2.0-flash-001', label: 'Gemini 2.0 Flash' },
  { id: 'google/gemini-2.5-pro', label: 'Gemini 2.5 Pro (Highest capability)' },
  { id: 'google/gemini-flash-1.5', label: 'Gemini 1.5 Flash' },
];

const TOOLS = [
  {
    type: 'function',
    function: {
      name: 'navigate_to_station',
      description: 'Drive the mobile robot base to a known station / pedestal by its tag ID.',
      parameters: {
        type: 'object',
        properties: {
          station_id: {
            type: 'number',
            description: 'The tag ID of the station to navigate to (e.g. 200, 201, 202, 203, 204, 205, 206).',
          },
        },
        required: ['station_id'],
      },
    },
  },
  {
    type: 'function',
    function: {
      name: 'pick_up_object',
      description: 'Pick up an object located at the current station using the robot arm.',
      parameters: {
        type: 'object',
        properties: {
          tag_id: {
            type: 'number',
            description: 'Optional tag ID of the object to pick (e.g. 101, 102, 103, 105).',
          },
        },
      },
    },
  },
  {
    type: 'function',
    function: {
      name: 'explore_arena',
      description: 'Sweep and explore the arena to scan for fiducial tags and discover stations.',
      parameters: {
        type: 'object',
        properties: {},
      },
    },
  },
  {
    type: 'function',
    function: {
      name: 'look_for_items',
      description: 'Perform a camera scan at the current station to search for pickable items.',
      parameters: {
        type: 'object',
        properties: {},
      },
    },
  },
  {
    type: 'function',
    function: {
      name: 'put_in_basket',
      description: 'Deliver the currently held item into the robot payload basket.',
      parameters: {
        type: 'object',
        properties: {},
      },
    },
  },
  {
    type: 'function',
    function: {
      name: 'open_gripper',
      description: 'Open the robot arm gripper.',
      parameters: {
        type: 'object',
        properties: {},
      },
    },
  },
  {
    type: 'function',
    function: {
      name: 'reset_to_center',
      description: 'Move the robot base back to origin (0, 0, 0°).',
      parameters: {
        type: 'object',
        properties: {},
      },
    },
  },
  {
    type: 'function',
    function: {
      name: 'switch_camera',
      description: 'Switch the live video stream between base and arm camera.',
      parameters: {
        type: 'object',
        properties: {
          camera: {
            type: 'string',
            enum: ['base', 'arm'],
            description: 'Which camera to switch to.',
          },
        },
        required: ['camera'],
      },
    },
  },
  {
    type: 'function',
    function: {
      name: 'stop_robot',
      description: 'Stop any current navigation, movement, or exploration.',
      parameters: {
        type: 'object',
        properties: {},
      },
    },
  },
];

export async function sendOpenRouterChatMessage(
  apiKey: string,
  model: string = DEFAULT_MODEL,
  conversation: ChatMessage[],
  context: RobotStateContext,
  callbacks: RobotActionCallbacks,
): Promise<ChatMessage> {
  const stationSummary = context.knownStations.length
    ? context.knownStations.map((s) => `• Tag ${s.id} at x=${s.x.toFixed(2)}, y=${s.y.toFixed(2)}`).join('\n')
    : 'No tags discovered yet. Run exploration to find tags in the arena.';

  const systemPrompt = `You are the AI voice controller and assistant for Instant Robot (SO-101 robotic arm + LeKiwi holonomic base).
You can receive audio voice recordings directly from the user or text messages, understand their intent, and call tools to control the robot.

Current Robot State:
- Position: X=${context.robotX.toFixed(2)}m, Y=${context.robotY.toFixed(2)}m, Heading=${context.robotYawDeg.toFixed(0)}°
- Status: ${context.exploring ? 'Exploring arena' : context.navigating ? `Navigating to tag ${context.activeNavTag}` : context.atTag ? `At tag ${context.atTag}` : 'Idle'}
- Holding object: ${context.holdingItem ? 'Yes' : 'No'}
- Active camera: ${context.activeCamera}
- Visible tags (Arm cam): [${context.armDetectedTags.join(', ')}]
- Visible tags (Base cam): [${context.baseDetectedTags.join(', ')}]

Discovered Tags:
${stationSummary}

Rules:
1. When asked or told via voice/text to perform an action (e.g. "go to tag 200", "drive to tag 201", "pick tag 101", "explore", "switch camera to arm", "reset robot"), call the appropriate tool.
2. If the user sent audio voice input, listen to what they said, execute their request, and briefly respond explaining what you are doing.
3. Keep your spoken/text responses concise, friendly, and direct (1-2 sentences).
4. If asked to go to a tag that is not yet discovered, explain that exploration is needed first and offer to start exploration.`;

  const messages: any[] = [{ role: 'system', content: systemPrompt }];

  // Include recent conversation
  for (const msg of conversation.slice(-8)) {
    if (msg.audioBase64) {
      // Multimodal audio message
      messages.push({
        role: msg.role,
        content: [
          ...(msg.content ? [{ type: 'text', text: msg.content }] : [{ type: 'text', text: 'User voice command audio:' }]),
          {
            type: 'input_audio',
            input_audio: {
              data: msg.audioBase64,
              format: msg.audioFormat || 'wav',
            },
          },
        ],
      });
    } else {
      messages.push({
        role: msg.role,
        content: msg.content,
      });
    }
  }

  const response = await fetch('https://openrouter.ai/api/v1/chat/completions', {
    method: 'POST',
    headers: {
      'Authorization': `Bearer ${apiKey}`,
      'HTTP-Referer': typeof window !== 'undefined' ? window.location.origin : 'https://instant.robot',
      'X-Title': 'Instant Robot',
      'Content-Type': 'application/json',
    },
    body: JSON.stringify({
      model: model || DEFAULT_MODEL,
      messages,
      tools: TOOLS,
      tool_choice: 'auto',
      temperature: 0.2,
    }),
  });

  if (!response.ok) {
    const errorText = await response.text();
    let parsedMsg = errorText;
    try {
      const errJson = JSON.parse(errorText);
      parsedMsg = errJson.error?.message || errJson.message || errorText;
    } catch {}
    throw new Error(`OpenRouter error (${response.status}): ${parsedMsg}`);
  }

  const data = await response.json();
  const choice = data.choices?.[0]?.message;

  if (!choice) {
    throw new Error('No response message received from OpenRouter.');
  }

  const executedToolCalls: Array<{ name: string; args: any; result?: string }> = [];

  if (choice.tool_calls && choice.tool_calls.length > 0) {
    for (const toolCall of choice.tool_calls) {
      const fnName = toolCall.function?.name;
      let fnArgs: any = {};
      try {
        fnArgs = JSON.parse(toolCall.function?.arguments || '{}');
      } catch {}

      let toolResult = 'Action executed';

      try {
        if (fnName === 'navigate_to_station') {
          const res = await callbacks.onNavigate?.(fnArgs.station_id);
          toolResult = res || `Navigating to Station ${fnArgs.station_id}`;
        } else if (fnName === 'pick_up_object') {
          const res = await callbacks.onPickTag?.(fnArgs.tag_id);
          toolResult = res || `Picking up tag ${fnArgs.tag_id || 'item'}`;
        } else if (fnName === 'explore_arena') {
          const res = await callbacks.onExplore?.();
          toolResult = res || 'Exploration started';
        } else if (fnName === 'look_for_items') {
          const res = await callbacks.onLookForItems?.();
          toolResult = res || 'Looking for items';
        } else if (fnName === 'put_in_basket') {
          const res = await callbacks.onPutInBasket?.();
          toolResult = res || 'Putting item in basket';
        } else if (fnName === 'open_gripper') {
          const res = await callbacks.onOpenGripper?.();
          toolResult = res || 'Opened gripper';
        } else if (fnName === 'reset_to_center') {
          const res = await callbacks.onResetToCenter?.();
          toolResult = res || 'Reset robot to centre';
        } else if (fnName === 'switch_camera') {
          const res = await callbacks.onSwitchCamera?.(fnArgs.camera);
          toolResult = res || `Switched camera to ${fnArgs.camera}`;
        } else if (fnName === 'stop_robot') {
          const res = await callbacks.onStop?.();
          toolResult = res || 'Robot stopped';
        }
      } catch (err: any) {
        toolResult = `Error: ${err.message || String(err)}`;
      }

      executedToolCalls.push({
        name: fnName,
        args: fnArgs,
        result: toolResult,
      });
    }
  }

  let finalContent = choice.content || '';
  if (!finalContent && executedToolCalls.length > 0) {
    finalContent = executedToolCalls.map((t) => t.result).join('; ');
  }

  return {
    id: `msg-${Date.now()}-${Math.random().toString(36).slice(2, 6)}`,
    role: 'assistant',
    content: finalContent,
    timestamp: Date.now(),
    toolCalls: executedToolCalls.length > 0 ? executedToolCalls : undefined,
  };
}
