// OpenRouter Gemini AI Assistant with Direct Audio Input & Tool Calling
import type { ActModelDef, PathPlanAction } from './boardSim';

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
  knownStations: Array<{ id: number; label: string; description?: string; prop?: string; stationType?: string; x: number; y: number }>;
  armDetectedTags: number[];
  baseDetectedTags: number[];
  activeCamera: 'base' | 'arm';
  currentLayoutName?: string;
  availableStls?: string[];
  availableActModels?: ActModelDef[];
  activePlanSummary?: {
    name: string;
    status: string;
    currentStepIndex: number;
    totalSteps: number;
    steps: Array<{ action: string; description: string; status: string }>;
  } | null;
}

export interface RobotActionCallbacks {
  onNavigate?: (target: number | string) => Promise<string> | string | void;
  onPickTag?: (tagId?: number) => Promise<string> | string | void;
  onExplore?: () => Promise<string> | string | void;
  onLookForItems?: () => Promise<string> | string | void;
  onPutInBasket?: () => Promise<string> | string | void;
  onOpenGripper?: () => Promise<string> | string | void;
  onCloseGripper?: () => Promise<string> | string | void;
  onResetToCenter?: () => Promise<string> | string | void;
  onSwitchCamera?: (cam: 'base' | 'arm') => Promise<string> | string | void;
  onStop?: () => Promise<string> | string | void;
  onAddStation?: (args: { name: string; side?: 'left' | 'right'; prop?: string; station_type?: 'simple' | 'printer' | 'deposit' | 'modifier' | 'table'; stl_name?: string; item_count?: number }) => Promise<string> | string | void;
  onPrint3dObject?: (args: { station_id?: number; stl_name?: string }) => Promise<string> | string | void;
  onRenewDeposit?: (args: { station_id: number }) => Promise<string> | string | void;
  onRunMagicModification?: (args: { station_id: number; output_stl?: string }) => Promise<string> | string | void;
  onGetStationLayout?: () => Promise<any> | any;
  onSetStationLayout?: (args: { layout_name?: string; layout_json: string }) => Promise<string> | string | void;
  onRunActModel?: (modelId: string) => Promise<string> | string | void;
  onPlanMissionPath?: (args: {
    plan_name?: string;
    description?: string;
    steps: Array<{
      action: PathPlanAction;
      target?: number | string;
      model_id?: string;
      tag_id?: number;
      duration?: number;
      description: string;
    }>;
    auto_start?: boolean;
  }) => Promise<string> | string | void;
  onGetMissionPlan?: () => any;
  onAbortMissionPlan?: () => Promise<string> | string | void;
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
      description: 'Drive the mobile robot base to a known station / pedestal by its label name (e.g. "APPLE", "BANANA", "BASKET", "PLANT", "3D PRINTER") or tag ID.',
      parameters: {
        type: 'object',
        properties: {
          station_label: {
            type: 'string',
            description: 'The label name of the station to navigate to (e.g. "Apple", "Banana", "Basket", "Plant", "Bottle", "Block", "Orange", "3D Printer", "Deposit").',
          },
          station_id: {
            type: 'number',
            description: 'The numeric navigation tag ID of the station (e.g. 201, 202, 203, 204, 205).',
          },
        },
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
  {
    type: 'function',
    function: {
      name: 'create_station',
      description: 'Create a new station along the rack in simulation (e.g. 3D printer, deposit dispenser, magic modifier, simple, or table). Placed on the left or right of existing stations, before the stop tags.',
      parameters: {
        type: 'object',
        properties: {
          name: {
            type: 'string',
            description: 'The name or label of the new station (e.g. "3D Printer", "Screws Deposit", "Magic Modifier", "Assembly Table", "Coffee").',
          },
          side: {
            type: 'string',
            enum: ['left', 'right'],
            description: 'Whether to place the new station on the "left" (+Y corridor side) or "right" (-Y corridor side) of existing stations. Defaults to "right".',
          },
          station_type: {
            type: 'string',
            enum: ['simple', 'printer', 'deposit', 'modifier', 'table'],
            description: 'Station archetype: "printer" (3D printer), "deposit" (replenishable dispenser), "modifier" (magic transformation chamber), "simple" (standard item pedestal), or "table" (empty drop/work table).',
          },
          prop: {
            type: 'string',
            enum: ['block', 'apple', 'orange', 'banana', 'bottle', 'plant', 'basket', 'none'],
            description: 'Optional default prop kind to assign.',
          },
          stl_name: {
            type: 'string',
            description: 'Optional custom STL filename to assign to the station (e.g. "gear.stl", "bracket.stl").',
          },
          item_count: {
            type: 'number',
            description: 'Initial quantity of items for deposit stations (e.g. 3, 5).',
          },
        },
        required: ['name'],
      },
    },
  },
  {
    type: 'function',
    function: {
      name: 'print_3d_object',
      description: 'Command the 3D printer station to print an object from a specified STL file or part definition onto its print bed.',
      parameters: {
        type: 'object',
        properties: {
          station_id: {
            type: 'number',
            description: 'Tag ID or station ID of the 3D printer station. If omitted, uses the first available 3D printer.',
          },
          stl_name: {
            type: 'string',
            description: 'Optional name of the STL 3D model to print (e.g. "gear.stl", "bracket.stl").',
          },
        },
      },
    },
  },
  {
    type: 'function',
    function: {
      name: 'renew_deposit_station',
      description: 'Replenish and restock a deposit / dispenser station back to its original full item quantity.',
      parameters: {
        type: 'object',
        properties: {
          station_id: {
            type: 'number',
            description: 'Tag ID of the deposit station to replenish.',
          },
        },
        required: ['station_id'],
      },
    },
  },
  {
    type: 'function',
    function: {
      name: 'run_magic_modification',
      description: 'Activate a magic modification station to transform items inside into a new STL output object.',
      parameters: {
        type: 'object',
        properties: {
          station_id: {
            type: 'number',
            description: 'Tag ID of the magic modification station.',
          },
          output_stl: {
            type: 'string',
            description: 'Target STL model or output part name to produce.',
          },
        },
        required: ['station_id'],
      },
    },
  },
  {
    type: 'function',
    function: {
      name: 'get_station_layout',
      description: 'Get the full station layout JSON of the current scene, including all station definitions, archetypes, names, coordinates, tag numbers, and custom STLs.',
      parameters: {
        type: 'object',
        properties: {},
      },
    },
  },
  {
    type: 'function',
    function: {
      name: 'set_station_layout',
      description: 'Upload or apply a station layout to the simulation environment from a JSON string or layout specification.',
      parameters: {
        type: 'object',
        properties: {
          layout_name: {
            type: 'string',
            description: 'Optional name for the new layout (e.g. "Electronics Workbench", "Produce Store").',
          },
          layout_json: {
            type: 'string',
            description: 'JSON string containing either a list of stations or an object with { layoutName, stations: [...] }',
          },
        },
        required: ['layout_json'],
      },
    },
  },
  {
    type: 'function',
    function: {
      name: 'run_act_model',
      description: 'Execute an ACT (Action Chunking with Transformers) policy model on the robot arm for manipulation tasks.',
      parameters: {
        type: 'object',
        properties: {
          model_id: {
            type: 'string',
            description: 'The ACT model identifier (e.g. "lerobot/act_so100_pick_block", "lerobot/act_so100_sort_apple", "lerobot/act_so100_banana_grab", "lerobot/act_so100_basket_drop").',
          },
        },
        required: ['model_id'],
      },
    },
  },
  {
    type: 'function',
    function: {
      name: 'plan_mission_path',
      description: 'Create and orchestrate a multi-action sequential path plan for the robot to execute across locations and models. Note: the UI has NO manual edit controls for this path planner; only Gemini can construct and modify the path plan. Actions execute in succession.',
      parameters: {
        type: 'object',
        properties: {
          plan_name: {
            type: 'string',
            description: 'Descriptive title for the mission (e.g. "Pick Block & Store in Basket", "Multi-Fruit Retrieval").',
          },
          description: {
            type: 'string',
            description: 'Clear description of the mission objectives and routine.',
          },
          auto_start: {
            type: 'boolean',
            description: 'Whether to immediately begin executing the path plan. Defaults to true.',
          },
          steps: {
            type: 'array',
            description: 'Ordered sequence of steps to execute.',
            items: {
              type: 'object',
              properties: {
                action: {
                  type: 'string',
                  enum: ['navigate', 'run_act', 'pick_tag', 'put_basket', 'open_gripper', 'close_gripper', 'wait'],
                  description: 'Action to perform: "navigate" (drive to station), "run_act" (execute ACT policy), "pick_tag" (pick item by ArUco tag via IK), "put_basket" (deposit held item), "open_gripper", "close_gripper", or "wait".',
                },
                target: {
                  type: 'string',
                  description: 'Station label or numeric tag ID for navigation (e.g. "BLOCK", "APPLE", "BASKET", "201").',
                },
                model_id: {
                  type: 'string',
                  description: 'ACT model ID to run (e.g. "lerobot/act_so100_pick_block") when action is "run_act".',
                },
                tag_id: {
                  type: 'number',
                  description: 'ArUco tag ID to pick up when action is "pick_tag" (e.g. 101, 102).',
                },
                duration: {
                  type: 'number',
                  description: 'Wait duration in seconds when action is "wait".',
                },
                description: {
                  type: 'string',
                  description: 'Human-readable description of what this step does (e.g. "Drive to Block Station 201", "Run ACT Model: Block Pick", "Drive to Basket 204", "Deposit Block in Basket").',
                },
              },
              required: ['action', 'description'],
            },
          },
        },
        required: ['steps'],
      },
    },
  },
  {
    type: 'function',
    function: {
      name: 'get_mission_plan',
      description: 'Get the currently active path plan, its steps, and execution progress.',
      parameters: {
        type: 'object',
        properties: {},
      },
    },
  },
  {
    type: 'function',
    function: {
      name: 'abort_mission_plan',
      description: 'Stop and cancel the active mission path plan.',
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
    ? context.knownStations
        .map((s) => {
          const xNum = Number.isFinite(s.x) ? s.x : (Number.isFinite((s as any).p?.[0]) ? (s as any).p[0] : 0);
          const yNum = Number.isFinite(s.y) ? s.y : (Number.isFinite((s as any).p?.[1]) ? (s as any).p[1] : 0);
          const typeStr = s.stationType ? ` [${s.stationType.toUpperCase()}]` : '';
          const actStr = (s as any).actModelId ? ` [ACT: ${(s as any).actModelId}]` : '';
          return `• Tag ${s.id} (${s.label}${typeStr}${actStr}) at x=${xNum.toFixed(2)}, y=${yNum.toFixed(2)}`;
        })
        .join('\n')
    : 'No tags discovered yet. Run exploration to find tags in the arena.';

  const stlSummary = context.availableStls?.length
    ? `Available 3D STL Models: [${context.availableStls.join(', ')}]`
    : 'No custom STL models uploaded yet.';

  const actSummary = context.availableActModels?.length
    ? context.availableActModels
        .map((m) => `• ${m.name} [ID: ${m.id}] (Dataset: ${m.datasetId})${m.stationTag ? ` for Station ${m.stationTag}` : ''}`)
        .join('\n')
    : 'No ACT models registered in layout.';

  const planSummary = context.activePlanSummary
    ? `Active Mission Plan: "${context.activePlanSummary.name}" [Status: ${context.activePlanSummary.status}, Step ${context.activePlanSummary.currentStepIndex + 1}/${context.activePlanSummary.totalSteps}]\n` +
      context.activePlanSummary.steps.map((s, i) => `  ${i + 1}. [${s.status.toUpperCase()}] ${s.description}`).join('\n')
    : 'No active mission path plan.';

  const systemPrompt = `You are the AI voice controller and assistant for Instant Robot (SO-101 robotic arm + LeKiwi holonomic base).
You can receive audio voice recordings directly from the user or text messages, understand their intent, and call tools to control the robot.

Current Robot State:
- Position: X=${context.robotX.toFixed(2)}m, Y=${context.robotY.toFixed(2)}m, Heading=${context.robotYawDeg.toFixed(0)}°
- Active Layout: "${context.currentLayoutName || 'Default Layout'}" (${context.knownStations.length} stations)
- Status: ${context.exploring ? 'Exploring arena' : context.navigating ? `Navigating to tag ${context.activeNavTag}` : context.atTag ? `At tag ${context.atTag}` : 'Idle'}
- Holding object: ${context.holdingItem ? 'Yes' : 'No'}
- Active camera: ${context.activeCamera}
- Visible tags (Arm cam): [${context.armDetectedTags.join(', ')}]
- Visible tags (Base cam): [${context.baseDetectedTags.join(', ')}]
- ${stlSummary}

Available ACT Models (Hugging Face LeRobot):
${actSummary}

Current Path Plan:
${planSummary}

Discovered Tags & Stations:
${stationSummary}

Rules & Capabilities:
1. Basic Actions: Call navigate_to_station, pick_up_object, explore_arena, look_for_items, put_in_basket, open_gripper, reset_to_center, switch_camera, stop_robot when told to execute individual actions.
2. ACT Models: Use run_act_model(model_id) to run a single ACT policy model directly on the robot arm.
3. Path Planner & Multi-Action Missions:
   - There are NO manual UI controls for editing path plans: ONLY Gemini can compose or modify the path plan!
   - When the user asks for multi-step goals or complex routines (e.g. "go to the block, pick it up with ACT, bring it to the basket, then go to the apple and sort it"), compose a sequential path plan using plan_mission_path.
   - You can chain multiple locations, ArUco IK picks, and multiple trained ACT models in succession!
   - If the user asks to inspect or cancel a plan, use get_mission_plan or abort_mission_plan.
4. Station Archetypes & Specialized Tools:
   - 3D Printer: Call print_3d_object(station_id, stl_name) when told to 3D print an item.
   - Deposit Station: Call renew_deposit_station(station_id) when told to replenish / restock items.
   - Magic Modifier: Call run_magic_modification(station_id, output_stl) when told to modify or transmute items.
   - Create Station: Call create_station(name, station_type, prop, stl_name, item_count) to build stations.
5. If asked to inspect, read, export, or modify the station layout JSON/ZIP, use get_station_layout or set_station_layout.
6. If the user sent audio voice input, listen to what they said, execute their request, and briefly respond explaining what you are doing.
7. Keep your spoken/text responses concise, friendly, and direct (1-2 sentences).
8. Do not use emojis in your responses. Keep responses clean, professional, and formatted in Markdown when helpful.`;

  const messages: any[] = [{ role: 'system', content: systemPrompt }];

  // Include recent conversation
  for (const msg of conversation.slice(-8)) {
    if (msg.audioBase64) {
      // Multimodal audio message
      messages.push({
        role: msg.role,
        content: [
          {
            type: 'text',
            text: msg.content || 'Spoken user voice instruction:',
          },
          {
            type: 'input_audio',
            input_audio: {
              data: msg.audioBase64,
              format: msg.audioFormat || 'webm',
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
      'Authorization': `Bearer ${apiKey.trim()}`,
      'Content-Type': 'application/json',
      'HTTP-Referer': 'https://github.com/therealadityashankar/instant-robot',
      'X-Title': 'Instant Robot AI Controller',
    },
    body: JSON.stringify({
      model,
      messages,
      tools: TOOLS,
      tool_choice: 'auto',
      max_tokens: 450,
      temperature: 0.2,
    }),
  });

  if (!response.ok) {
    const errText = await response.text();
    let parsedMsg = errText;
    try {
      const j = JSON.parse(errText);
      parsedMsg = j.error?.message || errText;
    } catch {}
    throw new Error(`OpenRouter API error (${response.status}): ${parsedMsg}`);
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
        } else if (fnName === 'create_station') {
          const res = await callbacks.onAddStation?.({
            name: fnArgs.name,
            side: fnArgs.side,
            prop: fnArgs.prop,
            station_type: fnArgs.station_type,
            stl_name: fnArgs.stl_name,
            item_count: fnArgs.item_count,
          });
          toolResult = res || `Created station "${fnArgs.name}"`;
        } else if (fnName === 'print_3d_object') {
          const res = await callbacks.onPrint3dObject?.({
            station_id: fnArgs.station_id,
            stl_name: fnArgs.stl_name,
          });
          toolResult = res || `3D printing ${fnArgs.stl_name || 'object'}`;
        } else if (fnName === 'renew_deposit_station') {
          const res = await callbacks.onRenewDeposit?.({
            station_id: fnArgs.station_id,
          });
          toolResult = res || `Replenished deposit station ${fnArgs.station_id}`;
        } else if (fnName === 'run_magic_modification') {
          const res = await callbacks.onRunMagicModification?.({
            station_id: fnArgs.station_id,
            output_stl: fnArgs.output_stl,
          });
          toolResult = res || `Transformation executed at station ${fnArgs.station_id}`;
        } else if (fnName === 'get_station_layout') {
          const res = await callbacks.onGetStationLayout?.();
          toolResult = typeof res === 'string' ? res : JSON.stringify(res || {}, null, 2);
        } else if (fnName === 'set_station_layout') {
          const res = await callbacks.onSetStationLayout?.({
            layout_name: fnArgs.layout_name,
            layout_json: fnArgs.layout_json,
          });
          toolResult = res || 'Applied station layout';
        } else if (fnName === 'run_act_model') {
          const res = await callbacks.onRunActModel?.(fnArgs.model_id);
          toolResult = res || `Executed ACT model "${fnArgs.model_id}"`;
        } else if (fnName === 'plan_mission_path') {
          const res = await callbacks.onPlanMissionPath?.({
            plan_name: fnArgs.plan_name,
            description: fnArgs.description,
            steps: fnArgs.steps,
            auto_start: fnArgs.auto_start,
          });
          toolResult = res || `Created mission path plan with ${fnArgs.steps?.length || 0} steps`;
        } else if (fnName === 'get_mission_plan') {
          const res = await callbacks.onGetMissionPlan?.();
          toolResult = typeof res === 'string' ? res : JSON.stringify(res || {}, null, 2);
        } else if (fnName === 'abort_mission_plan') {
          const res = await callbacks.onAbortMissionPlan?.();
          toolResult = res || 'Aborted active mission plan';
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
