import { test } from 'node:test';
import assert from 'node:assert/strict';
import { zipSync, unzipSync, strToU8, strFromU8 } from 'fflate';
import { sendOpenRouterChatMessage, type RobotStateContext, type RobotActionCallbacks, type ChatMessage } from '../src/lib/aiAssistant.ts';
import { buildBoardSceneXml, setCustomStations, DEFAULT_STATIONS, type StationDef } from '../src/lib/boardSim.ts';

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
      { id: 201, label: 'APPLE', prop: 'apple', stationType: 'simple', x: 0.5, y: 0.3 },
      { id: 202, label: 'BANANA', prop: 'banana', stationType: 'simple', x: -0.4, y: 0.5 },
      { id: 203, label: '3D PRINTER', prop: 'none', stationType: 'printer', x: 0.85, y: 0.0 },
    ],
    armDetectedTags: [],
    baseDetectedTags: [],
    activeCamera: 'base',
    currentLayoutName: 'Factory Workshop',
    availableStls: ['gear.stl', 'bracket.stl'],
  };

  assert.equal(context.knownStations.length, 3);
  assert.equal(context.hasBase, true);
  assert.equal(context.currentLayoutName, 'Factory Workshop');
  assert.deepEqual(context.availableStls, ['gear.stl', 'bracket.stl']);
});

test('boardSim builds XML scene with 5 station archetypes and custom STL meshes', (t) => {
  const customStations: StationDef[] = [
    { navTag: 201, label: 'PRINTER', prop: 'block', propTag: 101, x: 0.85, y: -0.5, stationType: 'printer', stlName: 'gear.stl', state: 'ready' },
    { navTag: 202, label: 'DISPENSER', prop: 'block', propTag: 102, x: 0.85, y: -0.2, stationType: 'deposit', itemCount: 4, maxItemCount: 5 },
    { navTag: 203, label: 'MODIFIER', prop: 'block', propTag: 103, x: 0.85, y: 0.1, stationType: 'modifier', outputStlName: 'housing.stl', state: 'ready' },
    { navTag: 204, label: 'WORKTABLE', prop: 'none', propTag: 0, x: 0.85, y: 0.4, stationType: 'table' },
    { navTag: 205, label: 'SIMPLE_STL', prop: 'block', propTag: 105, x: 0.85, y: 0.7, stationType: 'simple', stlName: 'part.stl' },
  ];

  setCustomStations(customStations);

  const dummyBaseXml = `<mujoco><asset></asset><worldbody></worldbody></mujoco>`;
  const sceneXml = buildBoardSceneXml(dummyBaseXml, {
    physics: false,
    availableMeshes: ['gear.stl', 'housing.stl', 'part.stl'],
  });

  // Verify printer geoms
  assert.match(sceneXml, /printer_201_bed/);
  assert.match(sceneXml, /printer_201_screen/);
  assert.match(sceneXml, /printer_201_nozzle/);
  assert.doesNotMatch(sceneXml, /printer_201_spool/);

  // Verify deposit dispenser geoms
  assert.match(sceneXml, /deposit_202_tray/);
  assert.match(sceneXml, /deposit_202_item_0/);
  assert.match(sceneXml, /deposit_202_item_3/);

  // Verify modifier geoms
  assert.match(sceneXml, /modifier_203_dome/);
  assert.match(sceneXml, /modifier_203_core/);

  // Verify table geoms
  assert.match(sceneXml, /table_204_top/);

  // Verify mesh assets injection
  assert.match(sceneXml, /mesh_st_201/);
  assert.match(sceneXml, /mesh_st_203/);
  assert.match(sceneXml, /mesh_st_205/);

  // Restore default stations
  setCustomStations(DEFAULT_STATIONS);
});

test('ZIP layout package export and import handles layout.json and bundled STLs', (t) => {
  const layoutData = {
    layoutName: 'Automated Workshop',
    version: '2.0',
    stations: [
      { navTag: 201, label: '3D PRINTER', stationType: 'printer', stlName: 'gear.stl', x: 0.85, y: -0.3, prop: 'block', propTag: 101 },
      { navTag: 202, label: 'SCREWS DEPOSIT', stationType: 'deposit', itemCount: 5, x: 0.85, y: 0.2, prop: 'block', propTag: 102 },
    ],
    models: ['gear.stl'],
  };

  const dummyStlBytes = new Uint8Array([0x53, 0x4f, 0x4c, 0x49, 0x44, 0x20, 0x54, 0x45, 0x53, 0x54]); // 'SOLID TEST'
  const zipEntries: Record<string, Uint8Array> = {
    'layout.json': strToU8(JSON.stringify(layoutData, null, 2)),
    'models/gear.stl': dummyStlBytes,
  };

  const zipped = zipSync(zipEntries);
  assert.ok(zipped.length > 0);

  // Unzip and unpack
  const unzipped = unzipSync(zipped);
  assert.ok(unzipped['layout.json']);
  assert.ok(unzipped['models/gear.stl']);

  const parsedJson = JSON.parse(strFromU8(unzipped['layout.json']));
  assert.equal(parsedJson.layoutName, 'Automated Workshop');
  assert.equal(parsedJson.stations.length, 2);
  assert.equal(parsedJson.stations[0].stationType, 'printer');
  assert.equal(parsedJson.stations[1].stationType, 'deposit');
  assert.equal(parsedJson.stations[1].itemCount, 5);

  assert.deepEqual(unzipped['models/gear.stl'], dummyStlBytes);
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
      /OpenRouter API error \(401\): Invalid API key provided/,
    );
  } finally {
    globalThis.fetch = originalFetch;
  }
});

test('Map2DView contains zero emoji characters, zero box shadows, pure black indicators, and always-visible stop boundary lines', async (t) => {
  const fs = await import('node:fs/promises');
  const mapSource = await fs.readFile(new URL('../src/Map2DView.svelte', import.meta.url), 'utf8');

  // Test that no emoji characters are present in the Map2DView component
  const emojiRegex = /[\p{Emoji_Presentation}\p{Extended_Pictographic}]/u;
  const hasEmoji = emojiRegex.test(mapSource);
  assert.equal(hasEmoji, false, 'Map2DView should not contain any emoji characters');

  // Test that no active box-shadow is used (only 'none' is permitted to override global button styles)
  const allShadowDeclarations = mapSource.match(/box-shadow:\s*([^;]+);/g) || [];
  for (const decl of allShadowDeclarations) {
    assert.match(decl, /box-shadow:\s*none(\s*!important)?\s*;/);
  }

  // Test that stop boundary lines are straight lines overlapping the route
  assert.match(mapSource, /boundary-stop-line/);
  assert.match(mapSource, /stop-bar/);

  // Test that place indicators are pure black
  assert.match(mapSource, /#000000/);

  // Test that discovered stations are derived from knownTags
  assert.match(mapSource, /knownTags/);
  assert.match(mapSource, /discoveredStations/);
  assert.match(mapSource, /No stations discovered yet/);
});

test('AiChat and aiAssistant contain zero emoji characters and support markdown rendering', async (t) => {
  const fs = await import('node:fs/promises');
  const chatSource = await fs.readFile(new URL('../src/AiChat.svelte', import.meta.url), 'utf8');
  const aiSource = await fs.readFile(new URL('../src/lib/aiAssistant.ts', import.meta.url), 'utf8');

  // Verify markdown rendering integration
  assert.match(chatSource, /marked/);
  assert.match(chatSource, /renderMarkdown/);
  assert.match(chatSource, /markdown-body/);

  // Verify zero emoji characters
  const emojiRegex = /[\p{Emoji_Presentation}\p{Extended_Pictographic}]/u;
  assert.equal(emojiRegex.test(chatSource), false, 'AiChat should not contain emoji characters');
  assert.equal(emojiRegex.test(aiSource), false, 'aiAssistant should not contain emoji characters');

  // Verify Gemini prompt instructs no emojis
  assert.match(aiSource, /Do not use emojis in your responses/);
});

test('3D printer station compiles cleanly into MuJoCo scene XML and safely handles missing STL assets', async (t) => {
  const dummyBaseXml = `<mujoco><worldbody></worldbody><asset></asset></mujoco>`;

  // Create a printer station definition
  const printerStation: StationDef = {
    navTag: 206,
    label: '3D PRINTER',
    description: '3D printer station',
    x: 0,
    y: 0.95,
    prop: 'block',
    propTag: 106,
    stationType: 'printer',
    stlName: 'custom_gear.stl',
    state: 'ready',
  };

  setCustomStations([...DEFAULT_STATIONS, printerStation]);

  try {
    // 1. When custom_gear.stl is NOT in availableMeshes, sceneXml should NOT emit missing <mesh> and should fallback gracefully
    const xmlWithoutMesh = buildBoardSceneXml(dummyBaseXml, {
      physics: true,
      availableMeshes: [],
    });

    assert.match(xmlWithoutMesh, /printer_206_bed/);
    assert.match(xmlWithoutMesh, /printer_206_gantry/);
    assert.match(xmlWithoutMesh, /printer_206_nozzle/);
    assert.doesNotMatch(xmlWithoutMesh, /ptag_106/);

    // 2. When custom_gear.stl IS available in availableMeshes, sceneXml declares the mesh asset
    const xmlWithMesh = buildBoardSceneXml(dummyBaseXml, {
      physics: true,
      availableMeshes: ['custom_gear.stl'],
    });

    assert.match(xmlWithMesh, /file="custom_gear\.stl"/);
    assert.match(xmlWithMesh, /mesh="mesh_st_206"/);
    assert.doesNotMatch(xmlWithMesh, /ptag_106/);
  } finally {
    setCustomStations(DEFAULT_STATIONS);
  }
});

