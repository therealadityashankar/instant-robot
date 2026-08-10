// Catch identifiers that are used but never declared.
//
// `vite build` compiles Svelte components without resolving top-level names, so
// deleting a `const` that something still reads builds perfectly and throws only
// when that code path runs — which here meant a step of the pick failing in the
// browser with "X is not defined", one name at a time. `svelte-check` would catch
// it, but it cannot run against TypeScript 7 (its API changed), so this does the
// one job that matters: every referenced identifier must be declared somewhere in
// the file, imported, or a known global.
//
// Scope-insensitive on purpose. A name declared in an inner scope counts as
// declared, so this never complains about shadowing or use-before-declare — it
// only reports names that exist nowhere, which is exactly the deletion mistake.

import { test } from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync, readdirSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { dirname, join } from 'node:path';
import { parse } from 'acorn';
import { transformSync } from 'esbuild';

const here = dirname(fileURLToPath(import.meta.url));
const SRC = join(here, '..', 'src');

/** Names available at runtime without being declared in the file. */
const GLOBALS = new Set([
  // language + standard library
  'globalThis', 'undefined', 'NaN', 'Infinity', 'Object', 'Array', 'String', 'Number',
  'Boolean', 'Symbol', 'BigInt', 'Math', 'JSON', 'Date', 'RegExp', 'Error', 'TypeError',
  'RangeError', 'Map', 'Set', 'WeakMap', 'WeakSet', 'Promise', 'Proxy', 'Reflect',
  'Int8Array', 'Uint8Array', 'Uint8ClampedArray', 'Int16Array', 'Uint16Array',
  'Int32Array', 'Uint32Array', 'Float32Array', 'Float64Array', 'ArrayBuffer', 'DataView',
  'parseInt', 'parseFloat', 'isNaN', 'isFinite', 'encodeURIComponent', 'decodeURIComponent',
  'structuredClone', 'queueMicrotask', 'console',
  // DOM / browser
  'window', 'document', 'navigator', 'location', 'performance', 'fetch', 'URL', 'Blob',
  'File', 'FileReader', 'Image', 'FormData', 'Headers', 'Request', 'Response', 'WebSocket',
  'MediaStream', 'ResizeObserver', 'MutationObserver', 'IntersectionObserver',
  'requestAnimationFrame', 'cancelAnimationFrame', 'setTimeout', 'clearTimeout',
  'setInterval', 'clearInterval', 'TextEncoder', 'TextDecoder', 'devicePixelRatio',
  'HTMLCanvasElement', 'HTMLVideoElement', 'HTMLElement', 'HTMLInputElement',
  'HTMLSelectElement', 'HTMLButtonElement', 'HTMLDivElement', 'CanvasRenderingContext2D',
  'ImageData', 'Event', 'CustomEvent', 'KeyboardEvent', 'PointerEvent', 'MouseEvent',
  // Svelte runes
  '$state', '$derived', '$effect', '$props', '$bindable', '$inspect', '$host',
]);

/** Every name the file declares, anywhere, at any depth. */
function collectDeclared(node: any, out: Set<string>) {
  const pattern = (p: any) => {
    if (!p || typeof p !== 'object') return;
    switch (p.type) {
      case 'Identifier':
        out.add(p.name);
        break;
      case 'ObjectPattern':
        for (const prop of p.properties) pattern(prop.value ?? prop.argument);
        break;
      case 'ArrayPattern':
        for (const el of p.elements) pattern(el);
        break;
      case 'AssignmentPattern':
        pattern(p.left);
        break;
      case 'RestElement':
        pattern(p.argument);
        break;
    }
  };

  const visit = (n: any) => {
    if (!n || typeof n !== 'object') return;
    if (Array.isArray(n)) {
      n.forEach(visit);
      return;
    }
    switch (n.type) {
      case 'VariableDeclarator':
        pattern(n.id);
        break;
      case 'FunctionDeclaration':
      case 'FunctionExpression':
      case 'ArrowFunctionExpression':
      case 'ClassDeclaration':
      case 'ClassExpression':
        if (n.id) out.add(n.id.name);
        (n.params ?? []).forEach(pattern);
        break;
      case 'CatchClause':
        pattern(n.param);
        break;
      case 'ImportDeclaration':
        for (const spec of n.specifiers) out.add(spec.local.name);
        break;
      case 'LabeledStatement':
        out.add(n.label.name);
        break;
    }
    for (const key of Object.keys(n)) {
      if (key === 'type' || key === 'start' || key === 'end' || key === 'loc') continue;
      visit(n[key]);
    }
  };
  visit(node);
}

/** Every name the file reads as a value (not property names, not object keys). */
function collectReferenced(node: any, out: Set<string>) {
  const visit = (n: any, parent: any, key: string) => {
    if (!n || typeof n !== 'object') return;
    if (Array.isArray(n)) {
      n.forEach((c) => visit(c, parent, key));
      return;
    }
    if (n.type === 'Identifier') {
      const isProperty = parent?.type === 'MemberExpression' && key === 'property' && !parent.computed;
      const isKey = parent?.type === 'Property' && key === 'key' && !parent.computed;
      const isLabel = parent?.type === 'LabeledStatement' || parent?.type === 'BreakStatement' ||
        parent?.type === 'ContinueStatement';
      if (!isProperty && !isKey && !isLabel) out.add(n.name);
      return;
    }
    for (const k of Object.keys(n)) {
      if (k === 'type' || k === 'start' || k === 'end' || k === 'loc') continue;
      visit(n[k], n, k);
    }
  };
  visit(node, null, '');
}

/**
 * The component's script plus its markup expressions, as one JS module. Markup is
 * included because a deleted `const` is just as broken when a template reads it.
 */
function componentSource(svelte: string): string {
  const script = /<script[^>]*>([\s\S]*?)<\/script>/.exec(svelte);
  const body = script ? script[1] : '';
  const markup = svelte.slice(script ? script.index + script[0].length : 0);
  // Markup expressions: {...} outside of style blocks, as throwaway statements.
  const withoutStyle = markup.replace(/<style[\s\S]*?<\/style>/g, '');
  const exprs: string[] = [];
  const re = /\{([^{}]*(?:\{[^{}]*\}[^{}]*)*)\}/g;
  let m: RegExpExecArray | null;
  while ((m = re.exec(withoutStyle))) {
    const raw = m[1].trim();
    // Skip block openers/closers and directives — they aren't plain expressions.
    if (!raw || /^[#/:@]/.test(raw)) continue;
    exprs.push(`(${raw.replace(/^const\s+/, 'var ')});`);
  }
  return `${body}\nfunction __markup__() {\n${exprs.join('\n')}\n}\n`;
}

/**
 * Names that Svelte's own markup blocks introduce — each/await/snippet bindings
 * and `{@const}`. They're declared by the template, not by any JavaScript the
 * script block contains, so they have to be gathered from the markup directly.
 */
function markupBindings(svelte: string, out: Set<string>) {
  const ids = (pattern: string) => {
    for (const m of pattern.matchAll(/[A-Za-z_$][\w$]*/g)) out.add(m[0]);
  };
  for (const m of svelte.matchAll(/\{#each\s+[\s\S]*?\s+as\s+([^}(]+)/g)) ids(m[1]);
  for (const m of svelte.matchAll(/\{@const\s+([A-Za-z_$][\w$]*)/g)) out.add(m[1]);
  for (const m of svelte.matchAll(/\{[:#]then\s+([A-Za-z_$][\w$]*)/g)) out.add(m[1]);
  for (const m of svelte.matchAll(/\{[:#]catch\s+([A-Za-z_$][\w$]*)/g)) out.add(m[1]);
  for (const m of svelte.matchAll(/\{#snippet\s+([A-Za-z_$][\w$]*)\s*\(([^)]*)\)/g)) {
    out.add(m[1]);
    ids(m[2]);
  }
}

const files = readdirSync(SRC)
  .filter((f) => f.endsWith('.svelte'))
  .map((f) => join(SRC, f));

for (const file of files) {
  test(`${file.split('/').pop()} references only names it can resolve`, () => {
    const svelte = readFileSync(file, 'utf8');
    const source = componentSource(svelte);
    // Strip TypeScript so acorn can parse it; type-only names vanish with it.
    const js = transformSync(source, { loader: 'ts', format: 'esm' }).code;
    const ast = parse(js, { ecmaVersion: 'latest', sourceType: 'module' });

    const declared = new Set<string>();
    const referenced = new Set<string>();
    collectDeclared(ast, declared);
    markupBindings(svelte, declared);
    collectReferenced(ast, referenced);

    const missing = [...referenced].filter((n) => !declared.has(n) && !GLOBALS.has(n)).sort();
    assert.deepEqual(missing, [], `undeclared identifiers in ${file}: ${missing.join(', ')}`);
  });
}
