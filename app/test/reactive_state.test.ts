// Catch state the markup reads but that never triggers a re-render.
//
// In Svelte 5 a plain `let` is not reactive. Assigning to one updates the
// variable, and any `{#if}`, attribute or text that reads it keeps whatever value
// it had at mount — forever. Nothing errors, nothing warns, and `vite build` is
// perfectly happy; the symptom is a control that never appears or a readout stuck
// on its initial value, which reads as "the feature is broken" rather than as a
// missing rune. That is how `physics` came to gate a button that could never
// render: it is set while the scene loads, long after the first paint.
//
// Only flags the combination that is actually wrong — declared `let` without a
// rune, reassigned somewhere, AND read by the markup. A `let` written *from* the
// markup (an event handler doing `cancel = true`) is fine and is not reported.

import { test } from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync, readdirSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { dirname, join } from 'node:path';
import { parse } from 'acorn';
import { transformSync } from 'esbuild';

const here = dirname(fileURLToPath(import.meta.url));
const SRC = join(here, '..', 'src');

const RUNES = new Set(['$state', '$derived', '$props', '$bindable']);

/** Is this declarator initialised by a rune — `$state(…)`, `$derived.by(…)`, …? */
function isRuneInit(init: any): boolean {
  if (!init || init.type !== 'CallExpression') return false;
  const c = init.callee;
  if (c.type === 'Identifier') return RUNES.has(c.name);
  if (c.type === 'MemberExpression' && c.object.type === 'Identifier') return RUNES.has(c.object.name);
  return false;
}

/** Top-level `let` names declared without a rune. */
function plainTopLevelLets(program: any): Set<string> {
  const out = new Set<string>();
  for (const node of program.body) {
    if (node.type !== 'VariableDeclaration' || node.kind !== 'let') continue;
    for (const d of node.declarations) {
      if (d.id.type !== 'Identifier') continue; // destructuring: leave it alone
      if (isRuneInit(d.init)) continue;
      out.add(d.id.name);
    }
  }
  return out;
}

/** Names assigned to anywhere in the tree (`x = …`, `x += …`, `x++`). */
function assignedNames(node: any, out: Set<string>) {
  const visit = (n: any) => {
    if (!n || typeof n !== 'object') return;
    if (Array.isArray(n)) return n.forEach(visit);
    if (n.type === 'AssignmentExpression' && n.left?.type === 'Identifier') out.add(n.left.name);
    if (n.type === 'UpdateExpression' && n.argument?.type === 'Identifier') out.add(n.argument.name);
    for (const k of Object.keys(n)) {
      if (k === 'type' || k === 'start' || k === 'end' || k === 'loc') continue;
      visit(n[k]);
    }
  };
  visit(node);
}

/**
 * Identifiers an expression *reads*. Assignment targets don't count: an event
 * handler writing to a variable needs no reactivity, and flagging those would
 * bury the real cases.
 */
function readNames(node: any, out: Set<string>) {
  const local = new Set<string>();
  const declare = (p: any) => {
    if (!p || typeof p !== 'object') return;
    if (p.type === 'Identifier') local.add(p.name);
    else if (p.type === 'ObjectPattern') p.properties.forEach((q: any) => declare(q.value ?? q.argument));
    else if (p.type === 'ArrayPattern') p.elements.forEach(declare);
    else if (p.type === 'AssignmentPattern') declare(p.left);
    else if (p.type === 'RestElement') declare(p.argument);
  };
  const found = new Set<string>();
  const visit = (n: any, parent: any, key: string) => {
    if (!n || typeof n !== 'object') return;
    if (Array.isArray(n)) return n.forEach((c) => visit(c, parent, key));
    if (n.type === 'ArrowFunctionExpression' || n.type === 'FunctionExpression') {
      (n.params ?? []).forEach(declare);
    }
    if (n.type === 'VariableDeclarator') declare(n.id);
    if (n.type === 'Identifier') {
      const isProperty = parent?.type === 'MemberExpression' && key === 'property' && !parent.computed;
      const isKey = parent?.type === 'Property' && key === 'key' && !parent.computed;
      const isWriteTarget =
        (parent?.type === 'AssignmentExpression' && key === 'left') ||
        (parent?.type === 'UpdateExpression' && key === 'argument');
      if (!isProperty && !isKey && !isWriteTarget) found.add(n.name);
      return;
    }
    for (const k of Object.keys(n)) {
      if (k === 'type' || k === 'start' || k === 'end' || k === 'loc') continue;
      visit(n[k], n, k);
    }
  };
  visit(node, null, '');
  for (const n of found) if (!local.has(n)) out.add(n);
}

/** Everything the markup reads, as identifier names. */
function markupReads(svelte: string, scriptEnd: number): Set<string> {
  const out = new Set<string>();
  const markup = svelte
    .slice(scriptEnd)
    .replace(/<style[\s\S]*?<\/style>/g, '')
    // `bind:this={el}` is a DOM ref being filled in, not reactive state.
    .replace(/bind:this=\{[^}]*\}/g, '');
  const re = /\{([^{}]*(?:\{[^{}]*\}[^{}]*)*)\}/g;
  let m: RegExpExecArray | null;
  while ((m = re.exec(markup))) {
    const expr = blockExpression(m[1].trim());
    if (!expr) continue;
    try {
      const js = transformSync(`(${expr});`, { loader: 'ts', format: 'esm' }).code;
      readNames(parse(js, { ecmaVersion: 'latest', sourceType: 'module' }), out);
    } catch {
      /* not a standalone expression — the undefined-refs test covers parse errors */
    }
  }
  return out;
}

/**
 * The JavaScript inside a `{…}`, including the block openers.
 *
 * `{#if x}` is the single most likely place for a stale read to hide — a control
 * that never renders — so skipping block tags as "not expressions" would miss the
 * exact bug this test exists for. Returns null for tags that carry no expression.
 */
function blockExpression(raw: string): string | null {
  if (!raw) return null;
  if (/^[/]/.test(raw)) return null; // {/if}, {/each}
  if (!/^[#:@]/.test(raw)) return raw; // a plain expression
  const tag = /^[#:@]([a-z]+)(?:\s+([\s\S]*))?$/.exec(raw);
  if (!tag) return null;
  const [, name, rest = ''] = tag;
  switch (name) {
    case 'if':
    case 'key':
      return rest || null;
    case 'else':
      // `{:else}` has nothing; `{:else if cond}` does.
      return /^if\s+([\s\S]+)$/.exec(rest)?.[1] ?? null;
    case 'each': {
      // `{#each list as item, i (key)}` — the iterated expression is up to ` as `.
      const cut = rest.search(/\s+as\s+/);
      return (cut > 0 ? rest.slice(0, cut) : rest) || null;
    }
    case 'await': {
      // `{#await promise}` / `{#await promise then value}`.
      const cut = rest.search(/\s+then\s+|\s+catch\s+/);
      return (cut > 0 ? rest.slice(0, cut) : rest) || null;
    }
    case 'const': {
      // `{@const name = expr}` — only the right-hand side is a read.
      const eq = rest.indexOf('=');
      return eq > 0 ? rest.slice(eq + 1).trim() || null : null;
    }
    case 'html':
    case 'render':
      return rest || null;
    default:
      return null; // {:then x}, {:catch e}, {#snippet f(a)}, {@debug} — bindings, not reads
  }
}

const files = readdirSync(SRC)
  .filter((f) => f.endsWith('.svelte'))
  .map((f) => join(SRC, f));

for (const file of files) {
  test(`${file.split('/').pop()} makes the state its markup reads reactive`, () => {
    const svelte = readFileSync(file, 'utf8');
    const m = /<script[^>]*>([\s\S]*?)<\/script>/.exec(svelte);
    if (!m) return;
    const js = transformSync(m[1], { loader: 'ts', format: 'esm' }).code;
    const ast = parse(js, { ecmaVersion: 'latest', sourceType: 'module' }) as any;

    const plain = plainTopLevelLets(ast);
    const assigned = new Set<string>();
    assignedNames(ast, assigned);
    const reads = markupReads(svelte, m.index + m[0].length);

    const stale = [...plain].filter((n) => assigned.has(n) && reads.has(n)).sort();
    assert.deepEqual(
      stale,
      [],
      `read by the markup but not reactive (needs $state) in ${file}: ${stale.join(', ')}`,
    );
  });
}
