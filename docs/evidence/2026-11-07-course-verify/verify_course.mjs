#!/usr/bin/env node
/**
 * verify_course.mjs — independent "full lap + component coverage" verifier.
 *
 * What it does (nothing is modified outside its own output directory):
 *   1. spawns the simulator's own replay harness for scheme 1 / scheme 2
 *        cwd = <simulator dir>,  `node tests/replay.mjs <scheme> <outdir>`
 *      (that harness reads the *real* firmware route table
 *        <firmware>/src/route_table.c and writes scheme<n>.csv / .json)
 *   2. parses the CSV control-cycle log (ms,index,state,x,y,h,...)
 *   3. reports
 *        - whether the lap finished (final state ROUTE_FINISHED), total time,
 *          final index, failure code + name, final pose vs start point;
 *        - per route step: entry / exit time and pose, junction count, recovery
 *          and warning counters, duration, skipped / re-entered indices;
 *        - component coverage: every one of the 27 printed items in
 *          <sim>/src/track/map_data.ts (CIRCUIT_COMPONENTS) is checked against
 *          the driven polyline (axle centre). A component counts as passed when
 *          the shortest trajectory-segment distance to its centre is <= --pass-cm
 *          (default 6 cm). Missing / near-miss / repeated passes are listed.
 *          The 24 cm sensor-bar-centre trajectory is evaluated as well and
 *          reported, because that is what the grayscale bar physically samples.
 *        - on failure: the last 20 control cycles, plus per-step diagnostics.
 *   4. writes report.txt / report.json into --out and exits non-zero unless
 *      every scheme finished the lap with all components covered
 *      (`--allow-fail` forces exit 0 for archiving a known-bad baseline).
 *
 * Usage (from the firmware repo root, /Users/.../car_stm32):
 *   node docs/evidence/2026-11-07-course-verify/verify_course.mjs
 *   node docs/evidence/2026-11-07-course-verify/verify_course.mjs --out /tmp/cv --pass-cm 6
 *   node docs/evidence/2026-11-07-course-verify/verify_course.mjs --schemes 1
 *   node docs/evidence/2026-11-07-course-verify/verify_course.mjs --reuse --out /tmp/cv
 *     (--reuse parses an existing scheme<n>.csv without replaying again)
 *
 * NOTE: this script deliberately delegates driving to the simulator's replay
 * harness instead of re-implementing the plant, so the numbers stay whatever
 * the current simulator work tree actually produces.
 */

import { spawn } from 'node:child_process';
import { createRequire } from 'node:module';
import { pathToFileURL } from 'node:url';
import {
  existsSync,
  mkdirSync,
  readFileSync,
  statSync,
  writeFileSync,
} from 'node:fs';
import path from 'node:path';

/* ------------------------------------------------------------------ config */

const DEFAULT_SIM = process.env.STM32_SIM_DIR || '/Users/zhendong/Documents/stm32car_simulator';
const DEFAULT_ROUTE_TABLE =
  process.env.STM32_ROUTE_TABLE || '/Users/zhendong/Documents/PlatformIO/Projects/car_stm32/src/route_table.c';
const DEFAULT_OUT = '/tmp/course-verify';
const DEFAULT_SCHEMES = [1, 2];
const DEFAULT_PASS_CM = 6.0;
const DEFAULT_REPLAY_TIMEOUT_MS = 300000;
const EXPECTED_COMPONENTS = 27;
const START_POINT_FALLBACK = { x: 0, y: 110 }; // battery P on the left border
const STEPS_PREVIEW = 4; // control cycles kept as "approach" context
const FAIL_TAIL_ROWS = 20;

/* --------------------------------------------------------------- arguments */

function parseArgs(argv) {
  const opts = {
    sim: DEFAULT_SIM,
    routeTable: DEFAULT_ROUTE_TABLE,
    out: DEFAULT_OUT,
    schemes: DEFAULT_SCHEMES.slice(),
    passCm: DEFAULT_PASS_CM,
    timeoutMs: DEFAULT_REPLAY_TIMEOUT_MS,
    allowFail: false,
    reuse: false,
    quiet: false,
  };
  for (let i = 2; i < argv.length; ++i) {
    const a = argv[i];
    const next = () => {
      const v = argv[++i];
      if (v === undefined) throw new Error(`missing value after ${a}`);
      return v;
    };
    if (a === '--sim') opts.sim = path.resolve(next());
    else if (a === '--route-table') opts.routeTable = path.resolve(next());
    else if (a === '--out') opts.out = path.resolve(next());
    else if (a === '--schemes')
      opts.schemes = next()
        .split(',')
        .map((s) => Number(s.trim()))
        .filter((n) => Number.isInteger(n));
    else if (a === '--pass-cm') opts.passCm = Number(next());
    else if (a === '--timeout-ms') opts.timeoutMs = Number(next());
    else if (a === '--allow-fail') opts.allowFail = true;
    else if (a === '--reuse') opts.reuse = true;
    else if (a === '--quiet') opts.quiet = true;
    else if (a === '-h' || a === '--help') {
      printHelp();
      process.exit(0);
    } else throw new Error(`unknown argument: ${a}`);
  }
  if (!opts.schemes.length) throw new Error('no valid --schemes given');
  if (!Number.isFinite(opts.passCm) || opts.passCm <= 0) throw new Error('--pass-cm must be > 0');
  if (!Number.isFinite(opts.timeoutMs) || opts.timeoutMs <= 0) throw new Error('--timeout-ms must be > 0');
  return opts;
}

function printHelp() {
  console.log(`verify_course.mjs — replay + full-lap/component-coverage verifier

  --sim DIR          simulator checkout (default ${DEFAULT_SIM})
  --route-table FILE firmware route table (default ${DEFAULT_ROUTE_TABLE})
  --out DIR          output directory for replay CSVs and reports (default ${DEFAULT_OUT})
  --schemes LIST     comma separated scheme ids (default 1,2)
  --pass-cm N        axle-to-component-centre radius counted as "passed" (default ${DEFAULT_PASS_CM})
  --timeout-ms N     per-replay wall clock limit (default ${DEFAULT_REPLAY_TIMEOUT_MS})
  --reuse            parse existing scheme<n>.csv in --out, do not replay
  --allow-fail       always exit 0 (archive a known-bad baseline)
  --quiet            only print the final verdict block
`);
}

/* ------------------------------------------------ simulator meta (read only) */

/**
 * Load CIRCUIT_COMPONENTS / START_POINT / enums straight from simulator
 * sources through the same vite SSR loader the replay harness uses, so the
 * verifier never keeps its own copy of the track map.
 */
async function loadSimulatorMeta(simDir) {
  const pkg = path.join(simDir, 'package.json');
  if (!existsSync(pkg)) throw new Error(`not a simulator checkout (no package.json): ${simDir}`);
  const req = createRequire(pkg);
  let vitePath;
  try {
    vitePath = req.resolve('vite');
  } catch {
    throw new Error(`cannot resolve vite from ${simDir}/node_modules — run npm install there first`);
  }
  const vite = await import(pathToFileURL(vitePath).href);
  const server = await vite.createServer({
    root: simDir,
    server: { middlewareMode: true },
    logLevel: 'error',
    optimizeDeps: { noDiscovery: true, include: [] },
  });
  try {
    const map = await server.ssrLoadModule('/src/track/map_data.ts');
    const types = await server.ssrLoadModule('/src/sim/types.ts');
    const components = (map.CIRCUIT_COMPONENTS || []).map((c) => ({
      id: c.id,
      type: c.type,
      x: Number(c.x),
      y: Number(c.y),
      label: c.label,
      elementNumber: c.elementNumber,
    }));
    return {
      components,
      startPoint: map.START_POINT ? { x: map.START_POINT.x, y: map.START_POINT.y } : START_POINT_FALLBACK,
      hexCenter: map.HEXAGON_CENTER || null,
      enumName: (enumObj, value) => {
        if (!enumObj) return String(value);
        for (const [k, v] of Object.entries(enumObj)) if (v === value) return k;
        return String(value);
      },
      enums: {
        RouteFailure: types.RouteFailure || null,
        RouteState: types.RouteState || null,
        LineState: types.LineState || null,
        Action: types.Action || null,
      },
    };
  } finally {
    await server.close();
  }
}

/* ------------------------------------------------------------ replay runner */

function runReplay(opts, scheme) {
  return new Promise((resolve) => {
    const args = ['tests/replay.mjs', String(scheme), opts.out];
    const started = Date.now();
    const child = spawn(process.execPath, args, {
      cwd: opts.sim,
      env: { ...process.env, NO_COLOR: '1' },
      stdio: ['ignore', 'pipe', 'pipe'],
    });
    let stdout = '';
    let stderr = '';
    let timedOut = false;
    const timer = setTimeout(() => {
      timedOut = true;
      child.kill('SIGKILL');
    }, opts.timeoutMs);
    child.stdout.on('data', (d) => (stdout += d));
    child.stderr.on('data', (d) => (stderr += d));
    child.on('error', (err) => {
      clearTimeout(timer);
      resolve({ scheme, ok: false, code: -1, signal: null, stdout, stderr: stderr + String(err), timedOut, wallMs: Date.now() - started });
    });
    child.on('close', (code, signal) => {
      clearTimeout(timer);
      resolve({ scheme, ok: code === 0 && !timedOut, code, signal, stdout, stderr, timedOut, wallMs: Date.now() - started });
    });
  });
}

/* ---------------------------------------------------------------- CSV parse */

const NUMERIC = new Set([
  'ms', 'index', 'count', 'x', 'y', 'h', 'd', 'mask', 'pos', 'width', 'regions', 'center',
  'reject', 'smooth', 'lt', 'rt', 'lv', 'rv', 'lm', 'rm', 'lp', 'rp', 'entry', 'actionStart',
  'phase', 'recoveries', 'shortForward', 'shortLateral', 'warning', 'failure',
]);

function parseCsv(file) {
  const text = readFileSync(file, 'utf8').trim();
  const lines = text.split('\n');
  if (lines.length < 2) throw new Error(`CSV too short: ${file}`);
  const header = lines[0].split(',').map((s) => s.trim());
  for (const required of ['ms', 'index', 'state', 'x', 'y', 'h']) {
    if (!header.includes(required)) throw new Error(`CSV ${file} misses required column "${required}" (has: ${header.join(',')})`);
  }
  const rows = new Array(lines.length - 1);
  for (let li = 1; li < lines.length; ++li) {
    const cells = lines[li].split(',');
    const row = {};
    for (let ci = 0; ci < header.length; ++ci) {
      const key = header[ci];
      const raw = cells[ci];
      row[key] = NUMERIC.has(key) ? Number(raw) : String(raw);
    }
    rows[li - 1] = row;
  }
  return { header, rows };
}

function readJsonMaybe(file) {
  try {
    return JSON.parse(readFileSync(file, 'utf8'));
  } catch {
    return null;
  }
}

/* ----------------------------------------------------- geometry primitives */

function segmentPointDistance(px, py, ax, ay, bx, by) {
  const dx = bx - ax;
  const dy = by - ay;
  const lenSq = dx * dx + dy * dy;
  if (lenSq === 0) return Math.hypot(px - ax, py - ay);
  let t = ((px - ax) * dx + (py - ay) * dy) / lenSq;
  t = Math.max(0, Math.min(1, t));
  return Math.hypot(px - (ax + t * dx), py - (ay + t * dy));
}

/**
 * Distance metrics of one point against a sampled trajectory.
 * Returns min point distance, min segment distance, number of separate pass
 * episodes (a pass episode = a maximal run of consecutive segments inside the
 * radius) and where the closest approach happened.
 */
function trackMetrics(points, tx, ty, passCm) {
  let minPoint = Infinity;
  let minPointAt = 0;
  let minSeg = Infinity;
  let minSegAt = 0;
  let passes = 0;
  let insideRun = false;
  for (let i = 0; i < points.length; ++i) {
    const p = points[i];
    const d = Math.hypot(p.x - tx, p.y - ty);
    if (d < minPoint) {
      minPoint = d;
      minPointAt = i;
    }
    let dSeg = d;
    if (i > 0) {
      const q = points[i - 1];
      dSeg = segmentPointDistance(tx, ty, q.x, q.y, p.x, p.y);
    }
    if (dSeg < minSeg) {
      minSeg = dSeg;
      minSegAt = i;
    }
    const inside = dSeg <= passCm;
    if (inside && !insideRun) {
      ++passes;
      insideRun = true;
    } else if (!inside) {
      insideRun = false;
    }
  }
  return { minPoint, minPointAt, minSeg, minSegAt, passes };
}

/* ------------------------------------------------------------ step analysis */

function wrapAngle(a) {
  return Math.atan2(Math.sin(a), Math.cos(a));
}

/**
 * Per-run diagnostics: where the car actually started pivoting (heading departs
 * from the heading it entered the step with by > 0.15 rad) and the last sample
 * where the grayscale bar still saw any line (mask != 0).  Both are the two
 * numbers that matter when a step dies of EXIT_LOST.
 */
function runDiagnostics(list) {
  const entryHeading = list[0].h;
  let pivot = null;
  let lastMask = null;
  for (const row of list) {
    if (!pivot && Number.isFinite(row.h) && Math.abs(wrapAngle(row.h - entryHeading)) > 0.15) pivot = row;
    if (Number.isFinite(row.mask) && row.mask !== 0) lastMask = row;
  }
  return { pivot, lastMask };
}

/**
 * Build a per-route-step table from the CSV alone (index column).
 * A "step" spans from the first cycle with route index i to the last cycle
 * before the index changes (or the end of the log).
 */
function analyseSteps(rows) {
  const runs = [];
  for (const row of rows) {
    const last = runs[runs.length - 1];
    if (!last || last.index !== row.index) {
      runs.push({ index: row.index, firstRow: row, lastRow: row, rows: 1, list: [row] });
    } else {
      last.lastRow = row;
      last.rows += 1;
      last.list.push(row);
    }
  }
  const byIndex = new Map();
  for (const run of runs) {
    const entry = byIndex.get(run.index);
    if (!entry) byIndex.set(run.index, { index: run.index, visits: [run] });
    else {
      entry.visits.push(run);
      entry.reentered = true;
    }
  }
  const steps = [...byIndex.values()].sort((a, b) => a.index - b.index);
  const visitedOrder = runs.map((r) => r.index);
  const skipped = [];
  for (let i = 1; i < visitedOrder.length; ++i) {
    const gap = visitedOrder[i] - visitedOrder[i - 1];
    if (gap > 1) skipped.push({ from: visitedOrder[i - 1], to: visitedOrder[i], missing: gap - 1 });
  }
  return { runs, steps, visitedOrder, skipped };
}

function failureNames(meta) {
  const e = meta.enums.RouteFailure;
  const names = {};
  if (e) for (const [k, v] of Object.entries(e)) names[v] = k;
  return names;
}

function failureLabel(names, code) {
  if (!code) return 'ROUTE_FAILURE_NONE';
  return names[code] || `UNKNOWN(${code})`;
}

/* -------------------------------------------------------------- evaluation */

function evaluateScheme(scheme, csvFile, jsonFile, meta, opts) {
  const { rows } = parseCsv(csvFile);
  const json = jsonFile ? readJsonMaybe(jsonFile) : null;
  const last = rows[rows.length - 1];
  const first = rows[0];
  const failNames = failureNames(meta);
  const { runs, steps, visitedOrder, skipped } = analyseSteps(rows);

  const actionNames = meta.enums.Action ? Object.fromEntries(Object.entries(meta.enums.Action)) : null;
  const routeSteps = json && Array.isArray(json.steps) ? json.steps : [];
  const stepComments = routeSteps.map((s) => s.comment || '');
  const stepActions = routeSteps.map((s) => (actionNames ? actionNames[s.action] ?? String(s.action) : String(s.action)));

  const finished = last.state === 'ROUTE_FINISHED';
  const stopped = last.line === 'LINE_STOPPED' || last.state === 'ROUTE_FAILED';
  const failureCode = Number(last.failure) || 0;

  const start = meta.startPoint;
  const finalToStart = Math.hypot(last.x - start.x, last.y - start.y);

  // trajectories (axle centre and the sensor-bar centre 24 cm ahead)
  const axle = rows.map((r) => ({ x: r.x, y: r.y, ms: r.ms, index: r.index, state: r.state }));
  const GRAY_FORWARD_CM = 24.0;
  const sensor = rows.map((r) => ({
    x: r.x + GRAY_FORWARD_CM * Math.cos(r.h),
    y: r.y + GRAY_FORWARD_CM * Math.sin(r.h),
    ms: r.ms,
    index: r.index,
    state: r.state,
  }));

  const coverage = meta.components.map((c) => {
    const axleM = trackMetrics(axle, c.x, c.y, opts.passCm);
    const sensorM = trackMetrics(sensor, c.x, c.y, opts.passCm);
    const covered = axleM.minSeg <= opts.passCm;
    let verdict;
    if (covered) verdict = axleM.passes >= 2 ? `PASS x${axleM.passes}` : 'PASS';
    else if (axleM.minSeg <= opts.passCm * 2) verdict = 'NEAR-MISS';
    else verdict = 'MISS';
    const at = axle[axleM.minSegAt] || axle[axle.length - 1];
    return {
      ...c,
      axleMinPointCm: axleM.minPoint,
      axleMinSegCm: axleM.minSeg,
      passes: axleM.passes,
      sensorMinPointCm: sensorM.minPoint,
      sensorMinSegCm: sensorM.minSeg,
      sensorPasses: sensorM.passes,
      covered,
      verdict,
      closestMs: at ? at.ms : null,
      closestIndex: at ? at.index : null,
      closestState: at ? at.state : null,
      closestPose: at ? { x: at.x, y: at.y } : null,
    };
  });

  const coveredList = coverage.filter((c) => c.covered);
  const missing = coverage.filter((c) => !c.covered);
  const repeated = coverage.filter((c) => c.passes >= 2);
  const sensorCoveredCount = coverage.filter((c) => c.sensorPasses >= 1).length;
  const componentCountOk = meta.components.length === EXPECTED_COMPONENTS;

  const stepRows = steps.map((s) => {
    const firstVisit = s.visits[0];
    const lastVisit = s.visits[s.visits.length - 1];
    const entryRow = firstVisit.firstRow;
    const exitRow = lastVisit.lastRow;
    const diag = runDiagnostics(firstVisit.list);
    return {
      index: s.index,
      visits: s.visits.length,
      reentered: !!s.reentered,
      entryMs: entryRow.ms,
      exitMs: exitRow.ms,
      durationMs: exitRow.ms - entryRow.ms,
      entryPose: { x: entryRow.x, y: entryRow.y, h: entryRow.h },
      exitPose: { x: exitRow.x, y: exitRow.y, h: exitRow.h },
      entryState: entryRow.state,
      exitState: exitRow.state,
      junctionCount: exitRow.count,
      maxWarning: Math.max(...s.visits.map((v) => v.lastRow.warning || 0)),
      maxRecoveries: Math.max(...s.visits.map((v) => v.lastRow.recoveries || 0)),
      action: stepActions[s.index] || null,
      comment: stepComments[s.index] || null,
      cycles: s.visits.reduce((n, v) => n + v.rows, 0),
      pivotMs: diag.pivot ? diag.pivot.ms : null,
      pivotPose: diag.pivot ? { x: diag.pivot.x, y: diag.pivot.y } : null,
      lastMaskMs: diag.lastMask ? diag.lastMask.ms : null,
      lastMaskPose: diag.lastMask ? { x: diag.lastMask.x, y: diag.lastMask.y } : null,
      lastMaskValue: diag.lastMask ? diag.lastMask.mask : null,
      lastMaskState: diag.lastMask ? diag.lastMask.state : null,
    };
  });

  return {
    scheme,
    csvFile,
    jsonFile,
    cycles: rows.length,
    simTimeMs: last.ms,
    simTimeS: last.ms / 1000,
    firstState: first.state,
    finalState: last.state,
    finalLine: last.line,
    finished,
    stopped,
    failureCode,
    failureName: failureLabel(failNames, failureCode),
    finalIndex: last.index,
    finalPose: { x: last.x, y: last.y, h: last.h, d: last.d },
    finalToStartCm: finalToStart,
    routeStepCount: routeSteps.length || null,
    steps: stepRows,
    visitedOrder,
    skipped,
    componentCount: meta.components.length,
    componentCountOk,
    coverage,
    coveredCount: coveredList.length,
    sensorCoveredCount,
    missing,
    repeated,
    failureTail: rows.slice(-FAIL_TAIL_ROWS),
    approachTail: rows.slice(0, STEPS_PREVIEW),
  };
}

/* ---------------------------------------------------------------- reporting */

function fmt(n, digits = 2) {
  if (!Number.isFinite(n)) return 'n/a';
  return Number(n).toFixed(digits);
}

function pad(s, width) {
  s = String(s);
  return s.length >= width ? s : s + ' '.repeat(width - s.length);
}

function padStart(s, width) {
  s = String(s);
  return s.length >= width ? s : ' '.repeat(width - s.length) + s;
}

/* Scheme 2 is the no-dashed-route reference figure: it intentionally skips the
 * dashed x=55 section, so the resistor printed on that dashed wire (R26) is not
 * on the route.  Every other printed item must still be covered. */
const DASHED_ONLY_COMPONENT = 'r26';
function expectedCoveredCount(ev) {
  const dashedOnly = ev.coverage.filter((c) => c.id === DASHED_ONLY_COMPONENT).length;
  return ev.scheme === 2 && dashedOnly > 0 ? ev.componentCount - dashedOnly : ev.componentCount;
}
function schemeVerdict(ev) {
  return ev.finished && ev.coveredCount === expectedCoveredCount(ev) &&
    ev.componentCountOk && ev.failureCode === 0;
}

function renderReport(opts, meta, results, runInfo) {
  const out = [];
  const push = (s = '') => out.push(s);
  const now = new Date().toISOString();
  push('================================================================================');
  push(' verify_course — 整圈完成 + 27 印刷元件覆盖 独立验证');
  push('================================================================================');
  push(`run at            : ${now}`);
  push(`simulator         : ${opts.sim}`);
  push(`route table       : ${opts.routeTable}`);
  push(`output dir        : ${opts.out}`);
  push(`pass radius       : axle-centre <= ${opts.passCm} cm (sensor-bar centre also reported)`);
  push(`expected items    : ${EXPECTED_COMPONENTS} printed components`);
  if (meta.components.length !== EXPECTED_COMPONENTS) {
    push(`!! simulator map lists ${meta.components.length} components, expected ${EXPECTED_COMPONENTS}`);
  }
  push('');

  for (const r of runInfo) {
    push(`--- scheme ${r.scheme} replay ---`);
    if (r.skippedRun) push('   (reused existing CSV, replay not executed)');
    else {
      push(`   command : (cwd ${opts.sim}) node tests/replay.mjs ${r.scheme} ${opts.out}`);
      push(`   exit    : ${r.code}${r.signal ? ` signal=${r.signal}` : ''}${r.timedOut ? ' (TIMEOUT)' : ''}  wall ${r.wallMs} ms`);
      if (r.stderr && r.stderr.trim()) push(`   stderr  : ${r.stderr.trim().split('\n').slice(-5).join(' | ')}`);
      if (!r.ok) push('   !! replay harness failed — see log file next to the CSV');
    }
    push('');
  }

  const failedRuns = runInfo.filter((r) => !r.skippedRun && !r.ok);
  if (failedRuns.length) {
    push('!! replay harness failures (no CSV produced for these schemes)');
    for (const r of failedRuns) push(`   scheme ${r.scheme}: exit ${r.code}${r.timedOut ? ' TIMEOUT' : ''}`);
    push('');
  }

  for (const ev of results) {
    push('================================================================================');
    push(` SCHEME ${ev.scheme}`);
    push('================================================================================');
    push(`verdict            : ${schemeVerdict(ev) ? 'PASS (lap finished + all components covered)' : 'FAIL'}`);
    push(`ROUTE_FINISHED     : ${ev.finished ? 'yes' : 'no'}`);
    push(`sim duration       : ${ev.simTimeS.toFixed(2)} s (${ev.simTimeMs} ms, ${ev.cycles} control cycles)`);
    push(`final state        : ${ev.finalState} / ${ev.finalLine}`);
    push(`final route index  : ${ev.finalIndex}${ev.routeStepCount ? ` of ${ev.routeStepCount - 1}` : ''}`);
    push(`failure            : ${ev.failureCode} ${ev.failureName}`);
    push(`final pose         : x=${fmt(ev.finalPose.x)} y=${fmt(ev.finalPose.y)} h=${fmt(ev.finalPose.h, 3)} d=${fmt(ev.finalPose.d)} cm`);
    push(`final pose vs P    : ${fmt(ev.finalToStartCm)} cm from start point`);
    push(`components covered : ${ev.coveredCount}/${ev.componentCount} (axle centre <= pass radius)${ev.componentCountOk ? '' : '  !! map count != 27'}`);
    push(`                     : ${ev.sensorCoveredCount}/${ev.componentCount} also sampled by the sensor bar centre`);
    if (ev.missing.length) push(`missing            : ${ev.missing.map((c) => `${c.id}(${c.label})@${fmt(c.axleMinSegCm, 1)}cm`).join(', ')}`);
    if (ev.repeated.length) push(`repeated passes    : ${ev.repeated.map((c) => `${c.id} x${c.passes}`).join(', ')}`);
    if (ev.skipped.length) push(`skipped indices    : ${ev.skipped.map((s) => `${s.from}->${s.to} (missing ${s.missing})`).join(', ')}`);
    const reentered = ev.steps.filter((s) => s.reentered).map((s) => s.index);
    if (reentered.length) push(`re-entered indices : ${reentered.join(', ')}`);
    push('');

    push('-- per route step (from CSV index column) ------------------------------------');
    push(
      pad('#', 4) +
        ' | ' +
        pad('action', 12) +
        ' | ' +
        padStart('entries', 7) +
        ' | ' +
        padStart('entry_ms', 9) +
        ' | ' +
        padStart('exit_ms', 9) +
        ' | ' +
        padStart('dur_ms', 7) +
        ' | ' +
        padStart('junctions', 9) +
        ' | ' +
        padStart('recov', 5) +
        ' | ' +
        padStart('warn', 4) +
        ' | ' +
        pad('exit state', 15) +
        ' | entry pose -> exit pose',
    );
    for (const s of ev.steps) {
      push(
        pad(s.index, 4) +
          ' | ' +
          pad(s.action || '-', 12) +
          ' | ' +
          padStart(s.visits, 7) +
          ' | ' +
          padStart(s.entryMs, 9) +
          ' | ' +
          padStart(s.exitMs, 9) +
          ' | ' +
          padStart(s.durationMs, 7) +
          ' | ' +
          padStart(s.junctionCount, 9) +
          ' | ' +
          padStart(s.maxRecoveries, 5) +
          ' | ' +
          padStart(s.maxWarning, 4) +
          ' | ' +
          pad(s.exitState, 15) +
          ' | ' +
          `(${fmt(s.entryPose.x, 1)},${fmt(s.entryPose.y, 1)}) -> (${fmt(s.exitPose.x, 1)},${fmt(s.exitPose.y, 1)})`,
      );
    }
    if (!ev.steps.length) push('(no steps recorded)');
    push('');

    push('-- step diagnostics (turn onset = |heading - entry heading| > 0.15 rad, last mask != 0) --');
    push(
      pad('#', 4) +
        ' | ' +
        padStart('pivot_ms', 9) +
        ' | ' +
        pad('pivot pose', 17) +
        ' | ' +
        padStart('lastMask_ms', 11) +
        ' | ' +
        pad('lastMask pose', 17) +
        ' | mask | state at lastMask',
    );
    for (const s of ev.steps) {
      const tag = s.index === ev.finalIndex && !ev.finished ? ' *failed step*' : '';
      push(
        pad(s.index, 4) +
          ' | ' +
          padStart(s.pivotMs === null ? '-' : s.pivotMs, 9) +
          ' | ' +
          pad(s.pivotPose ? `(${fmt(s.pivotPose.x, 1)},${fmt(s.pivotPose.y, 1)})` : '-', 17) +
          ' | ' +
          padStart(s.lastMaskMs === null ? '-' : s.lastMaskMs, 11) +
          ' | ' +
          pad(s.lastMaskPose ? `(${fmt(s.lastMaskPose.x, 1)},${fmt(s.lastMaskPose.y, 1)})` : '-', 17) +
          ' | ' +
          padStart(s.lastMaskValue === null ? '-' : s.lastMaskValue, 4) +
          ' | ' +
          `${s.lastMaskState || '-'}${tag}`,
      );
    }
    push('');

    push('-- component coverage (axle centre distance; sensor bar = axle + 24 cm heading) --');
    push(
      pad('#', 3) +
        ' | ' +
        pad('id', 5) +
        ' | ' +
        pad('type', 10) +
        ' | ' +
        pad('el', 3) +
        ' | ' +
        padStart('x', 6) +
        ' | ' +
        padStart('y', 7) +
        ' | ' +
        padStart('axle', 7) +
        ' | ' +
        padStart('sens', 7) +
        ' | ' +
        padStart('axle#', 5) +
        ' | ' +
        pad('verdict', 11) +
        ' | closest hit (ms/index/state) / label',
    );
    ev.coverage.forEach((c, i) => {
      push(
        pad(i + 1, 3) +
          ' | ' +
          pad(c.id, 5) +
          ' | ' +
          pad(c.type, 10) +
          ' | ' +
          pad(c.elementNumber ?? '-', 3) +
          ' | ' +
          padStart(fmt(c.x, 1), 6) +
          ' | ' +
          padStart(fmt(c.y, 1), 7) +
          ' | ' +
          padStart(fmt(c.axleMinSegCm, 2), 7) +
          ' | ' +
          padStart(fmt(c.sensorMinSegCm, 2), 7) +
          ' | ' +
          padStart(c.passes, 5) +
          ' | ' +
          pad(c.verdict, 11) +
          ' | ' +
          `${c.closestMs}/${c.closestIndex}/${c.closestState} ${c.label}`,
      );
    });
    push('');

    if (!ev.finished) {
      push(`-- last ${FAIL_TAIL_ROWS} control cycles before stop --------------------------------`);
      push(
        padStart('ms', 7) +
          ' | ' +
          padStart('index', 5) +
          ' | ' +
          pad('state', 15) +
          ' | ' +
          padStart('count', 5) +
          ' | ' +
          padStart('x', 8) +
          ' | ' +
          padStart('y', 8) +
          ' | ' +
          padStart('mask', 4) +
          ' | ' +
          padStart('pos', 3) +
          ' | ' +
          pad('line', 13) +
          ' | ' +
          pad('fail', 4),
      );
      for (const r of ev.failureTail) {
        push(
          padStart(r.ms, 7) +
            ' | ' +
            padStart(r.index, 5) +
            ' | ' +
            pad(r.state, 15) +
            ' | ' +
            padStart(r.count ?? '-', 5) +
            ' | ' +
            padStart(fmt(r.x, 1), 8) +
            ' | ' +
            padStart(fmt(r.y, 1), 8) +
            ' | ' +
            padStart(r.mask ?? '-', 4) +
            ' | ' +
            padStart(r.pos ?? '-', 3) +
            ' | ' +
            pad(r.line, 13) +
            ' | ' +
            pad(r.failure ?? 0, 4),
        );
      }
      push('');
    }
  }

  push('================================================================================');
  push(' SUMMARY');
  push('================================================================================');
  for (const ev of results) {
    push(
      `scheme ${ev.scheme}: ${ev.finished ? 'ROUTE_FINISHED' : 'NOT-FINISHED'}  ` +
        `${ev.simTimeS.toFixed(2)}s  index ${ev.finalIndex}` +
        `${ev.routeStepCount ? `/${ev.routeStepCount - 1}` : ''}  ` +
        `failure=${ev.failureName}  components ${ev.coveredCount}/${ev.componentCount}` +
        ` (expected ${expectedCoveredCount(ev)} for this scheme)` +
        ` (sensor ${ev.sensorCoveredCount})${ev.componentCountOk ? '' : ' (COUNT MISMATCH)'}`,
    );
  }
  const allPass = results.length > 0 && results.every(schemeVerdict);
  push('');
  push(`overall: ${allPass ? 'PASS' : 'FAIL — at least one scheme did not complete the lap with full coverage'}`);
  push('limits : axle-centre proximity only (not the physical 8-channel reading); greyscale');
  push('         component footprints are not modelled here; positions come from the');
  push('         simulator map (calibration assumption, see 2026-11-06-new-track).');
  push('');
  return { text: out.join('\n'), allPass };
}

/* --------------------------------------------------------------------- main */

async function main() {
  const opts = parseArgs(process.argv);
  mkdirSync(opts.out, { recursive: true });

  if (!existsSync(path.join(opts.sim, 'tests', 'replay.mjs'))) {
    throw new Error(`replay harness not found: ${path.join(opts.sim, 'tests', 'replay.mjs')}`);
  }
  if (!existsSync(opts.routeTable)) throw new Error(`firmware route table not found: ${opts.routeTable}`);

  if (!opts.quiet) console.log(`loading track map from ${opts.sim} ...`);
  const meta = await loadSimulatorMeta(opts.sim);
  if (!opts.quiet) console.log(`components: ${meta.components.length} (expected ${EXPECTED_COMPONENTS})`);

  const runInfo = [];
  const results = [];
  for (const scheme of opts.schemes) {
    const csvFile = path.join(opts.out, `scheme${scheme}.csv`);
    const jsonFile = path.join(opts.out, `scheme${scheme}.json`);
    if (opts.reuse) {
      if (!existsSync(csvFile)) throw new Error(`--reuse given but ${csvFile} does not exist`);
      runInfo.push({ scheme, skippedRun: true, ok: true, code: 0, signal: null, timedOut: false, wallMs: 0, stderr: '', stdout: '' });
    } else {
      if (!opts.quiet) process.stdout.write(`replaying scheme ${scheme} ... `);
      const run = await runReplay(opts, scheme);
      writeFileSync(path.join(opts.out, `scheme${scheme}.replay.log`), run.stdout + run.stderr);
      runInfo.push(run);
      if (!opts.quiet) console.log(`exit ${run.code} in ${run.wallMs} ms`);
      if (!run.ok || !existsSync(csvFile)) {
        results.push({
          scheme,
          replayFailed: true,
          csvFile,
          jsonFile,
          error: run.timedOut
            ? `replay timed out after ${opts.timeoutMs} ms`
            : `replay exited ${run.code}${run.signal ? ` (${run.signal})` : ''}; see scheme${scheme}.replay.log`,
        });
        continue;
      }
    }
    const ev = evaluateScheme(scheme, csvFile, existsSync(jsonFile) ? jsonFile : null, meta, opts);
    results.push(ev);
  }

  const report = renderReport(opts, meta, results.filter((r) => !r.replayFailed), runInfo);
  writeFileSync(path.join(opts.out, 'report.txt'), report.text + '\n');
  writeFileSync(
    path.join(opts.out, 'report.json'),
    JSON.stringify(
      {
        generatedAt: new Date().toISOString(),
        simulator: opts.sim,
        routeTable: opts.routeTable,
        passCm: opts.passCm,
        expectedComponents: EXPECTED_COMPONENTS,
        componentCount: meta.components.length,
        overallPass: report.allPass,
        runs: runInfo,
        schemes: results,
      },
      null,
      2,
    ) + '\n',
  );
  console.log(report.text);

  const failedReplay = results.some((r) => r.replayFailed);
  if (failedReplay) {
    for (const r of results) if (r.replayFailed) console.error(`scheme ${r.scheme}: ${r.error}`);
    process.exitCode = 2;
    return;
  }
  process.exitCode = report.allPass || opts.allowFail ? 0 : 1;
}

main().catch((err) => {
  console.error(`verify_course: ${err && err.message ? err.message : err}`);
  process.exitCode = 3;
});
