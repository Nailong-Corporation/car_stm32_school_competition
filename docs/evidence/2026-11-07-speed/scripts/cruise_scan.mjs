/**
 * Route-prefix cruise-speed scan.
 *
 * Runs the real firmware route table (scheme 2) through the full simulator, but
 * overrides every step's cruise_speed so the sampled prefix can be compared at
 * different cruising speeds. Reverse-brake switches come from the environment
 * (see config.ts simEnvInt):
 *
 *   SIM_MOTOR_REVERSE_BRAKE=1 SIM_BRAKING_REVERSE_HW=1 \
 *     node tests/cruise_scan.mjs 2 100 /tmp/spd-scan/s2-100-rev
 *
 * Output: scheme2.csv (per control tick) + summary.json + a one-line summary.
 */
import { createServer } from 'vite';
import { readFileSync, writeFileSync, mkdirSync } from 'node:fs';

const scheme = Number(process.argv[2] ?? 2);
const cruise = Number(process.argv[3] ?? 60);
const out = process.argv[4] ?? `/tmp/spd-scan/s${scheme}-${cruise}`;
mkdirSync(out, { recursive: true });
const MAX_SIM_MS = 120000;

const server = await createServer({
  server: { middlewareMode: true },
  logLevel: 'error',
  optimizeDeps: { noDiscovery: true, include: [] },
});
try {
  const { LineFollowModel } = await server.ssrLoadModule('/src/sim/line_follow.ts');
  const { VehicleSimulator } = await server.ssrLoadModule('/src/sim/physics.ts');
  const { parseRouteTableC } = await server.ssrLoadModule('/src/sim/route_parser.ts');
  const { LineState, RouteState, RouteFailure } = await server.ssrLoadModule('/src/sim/types.ts');
  const cfg = await server.ssrLoadModule('/src/sim/config.ts');

  const source = readFileSync(
    '/Users/zhendong/Documents/PlatformIO/Projects/car_stm32/src/route_table.c',
    'utf8'
  );
  const steps = parseRouteTableC(source).schemes.find((s) => s.id === scheme).steps;
  for (const step of steps) step.cruise_speed = cruise;

  const f = new LineFollowModel(steps);
  const s = new VehicleSimulator(f);
  f.route.repeat_count = 1;
  s.requestStart();

  const rows = [];
  const events = [];
  let previousKey = '';
  for (let i = 0; i < MAX_SIM_MS / 5; i++) {
    s.step();
    const r = f.route;
    const o = f.observation;
    const row = {
      ms: s.sim_time_ms,
      index: r.index,
      state: RouteState[r.state],
      count: r.junctions_seen,
      x: +s.pose.x.toFixed(3),
      y: +s.pose.y.toFixed(3),
      h: +s.pose.heading.toFixed(4),
      d: +f.odometry.distance_cm.toFixed(3),
      mask: o.line_mask,
      pos: o.position,
      width: o.white_count,
      regions: o.region_count,
      lt: +f.left_target_cm_s.toFixed(3),
      rt: +f.right_target_cm_s.toFixed(3),
      lv: +s.actual_left_speed.toFixed(3),
      rv: +s.actual_right_speed.toFixed(3),
      lm: +f.odometry.left_speed_cm_s.toFixed(3),
      rm: +f.odometry.right_speed_cm_s.toFixed(3),
      lp: f.command.left_forward ? f.command.left_pwm : -f.command.left_pwm,
      rp: f.command.right_forward ? f.command.right_pwm : -f.command.right_pwm,
      rev: f.braking_reverse_active ? 1 : 0,
      rev_ms: f.braking_reverse_ms,
      failure: r.failure,
      line: LineState[f.state],
    };
    rows.push(row);
    const key = [r.index, r.state, r.junctions_seen].join(',');
    if (key !== previousKey) {
      events.push(row);
      previousKey = key;
    }
    if (f.state === LineState.LINE_STOPPED || r.state === RouteState.ROUTE_FINISHED) break;
  }

  // --- summary metrics -----------------------------------------------------
  const tracking = rows.filter((r) => r.line === 'LINE_TRACKING');
  const lost = rows.filter((r) => r.line === 'LINE_GAP_HOLD');
  const junction = rows.filter((r) => r.line === 'LINE_JUNCTION');
  const absPos = tracking.map((r) => Math.abs(r.pos));
  const final = rows[rows.length - 1];
  const failureName = Object.keys(RouteFailure).find((k) => RouteFailure[k] === final.failure);

  // Per-step timing from index changes.
  const stepStarts = [];
  let lastIndex = -1;
  for (const r of rows) {
    if (r.index !== lastIndex) {
      stepStarts.push({ index: r.index, ms: r.ms, x: r.x, y: r.y, state: r.state });
      lastIndex = r.index;
    }
  }
  // Measured speed shortly before each EXECUTING entry (approach speed to the turn).
  const executing = [];
  for (let i = 1; i < rows.length; i++) {
    if (rows[i].state === 'ROUTE_EXECUTING' && rows[i - 1].state !== 'ROUTE_EXECUTING') {
      const before = rows[Math.max(0, i - 20)]; // 100 ms earlier
      executing.push({
        index: rows[i].index,
        ms: rows[i].ms,
        x: rows[i].x,
        y: rows[i].y,
        entry_speed_100ms_before: +((before.lv + before.rv) / 2).toFixed(2),
        mask: rows[i].mask,
      });
    }
  }
  // Max speed during the run and per-step peak.
  const peakSpeed = Math.max(...rows.map((r) => Math.max(r.lv, r.rv)));
  const reverse = rows.filter((r) => r.rev);
  const reverseByStep = {};
  for (const r of reverse) reverseByStep[r.index] = (reverseByStep[r.index] ?? 0) + 5;

  const summary = {
    scheme,
    cruise_override: cruise,
    switches: {
      MOTOR_REVERSE_BRAKE_ENABLED: cfg.MOTOR_REVERSE_BRAKE_ENABLED,
      BRAKING_REVERSE_HARDWARE_CONFIRMED: cfg.BRAKING_REVERSE_HARDWARE_CONFIRMED,
      REVERSE_BRAKE_ARMED: cfg.REVERSE_BRAKE_ARMED,
    },
    reached_index: Math.max(...rows.map((r) => r.index)),
    completed_steps: Math.max(...rows.map((r) => r.index)),
    final: {
      ms: final.ms,
      index: final.index,
      state: final.state,
      x: final.x,
      y: final.y,
      distance_cm: final.d,
      failure: final.failure,
      failure_name: failureName,
      line: final.line,
    },
    peak_speed_cm_s: peakSpeed,
    tracking_frames: tracking.length,
    tracking_pct: +(100 * tracking.length / rows.length).toFixed(1),
    gap_hold_frames: lost.length,
    junction_frames: junction.length,
    max_abs_pos_tracking: absPos.length ? Math.max(...absPos) : null,
    rms_pos_tracking: absPos.length
      ? +Math.sqrt(absPos.reduce((a, b) => a + b * b, 0) / absPos.length).toFixed(3)
      : null,
    reverse_active_ms: reverse.length * 5,
    reverse_by_step_ms: reverseByStep,
    step_starts: stepStarts,
    executing_entries: executing,
  };

  const keys = Object.keys(rows[0]);
  writeFileSync(`${out}/rows.csv`, keys.join(',') + '\n' + rows.map((r) => keys.map((k) => r[k]).join(',')).join('\n'));
  writeFileSync(`${out}/summary.json`, JSON.stringify(summary, null, 2));
  writeFileSync(
    `${out}/events.jsonl`,
    events.map((e) => JSON.stringify({ ms: e.ms, index: e.index, state: e.state, count: e.count, x: e.x, y: e.y, d: e.d, mask: e.mask, pos: e.pos, lv: e.lv, rv: e.rv, rev: e.rev, failure: e.failure })).join('\n') + '\n'
  );
  console.log(JSON.stringify(summary));
} finally {
  await server.close();
}
