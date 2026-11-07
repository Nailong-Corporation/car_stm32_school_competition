import assert from 'node:assert/strict';
import { LineFollowModel } from '../src/sim/line_follow';
import { VehicleSimulator } from '../src/sim/physics';
import { RoutePlannerModel } from '../src/sim/route';
import { parseRouteTableC, DEFAULT_ROUTE_TABLE_C } from '../src/sim/route_parser';
import { CONTROL_PERIOD_S } from '../src/sim/config';
import {
  Action,
  GrayKind,
  JunctionEvidence,
  LineState,
  LineStopReason,
  RouteState,
  type GrayObservation,
  type RouteStep,
} from '../src/sim/types';

const center = (): GrayObservation => obs(0x18, 0);
const obs = (mask: number, position: number, kind = GrayKind.GRAY_VALID): GrayObservation => {
  let count = 0;
  let regions = 0;
  let inRegion = false;
  for (let i = 0; i < 8; i++) {
    if (mask & (1 << i)) {
      count++;
      if (!inRegion) regions++;
      inRegion = true;
    } else {
      inRegion = false;
    }
  }
  return {
    kind,
    raw_mask: mask,
    line_mask: mask,
    selected_mask: mask,
    white_count: count,
    region_count: regions,
    position,
    prefilter_mask: mask,
    seam_mask: 0,
  };
};
const reverse8 = (mask: number): number => {
  let r = 0;
  for (let i = 0; i < 8; i++) if (mask & (1 << i)) r |= 1 << (7 - i);
  return r;
};
const step = (action: Action, count: number, length = 100): RouteStep => ({
  junction_count: count,
  turn_speed: 50,
  action,
  cruise_speed: 50,
  angle_deg: action === Action.CUSTOM_TURN ? 90 : 0,
  corner_arc_enabled: false,
  segment_length_cm: length,
});

// A single line drifting from center to the right through 0x30 is not a new
// junction when the next filtered frame is the center+branch 0x70.
function smoothSequence(maskStart: number, maskSlow: number, maskCenter: number, position: number[]): void {
  const steps = [step(Action.LEFT, 1), step(Action.LEFT, 4), step(Action.STOP, 1)];
  const r = new RoutePlannerModel();
  r.init(steps);
  r.start(0, 0);
  let now = 0;
  let d = 0;
  const feed = (o: GrayObservation, n = 1) => {
    for (let i = 0; i < n; i++) {
      now += CONTROL_PERIOD_S * 1000;
      d += 0.1;
      r.update(o, d, 0, now);
    }
  };
  feed(center(), 3);
  feed(obs(maskStart, position[0]));
  feed(obs(maskSlow, position[1]));
  feed(obs(maskCenter, position[2]), 3);
  assert.equal(r.junctions_seen, 0, 'smooth same-side drift must not count a junction');
  assert.equal(r.index, 0);
  assert.equal(r.state, RouteState.ROUTE_FOLLOW);

  // A one-frame dropout does not release the same-side smooth latch.
  feed(obs(0, 0, GrayKind.GRAY_NONE));
  feed(obs(maskCenter, position[2]), 4);
  assert.equal(r.junctions_seen, 0, 'isolated noise must not renew the junction candidate');

  // Three stable center frames restore normal detection; a real center+branch
  // then confirms through the ordinary candidate path.
  feed(center(), 3);
  feed(obs(maskCenter, position[2]), 3);
  assert.equal(r.junctions_seen, 1, 'a genuine branch after center recovery must count');
  assert.equal(r.index, 0, 'target action starts on the current step');
  assert.equal(r.state, RouteState.ROUTE_EXECUTING);
}

if (!process.env.CASE || process.env.CASE === 'smooth') {
  smoothSequence(0x38, 0x30, 0x70, [1, 2, 3]);
  smoothSequence(reverse8(0x38), reverse8(0x30), reverse8(0x70), [-1, -2, -3]);
  console.log('PASS: smooth center+branch drift is not a second junction in either lateral direction');
}

// A visible captured exit must let the position PID steer against the turn;
// the old unconditional minimum-turn bias inverted this correction.
if (!process.env.CASE || process.env.CASE === 'capture') {
  for (const direction of [-1, 1]) {
  const steps = [step(direction > 0 ? Action.RIGHT : Action.LEFT, 1), step(Action.STOP, 1)];
  const f = new LineFollowModel(steps);
  f.state = LineState.LINE_TRACKING;
  f.route.state = RouteState.ROUTE_EXECUTING;
  f.route.steps = steps;
  f.route.index = 0;
  f.route.entry_heading = 0;
  f.route.phase_ms = 0;
  f.route.turn_capturing = true;
  f.odometry.heading_rad = direction * 1.2;
  f.update(obs(direction > 0 ? 0x07 : 0xe0, -direction * 5), { left: 0, right: 0 },
    10, CONTROL_PERIOD_S, false, true, direction * 1.2);
  assert.equal(f.route.turn_capturing, true);
  assert.ok(
    direction * (f.left_target_cm_s - f.right_target_cm_s) < 0,
    `captured exit must allow opposite steering, direction=${direction}, targets ${f.left_target_cm_s}/${f.right_target_cm_s}`
  );
  }
  console.log('PASS: captured exit keeps the PID steering direction in both turns');
}

// Ordinary route following has no unlimited blind hold after losing the line.
if (!process.env.CASE || process.env.CASE === 'loss') {
  for (const ticks of [1, 5]) {
  const steps = [step(Action.STRAIGHT, 99, 1000), step(Action.STOP, 1, 1000)];
  const f = new LineFollowModel(steps);
  let counts = { left: 0, right: 0 };
  let now = 0;
  for (let i = 0; i < 3; i++) {
    f.update(center(), counts, now, CONTROL_PERIOD_S, i === 2, true, 0);
    now += CONTROL_PERIOD_S * 1000;
  }
  assert.equal(f.state, LineState.LINE_TRACKING);
  for (let i = 0; i < 160 && f.state !== LineState.LINE_STOPPED; i++) {
    counts = { left: (counts.left + ticks) & 0xffff, right: (counts.right - ticks) & 0xffff };
    now += CONTROL_PERIOD_S * 1000;
    f.update(obs(0, 0, GrayKind.GRAY_NONE), counts, now, CONTROL_PERIOD_S, false, true, 0);
  }
  assert.equal(f.state, LineState.LINE_STOPPED);
  assert.equal(f.stop_reason, LineStopReason.LINE_STOP_LOST);
  if (ticks === 1) {
    assert.equal(now, 410, '400 ms must expire before accepting any later recovery');
    assert.ok(f.odometry.distance_cm < 8);
  } else {
    assert.ok(now < 410 && f.odometry.distance_cm >= 8, '8 cm must stop before the time limit');
  }
  }

  // A maneuver owns gray loss until its phase budget expires. The generic
  // 400 ms/8 cm guard must not take control in the middle of that turn.
  for (const phase of [RouteState.ROUTE_EXECUTING, RouteState.ROUTE_CLEARING]) {
    const f = new LineFollowModel([step(Action.RIGHT, 1), step(Action.STOP, 1)]);
    f.state = LineState.LINE_JUNCTION;
    f.route.state = phase;
    f.route.phase_ms = 20;
    f.startup_elapsed_s = 1;
    f.abnormal = true;
    f.last_reliable_ms = 0;
    f.update(obs(0, 0, GrayKind.GRAY_NONE), { left: 0, right: 0 }, 405, CONTROL_PERIOD_S, false, true, 0.8);
    assert.notEqual(f.state, LineState.LINE_STOPPED, 'ordinary maneuver must retain its loss budget');
  }
  console.log('PASS: FOLLOW loss stops at 400 ms or 8 cm; ordinary maneuvers retain phase ownership');
}

// End-to-end physical first corners: the old simulator re-counted the line it
// had just captured and drove into the empty area; the fixed port reaches the
// next route steps without that false junction.
if (!process.env.CASE || process.env.CASE === 'first') {
  const steps = parseRouteTableC(DEFAULT_ROUTE_TABLE_C).activeSteps;
  const f = new LineFollowModel(steps);
  const sim = new VehicleSimulator(f);
  sim.requestStart();
  let minY = sim.pose.y;
  for (let i = 0; i < 2200 && f.state !== LineState.LINE_STOPPED && f.route.index < 2; i++) {
    sim.step();
    minY = Math.min(minY, sim.pose.y);
    if (f.route.index === 1 && f.route.state === RouteState.ROUTE_EXECUTING) {
      assert.ok(sim.pose.x > 27, `second corner started too early at x=${sim.pose.x}`);
    }
  }
  assert.equal(f.route.index, 2, `expected two completed corners, index=${f.route.index}`);
  assert.ok(sim.pose.x > 40 && sim.pose.x < 58 && sim.pose.y < 30,
    `expected x=55 exit corridor, pose=${sim.pose.x},${sim.pose.y}`);
  assert.ok(minY < 6, `expected the car to reach the top edge, min y=${minY}`);
  assert.equal(f.state === LineState.LINE_STOPPED, false);
  console.log(`PASS: first corners follow the top edge (index=${f.route.index}, x=${sim.pose.x.toFixed(1)}, y=${sim.pose.y.toFixed(1)})`);
}
