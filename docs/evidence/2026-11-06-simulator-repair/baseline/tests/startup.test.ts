import assert from 'node:assert/strict';
import { LineFollowModel } from '../src/sim/line_follow';
import { VehicleSimulator } from '../src/sim/physics';
import { parseRouteTableC, DEFAULT_ROUTE_TABLE_C } from '../src/sim/route_parser';
import { CONTROL_PERIOD_S, LINE_CONFIRM_SAMPLES } from '../src/sim/config';
import { LineState } from '../src/sim/types';

const steps = parseRouteTableC(DEFAULT_ROUTE_TABLE_C).activeSteps;
const follow = new LineFollowModel(steps);
const sim = new VehicleSimulator(follow);
const tick = () => sim.step(CONTROL_PERIOD_S);
const reset = () => {
  follow.init(steps);
  sim.reset();
};

// A single click must start the controller after its actual sample confirmation.
sim.requestStart();
for (let i = 0; i < LINE_CONFIRM_SAMPLES - 1; i++) {
  tick();
  assert.equal(follow.state, LineState.LINE_IDLE);
  assert.equal(follow.command.standby, false);
}
tick();
assert.equal(follow.state, LineState.LINE_TRACKING);
for (let i = 0; i < 200; i++) tick();
assert.ok(sim.pose.y < 119, `Expected forward movement, y=${sim.pose.y}`);
assert.ok(follow.odometry.distance_cm > 1);
assert.ok(sim.actual_left_speed > 0 && sim.actual_right_speed > 0);
console.log(`PASS: one START moves the car ${(120 - sim.pose.y).toFixed(1)} cm after one second`);

// Pause is no ticks. A second play request must continue the existing route/time.
const previousTime = sim.sim_time_ms;
const previousDistance = follow.odometry.distance_cm;
sim.requestStart();
tick();
assert.equal(sim.sim_time_ms, previousTime + 5);
assert.ok(follow.odometry.distance_cm >= previousDistance);
console.log('PASS: pause / continue preserves controller progress');

// The UI single-step path must warm up without resetting its pending request.
reset();
for (let i = 0; i < LINE_CONFIRM_SAMPLES; i++) {
  sim.requestStart();
  tick();
  assert.equal(sim.sim_time_ms, (i + 1) * 5);
}
assert.equal(follow.state, LineState.LINE_TRACKING);
console.log('PASS: single-step advances exactly 5 ms and starts after confirmation');

// Playing after idle samples must also start, regardless of elapsed time.
reset();
tick();
tick();
sim.requestStart();
for (let i = 0; i < LINE_CONFIRM_SAMPLES; i++) tick();
assert.equal(follow.state, LineState.LINE_TRACKING);
console.log('PASS: START works after time has already advanced in idle');

// Reset must cancel a pending request and allow an explicit new START.
reset();
sim.requestStart();
tick();
reset();
for (let i = 0; i < 10; i++) tick();
assert.equal(follow.state, LineState.LINE_IDLE);
sim.requestStart();
for (let i = 0; i < LINE_CONFIRM_SAMPLES; i++) tick();
assert.equal(follow.state, LineState.LINE_TRACKING);
console.log('PASS: reset cancels stale START and a new click starts normally');

// No-line rejection must stay rejected; a later line must not auto-start.
reset();
sim.reset({ x: -50, y: 120 });
sim.requestStart();
for (let i = 0; i < 10; i++) tick();
assert.equal(follow.state, LineState.LINE_IDLE);
assert.equal(follow.command.standby, false);
sim.pose.x = 0;
for (let i = 0; i < 10; i++) tick();
assert.equal(follow.state, LineState.LINE_IDLE);
sim.requestStart();
for (let i = 0; i < LINE_CONFIRM_SAMPLES; i++) tick();
assert.equal(follow.state, LineState.LINE_TRACKING);
console.log('PASS: stable-line guard remains enforced; rejected START is not retried');
