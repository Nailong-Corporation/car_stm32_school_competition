# Full-course work in progress — round 1

The user authorizes both projects and reasonable route adjustments. The objective is a complete lap with physically plausible vehicle dynamics, without map coordinates in the controller. This objective is NOT yet achieved.

## Latest user correction: scheme1 row9 count 1 → 2

The user points to the early left turn in the screenshot and explicitly requests incrementing that row's count by one. Applied to C table and TS bundled default, without changing controller logic or physical constants. Kept as requested.

Replay using the edited actual C table now passes the first observation at axle x300.1/y119.6 (count1), then executes on the second at x270.1/y119.7 (count2), completing the left onto x245. This delays the turn by one observation but still does not reach intended Q13/x345. The previous turn had already swept the sensor past x345. The full lap remains failed: HEXAGON index10 failure11 at58.340s near(247.6,153.5). Latest replay is in `count-plus-one/`; `current/` below is the earlier count1 baseline. Do not claim the +1 table edit solved the route geometry.

## Current implemented changes

- Simulator plant: continuous rolling wheel speeds with distinct driven response (120 ms), standby coasting (350 ms), and enabled zero-PWM short brake (80 ms), with acceleration/deceleration limited to 250/300 cm/s². These are explicit uncalibrated assumptions, not hardware measurements. No route-phase or map-dependent physics. Encoder pulses and pose integrate the SAME trapezoidal wheel travel. Midpoint/sinc integration replaces heading-then-forward Euler.
- GUI no longer automatically freezes simulation while physical wheel speed remains nonzero after controller STOP/FAILED/FINISHED. It allows coasting below 0.05 cm/s before auto-pause; explicit user pause retains its meaning.
- C and TS ordinary +/-90-degree turn: after front-bar detection, advance by sensor offset minus half track (24−7=17 cm), holding incoming heading before pivoting. This has a separate bounded 3000 ms approach budget. Line acquisition at 0.7 rad is retained, but completion additionally needs turn >= target−0.15 rad. This fixes early apparent completion around 40–60 degrees. This initial geometry still needs refinements for braking displacement and short neighboring nodes.
- Removed the previous distance-based missing-count fallback in both projects. Real observed junction count is again required.
- TS braking prototype uses conservative deceleration 40 cm/s² and an earlier geometric target distance. C braking planner still uses its old values/length convention; this discrepancy MUST be resolved with route-length semantics before claiming C/simulator end-to-end parity.

## Reproduction and verified progress

`/Users/zhendong/Documents/stm32car_simulator/tests/replay.mjs` reads the actual C route table, chooses a scheme, runs one lap (repeat_count=1), and writes per-tick CSV plus transition JSON. It does not use the duplicated GUI table as the authority. Command, run from simulator root:

```
node tests/replay.mjs 1 /absolute/evidence/directory
```

Current replay in `current/` (120 ms drive / 350 ms coast / 80 ms brake):

- First corner completes close to east, y≈−0.8, rather than at ≈45 degrees.
- Q4 counted near axle x≈70; Q6 target near x≈220. Schemes1/2/3 retain their Q4/Q6 regression coverage.
- Bottom nodes x=150/245/345/375 are all counted. Index7 count=4 authorizes the bottom-right left turn. Zero navigation recoveries through index8.
- At 47.255 s reaches index9 at axle (360.6,118.3), pointing west. The front bar is already west of target x345: this short pair is still missed.
- Index9 later incorrectly turns near x300. Index10 HEXAGON fails at 61.330 s, (306.6,194.2), failure11. Advancing to index10 is NOT evidence that index9 followed the intended path.

## Validation

- `tests/physics.test.ts`: finite stop/steering response at dt1/5/10 ms, distinct coast/brake stopping distances, acceleration bound, coasting encoder pulses and consistency of pose yaw with wheel travel. Passes independently via `node tests/run.mjs --physics-only`.
- `test/test_axle_turn.c`: ASan/UBSan mirrored approach, no-line approach transit, rejection of 46-degree completion, accepted near-90 exit. Passes.
- Simulator full suite passes after requiring all four actual bottom-node counts. Build passes.
- Firmware PlatformIO builds (RAM10376, Flash43176 at initial geometry revision). Latest final build log is authoritative.
- Full historical C suite is NOT green: earlier fixtures assert immediate pivot or 0.8 rad completion; current first unresolved failure is wrong-way test before completing the new approach stage. Do not weaken actual safety invariants to make these fixtures pass. Update phase-aware fixtures and add coverage for approach heading deviation, reset and bounds.

## Rejected experiments

- Independent body velocity/yaw filters layered on wheel speeds (encoder/pose mismatch).
- Route-phase-only inertia and switching it off at handoff (velocity discontinuity).
- Large post-turn ignore windows (suppressed real Q4).
- Queuing arbitrary width/edge masks while turning (confused ordinary exit sweep with next junction). Removed from both C/TS.
- Inner-wheel pre-spooling during approach (changes turn angle before the pivot baseline, causing timeout). Removed from both C/TS.
- Merely changing motor tau or turn speed to pass old trajectories. The finite coast plant stays enabled.

## Next work

1. Resolve the 30 cm neighboring turns by feasible path adjustment or by local sensor/odom geometry that distinguishes a NEW branch from the current exit. No unconditional distance/missing-count credit.
2. HEXAGON map has four rays at (345,170), not the assumed tangent continuation. Ring winding and entry/exit turns need separate contracts; existing signed300 from approach heading is inconsistent with the intended route.
3. Resolve C/TS braking distance parity; migrate synthetic C fixtures to explicit approach and pivot phases; replay all intended route schemes to final STOP with correct node order.
4. Add parameter sweeps after deterministic nominal completion. Real vehicle inertia/traction remains uncalibrated. No hardware access, flashing, or motor commands performed.
