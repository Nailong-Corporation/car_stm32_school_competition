# C 侧同步清单（反向减速 / 快速版速度）

配套 `ts-reverse-brake.patch`（仿真器 TS 端）与 `README.md`（结果、参数、扫参边界）。
**结论：本补丁不需要改动任何 C 源码。** 固件里 `MOTOR_REVERSE_BRAKE_ENABLED` 的完整脚手架（常数、
负 PWM 分支、arm 分支、motor 负指令门限）都已存在，且与 TS 端行为 1:1 对应；默认 0 时行为不变。
下面逐项给出对应位置与"若要打开开关才需要动"的地方。

## A. 常数（`include/motion_config.h`）——已存在，默认值不用改

| 常数 | 行 | 默认 | TS 端 | 说明 |
| --- | --- | --- | --- | --- |
| `BRAKING_ACCEL_FILTER_TAU_S` | 211 | 0.080f | `src/sim/config.ts` 新增同名 | TS 之前缺失，本补丁补上 |
| `BRAKING_REVERSE_MAX_MS` | 213 | 250U | 同名 | 按段累计反向力矩时间上限 |
| `BRAKING_REVERSE_MIN_SPEED_CM_S` | 214 | 8.0f | 同名 | 低速切断反向输出；**实验建议 30**（8 会破坏方案 6 的环岛后弯，见 README §6.3） |
| `BRAKING_REVERSE_GUARD_CM` | 215 | 3.0f | 同名 | `braking_decide()` 的剩余距离余量，TS 已调用 |
| `BRAKING_REVERSE_HARDWARE_CONFIRMED` | 222-223 | 0 | 同名 | ⚠ **arm 与负 PWM 的双开关之一**；硬件未确认前保持 0 |
| `MOTOR_REVERSE_BRAKE_ENABLED` | 232-233 | 0 | 同名 | ⚠ 同上；`#if … && (!MOTOR_REVERSE_BRAKE_ENABLED \|\| !PRESET_ROUTE_ENABLED)` 的编译门禁已正确 |
| `REVERSE_BRAKE_MARGIN_CM_S` | 244 | 3.0f | 同名 | 实测高于目标此值才施加反向力矩 |
| `REVERSE_BRAKE_KP` | 245 | 4.0f | 同名（未使用） | **建议清理**：C 端当前也没有任何引用（负向力矩走的是 `speed_control.c` 里共享的 `SPEED_KP/KI`）。要么在 `wheel()` 里真正用它做反向通道增益，要么删掉，避免"看着像调参旋钮其实无效" |
| `REVERSE_BRAKE_MAX_PWM` | 246 | 60.0f | 同名 | 负向 PWM 幅值上限 |

## B. 代码路径（全部已存在，无需改）

| 位置 | 作用 | TS 对应 |
| --- | --- | --- |
| `src/speed_control.c:38,50-63` | `wheel()` 的负 PWM 分支：`reverse_brake && measured > MIN && measured > target+MARGIN` → 同一速度 PID、限幅 `[-REVERSE_BRAKE_MAX_PWM, 0]` | `src/sim/speed_control.ts` `wheel()` 新增分支（逐行同构） |
| `src/speed_control.c:91,123,127` | `speed_control_update_braking(..., reverse_brake, ...)` 形参与双轮传入 | `speed_control.ts` `update()` 新增 `reverse_brake` 形参 |
| `src/braking_planner.c` `braking_decide()` / `include/braking_planner.h:10` | `required_decel`、`reverse_needed`、`distance_exhausted` | `src/sim/braking_planner.ts` 已有，**本补丁才在 `line_follow.ts` 真正调用** |
| `src/line_follow.c:638-639` | braking 段起点重置 `braking_reverse_ms/active` | `line_follow.ts` brakingOrigin 变化分支同样重置 |
| `src/line_follow.c:663-665` | 调用 `braking_decide`，写 `braking_required_cm_s2` | 同构 |
| `src/line_follow.c:673-684` | arm 条件：`reverse_needed && !distance_exhausted && narrow && stable && state==FOLLOW && fminf(l,r)>MIN_SPEED && 累计<MAX_MS && frame_ms<=MAX_MS-累计` | 同构（TS 外层另加开关包裹，见 D） |
| `src/line_follow.c:1084-1087` | 释放条件：`state!=TRACKING \|\| fminf(lm-lt, rm-rt) <= MARGIN` → `false`；随后按帧累加 `braking_reverse_ms` | 同构 |
| `src/line_follow.c:1092` | 传入 `braking_reverse_active && state==LINE_TRACKING && narrow` | 同构 |
| `src/motor.c:115-119` | 未开反向制动或未开编码器时拒绝任何负 PWM；开启时限幅 `>=-REVERSE_BRAKE_MAX_PWM` | TS 的 `physics.ts` 只按 `pwm<0` 进反向制动支路（仿真无 STBY/方向引脚模型） |

## C. 仿真器独有的 plant 假设（C 侧没有对应常数，不要照抄）

- `MOTOR_REVERSE_BRAKE_DECEL_CM_S2 = 900.0`（`src/sim/config.ts`）：反向力矩叠加滚阻后可达的减速度上限，
  且模型把轮速钳在 ≥0（绝不倒转）。**这是仿真假设，实车需要用架空/低速测试标定**；
  实车若能测出反向 PWM 的实际减速度，应把这条写进 `docs/` 而不是新增固件常数。

## D. 已知 C/TS 差异（需要 Lead 决定，二者取一）

C 端 `braking_decide()` 与 `route.navigation_warning = ROUTE_FAILURE_BRAKE_DISTANCE` 是**无条件**执行的；
TS 端为了保住"默认关闭逐位不变"把整段包在 `if (BRAKING_REVERSE_HARDWARE_CONFIRMED && MOTOR_REVERSE_BRAKE_ENABLED)` 里。
代价：关闭状态下 TS 的 CSV `warning` 列不会出现 13（C 会出现），位姿/速度/时间完全一致
（实测 `scheme1.csv` 从第 301 行起仅该列不同）。

- 方案 i（保持现状）：仿真 GUI/回放的 warning 列与固件在关闭状态下不同帧；不影响运动学对比。
- 方案 ii（贴 C）：删掉 TS 的开关包裹，`brakingDecide` 无条件执行 → 普通组 CSV 的 `warning` 列变化
  （位姿/速度/时间不变），需重新基线化 `docs/evidence/2026-11-07-course-verify/` 的指纹。

## E. 打开开关时的建议实验参数（**未上板，仅仿真最优**）

若要按 README 的"实验性上行配置"在固件里试：

```c
#define BRAKING_REVERSE_HARDWARE_CONFIRMED 1   /* 架空/低速确认反向力矩方向后再置 1 */
#define MOTOR_REVERSE_BRAKE_ENABLED        1
#define BRAKING_SAFE_DECEL_CM_S2         450.0f
#define BRAKING_REVERSE_MIN_SPEED_CM_S    30.0f
#define BRAKING_REVERSE_MAX_MS           120U
/* JUNCTION_SETTLE_SPEED_CM_S 28 -> 34 */
```

⚠ 这组常数在仿真里能让方案 5 从 65.78 → 62.34 s（27/27），但同时会让 **方案 2 在环岛入口 EXIT_LOST**、
方案 6 必须把 turn 降到 20 才勉强跑完（72.63 s，反而更慢）。**因此不建议整组照抄**，
建议先只把 `MOTOR_REVERSE_BRAKE_ENABLED/HARDWARE_CONFIRMED` 打开、其余常数不动，在架空台架确认
"负 PWM 只减速不倒转"之后再逐项扫参。

## F. 路径表（`test/new_track_routes.json` → `test/sync_new_track_routes.py`）

推荐落地只改一组 per-scheme 值（`turn ≤ cruise`）：

```json
"5": { "cruise_speed_cm_s": 220, "turn_speed_cm_s": 64 },
"6": { "cruise_speed_cm_s": 160, "turn_speed_cm_s": 28 },
"7": { "cruise_speed_cm_s": 260, "turn_speed_cm_s": 64 }
```

改完必须 `python3 test/sync_new_track_routes.py` 重新生成 `src/route_table.c`（与仿真器内置表），
再跑 `node tests/firmware_parity.mjs` 与 `sh test/run_host_tests.sh`（`turn ≤ cruise` 由生成器/路由校验把关）。
本任务**没有**修改主树 manifest 与 `route_table.c`（只改 `/tmp/sim-fast` 的本地副本）。
