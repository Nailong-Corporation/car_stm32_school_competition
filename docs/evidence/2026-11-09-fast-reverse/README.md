# 2026-11-09 快速版路径 + 反向减速（negative PWM）：仿真器实现与极限提速

工作目录：`/Users/zhendong/Documents/PlatformIO/Projects/car_stm32`
仿真器副本：**`/tmp/sim-fast`**（主树 `stm32car_simulator` 的 tar 拷贝 + `node_modules` 软链）
本目录产物：`README.md`、`ts-reverse-brake.patch`、`c-sync-checklist.md`、
`regression-proof.txt`、`final-A.txt` / `final-Ball.txt` / `final-B5.txt` / `final-B6.txt` / `final-B7.txt`、
`sweep-log.csv`（全部候选配置的原始结果）

> 边界：本目录所有结论来自**仿真器回放**，没有烧录、没有操作电机；`MOTOR_REVERSE_BRAKE_DECEL_CM_S2`
> 是仿真 plant 假设值，反向力矩的实车效果（抓地/电流/后坐）未验证。

---

## 1. 结论（推荐落地配置）

**推荐：反向制动保持默认关闭（`MOTOR_REVERSE_BRAKE_ENABLED=0` / `BRAKING_REVERSE_HARDWARE_CONFIRMED=0`），
快速版只改表内 per-scheme 速度。** 理由见 §6：开启反向制动的全局增益存在，但会让普通组 2、快速组 6
在环岛入口/环岛出口失稳，属于"参数刀锋"，在硬件未确认前不值得。

| 方案 | cruise / turn（表内） | 状态 | 时间 | 覆盖 | 说明 |
| --- | --- | --- | --- | --- | --- |
| 1 走虚线含环岛（普通） | 160 / 28 | ROUTE_FINISHED, failure=0, idx 29/29 | **66.42 s** | 27/27 | 与主树逐字节一致 |
| 2 不走虚线（普通） | 160 / 28 | FINISHED, 0, 27/27 | **62.69 s** | 26/26 | 逐字节一致 |
| 3 外框（普通） | 160 / 28 | FINISHED, 0, 4/4 | **11.32 s** | 14/14 | 逐字节一致 |
| 4 走虚线不进环岛（普通） | 160 / 28 | FINISHED, 0, 29/29 | **55.70 s** | 26/26 | 逐字节一致 |
| **5 快速版（=1）** | **220 / 64** | FINISHED, 0, 29/29 | **65.70 s** | **27/27** | 唯一改动：cruise 260→**220**（Q18 覆盖硬约束） |
| **6 快速版（=2）** | **160 / 28** | FINISHED, 0, 27/27 | **62.69 s** | **26/26** | 无法提速：环岛入口在 cruise ≥180 就失稳（§6.2） |
| **7 快速版（=3）** | **260 / 64** | FINISHED, 0, 4/4 | **10.46 s** | **14/14** | 与 3 的覆盖集合一致 |

- 三组快速版全部 `ROUTE_FINISHED`、`failure=0`、index 到底；5 = 27/27、6 = 26/26（R26 虚线外，设计跳过）、
  7 = 14/14 且与普通 3 的覆盖集合相同。
- **相对现状**：5 从 65.78 → 65.70 s 且覆盖从 26/27 修到 **27/27**（现状 260/64 时 Q18 = 6.39 cm > 6 cm 判漏）；
  6 从 `FAILED idx18 fail=11` → **PASS 62.69 s**；7 保持 10.46 s。
- 也就是说：**不开反向制动时，5/6/7 基本没有可用提速空间**（-0.08 s / 0 s / 0 s）。真正卡点是
  转角速度（settle 34 会让普通组失稳）、环岛 24 cm/s 全局速度和 Q18 覆盖，不是 cruise。

**实验性上行配置（不推荐直接上板）**：反向制动 ON（`MOTOR_REVERSE_BRAKE_ENABLED=1`、
`BRAKING_REVERSE_HARDWARE_CONFIRMED=1`）+ `BRAKING_SAFE_DECEL_CM_S2=450` +
`BRAKING_REVERSE_MIN_SPEED_CM_S=30` + `BRAKING_REVERSE_MAX_MS=120` + `JUNCTION_SETTLE_SPEED_CM_S=34`：

| 方案 | cruise / turn | 时间 | 覆盖 | 结果 |
| --- | --- | --- | --- | --- |
| 5 | 260 / 64 | **62.34 s**（-3.36 s） | 27/27 | PASS |
| 6 | 160 / 20 | 72.63 s（**+9.9 s**） | 26/26 | 反而更慢，且 turn=20 时 Q18 余量只剩 0.07 cm |
| 7 | 260 / 64 | 10.17 s | 14/14 | PASS |

同一组全局常数下普通组也被拖坏：**2 在 idx18 环岛入口 EXIT_LOST（fail=11）**、4 变 54.67 s（覆盖率仍 26/26）、
1 66.42 s 不变、3 11.21 s。详见 `final-Ball.txt`。这就是"反向制动不能全局打开"的直接证据。

---

## 2. 复现命令

```sh
# 建立副本（主树仿真器只读，不在主树改任何源码）
mkdir -p /tmp/sim-fast
(cd /Users/zhendong/Documents/stm32car_simulator && \
 tar -cf - --exclude=node_modules --exclude=dist --exclude=.vite-test-cache --exclude=.vite . | \
 (cd /tmp/sim-fast && tar -xf -))
ln -s /Users/zhendong/Documents/stm32car_simulator/node_modules /tmp/sim-fast/node_modules

# 副本里让回放读本地路径表（便于改表重生成）
cp /Users/.../car_stm32/src/route_table.c          /tmp/sim-fast/local_route_table.c
cp /Users/.../car_stm32/test/new_track_routes.json /tmp/sim-fast/local_new_track_routes.json
sed -i '' "s|<主树>/src/route_table.c|/tmp/sim-fast/local_route_table.c|" /tmp/sim-fast/tests/replay.mjs

# 单组回放（对应本 README 表格）
cd /tmp/sim-fast && node tests/replay.mjs 5 /tmp/r5
node coverage.mjs /tmp/r5/scheme5.csv 5      # 元件覆盖（口径同 verify_course：轴心 ≤6 cm）
node analyze.mjs  /tmp/r5/scheme5.csv        # 逐段耗时/速度

# 独立验证器（主树脚本，指向副本）
cd /Users/zhendong/Documents/PlatformIO/Projects/car_stm32 && \
node docs/evidence/2026-11-07-course-verify/verify_course.mjs --sim /tmp/sim-fast \
  --out /tmp/cv5 --schemes 5 --quiet

# 扫参驱动（改常数 + 重生成表 + 回放 + 覆盖，全部落在 /tmp/sim-fast）
cd /tmp/sim-fast && python3 sweep.py --tag t1 --schemes 5,6,7 \
  --config '{"MOTOR_REVERSE_BRAKE_ENABLED":1,"BRAKING_REVERSE_HARDWARE_CONFIRMED":1}' \
  --table  '{"5":[260,64],"6":[160,28],"7":[260,64]}'
```

`sweep.py` 每次从 `tuning/config.pristine.ts`（= 本补丁的默认关闭基线）和
`tuning/routes.pristine.json`（= 主树 manifest）重建，因此任何一组结果都可单独复现；
`local_sync.py` 是 `test/sync_new_track_routes.py` 的本地化拷贝（只写 `/tmp/sim-fast/local_route_table.c`）。

---

## 3. 反向制动实现（TS 侧，见 `ts-reverse-brake.patch`）

四个文件、默认关闭、**关闭时逐字节无行为变化**：

1. `src/sim/config.ts`
   - 新增与固件**同名**开关/参数：`MOTOR_REVERSE_BRAKE_ENABLED=0`、`BRAKING_REVERSE_HARDWARE_CONFIRMED=0`、
     `BRAKING_REVERSE_MAX_MS=250`、`BRAKING_REVERSE_MIN_SPEED_CM_S=8.0`、`BRAKING_REVERSE_GUARD_CM=3.0`、
     `REVERSE_BRAKE_MARGIN_CM_S=3.0`、`REVERSE_BRAKE_KP=4.0`（固件里已定义但当前未被使用，仅为对齐）、
     `REVERSE_BRAKE_MAX_PWM=60.0`；补上 C 侧已有而 TS 缺失的 `BRAKING_ACCEL_FILTER_TAU_S=0.080`。
   - 新增**仿真 plant 假设**（固件没有这个常数）：`MOTOR_REVERSE_BRAKE_DECEL_CM_S2=900.0`，
     语义是"反向力矩叠加滚阻后可达的减速度"，仍受抓地/电流限制，**模型把轮速下限钳在 0，绝不倒转**。
2. `src/sim/physics.ts`：`advanceWheel()` 增加 `reverse` 分支——负 PWM 时把轮速以
   `MOTOR_RESPONSE_TAU_S` 拉向该指令对应的**负**自由转速，但每帧只允许减速、受
   `MOTOR_REVERSE_BRAKE_DECEL_CM_S2` 限制并 `max(0, …)`。零输出仍是滑行/短刹语义，未改动。
3. `src/sim/speed_control.ts`：`wheel()` 增加与 `src/speed_control.c` 完全对应的分支：
   `reverse_brake && measured > BRAKING_REVERSE_MIN_SPEED_CM_S && measured > target + REVERSE_BRAKE_MARGIN_CM_S`
   时用同一速度 PID、限幅 `[-REVERSE_BRAKE_MAX_PWM, 0]` 输出；`update()` 增加 `reverse_brake` 形参。
4. `src/sim/line_follow.ts`：补上 C 侧已有的 arm 逻辑——新增 `braking_reverse_ms /
   braking_required_cm_s2 / braking_accel_cm_s2 / braking_speed_primed / braking_previous_speed_cm_s`，
   在 braking planner 段调用 `brakingDecide()`（此前 TS 只 import 未调用），按
   `reverse_needed && !distance_exhausted && narrow && stable && state==FOLLOW && 双轮>MIN_SPEED &&
   累计<MAX_MS` 置 `braking_reverse_active`；每帧在目标算完后按
   `min(lm−lt, rm−rt) ≤ MARGIN` 释放，并在 `state==TRACKING && narrow` 时把它传进 `speed.update()`；
   同时镜像 C 的 `ROUTE_FAILURE_BRAKE_DISTANCE` 诊断。

**与 C 的唯一差别（请 Lead 决定）**：C 的 `braking_decide()` 与
`route.navigation_warning=ROUTE_FAILURE_BRAKE_DISTANCE` 是**无条件**执行的，而 TS 端为了保住
"默认关闭逐位不变"把这整段包在 `#if` 等价的 `if (BRAKING_REVERSE_HARDWARE_CONFIRMED && MOTOR_REVERSE_BRAKE_ENABLED)`
里（`config.ts` 是运行时常量，不是编译期字面量）。这导致关闭状态下 TS 的 CSV `warning` 列与 C 不同
（实测 scheme 1 首段第 301 行起 `warning: 0 → 13`，**位姿/速度/时间完全一致**）。见 §7 的两种收口方案。

---

## 4. 参数表

### 4.1 全局常数（推荐配置 = 全部保持主树值，本补丁只加了默认关闭的新常数）

| 常数 | 主树/推荐值 | 备注 |
| --- | --- | --- |
| `MOTOR_REVERSE_BRAKE_ENABLED` | **0** | 保持默认关闭 |
| `BRAKING_REVERSE_HARDWARE_CONFIRMED` | **0** | 硬件未确认 |
| `BRAKING_REVERSE_MAX_MS` | 250 | 实验里 120 更稳（见 §6） |
| `BRAKING_REVERSE_MIN_SPEED_CM_S` | 8.0 | 实验里 30 更稳（8 会破坏 6 的环岛出口） |
| `BRAKING_REVERSE_GUARD_CM` | 3.0 | |
| `REVERSE_BRAKE_MARGIN_CM_S` | 3.0 | |
| `REVERSE_BRAKE_MAX_PWM` | 60.0 | 20 也照样破坏 6 → 关键不是幅值而是"是否 arm"（见 §6.3） |
| `BRAKING_SAFE_DECEL_CM_S2` | 250 | 开反向制动时提到 450 才有收益 |
| `JUNCTION_SETTLE_SPEED_CM_S` | 28 | 34 与反向制动联用才成立；单独 34 会打坏 5（§6.4） |
| `CORNER_ARC_SPEED_CM_S` | 70 | 90/110/140 全部在第一个弯就崩（§6.5） |
| `ROUTE_SHORT_SPEED_CM_S` | 60 | 80/100/130/160 对 5 的时间影响 <0.6 s（段短、包络先到） |
| `MOTOR_MAX_ACCEL/DECEL` | 500/500 | 未改；plant 上限 |
| `MOTOR_REVERSE_BRAKE_DECEL_CM_S2` | 900（仿真假设） | 反向力矩+滚阻的仿真 plant，实车需实测 |

### 4.2 per-scheme 表内速度（manifest 键 `cruise_speed_cm_s` / `turn_speed_cm_s`）

```json
"5": { "cruise_speed_cm_s": 220, "turn_speed_cm_s": 64 },
"6": { "cruise_speed_cm_s": 160, "turn_speed_cm_s": 28 },
"7": { "cruise_speed_cm_s": 260, "turn_speed_cm_s": 64 }
```

（1/2/3/4 维持 160/28；`turn ≤ cruise` 全程满足。）

---

## 5. 结果与用时构成（为什么快不起来）

`node analyze.mjs` 的逐段统计（scheme 5, cruise 260/turn 64, reverse off）：

| 段落类型 | 段数（scheme 5） | 单段耗时 | 合计 |
| --- | --- | --- | --- |
| 滚动圆弧（arc=true，`CORNER_ARC_SPEED_CM_S=70`） | 15 | 0.35–0.5 s | ≈6 s |
| 环岛 HEXAGON（`HEXAGON_SPEED_CM_S=24`，与 settle 取小） | 1 | **14.0 s** | 14.0 s |
| 直角枢轴 / short_link 盲行（`outer=min(turn, settle)=24–28`） | ~13 | 1.5–3.4 s | ≈28 s |
| 直线（cruise 包络） | 其余 | — | ≈18 s |

- 环岛 14.0 s 占 scheme 5 的 21%，由**全局** `HEXAGON_SPEED_CM_S=24` 决定；把它提到 32/40/60
  对 5/6 的回放**完全一致**（时间、覆盖都不变），因为 `outer = min(step.turn_speed, HEXAGON_SPEED,
  JUNCTION_SETTLE=28)` 被 settle 卡住；要再快只能改 settle 或给环岛单独的 per-scheme 速度
  （需要第 9 个表字段，超出本任务范围）。
- 枢轴段 1.5–3.4 s/段：把 settle 提到 40 立刻 `EXIT_LOST`；34 是唯一还站得住的点，且**必须**与反向制动
  联用（单独 34 会让 5 在 idx20 出口失稳）。这就是"提速真正瓶颈是转弯/环岛，不是 cruise"的数据。
- cruise 260 → 220 只掉 0.08 s（5 的段普遍 ≤215 cm，`sqrt(entry²+2·a·s)` 包络先于 cruise 到顶），
  而 240/260 会让 Q18 掉到 6.39 cm（>6 cm 判漏计）→ 27/27 需要 **cruise ≤ 220**。

---

## 6. 失败边界（扫参记录，完整表见 `sweep-log.csv`）

> 所有行都是 `python3 sweep.py` 的实际输出（`final-A/B*` 为归档结果）。除注明外，`turn=64` 时枢轴受
> settle 限制，改 turn 不改时间。

### 6.1 Q18 覆盖 ↔ cruise（scheme 5，reverse off，全部常数取主树值）

| cruise/turn | 时间 | 覆盖 | 结论 |
| --- | --- | --- | --- |
| 180/64 | 65.77 s | 27/27 | PASS |
| 200/64 | 66.09 s | 27/27 | PASS |
| **220/64** | **65.70 s** | **27/27** | **推荐** |
| 240/64 | 65.78 s | 26/27 (q18 6.39) | 漏 Q18 |
| 260/64 | 65.78 s | 26/27 | 主树现状 |
| 260/48、260/28 | 65.83 / 65.94 s | 26/27 | turn 不改变结论 |

### 6.2 scheme 6 的环岛入口（cruise 硬上限 ≈160）

| 配置 | 结果 |
| --- | --- |
| 160/28（=方案 2） | **PASS 62.69 s，26/26** |
| 180/64 | FAIL，idx18 `EXIT_LOST`(11)，9 次恢复 |
| 200/240/280 + turn 28/44/64 | 全部 FAIL，idx18 |
| 160 + 反向制动（min 8/30/45/50/55/58） | FAIL，idx21（环岛后 (375,110) 转弯丢线） |
| 160 + 反向制动 min=60 | PASS 64.22 s（但 scheme 5 在 min≥58 会崩，二者窗口不重叠） |
| 160/turn=20 + 反向制动 | PASS 72.63 s（覆盖率 26/26，但比原来慢 10 s） |
| 160/turn=20，反向制动关 | 跑完 71.7 s，仅 25/26 |

根因（用 `dbg_ring.mjs` 逐帧追踪）：环岛入口行只有 50 cm，进入前 90° 枢轴会留下
**5–8 cm 横向偏差**；cruise 160 时偏差 5.2 cm（`ROUTE_WAYPOINT_RING_CONFIRM_CM=6` 刚好过），
cruise ≥180 时偏差 7.1 cm，环岛行被当成"过路路口"反复 CLEARING/计数，小车横穿环岛内部后
`EXIT_LOST`。把 `ROUTE_WAYPOINT_RING_CONFIRM_CM` 放宽到 9/12/16 cm **完全无效果**（回放逐字节相同），
说明卡的不是这个闸门，而是 CANDIDATE/CLEARING 阶段没有位姿横偏修正 —— **修这个会改到普通组 2/4**
（它们与 6 共享同一环岛入口几何），因此本任务只记录不修。

### 6.3 反向制动的"arm 与否"比幅值更关键（scheme 6，160/28）

| `REVERSE_BRAKE_MAX_PWM` | 结果 |
| --- | --- |
| 60 / 20 | 均 FAIL，idx21（54.66 s） |

即使把反向 PWM 上限压到 20（负向自由转速仅 -39 cm/s），plant 里 `MOTOR_RESPONSE_TAU_S=0.12` 支路
仍会给出 ≈900 cm/s² 的减速（而不是滑行的 0.35 s 时间常数），所以**只要 arm 就等价于"急刹"**。
`BRAKING_REVERSE_MIN_SPEED_CM_S`（arm 的最低轮速）才是真正的安全阀：8/30 破坏 6 的环岛后弯，
60 才保护住低速段，但 58 就足以让 5 在 idx10 崩。

### 6.4 settle 单独提（不配反向制动）

| 配置 | 结果 |
| --- | --- |
| settle 28 | 5 = 65.78 s（26/27） |
| settle 30 | 5 跑完 64.47 s，但 26/27（q18 6.49） |
| settle 32 | 5 = 63.45 s，27/27（需反向制动；单独用时见下） |
| settle 34 | 5 = 62.69 s，27/27（配反向制动）；**单独 34 → 5 在 idx20 失稳（46.54 s，22/27）** |
| settle 36 | 5 在 idx10 崩（25.78 s，13/27） |

### 6.5 圆弧速度（scheme 5/6/7）

| `CORNER_ARC_SPEED_CM_S` | 结果 |
| --- | --- |
| 70 | 基线可用 |
| 90 | 5 在 idx3 崩、7 跑完但只覆盖 9/14（沿边元件差 7.9–8.5 cm，弧线外抛） |
| 110 / 140 | 第一个弯就脱线（5/6/7 全部 idx1–2 失败） |

结论：`CORNER_ARC_RADIUS_CM=13` + `CORNER_ARC_HEADING_KP=22` + `CORNER_ARC_DIFF_SLEW=900` 的滚动圆弧
在 70 cm/s 已接近该控制器可跟踪的极限；再快需要按速度缩放前视/增益 + 差速前馈重构（另一项工作）。

### 6.6 `ROUTE_SHORT_SPEED_CM_S`（short 段巡航上限）

80/100/130/160 cm/s 在反向制动配置下让 5 的时间在 65.47–66.02 s 之间来回（无单调收益）：
短段的可用速度本来就被 `sqrt(entry²+2·a_safe·(len−24−14−margin))` 包络和 settle 卡住，
且该常数是全局的（会改普通组），因此**保持 60**。

---

## 7. 回归证明（`regression-proof.txt`）

1. **反向制动默认关闭 = 逐字节无操作**：补丁后（pristine 常数、pristine 表）对 1–7 全部重跑，
   `scheme1..7.csv` 与主树基线 **逐字节相同**（`cmp` 通过），时间 66.42/62.69/11.32/55.70/65.78/48.85/10.46 s
   与主树现状一致。
2. **推荐配置（变体 A：只改表内 5/6/7 速度）**：全局常数一个没动，因此 1/2/3/4（以及表值未变的 7）
   的 `scheme<n>.csv` 与基线仍 **逐字节相同**（1 = 66.42 s、2 = 62.69 s、3 = 11.32 s、4 = 55.70 s、7 = 10.46 s）。
   只有 5、6 的 CSV 变化——它们本来就只有表内 cruise/turn 被改（5: 260→220，6: 260/64→160/28）。
3. 快速组在变体 A 下：5 = 65.70 s 27/27、6 = 62.69 s 26/26、7 = 10.46 s 14/14，全部 FINISHED/failure=0/index 到底
   （`final-A.txt`，`node tests/replay.mjs` × 7 + `coverage.mjs` 口径 ≤6 cm）。
4. 补丁本身也做了落地校验：把 `ts-reverse-brake.patch` 应用到主树仿真器四个文件的干净副本
   （`patch -p1`）后与 `/tmp/sim-fast` 版本**逐字节相同**；副本上 `npx tsc --noEmit` 通过。

**关于 C/TS checkpoint 差异的收口选项**（供 Lead 选择）：
- 方案 i（当前补丁）：TS 的 decide/arm 段包在开关里，关闭时与主树回放逐字节一致，但 C 在关闭状态下
  仍会产生 `warning=BRAKE_DISTANCE(13)` 诊断列 → 仿真 GUI 的 warning 列与固件不完全同帧。
- 方案 ii（更贴 C）：TS 去掉开关包裹（`brakingDecide` 无条件执行），代价是普通组 CSV 的 `warning` 列
  变化（位姿/速度/时间不变），需要重新基线化 `docs/evidence/2026-11-07-course-verify/` 的指纹。

---

## 8. C 侧同步清单

见同目录 **`c-sync-checklist.md`**（逐文件/逐行；结论：**本补丁不需要改 C 代码**，
固件 `src/speed_control.c` / `src/line_follow.c` / `include/motion_config.h` 的脚手架与 TS 端行为已 1:1 对应，
需要同步的只有"若要打开开关时的实验参数"和两条建议清理项）。

---

## 9. 未做 / 下一步

- 未烧录、未操作电机；`MOTOR_REVERSE_BRAKE_DECEL_CM_S2`、反向 PWM 的实车减速与后坐均未验证，
  因此推荐配置保持 `MOTOR_REVERSE_BRAKE_ENABLED=0`。
- 若要真正把 5/6 提速到 60 s 以内，需要三项控制器级改动（都不在本任务的可调清单内）：
  1) CANDIDATE/CLEARING 阶段加入位姿横偏修正（解 scheme 6 环岛入口 + scheme 2/4 同步受益）；
  2) 环岛速度改为 per-scheme（第 9 个表字段或按 row cruise 派生），现在 14 s 里 21% 是 24 cm/s 的死区；
  3) 滚动圆弧按速度缩放前视/增益/差速前馈，才能把 `CORNER_ARC_SPEED_CM_S` 从 70 提到 100+。

## 8. Lead 合入记录（按方案 A 落地，并补一条“按组门控”）

- 已按推荐落地：manifest 5=220/64、6=160/28、7=260/64；**反向制动开关保持 0**
  （`MOTOR_REVERSE_BRAKE_ENABLED=0` / `BRAKING_REVERSE_HARDWARE_CONFIRMED=0`），
  七组回放 CSV 与合入前逐字节相同。
- 仿真器反向制动实现（本目录 `ts-reverse-brake.patch`）已合入主树，且在关闭状态下行为零变化
  （七组 CSV 哈希核对一致）。
- 新增“按组门控”并按原样保留：`REVERSE_BRAKE_MIN_CRUISE_CM_S=200`
  （C `include/motion_config.h` + `src/line_follow.c` 的 arm 条件、TS `config.ts` + `line_follow.ts`）。
  这样将来开启反向制动时，只有表内 cruise ≥200 的快速行会 arm，普通组 1–4 保持原行为；
  这是对“反向减速不能全局打开”的最小化补救。
- 试开记录（Lead 实测，默认 MIN_SPEED/MAX_MS 下）：开关=1 + 门控 200 时七组仍全部完成，
  但**方案 5 变慢到 67.76 s、7 到 10.71 s**（默认反转参数是“白费时间”而非帮忙），
  因此维持关闭；要真正吃到反向制动的收益，需要 `SAFE_DECEL≈450 / MIN_SPEED≈30 / MAX_MS≈120`
  这组参数（本目录 §6 的上行配置），而它对环岛入口仍然敏感，等硬件实测后再开。
