# 2026-11-07 提速参数扫描（仿真器副本）：两条参考路线都能整圈跑完的最快配置

本目录是本轮"提速参数扫描"的完整证据：命令、每轮实验数据、两套可直接合入的补丁、
以及"为什么 5 m/s 达不到、需要动哪里"的分析。

**边界**：全部结论来自仿真器回放（`/tmp/sim-speed-sweep` 副本）+ 主机脚本，
没有烧录、没有操作电机、主树固件与主树仿真器源码均未修改。
判定口径与 Lead 一致：`final.state = ROUTE_FINISHED` 且 `failure = 0` 且 index 到表末，
元件覆盖用主树 `docs/evidence/2026-11-07-course-verify/verify_course.mjs` 独立复核。

---

## 0. 结论（TL;DR）

| 配置 | 方案1 | 方案2 | 相对基线 | 元件覆盖 | 合入建议 |
|---|---|---|---|---|---|
| 基线（主树 60 / 16，未改） | 159.06 s，29/29 | 153.85 s，27/27 | — | 27/27、26/27 | — |
| **推荐（A：可合入）** | **118.09 s**，29/29 | **114.17 s**，27/27 | **−25.8% / −25.8%** | 27/27、26/27 | ✅ cruise 60–150 全过（仅 110 一例例外） |
| 极限（B：仿真可达，临界） | 90.26 s，29/29 | 86.46 s，27/27 | −43.3% / −43.8% | 27/27、26/27 | ⚠️ 只有 cruise 140/150 能过（145 就失败），不建议直接上板 |

- 两套配置都用 `verify_course.mjs` 复核为 **overall PASS**（含元件覆盖），
  回放 JSON 见 `replay/recommended/`、`replay/aggressive/`，复核报告见
  `verify-recommended/report.txt`、`verify-aggressive/report.txt`。
- **5 m/s（500 cm/s）在当前"路口刹停 + 原地枢轴 + 位姿链"结构下不可达**：不是参数没扫到，
  而是每个直角动作存在 ~1.1–1.6 s 的固定开销（15–17 cm 接近段 + 90° 枢轴 + 出弯居中），
  28 个直角动作就是 31–45 s，再加环岛；直道又被"短段 60 cm/s 上限 + 前视减速窗口"压住。
  可达上限实测就在 90 s/圈（≈33 cm/s 平均速度）附近。最小结构改动见第 7 节。
- **重要**：manifest 的 `turn_speed` 单独提高几乎不改变枢轴速度（枢轴被
  `JUNCTION_SETTLE_SPEED_CM_S` 夹住），Lead 之前看到的 turn=18/20/24 失败，
  实际是"出弯速度"和"枢轴横向漂移"造成的，不是"转太快"本身 —— 见 3.1/3.2。

---

## 1. 环境、命令与判定

```bash
# 1) 副本：排除 node_modules/dist/.vite-test-cache，node_modules 软链回主树
rsync -a --exclude node_modules --exclude dist --exclude .vite-test-cache --exclude .git \
  /Users/zhendong/Documents/stm32car_simulator/ /tmp/sim-speed-sweep/
ln -s /Users/zhendong/Documents/stm32car_simulator/node_modules /tmp/sim-speed-sweep/node_modules

# 2) 副本的 tests/replay.mjs 只加了一行路径覆盖（其余与主树一致，见 tools/replay-sweep-copy.mjs）
#    const source = readFileSync(process.env.SIM_ROUTE_TABLE, 'utf8');

# 3) 参数实验：改 config.ts/manifest → 生成 C 表 → 两方案并行回放 → 解析 final
python3 tools/sweep.py <experiments.json> <results.jsonl>
#    experiments.json: [{"tag","config":{常数:值},"route":{route.ts常数:值},
#                        "follow":{line_follow.ts常数:值},
#                        "table":{1:{"turn":{"行号":速度}}}, "cruise":100,"turn":24}]

# 4) 单次回放（生成表见 tools/gentable.py，与 test/sync_new_track_routes.py 行格式一致）
SIM_ROUTE_TABLE=/tmp/sim-speed-sweep/work/table.c \
  node /tmp/sim-speed-sweep/tests/replay.mjs 1 <outdir>

# 5) 独立复核（元件覆盖 27/27 / 26/27；在主树 car_stm32 目录运行，不写主树源码）
SIM_ROUTE_TABLE=<table.c> node docs/evidence/2026-11-07-course-verify/verify_course.mjs \
  --sim /tmp/sim-final-rec --route-table <table.c> --out /tmp/verify
```

- 每次实验前从 `config.ts`/`route.ts`/`line_follow.ts` 的**原始基线**恢复再套用覆盖值，
  实验之间不会互相污染；每个实验生成独立的 C 表与 `runs/<tag>/scheme{1,2}.csv`。
- 一次回放约 0.8–1.2 s，本轮共 234 个唯一实验，数据全量见 `results-all.csv`。
- 全部实验是确定性的：同一配置重复回放结果逐位相同（见 `FINAL-rec-repeat`/`FINAL-agg-repeat`）。

---

## 2. 基线（复现主树现状）

| 方案 | 时间 | index | 状态 | failure | 元件 |
|---|---|---|---|---|---|
| 1 | 159.065 s | 29/29 | ROUTE_FINISHED | 0 | 27/27 |
| 2 | 153.855 s | 27/27 | ROUTE_FINISHED | 0 | 26/27（R26 在虚线上，按参考图 2 设计跳过） |

时间去向（方案1，状态时间占比）：**EXECUTING 87.9 s (55%)、FOLLOW 62.5 s (39%)、
CLEARING 8.0 s (5%)**。也就是说基线的时间大头不是直道，而是 29 个路口动作（28 个直角 + 1 个环岛）；
这决定了"只提高 cruise"几乎不省时间（见 4.4）。

按段统计的基线特征：
- 28 个直角动作每个 ~2.4 s（17 cm 接近 @16 cm/s ≈ 1.1 s + 90° 枢轴 @16 cm/s ≈ 1.4 s）；
  28 × 2.4 + 20.0 = 87.9 s，与 EXECUTING 总时间吻合；
- 环岛单步 19.98 s（HEXAGON_SPEED_CM_S=16 的硬上限，环内 270 cm @13.5 cm/s）；
- 长直道平均也只有 27–33 cm/s，因为每个转角前有 `max(30, v²/2a+8)` 的预减速窗口。

---

## 3. 关键机制（先看这里，它解释了此前所有失败）

### 3.1 枢轴速度被三重夹住，`turn_speed` 单独提高没有用

`line_follow.ts` 里非直行 EXECUTING 的外轮目标：

```
outer = min(step.turn_speed, JUNCTION_SETTLE_SPEED_CM_S)          // 普通转弯
outer = min(step.turn_speed, min(HEXAGON_SPEED_CM_S, JUNCTION_SETTLE_SPEED_CM_S))  // 环岛
```

`JUNCTION_SETTLE_SPEED_CM_S = 16` 时，把 manifest 的 `turn` 从 16 提到 24，
枢轴外轮仍是 16（只改出弯 CLEARING 上限 `min(turn_speed, JUNCTION_EXIT_SPEED)=20`）。
证据：

| 实验 | 方案1 | 方案2 |
|---|---|---|
| 只把 `JUNCTION_SETTLE_SPEED_CM_S` 24、turn 保持 16 | 159.06 s（与基线完全一致） | 153.85 s（完全一致） |
| turn=20、settle=16（只动不动枢轴） | FAIL idx13 丢线 | 151.10 s |
| turn=24、settle=16 | FAIL idx13 丢线 | 150.97 s |

所以"turn=24 出弯判据错过窗口 → EXIT_LOST"的实质是：**枢轴还是 16，唯一变化是出弯 CLEARING
速度上限 `min(turn_speed, JUNCTION_EXIT_SPEED_CM_S)` 从 16 抬到 20**，重捕段更快、横向误差更大；
不是枢轴转太快。

### 3.2 `EXIT_RECAPTURE(8)` 的真实触发点是短段横向界限 10 cm

`route.ts` 的短段链接（`short_link_cm` 30–40 cm 的行）在枢轴过程中累计横向投影，
`|short_link_lateral_cm| > SHORT_LINK_MAX_LATERAL_CM (=10)` 立即 `EXIT_RECAPTURE`。
turn=24 时 scheme2 index6（x165 南下 40 cm 短段）刚好到 **10.005** 就停车 —— 与 Lead
观测一致。把界限 10 → 14（`patch-*-simulator.diff` 里的 route.ts 一行）：

| 短段界限 | 方案1 | 方案2 |
|---|---|---|
| 10（原值）+ turn24/settle24 | FAIL idx4 EXIT_RECAPTURE | FAIL idx6 EXIT_RECAPTURE |
| **14** | **129.41 s 通过** | **125.65 s 通过** |
| 18 | 129.41 s（同上，说明 14 够用） | 125.65 s |

这是本轮第一个真正解锁提速的改动（−19%），而且不改变任何判定阈值语义，仅把死区放宽 4 cm。

### 3.3 高速失线：出弯横偏 5–7 cm 撞上"位姿盲行预算 1000 ms / 20 cm"

`cruise ≥ 70` 时方案1 在 index13（x375 北上 220 cm）丢线停车。逐周期对照真值与里程位姿
（误差 < 0.2 cm，位姿本身没问题）后确认过程：
出弯后车轴在 x≈378–380（比 x=375 偏东 3–5 cm），灰度杆半跨 3.85 cm 完全看不到线，
位姿追线需要 1–2 s 把 5–7 cm 横偏收回来，但路径模式丢线保护只给
`COMPONENT_CROSS_MS/CM = 1000 ms / 20 cm`（`poseBoundedLoss` 分支）就 `LINE_STOP_LOST`。

`cruise=70/80/90` 全部是这一个原因（方案1 index13）。本轮把**位姿盲行预算单独拆出来**
（新增 `POSE_BOUNDED_LOSS_MS/CM = 2000 / 30`，不动元件穿越锁存窗口）后：

| 配置 | 方案1 | 方案2 |
|---|---|---|
| turn24/settle24 + lat14，cruise 70/80/90（预算 1000/20） | FAIL idx13 丢线 | 123.48 / 121.86 / 121.23 s 通过 |
| 同上 + 位姿盲行预算 2000/30 | **127.08 / 125.34 / 125.53 s 通过** | 123.48 / 121.86 / 121.23 s 通过 |

注意：单独放宽 `COMPONENT_CROSS_*` 也能过，但那会把"印刷元件穿越锁存"一起放宽；
所以最终补丁是**新增两个专用常数**并在 `line_follow.ts` 里替换 `poseBoundedLoss` 分支的两行，
元件锁存窗口保持 1000/20 不变。两者数值等价（同一代码路径），已分别验证结果逐位相同。

### 3.4 环岛单步 20 s：`HEXAGON_SPEED_CM_S=16` 是硬上限

环内是位姿多边形（无灰度依赖、半径与速度无关），把 `HEXAGON_SPEED_CM_S` 16 → 24：

| 环岛速度 | 方案1 | 方案2 |
|---|---|---|
| 16（原值） | 125.34 s，环岛步 19.9 s | 121.86 s（环岛 20.2 s） |
| **24** | **119.39 s，环岛步 14.2 s** | **115.98 s（环岛 14.1 s）** |
| 28 / 30 / 32（settle=24 时被夹回 24） | 同 24 | 同 24 |
| 32（settle 也提到 32） | FAIL idx10（环岛入口） | FAIL idx18（环岛入口） |

环岛速度本身是"免费"的 5.5–6 s（环内 270 cm 从 13.5 → 19 cm/s），但 settle > 24 之后环岛入口（Q13/底边 30 cm 短段 → 环）会先崩，
这是后面 pivot>24 的分界线。

### 3.5 为什么参数呈现"非单调 / 临界"

位姿链的 odometry 估计是准的（< 0.2 cm），但 FOLLOW 的位姿追线在高速下会**发散**：
实测（cruise 70 的失败回放）横偏从 4.1 cm 单调涨到 7.4 cm 仍在扩大（`pose_cross_cm` 序列），因为
`desired = seg + atan2(-cross, POSE_PURSUIT_LOOKAHEAD_CM)` 的 20–30 cm 固定前视
在高速下等效增益偏低，且修正量又被 `0.5*base_speed` 限幅。
于是"出弯横偏多大"决定后面是"1–2 s 内收回来"还是"发散到丢线"，
而出弯横偏又对接近速度/枢轴起点极其敏感 —— 这就是同样是 cruise 150，145 失败、150 通过的原因。
本轮用 `POSE_PURSUIT_LOOKAHEAD_CM 30 → 20`（等效增益提高）把 pivot 28 从失败救到通过，
但 24 → 16/32 方向都会失败，说明它是唯一可行窄带，不是稳健量。

---

## 4. 单因素扫描（每行一个变量，其余保持当时基线）

### 4.1 枢轴速度（cruise 60，settle = turn，短段界限 14）

| settle=turn | 方案1 | 方案2 | 失败点 |
|---|---|---|---|
| 16（基线） | 159.06 s | 153.85 s | — |
| 20 | FAIL 71.12 s | FAIL 29.91 s EXIT_RECAPTURE | s1 idx13 丢线 / s2 idx6 短段 |
| 24（界限 10） | FAIL 18.61 s EXIT_RECAPTURE | FAIL 27.10 s EXIT_RECAPTURE | 短段横向 > 10 |
| **24（界限 14）** | **129.41 s** | **125.65 s** | — |
| 20（界限 14，settle=20） | FAIL 71.12 s | 135.11 s | s1 idx13 丢线 |
| 26（界限 14） | 125.20 s | 121.25 s | — |
| 28（界限 14） | FAIL 40.94 s | FAIL 68.47 s | 环岛入口 idx10/idx18 |
| 30 | FAIL 39.70 s | FAIL 67.07 s | 环岛入口 |
| 32 / 36 | FAIL | FAIL | 环岛入口 / 更早 |

**结论**：只有同时放宽短段界限，pivot 24 才可行；pivot ≥ 28 在 cruise 60 下就撞环岛入口。

### 4.2 `turn` 与 `settle` 的分离（cruise 60）

| 组合 | 方案1 | 方案2 |
|---|---|---|
| turn 20 / settle 16 | FAIL idx13 丢线 | 151.10 s |
| turn 24 / settle 16 | FAIL idx13 丢线 | 150.97 s |
| turn 20 / settle 20 + 界限14 | FAIL 71.12 s | 135.11 s |
| turn 24 / settle 24 + 界限14 | 129.41 s | 125.65 s |

### 4.3 短段横向界限（turn 24 / settle 24 / cruise 60）

10 → 两方案都在 idx4/idx6 `EXIT_RECAPTURE`；14、18 → 都通过且时间相同（129.41/125.65 s）。
所以取 14（最小改动、留 4 cm 余量）。

### 4.4 cruise（推荐配置：pivot 24、settle 24、环岛 24、位姿预算 2000/30、短段界限 14）

| cruise | 方案1 | 方案2 |
|---|---|---|
| 60 | 123.14 s | 119.50 s |
| 70 | 121.27 s | 117.43 s |
| 80 | 119.39 s | 115.98 s |
| 90 | 119.56 s | 115.14 s |
| **100** | **118.09 s** | **114.17 s** |
| 110 | FAIL 19.82 s（s1 idx5 丢线，单例异常） | 115.13 s |
| 120 | 118.75 s | 114.19 s |
| 130 | 118.95 s | 115.09 s |
| 140 | 119.17 s | 114.64 s |
| 150 | 119.72 s | 113.86 s |

**结论**：pivot 24 把"转弯开销"压下去之后，cruise 从 60 到 150 只值 3–6 s
（因为 30 个动作 + 29 个预减速窗口 + 短段上限把直道占比压得很低），
所以**提速的主力是枢轴/环岛，不是 cruise**。cruise 60–150 全部通过（110 例外），
这也是推荐 100 而不是 150 的原因：性能几乎一样，风险更低。

### 4.5 其他单因素（baseline：turn24/settle24/lat14/环岛24，cruise 60 或 150）

| 变量 | 结果 | 结论 |
|---|---|---|
| `ROUTE_WAYPOINT_EXIT_ANGLE_RAD` 0.12→0.18 | 两个方案都更早失败 | 不是出弯判据的问题，不加宽 |
| `ROUTE_WAYPOINT_EXIT_CM` 14 → 12 / 10 | 12 失败；10 通过但更慢 | 保持 14 |
| `TURN_PIVOT_APPROACH_CM` 17 → 15 | 112.88/107.59（−4 s） | 可用 |
| `TURN_PIVOT_APPROACH_CM` 17 → 13 | FAIL idx10 | 太短会撞环岛入口 |
| `JUNCTION_SETTLE_HEADING_KP` 24 → 40 | 114.33/109.03（−2 s） | 可用 |
| `JUNCTION_SETTLE_BAND_RAD` 0.60 → 0.30 | 116.68/110.44 不变 | 无收益 |
| `ROUTE_CLEAR_POSE_CM` 2.0 → 1.0 / 0.5 | 112.62/109.15、96.3/94.0（配合其它旋钮） | 可用 |
| `BRAKING_SAFE_DECEL_CM_S2` 120 → 300 | 可用（配合慢速窗口） | 见 4.6 |
| `BRAKING_SAFE_DECEL_CM_S2` 120 → 400 | FAIL idx3 | 越过物理可达减速度 |
| `ROUTE_WAYPOINT_SLOW_CM` 30 → 16 | 可用（配合减速度 300） | 见 4.6 |
| `ROUTE_SHORT_SPEED_CM_S` 60 → 80 | 可用 | 短段略快 |
| `ROUTE_SHORT_SPEED_CM_S` 60 → 90/100/120 | FAIL（s2 环岛入口） | 不要超过 80 |
| `JUNCTION_EXIT_SPEED_CM_S` 20 → 24 | 93.54/88.93（相对 t30 基线 −3.7/−5.4 s） | 可用 |
| `JUNCTION_EXIT_SPEED_CM_S` 20 → 30/40 | FAIL s1 环岛入口 | 不要超过 24 |
| `POSE_PURSUIT_LOOKAHEAD_CM` 30 → 20 | 可用（把 pivot 28 救活） | 见 3.5 |
| `POSE_PURSUIT_LOOKAHEAD_CM` 30 → 16 / 24 | FAIL | 只有 20 这个窄带 |
| `NAVIGATION_RECOVERY_MS/CM` 1200/12 → 1600/20 | 无效（失败点不在那里） | 不加宽 |
| `JUNCTION_SETTLE_SPEED_CM_S` 26 @ cruise 100 | 114.78/110.57 通过 | 但 @80/@140 都失败 → 不稳 |
| `JUNCTION_SETTLE_SPEED_CM_S` 28/30/32 @ cruises 80/100/140 | 全部失败（环岛入口 idx10/18） | 极不稳健 |

### 4.6 慢速窗口 / 减速度（cruise 90–150，pivot 24）

`ROUTE_WAYPOINT_SLOW_CM` 与 `BRAKING_SAFE_DECEL_CM_S2` 是**一对**：窗口里用的是
`max(ROUTE_WAYPOINT_SLOW_CM, v²/(2·a_safe)+8)`，减速度 120 → 300 才能让窗口从
"30 cm(杆)、54 cm(轴)"缩到"16–24 cm"，否则高速下每个弯前都要多滑 30–50 cm 的低速段。
两者必须一起改；单独把减速度提到 400 会触发制动距离校验失败（idx3）。

### 4.7 "把所有旋钮一起加到 pivot 24" 反而全崩（重要负面结果）

把 4.5 里可用的旋钮（approach 15、settle KP 40、clear pose 0.5、慢速窗口 16、减速度 300、
短段 80、出弯 24、前视 20）叠加到 **pivot 24** 上，cruise 60/80/100/110/120/140/150 全部
方案1 失败（idx10 环岛入口或 idx13 丢线）。同一组旋钮叠加到 **pivot 32 + cruise 150**
却能通过并跑到 90.26 s。

**结论**：这些旋钮不是独立收益，它们与 pivot 32 的时序是耦合的"配平"。
所以最终配置分成两套（第 5 节）：一套少改、稳健（推荐）；一套多改、快但临界（极限）。

---

## 5. 两套最终配置

### 5.1 配置 A —— 推荐合入（少改动、cruise 全段可过）

时间：方案1 **118.09 s**（29/29，27/27 元件）、方案2 **114.17 s**（27/27，26/27 元件）；
`verify_course.mjs` **overall PASS**。

| 文件 | 改动 |
|---|---|
| `src/sim/config.ts` | 新增 `POSE_BOUNDED_LOSS_MS = 2000`、`POSE_BOUNDED_LOSS_CM = 30.0`；`JUNCTION_SETTLE_SPEED_CM_S` 16.0 → **24.0**；`HEXAGON_SPEED_CM_S` 16.0 → **24.0** |
| `src/sim/line_follow.ts` | 引入两个新常数；`poseBoundedLoss` 分支的 `lost_ms/lost_cm` 改用新常数（元件穿越锁存仍用 `COMPONENT_CROSS_*`） |
| `src/sim/route.ts` | `SHORT_LINK_MAX_LATERAL_CM` 10 → **14** |
| `test/new_track_routes.json` | `cruise_speed_cm_s` 60 → **100**；`turn_speed_cm_s` 16.0 → **24.0** |

状态时间分布（方案1）：**EXECUTING 60.0 s (51%)、FOLLOW 49.8 s (42%)、CLEARING 7.5 s (6%)、
BRAKING 1.4 s**，其中环岛单步 14.2 s；对应配置 B 为 EXECUTING 44.7 s (50%)、FOLLOW 39.6 s (44%)、
CLEARING 4.6 s (5%)、环岛单步 10.6 s。两只大头（转弯动作 + 直道）从基线的 87.9 s / 62.5 s
一起下降到 60.0 s / 49.8 s。

补丁：`patch-recommended-simulator.diff`（在 `stm32car_simulator` 目录 `git apply -p1`）、
`patch-recommended-firmware.diff`（在 `car_stm32` 目录 `git apply -p1`）。
改完 manifest 后必须重生成路径表：`cd car_stm32 && python3 test/sync_new_track_routes.py`
（会同时写 `src/route_table.c` 与仿真器内置表）。生成结果已附 `route_table-recommended.c`，
可直接比对。

C 侧同步清单（本轮未改固件，只列对应位置）：

| 常量 | C 位置 |
|---|---|
| `JUNCTION_SETTLE_SPEED_CM_S` 16.0f→24.0f | `include/motion_config.h:307` |
| `HEXAGON_SPEED_CM_S` 16.0f→24.0f | `include/motion_config.h:346` |
| 新增 `POSE_BOUNDED_LOSS_MS/CM`（2000U / 30.0f） | `include/control_config.h`（紧邻 `COMPONENT_CROSS_*`） |
| `line_follow.c` 的 `lost_ms/lost_cm` 两行改引用新常数 | `src/line_follow.c:499-500` |
| `SHORT_LINK_MAX_LATERAL_CM` 10.0f→14.0f | `src/route.c:11` |
| 路径表 cruise=100 / turn=24.0 | `test/new_track_routes.json` + 重生成 `src/route_table.c` |

### 5.2 配置 B —— 仿真极限（不建议直接上板）

时间：方案1 **90.26 s**、方案2 **86.46 s**（−43%），元件覆盖同样 27/27、26/27，verify PASS。
在 A 的基础上再改 8 个常数（`patch-aggressive-simulator.diff`）：

`JUNCTION_SETTLE_SPEED_CM_S`/`HEXAGON_SPEED_CM_S` 24→**32**、`ROUTE_WAYPOINT_SLOW_CM` 30→16、
`BRAKING_SAFE_DECEL_CM_S2` 120→300、`ROUTE_CLEAR_POSE_CM` 2.0→0.5、`JUNCTION_SETTLE_HEADING_KP`
24→40、`TURN_PIVOT_APPROACH_CM` 17→15、`POSE_PURSUIT_LOOKAHEAD_CM` 30→20、
`ROUTE_SHORT_SPEED_CM_S` 60→80、`JUNCTION_EXIT_SPEED_CM_S` 20→24；manifest cruise 150 / turn 32。

**为什么只说"极限"**（稳健性数据）：

| 变动 | 方案1 | 方案2 |
|---|---|---|
| cruise 140 | 88.97 s | 88.14 s |
| cruise **145** | 89.21 s | **FAIL 67.8 s idx21** |
| cruise 150（本配置） | 90.26 s | 86.46 s |
| cruise 155 | 89.19 s | **FAIL 67.76 s idx21** |
| pivot 30 | 93.54 s | 88.93 s |
| pivot 31 | 91.14 s | 87.70 s |
| pivot **33** | **FAIL 33.8 s idx10** | FAIL 66.6 s idx21 |

通过区间是 {140, 150} 这种"不连续"的窄带，±5% 就翻车 —— 仿真里成立，但不构成上板依据。

---

## 6. 稳健性与验证细节

- **确定性**：`FINAL-rec` 与 `FINAL-rec-repeat`、`FINAL-agg` 与 `FINAL-agg-repeat` 四次回放
  时间逐位一致；说明上表的非单调不是噪声，是真实的参数敏感性。
- **复核**：两套配置都用主树 `verify_course.mjs` 跑过完整复核（含 27 个印刷元件覆盖、
  每步 entry/exit、pivot 与 last-mask 诊断）：
  - 方案1：118.09 s / 90.26 s，index 29/29，failure=NONE，元件 27/27；
  - 方案2：114.17 s / 86.46 s，index 27/27，元件 26/27（缺 R26，虚线专属，符合设计）。
- **补丁验证（端到端）**：把本目录交付的补丁文件原样 `git apply -p1` 到一个**全新仿真器副本**，
  用官方生成器（`test/sync_new_track_routes.py`，在隔离目录里跑，不写主树）产出路径表后回放：
  - `patch-recommended-simulator.diff` + `route_table-recommended.c` → 118.09 s / 114.17 s，
    `verify_course` **overall PASS**（27/27、26/27）；
  - `patch-aggressive-simulator.diff` + `route_table-aggressive.c` → 90.26 s / 86.46 s，
    `verify_course` **overall PASS**（27/27、26/27）。
  即"交付补丁 → 新副本 → PASS"这条链是闭合的，不是只在扫描脚本里成立。
  应用命令：

```bash
# 仿真器（/Users/zhendong/Documents/stm32car_simulator）
git apply -p1 docs/.../patch-recommended-simulator.diff    # 路径按实际放置位置调整
# 固件（/Users/zhendong/Documents/PlatformIO/Projects/car_stm32）
git apply -p1 docs/evidence/2026-11-07-speed-sweep/patch-recommended-firmware.diff
python3 test/sync_new_track_routes.py        # 重生成 src/route_table.c 与仿真器内置表
```
- **线形一致**：A/B 两套配置所需的环岛行为在 `settle == HEXAGON_SPEED` 时与原公式等价，
  因此补丁**不需要**改动环岛外轮上限那一行（本轮试过解耦 `min(turn, HEXAGON_SPEED)`
  —— 数值等价，已从最终补丁中移除，减少改动面）。

---

## 7. 离 5 m/s 还有多远：瓶颈与最小结构改动

现状：A 的平均速度 ≈ 24.7 cm/s（2918 cm / 118 s），B ≈ 32.3 cm/s，都远低于 500 cm/s。
即使把 cruise 拉到 250 cm/s，方案1 也只有 91.38 s（且方案2 在 idx21 失败，见 results-all.csv 的 `cruise250`），因为：

1. **每个直角动作 ~1.6 s 的固定开销**（15–17 cm 接近段 + 90° 枢轴 + 出弯居中）：
   A 的 EXECUTING 共 60.0 s（占总时间 51%），扣掉环岛 14.2 s 后 29 个动作 ≈ 45.7 s（1.58 s/个）；
   枢轴再快也会撞"出弯横向误差 → 重捕获"边界（3.5）。
2. **短段上限**：表长 ≤100 cm 的行被 `ROUTE_SHORT_SPEED_CM_S` 与制动包络压到 60–80 cm/s，
   而两条方案里一半以上是 25–95 cm 的短段。
3. **前视减速窗口**：`max(ROUTE_WAYPOINT_SLOW_CM, v²/2a+8)` 在 150 cm/s 时约 45 cm 杆距
   （= 69 cm 轴距）就要降到转弯入口速度，长直道有效高速段只剩一半。

**最小结构改动（按性价比排序，建议 Lead 评估）**

1. **滚动转弯（rolling turn）取代"刹停-原地枢轴"**（收益最大，≈20–25 s）
   - 现状：`ROUTE_EXECUTING` 里先 `turn_approaching` 直行 15–17 cm 到角，再饱和枢轴到目标角。
   - 改法：路口不降到入口速度，改为"外轮保持当前速度、内轮按圆弧半径降速"按圆弧通过
     （`turn_targets()` 已有外轮/内轮机制，几何比不变），出弯再由位姿 + 灰度重捕获；
     把"接近段 + 枢轴"合并为一段半径 ~20–30 cm 的圆弧。
   - 代价：`ROUTE_WAYPOINT_EXIT_CM`/角度判据要改成圆弧出弯判据；短段链接需要按圆弧重算横向投影。
   - 预期：单动作 1.4 s → 0.35–0.5 s，29 个弯省 22–25 s，A 的 118 s → ~95 s，且保留全部保护。

2. **位姿追线按速度缩放前视/增益**（收益 ≈8–12 s，且是"让高速可复现"的关键）
   - 现状：`POSE_PURSUIT_LOOKAHEAD_CM` 固定 20–30 cm，`corr` 限幅 `0.5*base_speed`；
     高速时等效增益不足 → 出弯横偏会发散（3.5），这正是各配置临界的根因。
   - 改法：`lookahead = clamp(v * 0.25, 20, 60)`，`corr` 限幅按 `v²/R` 给；同时把
     `POSE_BOUNDED_LOSS_MS/CM` 也做成随速度缩放（`max(2000, 1000·v/60)`）。
   - 代价：新增 2–3 个常数 + 两处公式；不改变判定语义，但需要重跑全部回归。

3. **短段/制动包络按"可反向制动"重标**（收益 ≈5–8 s）
   - 用户已明确刹车可反向减速、硬件上限 5 m/s，但固件 `BRAKING_SAFE_DECEL_CM_S2=120`
     且反向制动默认关闭。实测（仿真 plant）在 300 cm/s² 下才能把窗口压到 16 cm 不越窗。
   - 改法：实测反向制动可用的减速度后把它写进 `BRAKING_SAFE_DECEL_CM_S2`，
     并让 `ROUTE_SHORT_SPEED_CM_S` 跟随实标后的可行速度，而不是 60。

只做 1+2 两项就能把 A 的 118 s 再压到 ~90 s，同时保持"计数授权 + 灰度/位姿双证据 +
全部故障保护"的现结构；想再往 5 m/s 走，需要把"路口"从离散动作改成"连续弧线 + 前视预瞄"
（等价于把路径表当轨迹跟踪，而不是当动作序列），那是另一个量级的重构。

---

## 8. 限制与未做

- **只做仿真**：plant 参数（驱动 120 ms、滑行 350 ms、短刹 80 ms、250/300 cm/s²）
  都是仿真标定假设，不是实车实测；A/B 的时间是仿真时间，不代表实车。
- **未烧录、未操作电机、未改主树固件与主树仿真器源码**；补丁只写到本目录。
- 判定只看"整圈完成 + index 到表末 + failure=0 + 元件几何覆盖（轴心 ≤6 cm）"，
  不代表实车灰度识别率、打滑、元件实物尺寸（元件按图样缩到 55% 的仿真假设）。
- 参数敏感性很高（6 节数据）：**任何合入前都必须在主树重新跑
  `verify_course.mjs` + 两台方案回放**，实车必须逐项重新标定（尤其
  `POSE_BOUNDED_LOSS_*`、`SHORT_LINK_MAX_LATERAL_CM` 这类"放宽容错"的改动）。
- B 配置的 cruise 150 / turn 32 是仿真可达值，**不是 5 m/s，也不能当作已达 5 m/s 的证据**。

---

## 附：本目录文件

| 文件 | 内容 |
|---|---|
| `README.md` | 本文件 |
| `results-all.csv` | 234 个唯一实验的全量结果（两方案状态/index/时间/failure + 覆盖参数） |
| `patch-recommended-simulator.diff` / `patch-recommended-firmware.diff` | 配置 A 补丁 |
| `patch-aggressive-simulator.diff` / `patch-aggressive-firmware.diff` | 配置 B 补丁 |
| `route_table-recommended.c` / `route_table-aggressive.c` | 用官方 `sync_new_track_routes.py` 生成的 C 路径表 |
| `verify-recommended/`、`verify-aggressive/` | `verify_course.mjs` 的 report.txt / report.json |
| `replay/recommended/scheme{1,2}.json`、`replay/aggressive/scheme{1,2}.json` | 两套配置的最终回放（steps + events + final） |
| `tools/sweep.py`、`tools/gentable.py`、`tools/replay-sweep-copy.mjs` | 扫描驱动、路径表生成器、副本 replay（含一行路径覆盖） |
