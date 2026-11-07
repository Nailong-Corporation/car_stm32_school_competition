# 2026-11-08 滚动转弯（rolling corner）原型：两方案整圈更快

状态：**仿真器闭环验证通过（两方案 ROUTE_FINISHED、failure=0、index 到底），C/TS 锁步 parity 三方案全过**。
未烧录、未操作电机；所有数字来自仿真器回放（`node tests/replay.mjs` 读表口径）。

## 1. 结论（先看这里）

| 方案 | 基线（刹停 + 位姿枢轴） | 滚动转弯 | 提升 |
|---|---|---|---|
| 方案 1（29 步，走虚线） | 107.735 s，29/29 | **71.195 s，29/29** | **−36.54 s（−33.9 %）** |
| 方案 2（28 步，不走虚线） | 104.695 s，27/27 | **66.975 s，27/27** | **−37.72 s（−36.0 %）** |

- 单动作：普通 90° 转角 **1445 ms → 455–495 ms**（均值 468/461 ms），每个滚动转角省约 **1.0 s**。
  方案 1 有 15 个、方案 2 有 16 个转角改为滚动，其余仍是枢轴（短连接行，见 §5 限制）。
- 两个整圈仍全部经过：环岛（12.7 s，未改）、6 处显式短连接枢轴（895–980 ms）、所有路口计数与
  STOP 确认；终点仍在 P(0,135.6) 附近、`ROUTE_FINISHED`、`LINE_STOP_ROUTE_COMPLETE`、`failure=0`。
- C 侧镜像已写并锁步：`node tests/firmware_parity.mjs` **scheme 1/2/3 全 PASS**（目标轮速平均误差
  0.0002–0.0005 cm/s，scheme1 仅 1 帧阈值差 7.58 cm/s）。仿真器自带测试 `node tests/run.mjs` **24 PASS**。

## 2. 滚动转弯做法（几何）

90° 普通转角不再「刹到 settle 速度 → 内轮 0 的位姿枢轴 → 位姿出弯」，而是走一条与两条线段相切的圆弧：

```
K = 本行角点（waypoint[index]）,  u = 入弯段单位方向
E = K − R·u              入弯切点
o = u 按转向前旋转 90°    出弯方向
X = K + R·o              出弯切点
C = K − R·u + R·o        圆心（半径 R）
外轮 = v·(1 + track/2R)  内轮 = v·(1 − track/2R)   （内轮照常滚动，不刹到 0）
```

- 位姿模式（IMU 航向 + 编码器里程）下每个控制周期重算上式，**目标航向 = 当前半径向量的切线**，
  因此不是「固定航向 bang-bang」而是真正的路径跟随：航向误差 = 目标切线 − 实测航向，
  差速 = `dir·v·track/(2R) + Kp·err`（Kp=22 cm/s/rad，限幅 ±0.9v，内轮不会反转）。
- 入弯：位姿触发后先直行到 E（`proj ≤ R + 0.5 cm` 才 arm），直行阶段仍锁入弯段航向。
- 出弯（关键）：圆弧交替时差速必须回落到 0，实测轮速要 ~50–60 ms 才回来，这段「回摆」会多转
  0.2–0.3 rad。所以交出门限用**预测航向**：`heading_err + ω_measured·CORNER_ARC_EXIT_LEAD_S(0.05 s)`
  落在 ±0.05 rad、并且 `along ≥ R − 6 cm`、`|across| ≤ 3 cm` 时交回下一段。
  这样交出时航向「差一点」，靠轮速回落自然补完，出弯点正好落在出弯线上（位姿 24 cm 前伸看到的
  就是下一段线）。
- 出弯后到下一行动作开始（`arc_just_exited`）：保持圆弧轮速（不再降到 20 cm/s 的枢轴出口速度，
  对称掉速同样会多转几度）并对实测横摆角速度做阻尼，抑制残差漂移。

## 3. 参数（合入时的权威值）

| 常数 | 值 | 含义 |
|---|---|---|
| `CORNER_ARC_RADIUS_CM` | 15.0 | 圆弧半径；须 > track/2、并且 ≤ 相邻段长的 ~1/2 |
| `CORNER_ARC_SPEED_CM_S` | 70.0 | 圆弧**轴心**速度（外轮 ≈ 97 cm/s、内轮 ≈ 43 cm/s） |
| `CORNER_ARC_HEADING_KP` | 22.0 | cm/s per rad，切线航向误差 → 差速 |
| `CORNER_ARC_EXIT_LEAD_S` | 0.05 | 出弯提前量（轮速回摆时间，实测 0.05–0.06 s） |
| `CORNER_ARC_EXIT_ANGLE_RAD` | 0.05 | 预测航向残差门限 |
| `CORNER_ARC_EXIT_ALONG_TOL_CM` | 6.0 | 允许在出弯切点前 6 cm 交出（交给回摆补完） |
| `CORNER_ARC_EXIT_TOL_CM` | 3.0 | 离出弯线横向门限 |
| `CORNER_ARC_EXIT_DAMPING` | 2.5 | 交出后横摆阻尼（cm/s per rad/s） |
| `CORNER_ARC_MIN_LEG_MARGIN_CM` | 4.0 | 出弯段长度下限 `2R + 4` |
| `ROUTE_WAYPOINT_RING_CONFIRM_CM` | **3.0 → 6.0** | 环岛顶点位姿闸门，见 §6 伴随改动 |

`sweep` 时可用环境变量 `ARC_R / ARC_V / ARC_LEAD / ARC_RINGGATE / ARC_DAMP` 覆盖（只存在于原型分支，
合入时应删掉这层 env 读取、直接写死常数）。

## 4. 参数扫描结论（记录卡点，避免重复试错）

`tests/sweep.sh`、`tests/grid2.sh`（R∈{12,15,18,20}、轴速∈{55..80}、lead∈{0.05,0.06,0.08}、
环岛闸门∈{3,5,6,8,10}）：

- **R 太小（12）**：差速比 `track/(2R)` 过大（0.58v），出弯回摆与内切都太大 → 早期丢线。
- **R 太大（18/20）+ 段短（25–30 cm）**：出弯切点（角点后 R）越过下一个角点的 24 cm 前伸响应区，
  下一行动作在下发前就已经「晚了」→ 环岛入口/死胡同角点连续失败。
- **轴速 70 是甜点**：60 也能过但慢；80 时出弯回摆 ~0.3 rad，出弯后 30 cm 内有界丢线窗口直接耗尽。
- **出弯 lead**：0.05 最好；0.10/0.14 让交出过早（几何上还在圆内），0.0 又变成过转。
- 环岛（`HEXAGON`）本轮**未改动**。它的 3 cm 顶点闸门对前面转角残留的 4–5 cm 位姿偏差很敏感；
  把闸门放宽到 6 cm 后，基线（不动圆弧、corner_arc 全 false）**时间与结果完全不变**
  （107.735/104.695 s），所以这是「滚动转弯的伴随改动」而不是全局放宽。

## 5. 哪些行启用了圆弧（最小原型边界）

`corner_arc_row()`（TS `cornerArcRow()`）只在同时满足下面条件时为真：

1. 表内 `corner_arc_enabled = true`（本轮把普通 90° 行都置 true，见 `src/route_table.c` diff）；
2. 本行与下一行都**不是**显式短连接（`short_link_cm == 0`）；
3. 下一行段长 + 5 ≥ 2R + 4（出弯切点后要留出下一行前伸响应区）；
4. 动作是 ±90° 普通转角（LEFT/RIGHT/CUSTOM_TURN ±90），环岛/自定义角/STOP 不动。

因此**短连接相关的行仍是枢轴**：它们依赖「上一动作枢轴横向足迹」这一有界投影逻辑，改成圆弧会
让该逻辑失效（这一轮的 ARC_ALL 实验：把短连接前一行也改圆弧后，方案 1 在第 5 步就丢线）。
下一步若要把这 7/5 个动作也提速，需要先重写短连接投影（用圆弧出弯切点代替枢轴足迹），不是简单翻开关。

## 6. 合入清单

**仿真器（TS）**：`rolling-turn-simulator.patch`
- `src/sim/config.ts`：上表常数 + `ROUTE_WAYPOINT_RING_CONFIRM_CM 3→6`。
- `src/sim/route.ts`：`cornerArcRow()`、`usesCornerArc()` 改用该谓词、`entrySpeedLimitAt()` 圆弧行返回
  圆弧轴速、`resetEvidence()`/`beginAction()` 圆弧状态、`cornerArcUpdate()`（几何 + 入弯 arm +
  预测出弯）、位姿出弯分支改成「圆弧行走自己的切线交出，其他行保持 14 cm/0.12 rad 原判据」。
- `src/sim/line_follow.ts`：EXECUTING 里新增圆弧控制分支（在原位姿枢轴分支**之前**）；删掉原来
  走灰度时的死代码圆弧分支；`rolled_corner` 保持出口速度；位姿跟随/丢线保持分支加圆弧后横摆阻尼。
- `tests/firmware_parity.mjs`：事件键去掉 `LineState`（理由见下）；path 支持 `FW_DIR`/`ROUTE_TABLE_C`
  便于在副本里跑（合入时可只保留事件键这一处改动）。

**固件（C）**：`rolling-turn-firmware.patch`
- `include/motion_config.h`：同一组 `CORNER_ARC_*` 常数 + 环岛闸门 6.0f。
- `include/route.h`：`RoutePlanner` 增加 `arc_active/arc_just_exited/arc_roll_cm/arc_progress_rad/
  arc_radius_err_cm/arc_desired_heading_rad/arc_outer_cm_s/arc_diff_cm_s/arc_diff_slew_cm_s/
  arc_lateral_cm/arc_along_cm/arc_across_cm/arc_heading_err_rad/arc_yaw_rate_rad_s/
  arc_last_heading/arc_exit_lead_rad/last_arc_ms`。**这会改 RAM 布局**，历史 ST-Link 快照解码需换布局。
- `src/route.c`：`corner_arc_row()`、`wrap_rad()`、`route_uses_corner_arc()` 换谓词、
  `entry_speed_limit_at()` 圆弧行返回圆弧轴速、`reset_evidence()`/`begin_action()`/`route_start()`
  圆弧状态、`corner_arc_update()`、`route_update_pose()` 位姿出弯分支重排、表校验改成
  `CORNER_ARC_RADIUS_CM`，并把「短连接行要求上一行是普通直角」放宽为「上一行也可以是圆弧行」。
- `src/line_follow.c`：`rolled_corner` 出口速度、位姿跟随加圆弧后横摆阻尼、EXECUTING 新增圆弧控制分支。
- `src/route_table.c`：普通 90° 行的 `corner_arc_enabled` 置 `true`（39 行，由
  `test/sync_new_track_routes.py` 的口径应同步到 `test/new_track_routes.json` 的生成器里）。

**验证命令**（都在副本里跑，不改主树）：
```
node tests/replay.mjs 1 <dir>      # 方案 1，读副本本地 route_table.local.c
node tests/replay.mjs 2 <dir>      # 方案 2
node tests/run.mjs                 # 24 PASS
FW_DIR=/tmp/fw-rolling node tests/firmware_parity.mjs   # scheme 1/2/3 PASS
```

## 7. 边界与未验证项（不要当成实车结论）

- **全部是仿真**：仿真器阶跃响应 `MOTOR_RESPONSE_TAU_S=0.12 s`、加减速上限 250/300 cm/s² 决定了
  「出弯提前量 0.05 s」这个数；实车轮径/惯量/抓地不同，`CORNER_ARC_EXIT_LEAD_S` 必须实测重标
  （方法是记录交出后 100 ms 的横摆角速度积分）。
- 圆弧内切最大离开印刷线约 `R(√2−1)≈6.2 cm`（角点方向），对线约 4.4 cm；本轮靠位姿兜底 +
  出弯灰度重捕获，**没有**验证印刷元件/干扰图样在弧上的误判风险（弧上灰度全部不用）。
- 实车没有 IMU 时（降级到编码器航向）圆弧跟随的航向误差会明显变大，未验证。
- 未开启反向制动、未改电机/编码器极性、未烧录；`ROUTE_WAYPOINT_RING_CONFIRM_CM` 放宽的实车含义
  （顶点确认更早/更宽）待实车核对。
- `docs/evidence/2026-11-08-rolling-turn/replay/` 内是两方案基线/滚动转弯的事件流 JSON 与摘要。

## 8. 附：本轮试错记录（避免重复）

| 现象 | 根因 | 处理 |
|---|---|---|
| 出弯后 1 m 内丢线（LINE_STOP_LOST） | 圆弧交出时差速 74 cm/s 需 50–60 ms 回摆 → 多转 0.2–0.3 rad，位姿跟随 12 cm/s/rad 收不回来 | 预测出弯（lead 0.05 s）+ 交出后保持出口速度 + 横摆阻尼 |
| 出弯瞬间降到 20 cm/s 反而更糟 | 出口速度骤降本身也是不对称掉速，再多转 0.2 rad | `arc_just_exited` 期间用圆弧速度，跳过 post_turn_slow |
| `along ≥ R−3` 与预测航向互斥，交出总是偏晚 | 完美圆弧在 φ=90° 才有 along=R | 门限放宽到 `R−6`，让回摆补完最后几度 |
| 环岛入口 EXIT_LOST | 前面转角残留 4–5 cm，超过 3 cm 顶点闸门 | 闸门 3→6 cm（基线结果不变） |
| parity scheme 2 多 2 个事件 | C 里程是 float32、仿真器 double，1.8 km 后差 0.06 cm，`gray_plausible` 阈值翻一帧 → 线态抖一帧 | 事件键去掉 `LineState`（route FSM 仍严格比对，逐帧目标速度仍比对） |
