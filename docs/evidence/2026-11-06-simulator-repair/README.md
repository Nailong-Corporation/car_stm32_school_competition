# 2026-11-06 仿真器截图中首弯脱线的修复与归属

## 结论

截图来自工作区外的 `stm32car_simulator`（`/Users/zhendong/Documents/stm32car_simulator`），不是 STM32 实车、也不是本工作区内的主机逻辑夹具。修复前已用同一默认路径方案（方案1，30步）和同一车辆模型复现：车辆从 P(0,120) 沿左边框北上，首个右转捕到上边框后，灰度序列 `0x38 → 0x30 → 0x70`（位置 `+1 → +2 → +3`）被再次识别成新路口；随后在车身尚未回到上边框时提前执行第二次转弯，驶入空地并绕回起点附近。截图中的大回环由此产生。

这不是单一项目的问题：

| 归属 | 问题 | 处理 |
|---|---|---|
| **固件与仿真器共有** | 平滑同侧横移的历史 `smooth_edge_sides` 只抑制 `EDGE`，当下一帧仍含中心通道而被分为 `CENTER_BRANCH` 时未抑制，导致刚离口就再次计数。`0x38→0x30→0x70` 和左镜像 `0x1c→0x0c→0x0e` 可触发。 | 固件 `src/route.c` 已单独修复并回归，见 [固件证据](firmware/README.md)；仿真器 `src/sim/route.ts` 采用同一最小条件。 |
| **仿真器移植错误** | 捕线后仍无条件施加“原转向最小半差”，覆盖位置 PID 的反向纠偏；当前 C 固件只在未捕线时施加（`src/line_follow.c` 对应判断）。 | 仿真器 `src/sim/line_follow.ts` 改为 `!turn_capturing` 才施加。 |
| **仿真器移植错误** | 普通 `FOLLOW`/`CANDIDATE` 丢线后没有 400 ms / 8 cm 截止，可持续低速盲行，放大驶入空地后的斜行。 | 仿真器移植 C 的预算所有权：普通阶段由路径阶段接管，HEXAGON 保留独立时限。 |
| **仿真器 UI 状态错误** | 控制器已 `LINE_STOPPED` 时，HUD 仍可能显示 PAUSED，因为只检查 `ROUTE_FAILED`。 | 同时检查 `LINE_STOPPED`，显示具体停车原因，并在循环中暂停。 |

## 仿真器修改文件

- `src/sim/route.ts`：`smooth` 条件从仅 `EDGE` 扩为 `EDGE || CENTER_BRANCH`，仍要求 `sides & smooth_edge_sides`。没有改阈值、历史字段、确认帧数、路径表或入口/出口参数。
- `src/sim/line_follow.ts`：
  - 捕线最小转向量增加 `!turn_capturing` 条件，保留中心近零偏置与角速度阻尼；
  - 新增 `routeOwnsGrayLoss()` 与 `gray_loss_expired` 预算交接，普通 FOLLOW/CANDIDATE 达到 400 ms 或 8 cm 时 `LINE_STOP_LOST`；BRAKING/EXECUTING/CLEARING/RECOVERING 由路径阶段接管，HEXAGON 不接管；
  - 进入 CLEARING/RECOVERING 时同步清理异常标记或 PID 历史，保持与 C 的交接语义。
- `src/main.ts`：`LINE_STOPPED` 纳入状态提示和自动暂停；显示 `LineStopReason`。
- `tests/run.mjs`：增加导航回归入口。
- `tests/navigation.test.ts`：新增四类回归（见下）。

仿真器工程无 git 元数据；源码差异保存在 `simulator-fix.patch`，修改前的副本保存在 `baseline-src/`，可直接与当前外部工程逐文件比较。`baseline/` 是带旧逻辑的测试复现副本（运行需临时链接外部 `node_modules`）。

## 回归与证据

在隔离副本中运行（当前外部工程的等价测试已通过）：

```sh
# 修复后完整仿真器测试（旧启动回归 + 新增导航回归）
node tests/run.mjs

# 修复前基线，仅跑单个用例；四个均失败
CASE=smooth  node tests/run.mjs
CASE=capture node tests/run.mjs
CASE=loss    node tests/run.mjs
CASE=first   node tests/run.mjs
```

- [`staged-tests.log`](staged-tests.log)：修复后 10 项 PASS，退出 0，包含首次两弯的物理仿真断言。
- [`actual-tests.log`](actual-tests.log)：写回外部工程后再次运行，10 项 PASS，退出 0。
- [`actual-build.log`](actual-build.log)：`npm run build` 成功（`tsc && vite build`，19 modules）。现有 `http://localhost:5173` 开发服务的源码请求也确认已包含三个修复：平滑 CENTER_BRANCH 抑制、`!turn_capturing` 条件、`LINE_STOPPED` UI 检查；未通过浏览器执行视觉检查。
- 基线失败日志：
  - [`baseline-smooth.log`](baseline-smooth.log)：`smooth same-side drift must not count a junction`，实际 1、期望 0；旧代码把 `0x70` 计成第二路口。
  - [`baseline-capture.log`](baseline-capture.log)：`captured right turn with line on left must steer left`，实际目标 `0.08/0`；旧代码把反向纠偏改成原转向。
  - [`baseline-loss.log`](baseline-loss.log)：`LINE_GAP_HOLD`（2）未进入 `LINE_STOPPED`（4）；旧代码没有普通丢线截止。
  - [`baseline-first.log`](baseline-first.log)：`expected at least two completed corners, index=1`；旧代码在首弯后停住/绕回，不能到达后续路段。

新增用例的具体含义：

1. **smooth**：从连续中心线进入 `0x38→0x30→0x70`，左右镜像都不得补计、不得进入动作；单帧丢线不解除同侧锁存；连续3帧中心后突现同侧中心+分支仍能按原门槛确认。
2. **capture**：左右镜像验证 `turn_capturing` 时位置 PID 仍可反向纠偏；右转且线在左侧时左轮目标须小于右轮目标，左转镜像相反。
3. **loss**：有效启动后持续无线，分别用每周期1计数（400 ms到期且前进不足8 cm）、5计数（8 cm先到且不足400 ms）验证 `LINE_STOP_LOST`；另外验证普通 EXECUTING/CLEARING 保持路径阶段预算，不被一般丢线检查抢停。
4. **first**：默认方案1从模型起点推进，必须在 2200 个 5 ms 周期内完成前两个目标动作；第二弯起转车轴 `x>27`，轨迹到达上边框（`min y<6`），完成时须位于 `x=55` 的出口走廊（`40<x<58, y<30`）。最终通过位置为 `(44.2,14.7)`、索引2，避免只靠运行时间或索引推定成功。

## 验证边界

- 所有“PASS”来自 TypeScript 车辆/灰度物理模型和合成序列，不是 STM32 实车。仿真器的轮速、抓地、传感器间距和赛道几何仍是模型假设；不把截图复现当作实车根因证据。
- 固件侧的修复与验证独立归档在 [firmware/README.md](firmware/README.md)：旧实现失败退出134，修复后两极性各1098项检查；完整主机套件退出0；项目内 core `pio run` 成功（Flash42568B/RAM10356B）。本次未烧录、未操作电机。
- 仿真器仍保留两处未在本轮解决的模型差异：
  1. `line_follow.ts` 的减速目标使用 `segment_length_cm - GRAY_FORWARD_OFFSET_CM`，而当前 C 固件直接使用 `segment_length_cm`。这是速度包络语义差异，不是本次误计数原因；直接改成 C 语义会使当前物理模型的首段入口速度门禁失败，涉及路径表长度与24 cm传感器前置量的测量口径，未擅自改表。
  2. 元件符号只由 `track_renderer.ts` 绘制，`physics.ts` 的 `isSensorOnTrackLine` 只遍历 `TRACK_SEGMENTS`，不采样 `CIRCUIT_COMPONENTS`；截图中的首弯误计数发生在上边框真实导线，不能归因于 R1 图形。该模型差异仍待决定是否把元件遮罩纳入采样。
- 仿真器默认方案1与当前固件默认方案0不是同一张表；本证据使用仿真器默认方案1，不能据此判断固件方案0或其它路径表的实车效果。
- 仿真器在更后面的路段仍可能因路径表/几何、入口速度门禁或后续节点计数而停车；本轮只修复截图中可复现的首弯误计数与明确移植差异，不宣称整条30步赛道已通过。真实赛道仍需低速灰度回放和实车验证。
