# 2026-11-06 方案1第3项在 Q6 前右转停车：复现与修复

## 截图复现

截图 HUD 为方案1第3项 `CUSTOM_TURN`、计数 `2/2`、17.110 s、`(259.1, 164.6)`、世界航向 `89.9°`。在当前 TypeScript 物理仿真器中精确复现：

```sh
node docs/evidence/2026-11-06-q6-stop/reproduce.mjs
```

基线 [`baseline-transitions.log`](baseline-transitions.log) / [`baseline-trace.csv`](baseline-trace.csv) 的关键点：

| 时间 | 前探头灰度 | 计数/动作 | 车轴 | 解释 |
|---|---|---|---|---|
| 9.880 s | `0x0f` 四路 | 计数0 | `(70.58, 111.38)` | S0 已看到 x=95 的 Q4 支线，却被上一轮的同侧 `EDGE||CENTER_BRANCH` 平滑抑制 |
| 11.485 s | `0xff` 全宽线 | count 1/3步 count 2/2 | `(220.00, 120.14)` | 实际是 Q6 前的竖线端，被当成第1个路口并直行通过 |
| 13.355 s | `0x1f` 五路 | count 2，开始右转 | `(249.99, 120.10)` | Q6 已被计完，随后 x=275 的竖线尾被当成第2个路口 |
| 17.110 s | 无线 | ACTION_TIMEOUT/EXIT_LOST | `(259.05, 164.57)` | 在错误转向的空地区域停车，与截图一致 |

HUD 中 `89.9° (3.14 rad)` 也是仿真器显示错误：97°/约3.14 rad 是相对起始北向的内部航向，90° 是世界坐标 Yaw；两者不能显示为同一角的两种单位。修复后统一显示世界坐标角。

## 归属

| 项目 | 问题 | 修复 |
|---|---|---|
| **固件 + 仿真器共有** | 上一轮把所有同侧 `CENTER_BRANCH` 都纳入平滑抑制，导致真实 Q4 的四路分支被漏掉；只取消抑制又会重新放行首弯三路幽灵。 | `src/route.c` 改为：`EDGE` 仍抑制，`CENTER_BRANCH` 仅在 `white_count <= JUNCTION_EDGE_MAX_WIDTH(3)` 时抑制；四路及以上恢复原确认。 |
| **固件 + 仿真器共有** | Q4 计数后车辆仍以约 24° 的斜入航向做非目标直行保持，丢失 y=120 走廊；几何上应沿上一弯的预期出口轴回正，而不是把斜入角当直行轴。 | `line_follow` 增加一次有界的临时几何出口轴（±90° 普通目标转弯的 `entry_heading + turn_target`）；仅相邻非目标候选、过口 CLEARING/图形恢复使用，原居中出口门禁和角基准不变。 |
| **仿真器独有** | HUD 把世界 Yaw 与相对起始航向混排，造成 `89.9° (3.14 rad)`。 | `src/main.ts` 统一显示世界坐标 Yaw 的度/弧度。 |
| **仿真器模型差异（未改）** | 减速仍减去 24 cm 前伸量；元件符号只绘制、不参与传感器采样。 | 保留并记录，不能据此把轨迹当实车根因。 |

路径表、动作几何、入口速度门禁、400 ms/8 cm 丢线、动作/离口时限、硬保护和参数均未改变；本次仍只做主机测试与构建，未烧录、未操作硬件。

## 固件改动

- `src/route.c`：`smooth` 增加 `CENTER_BRANCH` 宽度资格（≤3路）。
- `include/line_follow.h`：`LineFollow` 增加 `exit_heading_pending`、`exit_heading_rad`、`exit_heading_origin_cm`。
- `include/motion_config.h`：新增 `JUNCTION_PASS_ALIGNED_RAD=0.10`、`JUNCTION_PASS_SETTLED_YAW_RAD_S=0.15`，均为未实测初值。
- `src/line_follow.c`：
  - 普通 ±90° 目标转弯从 EXECUTING 转 CLEARING 时锁存几何出口轴；
  - 非目标 CANDIDATE、passing CLEARING、passing component recovery 的保持航向使用该轴，但引用误差不超过原 `JUNCTION_SETTLE_BAND_RAD=0.6`；
  - 连续3帧中心且回正到 0.10 rad 或角速度低于 0.15 rad/s 释放；超过传感器前伸 24 cm、新目标 EXECUTING、START、RECOVERING 均清空；
  - 不写 `route.entry_heading/action_start_heading`、不改计数、阶段时限、有效出口证据或回调。
- `test/test_pass_heading.c` 与 `test/run_pass_heading_tests.sh`：左右镜像验证 0x38/0x30/0x70 三路仍不计、0x0f 真分支计数、非目标过口使用出口轴、侧边图形仍不能成为出口、24 cm/对齐/低角速度/新目标清空。

## 仿真器改动

- `src/sim/route.ts`：与 C 相同的 `CENTER_BRANCH` 宽度资格。
- `src/sim/line_follow.ts`：与 C 相同的临时几何出口轴及其释放；并修复“进入 RECOVERING 当帧”未清空轴的顺序问题。
- `src/sim/config.ts`：与两个新初值对应。
- `src/main.ts`：统一世界 Yaw 显示。
- `tests/q4_q6.test.ts`：新增三／四／五路镜像检测、出口轴释放、三种方案的 Q4→Q6 物理回归；`tests/run.mjs` 接入。
- `src/main.ts`、`tests/run.mjs` 及上述文件已写回 `/Users/zhendong/Documents/stm32car_simulator`。

## 验证结果

- 固件完整主机套件：[`host-tests.log`](host-tests.log) 退出0，包含 `mirrored post-turn ... wider real center branches`（两极性各1554检查）和 `mirrored early-exit passage heading`（3283检查）。
- 出口轴旧实现：[`firmware/passage-before.log`](firmware/passage-before.log) 在非目标过口纠偏断言失败（退出134）；当前实现 [`firmware/passage-after.log`](firmware/passage-after.log) 通过。
- Q4 旧上一轮实现：[`firmware/q4-regression-before.log`](firmware/q4-regression-before.log) 退出134；当前实现 [`firmware/q4-regression-after.log`](firmware/q4-regression-after.log) 退出0。
- 固件交叉构建：[`pio-build.log`](pio-build.log) 成功，Flash 42964 B、RAM 10368 B（RAM 增加 12 B 为三字段）。
- 仿真器完整测试：[`simulator-tests.log`](simulator-tests.log) 15项 PASS；`npm run build` 成功（19 modules）。当前开发服务的源码请求已包含修复。
- 仿真器 Q4/Q6 物理回归：方案1、2、3 均通过 Q4→Q6；过程无隐藏 RECOVERING。当前物理模型后续仍可能在其它图形或表/几何处停车，不宣称整条30步赛道通过。
- 仿真器顺序回归：[`ts-before-order.log`](ts-before-order.log) 在旧的 RECOVERING 清空位置失败（退出1）；修复后通过，见 `simulator-tests.log` 的 `clearance timeout clears...`。

## 边界

Q4 的 `0x0f` 证据来自仿真八路离散模型；三路真实支线仍可能被漏计，四路图形仍可能被误计。24°斜入角、0.10/0.15/0.6/24 cm 与几何出口轴均未实车标定。路径表仍为用户方案表（方案1/2/3 由宏选择，固件默认方案0），未在本轮修改。真实深蓝底、元件遮挡、传感器标定、抓地和转弯时机须用低速实车灰度回放验证。
