# 2026-11-06 固件：首弯出口平滑横移误计数

## 确证问题与归属

模拟器首弯之后的单一区域灰度序列 `0x38 → 0x30 → 0x70`，加权位置为 `+1 → +2 → +3`。原 C 固件与 TypeScript 移植都有同一判定缺口：先前重叠窄线平滑向右移，已经锁存右侧 `smooth_edge_sides`，但 `0x70` 仍含中心通道 S4 且右侧至少两路，因此被分型为 `CENTER_BRANCH`。原 `smooth` 仅抑制 `EDGE`，不能阻止该帧重新进入候选和误计数。左镜像 `0x1c → 0x0c → 0x0e` 同理。

这里的“复现”是模拟器及合成灰度回放，不是 STM32 实车新现场。本目录提供 C 检测器的旧实现失败和修复后回归证据。仿真器独立差异（捕线强转覆盖位置纠偏、缺少普通丢线截止、减速重复扣前伸量、元件显示未接采样）由外部仿真器任务分别处理；本次不修改外部项目。

## 最小修改

`src/route.c` 的 `smooth` 条件从仅 `JUNCTION_EVIDENCE_EDGE` 改为 `EDGE || CENTER_BRANCH`，继续与既有 `sides & smooth_edge_sides` 相交。没有新增阈值或历史字段，也没有更改已有锁存形成/连续3帧中心恢复的解锁条件。不同侧的分支与 `SEPARATED` 仍采用原连续证据规则。

未修改 `src/route_table.c`、生产参数、位置/速度控制、动作几何、入口/出口/离口/阶段预算及任何硬保护。已有用户路径表和路径表专项测试保留；本轮只向完整套件增加平滑中心分支专项入口。

## 回归设计

`test/test_smooth_center_branch.c` 从真实 route FSM 流程建立夹具：中心历史 → 首个真实节点 → 转弯 → 居中出口确认 → CLEARING 去重 → 索引1 FOLLOW。随后输入上述横移序列，验证：

- 左右镜像、下一段目标计数1或4都不补计，不进入动作或直行过口，不推进索引。
- 短无线噪声后返回相同侧的中心相连分支/边缘不会解除锁存。
- 恢复连续3帧中心线才清除抑制；随后突现同侧强中心分支/弱L形仍能确认。
- 无需先恢复中心，反侧强分支和同侧独立分离分支仍能确认。
- 原始 GPIO 按两种极性经过传感器多数滤波后，仍拒绝该平滑序列。

该专项直接使用 `route` 与 `gray_sensor`，所以不依赖 IMU/编码器物理模型；完整主机套件另行覆盖原有三个控制模式、两极性、IMU切换、HEXAGON、转角/丢线/入口预算、速度与HAL保护及全部四张路径表。

## 命令及实际结果

```sh
# 原实现快照保留在本目录，以下命令预期断言失败：
sh test/run_smooth_center_branch_tests.sh docs/evidence/2026-11-06-simulator-repair/firmware/route-before.c

# 修复后专项及完整回归：
sh test/run_smooth_center_branch_tests.sh
sh test/run_host_tests.sh

# 项目既有 core，构建当前头文件和默认路径方案：
PLATFORMIO_CORE_DIR="$PWD/.pio/hexagon-core" pio run

git diff --check
```

- `regression-before.log`：旧实现退出134。真实首转与离口后，索引1计数从0增为1，状态2（`ROUTE_EXECUTING`），触发“不补计”断言。
- `regression-after.log`：退出0，两种极性各1098项检查；每种都覆盖左右镜像。
- `host-tests.log`：完整套件退出0，113行输出，其中103行含PASS；四张用户路径表专项均通过。C11、严格警告与ASan/UBSan沿用既有入口。
- `pio-build.log`：构建退出0，Flash42568 B、RAM10356 B。STM32 HAL存在既有unused-parameter警告，未修改第三方代码。
- `route-fix.patch`：本轮路口实现差异；`route-before.c` 用于重复执行旧实现失败回归。
- `git diff --check`：退出0。README总体与状态Mermaid文字已同步，未运行图形渲染器。

## 局限

这是基于八路离散灰度的启发式：真实L形若提供同样的平滑重叠单线输入，也可能被抑制并漏计。回归只证明该序列不会被误计及指定对照证据仍能确认，不证明任意赛道拓扑可分辨，也不证明整圈轨迹、传感器间距、抓地、轮速或机械尺寸已标定。

本次未烧录、未操作电机或任何硬件。需要在低速真实赛道记录连续原始/滤波灰度、候选类型及侧别、路口计数/索引、转向起点和出口轨迹，统计误计与漏计后再评估阈值。
