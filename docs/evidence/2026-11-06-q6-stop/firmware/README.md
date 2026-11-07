# 2026-11-06 固件：Q4 漏检的宽度限定平滑抑制

## 问题归属

仿真方案1的 17.110 s 停车发生在索引3、count 2/2、位置 `(259.1,164.6)`。全周期 trace 中，第一处 count 被 Q6 之前的全宽线占用；x=95 的 Q4 未计。该漏检同时存在于 C 固件和 TypeScript 移植的判定逻辑，不是路径表或新引入的实车现象。

Q4 位置是 `(95,120)`，在 x=55→x=245 的 y=120 水平线上；仿真路线从 x≈70、y≈111 切入水平线时，前探头位于 x≈94。`0x0f` 的 S0 位于 `(93.8,117.8)`，与 x=95 竖直线相距约 1.2 cm，S1–S3 仍在 y=120 水平线上。因此 `0x0f` 是横移单线与新支线的四路真实交叠证据，不是单纯的三路横移噪声。

上一轮将 `CENTER_BRANCH` 加入同侧 `smooth_edge_sides` 抑制后，真实 Q4 分支也会被抑制；但完全取消该抑制会重新允许首弯幽灵三路 `0x70`。本修复以连通分支宽度区分：三路横移继续抑制，四路及以上恢复原有强候选确认。

## 修改

只修改 `src/route.c` 一个判定：

```c
bool smooth=(kind==JUNCTION_EVIDENCE_EDGE ||
    (kind==JUNCTION_EVIDENCE_CENTER_BRANCH &&
     o.white_count<=JUNCTION_EDGE_MAX_WIDTH)) &&
    (sides&p->smooth_edge_sides);
```

`JUNCTION_EDGE_MAX_WIDTH` 当前为 3。原 `EDGE` 行为、中心历史、三帧中心解锁、候选侧别/帧数/位移、入口门禁和所有阶段/反馈保护均不变。此子任务未修改 `line_follow.c`、路径表、标定或其它源码；相关的临时几何出口轴改动在上级 [Q6 证据](../README.md) 中单独记录和验证。

## 专项验证

`test/test_smooth_center_branch.c` 在真实 `FOLLOW → CANDIDATE → EXECUTING → CLEARING → FOLLOW` 流程后先复现三路横移，再分别输入两组左右镜像四路分支 `0x78/0x1e`、`0xf0/0x0f`，以及五路对照 `0xf8/0x1f`。`ticks()` 已经负责镜像，因此每组固定同一基准 mask；测试显式断言每组 `white_count=4/4/5` 和单区域。原有窄横移继续不计数，四/五路必须经原2帧同侧强目标门槛确认（不能等第3帧中心解锁来掩盖判定）。中心恢复、反侧/分离分支、无线中断和两种原始极性既有用例保留。

命令：

```sh
# 上一轮实际的 blanket CENTER_BRANCH 抑制：三路 drift 通过，四路遗漏失败
sh test/run_smooth_center_branch_tests.sh \
  docs/evidence/2026-11-06-q6-stop/firmware/route-before-q4.c
# 当前宽度限定实现
sh test/run_smooth_center_branch_tests.sh
# 更早的 EDGE-only 历史实现，保留其三路 ghost 失败证据
sh test/run_smooth_center_branch_tests.sh \
  docs/evidence/2026-11-06-q6-stop/firmware/route-before.c
```

结果：

- `q4-regression-before.log`：上一轮 blanket CENTER_BRANCH 抑制实现退出134。前面的三路 drift 用例通过，随后在 `test_wider_center_branch_overrides_narrow_latch` 的四路应计数断言失败，独立证明本轮 Q4 缺口。
- `q4-regression-after.log`：当前宽度限定实现退出0；两种极性各1554项检查通过，包含窄漂移、两组四路/五路真实分支、中心重置、侧别/分离证据和原始极性链。
- `route-before-q4.c`：从当前源码仅反向去掉 `CENTER_BRANCH` 的宽度资格恢复上一轮 blanket 判定，用于本轮 Q4 前后对照，不修改生产源码。
- `regression-before.log`、`route-before.c`：更早的 EDGE-only 历史实现，退出134（三路 drift 已错误补计）；仅保留历史证据，不作为本轮 Q4 基线。
- `regression-after.log`：初版专项历史结果（1396检查），保留原日志；最终结果以前述 q4 日志为准。
- `route-width-fix.patch`：从 EDGE-only 历史实现到当前判定的差异；`route-q4-fix.patch` 为从上一轮 blanket 抑制到当前宽度限定判定的差异。

## 边界

这是离散八路的宽度启发式，不是元件几何或语义识别。真实四路图形仍可能被误计，三路真实支线仍可能漏计；仿真传感器间距、切角轨迹和 Q4 入口也未由本测试证明。专项只验证 route 判定，没有重新运行完整主机套件或交叉构建，也未烧录、未操作硬件。完整航向交接与后续索引结果由主任务单独验证，不能归因于本项修改。