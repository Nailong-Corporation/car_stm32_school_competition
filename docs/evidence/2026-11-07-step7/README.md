# 2026-11-07 第7步右下角越界修复与减速惯性

## 结论

右下角越界已修复：`x=150` 的首个中间支线仍可能在转弯后因中心历史过期而漏计，但新增的**有界段末回退**会在前探头已经到本段末端时允许最终动作。第7步在 `x≈350` 看到右边框即按目标左转，不再把右边框当第3个节点直行到 `(382,220)`。

这不是把 `junction_count` 改小，也不是把所有路口改成距离触发；回退只对“计数仍不足、前探头已到段末15 cm内、非直行最终动作”生效，普通中间节点仍必须靠灰度确认。

减速惯性恢复为**滑行后转弯**：仿真器 `ROUTE_BRAKING` 不再强制把实际轮速置零，而是保持零目标、按电机时间常数 `MOTOR_RESPONSE_TAU_S=0.040 s` 衰减；实测轮速降到入口门禁后，固件逻辑开始转弯。因此灰度看到线后不会立刻停住，而会带着惯性越过线，再按固件转弯。此前“双轮≤0.5 cm/s才放行”的强制停车实验已移除。40 ms 是当前路径/回归仍稳定时的上限；继续增大到42 ms会重现首弯早计数。

## 证据

- [`step7-final-trace.log`](step7-final-trace.log) 记录：`x≈350.1` 进入 `ROUTE_CANDIDATE`，下一帧 `junctions_seen=3`、`ROUTE_EXECUTING`，在 `x≈351.4` 开始左转；`x≈363.5` 进入 CLEARING，`x≈364.5` 推进到索引8，轴未超过 `x=375`。
- 回归 `tests/q4_q6.test.ts` 新增方案1右下角用例：必须在右边框前进入 EXECUTING、无 `navigation_recoveries`、完成后索引8且 `x<376`。
- 仿真完整测试16项 PASS；`npm run build` 成功。
- C 完整主机套件、`pio run` 见本目录日志；C 的 `JUNCTION_TARGET_FALLBACK_CM=15` 与仿真同规则。未烧录、未操作电机。

## 仍未修复：入环前30 cm短段

第8步在 `x=375` 左转完成时车轴约 `(361.8,130.0)`；第9步目标 `x=345`、长度30 cm，24 cm前探头已到约 `x=337`，目标已在探头后方。车辆之后在 `x≈300` 把环岛左侧结构当目标，误左转进入环岛左侧。该问题需要改路径表短段/触发点，或引入短段里程先验；本轮没有擅自改用户路径表，也没有把该短段改成距离盲转。

## 速度

- 动作内普通转弯仍为 `JUNCTION_SETTLE_SPEED_CM_S=16`；此前直接提高到18～20 cm/s会在首弯后 `x≈14` 再次早计数，未采用。
- 已确认出口后的 CLEARING 为 `JUNCTION_EXIT_SPEED_CM_S=20`。

## 验证文件

- [`host-tests.log`](host-tests.log)
- [`pio-build.log`](pio-build.log)
- [`simulator-tests.log`](simulator-tests.log)
- [`simulator-build.log`](simulator-build.log)

- [`step7-final-trace.log`](step7-final-trace.log)：右下角最终事件（25.600s候选350.1、25.680s EXECUTING 351.4、27.355s索引8、轴364.5<376）。
