# 2026-10-29 第一项 RIGHT 过转停车

通过 ST-Link 读取，未复位、烧录或操作电机；短暂停核后 detach 恢复运行，随后关闭 OpenOCD。
读取前本地 ELF 转出的 28520 字节 binary 与板上 Flash 对应全部字节一致，SHA256 为 `7ab2258e9ceb381c951ad6696e705b899d6bf21185e04b8cc857d97ef7d6e062`。新构建尚未烧录。

停车快照：

- `reason=LINE_STOP_ROUTE`，`route_failure=ROUTE_FAILURE_TURN_LIMIT`。
- `route_index=0`，更新前 `ROUTE_EXECUTING`，本项 RIGHT / corner_arc_enabled=false。
- `sample_ms=6235`，动作年龄 585 ms。
- `turn_rad=2.10115075`，上限 2.1 rad，约 120.39° / 120.32°。
- 灰度 `GRAY_VALID`，mask=0x20，position=3，中心确认计数为 0。
- 出口候选 105 ms / 1.08709717 cm，但尚未中心确认。
- 左右目标 5.79395914 / 6.20604086 cm/s；实测 12.1585073 / 3.78320765 cm/s。

直接停车原因是过转。旧普通扫转沿用 60 cm/s 基础值，初始外轮可达 120 cm/s；无线末段仍固定单轮同向扫转，偏侧捕获时位置项又可能抵消航向回正。快照证明停车瞬间实际轮差仍使车右转，但没有整段速度历史，不能确定各因素的贡献或声称已标定速度环。

修正：非圆弧基础上限独立设为 6 cm/s，约 72° 后无线也按剩余航向收敛；普通直角超过目标角且捕获偏侧线时，保证航向回正不被位置项抵消。保持原过转/反向保护和退出规则。回归覆盖左右镜像、高速入口目标下降、无线超角回正、position=±3 捕获、中心确认及硬上限停车；实车效果待复测。

验证结果：

- `pio run` 成功，Flash 28336 B，RAM 1656 B。
- 新增回归在修改前代码上失败于非圆弧基础速度上限断言。
- 两种灰度极性的导航专项（仅在临时测试副本中排除既有 `test_reference_path`，仓库测试入口保留该项）各通过 221545 次检查，启用 AddressSanitizer / UndefinedBehaviorSanitizer。
- `sh test/run_host_tests.sh` 未全通过：导航 `test_reference_path` 的综合条件断言失败；恢复本次修改前控制代码与原速度预期后，同一断言仍失败。前置传感器/PID、encoder HAL、motor HAL、生产配置启动测试通过；后续完整测试未执行。
- `git diff --check` 通过。未烧录，未进行实车复测。
