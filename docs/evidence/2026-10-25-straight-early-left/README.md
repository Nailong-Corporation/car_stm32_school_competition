# 第二个 STRAIGHT 提前左转：距离门禁与位置控制保护

2026-10-25 用户反馈“在第二个Straight就左转了”，并明确确认第二个 STRAIGHT 是电阻出口，出口后还应再直行 20 cm 才到 LEFT。

ST-Link 只读连接，短暂停核后 detach 恢复，未复位、未烧录、未写运行变量。修改前 ELF 二进制 27788 B 与板上相应全部字节一致，差异 0；SHA-256 `1185a71d37b16a259720cecded07e91d2a668ce129ce298aca4ced57c759050d`。旧 ELF、全 Flash 在 `/tmp/car-straight-early-left/` 临时目录，可能被清理；诊断见 [gdb.txt](gdb.txt)。

读取时停止原因是 LINE_STOP_ROUTE / ACTION_TIMEOUT，索引 5（CUSTOM_TURN +120°），EXECUTING 4000 ms，0xff 歧义；auto_resume_count=3。保留的 auto_resume_snapshot 是索引 4（CUSTOM_TURN -60°）动作超时。它们已晚于索引 3 LEFT，因此没有左转触发瞬间的快照，不能把本次读取直接称为提前左转的物理根因证明，也不修改这两个后续超时的策略。

源码中，LEFT 表项 20±10 cm 在前进 10 cm 时即允许分支候选或骤变强制入口；此前距离通过修正没有限制 STRAIGHT 后的提前转向，位置 PID 也可能在切项后跟随出口侧线。此次按用户确认的几何，紧跟 STRAIGHT 的 LEFT/RIGHT/CUSTOM_TURN 在走满本项标称距离前禁止这两类路口入口，同时冻结位置 PID、保持上一项出口轴。到标称位置后恢复原转弯确认与保护，20±10 cm 因此在 20～30 cm 接受转向路口，保留越窗停车。自动恢复不绕过标称距离门槛；其他普通转弯与最终 STOP 保留原规则。

新门禁结束时建立新的普通丢线时间基准、清除旧连续观测与 PID 历史，不伪造稳定线证据；有意无中心线直行不消耗下一阶段的丢线预算，但到点后无线仍按 400 ms/8 cm 保护停车。STRAIGHT 的表速通过和满速纠偏保持，下一 LEFT 自身接近减速仍正常生效。路径表及用户参数未调整。

回归覆盖：连续两个 20 cm STRAIGHT 后，20 cm 之前的全排、多区域、左右侧线及无线不能派发转弯，也不能通过位置 PID 产生侧线轮差；20 cm 精确边界普通候选/强制入口仍能执行，左右与自定义正负角均覆盖；自动恢复不绕过门槛，超上界仍停车；其他转弯保留提前容差；无中心线门禁释放后的新丢线预算及停车；80 cm/s STRAIGHT 回归仍保留。

最终验证：完整 `sh test/run_host_tests.sh` 通过，两种极性下导航各 325576 项检查，包含 ASan/UBSan、三个模式及丢线/地缝开关回归；`pio run`（genericSTM32F103C8）成功，Flash 27704/65536 B、RAM 1448/20480 B；`git diff --check` 通过。README 及 Mermaid 同步更新。修正版未烧录、未实车复测。
