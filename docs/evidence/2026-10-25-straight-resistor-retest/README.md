# STRAIGHT 吞掉后续 LEFT 的现场与修正

2026-10-25 用户反馈该左转的路口直行，且 STRAIGHT 保护不应降速。最初 ST-Link 识别到约 3.22 V，但无法访问 MCU；用户重新连接后成功。仅 GDB 短暂停核读取并 detach 恢复，未复位、未烧录、未写运行变量。

读取前 ELF 保存于 `/tmp/car-resistor-retest/firmware.elf`，导出二进制 27516 B 与板上对应全部字节一致，差异 0。SHA-256：`0b48f95bd21b3af72dd165f43f90fc75610be48e672988c972165ab5581dbe7a`。原始 Flash 和旧 ELF 在上述临时目录，可能被清理；原始诊断见 [gdb.txt](gdb.txt)。

停车原因 LINE_STOP_ROUTE / ROUTE_FAILURE_ACTION_TIMEOUT，索引 1（第一个 STRAIGHT），停车前 EXECUTING / LINE_JUNCTION，4500 ms；line_mask=0xff，全目标歧义，出口投影累计 133.633453 cm，route_progress=161.222977 cm。左右目标约 30.024807/29.975193 cm/s，反馈 31.4822445/30.3617516 cm/s，无速度故障。auto_resume_count=0。表中的 LEFT 是索引 3，故这次现场是仍停留在第一个 STRAIGHT，并非已经进入 LEFT 后忽略转向。

上一转弯交接至索引 1 时已继承 27.8334751 cm，实际启动 STRAIGHT 时路段进度约 27.863 cm；原表为 20±10 cm。旧逻辑从此触发点重新计算后续起点，且必须中心线退出，在没有中心线的矩形图形内长时间占用直行动作。当前观测不能证明完整传感器历史，但停留阶段和累计距离已直接确认。

修正将显式 STRAIGHT 作为测距后的里程通过点：全段保持路段轴、冻结位置 PID；窗口内开始动作，出口位移从本项标称里程点起算，越过标称点后进入距离离口，达到 min(3 cm, 下一项窗口起点) 即交接，无需中心线。下段仍保留正常路口/转向/STOP 确认。非最终轮 STOP 映射为 STRAIGHT 不采用该无形状里程通过语义。

FOLLOW / EXECUTING / CLEARING 全阶段使用本项速度目标，取消 STRAIGHT 的接近、转弯、干扰保持和曲线降速；满速航向纠偏钳外轮而允许内轮降低目标。1 s 启动/恢复包络、轮速上限、反馈/时序/按键停车及动作/离口超时保留。后续转弯自身减速仍生效。

新增回归：两个 20 cm STRAIGHT 在无中心线、多区域/无线/左右单点反复切换下，按40 cm标称总里程推进到 LEFT/RIGHT；实际极性转换与多数滤波；80 cm/s 目标不降速且满速能纠偏；下一弯正常确认；本次滞后交接数据回放；紧邻下一路口及静止超时/越窗保护。旧中心线退出断言按新语义调整，STOP 循环确认规则保留。

修正版未烧录、未实车复测。该策略依赖已测距离和编码器航向，不再对显式 STRAIGHT 逐个确认真实路口。

最终验证：`sh test/run_host_tests.sh` 全部通过，两种极性下导航各 323582 项检查，三个模式、ASan/UBSan、丢线与地缝开关回归通过；`pio run`（genericSTM32F103C8）成功，Flash 27512/65536 B、RAM 1448/20480 B；`git diff --check` 通过。README 文字与 Mermaid 图示已同步。路径表、圈数和生产速度参数保留用户当前配置。
