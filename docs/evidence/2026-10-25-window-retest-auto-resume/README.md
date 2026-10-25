# 短段修正复测仍越窗与停车后在线恢复

2026-10-25 用户反馈“修改后还是卡在那里，在线上自动继续没有生效”。ST-Link 只读检查短暂停核后 detach 恢复，未复位、未烧录、未写运行变量。原始快照见 [gdb.txt](gdb.txt)。用户本轮明确选择“路径原因停车后，重新检测到连续有效线也自动恢复当前段”，覆盖原 STOPPED 必须明确 START 的约定，仅用于下述可恢复路径停车。

## 版本与快照

读取前保留 ELF，objcopy 二进制 26628 B 与板上 Flash 对应全部字节一致，差异 0，确认已运行上一轮短段修正（含第 4 项关闭圆弧及短段规划轴继承）。

- ELF 二进制 SHA-256：`2e36b3af376492fa3d19417e09318e334f95dd1c9013c3bb012573a79c3d3b45`。
- 全 64 KiB Flash SHA-256：`ed63bef325c16f7597f0b317a83e2d8880225535ecbf3b62815afedeed550612`。
- 原始 ELF / Flash / RAM 本机目录：`/tmp/car_stm32_retest_20261025/`；临时目录可能被清理。

停车前：首轮第 5 项 LEFT（索引 4），`LINE_TRACKING / ROUTE_FOLLOW`，`LINE_STOP_ROUTE / ROUTE_FAILURE_WINDOW`；进度 30.0835876 cm 超出 20±10 cm 上界。灰度 0x10、单一区域、position=1，速度故障 NONE。目标 20.2880/17.7120 cm/s，反馈 21.7214/14.4756 cm/s。上段交接进度 17.5270348 cm，比上一轮 21.6004162 cm 提前，但仍未在原窗口内确认当前路口。

读取时灰度 0x04、有效单线 position=-3，连续有效计数为 3，车辆仍 STOPPED。原“在线继续”只在 ROUTE_EXECUTING 中处理，FOLLOW 越窗先进入 ROUTE_FAILED；停止状态只接受按键/调试 START，所以不会自动继续。不能把旧规则解释为停车后自动恢复已经生效。

## 新的自动恢复语义

`include/motion_config.h` 新增 `ROUTE_AUTO_RESUME_ENABLED=1`。只在 STOPPED、停止原因 LINE_STOP_ROUTE，且停车后相邻合法周期连续 3 帧有效单一区域目标线时自动恢复。无线、超宽、多区域或采样间断清空连续证据。

- WINDOW / CONFIRM_TIMEOUT：保留当前索引、圈数、规划轴、原始里程基准；回到 FOLLOW，以恢复时里程另外建立长度 `max(2×本项tolerance, 灰度前置距离)` 的检测区间。当前第 5 项为 20 cm，即从恢复位置向前 20 cm 内等待当前路口，防止立刻再次触发旧 30 cm 上限。原 route_progress_cm 不重写。此阶段基础目标不超过 JUNCTION_APPROACH_SPEED_CM_S=20 cm/s。
- EXECUTING / CLEARING 的 ACTION_TIMEOUT：重新获得稳定单线后进入 CLEARING，重建独立 1500 ms 离口预算及离口距离，仍完成原去重条件才推进表项；不在没有路口证据时提前消费 FOLLOW 中的路径项。
- 不恢复最终 STOP、按键停车、丢线停车、非法路径、过转/反向、编码器及控制故障。上电仍不自动启动。人工 START 仍从第 0 项清零圈数开始。
- 输出从零按 1 s 启动包络渐升，重置位置/速度 PID 历史。连续在线但一直没有路口时，恢复窗口耗尽仍停车；重新满足三帧单线可再次续当前项。不能宣称窗口偏差或路口识别的物理根因已经消除。

恢复前快照复制到 g_diagnostics.auto_resume_snapshot，当前 stop_snapshot 清空，使下次停车能锁存新证据；新增 auto_resume_count / auto_resume_last_ms / route_recovery_window_active / route_recovery_window_progress_cm / route_recovery_window_limit_cm。start_result=7 为 LINE_START_AUTO_RESUMED。人工 START 清理这一恢复诊断会话。

## 验证

完整 sh test/run_host_tests.sh 通过，两种极性下导航各 344584 项检查；覆盖本次索引4冻结状态的停车→三帧在线→当前项继续→原窗口外确认路口、圈数/里程保留、断续/歧义/迟到帧拒绝、故障与最终STOP不恢复、新快照更新、人工START归零及动作/离口超时恢复。额外以 ROUTE_AUTO_RESUME_ENABLED=0 编译运行导航套件，通过 343435 项检查。

pio run（genericSTM32F103C8）通过：Flash 26960 B / 65536 B，RAM 1448 B / 20480 B。git diff --check 通过。修正版未烧录、未进行实车自动恢复或赛道验证。
