# 连续三路时停车：ST-Link 只读检查

2026-10-24 通过 ST-Link 短暂停核读取后 detach 恢复；未复位、未烧录、未修改运行变量。原始变量输出见 [gdb.txt](gdb.txt)。

停车前锁存快照显示：

- `reason=LINE_STOP_ROUTE`，`route_failure=ROUTE_FAILURE_ACTION_TIMEOUT`。
- `route_index=4`（第 5 项 LEFT），`route_state_before_update=ROUTE_EXECUTING`，阶段年龄 4500 ms；已完成轮数 2。
- 灰度 `raw_mask=line_mask=selected_mask=0x38`，`kind=GRAY_VALID`，`white_count=3`，`region_count=1`，`position=1`。连续三路有效，不是宽线歧义。
- 转角 -1.2843647 rad（约 -73.59°）；出口候选 pending=false、确认帧数 0。
- 左右目标 5.2 / 6.8 cm/s，实测反馈 5.74858 / 6.12315 cm/s，速度故障 NONE。

当前 LEFT 的直接退出门槛 1.35 rad；新建有界出口候选需至少 1.30 rad，之后需 50 ms / 0.25 cm 及最终连续中心线。快照角度距离 1.30 rad 仍差约 0.01564 rad（0.90°）。成熟证据的回差条件不允许无候选时直接绕过此门槛。快照不能还原此前整段灰度、轮速或出口候选清零原因；只能确认本次停止由执行阶段超时触发，不能把它归因于三路歧义。

读取 Flash 的前 24832 字节与本次本地构建比较仅两字节不同：`0x08001fde` 板上 0x06 / 本地 0x07（灰度区域宽度比较）；`0x080039d6` 板上 0x04 / 本地 0x05（有效单线宽度比较）。这对应板上最多连续 5 路、本地最多连续 6 路；其余比较字节一致，包括路径表与退出逻辑。本地 ELF 可用于此次变量读取。Flash 原始副本保存在本机 `/tmp/car_stm32_inspect_20261024/flash.bin`，临时目录可能被清理。

本次按用户要求将编码器单一连续区域上限改为 6，7～8 路歧义；不会改变上述三路出口超时。未调整转角、阶段时限或速度参数。同步修正出口测试中旧的六路超宽样例为全八路歧义样例，完整 `sh test/run_host_tests.sh`（两种极性、三个控制模式及相关保护）通过；`pio run` 和 `git diff --check` 通过。新固件未烧录，实车效果待验证。
