# 2026-10-24 IMG_1010 短段交接修正

## 板上诊断（修改前）

ST-Link 1000 kHz，目标电压约 3.245 V。未复位、未烧录；短暂停核读 SRAM 后 resume，随后再次短暂停核读代码与路径表后 resume。SRAM 保存为 `2026-10-24-short-handoff-stop-ram.bin`，离线按读取时 ELF 解析为 `2026-10-24-short-handoff-stop-decoded.txt`。

- 路口代码区 0x08003800 起 4096 B、表区 0x08005698 起 180 B 均与读取时本地 ELF 对应区域逐字节一致（不是整个 Flash 校验）。
- 前三项：76±5 cm / +90°；82±5 cm / -90°；10±5 cm / -60°。
- 停车原因 LINE_STOP_ROUTE / ROUTE_FAILURE_WINDOW；index=2，停车前 LINE_TRACKING / ROUTE_FOLLOW。
- 停车采样 22845 ms，route_progress=15.0127563 cm，超过第三项 15 cm 上界。
- 切段时间 22795 ms，继承 outgoing=14.2362957 cm，仅余 0.7637 cm 窗口；50 ms 后越窗。
- 停车前 line_mask=0x60，单区域两路有效目标线，position=4；左右实测轮速 19.6408577 / 12.1928759 cm/s，speed_fault=NONE。
- 此次上一弯进入 CLEARING 的时间 22510 ms，clear_cm=174.87355；旧代码在 178.704147 cm 才切段。不能从单次快照还原离口全过程的传感器历史。

## 程序修正

保持用户九项路径表、所有距离/容差/角度/速度与既有丢线开关。普通离口仍为 3 cm；若已确认转弯出口、下一路口进入标称位置前 5 cm 且尚未越过窗口上界，允许离口前进至少 0.5 cm 且连续 3 帧单一区域窄线后交接下一项。直行及小于 JUNCTION_CAPTURE_MIN_RAD 的转角不能使用早交接。无/宽/多区域线重置连续证据，出口线在探头范围内横移不重置窄线保持。

早交接保留真实出口方向投影及下一段原窗口，不把进度归零或扩大容差。转交已验证的相邻窄线历史，使紧邻下一弯仍可按原规则响应整排骤变；中心分支记忆仅在当前帧确实居中时建立。无路口证据不会只因到达里程而转弯，过期帧不触发骤变，越窗/错误转向/动作与离口时限继续生效。

新增 RoutePlanner 和停车快照字段 handoff_early / segment_start_progress_cm；后者保存切段时继承的出口方向进度。

## 软件验证与边界

- 新短段回归在修改前失败：0.51 cm 离口后仍留在上一项。
- 修改后覆盖 10/15 cm 短下一段、左右镜像、偏侧窄线、0.5 cm 最小距离、帧数、中断/过期证据、歧义/无线、静止、直行动作、远路口、已越窗、离口独立超时、下一项强制转向及控制器非负目标输出。
- 旧四项完整理想运动学回归改为明确的固定参考表；生产九项表仍由 test_configured_start 使用实际配置验证启动。循环测试显式设置测试轮数，不依赖用户生产值。
- `sh test/run_host_tests.sh` 全部通过：黑/白两种极性、三个控制组合、丢线开关两个分支；导航各 342380 checks。
- `pio run` 成功：RAM 1116 B，Flash 23508 B。
- `git diff --check` 通过；README/导航说明和状态图同步。

未烧录，板上仍为读取时旧版本。本次是软件修正与合成输入验证，不能宣称实际赛道已通过；更早交接是否足以捕获该紧邻路口，仍需上传后实测。

## 文件指纹

- car-1010-before.elf: `76e3030d71a4f62d3a0fe0f48a5bca318faba10cb7750baea1aa96689dc55df5`
- 2026-10-24-short-handoff-before-code.bin: `52562bf9eb187a81a0fc94cf346cf94d4629c906bb86611b1a7a175883a33799`
- 2026-10-24-short-handoff-before-table.bin: `83b7b848e6b464e55fe10d063bac529c1938aac4a900ff5f40760459f34d040e`
- firmware.bin: `6c3a4daeac49dfe0b666a5383e4a448fe4f9a63720607ebf49833839271f28dc`
