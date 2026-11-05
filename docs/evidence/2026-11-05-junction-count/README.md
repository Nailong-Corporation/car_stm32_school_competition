# 2026-11-05 小车不数路口、见到路口就转

## 现场读取边界

用户授权 ST-Link 只读并明确不要求检查固件一致性。PB5/STBY 低时短暂停核，读取 SRAM 16 KiB、route 表 512 B 及 `route_target_ready`/`route_update` 少量 Flash 后立即 resume；未复位、烧录、请求 START 或写控制变量。按读取前本地 ELF（18:50 构建）解释布局；未做整片 Flash 比对，不宣称板上二进制完全一致。

## 现场状态（20:43 读取）

- `follow.state=LINE_STOPPED`，停车原因 `LINE_STOP_ENCODER`；停车前 `route.state=ROUTE_EXECUTING`、`route.index=0`、`junctions_seen=4`、`navigation_recoveries=3`、`navigation_warning=5`（`ACTION_TIMEOUT`）。
- START 于约 1.2 s 接受，20.978 s 因轮速反向锁存停车；动作入口约 244.93 cm，停车约 247.71 cm，`route.entry_heading`/动作起点使估计转角仅约 -0.166 rad。
- 最后 640 ms `navigation_trace` 全为无线，不能据此反推首弯捕获过程；`turn_trace` 显示 18.4～18.8 s 曾在约 1.10 rad、`line_mask=0x1c` 下持续位置巡线，随后该动作超时。
- route 表为工作树版本：`CUSTOM_TURN -90`、计数 1/4/2/3、长度 60/180/60/200。首项计数 1，任何后续路口都会立即满足目标。

## 源码机制

1. `CUSTOM_TURN` 未到目标角时，出口线被提前捕获后位置 PID 接管转向；在约 63°（1.10 rad）时剩余角度只靠位置纠偏推动，3 s 动作时限内未到 90°，触发 `ACTION_TIMEOUT`。
2. 恢复保留 `index=0` 和已满足的计数 1；车辆继续前行，下一个路口再次立即成为目标，于是表现为“不数路口、见口就转”。
3. 同时 `route_target_ready` 原为“计数 OR 距离”，计数 4 的项在距离先到时会提前授权，进一步表现为不数够路口。

## 修正

- `route_target_ready`：`junction_count>0` 只按计数授权；`segment_length_cm` 只用于速度/减速包络和距离耗尽后的低速搜口；计数 0 旧表项仍按长度。
- ±90° `CUSTOM_TURN` 与 LEFT/RIGHT 几何相同：捕到连续居中窄出口后立即停止强转、交回位置 PID 并进入 CLEARING；非90°自定义角仍要求到角。避免捕线后继续强转、越过出口线丢线。
- 路径表未改；普通转弯、直行、HEXAGON、入口门禁、阶段时限、丢线/图形恢复及全部反馈保护未放宽。

## 验证

- 主机：`sh test/run_host_tests.sh` 退出 0（93 行 PASS）。新增自定义 90° 未到角继续航向控制、距离耗尽后仍须第 4 个路口才执行的回归。
- 最小对照（`old_vs_new_target_ready.c`）：真实表第 2 项（计数 4、长度 180）在总段距离已到 200 cm 时只确认第 1 个路口。修正前 `passing=0 / state=ROUTE_EXECUTING`（错误地立即转向），修正后 `passing=1 / state=ROUTE_CLEARING`（直行通过）。
- 真实表全流程（索引推进）：J1 目标执行；J2～J4 计数 1～3 直行通过；J5 计数到 4 才执行；J6 直行、J7 计数 2 执行；J8/J9 直行、J10 计数 3 STOP 完成。
- 构建：项目内 core `pio run` 成功，Flash 42604 B / RAM 10356 B。
- 未烧录、未操作电机；实车转角完成、计数与出口效果待复测。

## 文件

- `ram-live-2043.bin`：本次 20:43 现场 SRAM 16 KiB。
- `route-table-live-2043.bin`：0x0800A200 起 512 B，含实际 route 表。
- `old_vs_new_target_ready.c`：修正前后 `route_target_ready` 语义的最小对照用例。
- `ram.bin`、`ram-first.bin`、`read-time.elf`、`navigation_trace.csv`：此前 19:09～19:32 的只读记录，保留作对照。
