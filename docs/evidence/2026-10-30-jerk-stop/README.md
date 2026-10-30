# 启动即停：速度斜坡被位置曲线钳死（2026-10-30）

## 现场与操作边界

用户反馈“启动后电机动一下就停止”。沿用只读现场授权：不复位、不烧录、不发送 START/STOP、不写控制变量、不操作电机。STBY 预检查为低后短暂停核读取 SRAM 前 4096 B 并请求 resume，未读 Flash、未冻结看门狗、未核对固件一致性。原始数据 `ram.bin`、日志 `openocd.log`，读取前本地 ELF 保存为 `read-time.elf`。

本次按当前本地 ELF 解码，控制器与诊断两份 `stop_snapshot` 128 字节逐字节相等、当前时基/状态吻合，布局解释一致；仍未读 Flash，不宣称板上二进制一致。

## 现场数据

| 项目 | 结果 |
| --- | --- |
| 按键 / START | 各 1 次，`LINE_START_ACCEPTED` |
| 请求灰度 | raw/line = 0x10（S4 单区域，position=+1） |
| START 时刻 | 8087 ms（`route.segment_ms`） |
| 停车时刻 | 9282 ms，START 后 1195 ms |
| 停车原因 | `LINE_STOP_ENCODER` / `SPEED_FAULT_STALL`，wheel=1（左轮） |
| 停车目标 | 左 14.319 / 右 10.719 cm/s，base 12.519 |
| 停车实测 | 两轮均 0 cm/s，`distance_cm=0`、左右轮程 0 |
| 编码器原始计数 | 停车帧与读取时均 0/0，全程无计数 |
| 速度斜坡 | `braking_slew.speed=15.073`、`acceleration=160`（已饱和） |
| 启动包络 | `startup_elapsed_s=1`（已走完） |
| 路径 | FOLLOW / 索引 0 / 计数 0，无路径故障 |
| IMU | 阶段 2 探测失败，1542 ms 降级编码器，未阻塞启动 |

## 源码可复现根因

`src/line_follow.c` 的最后路段速度规划里有一行：

```c
f->braking_slew.speed = fminf(f->braking_slew.speed, fmaxf(left_target, right_target));
```

而实际轮目标又由「斜坡 × 位置曲线 `(1 - 0.35·|pos|/7)`」经 `next_outer` 算出。两者相扣形成衰减反馈，收敛于：

```
S = rise·dt / (1 - curve) = 16 / |位置偏差|  cm/s
```

| 位置偏差 | 目标速度收敛值 |
| --- | --- |
| 0 | 正常升到巡航包络 |
| 1 | 约 15–16 cm/s |
| 2 | 约 8 cm/s |
| 3 | 约 5 cm/s |

现场 `position=+1`、斜坡 15.073、加速度已饱和在 160，与理论收敛完全吻合；`startup_elapsed_s=1` 说明包络早已走完却仍停在 15 cm/s。目标过低使前馈 + PID 的 PWM 只有约三成，车轮未能持续转动，350 ms 堵转保护停车——与“动一下就停”一致。

同一机制也解释历史证据中的“基础目标仅 14 cm/s”“进入离口后仅走 0.29 cm”。

## 修正

删除该钳位，速度斜坡只跟踪速度包络：

- 包络下降仍由 `braking_slew()` 的 `cap` 即时钳低，接近路口不会超速。
- CANDIDATE / 搜口 / 离口的状态限速仍由独立 `fminf` 处理。
- 目标上升仍受 `next_outer`/`next_base` 的 `rise·dt` 限制，不会跳变。
- 堵转、反向、路径、丢线、时序保护与全部阈值保持不变。

## 验证

- 新增 `test/test_navigation.c::test_final_leg_ramp`：位置偏差 +1 持续 400 周期应加速超过 50 cm/s；包络骤降必须即时降到 `turn_speed`（该段仅在 `TURN_APPROACH_SLOWDOWN_ENABLED` 下检查）。
- 旧源码（恢复该钳位）该用例断言失败、退出 134，见 `regression-before.log`；修正后通过，见 `regression-after.log`。
- `sh test/run_host_tests.sh` 与 `pio run` 结果见 `host-tests.log`/`.exit`、`build.log`/`.exit`；`git diff --check` 通过。
- 证据与日志归档于本目录；`ram.bin` 为原始 SRAM，`decoded.txt` 为按 `read-time.elf` 的解码。

## 未验证边界

本次只证明软件机制可复现并已修正，**未烧录、未操作电机、未做实车复测**。现场两轮编码器计数全程为 0，仍需在电机插头重新接好后确认编码器确实计数（手转轮子观察 `g_diagnostics.left_encoder/right_encoder`）；若修正后车轮仍不动，应按机械/接线/供电排查，不靠放宽堵转阈值掩盖。LineFollow 结构本次未再改变，历史快照布局不受影响。