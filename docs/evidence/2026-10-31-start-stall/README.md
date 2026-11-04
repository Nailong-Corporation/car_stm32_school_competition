# 2026-10-31 启动后堵转停车，只读现场诊断

本次沿用已授权的 ST-Link 只读现场诊断：运行态 GPIOB ODR 读数全零，弃用；短暂停核读 GPIOB CRH/IDR/ODR 与 SRAM 0x20000000 起 12 KiB，然后请求 resume。未发送 START、复位、烧录、读 Flash 或冻结看门狗。保存读取前本地 ELF 以解析 148 B 停车快照；未比对板上 Flash，所以结果以本地布局适用为前提。两份软件停车快照逐字节相等；本地 ELF 仅供布局解释，不证明板上二进制一致。ST-Link 电压约 3.22 V，暂停时 PB5 STBY 低。

- 控制循环仍在运行：control_count=18381，last_sample_ms=92435。button_press_count=1、start_request_count=1、start_result=LINE_START_ACCEPTED；请求时灰度 0x10（单区域）。
- 停车快照：3195 ms，LINE_TRACKING -> LINE_STOP_ENCODER / SPEED_FAULT_STALL，wheel=1；路线 FOLLOW/index=0、无路径故障。按 83 帧 ×5 ms 的启动轨迹，START 约 2780 ms 被接受，约 415 ms 后堵转锁定；两轮所有轨迹样本计数 0/0、反馈 0/0、里程 0。停车前目标左 19.7125、右 16.1125 cm/s，采样灰度 0x10。
- IMU 初始化阶段 9 失败，535 ms 锁定降级编码器；这不是此次启动拒绝或停车的原因。braking_reverse_ms=0、braking_reverse_active=false、最后一段距离 0：新距离/反向制动逻辑未运行。
- 软件能证明 START 已接受并因无轮速反馈触发堵转保护，不能仅凭零计数判定电机未转、编码器 3.3 V 再次脱落、轮线拔开或供电异常。未读取板上 Flash，也没有在运动中测量实际电机电压/电流。保持堵转保护，不为启动强行放宽阈值。

下一步：断电确认电机插头、编码器 3.3 V/GND 与 A/B 相接触；架空上电手转左右轮观察 `left_encoder/right_encoder` 是否变化，再架空低速试运行核对正向计数与实际轮转，最后回到赛道。编码器故障锁定须排除硬件原因后复位；本次未执行复位或试车。原始 `ram.bin`、离线 `ram.core`、读取时 `read-time.elf`、`decoded.txt` 与 `openocd.log` 同目录保存。