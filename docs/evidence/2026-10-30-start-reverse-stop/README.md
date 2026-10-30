# 2026-10-30 无法启动：START 接受后右轮反向保护

## 只读现场

沿用用户 ST-Link 读取授权，不检查固件一致性、不读 Flash、不复位、不烧录，
不写运行变量或请求 START。1000 kHz SWD 短暂停核读 SRAM 4096 B（约 62 ms），
同时读取 GPIOB IDR/ODR，立即 resume 并关闭 OpenOCD，未冻结看门狗。
电压约 3.240 V。原始数据为 `ram.bin`，读取日志为 `openocd.log`。
依本地 ELF 符号和类型离线解析为 `decoded.txt`；未检查板上版本，结论以该布局适用为前提。

## 关键数据

- `button_press_count=1`、`start_request_count=1`、`start_result=LINE_START_ACCEPTED`。
- START 时刻（路径 segment_ms）3657 ms，首次停车采样4472 ms，相隔815 ms。
- `stop_snapshot.reason=LINE_STOP_ENCODER`，`speed_fault=SPEED_FAULT_REVERSE`、wheel=2（右轮）。
- 停车前 TRACKING、路径 FOLLOW、index=0、junctions_seen=0、route_failure=NONE。
- 有效单区域线 `0x08`，position=-1，可靠线年龄0 ms。
- 左右目标0 / 2.60788321 cm/s，左右实测0.648663521 / -0.593795598 cm/s。
- 计数 left=2 / right=65533；计数绝对值不能替代当帧增量与完整运动历史。
- 当前生产反向阈值为 -0.5 cm/s，右反馈低于阈值，触发立即保护。
- IMU init_failure_stage=2（地址/身份探测阶段），初始化尝试3次；首控制帧1542 ms锁定编码器降级。
  该 IMU 失败没有阻塞 START。降级后俯仰监测不可用。
- 读取时当前灰度已为无线，不能用它倒推 START 时不在线；START 时灰度为 `0x10`。
- GPIOB IDR=`0x0f50`（本次采样PB10为高），ODR=`0x0413`（PB5 STBY为低）。

## 结论与边界

本次不是启动门禁拒绝，而是启动后约0.815秒右轮反向反馈导致锁定停车。
当前策略下再次 START 不能清除编码器故障，需排除原因后复位重新初始化。
助手没有执行复位或重新启动。

反馈仅比阈值多负约0.094 cm/s，低速抖动/回弹/外力、实际轮向或编码器信号异常
均需进一步分辨。快照不保存完整计数序列，不能认定真实持续反转，
也不能据此反转右编码器极性、改为绝对速度或关闭反向保护。
本次不修改固件与保护阈值。需核对右轮前进时原始计数是否按既定 -1 极性递减，
以及启动时是否有手动转轮/挪车/负载回弹；单个绝对计数不足以判断接线。

`reproduce.c` 只在主机复现快照速度输入触发右侧保护及编码器故障锁定，
不能证明硬件负反馈的来源。另行运行实际生产配置 START 测试检查正常启动门禁。

验证完成：两项主机测试均在 ASan/UBSan、`-Wall -Wextra -Werror -pedantic`
下编译并退出0，结果见 `host-tests.log`；生产配置测试包括 IMU 首帧失败可启动、
无线及非法路径拒绝。`git diff --check` 通过。本次未修改固件或配置，未重新构建/烧录固件。