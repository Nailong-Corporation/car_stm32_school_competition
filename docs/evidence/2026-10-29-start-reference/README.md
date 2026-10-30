# 2026-10-29 参考测距工程修正右编码器符号

用户反馈仍无法启动，并提供 encoder_distance_test_stm32/platformio.ini。该文件作为参考配置读取，未将参考文档视为额外指令；未修改测距工程。

## 当前板上证据

通过 ST-Link 获取 Flash 和 SRAM 后恢复运行并退出 OpenOCD，未烧录、请求启动或写控制变量。读取日志见 read.txt。Flash 前 37,216 B 与读取前保存的 before.bin 逐字节一致，核对见 verify.txt，因此用 before.elf 解读本次 SRAM。snapshot.core 是从转储构造的离线 ELF 内存容器，只有内存段，没有 CPU 寄存器；diagnostics.txt 的变量来自该容器，不是 live GDB 或带寄存器的真实崩溃转储。两个内存区域依次读取，不承诺跨区原子快照。

- START 已接受，按键/请求各一次；LINE_STOP_ENCODER，首次停车快照为 SPEED_FAULT_REVERSE，轮侧 2（右）。
- 停车时仍为有效单线 0x08，左右计数 16/65535，速度 +4.53001881/−0.593795598 cm/s，目标 17.2/20.8 cm/s。
- IMU 已就绪，初始化失败阶段 0，俯仰未锁定；本次不是 IMU 初始化或无线启动拒绝。
- 当前停稳计数 17/65513，不替代停车快照。计数方向不能单独证明物理前进方向。

## 工程对照与修改

两个 platformio.ini 完全一致，均 genericSTM32F103C8/stm32cube/64MHz HSI 配置；motor.c、encoder.c 完全一致。电机正指令极性均左 IN1 低、右 IN1 高。

测距工程编码器符号左 +1 / 右 −1，而本工程左 +1 / 右 +1。结合板上右原始计数从 0 回绕到 65535，将本工程 ENCODER_RIGHT_SIGN 改为 −1，使该负计数增量换算为正轮速。左符号保持 +1，不修改电机极性、IMU、启动条件、距离比例、速度或反馈保护。README 同步当前符号与证据。

## 验证与边界

test_configured_start.c 新增实际生产标定下左正计数、右 0→65535 的序列，要求两侧反馈为正、无速度故障且继续 TRACKING/输出使能。临时复制头文件恢复旧右 +1 后，此用例在右轮正反馈断言处失败；新符号在巡黑/巡白 × IMU 开/关四种组合全部通过（严格警告及 ASan/UBSan），见 focused.log。无线、路径速度超限及 IMU 无效门禁仍覆盖。

pio run 成功：Flash 36,940 B，RAM 3,816 B，见 build.log。git diff --check 通过。完整入口 sh test/run_host_tests.sh 中生产启动和实际 MPU 驱动测试通过，随后在已有 test_heading_scale.c:26 失败，见 suite.log。该测试使用独立硬编码符号 −1/+1，断言预期距离比例 1.53，而当前配置 2.15；修改右生产符号不改变该用例的标定符号，本次未修改距离比例或掩盖失败。

新固件未烧录，无实车启动成功证据。后续须将新固件另行烧录后，确认两轮物理前进与两侧修正反馈为正。软件计数回放不替代架空或赛道验证。
