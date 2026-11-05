# 2026-11-05 在线无法启动：ST-Link 现场

用户请求“查看 stlink 日志，车在线上无法启动”。本次通过 OpenOCD/ST-Link 1 MHz SWD 短暂停核，读取 PB5/STBY 为低（GPIOB ODR=0x713）后保存 SRAM 起始 12 KiB，并执行 resume/shutdown。未复位、烧录、请求 START、写控制变量、冻结看门狗或读取 Flash。OpenOCD 退出 0，ram.bin 长度 12288 字节。日志未输出 mdw 的 GPIO 输入读数，不据此报告输入电平。

读取前保存当前本地 firmware.elf 为 read-time.elf。ram.core 为 ram.bin 构造的离线 ELF32 内存容器，不含 CPU 寄存器；GDB 缺少寄存器警告为预期。未核对 Flash，不能保证板上固件与本地 ELF 完全一致；诊断与 follow 的状态、启动结果、灰度、降级来源吻合，支持本次布局解释。

## 现场结果

- 采样时刻 94568 ms，控制间隔 5 ms；IDLE，LINE_STOP_NONE，无停车快照，轮目标/命令为零且输出禁用。
- 按键事件 6 次，START 请求 6 次；最近结果 LINE_START_REJECTED_LINE。请求时 raw=0x0C，line=0xF3。
- S0..S7 左到右：0x0C 原始高电平在 S2/S3；反相后的 0xF3 有 S0/S1 和 S4..S7 两个区域，不满足单一区域窄线连续 3 帧的启动要求。
- 读取时 raw=0x00、line=0xFF、GRAY_AMBIGUOUS、8 个目标通道，valid_count=0；sensor.history 三帧均 0xFF。当前仍不满足启动要求。
- 本地 control_config.h 的 GRAY_INVERT_ENABLED=1 为巡白线；现场 raw/line 互补也与反相吻合。若实际为高电平黑线、低电平背景，则应使用 0；仅凭本快照不能确认真实赛道颜色、电气极性、安装高度或传感器阈值，不自动修改极性。
- 路径 valid=true，里程 ready=true，无路径故障；IMU 初始化失败阶段 2，913 ms 锁定为 HEADING_ENCODER / HEADING_FALLBACK_STARTUP。此次拒绝启动结果是灰度门禁，非 IMU 门禁。

START 是一次事件，拒绝后不排队。目标线连续稳定后须松开并重新按 START。下次实际启动应确保可控停车；本次没有发送任何启动请求。

## 文件与验证边界

openocd.log 保存连接和读取日志；decoded.txt 保存完整离线解码；verification.txt 交叉核对控制器关键字段。未修改固件或配置，因此不重新构建、运行主机测试或烧录。当前结论解释软件拒绝启动条件，不宣称已证明物理极性或修复实车问题。
