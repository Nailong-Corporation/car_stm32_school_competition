# 左转路口前停车：距离包络与反馈截止不一致

用户授权 ST-Link 检查并修复，明确不检查固件一致性。读取前保存本地 `read-time.elf`；OpenOCD 1 MHz SWD 初始化后读取 GPIOB ODR=0，PB5/STBY 低，短暂停核并读取 SRAM 16 KiB，随后执行 resume、shutdown。未复位、烧录、读取 Flash、冻结看门狗、请求 START 或写控制变量。`openocd.log` 记录连接与停核；命令包含 dump_image/resume/shutdown，读取文件完整为16384 B。`ram.core` 是离线生成的 SRAM ELF 容器，不含目标CPU寄存器，因此 GDB 提示缺少通用寄存器是预期的。

## 布局与读数

`read-time.elf` 中 follow=0x20000130、大小10104 B，g_diagnostics=0x200029b4、大小752 B；两份160 B停车快照位于0x200003c4与0x20002a40，逐字节一致。枚举、当前状态与环形索引可解析；仅据此确认本地布局可解释现场，不宣称板上二进制完全一致。`decoded.txt`、`layout.txt` 和 `decode_trace.py` 保存解析依据；脚本仅处理离线数据，不访问硬件。

- 唯一 START 已接受，路径段起点时间4560 ms，停车6375 ms（1815 ms后）；首项 LEFT、索引0、节点0，停车前 LINE_TRACKING / ROUTE_FOLLOW。
- `LINE_STOP_ROUTE / ROUTE_FAILURE_BRAKE_DISTANCE=13`；速度故障NONE，非出口丢线/动作超时/过转。
- 灰度raw/line=0x18，单区域、两路、position=0，距最后可靠线仅5 ms。
- 停车前左右目标31.3403/31.3691 cm/s，滤波反馈37.2934/33.9219 cm/s。末段进度60.1263 cm（起点-0.099811 cm，累计distance=60.0265 cm）。
- 反馈差分低通减速度约70.582 cm/s²、需求减速度为inf、反向力矩未启用。该瞬时估计不是完整制动力标定，不能据此直接把安全减速度改成70.582。
- IMU初始化阶段9失败，535 ms已锁定编码器航向来源；此次不是IMU失败阻止启动。读取时已过437830 ms，停车后滑行/外力等可能改变累计里程，不能拿当前71.24 cm代替停车60.03 cm。
- `navigation_trace.csv` 恢复128个5 ms周期，5740～6375 ms。始终FOLLOW，末帧FAILED；没有路口候选，灰度仅0x18/0x10。车辆最终物理位置不能反推当时已确认了路口。

## 修正

原首段L=65 cm，margin=2 cm：速度包络使用剩余65-d-2，到63 cm才请求16 cm/s；但braking_decide再扣guard=3 cm，在60 cm已要求反馈不高于16+3=19 cm/s。包络与保护截止错开3 cm。即使合成车辆按请求目标行驶，也触发停车（`regression-before.log`，退出134）。仅统一距离仍因30 ms轮速滤波加一周期响应出现目标16.5167、反馈20.2622的边界失败（`boundary-only.log`），不是充分修复。

现统一硬边界s=max(0,L-d-margin-guard)，guard只扣一次；不改变原物理停车边界60 cm。速度包络再使用s_cap=max(0,s-v*(SPEED_FILTER_TAU_S+dt))，提前覆盖已知反馈/控制延迟；硬停车判定继续用s，不因软件响应预留提前或延后。默认5 ms下响应时间为35 ms，包络仍为sqrt(v_entry²+2*a_safe*s_cap)。新增纯规划函数参数response_s并验证非法输入，数据结构/RAM布局未变。

保留用户进入任务前的巡黑线和赛道1四项65/70/50/70 cm配置，差异归档在`preexisting.patch`；不擅自修改路径长度、节点数、速度、标定、反向制动或保护阈值。减速只限制速度，距离不触发转弯或补计。超速实测入口BRAKING、丢线、动作、编码器及任务保护继续生效。现场反馈原样重放仍应以原因13停车；修复作用是更早降目标，不是豁免历史超速。

## 验证与边界

- 新增端到端回归用65 cm / 50 cm/s配置，左右镜像及最终STOP，真实odometry低通和一周期目标执行；通过硬边界仍无分支时继续FOLLOW/计数0，真实分支确认才执行动作。该模型假设轮子跟随目标，不模拟真实惯性/抓地。
- 原现场超速反馈的定向输入仍触发BRAKE_DISTANCE/禁用输出，入口滤波略超限时仍等待BRAKING，不放宽16 cm/s实际入口门禁。
- 修改后定向ASan/UBSan回归退出0，13429项检查；完整`sh test/run_host_tests.sh`与`pio run`均退出0，见相应exit/log文件。构建Flash42532 B、RAM11432 B。
- README总体数据流/状态图与导航文档同步，`git diff --check`通过。未烧录、未操作电机；新固件效果、65 cm的测量基准、轮径尺度和120 cm/s²制动能力仍需实车核实。不得把本次软件逻辑通过称为实车问题已完全消失。
