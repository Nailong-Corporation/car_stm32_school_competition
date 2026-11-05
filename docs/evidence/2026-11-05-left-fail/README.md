# 2026-11-05 左转概率失败与第二段冲出

## 只读现场

用户授权 ST-Link 读日志，明确不检查固件一致性。OpenOCD/ST-Link 1 MHz SWD，运行态读取 GPIOB ODR=0（PB5/STBY低）后，短暂停核保存 SRAM 起始 16 KiB，随后 resume/shutdown，退出0。未读取 Flash、复位、烧录、冻结看门狗、请求 START 或写控制变量。读取前本地 ELF 保存在 [read-time.elf](read-time.elf)，仅用作符号/类型布局；不证明板上二进制一致。

离线容器 [ram.core](ram.core) 不含 CPU 寄存器，GDB 寄存器警告为预期。控制器 stop_snapshot（0x200003c0）与 g_diagnostics.stop_snapshot（0x20002624）148 B 逐字节相同；状态、时间、索引、里程与轨迹吻合，支持本次布局解释。原始数据为 [ram.bin](ram.bin)，连接记录 [openocd.log](openocd.log)，解码为 [decoded.txt](decoded.txt) 和 [verification.txt](verification.txt)。

## 现场结论与边界

- 最近一次 START 已接受；停车发生在31128 ms，`LINE_STOP_LOST`，此前 `LINE_GAP_HOLD / ROUTE_FOLLOW`，无速度故障或路径 failure。
- 第二项 LEFT，索引1、计数2；本地用户表为最多4节点或219 cm。上次动作完成锚点166.064 cm，本次节点确认356.184 cm，段进度约190.120 cm。按现有规则尚未到目标，进入非目标直行 CLEARING，而非执行第二次LEFT。
- [turn_trace.csv](turn_trace.csv) 30103 ms起记录该非目标节点，目标约77.57/77.57 cm/s，实速77.15/72.15；随后持续无线仍下发高目标。30448 ms到CLEARING距离20.002 cm限界，warning=ACTION_TIMEOUT、navigation_recoveries=1，转RECOVERING。该 warning后来被到距的BRAKE_DISTANCE覆盖，不能只看最终warning还原原因。
- [navigation_trace.csv](navigation_trace.csv) 30493～31128 ms共128帧覆盖停车前640 ms；30668 ms恢复窄线达到离口证据后回FOLLOW，30728 ms最后连续有效线，31128 ms年龄400 ms停车。快照段进度222.577 cm、目标15.842/16.158、实速14.256/12.787 cm/s。
- IMU初始化阶段2失败，913 ms起编码器降级；不是本次停车故障。真实转角精度、新机械尺寸下标定仍待验证。
- 首个LEFT本次成功交接：turn_exit[0]记录27948 ms进入CLEARING、28143 ms交接下一项。当前RAM不包含用户所述“第一段失败那次”的完整现场，不能认定该现象根因已由实车证明。源码中“超速节点确认→BRAKING零目标→轮速合格但无线→RECOVERING直行→失线停车”的链路与现象相符，作为预防修复。
- 短轨迹不能证明此前所有路口都计数正确，也不能判断190 cm处的物理节点是否应为目标。未擅自补计、按失线强转或改路径表。总段长度定义本身截至灰度触发点，不能再扣除24 cm前置传感器距离。

## 修正

1. `TURN_APPROACH_SLOWDOWN_ENABLED=1` 时，<=100 cm（含未知0）的LEFT/RIGHT/CUSTOM_TURN/HEXAGON，在FOLLOW/CANDIDATE及下一段CLEARING预看阶段按有效入口速度低速接近，当前上限min(turn_speed,16)。避免短段先加速再到节点零输出等待降速。STOP/STRAIGHT保留原规划，开关关闭保留原短段请求。
2. 非目标CANDIDATE非窄线/非居中，以及非目标CLEARING未连续确认3帧合格居中出口、图形恢复活动期间，基础与双轮目标上限16 cm/s。连续可靠出口恢复原距离包络，并按原斜坡加速。计数、索引和总段原点保持。
3. 最终差速混合后按同比例限制双轮，保留转向方向。已知总段的速度平方包络同样限制双轮，防GAP_HOLD加法混合在基础16时给出超过16的外轮目标。独立BrakingSlew只跟踪速度包络，不重新钳到曲线轮目标，避免回归历史斜坡衰减问题。

保留实测轮速入口门禁、BRAKING无线不启动动作、RECOVERING流程、所有阶段时距/转角/出口证据与堵转、反向、计数、调度、俯仰保护。未开启反向制动或短刹，STBY低仍是待机。目标变低不能证明实际轮速立即降低。

用户当前表保持：LEFT/LEFT/LEFT/STOP；计数1/4/2/3；长度65/219/70/219 cm；表速50/200/200/200；turn_speed均50。此处仅同步文档，不改表值。

## 验证

修复前源码从本轮修改前的工作区内容恢复于 [line_follow.c.before](line_follow.c.before)，不是git HEAD（工作区已有其他改动）。新增测试short/clear/recovery在旧逻辑均断言失败，子进程SIGABRT，见before-*.log。早期host-tests*.log保留旧速度期望与夹具净位移不足引起的失败，不当作通过记录。

最终完整 `sh test/run_host_tests.sh` 退出0，见 [host-verified.log](host-verified.log)，97行含PASS（含重复配置及HEXAGON辅助输出）。新增专项两极性×IMU开关四配置各23352检查，覆盖左右镜像、低速节点直接入弯、短段启动/偏线、长段保速与到距不盲转、非目标无线/图形限速、可靠出口恢复及GAP_HOLD双轮包络。IMU开关专项默认夹具使用编码器降级；完整套件另含真实IMU合成输入/当帧降级回归，不冒称实车IMU验证。

`PLATFORMIO_CORE_DIR="$PWD/.pio/hexagon-core" pio run` 退出0，见 [pio-final.log](pio-final.log)，Flash42568 B / RAM10356 B。使用既有项目内core避免已知全局缓存锁权限问题；本次未重新尝试全局core。`git diff --check`通过，README两幅Mermaid与导航文档已同步，未运行渲染器。生产修改仅在line_follow控制逻辑，本次不增加RAM字段。旧现场始终以read-time.elf解码，不用新构建替换。

未烧录或操作电机；主机模型不能证明实车惯性、计数正确性、抓线和成功率。第二项目标识别还须用整段节点轨迹核对4/219配置，降低无线速度本身不会把count2/190节点变成目标。
