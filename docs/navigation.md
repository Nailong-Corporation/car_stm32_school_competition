# 双环、预设路径与编码器定位

首版日期：2026-10-19（Asia/Shanghai），取代“编码器只初始化、无速度闭环”和旧 SEARCH 丢线寻线策略。2026-10-23 当前配置为编码器与预设路径均开启、巡黑线、软件标定确认启用，编码器方向为 -1/+1。最新修改将路径目标由 12 提高到 25 cm/s，明确 START 后用 1 s 规划加速，取消整段行驶时间限制，保留窗口减速及分阶段直角转弯。此次直线停车没有诊断快照，实车原因尚未确认。软件构建与实车验证分别记录。

## 编译开关与普通电压控制

`include/motion_config.h` 使用两个独立编译开关，当前均为 1，修改后重新编译；不支持行驶中动态切换。2026-10-22 新增 `PRESET_ROUTE_ENABLED`，可设为 0 在编码器双环下单独测试巡线。

- **0**：不初始化/读取编码器，不运行里程、速度环、距离路口导航或路径 STOP callback；状态为 `ROUTE_DISABLED`。白线位置 PID 直接输出左右 PWM，通过占空比调节电机平均电压，轮速保持开环。尺寸参数为 0 不影响该模式的启动。
- **1**：启用编码器和双环；保留尺寸标定门禁和反馈异常停车。厘米路径另由 `PRESET_ROUTE_ENABLED` 选择。

`PRESET_ROUTE_ENABLED=0` 为仅巡线：忽略传入路径表和回调，不校验路径、不采用表内速度，不执行预设转向、距离窗口、路段超时或路径 STOP，状态保持 `ROUTE_DISABLED`。编码器仅巡线使用 `LINE_FOLLOW_SPEED_CM_S=12.0f` 作为基础目标速度，保留弯道降速、双轮速度环、干扰航向保持、400 ms / 8 cm 超限停车、任务/反馈故障和明确停止。全白、宽线和多区域一律进入干扰保持，不保证分岔处的分支选择。PWM 模式继续使用独立 PWM 参数。

`PRESET_ROUTE_ENABLED=1` 为本文的预设厘米路径、路口 FSM 与 STOP callback，必须同时配置 `ENCODER_CONTROL_ENABLED=1`，否则编译报错。两个开关都只允许 0 / 1。

PWM 模式的参数独立使用 PWM 单位：正常基础 255、最大偏差时基础 40、干扰基础上限 35、上升斜率 255 PWM 指令/s（1 s 达到居中目标）、位置 PID 8/0/0.08。输出限于非负且不超过 255（255 标尺），这些 PWM 参数不用于当前编码器速度目标。不把路径表中的 cm/s 数字直接当作 PWM。

PWM 模式要求明确启动且连续 3 帧有效窄白线。遇到无白线、宽线、全白或多区域时，冻结位置 PID，不提高基础 PWM，保留最近的有界 PWM 差值并在剩余异常预算内衰减；这不具备测量航向或纠正偏航的能力。恢复需连续 3 帧，重置 PID 历史；最后稳定白线起 400 ms 停车，孤立有效帧不续时，先检查截止再接受恢复。没有编码器，不使用 8 cm 距离限额，也不进行厘米路径动作。PB10、调试停止、任务时序检查、IWDG 和上电停止保留。

2026-10-23 按用户要求只保留 `genericSTM32F103C8` 一个环境：`pio run` 和一键上传都遵循头文件，不通过其他环境覆盖模式。模式与标定见 `include/motion_config.h`，颜色见 `include/control_config.h`。固件统一输出到 `.pio/build/genericSTM32F103C8/`。`g_diagnostics.encoder_control_enabled`、`preset_route_enabled` 表示实际模式；关闭编码器时里程和速度保持未测量的 0 值，不能解释为实际零速。

主机测试脚本分别编译 PWM 仅巡线、编码器仅巡线、编码器预设路径三个组合。PWM 测试不链接 `odometry.c` 或 `speed_control.c`；编码器仅巡线测试不链接 `route_table.c`，并检查传入路径表不会产生动作或回调。下面的双环/里程内容适用于编码器开关为 1，路径/路口内容还要求路径开关为 1。

## 单位与双环

- 白线位置：权重单位，S0…S7 为 −7…+7；`error = position - 0`。
- 位置 PID 外环输出：左右目标速度的半差，单位 cm/s。
- `left_target = base + correction`，`right_target = base - correction`；正修正期望向右转。
- 轮速 PID 内环：每轮独立，以“目标 cm/s − 编码器测量 cm/s”为误差，输出 PWM 指令。
- 目标速度限于 0…25 cm/s；每次 START 使用 `min(1, 启动累计实际 dt / 1 s)` 的基础目标包络。居中直线在 1 s 内达到表中目标；恢复和路段切换的上升斜率仍为 40 cm/s²。降速/停车不等待斜坡，不输出倒转命令。
- PWM 沿用 0…255 标尺、运行上限 255，最终由 TIM3 ARR=6399 映射成 CCR。编码器模式目标 25 cm/s 不等于 PWM=255，实际加速和轮速达成时间仍需测量。

`pid_update_bounded()` 支持不对称输出边界和前馈：速度内环按 `前馈 + PID` 总输出是否越过 0…255 决定积分是否接受，避免只限最终输出却继续积累积分。位置环使用对称边界；两类 PID 都有积分限幅、微分滤波、非法值拒绝和历史复位。

外环初值 P=1.8、I=0、D=0.025；内环 P=2、I=8、D=0，前馈为 `3 × target_cm_s`。轮速低通时间常数 30 ms。它们都是软件初值，实际应先标定/调试速度内环，再调位置外环。

停车、启动、干扰后稳定恢复、路口动作切换会清理相应 PID 历史。干扰期间位置 PID 不更新，速度内环继续工作；不是直接沿用上一 PWM。

## 编码器与标定入口

左轮 PA8/PA9 对应 TIM1_CH1/CH2，右轮 PB6/PB7 对应 TIM4_CH1/CH2。`encoder_init()` 使用 `TIM_ENCODERMODE_TI12`，PSC=0、ARR=65535，两输入直接映射、IC 滤波值 6、无内部上下拉，启动 CH1+CH2。TI12 在两相边沿四倍频计数，见 [ST HAL](https://github.com/STMicroelectronics/stm32f1xx-hal-driver/blob/master/Src/stm32f1xx_hal_tim.c)与 [RM0008](https://www.st.com/resource/en/reference_manual/cd00171190.pdf)。滤波参数与信号波形仍需实测。

每个控制周期读取两个计数器，不启用逐脉冲或溢出中断。软件将差值解码为 −32767…+32767 的有符号增量，处理正常正向/反向回绕；恰好半圈差值拒绝。配置时检查最高估计轮速、15 ms 最大间隔和计数比例是否会超过半圈。实际若跨越多个硬件回绕仍可能混叠，必须用真实最高转速验证采样约束。

在 `include/motion_config.h` 填写：

| 宏 | 含义 | 当前值 |
| --- | --- | --- |
| `LEFT_WHEEL_DIAMETER_CM` / `RIGHT_WHEEL_DIAMETER_CM` | 左右轮有效直径，cm | 均为 4.8，实物有效直径待核对 |
| `LEFT_COUNTS_PER_WHEEL_REV` / `RIGHT_COUNTS_PER_WHEEL_REV` | 输出轮完整一圈、已含减速与四倍频的计数 | 均为 1560，按用户提供的 L150PRO 参数计算，实物待核对 |
| `WHEEL_TRACK_CM` | 左右轮中心距，cm | 14.5，用户于 2026-10-23 实测 |
| `ENCODER_LEFT_SIGN` / `ENCODER_RIGHT_SIGN` | 正向行驶时增量符号修正，±1 | 左 -1 / 右 +1，依据启动后计数回绕修正，实车待复测 |
| `ODOMETRY_CALIBRATION_CONFIRMED` | 软件标定配置确认开关 | 1，按用户明确要求启用；实物计数和方向尚未验证 |

若厂家给的是电机轴单相每圈周期数 PPR，减速比为 G，则输出轮四倍频计数约为 `PPR × 4 × G`。若厂家已经给出四倍频 CPR 或输出轴每圈计数，则不能重复乘 4 或减速比。规格不清时优先停车手转多圈，直接测最终计数。

2026-10-23 用户提供 L150PRO 减速比为 1:30、霍尔编码器电机轴单相每圈 13 个周期，因此两侧均采用 `13 × 30 × 4 = 1560`。来源为用户摘录的[钉钉文档图 3-3](https://alidocs.dingtalk.com/i/nodes/Gl6Pm2Db8D6PkYv6h9PQXOmdVxLq0Ee4)，本次未能独立读取原文。同日用户实测左右轮中心间距为 14.5 cm，已填写 `WHEEL_TRACK_CM=14.5f`。随后按用户“直接完成标定”的明确要求，将 `ODOMETRY_CALIBRATION_CONFIRMED` 设为 1，使用现有轮径 4.8 cm、每圈计数 1560、轮距 14.5 cm、方向最初为 `+1/+1`。随后用户调试截图显示 `LINE_START_ACCEPTED`、`LINE_STOP_ENCODER`、原始左计数 65527 / 右计数 2；按初始化为 0 解释，左计数等效为 -9，提示左侧符号相反，因此修正为 `-1/+1`，右侧稳定反馈及修正后运行仍待复测。这些参数通过软件有效性检查后允许进入后续启动判断；每圈计数仍为规格计算值，实物计数、轮径及正向符号尚未验证。

换算：

```text
left_cm_per_count  = π × left_diameter_cm  / left_counts_per_wheel_rev
right_cm_per_count = π × right_diameter_cm / right_counts_per_wheel_rev
Δleft_cm  = signed_left_counts  × left_cm_per_count
Δright_cm = signed_right_counts × right_cm_per_count
Δdistance_cm = (Δleft_cm + Δright_cm) / 2
Δheading_rad = (Δleft_cm - Δright_cm) / wheel_track_cm
wheel_speed_cm_s = Δwheel_cm / actual_dt
```

航向向右为正，与位置外环符号一致。里程是车体中心沿行驶方向的累计相对距离；没有地图坐标、绝对定位或打滑修正。倒向增量保持符号，不通过绝对值把倒退计算成前进。

标定确认置 0 或参数无效时，TIM1/TIM4 仍采集原始计数，主循环仍读取灰度并喂狗。PB10 启动请求被拒绝，状态为 `LINE_STOP_CALIBRATION`，STBY 保持低；可用诊断计数检查手动转轮。当前确认已置 1，正常初始化后 `g_diagnostics.odometry_ready` 应为 1；仍保留可靠目标线启动门禁以及编码器、任务、丢线和路径故障停车。

## 路径存储与停止回调

`include/route.h` 定义 `Action` 枚举及 `RouteStep` 结构体，`src/route_table.c` 的只读表编译进固件 Flash：

```c
typedef enum { LEFT, RIGHT, STRAIGHT, STOP } Action;
typedef struct {
    float distance;   // cm
    float tolerance;  // ±cm
    Action action;
    int speed;        // cm/s，不是 PWM
} RouteStep;

const RouteStep route[] = {
    {104.0f, 8.0f, LEFT, 25},
    {131.0f, 8.0f, LEFT, 25},
    {60.0f, 5.0f, LEFT, 25},
    {39.0f, 5.0f, LEFT, 25},
    {70.0f, 5.0f, STOP, 25}, // 仍要求路口确认
};
```

表中距离和容差保留现有设置，本次仅将各段目标提高到 25 cm/s，未重新测量赛道。首段以明确启动时的里程为零；下一段以“上一动作完成并离开路口消抖区”的位置为零，不是起点累计里程。表项要求正距离、有限且小于距离的非负容差、0…25 范围内的正目标速度；最后一项必须 STOP，STOP 不能位于表中间。启动时会重置路径索引到 0，不自动续跑一半路径。

所有动作都要求 `distance - tolerance <= 当前路段里程 <= distance + tolerance`。STOP 不因里程单独到达而成功；没有路口并越过距离窗口上界则以 `LINE_STOP_ROUTE` 停车，不调用成功回调。

按用户最新要求，整段行驶直接取消时间限制：不设置固定 15 s 或按长度/速度计算的预算，不因路段经过的时间而失败停车。启动或推进下一项时仍建立里程基准，并保留 `segment_ms` 用于 `route_elapsed_ms` 诊断；经过毫秒时基回绕也不会触发路段时间停车。越过距离窗口上界、路口候选确认 100 ms、动作与离口 4500 ms、丢线/反馈/任务保护仍独立生效。

`g_diagnostics.route_failure` 锁存具体原因至下一次 START：0 无失败、1 非法配置/输入、2 越窗、3 已停用（旧路段超时）、4 候选确认超时、5 动作/离口超时、6 过转、7 反向转弯。结合 `stop_reason`、`route_state`、`route_index`、`route_progress_cm`、`route_elapsed_ms` 定位。`route_elapsed_ms` 为采样当前时间减路段起始时间，停车后仍增长；它不是锁存的停车时刻。可按 START 再启动只排除了部分锁定故障，不能直接判定具体停车原因；整段行驶计时已不再触发停车。

`RouteStopCallback` 为 `void (*)(size_t step_index, void *context)`。STOP 确认后路口状态进入 `ROUTE_FINISHED`，控制器立即清空目标和 PWM 命令、复位各 PID。主循环下发 `motor_stop()` 后调用 `route_dispatch_stop_callback()`，先锁存一次性标志再调用用户代码，重复分派及回调重入都不会重复执行。

默认回调在 `src/main.c`：

```c
void route_stop_callback(size_t step_index, void *context)
{
    (void)context;
    ++g_diagnostics.route_stop_events;
    g_diagnostics.route_stopped_step = (uint32_t)step_index;
}
```

回调属于主循环任务，不在 ISR 内；应保持短小，不允许阻塞延时、长串口输出或重新使能电机。任务耗时检查包含回调。按键停止、超时与故障不会生成成功回调；任务在分派前已超时则取消待分派事件。

## 路口有限状态机

| 状态 | 条件与动作 |
| --- | --- |
| `ROUTE_FOLLOW` | 正常目标线双环控制；进入距离窗口先把基础目标降至不超过 6 cm/s，合适形状才进入候选 |
| `ROUTE_CANDIDATE` | 中心传感器 S3/S4 至少一路白，白色通道至少 4 路，且出现目标侧分支；降速保持航向 |
| 候选确认 | 连续至少 3 帧，确认期间前进至少 0.1 cm，100 ms 内、仍在窗口；图形消失清候选，但不刷新丢线基准或路段时限 |
| `ROUTE_EXECUTING` | LEFT/RIGHT 内轮目标 0、外轮 12 cm/s，转过 1.25 rad 后外轮降至 6 cm/s；STRAIGHT 保持入口航向 |
| 转弯退出 | 期望方向至少转过 1.35 rad，同时中心窄目标线连续 3 帧；超过 2.1 rad 或反向转过 0.35 rad 停车 |
| 直行退出 | 从确认位置起前进至少 3 cm，同时中心窄白线连续 3 帧 |
| `ROUTE_CLEARING` | 恢复目标线 PID，基础目标不超过 6 cm/s；再离开至少 3 cm 并连续 3 帧窄目标线后消费下一项并重建路段基准 |
| `ROUTE_FINISHED` | STOP 路口确认，停车并等待主循环分派一次性回调 |
| `ROUTE_FAILED` | 越过窗口、候选超时、动作与离口累计 4500 ms、异常方向等；停车并保留具体原因 |

LEFT 分支要求 S0/S1 至少一路白；RIGHT 要求 S6/S7 至少一路白；STRAIGHT 和 STOP 接受任一侧分支。普通窄白线和只有边缘的孤立白点不构成路口。当前没有图案语义识别或路口类型学习；阈值需根据真实线宽、传感器高度和阵列跨度标定。

原配置基础 6、半差 4 cm/s，左右目标 2/10 cm/s，在 14.5 cm 轮距下理论角速度为 `8/14.5 rad/s`，90° 约需 2.85 s，超过原动作与离口共用的 2.5 s。现在转弯半差等于基础值，左转目标为 0/12、右转为 12/0 cm/s，理想半径从 10.875 cm 缩小为半轮距 7.25 cm；接近 1.25 rad 后目标改为 0/6 或 6/0 cm/s 找线。动作与离口总预算改为 4.5 s，退出仍要求转角和连续中心目标线，并保留最大转角/反向保护。较低表内速度会相应限制外轮目标。内轮目标为零不代表物理锁轮，未使用倒转或双轮反向原地转弯。何时开始转弯、传感器到轮轴的距离、最低转角、离口距离和实际时限仍需实车标定。

## 元器件图形与丢线航向保持

普通位置控制只接受单一、宽度不超过 4 路的有效区域。即使底层传感器分类通过连续性选中了多区域中的一段，导航层仍将其视为干扰。无白线、全白、宽线、多区域以及未确认路口都不更新正常位置 PID。

第一次异常保存当前编码器航向 `hold_heading_rad`，将基础速度降到不超过 6 cm/s：

```text
correction_cm_s = heading_kp × (hold_heading_rad - heading_rad)
left_target  = hold_base + correction_cm_s
right_target = hold_base - correction_cm_s
```

这样保持进入干扰时的航向，不把干扰图形转换成白线转向误差。位置积分与微分历史保持冻结，速度内环持续调节。连续 3 帧单一窄白线、且尚未达到截止时限，才重建位置/速度 PID 历史并恢复巡线。

从最后稳定白线起 400 ms，或自进入异常起累计偏移 8 cm，任一先到便停车。异常种类切换及孤立有效帧不刷新基准；截止检查优先于本周期恢复或路口确认。旧的“120 ms 保持 + 280 ms SEARCH”已移除。本次不通过扩大丢线时限假定元器件图案一定可穿越。

一维传感器无法区分完全相同的观测：窗口内与路口同形的图案可能被确认，持续窄图形可能被当作白线。编码器不能发现所有轮胎打滑；仅靠双轮里程保持的是估计航向。必须在真实图案、虚线与电量变化下记录并标定。

## 故障与验证

- 上电停止，启动需尺寸配置有效、明确请求、连续有效窄白线。
- 两路编码器初始化/启动返回值逐项检查；右路启动失败会停止已启动左路，随后异常停车。
- 计数跳变超出物理速度上界及量化余量、不可判定半圈增量、非法 dt/数值停车。
- 有非零速度指令时，反向反馈小于 −0.5 cm/s，或目标至少 3 cm/s 而测得速度低于 0.5 cm/s 持续 350 ms，则以编码器/堵转故障停车并锁定，需复位。这不能检测所有单相断线、打滑或错误脉冲，阈值受分辨率影响。
- 任务单次间隔 >15 ms 或计算/回调耗时 >=5 ms 锁定故障停车；保留 IWDG。
- 标定确认门禁不替代硬件电平、供电、STBY 停车、架空轮向和实车验证。

主机入口 `sh test/run_host_tests.sh` 包含传感器/PID/PWM、双环与里程/路口/干扰/完整示例流程，以及电机和编码器 HAL 模拟；均开启 ASan/UBSan。`pio run -e genericSTM32F103C8` 交叉构建所有生产模块。准确执行结果见 [验证记录](validation.md)。
