# 双环、预设路径与编码器定位

日期：2026-10-19（Asia/Shanghai）。这是用户本次授权的软件实现，取代“编码器只初始化、无速度闭环”和旧 SEARCH 丢线寻线策略。没有烧录或操作电机；车轮尺寸与编码器规格暂未知，默认禁止启动。

## 单位与双环

- 白线位置：权重单位，S0…S7 为 −7…+7；`error = position - 0`。
- 位置 PID 外环输出：左右目标速度的半差，单位 cm/s。
- `left_target = base + correction`，`right_target = base - correction`；正修正期望向右转。
- 轮速 PID 内环：每轮独立，以“目标 cm/s − 编码器测量 cm/s”为误差，输出 PWM 指令。
- 目标速度限于 0…25 cm/s，基础速度上升斜率 40 cm/s²。不输出倒转命令。
- PWM 仍沿用 0…255 标尺、运行上限 100，最终由 TIM3 ARR=6399 映射成 CCR。

`pid_update_bounded()` 支持不对称输出边界和前馈：速度内环按 `前馈 + PID` 总输出是否越过 0…100 决定积分是否接受，避免只限最终输出却继续积累积分。位置环使用对称边界；两类 PID 都有积分限幅、微分滤波、非法值拒绝和历史复位。

外环初值 P=1.8、I=0、D=0.025；内环 P=2、I=8、D=0，前馈为 `3 × target_cm_s`。轮速低通时间常数 30 ms。它们都是软件初值，实际应先标定/调试速度内环，再调位置外环。

停车、启动、干扰后稳定恢复、路口动作切换会清理相应 PID 历史。干扰期间位置 PID 不更新，速度内环继续工作；不是直接沿用上一 PWM。

## 编码器与标定入口

左轮 PA8/PA9 对应 TIM1_CH1/CH2，右轮 PB6/PB7 对应 TIM4_CH1/CH2。`encoder_init()` 使用 `TIM_ENCODERMODE_TI12`，PSC=0、ARR=65535，两输入直接映射、IC 滤波值 6、无内部上下拉，启动 CH1+CH2。TI12 在两相边沿四倍频计数，见 [ST HAL](https://github.com/STMicroelectronics/stm32f1xx-hal-driver/blob/master/Src/stm32f1xx_hal_tim.c)与 [RM0008](https://www.st.com/resource/en/reference_manual/cd00171190.pdf)。滤波参数与信号波形仍需实测。

每个控制周期读取两个计数器，不启用逐脉冲或溢出中断。软件将差值解码为 −32767…+32767 的有符号增量，处理正常正向/反向回绕；恰好半圈差值拒绝。配置时检查最高估计轮速、15 ms 最大间隔和计数比例是否会超过半圈。实际若跨越多个硬件回绕仍可能混叠，必须用真实最高转速验证采样约束。

在 `include/motion_config.h` 填写：

| 宏 | 含义 | 当前值 |
| --- | --- | --- |
| `LEFT_WHEEL_DIAMETER_CM` / `RIGHT_WHEEL_DIAMETER_CM` | 左右轮有效直径，cm | 0，待测 |
| `LEFT_COUNTS_PER_WHEEL_REV` / `RIGHT_COUNTS_PER_WHEEL_REV` | 输出轮完整一圈、已含减速与四倍频的计数 | 0，待测 |
| `WHEEL_TRACK_CM` | 左右轮中心距，cm | 0，待测 |
| `ENCODER_LEFT_SIGN` / `ENCODER_RIGHT_SIGN` | 正向行驶时增量符号修正，±1 | +1，待验证 |
| `ODOMETRY_CALIBRATION_CONFIRMED` | 上述参数和方向已确认后置 1 | 0，禁止启动 |

若厂家给的是电机轴单相每圈周期数 PPR，减速比为 G，则输出轮四倍频计数约为 `PPR × 4 × G`。若厂家已经给出四倍频 CPR 或输出轴每圈计数，则不能重复乘 4 或减速比。规格不清时优先停车手转多圈，直接测最终计数。

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

标定确认前 TIM1/TIM4 仍采集原始计数，主循环仍读取灰度并喂狗。PB10 启动请求被拒绝，状态为 `LINE_STOP_CALIBRATION`，STBY 保持低；可用诊断计数检查手动转轮。程序不使用猜测的轮径驱动厘米路径。

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
    {30.0f, 5.0f, LEFT, 12}, // 唯一行驶示例
    {20.0f, 4.0f, STOP, 8},  // 停止项，也要求路口确认
};
```

这是示例，不是已测赛道。首段以明确启动时的里程为零；下一段以“上一动作完成并离开路口消抖区”的位置为零。所以示例不表示从起点累计 50 cm 停车。表项要求正距离、有限且小于距离的非负容差、0…25 范围内的正目标速度；最后一项必须 STOP，STOP 不能位于表中间。启动时会重置路径索引到 0，不自动续跑一半路径。

所有动作都要求 `distance - tolerance <= 当前路段里程 <= distance + tolerance`。STOP 不因里程单独到达而成功；没有路口并越过上界/超出路段时限则以 `LINE_STOP_ROUTE` 停车，不调用成功回调。

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
| `ROUTE_FOLLOW` | 正常白线双环控制；只有距离窗口内的合适形状才能进入候选 |
| `ROUTE_CANDIDATE` | 中心传感器 S3/S4 至少一路白，白色通道至少 4 路，且出现目标侧分支；降速保持航向 |
| 候选确认 | 连续至少 3 帧，确认期间前进至少 0.1 cm，100 ms 内、仍在窗口；图形消失清候选，但不刷新丢线基准或路段时限 |
| `ROUTE_EXECUTING` | LEFT/RIGHT 使用非负速度差；STRAIGHT 保持动作开始时的航向；有独立 2500 ms 动作时限 |
| 转弯退出 | 期望方向至少转过 1.0 rad，同时中心窄白线连续 3 帧；超过 2.1 rad 或反向转过 0.35 rad 停车 |
| 直行退出 | 从确认位置起前进至少 3 cm，同时中心窄白线连续 3 帧 |
| `ROUTE_CLEARING` | 恢复白线控制，再离开至少 3 cm 并连续 3 帧窄白线；之后才消费下一项并重建路段基准，防止同一路口重复触发 |
| `ROUTE_FINISHED` | STOP 路口确认，停车并等待主循环分派一次性回调 |
| `ROUTE_FAILED` | 越过窗口、路段 15000 ms、候选超时、动作与离口累计 2500 ms、异常方向等；停车 |

LEFT 分支要求 S0/S1 至少一路白；RIGHT 要求 S6/S7 至少一路白；STRAIGHT 和 STOP 接受任一侧分支。普通窄白线和只有边缘的孤立白点不构成路口。当前没有图案语义识别或路口类型学习；阈值需根据真实线宽、传感器高度和阵列跨度标定。

转弯由前进差速完成，基础速度不超过 6 cm/s，修正量 ±4 cm/s 并受非负边界限制；不倒车、不原地旋转。何时开始转弯、最低转角、离口距离与时限要结合传感器到轮轴的距离实测，现有数值不能保证通过真实 90° 路口。

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
