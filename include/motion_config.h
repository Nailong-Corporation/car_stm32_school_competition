#ifndef MOTION_CONFIG_H
#define MOTION_CONFIG_H

/* ==================== 一、控制模式选择 ====================
 * 0：白线位置 PID 直接调节 PWM，电机速度保持开环，不使用编码器和厘米路径。
 * 1：启用编码器、位置/速度双环、厘米路径表及路口状态机，需完成尺寸标定。
 * 这是编译开关，修改后须重新构建；允许通过编译参数覆盖。
 * encoder_closed_loop 构建环境会强制指定为 1。
 */
#ifndef ENCODER_CONTROL_ENABLED
#define ENCODER_CONTROL_ENABLED       0
#endif
#if ENCODER_CONTROL_ENABLED != 0 && ENCODER_CONTROL_ENABLED != 1
#error "ENCODER_CONTROL_ENABLED must be 0 or 1"
#endif

/* ==================== 二、普通 PWM 模式参数 ====================
 * 仅在开关为 0 时使用；以下指令采用 0～255 标尺，并受 MOTOR_PWM_LIMIT 限制。
 * PWM 用于调节平均驱动电压，不表示已知轮速；不得与 cm/s 参数混用。
 */
#define OPEN_LOOP_BASE_PWM           55.0f  /* 白线居中时的目标基础 PWM。 */
#define OPEN_LOOP_TURN_BASE_PWM      40.0f  /* 位置偏差达到最大时降低到此基础 PWM。 */
#define OPEN_LOOP_HOLD_PWM           35.0f  /* 丢线/干扰期间的基础 PWM 上限，不主动加速。 */
#define OPEN_LOOP_PWM_RISE_PER_S     200.0f /* 基础 PWM 每秒最大上升量；降速与停车不等待斜坡。 */
#define OPEN_LOOP_POSITION_KP         8.0f  /* 位置比例项，输出左右 PWM 的半差。 */
#define OPEN_LOOP_POSITION_KI         0.0f  /* 位置积分项初值，未标定前暂不启用。 */
#define OPEN_LOOP_POSITION_KD         0.08f /* 位置微分项，配合公共微分低通使用。 */

/* ==================== 三、轮径、计数比例与方向标定 ====================
 * 仅编码器模式要求这些尺寸有效且确认标志为 1；0 表示参数未知，禁止该模式启动。
 * 左右轮直径可独立标定，以补偿轮胎有效直径差异。
 * 每输出轮一圈计数必须已经包含减速比和 TI12 四倍频，不能重复乘入。
 * 若厂家只给电机轴单相每圈周期数，则输出轮计数约为该值 × 4 × 减速比。
 */
#define ODOMETRY_CALIBRATION_CONFIRMED 0    /* 尺寸、计数口径及正向符号实测确认后置 1。 */
#define LEFT_WHEEL_DIAMETER_CM        0.0f /* 左轮有效直径，单位 cm。 */
#define RIGHT_WHEEL_DIAMETER_CM       0.0f /* 右轮有效直径，单位 cm。 */
#define LEFT_COUNTS_PER_WHEEL_REV     0.0f /* 左输出轮完整一圈的最终四倍频计数。 */
#define RIGHT_COUNTS_PER_WHEEL_REV    0.0f /* 右输出轮完整一圈的最终四倍频计数。 */
#define WHEEL_TRACK_CM               0.0f /* 左右轮中心间距，单位 cm，用于推算航向变化。 */
#define ENCODER_LEFT_SIGN             1    /* 左轮计数方向修正，只允许 +1 或 -1，前进应为正。 */
#define ENCODER_RIGHT_SIGN            1    /* 右轮计数方向修正，只允许 +1 或 -1，前进应为正。 */

/* ==================== 四、编码器采样与轮速量程 ====================
 * 输入滤波作用于定时器硬件，轮速低通作用于软件换算后的 cm/s 测量值。
 * 计数跳变上界与目标速度上限用途不同；还需实测保证相邻采样不跨过半个计数器量程。
 */
#define ENCODER_INPUT_FILTER         6U     /* 两相输入捕获滤波配置值，不是毫秒数。 */
#define ODOMETRY_MAX_SPEED_CM_S      100.0f /* 计数合理性检查的轮速上界，另允许计数量化余量。 */
#define SPEED_FILTER_TAU_S           0.030f /* 测量轮速的一阶低通时间常数，单位 s。 */
#define WHEEL_TARGET_MAX_CM_S        25.0f  /* 每个车轮目标速度上限，单位 cm/s。 */

/* ==================== 五、左右轮速度 PID 内环 ====================
 * 仅编码器模式使用；误差为目标速度减实测速度，单位 cm/s，最终输出为 PWM 指令。
 * 左右轮分别保存 PID 历史，前馈与 PID 相加后按 0～MOTOR_PWM_LIMIT 做抗饱和限幅。
 */
#define SPEED_KP                     2.0f  /* 速度比例系数。 */
#define SPEED_KI                     8.0f  /* 速度积分系数，实际积分使用 dt。 */
#define SPEED_KD                     0.0f  /* 速度微分系数初值，当前为 0。 */
#define SPEED_INTEGRAL_LIMIT         10.0f /* 速度误差积分的绝对值上限，积分单位为 cm。 */
#define SPEED_FEEDFORWARD             3.0f  /* 每 1 cm/s 目标速度对应的 PWM 前馈系数。 */

/* ==================== 六、编码器反馈与堵转保护 ====================
 * 目标达到指定值而测得速度持续偏低时停车；反向反馈越过负阈值也会报故障。
 * 这些判断不能覆盖所有断线或打滑情况，阈值与超时须按编码器分辨率及启动特性标定。
 */
#define SPEED_STALL_TARGET_CM_S       3.0f  /* 目标至少达到此速度时才累计低速保护时间。 */
#define SPEED_STALL_MEASURED_CM_S     0.5f  /* 低速判定阈值；反向反馈使用其负值作为界限。 */
#define SPEED_STALL_TIMEOUT_S         0.350f /* 低速持续到此秒数即故障停车。 */

/* ==================== 七、白线位置外环与目标速度规划 ====================
 * 仅编码器模式使用；位置误差来自 -7～+7 权重，PID 输出为两轮目标速度的半差（cm/s）。
 * 正修正使左目标速度大、右目标速度小，期望向右转；普通 PWM 模式使用上方独立参数。
 */
#define POSITION_KP_CM_S              1.8f   /* 白线位置比例系数。 */
#define POSITION_KI_CM_S              0.0f   /* 白线位置积分系数初值。 */
#define POSITION_KD_CM_S              0.025f /* 白线位置微分系数。 */
#define POSITION_CURVE_SLOWDOWN       0.35f  /* 最大位置偏差时，相对路段基础速度的降速比例。 */
#define TARGET_RISE_CM_S2            40.0f  /* 基础目标速度最大上升斜率，单位 cm/s²。 */

/* ==================== 八、丢线、图形干扰与航向保持 ====================
 * 两模式均冻结普通位置 PID，并从最后稳定白线起计时；孤立有效帧不刷新预算。
 * 编码器模式降速并纠正估计航向，另检查厘米距离；PWM 模式只衰减历史差值。
 */
#define HOLD_SPEED_CM_S              6.0f   /* 编码器模式干扰保持的基础速度上限，单位 cm/s。 */
#define HEADING_KP_CM_S_PER_RAD       12.0f /* 每 rad 航向误差产生的目标速度半差，单位 (cm/s)/rad。 */
#define INTERFERENCE_TIMEOUT_MS     400U   /* 两模式共用：最后稳定观测起的异常总时限，单位 ms。 */
#define INTERFERENCE_MAX_DISTANCE_CM 8.0f  /* 仅编码器模式：进入异常后允许的里程偏移上限，单位 cm。 */

/* ==================== 九、路口候选确认 ====================
 * 仅编码器模式使用；还必须位于当前路径项距离窗口内，形状满足中心白线和所需分支。
 * 连续帧数、前进距离和时间窗口联合约束，静止图形或孤立噪声不能直接触发动作。
 */
#define JUNCTION_CONFIRM_SAMPLES     3U    /* 候选路口至少连续出现的控制采样帧数。 */
#define JUNCTION_CONFIRM_DISTANCE_CM 0.10f /* 同一候选确认期间至少前进的距离，单位 cm。 */
#define JUNCTION_CONFIRM_TIMEOUT_MS  100U  /* 同一候选完成确认的最长时间，单位 ms。 */

/* ==================== 十、路口动作、转角与离口去重 ====================
 * 转弯使用非负前进差速；达到退出几何条件并连续获得中心窄白线后进入离口阶段。
 * 动作和离口共用一个超时预算，防止同一路口重复推进路径或无限转弯。
 */
#define JUNCTION_ACTION_TIMEOUT_MS   2500U /* 从动作开始到离口完成的总时限，单位 ms。 */
#define JUNCTION_TURN_MIN_RAD         1.0f /* 左右转允许退出的最小期望方向转角，单位 rad。 */
#define JUNCTION_TURN_MAX_RAD         2.1f /* 超过此期望方向转角则异常停车，单位 rad。 */
#define JUNCTION_WRONG_WAY_RAD        0.35f /* 反向转角超过此值则异常停车，单位 rad。 */
#define JUNCTION_CENTER_LIMIT         2.0f /* 退出动作时允许的中心位置误差，单位为传感器权重。 */
#define JUNCTION_TURN_SPEED_CM_S      6.0f /* 路口动作基础速度上限，单位 cm/s。 */
#define JUNCTION_TURN_DIFF_CM_S       4.0f /* 左右转目标速度半差的幅值，单位 cm/s。 */
#define JUNCTION_CLEAR_DISTANCE_CM    3.0f /* 直行退出的最小前进距离，以及离口去重阶段的额外距离，单位 cm。 */

/* ==================== 十一、整段路径保护 ====================
 * 启动或上一动作离口完成时建立新路段基准；超时或越过距离窗口仍未确认路口则失败停车。
 * STOP 同样要求在窗口内确认路口，不能仅因到达预计里程就调用成功回调。
 */
#define ROUTE_SEGMENT_TIMEOUT_MS     15000U /* 等待本段路口的最长时间，单位 ms。 */

#endif
