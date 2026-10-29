#ifndef MOTION_CONFIG_H
#define MOTION_CONFIG_H

/* ==================== 一、控制模式选择 ====================
 * 0：白线位置 PID 直接调节 PWM，电机速度保持开环，不使用编码器和厘米路径。
 * 1：启用编码器及位置/速度双环，需完成尺寸标定；路径由下面的独立开关选择。
 * 这是编译开关，修改后须重新构建；允许通过编译参数覆盖。
 * PlatformIO 只使用 genericSTM32F103C8 环境，编译和上传遵循本文件的开关。
 */
#ifndef ENCODER_CONTROL_ENABLED
#define ENCODER_CONTROL_ENABLED       1
#endif
#if ENCODER_CONTROL_ENABLED != 0 && ENCODER_CONTROL_ENABLED != 1
#error "ENCODER_CONTROL_ENABLED must be 0 or 1"
#endif

/* 预设路径开关：0 仅沿白线巡线，1 按 route_table.c 执行距离/路口/STOP。
 * 当前为 1；设 0 可单独测试巡线，仍保留明确启动、丢线及故障停车。
 * 路径使用厘米定位，置 1 时必须同时开启 ENCODER_CONTROL_ENABLED。
 * 编译环境不覆盖模式；修改本文件后使用同一个编译/上传入口。
 */
#ifndef PRESET_ROUTE_ENABLED
#define PRESET_ROUTE_ENABLED          1
#endif
#if PRESET_ROUTE_ENABLED != 0 && PRESET_ROUTE_ENABLED != 1
#error "PRESET_ROUTE_ENABLED must be 0 or 1"
#endif
#if PRESET_ROUTE_ENABLED && !ENCODER_CONTROL_ENABLED
#error "PRESET_ROUTE_ENABLED requires ENCODER_CONTROL_ENABLED=1"
#endif

/* MPU6050 航向选择：0 维持编码器轮差估角；1 所有角度/位姿改用 IMU Z 轴积分。
 * 默认 0，接好 PB8/PB9 和 3.3 V、确认装配方向后再改 1 并重新编译。
 * 编码器仍负责里程和左右轮速度闭环；IMU 不提供绝对偏航或厘米位移。
 */
#ifndef MPU6050_ENABLED
#define MPU6050_ENABLED                1
#endif
#if MPU6050_ENABLED != 0 && MPU6050_ENABLED != 1
#error "MPU6050_ENABLED must be 0 or 1"
#endif
#if MPU6050_ENABLED && !ENCODER_CONTROL_ENABLED
#error "MPU6050_ENABLED requires ENCODER_CONTROL_ENABLED=1"
#endif

/* AD0 接 GND 时 7-bit 地址 0x68；若 AD0 接 3.3 V 则改为 0x69。 */
#define MPU6050_ADDRESS_7BIT           0x68U
/* 实际 MPU6050 倒置安装（芯片面朝下、芯片 Z 正轴朝地面），车体右转时
 * 在传感器坐标下读得正 Z 角速度：+1 对应全系统右转正航向。
 * 仅使用水平面偏航角，不融合俯仰/横滚；必须近似水平固定并手持右转核实。 */
#define MPU6050_YAW_SIGN               1
#define MPU6050_GYRO_BIAS_SAMPLES      128U
#define MPU6050_GYRO_CALIBRATION_MS    5U
#define MPU6050_I2C_TIMEOUT_MS         2U

/* MPU6050 俯仰防拿起保护：默认开启，只有 MPU6050_ENABLED=1 时生效。
 * 通过加速度计重力方向计算车体前后俯仰角的绝对值，超过 40° 连续
 * 50ms 后锁定停车；不参与原有水平面偏航导航。
 * 倒置安装仅改变符号，绝对值门槛保持不变。传感器 X 轴必须沿车体前后方向。
 */
#ifndef MPU6050_PITCH_STOP_ENABLED
#define MPU6050_PITCH_STOP_ENABLED     1
#endif
#if MPU6050_PITCH_STOP_ENABLED != 0 && MPU6050_PITCH_STOP_ENABLED != 1
#error "MPU6050_PITCH_STOP_ENABLED must be 0 or 1"
#endif
#define MPU6050_PITCH_STOP_DEG         40.0f
#define MPU6050_PITCH_CONFIRM_MS       50U
#define MPU6050_PITCH_FILTER_TAU_S     0.040f
/* 重力加速度偏离过大时姿态不可信（例如瞬态加速），不计入持续倾斜时间。 */
#define MPU6050_PITCH_MIN_G            0.70f
#define MPU6050_PITCH_MAX_G            1.30f

/* 路径可恢复停车后，连续有效单线自动恢复当前项；不从第 0 项重跑。
 * 不恢复按键/最终 STOP、无效路径、反向/过转、编码器或控制故障。
 */
#ifndef ROUTE_AUTO_RESUME_ENABLED
#define ROUTE_AUTO_RESUME_ENABLED     1
#endif
#if ROUTE_AUTO_RESUME_ENABLED != 0 && ROUTE_AUTO_RESUME_ENABLED != 1
#error "ROUTE_AUTO_RESUME_ENABLED must be 0 or 1"
#endif

/* 预设路径执行轮数：0 无限循环，1 单轮，N 连续执行 N 轮。
 * 非最后一轮的末项 STOP 按 ROUTE_REPEAT_ACTION 转向通过，离口去重后返回第 0 项。
 * 每次明确 START 清零已完成轮数；仅路径模式使用，赛道须能闭合回到首段。
 */
#ifndef ROUTE_REPEAT_COUNT
#define ROUTE_REPEAT_COUNT           0U
#endif
#if ROUTE_REPEAT_COUNT < 0 || ROUTE_REPEAT_COUNT > 4294967295ULL
#error "ROUTE_REPEAT_COUNT must be in 0..UINT32_MAX"
#endif

/* 当前闭合赛道末项需要左转；可选 LEFT / RIGHT / STRAIGHT。 */
#define ROUTE_REPEAT_ACTION         LEFT

/* 两种控制模式共用的启动包络；按实际 dt 累计，不阻塞传感器与停车保护。 */
#define STARTUP_RAMP_TIME_S           1.0f /* 明确 START 后升至正常目标的时间，单位 s。 */

/* ==================== 二、普通 PWM 模式参数 ====================
 * 仅在开关为 0 时使用；以下指令采用 0～255 标尺，并受 MOTOR_PWM_LIMIT 限制。
 * PWM 用于调节平均驱动电压，不表示已知轮速；不得与 cm/s 参数混用。
 */
#define OPEN_LOOP_BASE_PWM          255.0f  /* 目标线居中时满占空比，按上升斜坡达到。 */
#define OPEN_LOOP_CORRECTION_MAX_PWM 127.5f /* 半差上限；外轮满输出时内轮最低为零。 */
#define OPEN_LOOP_HOLD_PWM           35.0f  /* 完全丢线期间的基础 PWM 上限，不主动加速；歧义沿用正常基准。 */
#define OPEN_LOOP_PWM_RISE_PER_S (OPEN_LOOP_BASE_PWM / STARTUP_RAMP_TIME_S)
#define OPEN_LOOP_POSITION_KP         8.0f  /* 位置比例项，输出左右 PWM 的半差。 */
#define OPEN_LOOP_POSITION_KI         0.0f  /* 位置积分项初值，未标定前暂不启用。 */
#define OPEN_LOOP_POSITION_KD         0.08f /* 位置微分项，配合公共微分低通使用。 */

/* ==================== 三、轮径、计数比例与方向标定 ====================
 * 仅编码器模式要求这些尺寸有效且确认标志为 1；0 表示参数未知，禁止该模式启动。
 * 左右轮直径可独立标定，以补偿轮胎有效直径差异。
 * 每输出轮一圈计数必须已经包含减速比和 TI12 四倍频，不能重复乘入。
 * 若厂家只给电机轴单相每圈周期数，则输出轮计数约为该值 × 4 × 减速比。
 */
#define ODOMETRY_CALIBRATION_CONFIRMED 1    /* 2026-10-23 按用户要求启用当前标定配置；实物计数及方向待验证。 */
/* 2026-10-23 独立 1 m 测试：旧比例下实际走 1.53 m，乘 1.53 后用户确认距离正确。
 * 同步采用测试工程的公共距离系数，统一修正里程、反馈轮速和编码器估计航向。
 * 保留下面的轮径/每圈计数原值，不重复补偿；直线测距通过不代表转角/路径已实测。
 */
#define ODOMETRY_DISTANCE_SCALE       1.53f
/* 2026-10-29：本轮编码器估计97.59°，用户目测约75°，暂取75/97.59≈0.77。
 * 只修正轮差到航向的比例，不改直线距离/轮速；目测初值，非精确转角标定。
 * 主机理想几何回归可显式置1，生产固件仅使用本头文件默认值。 */
#ifndef ODOMETRY_HEADING_SCALE
#define ODOMETRY_HEADING_SCALE        0.77f
#endif
#define LEFT_WHEEL_DIAMETER_CM        4.8f /* 左轮有效直径，单位 cm。 */
#define RIGHT_WHEEL_DIAMETER_CM       4.8f /* 右轮有效直径，单位 cm。 */
/* 用户提供 L150PRO 参数：电机轴单相 13 周期/圈、减速比 30，TI12 四倍频后
 * 每输出轮一圈为 13 × 30 × 4 = 1560；已含减速与四倍频，实物计数及方向待核对。 */
#define LEFT_COUNTS_PER_WHEEL_REV  1560.0f /* 左输出轮完整一圈的最终四倍频计数。 */
#define RIGHT_COUNTS_PER_WHEEL_REV 1560.0f /* 右输出轮完整一圈的最终四倍频计数。 */
#define WHEEL_TRACK_CM              14.5f /* 用户于 2026-10-23 实测左右轮中心间距，单位 cm。 */
/* 2026-10-25 实测输入：灰度排中心在轮轴中点前方 16 cm。
 * 90° 切角以轴中点 R=10 cm 规划；须 R>track/2，保证内轮前进。
 * 灰度触发点近似路口中心：先直行 L-R=6 cm 至圆弧切点。
 */
#define GRAY_FORWARD_OFFSET_CM       16.0f
#define CORNER_RADIUS_CM             10.0f
#define CORNER_HEADING_KP            8.0f /* 编码器轨迹航向误差到轮速半差。 */
#define ENCODER_LEFT_SIGN            -1    /* 启动后原始计数 65527 指向负向回绕，取反；实车待复测。 */
#define ENCODER_RIGHT_SIGN            1    /* 右轮计数方向修正，只允许 +1 或 -1，前进应为正。 */

/* ==================== 四、编码器采样与轮速量程 ====================
 * 输入滤波作用于定时器硬件，轮速低通作用于软件换算后的 cm/s 测量值。
 * 计数跳变上界与目标速度上限用途不同；还需实测保证相邻采样不跨过半个计数器量程。
 */
#define ENCODER_INPUT_FILTER         6U     /* 两相输入捕获滤波配置值，不是毫秒数。 */
#define ODOMETRY_MAX_SPEED_CM_S      100.0f /* 计数合理性检查的轮速上界，另允许计数量化余量。 */
#define SPEED_FILTER_TAU_S           0.030f /* 测量轮速的一阶低通时间常数，单位 s。 */
#define WHEEL_TARGET_MAX_CM_S        120.0f  /* 每个车轮目标速度上限，单位 cm/s。 */

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
#define TARGET_RISE_CM_S2           160.0f  /* 恢复/切段加速斜率，单位 cm/s²；6→40 目标约 0.213 s，START 仍受 1 s 包络。 */
#define LINE_FOLLOW_SPEED_CM_S        12.0f /* 编码器仅巡线时的基础速度；路径模式改用表中 speed。 */

/* ==================== 八、丢线、图形干扰与航向保持 ====================
 * 两模式均冻结普通位置 PID，并从最后稳定白线起计时；孤立有效帧不刷新预算。
 * 歧义不额外降速；完全丢线按保持上限降速。编码器纠正估计航向并检查厘米距离；PWM 衰减历史差值。
 */
#define HOLD_SPEED_CM_S              40.0f   /* 编码器模式完全丢线保持的基础速度上限，单位 cm/s。 */
#define HEADING_KP_CM_S_PER_RAD       12.0f /* 每 rad 航向误差产生的目标速度半差，单位 (cm/s)/rad。 */
#define INTERFERENCE_TIMEOUT_MS     400U   /* 两模式共用：最后稳定观测起的异常总时限，单位 ms。 */
#define INTERFERENCE_MAX_DISTANCE_CM 8.0f  /* 仅编码器模式：进入异常后允许的里程偏移上限，单位 cm。 */

/* ==================== 九、路口候选确认 ====================
 * 仅开启预设路径时使用；还必须位于当前路径项距离窗口内，形状满足中心白线和所需分支。
 * LEFT/RIGHT 还接受连续中心线到窄侧边分支的短时历史，适配 L 形直角。
 * 普通候选由连续帧数、前进距离和时间窗口联合确认。
 * LEFT/RIGHT 在连续窄线后的整排骤变可优先强制执行，不要求仍在线上或目标侧分支。
 */
#define JUNCTION_CONFIRM_SAMPLES     3U    /* 候选路口至少连续出现的控制采样帧数。 */
#define JUNCTION_CONFIRM_DISTANCE_CM 0.10f /* 同一候选确认期间至少前进的距离，单位 cm。 */
#define JUNCTION_CONFIRM_TIMEOUT_MS  100U  /* 同一候选完成确认的最长时间，单位 ms。 */

#define JUNCTION_CENTER_MEMORY_MS     250U /* L 弯中心线历史的最长寿命；孤立帧不能建立历史。 */
#define JUNCTION_CENTER_MEMORY_CM     1.5f /* 同一距离窗口内中心线到侧边分支的最大前进距离。 */

/* 整排骤变使用极性转换和多数滤波后的 line_mask；仅当前 LEFT/RIGHT 距离窗口内生效。
 * 前一帧须属于连续 LINE_CONFIRM_SAMPLES 帧窄线，且间隔不超过合法控制周期上限。
 * 全丢线直接触发；其余要求至少 2 路目标，且满足下列任一明显变化。
 * 平滑单步横移 2 权重不会命中；这是形状启发式，不能识别真实曲线/元器件语义。
 */
#define JUNCTION_ABRUPT_CHANGED_BITS   4U   /* 相邻两帧至少四路翻转。 */
#define JUNCTION_ABRUPT_WIDTH_GROWTH   2U   /* 目标线路数一次增加至少两路。 */
#define JUNCTION_ABRUPT_POSITION_JUMP  4.0f /* 窄线横向位置一次跳变的权重阈值。 */

/* 仅路径模式：从表内路口标称位置前 5 cm 开始减速，不扣除距离容差。
 * 5 cm 是按用户反馈缩短后的待实车标定提前量；目标按速度平方随剩余距离插值，非实测制动距离。
 */
#define JUNCTION_APPROACH_DISTANCE_CM 5.0f
#define JUNCTION_APPROACH_SPEED_CM_S 20.0f /* 入弯前及候选基础目标，与转弯动作速度独立。 */

/* ==================== 十、路口动作、转角与离口去重 ====================
 * 转弯使用非负前进差速；正常几何出口确认、满 1 s 连续在线或超时在线可进入离口。
 * 转弯与离口分别使用有界超时预算，防止重复推进路径或无限保持。
 */
#define JUNCTION_ACTION_TIMEOUT_MS   3000U /* 90° 转弯时限；自定义大角度仍按角度比例扩展。 */
#define JUNCTION_STRAIGHT_TIMEOUT_MS 4500U /* 直行动作保留原时限。 */
#define JUNCTION_LINE_EXIT_AFTER_MS  1000U /* 转弯满 1 s 后，连续有效单线可结束转弯。 */
#define JUNCTION_CLEAR_TIMEOUT_MS    1500U /* 确认出口后的独立离口时限，早交接也不能刷新。 */
#define JUNCTION_TURN_MIN_RAD         1.35f /* 旧几何门槛供圆弧/原策略；普通非圆弧直角还须达到目标90°。 */
/* 编码器估计接近退出角且已连续沿中心出口线前进时，允许有界退出。
 * 仅在 1.30 rad 以上、连续窄线 50 ms 且前进 0.25 cm 时生效，最终仍需连续中心线。 */
#define JUNCTION_EXIT_ANGLE_MARGIN_RAD 0.05f
/* 原角度/连续窄线/前进证据成熟后，回线时允许额外 0.02 rad 角度回差；
 * 无成熟证据不放宽，小角度限制为目标绝对值的 5%。 */
#define JUNCTION_EXIT_HYSTERESIS_RAD  0.02f
#define JUNCTION_EXIT_CONFIRM_MS      50U
#define JUNCTION_EXIT_CONFIRM_CM      0.25f
#define JUNCTION_TURN_MAX_RAD         2.1f /* 超过此期望方向转角则异常停车，单位 rad。 */
#define JUNCTION_WRONG_WAY_RAD        0.35f /* 反向转角超过此值则异常停车，单位 rad。 */
#define JUNCTION_CENTER_LIMIT         2.0f /* 退出动作时允许的中心位置误差，单位为传感器权重。 */
#define STRAIGHT_ENTRY_ALIGN_RAD      0.05f /* 圆弧接 STRAIGHT：出口航向误差最多约 2.9°，连续 3 帧对齐后离口。 */
#define JUNCTION_TURN_SPEED_CM_S      60.0f /* 轴中点圆弧速度；18 对应内/外轮 4.95/31.05 cm/s。 */
#define JUNCTION_CAPTURE_MIN_RAD     0.70f /* 约 40° 后允许捕获侧边窄出口线，仍须满足最终退出转角。 */
#define JUNCTION_CAPTURE_SPEED_CM_S   6.0f /* 捕获后前进找正的基础目标，避免慢速扫线代替沿线收敛。 */
#define JUNCTION_CAPTURE_MAX_HALF_DIFF_CM_S 12.0f /* 非圆弧见线后位置半差上限，不再叠加高速放大。 */
#define JUNCTION_CAPTURE_LOSS_GRACE_MS 30U /* 已见线后短暂失线先直行，避免立即恢复大差速扫转。 */
#define JUNCTION_CLEAR_CENTER_MS       50U /* 离口交接须连续中心线，不能偶尔扫过中心就成功。 */
#define JUNCTION_CLEAR_CENTER_CM       2.0f
#define JUNCTION_CLEAR_MAX_CM         20.0f /* 尚未稳定回线时禁止沿旧航向跑出数十厘米。 */
#define JUNCTION_RECAPTURE_LOOKAHEAD_CM 8.0f /* 以已见中心线点和规划出口轴进行有界回线。 */
#define JUNCTION_RECAPTURE_BIAS_RAD    0.35f
#define JUNCTION_RECAPTURE_MAX_HALF_RATIO 0.4f /* 回线半差请求比例上限；反馈分配后内轮仍保持正向。 */
#define STEERING_FEEDBACK_MIN_CM_S      5.0f /* 外轮已可靠前进时，用实测轮速保留内轮纠偏余量。 */
#define STEERING_FEEDBACK_MIN_HALF_CM_S 0.1f
#define JUNCTION_CAPTURE_MIN_TURN_CM_S 0.8f /* 最小退出转角前的转向半差下限，防止位置/航向修正抵消。 */
#define JUNCTION_CAPTURE_HEADING_KP   8.0f /* 捕获阶段向入口 ±90° 施加的航向半差，单位 (cm/s)/rad。 */
#define JUNCTION_TURN_TARGET_RAD      1.570796327f /* 直角动作的估计航向目标，单位 rad。 */
#define JUNCTION_TURN_SLOW_RAD        1.25f /* 约 72° 后降低外轮目标，减少冲过出口目标线。 */
#define JUNCTION_TURN_SLOW_CM_S       3.0f /* 有线航向修正的速度归一化参考；无线续转不采用此低速上限。 */
#define JUNCTION_CLEAR_DISTANCE_CM    3.0f /* 直行退出的最小前进距离，以及离口去重阶段的额外距离，单位 cm。 */
/* 下一路口已进入标称位置前 5 cm 时，允许缩短上一弯的离口去重距离。
 * 仅已完成转角/出口确认的转弯使用；仍需这里的前进距离及新的连续窄线保持。 */
#define JUNCTION_SHORT_CLEAR_CM       0.5f
/* 紧邻路口的出口中心线附带入口侧孤立边缘分支：仅在已确认转弯内使用。 */
#define JUNCTION_BRANCH_EXIT_OVERRUN_RAD 0.35f
#define JUNCTION_BRANCH_EXIT_CONFIRM_MS 100U
#define JUNCTION_BRANCH_EXIT_CONFIRM_CM 0.5f


/* ==================== 十一、路径距离窗口保护 ====================
 * 启动建立首段基准，切段保留转弯及离口的出口方向位移；整段行驶不设时间限制。
 * 越过距离窗口仍未确认路口则失败停车；路口确认和动作/离口仍有各自时限。
 * STOP 同样要求在窗口内确认路口，不能仅因到达预计里程就调用成功回调。
 */

#endif
