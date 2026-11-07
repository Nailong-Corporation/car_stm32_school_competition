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

/* MPU6050 航向选择：0 编码器轮差估角；1 优先 IMU，失败锁定降级编码器。
 * 当前 1；初始化失败不阻塞 START，运行丢样从当前航向继续累计轮差。
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
#define MPU6050_POWER_SETTLE_MS         100U /* 模块电源启动后再探测，电机保持待机。 */
#define MPU6050_PROBE_ATTEMPTS            5U
#define MPU6050_PROBE_RETRY_MS           20U
#define MPU6050_INIT_ATTEMPTS             3U /* 通信/配置异常后复位 I2C 并完整重试。 */
#define MPU6050_INIT_RETRY_MS           100U
#define MPU6050_BUS_EDGE_MS               1U /* 仅启动总线恢复，开漏电平转换后等待。 */
#define MPU6050_BUS_WAIT_MS               3U /* SCL/SDA 电平确认有界，不在运动中执行。 */
/* 实际 MPU6050 倒置安装（芯片面朝下、芯片 Z 正轴朝地面），车体右转时
 * 在传感器坐标下读得正 Z 角速度：+1 对应全系统右转正航向。
 * 仅使用水平面偏航角，不融合俯仰/横滚；必须近似水平固定并手持右转核实。 */
#define MPU6050_YAW_SIGN               1
#define MPU6050_GYRO_SETTLE_MS         500U /* 唤醒/配置后等待，随后校验静止零偏。 */
#define MPU6050_GYRO_BIAS_SAMPLES       32U /* 每窗名义 160 ms；静止质量门限不变。 */
#define MPU6050_GYRO_CALIBRATION_MS    5U
#define MPU6050_CALIBRATION_ATTEMPTS   5U /* 质量不合格时重采独立窗口，持续异常仍拒绝。 */
#define MPU6050_I2C_TIMEOUT_MS         2U
#define MPU6050_INIT_I2C_TIMEOUT_MS   25U /* 电机待机时允许更长初始化事务，运行期仍 2 ms。 */

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

/* Historical retry switch retained for compatibility; route_resume always rejects. */
#ifndef ROUTE_AUTO_RESUME_ENABLED
#define ROUTE_AUTO_RESUME_ENABLED     0
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
#define ODOMETRY_CALIBRATION_CONFIRMED 1    /* 沿用软件标定门禁确认；换轮后距离比例与转角仍待实测重标。 */
/* 历史：2026-10-23 独立 1 m 测试曾用 1.53 修正旧轮径；当前公共距离系数为 2.15。
 * 2026-11-02 更换 6.5 cm 轮后保留此系数和每圈计数 1560，均需按新轮重测。
 * 此系数只在每计数轮程处乘一次，里程、反馈轮速和编码器估计航向共用。
 */
#define ODOMETRY_DISTANCE_SCALE       3.05f
/* 2026-10-29：本轮编码器估计97.59°，用户目测约75°，暂取75/97.59≈0.77。
 * 只修正轮差到航向的比例，不改直线距离/轮速；目测初值，非精确转角标定。
 * 主机理想几何回归可显式置1，生产固件仅使用本头文件默认值。 */
#ifndef ODOMETRY_HEADING_SCALE
#define ODOMETRY_HEADING_SCALE        0.77f
#endif
#define LEFT_WHEEL_DIAMETER_CM        6.5f /* 左轮有效直径，单位 cm（2026-11-02 用户更新为 65mm）。 */
#define RIGHT_WHEEL_DIAMETER_CM       6.5f /* 右轮有效直径，单位 cm（2026-11-02 用户更新为 65mm）。 */
/* 用户提供 L150PRO 参数：电机轴单相 13 周期/圈、减速比 30，TI12 四倍频后
 * 每输出轮一圈为 13 × 30 × 4 = 1560；已含减速与四倍频，实物计数及方向待核对。 */
#define LEFT_COUNTS_PER_WHEEL_REV  1560.0f /* 左输出轮完整一圈的最终四倍频计数。 */
#define RIGHT_COUNTS_PER_WHEEL_REV 1560.0f /* 右输出轮完整一圈的最终四倍频计数。 */
#define WHEEL_TRACK_CM              14.0f /* 2026-11-02 用户更新两轮中心间距，单位 cm。 */
/* 2026-11-02 用户更新：轮轴中心距离灰度传感器中心距离为 24 cm。
 * 90° 圆弧 R=L=24 cm，路口识别后从当前位置开始规划；须 R>track/2。
 * 触发点与实际路口中心的偏差以及轮胎打滑需实车验证。
 */
#define GRAY_FORWARD_OFFSET_CM       24.0f
#define CORNER_RADIUS_CM             24.0f
#define CORNER_MAX_HALF_RATIO         0.8f /* 圆弧航向修正有界，保证两轮目标均为正。 */
#define CORNER_HEADING_KP            8.0f /* 编码器轨迹航向误差到轮速半差。 */
#define ENCODER_LEFT_SIGN             1    /* 2026-10-29 电机极性反转后启动快照左原始计数 +1，旧 -1 触发反向停车；轮向待复测。 */
#define ENCODER_RIGHT_SIGN           -1    /* 2026-10-29 与测距工程一致：右原始计数 0→65535，乘 -1 后为正反馈；实车轮向仍待核对。 */

/* ==================== 四、编码器采样与轮速量程 ====================
 * 输入滤波作用于定时器硬件，轮速低通作用于软件换算后的 cm/s 测量值。
 * 计数跳变上界与目标速度上限用途不同；还需实测保证相邻采样不跨过半个计数器量程。
 */
#define ENCODER_INPUT_FILTER         6U     /* 两相输入捕获滤波配置值，不是毫秒数。 */
/* 2026-10-30 用户信息：小车最高可能达到 5 m/s（500 cm/s），量程与目标体系按此预留。 */
#define VEHICLE_MAX_SPEED_CM_S       500.0f /* 车辆最高可能轮速；前馈换算基准，待实测确认。 */
#define ODOMETRY_MAX_SPEED_CM_S      650.0f /* 计数合理性检查轮速上界 = 5 m/s×1.3 余量；超过按计数跳变故障。 */
#define SPEED_FILTER_TAU_S           0.030f /* 测量轮速的一阶低通时间常数，单位 s。 */
#define WHEEL_TARGET_MAX_CM_S        500.0f /* 单轮目标速度天花板；路径表 cruise/turn 均不得超过。 */

/* ==================== 五、左右轮速度 PID 内环 ====================
 * 仅编码器模式使用；误差为目标速度减实测速度，单位 cm/s，最终输出为 PWM 指令。
 * 左右轮分别保存 PID 历史，前馈与 PID 相加后按 0～MOTOR_PWM_LIMIT 做抗饱和限幅。
 */
#define SPEED_KP                     2.0f  /* 速度比例系数。 */
#define SPEED_KI                     8.0f  /* 速度积分系数，实际积分使用 dt。 */
#define SPEED_KD                     0.0f  /* 速度微分系数初值，当前为 0。 */
#define SPEED_INTEGRAL_LIMIT         10.0f /* 速度误差积分的绝对值上限，积分单位为 cm。 */
/* 2026-10-30 按 5 m/s 满量程修正：前馈 = 满 PWM 255 ÷ 500 cm/s ≈ 0.51 PWM/(cm/s)。
 * 旧值 3.0 在约 85 cm/s 即饱和，差速修正随之失效；仍为初值，需台架实测重标。 */
#define SPEED_FEEDFORWARD             (255.0f / VEHICLE_MAX_SPEED_CM_S)

/* ==================== 六、编码器反馈与堵转保护 ====================
 * 目标达到指定值而测得速度持续偏低时停车；反向反馈越过负阈值也会报故障。
 * 这些判断不能覆盖所有断线或打滑情况，阈值与超时须按编码器分辨率及启动特性标定。
 */
#define SPEED_STALL_TARGET_CM_S       3.0f  /* 目标至少达到此速度时才累计低速保护时间。 */
#define SPEED_STALL_MEASURED_CM_S     0.5f  /* 堵转低速判定阈值。 */
#define SPEED_STALL_TIMEOUT_S         0.350f /* 低速持续到此秒数即故障停车。 */
/* 单计数负反馈先撤去双轮驱动，持续/位移确认后锁定；零目标也检测。
 * 以下是未实测初值，待机不等同于已经验证的制动。 */
#define SPEED_REVERSE_THRESHOLD_CM_S  0.5f
#define SPEED_REVERSE_CONFIRM_S       0.050f
#define SPEED_REVERSE_CONFIRM_CM      0.10f
#define SPEED_REVERSE_IMMEDIATE_CM_S  5.0f

/* ==================== 七、白线位置外环与目标速度规划 ====================
 * 仅编码器模式使用；位置误差来自 -7～+7 权重，PID 输出为两轮目标速度的半差（cm/s）。
 * 正修正使左目标速度大、右目标速度小，期望向右转；普通 PWM 模式使用上方独立参数。
 */
#define POSITION_KP_CM_S              1.8f   /* 白线位置比例系数。 */
#define POSITION_KI_CM_S              0.0f   /* 白线位置积分系数初值。 */
#define POSITION_KD_CM_S              0.025f /* 白线位置微分系数。 */
#define POSITION_CURVE_SLOWDOWN       0.0f  /* 最大位置偏差时，相对路段基础速度的降速比例。 */
/* 转弯速度功能独立开关：修改头文件后 pio run，不增加构建环境。 */
#ifndef TURN_APPROACH_SLOWDOWN_ENABLED
#define TURN_APPROACH_SLOWDOWN_ENABLED 1
#endif
#define JUNCTION_APPROACH_SPEED_CM_S 50.0f /* 未知长度末段总上限；转弯还受实际入弯慢速上限约束。 */
#define BRAKING_ACCEL_FILTER_TAU_S 0.080f /* 轮速差分/IMU 纵向加速度低通。 */
#define BRAKING_IMU_TILT_LIMIT_DEG 5.0f /* 相对静止基线超限则使用编码器差分。 */
#define BRAKING_REVERSE_MAX_MS 250U /* 每段累计反向力矩时间上限。 */
#define BRAKING_REVERSE_MIN_SPEED_CM_S 8.0f /* 低速前切断反向输出，禁止倒车。 */
#define BRAKING_REVERSE_GUARD_CM 3.0f /* 预留反向作用期间的距离/反应余量。 */
/* IMU X 正向和反向力矩需架空/低速核对后分别启用。未确认时仍以编码器差分规划，
 * 若被动减速不能满足距离约束，路径入口继续停车，不假定反转有效。 */
#ifndef BRAKING_IMU_ACCEL_SIGN_CONFIRMED
#define BRAKING_IMU_ACCEL_SIGN_CONFIRMED 0
#endif
#define BRAKING_IMU_ACCEL_SIGN 1
#ifndef BRAKING_REVERSE_HARDWARE_CONFIRMED
#define BRAKING_REVERSE_HARDWARE_CONFIRMED 0
#endif
#if (BRAKING_IMU_ACCEL_SIGN_CONFIRMED != 0 && BRAKING_IMU_ACCEL_SIGN_CONFIRMED != 1) || \
    (BRAKING_REVERSE_HARDWARE_CONFIRMED != 0 && BRAKING_REVERSE_HARDWARE_CONFIRMED != 1)
#error "Braking hardware confirmation flags must be 0 or 1"
#endif
#ifndef TURN_EXIT_ACCEL_ENABLED
#define TURN_EXIT_ACCEL_ENABLED        1
#endif
#ifndef MOTOR_REVERSE_BRAKE_ENABLED
#define MOTOR_REVERSE_BRAKE_ENABLED    0
#endif
#if BRAKING_REVERSE_HARDWARE_CONFIRMED && (!MOTOR_REVERSE_BRAKE_ENABLED || !PRESET_ROUTE_ENABLED)
#error "Reverse approach braking requires route and reverse motor control"
#endif
#if (TURN_APPROACH_SLOWDOWN_ENABLED != 0 && TURN_APPROACH_SLOWDOWN_ENABLED != 1) || \
    (TURN_EXIT_ACCEL_ENABLED != 0 && TURN_EXIT_ACCEL_ENABLED != 1) || \
    (MOTOR_REVERSE_BRAKE_ENABLED != 0 && MOTOR_REVERSE_BRAKE_ENABLED != 1)
#error "Turn speed switches must be 0 or 1"
#endif
#define TURN_EXIT_ACCEL_CM_S2        400.0f /* 出口确认后跨切段快速提速；原普通斜率160。 */
#define REVERSE_BRAKE_MARGIN_CM_S      3.0f /* 实测高于目标此值才施加反向力矩。 */
#define REVERSE_BRAKE_KP              4.0f /* 超速差映射到反向PWM，非倒车目标。 */
#define REVERSE_BRAKE_MAX_PWM        60.0f /* 255标尺下的反向指令上限。 */

#define TARGET_RISE_CM_S2           160.0f  /* 恢复/切段加速斜率，单位 cm/s²；6→40 目标约 0.213 s，START 仍受 1 s 包络。 */
#define LINE_FOLLOW_SPEED_CM_S        50.0f /* 编码器仅巡线时的基础速度；路径模式改用表中 cruise_speed。 */

/* ==================== 八、丢线、图形干扰与航向保持 ====================
 * 两模式均冻结普通位置 PID，并从最后稳定白线起计时；孤立有效帧不刷新预算。
 * 歧义按 AMBIGUOUS_SPEED_CM_S 限速；完全丢线按 HOLD_SPEED_CM_S 保持。编码器纠正估计航向并检查厘米距离；PWM 衰减历史差值。
 */
#define HOLD_SPEED_CM_S              40.0f   /* 编码器模式完全丢线保持的基础速度上限，单位 cm/s。 */
#define AMBIGUOUS_SPEED_CM_S         80.0f   /* 宽线/多区域歧义期间的基础速度上限，单位 cm/s。 */
#define HEADING_KP_CM_S_PER_RAD       12.0f /* 每 rad 航向误差产生的目标速度半差，单位 (cm/s)/rad。 */
#define INTERFERENCE_TIMEOUT_MS     400U   /* 两模式共用：最后稳定观测起的异常总时限，单位 ms。 */
#define INTERFERENCE_MAX_DISTANCE_CM 8.0f  /* 仅编码器模式：进入异常后允许的里程偏移上限，单位 cm。 */

/* ==================== 九、方向无关路口候选与动作 ====================
 * 近期中心历史 + 任意侧分支，连续帧/位移联合确认。失线与单帧骤变不触发。
 */
#define JUNCTION_CONFIRM_SAMPLES       3U
#define JUNCTION_CONFIRM_DISTANCE_CM   0.10f
#define JUNCTION_EARLY_CONFIRM_SAMPLES  2U /* 仅目标普通左右转、同侧强中心分支提前确认。 */
#define JUNCTION_EARLY_CONFIRM_CM       0.05f
#define JUNCTION_EDGE_CONFIRM_SAMPLES    4U /* 弱证据：边缘L形或分离分支，不与强候选共用三帧门槛。 */
#define JUNCTION_EDGE_CONFIRM_CM        0.15f
#define JUNCTION_EDGE_MAX_WIDTH           3U
#define JUNCTION_SMOOTH_POSITION_STEP    2.0f /* 重叠单线逐步横移，不能单凭移到边缘计数。 */
#define JUNCTION_CONFIRM_TIMEOUT_MS    100U
#define JUNCTION_CENTER_MEMORY_MS      250U
#define JUNCTION_CENTER_MEMORY_CM      1.5f
#define JUNCTION_ACTION_TIMEOUT_MS     3000U /* 90 degrees; larger custom angles scale proportionally. */
#define JUNCTION_STRAIGHT_TIMEOUT_MS   4500U
#define JUNCTION_CLEAR_TIMEOUT_MS      1500U
#define JUNCTION_TURN_MIN_RAD          1.35f /* Threshold scaling helper reference; exit requires actual target. */
#define JUNCTION_EXIT_CONFIRM_MS       50U
#define JUNCTION_EXIT_CONFIRM_CM       0.25f
/* 角度余量只配合加倍出口证据（100 ms / 0.5 cm）生效；
 * 余量 = 目标角 5%（直角 4.5°），满角度只需连续中心线帧数。
 * 动作超时仍不能豁免角度；错向/过转/超时保护不变。 */
#define JUNCTION_EXIT_ANGLE_MARGIN_RATIO 0.05f
#define JUNCTION_PASS_SPEED_CM_S       60.0f /* 非目标路口直行通过速度上限初值；仍受路段 cruise 封顶。 */
#define JUNCTION_TURN_MAX_RAD          2.1f
#define JUNCTION_WRONG_WAY_RAD         0.35f
#define JUNCTION_CENTER_LIMIT          2.0f
#define JUNCTION_CAPTURE_MIN_RAD       0.70f
#define JUNCTION_EXIT_ALIGNMENT_RAD    0.15f /* Capture alone does not certify a completed 90-degree turn. */
/* ±90° 动作的轮轴接近距离：节点由 8 路杆最外通道先看到（半跨约 3.85 cm），
 * 确认后还要滑行，因而不宜用满 24 cm。该值与仿真器 TURN_PIVOT_APPROACH_CM 同步。 */
#define TURN_PIVOT_APPROACH_CM         17.0f

/* 转弯已确认出口后再次丢线时，把保持航向向弯内侧回一点：枢轴可能早于真实
 * 转角数厘米，出口线会落在 8 路杆半跨之外。与仿真器 JUNCTION_EXIT_REWIND_RAD 同步。 */
#define JUNCTION_EXIT_REWIND_RAD       0.17f

#define JUNCTION_APPROACH_TIMEOUT_MS   3000U
#define JUNCTION_CLEAR_MAX_CM          20.0f
#define STEERING_FEEDBACK_MIN_CM_S     5.0f
#define STEERING_FEEDBACK_MIN_HALF_CM_S 0.1f
#define JUNCTION_CAPTURE_MIN_TURN_CM_S  0.8f
#define JUNCTION_CAPTURE_CENTER_BIAS_CM_S 1.2f /* 仅居中且PID近零时补角；不覆盖非零位置纠偏。 */
#define JUNCTION_CAPTURE_HEADING_KP    8.0f
#define JUNCTION_TURN_TARGET_RAD       1.570796327f
/* 未标定的转弯初值：动作开始即限外轮目标，近目标角切换航向增益；阻尼不是物理制动。 */
#define JUNCTION_SETTLE_BAND_RAD       0.60f /* 临近目标角时切换到收敛航向增益。 */
#define JUNCTION_PASS_ALIGNED_RAD      0.10f /* 连续中心且回正至出口轴约5.7°后，释放临时过口航向。 */
#define JUNCTION_PASS_SETTLED_YAW_RAD_S 0.15f /* 稳定中心且低角速度也可释放，避免累积角误差保留旧轴。 */
#define JUNCTION_SETTLE_SPEED_CM_S    28.0f /* 非直行转弯从动作开始的外轮目标上限；待实车标定。 */
#define JUNCTION_SETTLE_HEADING_KP    24.0f /* 近角度半差比例，受外轮上限约束。 */
#define JUNCTION_YAW_DAMPING           2.5f /* (cm/s)/(rad/s)，输出轮速半差。 */
#define JUNCTION_YAW_FILTER_TAU_S      0.030f
#define JUNCTION_EXIT_SPEED_CM_S      20.0f /* 转弯 CLEARING 上限，另受 turn_speed/cruise 限制。 */
#define COMPONENT_RECOVERY_SPEED_CM_S 16.0f /* 已捕线近出口/离口遇图形：保持进入时航向找线。 */
#define COMPONENT_RECOVERY_MAX_CM     12.0f
#define COMPONENT_RECOVERY_TIMEOUT_MS 1200U

/* Count-navigation braking: measured calibration required before setting lengths. */
#ifndef BRAKING_CALIBRATION_CONFIRMED
#define BRAKING_CALIBRATION_CONFIRMED 1
#endif
#ifndef BRAKING_SAFE_DECEL_CM_S2
#define BRAKING_SAFE_DECEL_CM_S2 120.0f
#endif
#define BRAKING_DISTANCE_MARGIN_CM 2.0f
#define BRAKING_TARGET_JERK_CM_S3 800.0f
#define MPU6050_CALIBRATION_MAX_BIAS_LSB 328.0f
#define MPU6050_CALIBRATION_MAX_CENTRAL_SPAN_LSB 64.0f
#define MPU6050_MAX_YAW_RATE_RAD_S 12.0f
#define JUNCTION_SEARCH_TIMEOUT_MS 2000U
#define JUNCTION_SEARCH_MAX_CM 12.0f
/* 2026-11-06 新版赛道：印刷元件符号（空心电阻/三极管/电池）在节点附近遮住导线，
 * 节点确认可能晚于几何路口，入口降速预算相应放宽；仍为有界保护，超限进入恢复停车。 */
#define JUNCTION_ENTRY_BRAKE_TIMEOUT_MS 2000U /* 入口滑行降速时限。 */
#define JUNCTION_ENTRY_BRAKE_MAX_CM 24.0f     /* 入口滑行降速距离。 */
#define JUNCTION_COUNT_CLEAR_MS 50U
#define JUNCTION_COUNT_CLEAR_CM 2.0f /* 离口重新武装距离：防同一路口尾缘重复计数；路口间距须大于此值。 */
#define JUNCTION_STRAIGHT_EXIT_MAX_WIDTH 3U /* 直行过图形的出口：居中且至多3路，侧边/宽图形不解锁。 */
#define JUNCTION_ADJACENT_EXIT_MS 20U /* 已确认转弯出口后，短中心走廊与新分离分支联合交接。 */
#define JUNCTION_ADJACENT_EXIT_CM 0.30f
#define OPPORTUNISTIC_FALLBACK_ENABLED 0

/* ==================== HEXAGON: all values are uncalibrated bounds ====================
 * Signed angle is net unwrapped yaw since action entry, NOT polygon vertex count.
 * Entry/exit guides assume tangent straight exit with an opposite-side branch.
 */
#define HEXAGON_MAX_ANGLE_DEG          720.0f
#define HEXAGON_SPEED_CM_S             24.0f
#define HEXAGON_ENTRY_MIN_RAD           0.35f
#define HEXAGON_ENTRY_TIMEOUT_MS     3000U
#define HEXAGON_ENTRY_MAX_CM           30.0f
#define HEXAGON_ACTION_TIMEOUT_MS    30000U /* Separate from ordinary 3000/4500 ms turns. */
#define HEXAGON_ACTION_MAX_CM         400.0f
#define HEXAGON_EXIT_TIMEOUT_MS       3000U
#define HEXAGON_EXIT_MAX_CM            30.0f
#define HEXAGON_OVERANGLE_RAD           2.0943951f /* at most 120 deg after minimum yaw */
#define HEXAGON_EXIT_CONFIRM_MS        100U
#define HEXAGON_EXIT_CONFIRM_CM          0.5f
/* Entry may need to cross the branch before seeing the ring. Bounded blind guide. */
#define HEXAGON_ENTRY_LOST_MS         1200U
#define HEXAGON_ENTRY_LOST_CM           12.0f
/* Once ring line is captured, use existing 400 ms/8 cm bounds plus one graphic recovery. */

/* 2026-11-04: total-segment planning; table cruise applies only above 100 cm. */
#ifndef ROUTE_SHORT_SPEED_CM_S
#define ROUTE_SHORT_SPEED_CM_S 60.0f
#endif
#define ROUTE_LONG_SEGMENT_CM 100.0f
/* RECOVERING: sustained loss since last confirmed narrow line, not phase duration. */
#define NAVIGATION_RECOVERY_MS 1200U
#define NAVIGATION_RECOVERY_CM 12.0f
#define NAVIGATION_RECOVERY_SPEED_CM_S 16.0f

/* ==================== 2026-11-07 pose (encoder/IMU) route navigation ====================
 * Mirrors /Users/zhendong/Documents/stm32car_simulator src/sim/config.ts.
 * The route table stores geometry lengths reduced by the 5 cm distance margin;
 * the absolute corner waypoint is recovered from the start pose plus the
 * cumulative (length, angle) chain.  The 8-channel bar still confirms every
 * corner it can see; the pose check runs in parallel so a printed symbol that
 * hides the wire (Q22 transistor) or a leg shorter than the 24 cm look-ahead
 * cannot stall the run (user-approved 2026-11-07 "grey first, pose fallback").
 * Host regression builds pass -DPOSE_NAV_ENABLED=0 to keep the legacy
 * grey-only contract; production keeps 1. */
#ifndef POSE_NAV_ENABLED
#define POSE_NAV_ENABLED 1
#endif
#define ROUTE_MAX_STEPS 32U
#define ROUTE_START_X_CM 0.0f
#define ROUTE_START_Y_CM 110.0f
#define ROUTE_START_HEADING_RAD (-1.570796327f) /* track frame: 0=+x east, +pi/2=+y south */
#define ROUTE_WAYPOINT_MARGIN_CM 5.0f
#define ROUTE_WAYPOINT_CONFIRM_CM 2.5f /* bar-to-corner radius that confirms a row */
#define ROUTE_WAYPOINT_FALLBACK_CM 18.0f /* odometry overrun that still hands the row over */
#define ROUTE_WAYPOINT_RING_CONFIRM_CM 3.0f /* ring vertex: axle, not the look-ahead bar */
#define ROUTE_WAYPOINT_GATE_CM 12.0f /* grey node must be this close to the row corner */
#define ROUTE_WAYPOINT_EXIT_CM 14.0f /* pose turn hand-off distance from the corner */
#define ROUTE_WAYPOINT_EXIT_ANGLE_RAD 0.12f /* looser hand-off covers a faster pivot's residual */
#define ROUTE_WAYPOINT_SLOW_CM 30.0f /* baseline pre-corner braking window */
#define ROUTE_CLEAR_POSE_CM 2.0f /* pose-mode clearance travel before the next row */
#define JUNCTION_POSE_GRAY_SPREAD 4.0f /* grey position (~0.55 cm/unit) vs pose cross-track */
#define JUNCTION_CROSS_KP 2.0f /* (cm/s) differential per cm of pose cross-track */
#define POSE_PURSUIT_LOOKAHEAD_CM 30.0f
#define POSE_GRAY_TRIM 0.25f
/* Hexagon ring geometry: regular hexagon, side 35 cm, right vertex (345,165).
 * Pose polygon: entry turn 30 deg at V0, five 60 deg vertices, exit turn 30 deg. */
#define HEXAGON_CENTER_X_CM 310.0f
#define HEXAGON_CENTER_Y_CM 165.0f
#define HEXAGON_SIDE_CM 35.0f
#define HEXAGON_TURN_RAD 1.047197551f
#define HEXAGON_ENTRY_TURN_RAD 0.523598776f
#define HEXAGON_TURN_TOL_RAD 0.06f
#define HEXAGON_ENTRY_APPROACH_MAX_CM 35.0f
#define HEXAGON_LEG_CROSS_MAX_CM 10.0f

#endif
