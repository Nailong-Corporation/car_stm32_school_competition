#ifndef ROUTE_H
#define ROUTE_H
#include "gray_sensor.h"
#include <stddef.h>

/* 距离列填 AUTO：按下一路口触发，完全忽略 tolerance。 */
#define AUTO (-1.0f)

typedef enum { LEFT, RIGHT, STRAIGHT, STOP, CUSTOM_TURN } Action;
typedef struct {
    float distance;   /* cm from start / previous junction reference point, or AUTO */
    float tolerance;  /* inclusive +/- cm intersection window; ignored for AUTO */
    Action action;    /* 数字 STRAIGHT 为里程点；AUTO 各动作均由路口形状触发。 */
    int speed;        /* target forward speed, cm/s, NOT PWM */
    float angle_deg;  /* CUSTOM_TURN only: (0, 360] clockwise, [-360, 0) counterclockwise. */
    bool corner_arc_enabled; /* true: 90° 圆弧切角；false: 原转弯控制。 */
} RouteStep;
typedef enum {
    ROUTE_FOLLOW, ROUTE_CANDIDATE, ROUTE_EXECUTING, ROUTE_CLEARING,
    ROUTE_FINISHED, ROUTE_FAILED, ROUTE_DISABLED
} RouteState;
/* 保留首个失败原因，区分可重启的路径停车与编码器/任务锁定故障。 */
typedef enum {
    ROUTE_FAILURE_NONE, ROUTE_FAILURE_INVALID, ROUTE_FAILURE_WINDOW,
    ROUTE_FAILURE_CONFIRM_TIMEOUT = 4, /* 旧路段超时值 3 停用，其余诊断编号保持。 */
    ROUTE_FAILURE_ACTION_TIMEOUT, ROUTE_FAILURE_TURN_LIMIT, ROUTE_FAILURE_WRONG_WAY,
    ROUTE_FAILURE_EXIT_RECAPTURE /* 8：捕获/离口超距仍未稳定回线；不自动恢复跳段。 */
} RouteFailure;
typedef void (*RouteStopCallback)(size_t step_index, void *context);
typedef struct {
    const RouteStep *steps;
    size_t count, index;
    RouteState state;
    RouteFailure failure;
    float origin_cm, entry_cm, entry_heading, clear_cm, approach_center_cm;
    float segment_heading_rad, action_start_heading; /* 短段规划入口轴 / 实际动作起始航向。 */
    bool recovery_window_active;
    float recovery_window_origin_cm; /* 自动恢复后检测窗口起点；原始路段里程仍保留。 */
    bool segment_heading_valid; /* 短段继承上一项规划出口轴。 */
    float outgoing_cm, projection_previous_cm, projection_previous_heading; /* 入口灰度参考点以来，灰度中心在出口方向的净位移。 */
    uint32_t segment_ms, phase_ms, approach_center_ms;
    uint32_t exit_candidate_ms;
    float exit_candidate_cm;
    bool exit_candidate_pending; /* 接近退出角时的连续中心出口线证据。 */
    Action repeat_action; /* 非最终轮末项的闭合动作，由 motion_config.h 初始化。 */
    uint32_t repeat_count, completed_laps; /* 0 无限；完成末项路口确认后累计，START 清零。 */
    uint8_t candidate_count, exit_count, approach_center_count;
    uint8_t previous_line_mask, narrow_count;
    uint32_t previous_line_ms;
    float previous_line_position;
    bool have_approach_center;
    bool valid, stop_pending, callback_dispatched;
    bool corner_arc_complete; /* 90° 轨迹完成后保持捕线，避免纠偏回落时重进圆弧。 */
    bool turn_capturing, lap_boundary; /* 出口线捕获 / 非末轮 STOP 的闭合动作。 */
    bool capture_line_seen, line_exit_confirmed;
    uint32_t capture_line_ms;
    float capture_start_cm;
    float line_exit_heading_rad; /* 连续实线确认后保存实际捕获航向，不再强凑估计90°。 */
    bool clear_center_pending;
    uint32_t clear_center_ms, clear_sample_ms;
    float clear_center_cm;
    bool turn_forced; /* 本项由整排骤变触发；保持至下一路段，便于诊断。 */
    RouteStopCallback stop_callback;
    void *callback_context;
    bool handoff_early; /* 当前段由短距离离口交接，保持至下一项或 START。 */
    float segment_start_progress_cm; /* 切段时继承的出口方向位移，供诊断。 */
    bool turn_exit_branch; /* 本帧为有界中心出口 + 孤立入口侧边缘分支，仅路径航向控制使用。 */
    uint8_t online_samples; /* 执行动作期间相邻有效周期的连续单线样本。 */
    uint32_t online_ms;
    bool turn_ended_on_line; /* 本项因满 1 s 在线或超时在线转入离口；切段/START 清零。 */
    uint8_t exit_line_mask; /* 本帧路径出口区域；不修改原始灰度观测。 */
    uint8_t loss_branch_samples;
    bool loss_branch_ready;
    uint32_t loss_branch_ms;
    float loss_branch_cm;
} RoutePlanner;

extern const RouteStep route[];
extern const size_t route_count;
void route_init(RoutePlanner *planner, const RouteStep *steps, size_t count,
                 RouteStopCallback callback, void *context);
void route_start(RoutePlanner *planner, float distance_cm, uint32_t now_ms);
/* 调用者已完成停车后的连续有效单线确认；保留索引/圈数和真实里程。 */
bool route_resume(RoutePlanner *planner, RouteState failed_phase, float distance_cm,
                  float heading_rad, uint32_t now_ms);
void route_update(RoutePlanner *planner, GrayObservation obs, float distance_cm,
                   float heading_rad, uint32_t now_ms);
/* 返回有效方向：CUSTOM_TURN 按符号映射 LEFT/RIGHT；非末轮 STOP 使用闭合动作。 */
Action route_current_action(const RoutePlanner *planner);
/* Effective LEFT/RIGHT above includes CUSTOM_TURN direction; signed target stays unwrapped. */
bool route_uses_corner_arc(const RoutePlanner *planner);
float route_turn_target_rad(const RoutePlanner *planner);
float route_turn_threshold_rad(const RoutePlanner *planner, float right_angle_threshold);
uint32_t route_action_timeout_ms(const RoutePlanner *planner);
bool route_is_narrow_line(GrayObservation obs);
bool route_step_is_auto(RouteStep step);
/* STRAIGHT 后的数字距离转弯须走满标称距离；AUTO 不使用此距离门禁。 */
bool route_waiting_after_straight(const RoutePlanner *planner, float distance_cm);
/* 90° 圆弧后紧接显式 STRAIGHT：图形入口前即按出口航向保护，不能借用普通捕线。 */
bool route_prepares_straight(const RoutePlanner *planner);
/* Caller MUST physically issue motor_stop / disabled output before dispatch.
 * Called in the main task, never an ISR. Callback must be short/nonblocking. */
void route_dispatch_stop_callback(RoutePlanner *planner);
#endif
