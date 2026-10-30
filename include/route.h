#ifndef ROUTE_H
#define ROUTE_H
#include "gray_sensor.h"
#include <stddef.h>

typedef enum { LEFT, RIGHT, STRAIGHT, STOP, CUSTOM_TURN } Action;
typedef struct {
    uint16_t junction_count; /* Confirmed physical junctions since last action clearance. */
    float turn_speed;        /* Maximum measured wheel speed before entering a turn, cm/s. */
    Action action;
    int cruise_speed;        /* cm/s, not PWM. */
    float angle_deg;         /* CUSTOM_TURN: clockwise positive, unwrapped. */
    bool corner_arc_enabled;
    float final_segment_length_cm; /* clearance -> target detection; 0 disables planning. */
} RouteStep;
typedef enum {
    ROUTE_FOLLOW, ROUTE_CANDIDATE, ROUTE_EXECUTING, ROUTE_CLEARING,
    ROUTE_FINISHED, ROUTE_FAILED, ROUTE_DISABLED, ROUTE_BRAKING
} RouteState;
/* 保留首个失败原因，区分可重启的路径停车与编码器/任务锁定故障。 */
typedef enum {
    ROUTE_FAILURE_NONE, ROUTE_FAILURE_INVALID, ROUTE_FAILURE_WINDOW,
    ROUTE_FAILURE_CONFIRM_TIMEOUT = 4, /* 旧路段超时值 3 停用，其余诊断编号保持。 */
    ROUTE_FAILURE_ACTION_TIMEOUT, ROUTE_FAILURE_TURN_LIMIT, ROUTE_FAILURE_WRONG_WAY,
    ROUTE_FAILURE_SEARCH = 9, ROUTE_FAILURE_ENTRY_SPEED = 10,
    ROUTE_FAILURE_EXIT_LOST = 11, /* 已捕获出口再次失线，超过独立时间/距离边界。 */
    ROUTE_FAILURE_CLEAR_LINE_LOST = 12, /* 直行过口未恢复可靠出口线，禁止长距离盲行。 */
    ROUTE_FAILURE_EXIT_RECAPTURE = 8 /* 8：捕获/离口超距仍未稳定回线；不自动恢复跳段。 */
} RouteFailure;
typedef void (*RouteStopCallback)(size_t step_index, void *context);
typedef struct {
    uint16_t junctions_seen;
    bool passing_junction, final_segment_active, search_pending;
    float final_segment_origin_cm, search_origin_cm;
    uint32_t search_ms, last_sample_ms;
    float measured_left_cm_s, measured_right_cm_s;
    const RouteStep *steps;
    size_t count, index;
    RouteState state;
    RouteFailure failure;
    float origin_cm, entry_cm, entry_heading, clear_cm, approach_center_cm;
    float action_start_heading;
    uint32_t segment_ms, phase_ms, approach_center_ms;
    uint32_t exit_candidate_ms;
    float exit_candidate_cm;
    bool exit_candidate_pending; /* 接近退出角时的连续中心出口线证据。 */
    Action repeat_action; /* 非最终轮末项的闭合动作，由 motion_config.h 初始化。 */
    uint32_t repeat_count, completed_laps; /* 0 无限；完成末项路口确认后累计，START 清零。 */
    uint8_t candidate_count, exit_count, approach_center_count;
    bool have_approach_center;
    bool valid, stop_pending, callback_dispatched;
    bool corner_arc_complete; /* 90° 轨迹完成后保持捕线，避免纠偏回落时重进圆弧。 */
    bool turn_capturing, lap_boundary; /* 出口线捕获 / 非末轮 STOP 的闭合动作。 */
    bool capture_line_seen, line_exit_confirmed;
    uint8_t capture_line_samples;
    uint32_t capture_line_ms;
    float capture_line_cm;
    float line_exit_heading_rad;
    RouteStopCallback stop_callback;
    void *callback_context;
    bool turn_ended_on_line;
    uint8_t next_junction_samples;
    uint32_t next_junction_ms;
    float next_junction_cm;
    bool candidate_separated; /* 已在转弯离口中连续确认的新分离分支证据。 */
    bool line_confirmed_exit_used;
} RoutePlanner;

/* Speed feedback must be updated before route_update in production. */
void route_set_speed(RoutePlanner *planner, float left_cm_s, float right_cm_s);
extern const RouteStep route[];
extern const size_t route_count;
void route_init(RoutePlanner *planner, const RouteStep *steps, size_t count,
                 RouteStopCallback callback, void *context);
void route_start(RoutePlanner *planner, float distance_cm, uint32_t now_ms);
/* Legacy entry point always returns false: explicit START is required. */
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
bool route_is_clear_line(const RoutePlanner *planner, GrayObservation obs);
/* Caller MUST physically issue motor_stop / disabled output before dispatch.
 * Called in the main task, never an ISR. Callback must be short/nonblocking. */
void route_dispatch_stop_callback(RoutePlanner *planner);
#endif
