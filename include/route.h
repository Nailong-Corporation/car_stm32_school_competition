#ifndef ROUTE_H
#define ROUTE_H
#include "gray_sensor.h"
#include <stddef.h>

typedef enum { LEFT, RIGHT, STRAIGHT, STOP } Action;
typedef struct {
    float distance;   /* cm from start / previous action completion */
    float tolerance;  /* inclusive +/- cm intersection window */
    Action action;
    int speed;        /* target forward speed, cm/s, NOT PWM */
} RouteStep;
typedef enum {
    ROUTE_FOLLOW, ROUTE_CANDIDATE, ROUTE_EXECUTING, ROUTE_CLEARING,
    ROUTE_FINISHED, ROUTE_FAILED, ROUTE_DISABLED
} RouteState;
/* 保留首个失败原因，区分可重启的路径停车与编码器/任务锁定故障。 */
typedef enum {
    ROUTE_FAILURE_NONE, ROUTE_FAILURE_INVALID, ROUTE_FAILURE_WINDOW,
    ROUTE_FAILURE_CONFIRM_TIMEOUT = 4, /* 旧路段超时值 3 停用，其余诊断编号保持。 */
    ROUTE_FAILURE_ACTION_TIMEOUT, ROUTE_FAILURE_TURN_LIMIT, ROUTE_FAILURE_WRONG_WAY
} RouteFailure;
typedef void (*RouteStopCallback)(size_t step_index, void *context);
typedef struct {
    const RouteStep *steps;
    size_t count, index;
    RouteState state;
    RouteFailure failure;
    float origin_cm, entry_cm, entry_heading, clear_cm, approach_center_cm;
    float outgoing_cm, projection_previous_cm, projection_previous_heading; /* 路口动作参考点以来的出口方向位移。 */
    uint32_t segment_ms, phase_ms, approach_center_ms;
    uint32_t exit_candidate_ms;
    float exit_candidate_cm;
    bool exit_candidate_pending; /* 接近退出角时的连续中心出口线证据。 */
    uint32_t repeat_count, completed_laps; /* 0 无限；完成末项路口确认后累计，START 清零。 */
    uint8_t candidate_count, exit_count, approach_center_count;
    uint8_t previous_line_mask, narrow_count;
    uint32_t previous_line_ms;
    float previous_line_position;
    bool have_approach_center;
    bool valid, stop_pending, callback_dispatched;
    bool turn_capturing, lap_boundary; /* 出口线捕获 / 非末轮 STOP 的直行离口。 */
    bool turn_forced; /* 本项由整排骤变触发；保持至下一路段，便于诊断。 */
    RouteStopCallback stop_callback;
    void *callback_context;
} RoutePlanner;

extern const RouteStep route[];
extern const size_t route_count;
void route_init(RoutePlanner *planner, const RouteStep *steps, size_t count,
                 RouteStopCallback callback, void *context);
void route_start(RoutePlanner *planner, float distance_cm, uint32_t now_ms);
void route_update(RoutePlanner *planner, GrayObservation obs, float distance_cm,
                   float heading_rad, uint32_t now_ms);
/* 非最后一轮的末项 STOP 在执行阶段映射为 STRAIGHT。 */
Action route_current_action(const RoutePlanner *planner);
bool route_is_narrow_line(GrayObservation obs);
/* Caller MUST physically issue motor_stop / disabled output before dispatch.
 * Called in the main task, never an ISR. Callback must be short/nonblocking. */
void route_dispatch_stop_callback(RoutePlanner *planner);
#endif
