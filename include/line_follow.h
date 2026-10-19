#ifndef LINE_FOLLOW_H
#define LINE_FOLLOW_H
#include "odometry.h"
#include "route.h"
#include "speed_control.h"

typedef enum { LINE_IDLE, LINE_TRACKING, LINE_GAP_HOLD, LINE_JUNCTION, LINE_STOPPED } LineState;
typedef enum {
    LINE_STOP_NONE, LINE_STOP_REQUEST, LINE_STOP_LOST, LINE_STOP_FAULT,
    LINE_STOP_ENCODER, LINE_STOP_CALIBRATION, LINE_STOP_ROUTE, LINE_STOP_ROUTE_COMPLETE
} LineStopReason;
typedef struct {
    LineState state;
    LineStopReason stop_reason;
    Odometry odometry;
    RoutePlanner route;
    SpeedControl speed;
    Pid pid; /* White-line position: PWM difference when open, cm/s when closed. */
    MotorCommand command; /* Final PWM output in either mode. */
    bool have_reliable, abnormal;
    uint8_t valid_count;
    float reliable_position, base, correction, left_target_cm_s, right_target_cm_s;
    float hold_heading_rad, abnormal_origin_cm;
    float held_pwm_correction; /* Open-loop interference hold, not measured heading. */
    uint32_t last_reliable_ms;
} LineFollow;
void line_follow_init(LineFollow *follow, OdometryCalibration calibration,
                       const RouteStep *steps, size_t count,
                       RouteStopCallback callback, void *context);
void line_follow_stop(LineFollow *follow, LineStopReason reason);
void line_follow_update(LineFollow *follow, GrayObservation observation, EncoderCounts counts,
                        uint32_t now_ms, float dt, bool start_request);
bool line_follow_have_recent_position(const LineFollow *follow, uint32_t now_ms);
#endif
