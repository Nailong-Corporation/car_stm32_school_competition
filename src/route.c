#include "route.h"
#include "motion_config.h"
#include "braking_planner.h"
#include <math.h>
#include <string.h>

static void fail(RoutePlanner *p, RouteFailure reason)
{
    if (p->failure == ROUTE_FAILURE_NONE) p->failure = reason;
    p->state = ROUTE_FAILED;
}
bool route_is_narrow_line(GrayObservation o)
{
    return o.kind == GRAY_VALID && o.region_count == 1 && o.white_count > 0 &&
           o.white_count <= SENSOR_MAX_REGION_WIDTH;
}
bool route_is_clear_line(const RoutePlanner *p,GrayObservation o)
{
    if(!route_is_narrow_line(o)) return false;
    return route_current_action(p)!=STRAIGHT ||
        (fabsf(o.position)<=JUNCTION_CENTER_LIMIT && o.white_count<=JUNCTION_STRAIGHT_EXIT_MAX_WIDTH);
}
static bool separated_center_branch(GrayObservation o)
{
    /* Two regions with a center-bearing group and a disjoint outer branch.
     * Three isolated/noisy regions (e.g.0x99), lone edges and loss do not qualify. */
    return o.region_count==2 && o.white_count>=3 && (o.line_mask&0x18U)!=0 &&
        (o.line_mask&0xc3U)!=0;
}
static uint8_t branch_sides(GrayObservation o)
{
    unsigned left=o.line_mask&0x07U,right=(o.line_mask>>5U)&0x07U;
    return (uint8_t)((left && (left&(left-1U)) ? 1U:0U) |
                     (right && (right&(right-1U)) ? 2U:0U));
}
static uint8_t separated_sides(GrayObservation o)
{
    if(!separated_center_branch(o)) return 0;
    /* Grow the center-bearing component; a neighboring channel belongs to
     * the main line, not an independent opposite-side branch. */
    uint8_t center=o.line_mask&0x18U;
    for(unsigned i=0;i<7;++i)
        center|=o.line_mask&(uint8_t)((center<<1U)|(center>>1U));
    if(!(center&(uint8_t)(center-1U))) return 0; /* center group needs two channels */
    uint8_t outer=o.line_mask&(uint8_t)~center;
    return (uint8_t)(((outer&0x03U)?1U:0U)|((outer&0xc0U)?2U:0U));
}
static JunctionEvidence junction_shape(GrayObservation o,uint8_t *sides)
{
    *sides=0;
    uint8_t separated=separated_sides(o);
    if(separated) {*sides=separated;return JUNCTION_EVIDENCE_SEPARATED;}
    /* Graphics use the same temporal/motion proof and clearance lock. */
    if(o.region_count>1 && o.white_count>=3) {
        *sides=(uint8_t)(((o.line_mask&0x07U)?1U:0U)|((o.line_mask&0xe0U)?2U:0U));
        if(!*sides) *sides=3U;
        return JUNCTION_EVIDENCE_SEPARATED;
    }
    if(o.region_count!=1) return JUNCTION_EVIDENCE_NONE;
    *sides=branch_sides(o);
    if(*sides && (o.line_mask&0x18U) && o.white_count>=3)
        return JUNCTION_EVIDENCE_CENTER_BRANCH;
    if(*sides && route_is_narrow_line(o) && o.white_count<=JUNCTION_EDGE_MAX_WIDTH)
        return JUNCTION_EVIDENCE_EDGE;
    *sides=0;
    return JUNCTION_EVIDENCE_NONE;
}
static void reset_evidence(RoutePlanner *p, uint32_t now)
{
    p->next_segment_origin_valid=false;
    p->candidate_count=p->exit_count=p->approach_center_count=0;
    p->have_approach_center=p->exit_candidate_pending=false;
    p->turn_capturing=p->corner_arc_complete=false;
    p->line_exit_confirmed=p->turn_ended_on_line=p->capture_line_seen=false;
    p->capture_line_samples=0;
    p->next_junction_samples=0;p->candidate_separated=false;
    p->line_confirmed_exit_used=false;
    p->candidate_kind=JUNCTION_EVIDENCE_NONE;p->candidate_sides=0;
    p->next_junction_sides=0;
    p->candidate_reject=JUNCTION_REJECT_NONE;
    p->previous_line_valid=false;p->smooth_edge_sides=0;
    p->component_recovery_active=p->component_recovery_used=false;
    p->component_recovery_line_samples=0;
    p->last_sample_ms=now;
    p->hexagon=(HexagonNavigation){0};
}
void route_init(RoutePlanner *p, const RouteStep *steps, size_t count,
                RouteStopCallback callback, void *context)
{
    memset(p,0,sizeof(*p));
    p->steps=steps; p->count=count; p->repeat_count=ROUTE_REPEAT_COUNT;
    p->repeat_action=ROUTE_REPEAT_ACTION; p->stop_callback=callback; p->callback_context=context;
    p->valid=steps!=NULL && count>0;
    for(size_t i=0;p->valid && i<count;++i) {
        RouteStep s=steps[i];
        p->valid=(s.junction_count>0 || s.segment_length_cm>0) && (unsigned)s.action<=(unsigned)HEXAGON &&
            s.cruise_speed>0 && s.cruise_speed<=WHEEL_TARGET_MAX_CM_S &&
            isfinite(s.turn_speed) && s.turn_speed>0 && s.turn_speed<=s.cruise_speed &&
            isfinite(s.segment_length_cm) && s.segment_length_cm>=0 &&
            (s.segment_length_cm==0 || (BRAKING_CALIBRATION_CONFIRMED &&
             isfinite(BRAKING_SAFE_DECEL_CM_S2) && BRAKING_SAFE_DECEL_CM_S2>0)) &&
            (s.action!=CUSTOM_TURN || (isfinite(s.angle_deg) && s.angle_deg!=0 && fabsf(s.angle_deg)<=360)) &&
            (s.action!=HEXAGON || (isfinite(s.angle_deg) &&
             fabsf(s.angle_deg)>=60.0f && fabsf(s.angle_deg)<=HEXAGON_MAX_ANGLE_DEG &&
             !s.corner_arc_enabled)) &&
            (s.action!=STOP || i==count-1) &&
            (!s.corner_arc_enabled || (isfinite(CORNER_RADIUS_CM) && CORNER_RADIUS_CM>WHEEL_TRACK_CM*.5f));
    }
    if(p->valid && steps[count-1].action!=STOP) p->valid=false;
    p->state=p->valid?ROUTE_FOLLOW:ROUTE_FAILED;
    p->failure=p->valid?ROUTE_FAILURE_NONE:ROUTE_FAILURE_INVALID;
}
void route_start(RoutePlanner *p,float d,uint32_t now)
{
    p->index=0; p->completed_laps=0; p->junctions_seen=0;
    p->origin_cm=p->final_segment_origin_cm=d;
    p->segment_ms=now; p->passing_junction=p->lap_boundary=false;
    p->search_pending=p->stop_pending=p->callback_dispatched=false;
    p->final_segment_active=p->valid;
    p->navigation_warning=ROUTE_FAILURE_NONE;p->navigation_recoveries=0;
    p->failure=ROUTE_FAILURE_NONE;
    p->last_confirmed_kind=JUNCTION_EVIDENCE_NONE;
    p->candidate_restarts=p->smooth_edge_rejections=0;
    p->state=p->valid && isfinite(d)?ROUTE_FOLLOW:ROUTE_FAILED;
    if(p->state==ROUTE_FAILED) p->failure=ROUTE_FAILURE_INVALID;
    reset_evidence(p,now);
}
void route_set_speed(RoutePlanner *p,float left,float right)
{ p->measured_left_cm_s=left; p->measured_right_cm_s=right; }
/* 最终轮停车，其余轮从入口形状判断开始就使用闭合动作。 */
static bool another_lap(const RoutePlanner *p)
{
    return p->repeat_count == 0U || p->completed_laps < p->repeat_count - 1U;
}
Action route_current_action(const RoutePlanner *p)
{
    if (p->passing_junction) return STRAIGHT;
    Action action = p->steps[p->index].action;
    if (action == CUSTOM_TURN || action == HEXAGON) return p->steps[p->index].angle_deg < 0.0f ? LEFT : RIGHT;
    return action == STOP && (p->lap_boundary || another_lap(p)) ? p->repeat_action : action;
}

/* 普通左右转、闭合左右转及显式 ±90° 共用切角；其他自定义角度保持原策略。 */
bool route_uses_corner_arc(const RoutePlanner *p)
{
    return p->steps[p->index].action!=HEXAGON && p->steps[p->index].corner_arc_enabled &&
        fabsf(fabsf(route_turn_target_rad(p)) - JUNCTION_TURN_TARGET_RAD) < 0.0001f;
}


/* 编码器航向本身右转为正，不对 ±180°/360° 取模，以保留实际转动方向与圈数。 */
float route_turn_target_rad(const RoutePlanner *p)
{
    if (p->steps[p->index].action == CUSTOM_TURN || p->steps[p->index].action == HEXAGON)
        return p->steps[p->index].angle_deg * (JUNCTION_TURN_TARGET_RAD / 90.0f);
    Action action = route_current_action(p);
    return action == LEFT ? -JUNCTION_TURN_TARGET_RAD :
           action == RIGHT ? JUNCTION_TURN_TARGET_RAD : 0.0f;
}

float route_turn_threshold_rad(const RoutePlanner *p, float threshold)
{
    if(p->steps[p->index].action==HEXAGON)
        return fabsf(route_turn_target_rad(p))+HEXAGON_OVERANGLE_RAD;
    if (p->steps[p->index].action != CUSTOM_TURN) return threshold;
    float target = fabsf(route_turn_target_rad(p));
    /* 自定义角度正常退出须到达目标；慢扫/捕获/过转沿用直角参数的角度余量。
     * 小角度按比例压缩，避免门槛为负。 */
    if (threshold == JUNCTION_TURN_MIN_RAD) return target;
    if (threshold > JUNCTION_TURN_TARGET_RAD)
        return target + threshold - JUNCTION_TURN_TARGET_RAD;
    return fmaxf(target + threshold - JUNCTION_TURN_TARGET_RAD,
                 target * threshold / JUNCTION_TURN_TARGET_RAD);
}

uint32_t route_action_timeout_ms(const RoutePlanner *p)
{
    if(p->steps[p->index].action==HEXAGON && !p->passing_junction) return HEXAGON_ACTION_TIMEOUT_MS;
    if (route_current_action(p) == STRAIGHT) return JUNCTION_STRAIGHT_TIMEOUT_MS;
    float scale = fmaxf(1.0f, fabsf(route_turn_target_rad(p)) / JUNCTION_TURN_TARGET_RAD);
    return (uint32_t)ceilf((float)JUNCTION_ACTION_TIMEOUT_MS * scale);
}


/* Old automatic retry could erase junction identity; only explicit START can reset it. */
bool route_resume(RoutePlanner *p,RouteState phase,float d,float h,uint32_t now)
{ (void)p;(void)phase;(void)d;(void)h;(void)now;return false; }
static void begin_action(RoutePlanner *p,float d,float h,uint32_t now)
{
    p->state=ROUTE_EXECUTING; p->phase_ms=now;
    p->entry_cm=d;
    p->entry_heading=p->action_start_heading=h;
    reset_evidence(p,now);
    if(p->steps[p->index].action==HEXAGON && !p->passing_junction) {
        p->hexagon=(HexagonNavigation){.state=HEXAGON_ENTERING,
            .phase_ms=now,.action_ms=now,.line_ms=now,.phase_cm=d,.line_cm=d,
            .hold_heading=h};
    }
}
static float entry_speed_limit_at(const RoutePlanner *p,size_t index,bool active_action)
{
    Action a=p->steps[index].action;
    bool turn=a==LEFT || a==RIGHT || a==CUSTOM_TURN || a==HEXAGON ||
        (a==STOP && ((active_action && p->lap_boundary) || another_lap(p)) && p->repeat_action!=STRAIGHT);
    float limit=p->steps[index].turn_speed;
    if(a==HEXAGON) limit=fminf(limit,HEXAGON_SPEED_CM_S);
    return turn?fminf(limit,JUNCTION_SETTLE_SPEED_CM_S):limit;
}
float route_entry_speed_limit(const RoutePlanner *p)
{ return entry_speed_limit_at(p,p->index,true); }
static bool entry_speed_ready(const RoutePlanner *p)
{
    float limit=route_entry_speed_limit(p);
    return fabsf(p->measured_left_cm_s)<=limit && fabsf(p->measured_right_cm_s)<=limit;
}
static void accept_target(RoutePlanner *p,float d,float h,uint32_t now)
{
    if(p->steps[p->index].action==STOP) {
        bool repeat=another_lap(p);
        if(p->completed_laps<UINT32_MAX) ++p->completed_laps;
        if(!repeat) { p->state=ROUTE_FINISHED;p->stop_pending=true;return; }
        p->lap_boundary=true;
    }
    begin_action(p,d,h,now);
}
static void anchor_next_segment(RoutePlanner *p,float d,uint32_t now)
{
    p->next_segment_origin_cm=d;p->next_segment_ms=now;
    p->next_segment_origin_valid=true;p->search_pending=false;
}
static void enter_clear(RoutePlanner *p,float d,uint32_t now)
{
    p->state=ROUTE_CLEARING;p->phase_ms=now;p->clear_cm=d;
    p->exit_count=0;p->exit_candidate_pending=false;
    p->turn_capturing=false;p->turn_ended_on_line=true;
    if(!p->passing_junction && route_current_action(p)!=STRAIGHT)
        anchor_next_segment(p,d,now);
    if(route_current_action(p)==STRAIGHT) {
        p->capture_line_ms=now;p->capture_line_cm=d;
    }
}
static bool complete_clear(RoutePlanner *p,float d,uint32_t now)
{
    bool passed=p->passing_junction;
    if(passed) p->passing_junction=false;
    else {
        p->index=p->lap_boundary?0:p->index+1;p->lap_boundary=false;
        if(p->index>=p->count) {fail(p,ROUTE_FAILURE_INVALID);return false;}
        p->junctions_seen=0;
    }
    /* Passing an intermediate junction must not reset total segment progress. */
    if(!passed) {
        p->origin_cm=p->next_segment_origin_valid?p->next_segment_origin_cm:p->clear_cm;
        p->segment_ms=p->next_segment_origin_valid?p->next_segment_ms:p->phase_ms;
        p->final_segment_origin_cm=p->origin_cm;
        p->search_pending=p->steps[p->index].segment_length_cm>0 &&
            d-p->origin_cm>=p->steps[p->index].segment_length_cm;
        if(p->search_pending) {p->search_ms=now;p->search_origin_cm=d;}
    }
    p->final_segment_active=true;
    p->state=ROUTE_FOLLOW;reset_evidence(p,now);
    return true;
}
static bool component_recovery(RoutePlanner *p,GrayObservation o,float d,float h,
                               uint32_t now,bool adjacent,bool eligible)
{
    bool graphic=o.white_count>=2 && (o.region_count>1 ||
        o.white_count>JUNCTION_STRAIGHT_EXIT_MAX_WIDTH);
    if(!p->component_recovery_active && !p->component_recovery_used && eligible && graphic && adjacent) {
        p->component_recovery_active=p->component_recovery_used=true;
        p->component_recovery_ms=now;p->component_recovery_cm=d;
        p->component_recovery_heading=route_current_action(p)==STRAIGHT?p->entry_heading:h;
        p->component_recovery_line_samples=0;
        p->exit_count=0;p->exit_candidate_pending=false;
        p->next_junction_samples=p->approach_center_count=0;
    }
    if(!p->component_recovery_active) return false;
    p->turn_capturing=false;
    /* Fixed, one-shot budget. Noise and reason changes cannot renew it; check
     * the limits before accepting a recovered line on the same frame. */
    if((uint32_t)(now-p->component_recovery_ms)>=COMPONENT_RECOVERY_TIMEOUT_MS ||
       fabsf(d-p->component_recovery_cm)>=COMPONENT_RECOVERY_MAX_CM) {
        if(p->steps[p->index].action==HEXAGON) {
            fail(p,ROUTE_FAILURE_EXIT_RECAPTURE);return true;
        }
        /* This one-shot graphic hold has ended. Ordinary maneuvers resume
         * their angle/clearance control under the unchanged phase deadline. */
        p->component_recovery_active=false;
        return false;
    }
    bool line=route_is_narrow_line(o) && o.white_count<=JUNCTION_STRAIGHT_EXIT_MAX_WIDTH &&
        fabsf(o.position)<=JUNCTION_CENTER_LIMIT;
    if(!line || !adjacent) p->component_recovery_line_samples=0;
    if(line && adjacent && p->component_recovery_line_samples<LINE_CONFIRM_SAMPLES)
        ++p->component_recovery_line_samples;
    if(p->component_recovery_line_samples<LINE_CONFIRM_SAMPLES) return true;
    p->component_recovery_active=false;
    p->capture_line_ms=now;p->capture_line_cm=d;
    p->capture_line_samples=LINE_CONFIRM_SAMPLES;
    /* Return to the original exit/clearance gates, without advancing/counting
     * or changing the turn angle baseline. */
    return false;
}
static void hexagon_update(RoutePlanner *p,GrayObservation o,float d,float h,
                           uint32_t now,bool adjacent)
{
    HexagonNavigation *x=&p->hexagon;
    float sign=route_current_action(p)==LEFT?-1.0f:1.0f;
    float turn=sign*(h-p->entry_heading),target=fabsf(route_turn_target_rad(p));
    bool entering=x->state==HEXAGON_ENTERING;
    bool exiting=x->state==HEXAGON_EXITING;
    bool line=route_is_narrow_line(o) && o.white_count<=JUNCTION_STRAIGHT_EXIT_MAX_WIDTH;
    bool center=line && fabsf(o.position)<=JUNCTION_CENTER_LIMIT;
    x->use_line=false;
    /* All deadlines precede evidence acceptance, including graphic recovery.
     * Net yaw (not sum(abs(delta))) prevents oscillations from earning a lap. */
    if(turn < -JUNCTION_WRONG_WAY_RAD || turn>target+HEXAGON_OVERANGLE_RAD) {
        fail(p,turn<0?ROUTE_FAILURE_WRONG_WAY:ROUTE_FAILURE_TURN_LIMIT);return;
    }
    if((uint32_t)(now-x->action_ms)>=HEXAGON_ACTION_TIMEOUT_MS ||
       fabsf(d-p->entry_cm)>=HEXAGON_ACTION_MAX_CM ||
       (entering && ((uint32_t)(now-x->phase_ms)>=HEXAGON_ENTRY_TIMEOUT_MS ||
                     fabsf(d-x->phase_cm)>=HEXAGON_ENTRY_MAX_CM)) ||
       (exiting && ((uint32_t)(now-x->phase_ms)>=HEXAGON_EXIT_TIMEOUT_MS ||
                    fabsf(d-x->phase_cm)>=HEXAGON_EXIT_MAX_CM))) {
        fail(p,ROUTE_FAILURE_ACTION_TIMEOUT);return;
    }
    uint32_t loss_ms=entering?HEXAGON_ENTRY_LOST_MS:INTERFERENCE_TIMEOUT_MS;
    float loss_cm=entering?HEXAGON_ENTRY_LOST_CM:INTERFERENCE_MAX_DISTANCE_CM;
    if(!p->component_recovery_active &&
       ((uint32_t)(now-x->line_ms)>=loss_ms || fabsf(d-x->line_cm)>=loss_cm)) {
        fail(p,ROUTE_FAILURE_EXIT_LOST);return;
    }
    if(!adjacent) {
        x->line_samples=x->branch_samples=x->center_samples=0;
        x->center_pending=false;x->exit_armed=false;p->have_approach_center=false;
        if(x->state==HEXAGON_EXIT_CANDIDATE) x->state=HEXAGON_FOLLOWING;
    }
    /* A fresh opposite-side branch AFTER the configured angle is necessary.
     * The ordinary turn's early line-only completion is deliberately absent. */
    uint8_t sides;
    JunctionEvidence kind=junction_shape(o,&sides);
    uint8_t exit_side=sign>0?1U:2U;
    bool history=p->have_approach_center &&
        (uint32_t)(now-p->approach_center_ms)<JUNCTION_CENTER_MEMORY_MS &&
        d>=p->approach_center_cm && d-p->approach_center_cm<=JUNCTION_CENTER_MEMORY_CM;
    if(turn<target || p->component_recovery_active) x->exit_armed=false;
    bool branch=adjacent && turn>=target && sides==exit_side &&
        kind!=JUNCTION_EVIDENCE_NONE && !p->component_recovery_active;
    if(x->state==HEXAGON_FOLLOWING && x->exit_armed && branch && history) {
        x->exit_armed=false;
        x->state=HEXAGON_EXIT_CANDIDATE;x->branch_ms=now;x->branch_cm=d;
        x->branch_samples=0;p->candidate_kind=kind;x->hold_heading=h;
    }
    if(x->state==HEXAGON_EXIT_CANDIDATE) {
        if((uint32_t)(now-x->branch_ms)>=JUNCTION_CONFIRM_TIMEOUT_MS) {
            fail(p,ROUTE_FAILURE_CONFIRM_TIMEOUT);return;
        }
        if(!branch) {
            x->state=HEXAGON_FOLLOWING;x->branch_samples=0;
        } else {
            if(kind!=JUNCTION_EVIDENCE_CENTER_BRANCH) p->candidate_kind=kind;
            bool weak=p->candidate_kind!=JUNCTION_EVIDENCE_CENTER_BRANCH;
            unsigned samples=weak?JUNCTION_EDGE_CONFIRM_SAMPLES:JUNCTION_CONFIRM_SAMPLES;
            float distance=weak?JUNCTION_EDGE_CONFIRM_CM:JUNCTION_CONFIRM_DISTANCE_CM;
            if(x->branch_samples<samples) ++x->branch_samples;
            if(x->branch_samples>=samples && d-x->branch_cm>=distance) {
                x->state=HEXAGON_EXITING;x->phase_ms=now;x->phase_cm=d;
                x->hold_heading=h;x->center_samples=0;x->center_pending=false;
                x->line_samples=0;
            }
            /* Choose tangent continuation. Freeze position PID on the branch. */
            return;
        }
    }
    bool recovering=p->component_recovery_active;
    if(component_recovery(p,o,d,h,now,adjacent,x->line_seen && !line)) {
        x->line_samples=x->center_samples=0;x->center_pending=false;
        p->have_approach_center=false;
        return;
    }
    if(recovering) {
        /* component_recovery proved three centered frames before returning. */
        x->line_ms=now;x->line_cm=d;x->line_samples=LINE_CONFIRM_SAMPLES;
    }
    if(!line || !adjacent) x->line_samples=0;
    if(line && adjacent && x->line_samples<LINE_CONFIRM_SAMPLES) ++x->line_samples;
    bool stable=x->line_samples>=LINE_CONFIRM_SAMPLES;
    if(stable && (!entering || turn>=HEXAGON_ENTRY_MIN_RAD)) {
        x->line_seen=true;x->line_ms=now;x->line_cm=d;x->hold_heading=h;
        p->capture_line_seen=true;p->capture_line_ms=now;p->capture_line_cm=d;
    }
    if(entering) {
        if(stable && turn>=HEXAGON_ENTRY_MIN_RAD) {
            x->state=HEXAGON_FOLLOWING;x->phase_ms=now;x->phase_cm=d;
        } else return;
    }
    x->use_line=line && stable;
    if(center && stable) {
        p->have_approach_center=true;p->approach_center_ms=now;p->approach_center_cm=d;
        /* Require a post-angle corridor before accepting a new branch.
         * A branch already visible before the angle must not be banked. */
        if(turn>=target && x->state==HEXAGON_FOLLOWING) x->exit_armed=true;
    }
    if(x->state!=HEXAGON_EXITING) return;
    /* Angle must remain satisfied throughout outgoing-line confirmation.
     * No 95% shortcut and no success on deadline. A single center frame,
     * an edge, or lingering branch cannot complete HEXAGON. */
    bool exit=center && stable && adjacent && turn>=target;
    if(!exit) {x->center_samples=0;x->center_pending=false;}
    if(exit && !x->center_pending) {
        x->center_pending=true;x->center_ms=now;x->center_cm=d;
    }
    if(exit && x->center_samples<LINE_CONFIRM_SAMPLES) ++x->center_samples;
    if(x->center_samples>=LINE_CONFIRM_SAMPLES &&
       (uint32_t)(now-x->center_ms)>=HEXAGON_EXIT_CONFIRM_MS &&
       d-x->center_cm>=HEXAGON_EXIT_CONFIRM_CM) {
        x->state=HEXAGON_DONE;
        p->line_exit_confirmed=true;p->line_exit_heading_rad=h;
        enter_clear(p,d,now);
    }
}

size_t route_planning_index(const RoutePlanner *p)
{
    if(p->state==ROUTE_CLEARING && !p->passing_junction && p->next_segment_origin_valid) {
        size_t next=p->lap_boundary?0U:p->index+1U;
        if(next<p->count) return next;
    }
    return p->index;
}
float route_planning_origin(const RoutePlanner *p)
{
    return p->state==ROUTE_CLEARING && !p->passing_junction && p->next_segment_origin_valid?
        p->next_segment_origin_cm:p->final_segment_origin_cm;
}
float route_planning_entry_speed_limit(const RoutePlanner *p)
{
    bool preview=p->state==ROUTE_CLEARING && !p->passing_junction && p->next_segment_origin_valid;
    return entry_speed_limit_at(p,route_planning_index(p),!preview);
}
static void refresh_distance(RoutePlanner *p,float d,uint32_t now)
{
    RouteStep s=p->steps[route_planning_index(p)];
    if(s.segment_length_cm>0 && d-route_planning_origin(p)>=s.segment_length_cm &&
       !p->search_pending) {
        p->search_pending=true;p->search_ms=now;p->search_origin_cm=d;
    }
}
float route_cruise_speed(const RoutePlanner *p)
{
    RouteStep s=p->steps[route_planning_index(p)];
    return s.segment_length_cm>ROUTE_LONG_SEGMENT_CM?(float)s.cruise_speed:ROUTE_SHORT_SPEED_CM_S;
}
bool route_target_ready(const RoutePlanner *p,float d,bool next)
{
    RouteStep s=p->steps[p->index];
    return (s.junction_count>0 && (uint32_t)p->junctions_seen+(next?1U:0U)>=s.junction_count) ||
        (s.segment_length_cm>0 && d-p->origin_cm>=s.segment_length_cm);
}
bool route_recover_navigation(RoutePlanner *p,float d,float h,uint32_t now)
{
    if(p->state!=ROUTE_FAILED) return false;
    switch(p->failure) {
    case ROUTE_FAILURE_CONFIRM_TIMEOUT: case ROUTE_FAILURE_ACTION_TIMEOUT:
    case ROUTE_FAILURE_TURN_LIMIT: case ROUTE_FAILURE_WRONG_WAY:
    case ROUTE_FAILURE_SEARCH: case ROUTE_FAILURE_WINDOW: case ROUTE_FAILURE_ENTRY_SPEED:
        break;
    default: return false; /* Invalid feedback/config and sustained loss remain stops. */
    }
    p->navigation_warning=p->failure;
    if(p->navigation_recoveries<UINT32_MAX) ++p->navigation_recoveries;
    p->failure=ROUTE_FAILURE_NONE;
    reset_evidence(p,now);
    p->state=ROUTE_RECOVERING;p->phase_ms=now;p->clear_cm=d;p->entry_heading=h;
    p->capture_line_ms=now;p->capture_line_cm=d;
    return true;
}
void route_update(RoutePlanner *p,GrayObservation o,float d,float h,uint32_t now)
{
    if(p->state==ROUTE_FINISHED || p->state==ROUTE_FAILED || p->state==ROUTE_DISABLED) return;
    if(!p->valid || p->index>=p->count || !isfinite(d) || !isfinite(h) ||
       !isfinite(p->measured_left_cm_s) || !isfinite(p->measured_right_cm_s)) {
        fail(p,ROUTE_FAILURE_INVALID);return;
    }
    RouteStep s=p->steps[p->index];
    if(p->state==ROUTE_RECOVERING) {
        /* Only sustained loss expires recovery, never time spent on reliable line.
         * Deadline precedes any late recovery sample; isolated frames cannot renew it. */
        if((uint32_t)(now-p->capture_line_ms)>=NAVIGATION_RECOVERY_MS ||
           fabsf(d-p->capture_line_cm)>=NAVIGATION_RECOVERY_CM) {
            fail(p,ROUTE_FAILURE_EXIT_LOST);return;
        }
        bool narrow=route_is_narrow_line(o);
        bool continuous=(uint32_t)(now-p->last_sample_ms)<=CONTROL_MAX_INTERVAL_MS;
        p->last_sample_ms=now;
        if(!narrow || !continuous) {p->exit_count=0;p->exit_candidate_pending=false;}
        if(narrow && !p->exit_candidate_pending) {
            p->exit_candidate_pending=true;p->exit_candidate_ms=now;p->exit_candidate_cm=d;
        }
        if(narrow && p->exit_count<LINE_CONFIRM_SAMPLES) ++p->exit_count;
        if(narrow && p->exit_count>=LINE_CONFIRM_SAMPLES) {
            p->capture_line_ms=now;p->capture_line_cm=d;
        }
        if(p->exit_count>=LINE_CONFIRM_SAMPLES &&
           (uint32_t)(now-p->exit_candidate_ms)>=JUNCTION_COUNT_CLEAR_MS &&
           d-p->exit_candidate_cm>=JUNCTION_COUNT_CLEAR_CM) {
            /* Abort maneuver without claiming success or advancing the table.
             * Keep total progress/count, retry only at a fresh junction. */
            p->passing_junction=false;p->state=ROUTE_FOLLOW;reset_evidence(p,now);return;
        }
        return;
    }
    bool adjacent=(uint32_t)(now-p->last_sample_ms)<=CONTROL_MAX_INTERVAL_MS;
    p->last_sample_ms=now;
    if(p->state==ROUTE_FOLLOW || p->state==ROUTE_CANDIDATE || p->state==ROUTE_CLEARING)
        refresh_distance(p,d,now);
    if(p->state==ROUTE_FOLLOW || p->state==ROUTE_CANDIDATE) {
        bool narrow=route_is_narrow_line(o);
        bool center=narrow && fabsf(o.position)<=JUNCTION_CENTER_LIMIT;
        /* A continuous overlapping narrow line sliding to one edge is curve
         * evidence, not a newly appearing branch. Latch until stable center
         * returns; a noisy missing frame must not reclassify the same edge. */
        if(adjacent && narrow && o.white_count<=JUNCTION_EDGE_MAX_WIDTH &&
           !center && p->previous_line_valid &&
           (p->previous_line_mask&o.line_mask) &&
           fabsf(o.position-p->previous_line_position)<=JUNCTION_SMOOTH_POSITION_STEP &&
           fabsf(o.position)>fabsf(p->previous_line_position))
            p->smooth_edge_sides|=o.position<0?1U:2U;
        p->previous_line_valid=adjacent && narrow && o.white_count<=JUNCTION_EDGE_MAX_WIDTH;
        p->previous_line_mask=o.line_mask;p->previous_line_position=o.position;
        if(center) {
            if(!adjacent) p->approach_center_count=0;
            if(p->approach_center_count<LINE_CONFIRM_SAMPLES) ++p->approach_center_count;
            if(p->approach_center_count>=LINE_CONFIRM_SAMPLES) {
                p->have_approach_center=true;p->approach_center_ms=now;p->approach_center_cm=d;
                p->smooth_edge_sides=0;
            }
        } else p->approach_center_count=0;
        /* 中心历史只负责开启候选；连续分支帧保持该证据至确认/超时。
         * 200 cm/s 每帧前进1 cm，若三帧都重新要求1.5 cm中心记忆，
         * 尚未确认就会失效。形状中断/采样中断仍在下方立即清空候选。 */
        bool recent_center=p->have_approach_center &&
             (uint32_t)(now-p->approach_center_ms)<JUNCTION_CENTER_MEMORY_MS &&
             d>=p->approach_center_cm && d-p->approach_center_cm<=JUNCTION_CENTER_MEMORY_CM;
        bool history=(p->state==ROUTE_CANDIDATE && d>=p->entry_cm) || recent_center;
        uint8_t sides;
        JunctionEvidence kind=junction_shape(o,&sides);
        bool smooth=kind==JUNCTION_EVIDENCE_EDGE && (sides&p->smooth_edge_sides);
        bool shape=history && kind!=JUNCTION_EVIDENCE_NONE && !smooth;
        /* Real loss / single abrupt frames never count as junctions. */
        if(!shape || !adjacent) {
            p->candidate_reject=!adjacent?JUNCTION_REJECT_SAMPLE_GAP:
                smooth?JUNCTION_REJECT_SMOOTH_EDGE:!history?JUNCTION_REJECT_HISTORY:JUNCTION_REJECT_SHAPE;
            if(smooth) ++p->smooth_edge_rejections;
            p->candidate_count=0;p->candidate_separated=false;
            p->candidate_kind=JUNCTION_EVIDENCE_NONE;p->candidate_sides=0;
            p->state=ROUTE_FOLLOW;return;
        }
        p->candidate_reject=JUNCTION_REJECT_NONE;
        if(p->state==ROUTE_CANDIDATE && !(p->candidate_sides&sides)) {
            ++p->candidate_restarts;p->candidate_reject=JUNCTION_REJECT_SIDE_CHANGE;
            p->candidate_count=0;p->candidate_separated=false;p->state=ROUTE_FOLLOW;
            p->candidate_kind=JUNCTION_EVIDENCE_NONE;p->candidate_sides=0;
            /* The old candidate can latch its own center history, but cannot
             * lend expired history to a different branch. */
            if(!recent_center) return;
        }
        bool same_sides=p->state==ROUTE_CANDIDATE && p->candidate_sides==sides;
        if(p->state==ROUTE_FOLLOW) {
            p->state=ROUTE_CANDIDATE;p->phase_ms=now;p->entry_cm=d;p->entry_heading=h;p->candidate_count=0;
            p->candidate_kind=kind;p->candidate_sides=sides;
        } else {
            p->candidate_sides&=sides; /* L -> both -> R cannot accumulate as one branch. */
            if(kind!=JUNCTION_EVIDENCE_CENTER_BRANCH) p->candidate_kind=kind;
        }
        if((uint32_t)(now-p->phase_ms)>=JUNCTION_CONFIRM_TIMEOUT_MS) {
            fail(p,ROUTE_FAILURE_CONFIRM_TIMEOUT);return;
        }
        bool weak=p->candidate_kind!=JUNCTION_EVIDENCE_CENTER_BRANCH && !p->candidate_separated;
        /* Start a clear target-side turn earlier without provisional motion.
         * Both-side bridge frames, weak graphics and non-target junctions
         * keep the longer evidence; counting still follows real confirmation. */
        bool early=same_sides && p->candidate_kind==JUNCTION_EVIDENCE_CENTER_BRANCH &&
            route_target_ready(p,d,true) &&
            ((s.action==LEFT && sides==1U) || (s.action==RIGHT && sides==2U));
        unsigned samples=early?JUNCTION_EARLY_CONFIRM_SAMPLES:
            weak?JUNCTION_EDGE_CONFIRM_SAMPLES:JUNCTION_CONFIRM_SAMPLES;
        float distance=early?JUNCTION_EARLY_CONFIRM_CM:
            weak?JUNCTION_EDGE_CONFIRM_CM:JUNCTION_CONFIRM_DISTANCE_CM;
        if(p->candidate_count<samples) ++p->candidate_count;
        if(p->candidate_count<samples || d-p->entry_cm<distance) return;
        p->last_confirmed_kind=p->candidate_kind;
        if(p->junctions_seen<UINT16_MAX) ++p->junctions_seen; /* Saturate; lock until clearance. */
        p->passing_junction=!route_target_ready(p,d,false);
        p->search_pending=false;
        if(p->passing_junction) { begin_action(p,d,h,now);enter_clear(p,d,now);return; }
        /* 直行过口无强转风险，不设入口轮速门禁；转弯/STOP 仍先降速到位。 */
        if(route_current_action(p)!=STRAIGHT && !entry_speed_ready(p)) {
            p->state=ROUTE_BRAKING;p->phase_ms=now;p->entry_cm=d;p->entry_heading=h;return;
        }
        accept_target(p,d,h,now);return;
    }
    if(p->state==ROUTE_BRAKING) {
        if((uint32_t)(now-p->phase_ms)>=JUNCTION_ENTRY_BRAKE_TIMEOUT_MS ||
           fabsf(d-p->entry_cm)>JUNCTION_ENTRY_BRAKE_MAX_CM ||
           fabsf(h-p->entry_heading)>JUNCTION_WRONG_WAY_RAD) {
            fail(p,ROUTE_FAILURE_ENTRY_SPEED);return;
        }
        if(entry_speed_ready(p)) {
            /* The confirmed node may have passed under the front sensor while
             * coasting. Zero targets cannot find it again once speed is low.
             * Abort into the existing bounded recovery, never turn on loss or
             * treat recovered plain line as completion of this maneuver. */
            if(o.line_mask==0) {fail(p,ROUTE_FAILURE_ENTRY_SPEED);return;}
            accept_target(p,d,h,now);
        }
        return;
    }
    if(s.action==HEXAGON && !p->passing_junction && p->state==ROUTE_EXECUTING) {
        hexagon_update(p,o,d,h,now,adjacent);return;
    }
    Action action=route_current_action(p);
    float sign=action==LEFT?-1.0f:1.0f;
    float turn=(h-p->entry_heading)*sign;
    float target=fabsf(route_turn_target_rad(p));
    if(action!=STRAIGHT && (turn < -JUNCTION_WRONG_WAY_RAD ||
       turn>route_turn_threshold_rad(p,JUNCTION_TURN_MAX_RAD))) {
        fail(p,turn<0?ROUTE_FAILURE_WRONG_WAY:ROUTE_FAILURE_TURN_LIMIT);return;
    }
    if(p->state==ROUTE_EXECUTING) {
        if((uint32_t)(now-p->phase_ms)>=route_action_timeout_ms(p)) {
            fail(p,ROUTE_FAILURE_ACTION_TIMEOUT);return;
        }
        if(action==STRAIGHT) {
            /* A target straight action also needs a real narrow exit before
             * entering clearance; loss/graphics cannot complete the action. */
            bool exit=route_is_clear_line(p,o);
            if(!exit || !adjacent) p->exit_count=0;
            if(exit && p->exit_count<LINE_CONFIRM_SAMPLES) ++p->exit_count;
            if(p->exit_count>=LINE_CONFIRM_SAMPLES) {
                p->line_exit_confirmed=true;p->line_exit_heading_rad=h;
                enter_clear(p,d,now);
                if(!p->passing_junction) anchor_next_segment(p,d,now);
            }
            return;
        }
        bool narrow=route_is_narrow_line(o);
        /* Ordinary turns keep their angle/exit contract through gray loss.
         * Only the action bound aborts to RECOVERING; HEXAGON is separate. */
        if(component_recovery(p,o,d,h,now,adjacent,!narrow && p->capture_line_seen &&
           turn>=target*(1.0f-JUNCTION_EXIT_ANGLE_MARGIN_RATIO))) return;
        p->turn_capturing=narrow && turn>=route_turn_threshold_rad(p,JUNCTION_CAPTURE_MIN_RAD);
        if(p->turn_capturing) {
            if(!adjacent) p->capture_line_samples=0;
            if(p->capture_line_samples<LINE_CONFIRM_SAMPLES) ++p->capture_line_samples;
            if(!p->capture_line_seen || p->capture_line_samples>=LINE_CONFIRM_SAMPLES) {
                p->capture_line_ms=now;p->capture_line_cm=d;
            }
            p->capture_line_seen=true;
        } else p->capture_line_samples=0;
        if(turn>=target) p->corner_arc_complete=true;
        /* 普通转弯找到连续居中窄出口即继续巡线；自定义角保留角度门禁。 */
        bool angle_full=turn>=target;
        bool line_ready=s.action!=CUSTOM_TURN && p->turn_capturing &&
            o.white_count<=JUNCTION_STRAIGHT_EXIT_MAX_WIDTH;
        bool exit=narrow && fabsf(o.position)<=JUNCTION_CENTER_LIMIT &&
            (line_ready || angle_full || turn>=target-JUNCTION_EXIT_ANGLE_MARGIN_RATIO*target);
        if(!exit || !adjacent) {p->exit_count=0;p->exit_candidate_pending=false;}
        if(exit && !p->exit_candidate_pending) {
            p->exit_candidate_pending=true;p->exit_candidate_ms=now;p->exit_candidate_cm=d;
        }
        if(exit && p->exit_count<LINE_CONFIRM_SAMPLES) ++p->exit_count;
        if(p->exit_count>=LINE_CONFIRM_SAMPLES &&
           (line_ready || angle_full || ((uint32_t)(now-p->exit_candidate_ms)>=2U*JUNCTION_EXIT_CONFIRM_MS &&
                           d-p->exit_candidate_cm>=2.0f*JUNCTION_EXIT_CONFIRM_CM))) {
            p->line_confirmed_exit_used=line_ready && !angle_full;
            p->line_exit_confirmed=true;p->line_exit_heading_rad=h;enter_clear(p,d,now);
        }
        return;
    }
    if((uint32_t)(now-p->phase_ms)>=JUNCTION_CLEAR_TIMEOUT_MS || d-p->clear_cm>JUNCTION_CLEAR_MAX_CM) {
        fail(p,ROUTE_FAILURE_ACTION_TIMEOUT);return;
    }
    /* A turn has already confirmed its exit, so lateral recovery is allowed.
     * Straight passage must first prove a centered exit: a diode's side edge
     * is a valid sensor region but is not the intended straight line. */
    bool straight=route_current_action(p)==STRAIGHT;
    /* A new graphic can start before 2 cm of plain line after a turn.
     * Require a confirmed turn exit, a short continuous center corridor, then
     * three new separated-branch frames and real forward displacement.
     * Transfer only once all evidence exists; do not shorten normal dedup. */
    if(!straight && p->line_exit_confirmed && !p->passing_junction) {
        bool corridor=p->exit_candidate_pending && p->approach_center_count>=LINE_CONFIRM_SAMPLES &&
            (uint32_t)(now-p->exit_candidate_ms)>=JUNCTION_ADJACENT_EXIT_MS &&
            d-p->exit_candidate_cm>=JUNCTION_ADJACENT_EXIT_CM;
        uint8_t sides=separated_sides(o);
        bool branch=sides!=0;
        if(p->next_junction_samples>0 && !(p->next_junction_sides&sides)) {
            p->next_junction_samples=0;p->candidate_reject=JUNCTION_REJECT_SIDE_CHANGE;
            ++p->candidate_restarts;
        }
        if(adjacent && branch && (corridor || p->next_junction_samples>0)) {
            if(p->next_junction_samples==0) {
                p->next_junction_ms=now;p->next_junction_cm=d;
                p->next_junction_sides=sides;
            }
            p->next_junction_sides&=sides;
            if((uint32_t)(now-p->next_junction_ms)>=JUNCTION_CONFIRM_TIMEOUT_MS) {
                fail(p,ROUTE_FAILURE_CONFIRM_TIMEOUT);return;
            }
            if(p->next_junction_samples<JUNCTION_CONFIRM_SAMPLES) ++p->next_junction_samples;
            if(p->next_junction_samples>=JUNCTION_CONFIRM_SAMPLES &&
               d-p->next_junction_cm>=JUNCTION_CONFIRM_DISTANCE_CM) {
                uint32_t candidate_ms=p->next_junction_ms;float candidate_cm=p->next_junction_cm;
                uint8_t confirmed_sides=p->next_junction_sides;
                if(!complete_clear(p,d,now)) return;
                p->state=ROUTE_CANDIDATE;p->phase_ms=candidate_ms;p->entry_cm=candidate_cm;
                p->candidate_count=JUNCTION_CONFIRM_SAMPLES;p->candidate_separated=true;
                p->candidate_kind=JUNCTION_EVIDENCE_SEPARATED;p->candidate_sides=confirmed_sides;
                /* Consume the already recorded real samples on the new step;
                 * recursion changes state once and cannot re-enter this branch. */
                route_update(p,o,d,h,now);return;
            }
        } else p->next_junction_samples=0;
        if(adjacent && route_is_narrow_line(o) && fabsf(o.position)<=JUNCTION_CENTER_LIMIT) {
            if(p->approach_center_count<LINE_CONFIRM_SAMPLES) ++p->approach_center_count;
        } else p->approach_center_count=0;
    }
    /* A proven short corridor and new branch have priority: preserve actual
     * adjacent-junction counting instead of swallowing it as interference. */
    if(p->next_junction_samples==0 &&
       component_recovery(p,o,d,h,now,adjacent,straight ||
           (p->line_exit_confirmed && !route_is_narrow_line(o)))) return;
    bool clear=route_is_clear_line(p,o);
    if(!clear || !adjacent) {p->exit_count=0;p->exit_candidate_pending=false;}
    if(clear && !p->exit_candidate_pending) {
        p->exit_candidate_pending=true;p->exit_candidate_ms=now;p->exit_candidate_cm=d;
    }
    if(clear && p->exit_count<LINE_CONFIRM_SAMPLES) ++p->exit_count;
    if(straight && clear && p->exit_count>=LINE_CONFIRM_SAMPLES) {
        p->capture_line_ms=now;p->capture_line_cm=d;
        /* A straight target completes at its first confirmed narrow exit.
         * The following 50 ms/2 cm only unlock counting, not segment distance. */
        if(!p->passing_junction && !p->next_segment_origin_valid) {
            anchor_next_segment(p,d,now);refresh_distance(p,d,now);
        }
    }
    if(p->exit_count<LINE_CONFIRM_SAMPLES ||
       (uint32_t)(now-p->exit_candidate_ms)<JUNCTION_COUNT_CLEAR_MS ||
       d-p->exit_candidate_cm<JUNCTION_COUNT_CLEAR_CM) return;
    bool centered_exit=fabsf(o.position)<=JUNCTION_CENTER_LIMIT &&
        o.white_count<=JUNCTION_STRAIGHT_EXIT_MAX_WIDTH &&
        (straight || p->approach_center_count>=LINE_CONFIRM_SAMPLES);
    if(complete_clear(p,d,now) && centered_exit) {
        /* The confirmed exit already supplied real center history. Do not
         * demand three extra blank-corridor frames before the next graphic. */
        p->have_approach_center=true;p->approach_center_count=LINE_CONFIRM_SAMPLES;
        p->approach_center_ms=now;p->approach_center_cm=d;
    }
}
void route_dispatch_stop_callback(RoutePlanner *p)
{
    if(!p->stop_pending || p->callback_dispatched) return;
    p->stop_pending=false;p->callback_dispatched=true;
    if(p->stop_callback) p->stop_callback(p->index,p->callback_context);
}
