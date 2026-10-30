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
    if(o.region_count>2) return JUNCTION_EVIDENCE_NONE;
    uint8_t separated=separated_sides(o);
    if(separated) {*sides=separated;return JUNCTION_EVIDENCE_SEPARATED;}
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
        p->valid=s.junction_count>0 && (unsigned)s.action<=(unsigned)CUSTOM_TURN &&
            s.cruise_speed>0 && s.cruise_speed<=WHEEL_TARGET_MAX_CM_S &&
            isfinite(s.turn_speed) && s.turn_speed>0 && s.turn_speed<=s.cruise_speed &&
            isfinite(s.final_segment_length_cm) && s.final_segment_length_cm>=0 &&
            (s.final_segment_length_cm==0 || (BRAKING_CALIBRATION_CONFIRMED &&
             isfinite(BRAKING_SAFE_DECEL_CM_S2) && BRAKING_SAFE_DECEL_CM_S2>0)) &&
            (s.action!=CUSTOM_TURN || (isfinite(s.angle_deg) && s.angle_deg!=0 && fabsf(s.angle_deg)<=360)) &&
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
    p->final_segment_active=p->valid && p->steps[0].junction_count==1;
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
    if (action == CUSTOM_TURN) return p->steps[p->index].angle_deg < 0.0f ? LEFT : RIGHT;
    return action == STOP && (p->lap_boundary || another_lap(p)) ? p->repeat_action : action;
}

/* 普通左右转、闭合左右转及显式 ±90° 共用切角；其他自定义角度保持原策略。 */
bool route_uses_corner_arc(const RoutePlanner *p)
{
    return p->steps[p->index].corner_arc_enabled &&
        fabsf(fabsf(route_turn_target_rad(p)) - JUNCTION_TURN_TARGET_RAD) < 0.0001f;
}


/* 编码器航向本身右转为正，不对 ±180°/360° 取模，以保留实际转动方向与圈数。 */
float route_turn_target_rad(const RoutePlanner *p)
{
    if (p->steps[p->index].action == CUSTOM_TURN)
        return p->steps[p->index].angle_deg * (JUNCTION_TURN_TARGET_RAD / 90.0f);
    Action action = route_current_action(p);
    return action == LEFT ? -JUNCTION_TURN_TARGET_RAD :
           action == RIGHT ? JUNCTION_TURN_TARGET_RAD : 0.0f;
}

float route_turn_threshold_rad(const RoutePlanner *p, float threshold)
{
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
}
static bool entry_speed_ready(const RoutePlanner *p)
{
    float limit=p->steps[p->index].turn_speed;
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
static void enter_clear(RoutePlanner *p,float d,uint32_t now)
{
    p->state=ROUTE_CLEARING;p->phase_ms=now;p->clear_cm=d;
    p->exit_count=0;p->exit_candidate_pending=false;
    p->turn_capturing=false;p->turn_ended_on_line=true;
    if(route_current_action(p)==STRAIGHT) {
        p->capture_line_ms=now;p->capture_line_cm=d;
    }
}
static bool complete_clear(RoutePlanner *p,float d,uint32_t now)
{
    if(p->passing_junction) p->passing_junction=false;
    else {
        p->index=p->lap_boundary?0:p->index+1;p->lap_boundary=false;
        if(p->index>=p->count) {fail(p,ROUTE_FAILURE_INVALID);return false;}
        p->junctions_seen=0;
    }
    p->origin_cm=d;p->segment_ms=now;
    p->final_segment_active=p->junctions_seen+1U==p->steps[p->index].junction_count;
    p->final_segment_origin_cm=d;p->search_pending=false;
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
        fail(p,ROUTE_FAILURE_EXIT_RECAPTURE);return true;
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
void route_update(RoutePlanner *p,GrayObservation o,float d,float h,uint32_t now)
{
    if(p->state==ROUTE_FINISHED || p->state==ROUTE_FAILED || p->state==ROUTE_DISABLED) return;
    if(!p->valid || p->index>=p->count || !isfinite(d) || !isfinite(h) ||
       !isfinite(p->measured_left_cm_s) || !isfinite(p->measured_right_cm_s)) {
        fail(p,ROUTE_FAILURE_INVALID);return;
    }
    RouteStep s=p->steps[p->index];
    bool adjacent=(uint32_t)(now-p->last_sample_ms)<=CONTROL_MAX_INTERVAL_MS;
    p->last_sample_ms=now;
    if(p->state==ROUTE_FOLLOW || p->state==ROUTE_CANDIDATE) {
        if(p->final_segment_active && s.final_segment_length_cm>0 &&
           d-p->final_segment_origin_cm>=s.final_segment_length_cm) {
            if(!p->search_pending) {p->search_pending=true;p->search_ms=now;p->search_origin_cm=d;}
            if((uint32_t)(now-p->search_ms)>=JUNCTION_SEARCH_TIMEOUT_MS ||
               d-p->search_origin_cm>=JUNCTION_SEARCH_MAX_CM) {fail(p,ROUTE_FAILURE_SEARCH);return;}
        }
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
        if(p->state==ROUTE_FOLLOW) {
            p->state=ROUTE_CANDIDATE;p->phase_ms=now;p->entry_cm=d;p->candidate_count=0;
            p->candidate_kind=kind;p->candidate_sides=sides;
        } else {
            p->candidate_sides&=sides; /* L -> both -> R cannot accumulate as one branch. */
            if(kind!=JUNCTION_EVIDENCE_CENTER_BRANCH) p->candidate_kind=kind;
        }
        if((uint32_t)(now-p->phase_ms)>=JUNCTION_CONFIRM_TIMEOUT_MS) {
            fail(p,ROUTE_FAILURE_CONFIRM_TIMEOUT);return;
        }
        bool weak=p->candidate_kind!=JUNCTION_EVIDENCE_CENTER_BRANCH && !p->candidate_separated;
        unsigned samples=weak?JUNCTION_EDGE_CONFIRM_SAMPLES:JUNCTION_CONFIRM_SAMPLES;
        float distance=weak?JUNCTION_EDGE_CONFIRM_CM:JUNCTION_CONFIRM_DISTANCE_CM;
        if(p->candidate_count<samples) ++p->candidate_count;
        if(p->candidate_count<samples || d-p->entry_cm<distance) return;
        p->last_confirmed_kind=p->candidate_kind;
        ++p->junctions_seen; /* Lock immediately: no recount until clearance. */
        p->passing_junction=p->junctions_seen<s.junction_count;
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
        if(entry_speed_ready(p) && o.line_mask!=0) accept_target(p,d,h,now);
        return;
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
        if(action==STRAIGHT) {enter_clear(p,d,now);return;}
        bool narrow=route_is_narrow_line(o);
        /* Once a real exit has been captured, do not drive blind until the
         * full maneuver timeout. Check the deadline before accepting recovery. */
        if(!p->component_recovery_active && p->capture_line_seen &&
           ((uint32_t)(now-p->capture_line_ms)>=INTERFERENCE_TIMEOUT_MS ||
            fabsf(d-p->capture_line_cm)>=INTERFERENCE_MAX_DISTANCE_CM)) {
            fail(p,ROUTE_FAILURE_EXIT_LOST);return;
        }
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
    if(straight && !p->component_recovery_active &&
       ((uint32_t)(now-p->capture_line_ms)>=INTERFERENCE_TIMEOUT_MS ||
        fabsf(d-p->capture_line_cm)>=INTERFERENCE_MAX_DISTANCE_CM)) {
        fail(p,ROUTE_FAILURE_CLEAR_LINE_LOST);return;
    }
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
