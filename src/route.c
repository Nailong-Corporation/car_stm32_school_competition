#include "route.h"
#include "motion_config.h"
#include "braking_planner.h"
#include <math.h>
#include <string.h>

/* Explicit short links only: geometry in the table is never inferred from a
 * normal segment's trigger distance. Bounds include the preceding pivot. */
#define SHORT_LINK_MIN_CM 20.0f
#define SHORT_LINK_MAX_CM 40.0f
#define SHORT_LINK_MAX_LATERAL_CM 14.0f /* faster pivots project more lateral drift */
#define SHORT_LINK_MAX_HEADING_RAD 0.30f
#define SHORT_LINK_TIMEOUT_MS 6000U
static bool ordinary_right_angle(RouteStep s)
{
    return !s.corner_arc_enabled && (s.action==LEFT || s.action==RIGHT ||
        (s.action==CUSTOM_TURN && fabsf(fabsf(s.angle_deg)-90.0f)<0.001f));
}
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
    /* 印刷电阻也是计数节点；与真实支路共用连续帧、位移和离口去重。
     * 图形穿越保持航向的分类仍由巡线层独立使用，不否决这里的节点。 */
    bool hollow=o.region_count==2 && o.white_count>=2 &&
        (o.line_mask&0x18U)==0 && (o.line_mask&0x07U) && (o.line_mask&0xe0U);
    if((o.region_count==2 && o.white_count>=3) || hollow) {
        *sides=(uint8_t)(((o.line_mask&0x07U)?1U:0U)|((o.line_mask&0xe0U)?2U:0U));
        if(!*sides) *sides=3U;
        return JUNCTION_EVIDENCE_SEPARATED;
    }
    uint8_t separated=separated_sides(o);
    if(separated) {*sides=separated;return JUNCTION_EVIDENCE_SEPARATED;}
    /* 未满足图形证据的多区域不计数；单区域宽横边和侧分支均可计数。 */
    if(o.region_count!=1) return JUNCTION_EVIDENCE_NONE;
    *sides=branch_sides(o);
    /* 空心电阻横边通常只覆盖居中四路，不一定触及最外侧两路。 */
    if(o.white_count>=4 && (o.line_mask&0x18U)==0x18U && !*sides) *sides=3U;
    if(*sides && (o.line_mask&0x18U) && o.white_count>=3)
        return JUNCTION_EVIDENCE_CENTER_BRANCH;
    /* 2026-11-06: a narrow line with only an outer group (no central pair) is
     * a printed symbol's V arm / resistor cap, not a branch; the old weak EDGE
     * node is retired so symbols cannot authorize or miscount junctions. */
    *sides=0;
    return JUNCTION_EVIDENCE_NONE;
}
static void reset_evidence(RoutePlanner *p, uint32_t now)
{
    p->next_segment_origin_valid=false;
    p->candidate_count=p->exit_count=p->approach_center_count=0;
    p->have_approach_center=p->exit_candidate_pending=false;
    p->turn_capturing=p->corner_arc_complete=false;
    p->turn_rewind_active=p->turn_rewind_pending=false;
    p->line_exit_confirmed=p->turn_ended_on_line=p->capture_line_seen=false;
    p->capture_line_samples=0;
    p->next_junction_samples=0;p->candidate_separated=false;
    p->turn_branch_samples=0;p->turn_branch_sides=0;
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
    p->start_x_cm=ROUTE_START_X_CM;p->start_y_cm=ROUTE_START_Y_CM;
    p->start_heading_rad=ROUTE_START_HEADING_RAD;
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
            (!s.corner_arc_enabled || (isfinite(CORNER_RADIUS_CM) && CORNER_RADIUS_CM>WHEEL_TRACK_CM*.5f)) &&
            isfinite(s.short_link_cm) && (s.short_link_cm==0 ||
             (i>0 && s.short_link_cm>=SHORT_LINK_MIN_CM && s.short_link_cm<=SHORT_LINK_MAX_CM &&
              ordinary_right_angle(s) && ordinary_right_angle(steps[i-1])));
    }
    if(p->valid && steps[count-1].action!=STOP) p->valid=false;
    p->state=p->valid?ROUTE_FOLLOW:ROUTE_FAILED;
    p->failure=p->valid?ROUTE_FAILURE_NONE:ROUTE_FAILURE_INVALID;
}
void route_start(RoutePlanner *p,float d,uint32_t now)
{
    p->short_link_tracking=p->short_link_active=p->turn_approaching=false;
    p->post_turn_slow=false;p->post_turn_samples=0;
    /* Absolute corner waypoints: anchored start pose plus the cumulative table
     * chain (the table already subtracts the 5 cm margin, added back here). */
    {
        float wx=p->start_x_cm, wy=p->start_y_cm, wh=p->start_heading_rad;
        for(size_t i=0;i<p->count && i<ROUTE_MAX_STEPS;++i) {
            float len=p->steps[i].segment_length_cm>0.0f
                ? p->steps[i].segment_length_cm+ROUTE_WAYPOINT_MARGIN_CM : 0.0f;
            wx+=len*cosf(wh); wy+=len*sinf(wh);
            p->waypoint_x_cm[i]=wx; p->waypoint_y_cm[i]=wy;
            wh+=(p->steps[i].angle_deg*(float)M_PI)/180.0f;
        }
    }
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
    if(!p->short_link_active) p->entry_heading=h;
    p->action_start_heading=p->entry_heading;
    p->turn_approach_origin_cm=d;
    p->turn_approach_cm=fmaxf(0.0f,
        TURN_PIVOT_APPROACH_CM-(p->pose_mode?p->pose_trigger_late_cm:0.0f));
    p->turn_approaching=!p->passing_junction &&
        ((false && p->steps[p->index].action==HEXAGON) ||
         (!route_uses_corner_arc(p) &&
          fabsf(fabsf(route_turn_target_rad(p))-JUNCTION_TURN_TARGET_RAD)<0.0001f &&
          p->steps[p->index].action!=HEXAGON));
    reset_evidence(p,now);
    if(p->steps[p->index].action==HEXAGON && !p->passing_junction) {
        p->hexagon=(HexagonNavigation){.state=HEXAGON_ENTERING,
            .phase_ms=now,.action_ms=now,.line_ms=now,.phase_cm=d,.line_cm=d,.branch_cm=d,
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
static void start_short_link_tracking(RoutePlanner *p,float d,float h,uint32_t now)
{
    p->short_link_active=false;
    p->short_link_tracking=!p->passing_junction && p->index+1<p->count &&
        p->steps[p->index+1].short_link_cm>0 && ordinary_right_angle(p->steps[p->index]);
    if(!p->short_link_tracking) return;
    p->short_link_heading=p->entry_heading+route_turn_target_rad(p);
    p->short_link_forward_cm=p->short_link_lateral_cm=0;
    p->short_link_last_cm=d;p->short_link_last_heading=h;p->short_link_ms=now;
}
static void enter_clear(RoutePlanner *p,float d,uint32_t now)
{
    if(p->short_link_tracking && !p->passing_junction && p->line_exit_confirmed &&
       p->index+1<p->count && p->steps[p->index+1].short_link_cm>0) {
        /* Only a real confirmed preceding exit can grant this explicit geometry
         * transfer. No node is synthesized and STOP is excluded by validation. */
        if(fabsf(p->line_exit_heading_rad-p->short_link_heading)>SHORT_LINK_MAX_HEADING_RAD) {
            p->short_link_tracking=false;
        } else {
            /* The preceding pivot's lateral motion measures its effective
             * turn footprint, including coast. Reuse it only on this explicit
             * link instead of pretending every moving turn has radius track/2. */
            p->short_link_pivot_lead_cm=fmaxf(WHEEL_TRACK_CM*.5f,fabsf(p->short_link_lateral_cm));
            if(p->short_link_pivot_lead_cm>SHORT_LINK_MAX_LATERAL_CM ||
               p->steps[p->index+1].short_link_cm-p->short_link_pivot_lead_cm-p->short_link_forward_cm<1.0f) {
                p->short_link_tracking=false;fail(p,ROUTE_FAILURE_EXIT_RECAPTURE);return;
            }
            ++p->index;p->junctions_seen=0;
            p->origin_cm=p->final_segment_origin_cm=d;p->segment_ms=now;
            begin_action(p,d,p->short_link_heading,now);
            p->short_link_active=true;
            float remaining=p->steps[p->index].short_link_cm-p->short_link_pivot_lead_cm-p->short_link_forward_cm;
            p->turn_approach_origin_cm=d-(TURN_PIVOT_APPROACH_CM-remaining);
            return;
        }
    }
    p->state=ROUTE_CLEARING;p->post_turn_slow=p->pose_mode;p->phase_ms=now;p->clear_cm=d;
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
/* “几何 − 5 cm”的表内距离是车轴里程。传感器前伸 24 cm，短段上出口确认时
 * 前探头可能已经越过下一个节点；因此相邻交接额外允许“探头位置达标”：
 * 探头里程 = 车轴里程 + GRAY_FORWARD_OFFSET_CM。它只用于相邻节点的交接
 * 资格，不改变主计数的里程基准和速度包络。 */
static bool next_segment_sensor_ready(const RoutePlanner *p,float d)
{
    if(p->passing_junction || !p->next_segment_origin_valid) return false;
    size_t next=p->index+1U;
    if(next>=p->count) return false;
    float length=p->steps[next].segment_length_cm;
    if(!(length>0.0f)) return false;
    return d+GRAY_FORWARD_OFFSET_CM-p->next_segment_origin_cm>=length;
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
static float ring_vertex_x(int dir,int i,float sign)
{
    (void)sign;
    int k=dir>0 ? (6-(i%6))%6 : i%6;
    return HEXAGON_CENTER_X_CM+HEXAGON_SIDE_CM*cosf(-(float)k*(float)M_PI/3.0f);
}
static float ring_vertex_y(int dir,int i,float sign)
{
    (void)sign;
    int k=dir>0 ? (6-(i%6))%6 : i%6;
    return HEXAGON_CENTER_Y_CM+HEXAGON_SIDE_CM*sinf(-(float)k*(float)M_PI/3.0f);
}
/* Pose-driven regular hexagon traversal (2026-11-07).
 *
 * The ring has one external connection (right vertex V0), so a loop entered
 * from one side has to leave on the other side.  The table's HEXAGON angle
 * sign selects the direction (+ = clockwise) and the traversal uses the known
 * 35 cm edges / 60 deg vertices with odometry + IMU: waiting for the 24 cm
 * look-ahead bar to find the next vertex cannot work on 35 cm edges, and the
 * old tangent/branch exit assumption does not hold for this lollipop.
 * The 8-channel line still centres the car along each edge. */
static void hexagon_update(RoutePlanner *p,GrayObservation o,float d,float h,
                           uint32_t now,bool adjacent)
{
    HexagonNavigation *x=&p->hexagon;
    int dir=route_turn_target_rad(p)>=0.0f?1:-1;
    float h0=p->entry_heading;
    const float turn_angles[7]={HEXAGON_ENTRY_TURN_RAD,HEXAGON_TURN_RAD,HEXAGON_TURN_RAD,
        HEXAGON_TURN_RAD,HEXAGON_TURN_RAD,HEXAGON_TURN_RAD,HEXAGON_ENTRY_TURN_RAD};
    float cumulative[7];
    for(int i=0;i<7;++i) cumulative[i]=(i>0?cumulative[i-1]:0.0f)+turn_angles[i];
    float axle_x=p->pose_axle_x_cm,axle_y=p->pose_axle_y_cm;
    if((uint32_t)(now-x->action_ms)>=HEXAGON_ACTION_TIMEOUT_MS ||
       fabsf(d-p->entry_cm)>=HEXAGON_ACTION_MAX_CM) {
        fail(p,ROUTE_FAILURE_ACTION_TIMEOUT);return;
    }
    /* Direction / over-rotation guards (the polygon FSM bypasses the generic
     * EXECUTING checks, so it must not lose them). */
    {
        float target_rad=fabsf(route_turn_target_rad(p));
        float net=(h-h0)*(float)dir;
        if(net<-JUNCTION_WRONG_WAY_RAD) {fail(p,ROUTE_FAILURE_WRONG_WAY);return;}
        if(net>target_rad+HEXAGON_OVERANGLE_RAD) {
            fail(p,ROUTE_FAILURE_TURN_LIMIT);return;
        }
    }
    bool narrow=route_is_narrow_line(o);
    bool centered=narrow && fabsf(o.position)<=JUNCTION_CENTER_LIMIT;
    /* Only a completely blank bar is true loss inside the ring; the printed
     * R14 symbol and the vertex joins keep some channel lit. */
    if(o.line_mask!=0 && adjacent) {x->line_ms=now;x->line_cm=d;}
    if(x->state==HEXAGON_ENTERING) {
        /* The action starts when the look-ahead bar reaches V0; drive straight
         * until the axle is one turn-lead short of the vertex. */
        float vx=ring_vertex_x(dir,0,0.0f),vy=ring_vertex_y(dir,0,0.0f);
        float raw_gap=hypotf(axle_x-vx,axle_y-vy);
        float entry_lead=WHEEL_TRACK_CM*0.5f*tanf(0.5f*turn_angles[0]);
        if(raw_gap>entry_lead+0.6f) {
            x->turning=false;
            x->use_line=narrow && centered;
            x->target_heading=h0;
            if((uint32_t)(now-x->action_ms)>=HEXAGON_ACTION_TIMEOUT_MS ||
               fabsf(d-p->entry_cm)>=HEXAGON_ENTRY_APPROACH_MAX_CM)
                fail(p,ROUTE_FAILURE_ENTRY_SPEED);
            return;
        }
        x->turning=true;x->turn_index=0;
        x->target_heading=h0+(float)dir*turn_angles[0];
        x->vertex_x_cm=ring_vertex_x(dir,1,0.0f);x->vertex_y_cm=ring_vertex_y(dir,1,0.0f);
        x->leg_from_x_cm=vx;x->leg_from_y_cm=vy;
        x->state=HEXAGON_FOLLOWING;
        return;
    }
    if(x->state==HEXAGON_FOLLOWING) {
        if(x->turning) {
            x->use_line=false;
            float err=x->target_heading-h;
            if(fabsf(atan2f(sinf(err),cosf(err)))<=HEXAGON_TURN_TOL_RAD) {
                x->turning=false;x->leg_start_cm=d;
                x->line_seen=true;x->line_ms=now;x->line_cm=d;
                if((int)x->turn_index>=6) {
                    /* Pose-mode exit: the axle is at V0 and already turned onto
                     * the external line, hand over directly. */
                    x->state=HEXAGON_DONE;x->use_line=false;
                    p->line_exit_confirmed=true;p->line_exit_heading_rad=h;
                    enter_clear(p,d,now);return;
                }
                x->leg_from_x_cm=ring_vertex_x(dir,(int)x->turn_index,0.0f);
                x->leg_from_y_cm=ring_vertex_y(dir,(int)x->turn_index,0.0f);
                x->vertex_x_cm=ring_vertex_x(dir,(int)x->turn_index+1,0.0f);
                x->vertex_y_cm=ring_vertex_y(dir,(int)x->turn_index+1,0.0f);
            }
            return;
        }
        {
            float next_angle=fabsf(turn_angles[(int)x->turn_index+1]);
            float lead=WHEEL_TRACK_CM*0.5f*tanf(0.5f*next_angle);
            float gap=hypotf(x->vertex_x_cm-axle_x,x->vertex_y_cm-axle_y);
            if(gap<=lead+(p->pose_valid?0.6f:0.0f)) {
                ++x->turn_index;
                x->turning=true;
                x->target_heading=h0+(float)dir*cumulative[(int)x->turn_index];
                x->use_line=false;
                return;
            }
            x->use_line=centered || (narrow && fabsf(o.position)<=JUNCTION_CENTER_LIMIT+1.0f);
            /* Blind stretch: bound the leg by the pose cross-track instead of
             * the 400 ms grey-loss timer. */
            float ldx=x->vertex_x_cm-x->leg_from_x_cm;
            float ldy=x->vertex_y_cm-x->leg_from_y_cm;
            float llen=hypotf(ldx,ldy);
            if(llen>1e-3f)
                x->cross_cm=(-(axle_x-x->leg_from_x_cm)*ldy+
                             (axle_y-x->leg_from_y_cm)*ldx)/llen;
            else x->cross_cm=0.0f;
            if(p->pose_valid && fabsf(x->cross_cm)>HEXAGON_LEG_CROSS_MAX_CM) {
                fail(p,ROUTE_FAILURE_EXIT_LOST);return;
            }
            return;
        }
    }
    if(x->state==HEXAGON_EXITING) {
        x->use_line=centered;
        if((uint32_t)(now-x->phase_ms)>=HEXAGON_EXIT_TIMEOUT_MS ||
           fabsf(d-x->phase_cm)>=HEXAGON_EXIT_MAX_CM) {
            fail(p,ROUTE_FAILURE_EXIT_LOST);return;
        }
        if(!centered) {x->center_samples=0;x->center_pending=false;}
        else {
            if(!x->center_pending) {x->center_pending=true;x->center_ms=now;x->center_cm=d;}
            if(x->center_samples<LINE_CONFIRM_SAMPLES) ++x->center_samples;
        }
        if(x->center_samples>=LINE_CONFIRM_SAMPLES &&
           (uint32_t)(now-x->center_ms)>=HEXAGON_EXIT_CONFIRM_MS &&
           d-x->center_cm>=HEXAGON_EXIT_CONFIRM_CM) {
            x->state=HEXAGON_DONE;
            p->line_exit_confirmed=true;p->line_exit_heading_rad=h;
            enter_clear(p,d,now);
        }
        return;
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
    /* 最多路口数与总段距离任一满足后，仍须由调用方确认真实灰度节点。
     * 距离本身不触发动作；计数包含当前正在确认的节点。 */
    bool count_ready=s.junction_count>0 &&
        (uint32_t)p->junctions_seen+(next?1U:0U)>=s.junction_count;
    bool distance_ready=s.segment_length_cm>0 && d-p->origin_cm>=s.segment_length_cm;
    return count_ready || distance_ready;
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
    p->short_link_tracking=p->short_link_active=p->turn_approaching=false;
    p->navigation_warning=p->failure;
    if(p->navigation_recoveries<UINT32_MAX) ++p->navigation_recoveries;
    p->failure=ROUTE_FAILURE_NONE;
    reset_evidence(p,now);
    p->state=ROUTE_RECOVERING;p->phase_ms=now;p->clear_cm=d;p->entry_heading=h;
    p->capture_line_ms=now;p->capture_line_cm=d;
    return true;
}
void route_update_pose(RoutePlanner *p,GrayObservation o,float d,float h,uint32_t now,
                       float axle_x_cm,float axle_y_cm)
{
    if(p->state==ROUTE_FINISHED || p->state==ROUTE_FAILED || p->state==ROUTE_DISABLED) return;
    if(!p->valid || p->index>=p->count || !isfinite(d) || !isfinite(h) ||
       !isfinite(p->measured_left_cm_s) || !isfinite(p->measured_right_cm_s)) {
        fail(p,ROUTE_FAILURE_INVALID);return;
    }
    /* Pose (encoder/IMU) navigation state: the caller supplies the axle pose in
     * track cm. NaN (legacy wrapper) or POSE_NAV_ENABLED=0 keeps the grey-only
     * contract; see include/motion_config.h. */
    p->pose_valid = POSE_NAV_ENABLED && isfinite(axle_x_cm) && isfinite(axle_y_cm) &&
                    p->count>0 && p->count<=ROUTE_MAX_STEPS;
    p->pose_mode = p->pose_valid;
    p->pose_axle_x_cm=axle_x_cm;p->pose_axle_y_cm=axle_y_cm;
    if(p->pose_valid && p->index<p->count) {
        float wh=h-(float)M_PI*.5f;
        float sx=axle_x_cm+GRAY_FORWARD_OFFSET_CM*cosf(wh);
        float sy=axle_y_cm+GRAY_FORWARD_OFFSET_CM*sinf(wh);
        p->pose_gap_bar_cm=hypotf(sx-p->waypoint_x_cm[p->index],sy-p->waypoint_y_cm[p->index]);
        p->pose_bearing_rad=atan2f(p->waypoint_y_cm[p->index]-axle_y_cm,
            p->waypoint_x_cm[p->index]-axle_x_cm)+(float)M_PI*.5f;
        float px=p->index>0?p->waypoint_x_cm[p->index-1]:p->start_x_cm;
        float py=p->index>0?p->waypoint_y_cm[p->index-1]:p->start_y_cm;
        float dx=p->waypoint_x_cm[p->index]-px,dy=p->waypoint_y_cm[p->index]-py;
        float len=hypotf(dx,dy);
        if(len>1e-3f) {
            p->pose_cross_cm=(-(axle_x_cm-px)*dy+(axle_y_cm-py)*dx)/len;
            p->pose_cross_bar_cm=p->pose_cross_cm+
                GRAY_FORWARD_OFFSET_CM*sinf(wh-atan2f(dy,dx));
            p->pose_seg_heading_rad=atan2f(dy,dx)+(float)M_PI*.5f;
            p->pose_cross_valid=true;
        } else {
            /* Degenerate (zero-length) row: there is no segment line, so hold
             * the row's entry heading instead of bearing back to the corner. */
            p->pose_cross_valid=false;p->pose_cross_cm=0.0f;p->pose_cross_bar_cm=0.0f;
            p->pose_bearing_rad=p->entry_heading;p->pose_seg_heading_rad=p->entry_heading;
        }
    } else {
        p->pose_gap_bar_cm=INFINITY;p->pose_cross_valid=false;
        p->pose_cross_cm=0.0f;p->pose_cross_bar_cm=0.0f;
    }
    RouteStep s=p->steps[p->index];
    if(p->short_link_tracking) {
        float ds=d-p->short_link_last_cm,dh=h-p->short_link_last_heading;
        float half=.5f*dh;
        float chord=fabsf(half)<.0001f?1.0f:sinf(half)/half;
        float angle=.5f*(h+p->short_link_last_heading)-p->short_link_heading;
        p->short_link_forward_cm+=ds*chord*cosf(angle);
        p->short_link_lateral_cm+=ds*chord*sinf(angle);
        p->short_link_last_cm=d;p->short_link_last_heading=h;
        if(p->short_link_active) {
            if((uint32_t)(now-p->short_link_ms)>=SHORT_LINK_TIMEOUT_MS || !((uint32_t)(now-p->last_sample_ms)<=CONTROL_MAX_INTERVAL_MS) ||
               fabsf(p->short_link_lateral_cm)>SHORT_LINK_MAX_LATERAL_CM ||
               fabsf(h-p->short_link_heading)>SHORT_LINK_MAX_HEADING_RAD ||
               ds<-.01f || p->short_link_forward_cm>s.short_link_cm+2.0f) {
                p->short_link_tracking=p->short_link_active=false;
                fail(p,ROUTE_FAILURE_EXIT_RECAPTURE);return;
            }
            float remaining=s.short_link_cm-p->short_link_pivot_lead_cm-p->short_link_forward_cm;
            p->turn_approach_origin_cm=d-(TURN_PIVOT_APPROACH_CM-remaining);
        }
    }
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
        /* Parallel pose trigger: the table corner is known in track coordinates,
         * so encoders+IMU can hand the row over even when a printed symbol
         * hides the wire or the leg is shorter than the 24 cm look-ahead. */
        if(p->pose_mode && p->state==ROUTE_FOLLOW) {
            float wx=p->waypoint_x_cm[p->index],wy=p->waypoint_y_cm[p->index];
            float wh=h-(float)M_PI*.5f;
            float sx=axle_x_cm+GRAY_FORWARD_OFFSET_CM*cosf(wh);
            float sy=axle_y_cm+GRAY_FORWARD_OFFSET_CM*sinf(wh);
            float case_gap=hypotf(axle_x_cm-wx,axle_y_cm-wy);
            float bar_gap=hypotf(sx-wx,sy-wy);
            float proj=(wx-axle_x_cm)*cosf(wh)+(wy-axle_y_cm)*sinf(wh);
            float lateral=fabsf(-(wx-axle_x_cm)*sinf(wh)+(wy-axle_y_cm)*cosf(wh));
            float bar_proj=proj-GRAY_FORWARD_OFFSET_CM;
            bool passed=bar_proj<=ROUTE_WAYPOINT_CONFIRM_CM && lateral<=ROUTE_WAYPOINT_GATE_CM;
            bool reached=bar_gap<=ROUTE_WAYPOINT_CONFIRM_CM || passed;
            bool fallback=s.segment_length_cm>0.0f &&
                d-p->origin_cm>=s.segment_length_cm+ROUTE_WAYPOINT_FALLBACK_CM;
            (void)case_gap;
            p->pose_trigger_late_cm=bar_proj<0.0f?-bar_proj:0.0f;
            if((reached||fallback) && s.action!=STRAIGHT) {
                p->candidate_count=0;p->search_pending=false;p->passing_junction=false;
                if(p->junctions_seen<UINT16_MAX) ++p->junctions_seen;
                if(route_current_action(p)!=STRAIGHT && !entry_speed_ready(p)) {
                    p->state=ROUTE_BRAKING;p->phase_ms=now;p->entry_cm=d;p->entry_heading=h;return;
                }
                accept_target(p,d,h,now);return;
            }
        }
        bool narrow=route_is_narrow_line(o);
        bool center=narrow && fabsf(o.position)<=JUNCTION_CENTER_LIMIT;
        /* Post-turn slow ends as soon as the outgoing line is re-centred: the
         * pivot's yaw residual is gone and the next corner can be planned at
         * normal speed again (mirror of the simulator). */
        if(p->post_turn_slow && narrow && center && adjacent) {
            if(++p->post_turn_samples>=LINE_CONFIRM_SAMPLES) p->post_turn_slow=false;
        } else if(p->post_turn_slow && (!narrow || !center)) {
            p->post_turn_samples=0;
        }
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
        /* The three-channel drift is a single sliding line, but a real branch
         * adds at least a fourth channel before it can reach center. Keep the
         * existing latch for narrow connected shapes; wider connected shapes
         * retain the ordinary strong-branch confirmation. */
        bool smooth=(kind==JUNCTION_EVIDENCE_EDGE ||
            (kind==JUNCTION_EVIDENCE_CENTER_BRANCH &&
             o.white_count<=JUNCTION_EDGE_MAX_WIDTH)) &&
            (sides&p->smooth_edge_sides);
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
        bool pose_matches=true;
        if(p->pose_mode) {
            float gate=s.action==HEXAGON?ROUTE_WAYPOINT_RING_CONFIRM_CM:ROUTE_WAYPOINT_GATE_CM;
            float wh=h-(float)M_PI*.5f;
            float sx=axle_x_cm+GRAY_FORWARD_OFFSET_CM*cosf(wh);
            float sy=axle_y_cm+GRAY_FORWARD_OFFSET_CM*sinf(wh);
            pose_matches=hypotf(sx-p->waypoint_x_cm[p->index],sy-p->waypoint_y_cm[p->index])<=gate;
        }
        p->passing_junction=!route_target_ready(p,d,false) || !pose_matches;
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
        /* Sustained loss while braking cannot re-find the node; hand over to the
         * bounded recovery instead of letting the generic loss guard stop the
         * run. Single lost frames are still ignored above. */
        if(!p->short_link_active && !p->pose_mode && o.line_mask==0 &&
           (uint32_t)(now-p->phase_ms)>=INTERFERENCE_TIMEOUT_MS) {
            fail(p,ROUTE_FAILURE_ENTRY_SPEED);return;
        }
        /* Never start a pivot without any line under the sensor: a component
         * symbol hides the wire for a few cm, so wait for it to reappear or
         * hand over to the bounded recovery after the loss budget above. */
        if(entry_speed_ready(p) && (o.line_mask!=0 || p->short_link_active || p->pose_mode)) {
            if(p->short_link_active) {
                p->state=ROUTE_EXECUTING;p->phase_ms=now;p->entry_cm=d;
                p->turn_approaching=false;reset_evidence(p,now);
                start_short_link_tracking(p,d,h,now);return;
            }
            /* Printed component symbols hide the wire for a few cm around a
             * node, so a zero mask here is not proof the node is gone. The
             * action's own approach/exit contracts and the bounded recovery
             * still guard a blind pivot; do not abort on a single lost frame. */
            accept_target(p,d,h,now);
        }
        return;
    }
    if(s.action==HEXAGON && !p->passing_junction && p->state==ROUTE_EXECUTING) {
        hexagon_update(p,o,d,h,now,adjacent);return;
    }
    if(p->state==ROUTE_EXECUTING && p->turn_approaching) {
        if((uint32_t)(now-p->phase_ms)>=JUNCTION_APPROACH_TIMEOUT_MS) {
            fail(p,ROUTE_FAILURE_ACTION_TIMEOUT);return;
        }
        if(d-p->turn_approach_origin_cm<p->turn_approach_cm) return;
        if(p->short_link_active && !entry_speed_ready(p)) {
            p->state=ROUTE_BRAKING;p->phase_ms=now;p->entry_cm=d;return;
        }
        p->turn_approaching=false;
        /* Approach wheel inertia may already yaw the chassis; retain the
         * action's heading reference, including for explicit short links. */
        p->phase_ms=now;p->entry_cm=d;reset_evidence(p,now);
        start_short_link_tracking(p,d,h,now);
    }
    Action action=route_current_action(p);
    /* 角度右正左负：CUSTOM_TURN 的转向符号必须取自配置角，
     * 不能用动作枚举推断，否则 -90° 自定义角会被当成右转。 */
    float sign=route_turn_target_rad(p)<0.0f?-1.0f:1.0f;
    float turn=(h-p->entry_heading)*sign;
    float target=fabsf(route_turn_target_rad(p));
    /* Pose exit: on a short leg the outgoing line can run out under the 24 cm
     * look-ahead bar a few cm after the pivot, so the grey-only contract would
     * time out.  When the axle has reached the pivot angle at the known corner,
     * hand the next row over immediately. */
    if(p->pose_mode && action!=STRAIGHT && !p->passing_junction) {
        float gap=hypotf(axle_x_cm-p->waypoint_x_cm[p->index],
                         axle_y_cm-p->waypoint_y_cm[p->index]);
        if(gap<=ROUTE_WAYPOINT_EXIT_CM && turn>=target-ROUTE_WAYPOINT_EXIT_ANGLE_RAD) {
            p->line_exit_confirmed=true;p->line_exit_heading_rad=h;
            enter_clear(p,d,now);return;
        }
    }
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
        bool ordinary_rewind=ordinary_right_angle(s);
        if(!p->turn_rewind_active && ordinary_rewind) {
            if(!narrow && turn>=target-0.03f && adjacent) {
                if(!p->turn_rewind_pending) {p->turn_rewind_pending=true;p->turn_rewind_ms=now;}
                if((uint32_t)(now-p->turn_rewind_ms)>=150U) p->turn_rewind_active=true;
            } else p->turn_rewind_pending=false;
        }
        /* Ordinary turns keep their angle/exit contract through gray loss.
         * Only the action bound aborts to RECOVERING; HEXAGON is separate. */
        if(component_recovery(p,o,d,h,now,adjacent,!narrow && p->capture_line_seen &&
           turn>=target*(1.0f-JUNCTION_EXIT_ANGLE_MARGIN_RATIO))) return;
        /* 30–40 cm 短段：前探头在转弯途中就会越过下一个节点。这里只记录
         * “转过 45° 之后”出现的带中心支路证据，作为下一段相邻交接的种子，
         * 避免把本路口自身的转角读数重复计入。 */
        if(!p->passing_junction && turn>=0.8f) {
            uint8_t tsides=0;
            if(junction_shape(o,&tsides)!=JUNCTION_EVIDENCE_NONE && tsides!=0) {
                if(p->turn_branch_samples==0) p->turn_branch_sides=tsides;
                else p->turn_branch_sides&=(uint8_t)tsides;
                if(p->turn_branch_samples<UINT8_MAX) ++p->turn_branch_samples;
            }
        }
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
        /* 普通转弯找到连续居中窄出口即继续巡线；非90°自定义角保留角度门禁。
         * ±90° CUSTOM_TURN 与 LEFT/RIGHT 几何相同，捕到出口线后必须停止
         * 继续强转并交回位置 PID，否则会越过出口线、丢线。 */
        bool ordinary_exit=s.action!=CUSTOM_TURN ||
            fabsf(fabsf(route_turn_target_rad(p))-JUNCTION_TURN_TARGET_RAD)<0.0001f;
        bool angle_full=turn>=target;
        float alignment=p->turn_rewind_active?fmaxf(JUNCTION_EXIT_ALIGNMENT_RAD,JUNCTION_EXIT_REWIND_RAD+0.03f):JUNCTION_EXIT_ALIGNMENT_RAD;
        bool line_ready=ordinary_exit && turn>=target-alignment && p->turn_capturing &&
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
    /* Pose mode: the corner is known, so the clearance dedup cannot gain any
     * information; a couple of cm keeps the order without letting the outgoing
     * line loss stop a short leg. */
    if(p->pose_mode && d-p->clear_cm>=ROUTE_CLEAR_POSE_CM) {
        complete_clear(p,d,now);return;
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
        /* 相邻路口也可能以单侧 T 形（中心+分支）出现，例如 30 cm 短段
         * (245,140)->(245,170)；这里与普通路口共用同一形状分型，
         * 仍需走廊、连续帧、侧别一致和前进位移，不直接放行。 */
        uint8_t sides=0;
        JunctionEvidence adjacent_kind=junction_shape(o,&sides);
        bool branch=adjacent_kind!=JUNCTION_EVIDENCE_NONE && sides!=0;
        if(p->next_junction_samples>0 && !(p->next_junction_sides&sides)) {
            p->next_junction_samples=0;p->candidate_reject=JUNCTION_REJECT_SIDE_CHANGE;
            ++p->candidate_restarts;
        }
        bool sensor_ready=next_segment_sensor_ready(p,d);
        /* 转弯中已越过的节点：只要按探头距离计已到达下一段长度，就用当时的
         * 侧别证据补齐交接样本，不再要求出口后的新走廊。 */
        if(sensor_ready && p->next_junction_samples==0 && p->turn_branch_samples>=LINE_CONFIRM_SAMPLES &&
           (p->turn_branch_sides&sides)) {
            p->next_junction_ms=now;p->next_junction_cm=d;
            p->next_junction_sides=p->turn_branch_sides;
            p->next_junction_samples=JUNCTION_CONFIRM_SAMPLES;
        }
        if(adjacent && branch && (corridor || p->next_junction_samples>0 || sensor_ready)) {
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
                p->candidate_count=JUNCTION_CONFIRM_SAMPLES;
                p->candidate_separated=adjacent_kind==JUNCTION_EVIDENCE_SEPARATED;
                p->candidate_kind=adjacent_kind;p->candidate_sides=confirmed_sides;
                /* Consume the already recorded real samples on the new step;
                 * recursion changes state once and cannot re-enter this branch. */
                route_update_pose(p,o,d,h,now,axle_x_cm,axle_y_cm);return;
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
