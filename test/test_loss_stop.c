#include "line_follow.h"
#include "motion_config.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static void tick(LineFollow *f, EncoderCounts *c, uint32_t *now, uint8_t line, bool start)
{
    c->left += 3; c->right += 3; *now += 5;
    line_follow_update(f,gray_classify(line,true,0),*c,*now,.005f,start);
}
int main(void)
{
    const RouteStep route_steps[]={{1000,10,STOP,12, 0.0f, true}};
    OdometryCalibration cal={1.0f/(3.14159265359f*ODOMETRY_DISTANCE_SCALE),
        1.0f/(3.14159265359f*ODOMETRY_DISTANCE_SCALE),100,100,10,1,1,true};
    LineFollow f; EncoderCounts c={0}; uint32_t now=0;
    line_follow_init(&f,cal,route_steps,1,NULL,NULL);
    /* 无线启动仍拒绝，开关不能引入盲启动。 */
    tick(&f,&c,&now,0,true); assert(!f.command.enabled);
    for (unsigned i=0;i<LINE_CONFIRM_SAMPLES;++i)
        tick(&f,&c,&now,0x18,i==LINE_CONFIRM_SAMPLES-1);
    assert(f.state==LINE_TRACKING);
    for (unsigned i=0;i<210;++i) tick(&f,&c,&now,0x18,false);
    #if !ENCODER_CONTROL_ENABLED
    f.correction=10.0f; /* 超时后历史差值必须衰减至零，不可变成负值。 */
#endif
    for (unsigned i=0;i<360 && f.state!=LINE_STOPPED;++i)
        tick(&f,&c,&now,i%2 ? 0 : 0xff,false);
#if LOST_LINE_STOP_ENABLED
    assert(f.state==LINE_STOPPED && f.stop_reason==LINE_STOP_LOST);
    assert(f.stop_snapshot.valid);
#else
    /* 超过 400 ms、8 cm；NONE/AMBIGUOUS 切换不会影响开关语义。 */
    assert(f.state==LINE_GAP_HOLD && f.command.enabled);
    assert(isfinite(f.correction));
#if !ENCODER_CONTROL_ENABLED
    assert(fabsf(f.correction)<.001f);
#endif
    for(unsigned i=0;i<LINE_CONFIRM_SAMPLES;++i) tick(&f,&c,&now,0x18,false);
    assert(f.state==LINE_TRACKING && f.command.enabled);
    /* 时序故障仍停车并锁定快照。 */
    line_follow_update(&f,gray_classify(0x18,true,0),c,now+20,.020f,false);
    assert(f.state==LINE_STOPPED && f.stop_reason==LINE_STOP_FAULT);
#endif
    puts("PASS: loss stop switch, blind-start gate, prolonged loss and recovery/fault");
    return 0;
}
