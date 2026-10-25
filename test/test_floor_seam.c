#include "gray_test_input.h"
#include "line_follow.h"
#include <assert.h>
#include <stdio.h>

static unsigned checks;
#define CHECK(c) do { ++checks; assert(c); } while (0)
static GrayObservation sample(GraySensor *sensor, uint8_t line)
{
    return gray_sensor_update(sensor,raw_for_line(line),true,0);
}
static void test_masks(void)
{
    for (unsigned input=0;input<256;++input) {
        uint8_t expected=(uint8_t)input;
#if FLOOR_SEAM_FILTER_ENABLED
        expected=0;
        /* 独立按连续区域枚举，不复制相邻位运算实现。 */
        for (unsigned i=0;i<8;) {
            if (!(input & (1U<<i))) { ++i; continue; }
            unsigned first=i;
            while (i<8 && (input & (1U<<i))) ++i;
            if (i-first>=2) for (unsigned j=first;j<i;++j) expected|=(uint8_t)(1U<<j);
        }
#endif
        GraySensor sensor; gray_sensor_init(&sensor);
        GrayObservation obs=sample(&sensor,(uint8_t)input);
        CHECK(obs.raw_mask==raw_for_line((uint8_t)input));
        CHECK(obs.prefilter_mask==input && obs.line_mask==expected);
        CHECK(obs.seam_mask==(uint8_t)(input & (uint8_t)~expected));
        if (!expected) CHECK(obs.kind==GRAY_NONE);
    }
    GraySensor sensor; gray_sensor_init(&sensor);
    for (unsigned i=0;i<3;++i) (void)sample(&sensor,0x18);
    GrayObservation obs=sample(&sensor,0x98);
    CHECK(obs.prefilter_mask==0x18); /* 一帧边缘噪声仍先受时间多数滤波。 */
    obs=sample(&sensor,0x98);
    CHECK(obs.prefilter_mask==0x98);
#if FLOOR_SEAM_FILTER_ENABLED
    CHECK(obs.line_mask==0x18 && obs.seam_mask==0x80 && obs.region_count==1 && obs.position==0);
#else
    CHECK(obs.line_mask==0x98 && obs.seam_mask==0 && obs.region_count==2);
#endif
    gray_sensor_init(&sensor); obs=sample(&sensor,0xdb); /* 三个两路区域不能被滤成中心线。 */
    CHECK(obs.line_mask==0xdb && obs.region_count==3 && obs.seam_mask==0);
    gray_sensor_init(&sensor); obs=sample(&sensor,0xff);
    CHECK(obs.line_mask==0xff && obs.kind==GRAY_AMBIGUOUS);
}
static void test_start_and_tracking(void)
{
    const OdometryCalibration c={4.8f,4.8f,1560,1560,14.5f,-1,1,true};
    LineFollow f; GraySensor sensor; uint32_t now=0;
    line_follow_init(&f,c,route,route_count,NULL,NULL); gray_sensor_init(&sensor);
    for (unsigned i=0;i<10;++i) {
        now+=5;
        line_follow_update(&f,sample(&sensor,0x04),(EncoderCounts){0},now,.005f,i==9);
    }
#if FLOOR_SEAM_FILTER_ENABLED
    CHECK(f.state!=LINE_TRACKING && !f.command.enabled && f.start_result==LINE_START_REJECTED_LINE);
#else
    CHECK(f.state==LINE_TRACKING && f.start_result==LINE_START_ACCEPTED);
#endif
    line_follow_init(&f,c,route,route_count,NULL,NULL); gray_sensor_init(&sensor);
    for (unsigned i=0;i<10;++i) {
        now+=5;
        line_follow_update(&f,sample(&sensor,0x18),(EncoderCounts){0},now,.005f,i==9);
    }
    CHECK(f.state==LINE_TRACKING && f.start_result==LINE_START_ACCEPTED && f.command.enabled);
    for (unsigned i=0;i<3;++i) {
        now+=5;
        line_follow_update(&f,sample(&sensor,0x98),(EncoderCounts){0},now,.005f,false);
    }
#if FLOOR_SEAM_FILTER_ENABLED
    CHECK(f.state==LINE_TRACKING && f.observation.line_mask==0x18 && f.observation.prefilter_mask==0x98);
#else
    CHECK(f.state==LINE_GAP_HOLD && f.observation.region_count==2);
#endif
    line_follow_stop(&f,LINE_STOP_REQUEST);
    CHECK(f.stop_snapshot.observation.prefilter_mask==0x98);
#if FLOOR_SEAM_FILTER_ENABLED
    CHECK(f.stop_snapshot.observation.seam_mask==0x80);
#endif
}
static void test_junction_guard(void)
{
    const RouteStep steps[]={{10,5,LEFT,20,0, true},{100,5,STOP,20,0, true}};
    RoutePlanner p; route_init(&p,steps,2,NULL,NULL); p.repeat_count=1;
    route_start(&p,0,0);
    GraySensor sensor; gray_sensor_init(&sensor);
    for (unsigned i=0;i<5;++i) route_update(&p,sample(&sensor,0x18),9,0,5+5*i);
    for (unsigned i=0;i<3;++i) route_update(&p,sample(&sensor,0x99),9+.02f*(float)i,0,30+5*i);
#if FLOOR_SEAM_FILTER_ENABLED
    CHECK(p.state==ROUTE_FOLLOW && !p.turn_forced);
#else
    CHECK(p.state==ROUTE_EXECUTING && p.turn_forced); /* 展示相同细缝如何触发旧骤变规则。 */
#endif
    route_start(&p,0,0); gray_sensor_init(&sensor);
    for (unsigned i=0;i<5;++i) route_update(&p,sample(&sensor,0x18),9,0,5+5*i);
    for (unsigned i=0;i<3;++i) route_update(&p,sample(&sensor,0x80),9,0,30+5*i);
    CHECK(p.state==ROUTE_FOLLOW && !p.turn_forced); /* 过滤后为零也不能伪装成真实丢线骤变。 */
    route_start(&p,0,0); gray_sensor_init(&sensor);
    for (unsigned i=0;i<5;++i) route_update(&p,sample(&sensor,0x18),9,0,5+5*i);
    for (unsigned i=0;i<3;++i) route_update(&p,sample(&sensor,0),9,0,30+5*i);
    CHECK(p.state==ROUTE_EXECUTING && p.turn_forced); /* 真实无线原行为保留。 */
    route_start(&p,0,0); gray_sensor_init(&sensor);
    for (unsigned i=0;i<5;++i) route_update(&p,sample(&sensor,0x18),9,0,5+5*i);
    for (unsigned i=0;i<3;++i) route_update(&p,sample(&sensor,0x1f),9,0,30+5*i);
    CHECK(p.state==ROUTE_EXECUTING && p.turn_forced); /* 真实连续分支仍可转弯。 */
}
int main(void)
{
    test_masks(); test_start_and_tracking(); test_junction_guard();
    printf("PASS: seam filter=%d polarity=%d encoder=%d route=%d; %u checks\n",
           FLOOR_SEAM_FILTER_ENABLED,GRAY_INVERT_ENABLED,ENCODER_CONTROL_ENABLED,PRESET_ROUTE_ENABLED,checks);
    return 0;
}
