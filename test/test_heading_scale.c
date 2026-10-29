#include "odometry.h"
#include "motion_config.h"
#include "line_follow.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

/* 同一组编码器数据：轮程/轮速保持原比例，左右转位姿使用独立角度比例。 */
int main(void)
{
    const float pi=3.14159265359f;
    OdometryCalibration c={4.8f,4.8f,1560,1560,14.5f,-1,1,true};
    float cm_per_count=pi*4.8f/1560*1.53f;
    assert(fabsf(ODOMETRY_HEADING_SCALE-.77f)<.00001f);
    for(unsigned mirror=0;mirror<2;++mirror) {
        Odometry o; odometry_init(&o,c); assert(o.ready);
        EncoderCounts counts={30000,30000};
        assert(odometry_update(&o,counts,.005f));
        for(unsigned i=0;i<100;++i) {
            counts.left=(uint16_t)(counts.left-(mirror?0:20));
            counts.right=(uint16_t)(counts.right+(mirror?20:0));
            assert(odometry_update(&o,counts,.005f));
        }
        float wheel=2000*cm_per_count;
        float heading=(mirror?-1:1)*wheel/14.5f*.77f;
        assert(fabsf(o.distance_cm-wheel*.5f)<.001f);
        assert(fabsf(o.heading_rad-heading)<.001f);
        assert(fabsf(o.left_distance_cm-(mirror?0:wheel))<.001f);
        assert(fabsf(o.right_distance_cm-(mirror?wheel:0))<.001f);
        assert(fabsf((mirror?o.right_speed_cm_s:o.left_speed_cm_s)-20*cm_per_count/.005f)<.001f);
        /* 恒曲率精确积分，校验车轴和16cm前置探头共用修正后的角度。 */
        float radius=wheel*.5f/fabsf(heading);
        float x=radius*sinf(fabsf(heading));
        float y=(mirror?-1:1)*radius*(1-cosf(heading));
        assert(fabsf(o.x_cm-x)<.001f && fabsf(o.y_cm-y)<.001f);
        assert(fabsf(o.sensor_x_cm-(x+16*cosf(heading)))<.001f);
        assert(fabsf(o.sensor_y_cm-(y+16*sinf(heading)))<.001f);
    }
    Odometry straight; odometry_init(&straight,c);
    EncoderCounts counts={30000,30000};
    assert(odometry_update(&straight,counts,.005f));
    for(unsigned i=0;i<100;++i) {
        counts.left-=20; counts.right+=20;
        assert(odometry_update(&straight,counts,.005f));
    }
    assert(fabsf(straight.distance_cm-2000*cm_per_count)<.001f);
    assert(fabsf(straight.heading_rad)<.00001f);
    /* 实际生产比例的控制链：旧公式已经报90°时，修正后还约69.3°，不能离口。
     * 无线时不得仅凭角度成功；出现真实中心线后才进入捕获。 */
    for(unsigned mirror=0;mirror<2;++mirror) {
        RouteStep steps[]={{220,15,mirror?LEFT:RIGHT,120,0,false},
                           {100,10,STOP,40,0,false}};
        LineFollow f; line_follow_init(&f,c,steps,2,NULL,NULL);
        f.route.repeat_count=1;
        counts=(EncoderCounts){30000,30000};
        GrayObservation center=gray_classify(0x18,true,0);
        uint32_t now=0;
        for(unsigned i=0;i<3;++i) {
            now+=5;
            line_follow_update(&f,center,counts,now,.005f,i==2);
        }
        assert(f.state==LINE_TRACKING);
        f.route.state=ROUTE_EXECUTING; f.route.phase_ms=now;
        f.route.entry_cm=f.route.projection_previous_cm=f.odometry.distance_cm;
        f.startup_elapsed_s=STARTUP_RAMP_TIME_S;
        f.base=f.left_target_cm_s=f.right_target_cm_s=120;
        bool crossed_old_target=false;
        for(unsigned i=1;i<=120 && f.route.state==ROUTE_EXECUTING;++i) {
            counts.left=(uint16_t)(counts.left-(mirror?0:20));
            counts.right=(uint16_t)(counts.right+(mirror?20:0));
            now+=5;
            float raw_angle=i*20*cm_per_count/14.5f;
            float corrected=raw_angle*.77f;
            GrayObservation observation=corrected<pi*.5f?gray_classify(0,true,0):center;
            line_follow_update(&f,observation,counts,now,.005f,false);
            if(corrected<pi*.5f) assert(f.route.state==ROUTE_EXECUTING);
            if(raw_angle>=pi*.5f && corrected<pi*.5f) {
                crossed_old_target=true;
                assert(f.route.state==ROUTE_EXECUTING);
            }
            assert(f.state!=LINE_STOPPED);
            if(f.route.state==ROUTE_EXECUTING)
                assert(fabsf(fmaxf(f.left_target_cm_s,f.right_target_cm_s)-120)<.001f);
        }
        assert(crossed_old_target && f.route.state==ROUTE_CLEARING);
        assert(fabsf(f.odometry.heading_rad)>=pi*.5f);
    }
    puts("PASS: production heading scale, distance/speed, pose and mirrored 90-degree control gates");
    return 0;
}
