/* Feed the exact simulator observations/encoders/IMU to the production C controller. */
#include "line_follow.h"
#include "motion_config.h"
#include <stdio.h>

int main(void)
{
    LineFollow f;
    OdometryCalibration calibration={LEFT_WHEEL_DIAMETER_CM,RIGHT_WHEEL_DIAMETER_CM,
        LEFT_COUNTS_PER_WHEEL_REV,RIGHT_COUNTS_PER_WHEEL_REV,WHEEL_TRACK_CM,
        ENCODER_LEFT_SIGN,ENCODER_RIGHT_SIGN,ODOMETRY_CALIBRATION_CONFIRMED};
    line_follow_init(&f,calibration,route,route_count,NULL,NULL);
    f.route.repeat_count=1;
    unsigned now,kind,raw,line,selected,width,regions,prefilter,seam,left,right,start,valid;
    float position,dt,heading;
    while(scanf("%u %f %u %u %u %u %u %u %f %u %u %u %u %u %u %f",
        &now,&dt,&kind,&raw,&line,&selected,&width,&regions,&position,&prefilter,&seam,
        &left,&right,&start,&valid,&heading)==16) {
        GrayObservation o={.kind=(GrayKind)kind,.raw_mask=(uint8_t)raw,.line_mask=(uint8_t)line,
            .selected_mask=(uint8_t)selected,.white_count=(uint8_t)width,.region_count=(uint8_t)regions,
            .position=position,.prefilter_mask=(uint8_t)prefilter,.seam_mask=(uint8_t)seam};
        line_follow_update_heading(&f,o,(EncoderCounts){(uint16_t)left,(uint16_t)right},
            now,dt,start!=0,valid!=0,heading);
        printf("%zu %u %u %u %u %.7g %.7g %.9g %.9g\n",f.route.index,
            (unsigned)f.route.state,f.route.junctions_seen,(unsigned)f.route.failure,
            (unsigned)f.state,f.left_target_cm_s,f.right_target_cm_s,
            f.odometry.distance_cm,f.odometry.heading_rad);
    }
    return ferror(stdin)?1:0;
}
