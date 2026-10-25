#ifndef ODOMETRY_H
#define ODOMETRY_H
#include <stdbool.h>
#include <stdint.h>

typedef struct { uint16_t left, right; } EncoderCounts;
typedef struct {
    float left_diameter_cm, right_diameter_cm;
    float left_counts_per_rev, right_counts_per_rev, track_cm;
    int left_sign, right_sign;
    bool confirmed;
} OdometryCalibration;
typedef struct {
    OdometryCalibration calibration;
    EncoderCounts previous;
    bool ready, primed;
    float left_cm_per_count, right_cm_per_count;
    float distance_cm, heading_rad, left_speed_cm_s, right_speed_cm_s;
    float left_distance_cm, right_distance_cm; /* 有符号累计轮程。 */
    float x_cm, y_cm; /* 轮轴中点位姿：初始前方 x，初始右方 y。 */
    float sensor_x_cm, sensor_y_cm, sensor_distance_cm; /* 探头位姿 / 非负累计轨迹长度。 */
    int32_t left_delta, right_delta;
} Odometry;

void odometry_init(Odometry *odom, OdometryCalibration calibration);
bool odometry_update(Odometry *odom, EncoderCounts counts, float dt);
#endif
