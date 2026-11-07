#!/bin/sh
set -eu
project_dir=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
test_dir=$(mktemp -d "$project_dir/.pio-total-distance.XXXXXX")
test_binary="$test_dir/test"
trap 'rm -rf "$test_dir"' EXIT HUP INT TERM
for imu in 0 1; do
    for invert in 0 1; do
        for suite in test_total_distance test_navigation_review test_race_review; do
            "${CC:-cc}" -std=c11 -Wall -Wextra -Werror -pedantic -g -DPOSE_NAV_ENABLED=0 \
            -fsanitize=address,undefined -fno-omit-frame-pointer \
            -I"$project_dir/include" -DMPU6050_ENABLED="$imu" \
            -DGRAY_INVERT_ENABLED="$invert" -DODOMETRY_HEADING_SCALE=1.0f \
            -DENCODER_CONTROL_ENABLED=1 -DPRESET_ROUTE_ENABLED=1 \
            "$project_dir/test/$suite.c" "$project_dir/src/gray_sensor.c" \
            "$project_dir/src/pid.c" "$project_dir/src/odometry.c" \
            "$project_dir/src/braking_planner.c" "$project_dir/src/route.c" \
            "$project_dir/src/speed_control.c" "$project_dir/src/line_follow.c" \
            -lm -o "$test_binary"
            "$test_binary"
        done
    done
done
