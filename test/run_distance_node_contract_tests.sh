#!/bin/sh
set -eu
project_dir=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
test_binary=$(mktemp "${TMPDIR:-/tmp}/car_distance_contract.XXXXXX")
trap 'rm -f "$test_binary"' EXIT HUP INT TERM
python3 "$project_dir/test/sync_new_track_routes.py" --check
for invert in 0 1; do
    for imu in 0 1; do
        "${CC:-cc}" -std=c11 -Wall -Wextra -Werror -pedantic -g -DPOSE_NAV_ENABLED=0 \
            -fsanitize=address,undefined -fno-omit-frame-pointer \
            -I"$project_dir/include" -DGRAY_INVERT_ENABLED="$invert" \
            -DMPU6050_ENABLED="$imu" -DODOMETRY_HEADING_SCALE=1.0f \
            "$project_dir/test/test_distance_node_contract.c" \
            "$project_dir/src/gray_sensor.c" "$project_dir/src/pid.c" \
            "$project_dir/src/odometry.c" "$project_dir/src/braking_planner.c" \
            "$project_dir/src/route.c" "$project_dir/src/speed_control.c" \
            "$project_dir/src/line_follow.c" -lm -o "$test_binary"
        "$test_binary"
    done
done
