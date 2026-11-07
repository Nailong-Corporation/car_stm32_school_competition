#!/bin/sh
set -eu
project_dir=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
controller=${1:-"$project_dir/src/line_follow.c"}
test_binary=$(mktemp "${TMPDIR:-/tmp}/car_stm32_pass_heading.XXXXXX")
case "$test_binary" in /*/car_stm32_pass_heading.*) ;; *) exit 1 ;; esac
trap 'rm -f -- "$test_binary"' EXIT HUP INT TERM
"${CC:-cc}" -std=c11 -Wall -Wextra -Werror -pedantic -g -DPOSE_NAV_ENABLED=0 \
    -fsanitize=address,undefined -fno-omit-frame-pointer \
    -I"$project_dir/include" -DMPU6050_ENABLED=1 -DODOMETRY_HEADING_SCALE=1.0f \
    "$project_dir/test/test_pass_heading.c" "$project_dir/src/gray_sensor.c" \
    "$project_dir/src/pid.c" "$project_dir/src/odometry.c" "$project_dir/src/braking_planner.c" \
    "$project_dir/src/route.c" "$project_dir/src/speed_control.c" "$controller" \
    -lm -o "$test_binary"
"$test_binary"
