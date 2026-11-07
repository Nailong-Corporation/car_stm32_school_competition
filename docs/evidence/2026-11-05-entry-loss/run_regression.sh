#!/bin/sh
set -eu
project_dir=$(CDPATH= cd -- "$(dirname -- "$0")/../../.." && pwd)
cd "$project_dir"
test_dir=$(mktemp -d "$project_dir/.pio-entry-loss.XXXXXX")
trap 'rm -rf "$test_dir"' EXIT HUP INT TERM
for imu in 0 1; do
 for invert in 0 1; do
  cc -std=c11 -Wall -Wextra -Werror -pedantic -g -DPOSE_NAV_ENABLED=0 \
   -fsanitize=address,undefined -fno-omit-frame-pointer -Iinclude \
   -DMPU6050_ENABLED="$imu" -DGRAY_INVERT_ENABLED="$invert" -DODOMETRY_HEADING_SCALE=1.0f \
   test/test_entry_loss.c src/gray_sensor.c src/pid.c src/odometry.c \
   src/braking_planner.c src/route.c src/speed_control.c src/line_follow.c -lm -o "$test_dir/test"
  printf "entry-loss configuration: IMU=%s GRAY_INVERT=%s\n" "$imu" "$invert"
  "$test_dir/test" "$@"
 done
done
