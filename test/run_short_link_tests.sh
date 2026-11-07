#!/bin/sh
set -eu
project_dir=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
test_binary=$(mktemp "${TMPDIR:-/tmp}/car_short_link.XXXXXX")
trap 'rm -f "$test_binary"' EXIT HUP INT TERM
for invert in 0 1; do
    for imu in 0 1; do
        "${CC:-cc}" -std=c11 -Wall -Wextra -Werror -pedantic -g -DPOSE_NAV_ENABLED=0 \
            -fsanitize=address,undefined -fno-omit-frame-pointer \
            -I"$project_dir/include" -DGRAY_INVERT_ENABLED="$invert" \
            -DMPU6050_ENABLED="$imu" \
            "$project_dir/test/test_short_link.c" \
            "$project_dir/src/gray_sensor.c" "$project_dir/src/route.c" \
            -lm -o "$test_binary"
        "$test_binary"
    done
done
