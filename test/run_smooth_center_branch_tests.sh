#!/bin/sh
set -eu
project_dir=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
route_source=${1:-"$project_dir/src/route.c"}
test_binary=$(mktemp "${TMPDIR:-/tmp}/car_stm32_smooth.XXXXXX")
# mktemp supplies an absolute file path; limit cleanup to that generated basename.
case "$test_binary" in /*/car_stm32_smooth.*) ;; *) exit 1 ;; esac
trap 'rm -f -- "$test_binary"' EXIT HUP INT TERM
for invert in 0 1; do
    "${CC:-cc}" -std=c11 -Wall -Wextra -Werror -pedantic -g -DPOSE_NAV_ENABLED=0 \
        -fsanitize=address,undefined -fno-omit-frame-pointer \
        -I"$project_dir/include" -DGRAY_INVERT_ENABLED="$invert" \
        "$project_dir/test/test_smooth_center_branch.c" \
        "$project_dir/src/gray_sensor.c" "$route_source" -lm -o "$test_binary"
    "$test_binary"
done
