#!/bin/sh
set -eu
project_dir=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
test_binary=$(mktemp "${TMPDIR:-/tmp}/car_stm32_test.XXXXXX")
trap 'rm -f "$test_binary"' EXIT HUP INT TERM
for invert in 1 0; do
    echo "GRAY_INVERT_ENABLED=$invert"
    "${CC:-cc}" -DGRAY_INVERT_ENABLED="$invert" -std=c11 -Wall -Wextra -Werror -pedantic -g \
        -fsanitize=address,undefined -fno-omit-frame-pointer \
        -I"$project_dir/include" \
        "$project_dir/test/test_logic.c" \
        "$project_dir/src/gray_sensor.c" "$project_dir/src/pid.c" \
        "$project_dir/src/motor_command.c" \
        -lm -o "$test_binary"
    "$test_binary"
    "${CC:-cc}" -DGRAY_INVERT_ENABLED="$invert" -std=c11 -Wall -Wextra -Werror -pedantic -g \
        -fsanitize=address,undefined -fno-omit-frame-pointer \
        -I"$project_dir/test/fakes" -I"$project_dir/include" \
        "$project_dir/test/test_encoder_hal.c" "$project_dir/src/encoder.c" \
        -o "$test_binary"
    "$test_binary"
    "${CC:-cc}" -DGRAY_INVERT_ENABLED="$invert" -std=c11 -Wall -Wextra -Werror -pedantic -g \
        -fsanitize=address,undefined -fno-omit-frame-pointer \
        -I"$project_dir/test/fakes" -I"$project_dir/include" \
        "$project_dir/test/test_motor_hal.c" "$project_dir/src/motor.c" \
        "$project_dir/src/motor_command.c" -lm -o "$test_binary"
    "$test_binary"
    "${CC:-cc}" -DGRAY_INVERT_ENABLED="$invert" -std=c11 -Wall -Wextra -Werror -pedantic -g \
        -fsanitize=address,undefined -fno-omit-frame-pointer \
        -I"$project_dir/include" \
        -DENCODER_CONTROL_ENABLED=1 -DPRESET_ROUTE_ENABLED=1 \
        "$project_dir/test/test_navigation.c" "$project_dir/src/gray_sensor.c" \
        "$project_dir/src/pid.c" "$project_dir/src/odometry.c" \
        "$project_dir/src/route.c" "$project_dir/src/route_table.c" \
        "$project_dir/src/speed_control.c" "$project_dir/src/line_follow.c" \
        -lm -o "$test_binary"
    "$test_binary"
    "${CC:-cc}" -DGRAY_INVERT_ENABLED="$invert" -std=c11 -Wall -Wextra -Werror -pedantic -g \
        -fsanitize=address,undefined -fno-omit-frame-pointer \
        -I"$project_dir/include" \
        -DENCODER_CONTROL_ENABLED=1 -DPRESET_ROUTE_ENABLED=0 \
        "$project_dir/test/test_line_only.c" "$project_dir/src/gray_sensor.c" \
        "$project_dir/src/pid.c" "$project_dir/src/odometry.c" \
        "$project_dir/src/route.c" "$project_dir/src/speed_control.c" \
        "$project_dir/src/line_follow.c" -lm -o "$test_binary"
    "$test_binary"
    "${CC:-cc}" -DGRAY_INVERT_ENABLED="$invert" -std=c11 -Wall -Wextra -Werror -pedantic -g \
        -fsanitize=address,undefined -fno-omit-frame-pointer \
        -I"$project_dir/include" -DENCODER_CONTROL_ENABLED=0 -DPRESET_ROUTE_ENABLED=0 \
        "$project_dir/test/test_open_loop.c" "$project_dir/src/gray_sensor.c" \
        "$project_dir/src/pid.c" "$project_dir/src/motor_command.c" \
        "$project_dir/src/route.c" "$project_dir/src/line_follow.c" \
        -lm -o "$test_binary"
    "$test_binary"
done
