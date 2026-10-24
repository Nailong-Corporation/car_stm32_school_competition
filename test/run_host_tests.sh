#!/bin/sh
set -eu
project_dir=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
test_binary=$(mktemp "${TMPDIR:-/tmp}/car_stm32_test.XXXXXX")
trap 'rm -f "$test_binary"' EXIT HUP INT TERM
for invert in 1 0; do
    echo "GRAY_INVERT_ENABLED=$invert"
    "${CC:-cc}" -DGRAY_INVERT_ENABLED="$invert" -DFLOOR_SEAM_FILTER_ENABLED=0 -DLOST_LINE_STOP_ENABLED=1 -std=c11 -Wall -Wextra -Werror -pedantic -g \
        -fsanitize=address,undefined -fno-omit-frame-pointer \
        -I"$project_dir/include" \
        "$project_dir/test/test_logic.c" \
        "$project_dir/src/gray_sensor.c" "$project_dir/src/pid.c" \
        "$project_dir/src/motor_command.c" \
        -lm -o "$test_binary"
    "$test_binary"
    "${CC:-cc}" -DGRAY_INVERT_ENABLED="$invert" -DFLOOR_SEAM_FILTER_ENABLED=0 -DLOST_LINE_STOP_ENABLED=1 -std=c11 -Wall -Wextra -Werror -pedantic -g \
        -fsanitize=address,undefined -fno-omit-frame-pointer \
        -I"$project_dir/test/fakes" -I"$project_dir/include" \
        "$project_dir/test/test_encoder_hal.c" "$project_dir/src/encoder.c" \
        -o "$test_binary"
    "$test_binary"
    "${CC:-cc}" -DGRAY_INVERT_ENABLED="$invert" -DFLOOR_SEAM_FILTER_ENABLED=0 -DLOST_LINE_STOP_ENABLED=1 -std=c11 -Wall -Wextra -Werror -pedantic -g \
        -fsanitize=address,undefined -fno-omit-frame-pointer \
        -I"$project_dir/test/fakes" -I"$project_dir/include" \
        "$project_dir/test/test_motor_hal.c" "$project_dir/src/motor.c" \
        "$project_dir/src/motor_command.c" -lm -o "$test_binary"
    "$test_binary"
    "${CC:-cc}" -DGRAY_INVERT_ENABLED="$invert" -DFLOOR_SEAM_FILTER_ENABLED=0 -DLOST_LINE_STOP_ENABLED=1 -std=c11 -Wall -Wextra -Werror -pedantic -g \
        -fsanitize=address,undefined -fno-omit-frame-pointer \
        -I"$project_dir/include" \
        -DENCODER_CONTROL_ENABLED=1 -DPRESET_ROUTE_ENABLED=1 \
        "$project_dir/test/test_configured_start.c" "$project_dir/src/gray_sensor.c" \
        "$project_dir/src/pid.c" "$project_dir/src/odometry.c" \
        "$project_dir/src/route.c" "$project_dir/src/route_table.c" \
        "$project_dir/src/speed_control.c" "$project_dir/src/line_follow.c" \
        -lm -o "$test_binary"
    "$test_binary"
    "${CC:-cc}" -DGRAY_INVERT_ENABLED="$invert" -DFLOOR_SEAM_FILTER_ENABLED=0 -DLOST_LINE_STOP_ENABLED=1 -std=c11 -Wall -Wextra -Werror -pedantic -g \
        -fsanitize=address,undefined -fno-omit-frame-pointer \
        -I"$project_dir/include" \
        -DENCODER_CONTROL_ENABLED=1 -DPRESET_ROUTE_ENABLED=1 \
        "$project_dir/test/test_navigation.c" "$project_dir/src/gray_sensor.c" \
        "$project_dir/src/pid.c" "$project_dir/src/odometry.c" \
        "$project_dir/src/route.c" "$project_dir/src/route_table.c" \
        "$project_dir/src/speed_control.c" "$project_dir/src/line_follow.c" \
        -lm -o "$test_binary"
    "$test_binary"
    "${CC:-cc}" -DGRAY_INVERT_ENABLED="$invert" -DFLOOR_SEAM_FILTER_ENABLED=0 -DLOST_LINE_STOP_ENABLED=1 -std=c11 -Wall -Wextra -Werror -pedantic -g \
        -fsanitize=address,undefined -fno-omit-frame-pointer \
        -I"$project_dir/include" \
        -DENCODER_CONTROL_ENABLED=1 -DPRESET_ROUTE_ENABLED=0 \
        "$project_dir/test/test_line_only.c" "$project_dir/src/gray_sensor.c" \
        "$project_dir/src/pid.c" "$project_dir/src/odometry.c" \
        "$project_dir/src/route.c" "$project_dir/src/speed_control.c" \
        "$project_dir/src/line_follow.c" -lm -o "$test_binary"
    "$test_binary"
    "${CC:-cc}" -DGRAY_INVERT_ENABLED="$invert" -DFLOOR_SEAM_FILTER_ENABLED=0 -DLOST_LINE_STOP_ENABLED=1 -std=c11 -Wall -Wextra -Werror -pedantic -g \
        -fsanitize=address,undefined -fno-omit-frame-pointer \
        -I"$project_dir/include" -DENCODER_CONTROL_ENABLED=0 -DPRESET_ROUTE_ENABLED=0 \
        "$project_dir/test/test_open_loop.c" "$project_dir/src/gray_sensor.c" \
        "$project_dir/src/pid.c" "$project_dir/src/motor_command.c" \
        "$project_dir/src/route.c" "$project_dir/src/line_follow.c" \
        -lm -o "$test_binary"
    "$test_binary"
done

# 单独验证生产头文件丢线开关的两个分支；其余完整回归始终保留停车保护。
for loss_stop in 1 0; do
    for mode in "0 0" "1 0" "1 1"; do
        set -- $mode
        "${CC:-cc}" -std=c11 -Wall -Wextra -Werror -pedantic -g \
            -fsanitize=address,undefined -fno-omit-frame-pointer \
            -I"$project_dir/include" -DFLOOR_SEAM_FILTER_ENABLED=0 -DLOST_LINE_STOP_ENABLED="$loss_stop" \
            -DENCODER_CONTROL_ENABLED="$1" -DPRESET_ROUTE_ENABLED="$2" \
            "$project_dir/test/test_loss_stop.c" "$project_dir/src/gray_sensor.c" \
            "$project_dir/src/pid.c" "$project_dir/src/odometry.c" \
            "$project_dir/src/route.c" "$project_dir/src/speed_control.c" \
            "$project_dir/src/line_follow.c" "$project_dir/src/motor_command.c" \
            -lm -o "$test_binary"
        "$test_binary"
    done
done

# 地板练习开关与比赛原行为：两种极性和三个模式分别验证传感器到控制器链路。
for invert in 1 0; do
    for seam_filter in 1 0; do
        for mode in "0 0" "1 0" "1 1"; do
            set -- $mode
            "${CC:-cc}" -std=c11 -Wall -Wextra -Werror -pedantic -g \
                -fsanitize=address,undefined -fno-omit-frame-pointer \
                -I"$project_dir/include" -DGRAY_INVERT_ENABLED="$invert" \
                -DFLOOR_SEAM_FILTER_ENABLED="$seam_filter" -DLOST_LINE_STOP_ENABLED=1 \
                -DENCODER_CONTROL_ENABLED="$1" -DPRESET_ROUTE_ENABLED="$2" \
                "$project_dir/test/test_floor_seam.c" "$project_dir/src/gray_sensor.c" \
                "$project_dir/src/pid.c" "$project_dir/src/odometry.c" \
                "$project_dir/src/route.c" "$project_dir/src/route_table.c" \
                "$project_dir/src/speed_control.c" "$project_dir/src/line_follow.c" \
                "$project_dir/src/motor_command.c" -lm -o "$test_binary"
            "$test_binary"
        done
    done
done
