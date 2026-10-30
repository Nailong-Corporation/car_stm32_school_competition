#!/bin/sh
set -eu
project_dir=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
test_binary=$(mktemp "${TMPDIR:-/tmp}/car_stm32_test.XXXXXX")
trap 'rm -f "$test_binary"' EXIT HUP INT TERM
# Independent approach / exit / reverse brake switches, all eight combinations.
for approach in 0 1; do
    for exit_accel in 0 1; do
        for reverse_brake in 0 1; do
            "${CC:-cc}" -std=c11 -Wall -Wextra -Werror -pedantic -g \
                -fsanitize=address,undefined -fno-omit-frame-pointer \
                -I"$project_dir/include" -DMPU6050_ENABLED=0 -DODOMETRY_HEADING_SCALE=1.0f \
                -DENCODER_CONTROL_ENABLED=1 -DPRESET_ROUTE_ENABLED=1 \
                -DTURN_APPROACH_SLOWDOWN_ENABLED="$approach" \
                -DTURN_EXIT_ACCEL_ENABLED="$exit_accel" -DMOTOR_REVERSE_BRAKE_ENABLED="$reverse_brake" \
                "$project_dir/test/test_turn_speed_switches.c" "$project_dir/src/gray_sensor.c" \
                "$project_dir/src/pid.c" "$project_dir/src/odometry.c" "$project_dir/src/braking_planner.c" "$project_dir/src/route.c" \
                "$project_dir/src/route_table.c" "$project_dir/src/speed_control.c" \
                "$project_dir/src/line_follow.c" -lm -o "$test_binary"
            "$test_binary"
            "${CC:-cc}" -std=c11 -Wall -Wextra -Werror -pedantic \
                -I"$project_dir/test/fakes" -I"$project_dir/include" \
                -DMOTOR_REVERSE_BRAKE_ENABLED="$reverse_brake" \
                "$project_dir/test/test_motor_hal.c" "$project_dir/src/motor.c" \
                "$project_dir/src/motor_command.c" -lm -o "$test_binary"
            "$test_binary"
        done
    done
done
# Near-angle/no-line convergence and low-speed clearance; real IMU yaw and fallback.
for imu in 0 1; do
    "${CC:-cc}" -std=c11 -Wall -Wextra -Werror -pedantic -g \
        -fsanitize=address,undefined -fno-omit-frame-pointer \
        -I"$project_dir/include" -DMPU6050_ENABLED="$imu" -DODOMETRY_HEADING_SCALE=1.0f \
        "$project_dir/test/test_turn_settling.c" "$project_dir/src/gray_sensor.c" \
        "$project_dir/src/pid.c" "$project_dir/src/odometry.c" \
        "$project_dir/src/braking_planner.c" "$project_dir/src/route.c" \
        "$project_dir/src/speed_control.c" "$project_dir/src/line_follow.c" -lm -o "$test_binary"
    "$test_binary"
done
# Actual production switches and route table must accept a stationary centered-line START.
"${CC:-cc}" -std=c11 -Wall -Wextra -Werror -pedantic -g \
    -fsanitize=address,undefined -fno-omit-frame-pointer \
    -I"$project_dir/include" \
    "$project_dir/test/test_configured_start.c" "$project_dir/src/gray_sensor.c" \
    "$project_dir/src/pid.c" "$project_dir/src/odometry.c" \
    "$project_dir/src/braking_planner.c" "$project_dir/src/route.c" "$project_dir/src/route_table.c" \
    "$project_dir/src/speed_control.c" "$project_dir/src/line_follow.c" \
    -lm -o "$test_binary"
"$test_binary"
# Captured one-count reverse START: inhibit transient output, retain bounded fault lock.
"${CC:-cc}" -std=c11 -Wall -Wextra -Werror -pedantic -g \
    -fsanitize=address,undefined -fno-omit-frame-pointer \
    -I"$project_dir/include" \
    "$project_dir/test/test_reverse_feedback.c" "$project_dir/src/gray_sensor.c" \
    "$project_dir/src/pid.c" "$project_dir/src/odometry.c" \
    "$project_dir/src/braking_planner.c" "$project_dir/src/route.c" "$project_dir/src/route_table.c" \
    "$project_dir/src/speed_control.c" "$project_dir/src/line_follow.c" \
    -lm -o "$test_binary"
"$test_binary"
# Exercise the actual I2C driver with captured bad calibration and bus/register failures.
"${CC:-cc}" -DMPU6050_ENABLED=1 -std=c11 -Wall -Wextra -Werror -pedantic -g \
    -fsanitize=address,undefined -fno-omit-frame-pointer \
    -I"$project_dir/test/fakes" -I"$project_dir/include" \
    "$project_dir/test/test_mpu6050.c" "$project_dir/src/mpu6050.c" \
    "$project_dir/src/tilt_guard.c" "$project_dir/src/line_follow.c" \
    "$project_dir/src/odometry.c" "$project_dir/src/gray_sensor.c" \
    "$project_dir/src/pid.c" "$project_dir/src/braking_planner.c" "$project_dir/src/route.c" \
    "$project_dir/src/speed_control.c" -lm -o "$test_binary"
"$test_binary"
# 生产转角比例与理想几何分开验证；不覆盖PlatformIO固件配置。
"${CC:-cc}" -DMPU6050_ENABLED=0 -std=c11 -Wall -Wextra -Werror -pedantic -g \
    -fsanitize=address,undefined -fno-omit-frame-pointer \
    -I"$project_dir/include" "$project_dir/test/test_heading_scale.c" \
    "$project_dir/src/odometry.c" "$project_dir/src/gray_sensor.c" \
    "$project_dir/src/line_follow.c" "$project_dir/src/pid.c" \
    "$project_dir/src/braking_planner.c" "$project_dir/src/route.c" "$project_dir/src/route_table.c" \
    "$project_dir/src/speed_control.c" -lm -o "$test_binary"
"$test_binary"
# 含倒置正反号、40°门槛、瞬态滤波、持续确认和锁定逻辑。
"${CC:-cc}" -std=c11 -Wall -Wextra -Werror -pedantic -g \
    -fsanitize=address,undefined -fno-omit-frame-pointer \
    -I"$project_dir/include" "$project_dir/test/test_tilt_guard.c" \
    "$project_dir/src/tilt_guard.c" -lm -o "$test_binary"
"$test_binary"
# 健康 IMU 优先；初始化/运行失败锁定编码器降级，验证航向连续和非 IMU 故障保护。
"${CC:-cc}" -DMPU6050_ENABLED=1 -DODOMETRY_HEADING_SCALE=1.0f \
    -DGRAY_INVERT_ENABLED=1 -DFLOOR_SEAM_FILTER_ENABLED=0 \
    -DLOST_LINE_STOP_ENABLED=1 -std=c11 -Wall -Wextra -Werror -pedantic -g \
    -fsanitize=address,undefined -fno-omit-frame-pointer \
    -I"$project_dir/include" \
    "$project_dir/test/test_imu_heading.c" "$project_dir/src/odometry.c" \
    "$project_dir/src/tilt_guard.c" \
    "$project_dir/src/gray_sensor.c" "$project_dir/src/line_follow.c" \
    "$project_dir/src/pid.c" "$project_dir/src/braking_planner.c" "$project_dir/src/route.c" \
    "$project_dir/src/speed_control.c" -lm -o "$test_binary"
"$test_binary"
# 以下合成导航用例按无打滑理想几何生成计数，显式使用单位转角比例。
for invert in 1 0; do
    echo "GRAY_INVERT_ENABLED=$invert"
    "${CC:-cc}" -DMPU6050_ENABLED=0 -DODOMETRY_HEADING_SCALE=1.0f -DGRAY_INVERT_ENABLED="$invert" -DFLOOR_SEAM_FILTER_ENABLED=0 -DLOST_LINE_STOP_ENABLED=1 -std=c11 -Wall -Wextra -Werror -pedantic -g \
        -fsanitize=address,undefined -fno-omit-frame-pointer \
        -I"$project_dir/include" \
        "$project_dir/test/test_logic.c" \
        "$project_dir/src/gray_sensor.c" "$project_dir/src/pid.c" \
        "$project_dir/src/motor_command.c" \
        -lm -o "$test_binary"
    "$test_binary"
    "${CC:-cc}" -DMPU6050_ENABLED=0 -DODOMETRY_HEADING_SCALE=1.0f -DGRAY_INVERT_ENABLED="$invert" -DFLOOR_SEAM_FILTER_ENABLED=0 -DLOST_LINE_STOP_ENABLED=1 -std=c11 -Wall -Wextra -Werror -pedantic -g \
        -fsanitize=address,undefined -fno-omit-frame-pointer \
        -I"$project_dir/test/fakes" -I"$project_dir/include" \
        "$project_dir/test/test_encoder_hal.c" "$project_dir/src/encoder.c" \
        -o "$test_binary"
    "$test_binary"
    "${CC:-cc}" -DMPU6050_ENABLED=0 -DODOMETRY_HEADING_SCALE=1.0f -DGRAY_INVERT_ENABLED="$invert" -DFLOOR_SEAM_FILTER_ENABLED=0 -DLOST_LINE_STOP_ENABLED=1 -std=c11 -Wall -Wextra -Werror -pedantic -g \
        -fsanitize=address,undefined -fno-omit-frame-pointer \
        -I"$project_dir/test/fakes" -I"$project_dir/include" \
        "$project_dir/test/test_motor_hal.c" "$project_dir/src/motor.c" \
        "$project_dir/src/motor_command.c" -lm -o "$test_binary"
    "$test_binary"
    "${CC:-cc}" -DMPU6050_ENABLED=0 -DODOMETRY_HEADING_SCALE=1.0f -DGRAY_INVERT_ENABLED="$invert" -DFLOOR_SEAM_FILTER_ENABLED=0 -DLOST_LINE_STOP_ENABLED=1 -std=c11 -Wall -Wextra -Werror -pedantic -g \
        -fsanitize=address,undefined -fno-omit-frame-pointer \
        -I"$project_dir/include" \
        -DENCODER_CONTROL_ENABLED=1 -DPRESET_ROUTE_ENABLED=1 \
        "$project_dir/test/test_configured_start.c" "$project_dir/src/gray_sensor.c" \
        "$project_dir/src/pid.c" "$project_dir/src/odometry.c" \
        "$project_dir/src/braking_planner.c" "$project_dir/src/route.c" "$project_dir/src/route_table.c" \
        "$project_dir/src/speed_control.c" "$project_dir/src/line_follow.c" \
        -lm -o "$test_binary"
    "$test_binary"
    # Historical navigation models keep their original no-slowdown speed assumptions.
    "${CC:-cc}" -DMPU6050_ENABLED=0 -DODOMETRY_HEADING_SCALE=1.0f -DGRAY_INVERT_ENABLED="$invert" -DFLOOR_SEAM_FILTER_ENABLED=0 -DLOST_LINE_STOP_ENABLED=1 -std=c11 -Wall -Wextra -Werror -pedantic -g \
        -fsanitize=address,undefined -fno-omit-frame-pointer \
        -I"$project_dir/include" \
        -DENCODER_CONTROL_ENABLED=1 -DPRESET_ROUTE_ENABLED=1 \
        -DTURN_APPROACH_SLOWDOWN_ENABLED=0 -DTURN_EXIT_ACCEL_ENABLED=0 -DMOTOR_REVERSE_BRAKE_ENABLED=0 \
        "$project_dir/test/test_navigation.c" "$project_dir/src/gray_sensor.c" \
        "$project_dir/src/pid.c" "$project_dir/src/odometry.c" \
        "$project_dir/src/braking_planner.c" "$project_dir/src/route.c" "$project_dir/src/route_table.c" \
        "$project_dir/src/speed_control.c" "$project_dir/src/line_follow.c" \
        -lm -o "$test_binary"
    "$test_binary"
    "${CC:-cc}" -DMPU6050_ENABLED=0 -DODOMETRY_HEADING_SCALE=1.0f -DGRAY_INVERT_ENABLED="$invert" -DFLOOR_SEAM_FILTER_ENABLED=0 -DLOST_LINE_STOP_ENABLED=1 -std=c11 -Wall -Wextra -Werror -pedantic -g \
        -fsanitize=address,undefined -fno-omit-frame-pointer \
        -I"$project_dir/include" \
        -DENCODER_CONTROL_ENABLED=1 -DPRESET_ROUTE_ENABLED=0 \
        "$project_dir/test/test_line_only.c" "$project_dir/src/gray_sensor.c" \
        "$project_dir/src/pid.c" "$project_dir/src/odometry.c" \
        "$project_dir/src/braking_planner.c" "$project_dir/src/route.c" "$project_dir/src/speed_control.c" \
        "$project_dir/src/line_follow.c" -lm -o "$test_binary"
    "$test_binary"
    "${CC:-cc}" -DMPU6050_ENABLED=0 -DODOMETRY_HEADING_SCALE=1.0f -DGRAY_INVERT_ENABLED="$invert" -DFLOOR_SEAM_FILTER_ENABLED=0 -DLOST_LINE_STOP_ENABLED=1 -std=c11 -Wall -Wextra -Werror -pedantic -g \
        -fsanitize=address,undefined -fno-omit-frame-pointer \
        -I"$project_dir/include" -DENCODER_CONTROL_ENABLED=0 -DPRESET_ROUTE_ENABLED=0 \
        "$project_dir/test/test_open_loop.c" "$project_dir/src/gray_sensor.c" \
        "$project_dir/src/pid.c" "$project_dir/src/motor_command.c" \
        "$project_dir/src/braking_planner.c" "$project_dir/src/route.c" "$project_dir/src/line_follow.c" \
        -lm -o "$test_binary"
    "$test_binary"
done

# 单独验证生产头文件丢线开关的两个分支；其余完整回归始终保留停车保护。
for loss_stop in 1 0; do
    for mode in "0 0" "1 0" "1 1"; do
        set -- $mode
        "${CC:-cc}" -DMPU6050_ENABLED=0 -DODOMETRY_HEADING_SCALE=1.0f -std=c11 -Wall -Wextra -Werror -pedantic -g \
            -fsanitize=address,undefined -fno-omit-frame-pointer \
            -I"$project_dir/include" -DFLOOR_SEAM_FILTER_ENABLED=0 -DLOST_LINE_STOP_ENABLED="$loss_stop" \
            -DENCODER_CONTROL_ENABLED="$1" -DPRESET_ROUTE_ENABLED="$2" \
            "$project_dir/test/test_loss_stop.c" "$project_dir/src/gray_sensor.c" \
            "$project_dir/src/pid.c" "$project_dir/src/odometry.c" \
            "$project_dir/src/braking_planner.c" "$project_dir/src/route.c" "$project_dir/src/speed_control.c" \
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
            "${CC:-cc}" -DMPU6050_ENABLED=0 -DODOMETRY_HEADING_SCALE=1.0f -std=c11 -Wall -Wextra -Werror -pedantic -g \
                -fsanitize=address,undefined -fno-omit-frame-pointer \
                -I"$project_dir/include" -DGRAY_INVERT_ENABLED="$invert" \
                -DFLOOR_SEAM_FILTER_ENABLED="$seam_filter" -DLOST_LINE_STOP_ENABLED=1 \
                -DENCODER_CONTROL_ENABLED="$1" -DPRESET_ROUTE_ENABLED="$2" \
                "$project_dir/test/test_floor_seam.c" "$project_dir/src/gray_sensor.c" \
                "$project_dir/src/pid.c" "$project_dir/src/odometry.c" \
                "$project_dir/src/braking_planner.c" "$project_dir/src/route.c" "$project_dir/src/route_table.c" \
                "$project_dir/src/speed_control.c" "$project_dir/src/line_follow.c" \
                "$project_dir/src/motor_command.c" -lm -o "$test_binary"
            "$test_binary"
        done
    done
done

# Synthetic calibration verifies final-leg arming/search; does not certify hardware deceleration.
"${CC:-cc}" -std=c11 -Wall -Wextra -Werror -pedantic -g \
    -fsanitize=address,undefined -fno-omit-frame-pointer \
    -I"$project_dir/include" -DMPU6050_ENABLED=0 -DODOMETRY_HEADING_SCALE=1.0f \
    -DENCODER_CONTROL_ENABLED=1 -DPRESET_ROUTE_ENABLED=1 \
    -DBRAKING_CALIBRATION_CONFIRMED=1 -DBRAKING_SAFE_DECEL_CM_S2=30.0f \
    "$project_dir/test/test_navigation.c" "$project_dir/src/braking_planner.c" \
    "$project_dir/src/route.c" "$project_dir/src/gray_sensor.c" "$project_dir/src/line_follow.c" \
    "$project_dir/src/speed_control.c" "$project_dir/src/odometry.c" "$project_dir/src/pid.c" \
    -lm -o "$test_binary"
"$test_binary"
