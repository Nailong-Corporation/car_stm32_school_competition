#include "main.h"
#include "board_config.h"
#include "control_config.h"
#include "encoder.h"
#include "motor.h"
#include "motion_config.h"
#include "mpu6050.h"
#include <math.h>

volatile uint32_t g_control_request;
volatile ControlDiagnostics g_diagnostics;
static IWDG_HandleTypeDef watchdog;
static GraySensor sensor;
static LineFollow follow;
#if MPU6050_ENABLED
static Mpu6050 imu;
#endif

void route_stop_callback(size_t step_index, void *context)
{
    (void)context;
    /* Invoked once AFTER motor stop. Keep callbacks nonblocking and bounded. */
    ++g_diagnostics.route_stop_events;
    g_diagnostics.route_stopped_step = (uint32_t)step_index;
}

static bool clock_init(void)
{
    RCC_OscInitTypeDef osc = {0};
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    osc.HSIState = RCC_HSI_ON;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
    osc.PLL.PLLMUL = RCC_PLL_MUL16;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) return false;
    RCC_ClkInitTypeDef clocks = {0};
    clocks.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clocks.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clocks.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clocks.APB1CLKDivider = RCC_HCLK_DIV2;
    clocks.APB2CLKDivider = RCC_HCLK_DIV1;
    return HAL_RCC_ClockConfig(&clocks, FLASH_LATENCY_2) == HAL_OK &&
           HAL_RCC_GetSysClockFreq() == BOARD_SYSCLK_HZ;
}

static void sensor_gpio_init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef pins = {0};
    pins.Mode = GPIO_MODE_INPUT;
    pins.Pull = GRAY_INPUT_PULL;
    pins.Pin = GRAY_A_PINS;
    HAL_GPIO_Init(GPIOA, &pins);
    pins.Pin = GRAY_B_PINS;
    HAL_GPIO_Init(GPIOB, &pins);
    pins.Pin = START_STOP_PIN;
    pins.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(START_STOP_PORT, &pins);
}

static uint8_t read_raw(void)
{
    /* One register snapshot per port; channels remain eight separate GPIO inputs. */
    uint32_t a = GPIOA->IDR;
    uint32_t b = GPIOB->IDR;
    return (uint8_t)((a & 0x3FU) | ((b & 0x03U) << 6U));
}

static bool button_press(uint32_t now_ms)
{
    static bool button_initialized, button_candidate, button_stable;
    static uint32_t button_changed_ms;
    bool pressed = HAL_GPIO_ReadPin(START_STOP_PORT, START_STOP_PIN) == GPIO_PIN_RESET;
    if (!button_initialized) {
        button_initialized = true;
        button_candidate = button_stable = pressed;
        button_changed_ms = now_ms;
        return false; /* Held-at-boot button never starts the car. */
    }
    if (pressed != button_candidate) {
        button_candidate = pressed;
        button_changed_ms = now_ms;
    }
    if (button_candidate != button_stable && (uint32_t)(now_ms - button_changed_ms) >= BUTTON_DEBOUNCE_MS) {
        button_stable = button_candidate;
        return button_stable;
    }
    return false;
}

static bool watchdog_init(void)
{
    watchdog.Instance = IWDG;
    watchdog.Init.Prescaler = IWDG_PRESCALER_64;
    watchdog.Init.Reload = 399; /* nominal 640 ms at 40 kHz LSI, not a precise deadline */
    /* A halted debugger must not leave an enabled motor indefinitely. */
    __HAL_DBGMCU_UNFREEZE_IWDG();
    return HAL_IWDG_Init(&watchdog) == HAL_OK;
}

#if MPU6050_ENABLED
static void publish_imu_init(void)
{
    g_diagnostics.imu_enabled = true;
    g_diagnostics.imu_ready = imu.ready;
    g_diagnostics.imu_init_failure_stage = imu.init_failure_stage;
    g_diagnostics.imu_address_7bit = imu.address_7bit;
    g_diagnostics.imu_gyro_z_bias_lsb = imu.gyro_z_bias_lsb;
    g_diagnostics.imu_gyro_z_raw = imu.gyro_z_raw;
    g_diagnostics.imu_gyro_z_calibration_span_lsb = imu.gyro_z_calibration_span_lsb;
    g_diagnostics.imu_gyro_z_calibration_central_span_lsb = imu.gyro_z_calibration_central_span_lsb;
    g_diagnostics.imu_calibration_attempts = imu.calibration_attempts;
    g_diagnostics.imu_init_attempts = imu.init_attempts;
    g_diagnostics.imu_bus_recovery_count = imu.bus_recovery_count;
    g_diagnostics.imu_last_i2c_status = imu.last_i2c_status;
    g_diagnostics.imu_last_i2c_error = imu.last_i2c_error;
    g_diagnostics.imu_last_who_am_i = imu.last_who_am_i;
    g_diagnostics.imu_bus_line_mask = imu.bus_line_mask;
}
#endif

void Error_Handler(void)
{
    motor_stop();
    g_diagnostics.state = LINE_STOPPED;
    g_diagnostics.stop_reason = LINE_STOP_FAULT;
    g_diagnostics.left_command = g_diagnostics.right_command = 0.0f;
    __disable_irq();
    for (;;) { /* Watchdog resets if it has already been enabled. */ }
}

int main(void)
{
    motor_safe_gpio_init();
    if (HAL_Init() != HAL_OK || !clock_init()) Error_Handler();
    sensor_gpio_init();
#if MPU6050_ENABLED
    /* One bounded startup sequence, never while driving. Failed IMU calibration
     * no longer traps START in a retry-only loop: the controller uses encoders. */
    (void)mpu6050_init(&imu);
    publish_imu_init();
#endif
#if ENCODER_CONTROL_ENABLED
    if (!encoder_init()) Error_Handler();
#endif
    if (!motor_init()) Error_Handler();
    gray_sensor_init(&sensor);
    OdometryCalibration calibration = {
        LEFT_WHEEL_DIAMETER_CM, RIGHT_WHEEL_DIAMETER_CM,
        LEFT_COUNTS_PER_WHEEL_REV, RIGHT_COUNTS_PER_WHEEL_REV, WHEEL_TRACK_CM,
        ENCODER_LEFT_SIGN, ENCODER_RIGHT_SIGN, ODOMETRY_CALIBRATION_CONFIRMED != 0
    };
    line_follow_init(&follow, calibration,
#if PRESET_ROUTE_ENABLED
                     route, route_count, route_stop_callback, NULL);
#else
                     NULL, 0, NULL, NULL);
#endif
    g_diagnostics.encoder_control_enabled = ENCODER_CONTROL_ENABLED != 0;
    g_diagnostics.preset_route_enabled = PRESET_ROUTE_ENABLED != 0;
    g_diagnostics.odometry_ready = follow.odometry.ready;
    g_diagnostics.imu_enabled = MPU6050_ENABLED != 0;
    g_diagnostics.pitch_stop_enabled = MPU6050_ENABLED && MPU6050_PITCH_STOP_ENABLED;
#if MPU6050_ENABLED
    publish_imu_init();
#endif
    if (!watchdog_init()) Error_Handler();
    uint32_t last_control_ms = HAL_GetTick();
    bool start_pending = false;
    (void)button_press(last_control_ms);
    for (;;) {
        uint32_t now_ms = HAL_GetTick();
        bool pressed = button_press(now_ms);
        if (pressed) ++g_diagnostics.button_press_count;
        uint32_t request = g_control_request;
        g_control_request = 0;
        if (pressed && request != 2U)
            request = (follow.state == LINE_IDLE || follow.state == LINE_STOPPED) ? 1U : 2U;
        if (request == 2U) {
            line_follow_stop(&follow, LINE_STOP_REQUEST);
            motor_stop();
            start_pending = false;
        } else if (request == 1U) start_pending = true;

        uint32_t interval = (uint32_t)(now_ms - last_control_ms);
        if (interval >= CONTROL_PERIOD_MS) {
            last_control_ms = now_ms;
            EncoderCounts counts = {0};
#if ENCODER_CONTROL_ENABLED
            if (!encoder_read(&counts)) Error_Handler();
#endif
            GrayObservation observation = gray_sensor_update(&sensor, read_raw(),
                line_follow_have_recent_position(&follow, now_ms), follow.reliable_position);
            bool imu_valid = false;
            float imu_heading_rad = 0.0f;
#if MPU6050_ENABLED
            if (imu.ready && follow.heading_source == HEADING_IMU)
                imu_valid = mpu6050_update(&imu, (float)interval / 1000.0f, &imu_heading_rad);
            g_diagnostics.imu_ready = imu_valid;
            g_diagnostics.imu_gyro_z_raw = imu.gyro_z_raw;
#if MPU6050_PITCH_STOP_ENABLED
            g_diagnostics.pitch_deg = imu.pitch_guard.pitch_deg;
            g_diagnostics.pitch_over_threshold_ms = imu.pitch_guard.above_threshold_s * 1000.0f;
            g_diagnostics.pitch_stop_latched = imu.pitch_guard.tripped;
#endif
#endif
#if MPU6050_ENABLED && MPU6050_PITCH_STOP_ENABLED
            if (start_pending && imu_valid && follow.state == LINE_STOPPED &&
                follow.stop_reason == LINE_STOP_TILT && tilt_guard_acknowledge(&imu.pitch_guard)) {
                line_follow_acknowledge_tilt(&follow);
                g_diagnostics.pitch_stop_latched = imu.pitch_guard.tripped;
            }
#endif
            if (start_pending) {
                ++g_diagnostics.start_request_count;
                g_diagnostics.start_raw_mask = observation.raw_mask;
                g_diagnostics.start_line_mask = observation.line_mask;
            }
            line_follow_update_heading(&follow, observation, counts, now_ms,
                               (float)interval / 1000.0f, start_pending,
                               imu_valid, imu_heading_rad);
            g_diagnostics.heading_source = follow.heading_source;
            g_diagnostics.heading_fallback_reason = follow.heading_fallback_reason;
            g_diagnostics.heading_fallback_ms = follow.heading_fallback_ms;
            g_diagnostics.pitch_monitor_available = MPU6050_ENABLED && MPU6050_PITCH_STOP_ENABLED &&
                imu_valid && follow.heading_source == HEADING_IMU;
#if MPU6050_ENABLED && MPU6050_PITCH_STOP_ENABLED
            /* Tilt is a latched safety stop even in IDLE or a route auto-resume.
             * Run before motor_apply so no motor command from this cycle reaches STBY. */
            if (imu.pitch_guard.tripped) line_follow_stop(&follow, LINE_STOP_TILT);
#endif
            start_pending = false; /* One-shot; failed start is never queued for later. */
            uint32_t duration = (uint32_t)(HAL_GetTick() - now_ms);
            if (duration >= CONTROL_PERIOD_MS) line_follow_stop(&follow, LINE_STOP_FAULT);
            motor_apply(follow.command);
#if PRESET_ROUTE_ENABLED
            if (follow.stop_reason == LINE_STOP_ROUTE_COMPLETE) {
                motor_stop();
                route_dispatch_stop_callback(&follow.route);
            }
#endif
            duration = (uint32_t)(HAL_GetTick() - now_ms);
            if (duration >= CONTROL_PERIOD_MS) {
                line_follow_stop(&follow, LINE_STOP_FAULT);
                motor_stop();
            }
            ++g_diagnostics.control_count;
            g_diagnostics.last_sample_ms = now_ms;
            g_diagnostics.task_duration_ms = duration;
            g_diagnostics.interval_ms = interval;
            g_diagnostics.raw_mask = observation.raw_mask;
            g_diagnostics.line_mask = observation.line_mask;
            g_diagnostics.prefilter_mask = observation.prefilter_mask;
            g_diagnostics.seam_mask = observation.seam_mask;
            g_diagnostics.selected_mask = observation.selected_mask;
            g_diagnostics.observation = observation.kind;
            g_diagnostics.position = observation.position;
            g_diagnostics.state = follow.state;
            g_diagnostics.stop_reason = follow.stop_reason;
            g_diagnostics.start_result = follow.start_result;
            g_diagnostics.valid_line_samples = follow.valid_count;
            g_diagnostics.left_command = follow.command.left;
            g_diagnostics.right_command = follow.command.right;
            g_diagnostics.left_encoder = counts.left;
            g_diagnostics.right_encoder = counts.right;
            g_diagnostics.distance_cm = follow.odometry.distance_cm;
            g_diagnostics.heading_rad = follow.odometry.heading_rad;
            g_diagnostics.left_speed_cm_s = follow.odometry.left_speed_cm_s;
            g_diagnostics.right_speed_cm_s = follow.odometry.right_speed_cm_s;
            g_diagnostics.left_target_cm_s = follow.left_target_cm_s;
            g_diagnostics.right_target_cm_s = follow.right_target_cm_s;
            g_diagnostics.route_state = follow.route.state;
            g_diagnostics.route_failure = follow.route.failure;
            g_diagnostics.route_progress_cm = follow.odometry.distance_cm - follow.route.origin_cm;
            g_diagnostics.route_elapsed_ms = (uint32_t)(now_ms - follow.route.segment_ms);
            g_diagnostics.route_repeat_count = follow.route.repeat_count;
            g_diagnostics.route_completed_laps = follow.route.completed_laps;
            g_diagnostics.route_turn_capturing = follow.route.turn_capturing;
            g_diagnostics.route_turn_forced = false;
            g_diagnostics.route_candidate_samples = follow.route.candidate_count;
            g_diagnostics.route_candidate_kind=follow.route.candidate_kind;
            g_diagnostics.route_last_confirmed_kind=follow.route.last_confirmed_kind;
            g_diagnostics.route_candidate_reject=follow.route.candidate_reject;
            g_diagnostics.route_candidate_sides=follow.route.candidate_sides;
            g_diagnostics.route_candidate_restarts=follow.route.candidate_restarts;
            g_diagnostics.route_smooth_edge_rejections=follow.route.smooth_edge_rejections;
            g_diagnostics.route_turn_rad = follow.odometry.heading_rad - follow.route.entry_heading;
            g_diagnostics.route_index = (uint32_t)follow.route.index;
            g_diagnostics.stop_snapshot = follow.stop_snapshot;
            g_diagnostics.straight_entry_aligned = follow.straight_entry_aligned;
            g_diagnostics.straight_entry_handoff = follow.straight_entry_handoff;
            g_diagnostics.turn_exit[0] = follow.turn_exit[0];
            g_diagnostics.turn_exit[1] = follow.turn_exit[1];
            g_diagnostics.exit_anchor_valid = follow.exit_anchor_valid;
            g_diagnostics.exit_recovery_active = follow.exit_recovery_active;
            g_diagnostics.exit_lateral_error_cm = follow.exit_lateral_error_cm;
            g_diagnostics.auto_resume_snapshot = follow.auto_resume_snapshot;
            g_diagnostics.auto_resume_count = follow.auto_resume_count;
            g_diagnostics.auto_resume_last_ms = follow.auto_resume_last_ms;
            g_diagnostics.route_recovery_window_active = false;
            g_diagnostics.route_recovery_window_progress_cm = 0.0f;
            g_diagnostics.route_recovery_window_limit_cm = 0.0f;
            g_diagnostics.route_junctions_seen = follow.route.junctions_seen;
            g_diagnostics.route_final_segment_active = follow.route.final_segment_active;
            g_diagnostics.route_final_segment_progress_cm = follow.odometry.distance_cm-follow.route.final_segment_origin_cm;
            if (HAL_IWDG_Refresh(&watchdog) != HAL_OK) Error_Handler();
        }
        __WFI(); /* SysTick wakes every ms; no blocking delay or serial output. */
    }
}
