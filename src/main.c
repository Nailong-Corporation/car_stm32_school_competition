#include "main.h"
#include "board_config.h"
#include "control_config.h"
#include "encoder.h"
#include "motor.h"
#include "motion_config.h"

volatile uint32_t g_control_request;
volatile ControlDiagnostics g_diagnostics;
static IWDG_HandleTypeDef watchdog;
static GraySensor sensor;
static LineFollow follow;

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
    static bool initialized, candidate, stable;
    static uint32_t changed_ms;
    bool pressed = HAL_GPIO_ReadPin(START_STOP_PORT, START_STOP_PIN) == GPIO_PIN_RESET;
    if (!initialized) {
        initialized = true;
        candidate = stable = pressed;
        changed_ms = now_ms;
        return false; /* Held-at-boot button never starts the car. */
    }
    if (pressed != candidate) {
        candidate = pressed;
        changed_ms = now_ms;
    }
    if (candidate != stable && (uint32_t)(now_ms - changed_ms) >= BUTTON_DEBOUNCE_MS) {
        stable = candidate;
        return stable;
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
            if (start_pending) {
                ++g_diagnostics.start_request_count;
                g_diagnostics.start_raw_mask = observation.raw_mask;
                g_diagnostics.start_line_mask = observation.line_mask;
            }
            line_follow_update(&follow, observation, counts, now_ms,
                               (float)interval / 1000.0f, start_pending);
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
            g_diagnostics.route_index = (uint32_t)follow.route.index;
            if (HAL_IWDG_Refresh(&watchdog) != HAL_OK) Error_Handler();
        }
        __WFI(); /* SysTick wakes every ms; no blocking delay or serial output. */
    }
}
