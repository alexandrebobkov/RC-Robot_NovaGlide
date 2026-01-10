# NovaGlide Code Structure

This document contains the complete source code for the NovaGlide project.

## Table of Contents
  - [main/CMakeLists.txt](#main-cmakelists-txt)
  - [main/ESP-IDF_NovaGlide.c](#main-esp-idf-novaglide-c)
  - [main/control_task.c](#main-control-task-c)
  - [main/control_task.h](#main-control-task-h)
  - [main/dashboard.c](#main-dashboard-c)
  - [main/dashboard.h](#main-dashboard-h)
  - [main/scheduler.c](#main-scheduler-c)
  - [main/scheduler.h](#main-scheduler-h)
  - [main/system_init.c](#main-system-init-c)
  - [main/system_init.h](#main-system-init-h)
  - [subsystems/motors/CMakeLists.txt](#subsystems-motors-cmakelists-txt)
  - [subsystems/motors/motors.c](#subsystems-motors-motors-c)
  - [subsystems/motors/motors.h](#subsystems-motors-motors-h)
  - [subsystems/i2c_bus/CMakeLists.txt](#subsystems-i2c-bus-cmakelists-txt)
  - [subsystems/i2c_bus/i2c_bus.c](#subsystems-i2c-bus-i2c-bus-c)
  - [subsystems/i2c_bus/i2c_bus.h](#subsystems-i2c-bus-i2c-bus-h)
  - [subsystems/sensors/CMakeLists.txt](#subsystems-sensors-cmakelists-txt)
  - [subsystems/sensors/ina219_sensor.c](#subsystems-sensors-ina219-sensor-c)
  - [subsystems/sensors/ina219_sensor.h](#subsystems-sensors-ina219-sensor-h)
  - [subsystems/sensors/temp_sensor.c](#subsystems-sensors-temp-sensor-c)
  - [subsystems/sensors/temp_sensor.h](#subsystems-sensors-temp-sensor-h)
  - [subsystems/sensors/ultrasonic_sensor.c](#subsystems-sensors-ultrasonic-sensor-c)
  - [subsystems/sensors/ultrasonic_sensor.h](#subsystems-sensors-ultrasonic-sensor-h)
  - [subsystems/adc/CMakeLists.txt](#subsystems-adc-cmakelists-txt)
  - [subsystems/adc/adc.c](#subsystems-adc-adc-c)
  - [subsystems/adc/adc.h](#subsystems-adc-adc-h)
  - [subsystems/controls/CMakeLists.txt](#subsystems-controls-cmakelists-txt)
  - [subsystems/controls/joystick.c](#subsystems-controls-joystick-c)
  - [subsystems/controls/joystick.h](#subsystems-controls-joystick-h)
  - [subsystems/ui/CMakeLists.txt](#subsystems-ui-cmakelists-txt)
  - [subsystems/ui/ui.c](#subsystems-ui-ui-c)
  - [subsystems/ui/ui.h](#subsystems-ui-ui-h)
  - [subsystems/connectivity/CMakeLists.txt](#subsystems-connectivity-cmakelists-txt)
  - [subsystems/connectivity/mqtt_sys/CMakeLists.txt](#subsystems-connectivity-mqtt-sys-cmakelists-txt)
  - [subsystems/connectivity/mqtt_sys/mqtt_sys.c](#subsystems-connectivity-mqtt-sys-mqtt-sys-c)
  - [subsystems/connectivity/mqtt_sys/mqtt_sys.h](#subsystems-connectivity-mqtt-sys-mqtt-sys-h)
  - [subsystems/connectivity/wifi_sys/CMakeLists.txt](#subsystems-connectivity-wifi-sys-cmakelists-txt)
  - [subsystems/connectivity/wifi_sys/wifi_sys.c](#subsystems-connectivity-wifi-sys-wifi-sys-c)
  - [subsystems/connectivity/wifi_sys/wifi_sys.h](#subsystems-connectivity-wifi-sys-wifi-sys-h)
  - [subsystems/connectivity/espnow_sys/CMakeLists.txt](#subsystems-connectivity-espnow-sys-cmakelists-txt)
  - [subsystems/connectivity/espnow_sys/espnow_sys.c](#subsystems-connectivity-espnow-sys-espnow-sys-c)
  - [subsystems/connectivity/espnow_sys/espnow_sys.h](#subsystems-connectivity-espnow-sys-espnow-sys-h)


## main/CMakeLists.txt <a name="main-cmakelists-txt"></a>

```c
idf_component_register(
    SRCS "ESP-IDF_NovaGlide.c"
         "system_init.c"
         "scheduler.c"
         "control_task.c"
         "dashboard.c"
    INCLUDE_DIRS "."
    REQUIRES
        esp_wifi
        esp_netif
        nvs_flash
        motors
        adc
        wifi_sys
        mqtt_sys
        espnow_sys
        sensors
        controls
        ui
        i2c_bus)

```

## main/ESP-IDF_NovaGlide.c <a name="main-esp-idf-novaglide-c"></a>

```c
#include "esp_log_level.h"
#include "espnow_sys.h"
#include "esp_log.h"
#include "soc/gpio_num.h"
#include "system_init.h"
#include "scheduler.h"
#include "control_task.h"

// Subsystems
#include "motors.h"
#include "adc.h"
#include "temp_sensor.h"
#include "ina219_sensor.h"
#include "ultrasonic_sensor.h"
#include "wifi_sys.h"
#include "espnow_sys.h"
#include "mqtt_sys.h"
#include "ui.h"
#include "dashboard.h"
#include "i2c_bus.h"

// Telemetry bridge context
typedef struct {
    temp_sensor_system_t *temp;
    ina219_system_t *ina;
    motor_system_t *motors;
    mqtt_system_t *mqtt;
    ultrasonic_system_t *ultrasonic;
} telemetry_context_t;

// Task to bridge sensor data to MQTT
static void telemetry_bridge_task(void *arg) {
    telemetry_context_t *ctx = (telemetry_context_t *)arg;

    while (1) {
        // Update MQTT with latest sensor readings
        mqtt_update_temp(ctx->mqtt, ctx->temp->temperature);
        mqtt_update_battery(ctx->mqtt, ctx->ina->bus_voltage);
        mqtt_update_current(ctx->mqtt, ctx->ina->current * 1000.0f);
        mqtt_update_power(ctx->mqtt, ctx->ina->power);
        mqtt_update_pwm(ctx->mqtt, ctx->motors->left_pwm, ctx->motors->right_pwm);
        mqtt_update_proximity(ctx->mqtt, ctx->ultrasonic->distance_cm / 10.0f);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Display task to show current joystick values
static void display_joystick_task(void *arg) {
    espnow_system_t *espnow = (espnow_system_t *)arg;

    while (1) {
        ESP_LOGI("DISPLAY", "╔════════════════════════════════════╗");
        ESP_LOGI("DISPLAY", "║   CURRENT JOYSTICK VALUES          ║");
        ESP_LOGI("DISPLAY", "║   X-axis: %-8d                     ║", espnow->last_data.x_axis);
        ESP_LOGI("DISPLAY", "║   Y-axis: %-8d                ║", espnow->last_data.y_axis);
        ESP_LOGI("DISPLAY", "╚════════════════════════════════════╝");

        vTaskDelay(pdMS_TO_TICKS(2000));  // Display every 2 seconds
    }
}

void app_main(void)
{
    // System-level initialization
    system_init();

    // Set log levels to WARNING or ERROR only
    esp_log_level_set("*", ESP_LOG_ERROR);
    esp_log_level_set("DASHBOARD", ESP_LOG_INFO);  // Allow dashboard
    esp_log_level_set("i2c.master", ESP_LOG_NONE);  // Suppress I2C NACK errors (normal when ultrasonic is too close)

    // Subsystem instances
    static motor_system_t motors;
    static adc_system_t adc;
    static temp_sensor_system_t temp;
    static ina219_system_t ina;
    static ultrasonic_system_t ultra;
    static mqtt_system_t mqtt;
    static espnow_system_t espnow;
    static ui_system_t ui;

    // Initialize WiFi first (needed for ESP-NOW and MQTT)
    wifi_system_init();

    // Initialize I2C bus FIRST
    ESP_ERROR_CHECK(i2c_bus_init());
    i2c_bus_scan();

    // Initialize all subsystems
    motor_system_init(&motors);
    adc_system_init(&adc);
    temp_sensor_system_init(&temp);
    ina219_system_init(&ina);
    ultrasonic_system_init(&ultra);
    espnow_system_init(&espnow);
    mqtt_system_init(&mqtt);
    ui_system_init(&ui);

    // Start display task (optional - uncomment if needed)
    // xTaskCreate(display_joystick_task, "display", 2048, &espnow, 4, NULL);

    // Start control task (joystick -> motors)
    control_task_start(&motors, &espnow);

    // Start dashboard display
    static dashboard_context_t dash_ctx = {
        .motors = &motors,
        .espnow = &espnow,
        .temp = &temp,
        .ina = &ina,
        .ultrasonic = &ultra
    };
    dashboard_task_start(&dash_ctx);

    // Create data bridge task for MQTT telemetry
    static telemetry_context_t telem_ctx = {
        .temp = &temp,
        .ina = &ina,
        .motors = &motors,
        .mqtt = &mqtt,
        .ultrasonic = &ultra
    };
    xTaskCreate(telemetry_bridge_task, "telemetry", 4096, &telem_ctx, 5, NULL);

    // Scheduler wiring
    static scheduler_t sched = {
        .motors = &motors,
        .adc = &adc,
        .temp = &temp,
        .ina = &ina,
        .ultra = &ultra,
        .mqtt = &mqtt,
        .espnow = &espnow,
        .ui = &ui
    };
    scheduler_init(&sched);
    scheduler_start(&sched);
}

```

## main/control_task.c <a name="main-control-task-c"></a>

```c
#include "control_task.h"
#include "joystick.h"
#include "esp_log.h"

static const char *TAG = "CONTROL";

typedef struct {
    motor_system_t *motors;
    espnow_system_t *espnow;
} control_context_t;

static joystick_hal_t js;

static void control_task(void *arg) {
    control_context_t *ctx = (control_context_t *)arg;
    int pwm_left = 0;
    int pwm_right = 0;

    ESP_LOGI(TAG, "Control task started");

    // Initialize joystick HAL
    joystick_hal_init(&js);

    while (1) {
        // 1. Read raw joystick values from ESP-NOW
        int32_t rc_x = ctx->espnow->last_data.x_axis;
        int32_t rc_y = ctx->espnow->last_data.y_axis;

        // 2. Update joystick HAL (auto-calibration + normalization)
        js.update(&js, rc_x, rc_y);

        // 3. Mix normalized joystick values into motor PWM
        joystick_mix(js.norm_y, js.norm_x, &pwm_left, &pwm_right);

        // 4. Apply PWM to motors
        update_motors_pwm(ctx->motors, pwm_left, pwm_right);

        // 5. Debug output
        ESP_LOGI(TAG,
                 "RC raw=(%ld,%ld) norm=(%.2f,%.2f) PWM(L,R)=(%d,%d)",
                 (long)rc_x, (long)rc_y,
                 js.norm_x, js.norm_y,
                 pwm_left, pwm_right);

        vTaskDelay(pdMS_TO_TICKS(50));  // 20Hz control loop
    }
}

void control_task_start(motor_system_t *motors, espnow_system_t *espnow) {
    static control_context_t ctx;
    ctx.motors = motors;
    ctx.espnow = espnow;

    xTaskCreate(control_task, "control", 4096, &ctx, 15, NULL);
    ESP_LOGI(TAG, "Control task created");
}

```

## main/control_task.h <a name="main-control-task-h"></a>

```c
#ifndef CONTROL_TASK_H
#define CONTROL_TASK_H

#include "motors.h"
#include "espnow_sys.h"

void control_task_start(motor_system_t *motors, espnow_system_t *espnow);

#endif

```

## main/dashboard.c <a name="main-dashboard-c"></a>

```c
// dashboard.c
#include "dashboard.h"
#include "ultrasonic_sensor.h"
#include <stdio.h>
#include "esp_log.h"

// ANSI escape codes for terminal control
#define CLEAR_SCREEN "\033[2J"
#define CURSOR_HOME "\033[H"
#define CURSOR_HIDE "\033[?25l"
#define CURSOR_SHOW "\033[?25h"
#define COLOR_RESET "\033[0m"
#define COLOR_GREEN "\033[32m"
#define COLOR_YELLOW "\033[33m"
#define COLOR_CYAN "\033[36m"
#define COLOR_RED "\033[31m"
#define COLOR_BLUE "\033[34m"
#define COLOR_MAGENTA "\033[35m"
#define BOLD "\033[1m"

static void draw_box(const char *title) {
    printf("╔════════════════════════════════════════════════════════════╗\n");
    printf("║ " COLOR_CYAN BOLD "%-58s" COLOR_RESET " ║\n", title);
    printf("╠════════════════════════════════════════════════════════════╣\n");
}

static void draw_line(const char *label, const char *value, const char *color) {
    printf("║ " BOLD "%-20s" COLOR_RESET " : %s%-33s" COLOR_RESET "   ║\n", label, color, value);
}

static void draw_separator() {
    printf("╠════════════════════════════════════════════════════════════╣\n");
}

static void draw_bottom() {
    printf("╚════════════════════════════════════════════════════════════╝\n");
}

static void dashboard_task(void *arg) {
    dashboard_context_t *ctx = (dashboard_context_t *)arg;
    char buffer[50];

    // Hide cursor for cleaner display
    printf(CURSOR_HIDE);

    // Print help text ONCE before the loop
    printf("\n" COLOR_YELLOW "Dashboard Mode: Press Ctrl+] to exit monitor" COLOR_RESET "\n");
    vTaskDelay(pdMS_TO_TICKS(2000));  // Give user time to read


    while (1) {
        // Clear screen and move cursor to home
        printf(CLEAR_SCREEN CURSOR_HOME);

        // ========== HEADER ==========
        draw_box("ESP32-C3 ROBOT CONTROL DASHBOARD");

        // ========== ESP-NOW SECTION ==========
        snprintf(buffer, sizeof(buffer), "%d", ctx->espnow->last_data.x_axis);
        draw_line("Joystick X", buffer, COLOR_GREEN);

        snprintf(buffer, sizeof(buffer), "%d", ctx->espnow->last_data.y_axis);
        draw_line("Joystick Y", buffer, COLOR_GREEN);

        draw_separator();

        // ========== MOTORS SECTION ==========
        snprintf(buffer, sizeof(buffer), "%d", ctx->motors->left_pwm);
        draw_line("PWM Left", buffer, COLOR_YELLOW);

        snprintf(buffer, sizeof(buffer), "%d", ctx->motors->right_pwm);
        draw_line("PWM Right", buffer, COLOR_YELLOW);

        draw_separator();

        snprintf(buffer, sizeof(buffer), "%d", ctx->motors->motor1_rpm_pcm);
        draw_line("Motor 1 (L-Fwd)", buffer,
                  ctx->motors->motor1_rpm_pcm > 0 ? COLOR_GREEN : COLOR_RESET);

        snprintf(buffer, sizeof(buffer), "%d", ctx->motors->motor2_rpm_pcm);
        draw_line("Motor 2 (R-Fwd)", buffer,
                  ctx->motors->motor2_rpm_pcm > 0 ? COLOR_GREEN : COLOR_RESET);

        snprintf(buffer, sizeof(buffer), "%d", ctx->motors->motor3_rpm_pcm);
        draw_line("Motor 3 (L-Rev)", buffer,
                  ctx->motors->motor3_rpm_pcm > 0 ? COLOR_RED : COLOR_RESET);

        snprintf(buffer, sizeof(buffer), "%d", ctx->motors->motor4_rpm_pcm);
        draw_line("Motor 4 (R-Rev)", buffer,
                  ctx->motors->motor4_rpm_pcm > 0 ? COLOR_RED : COLOR_RESET);

        draw_separator();

        // ========== SENSORS SECTION ==========
        snprintf(buffer, sizeof(buffer), "%.2f °C", ctx->temp->temperature);
        draw_line("Temperature", buffer, COLOR_CYAN);

        snprintf(buffer, sizeof(buffer), "%.2f V", ctx->ina->bus_voltage);
        draw_line("Battery Voltage", buffer,
                  ctx->ina->bus_voltage > 7.0 ? COLOR_GREEN : COLOR_RED);

        snprintf(buffer, sizeof(buffer), "%.2f mA", ctx->ina->current * 1000.0f);
        draw_line("Current", buffer, COLOR_MAGENTA);

        snprintf(buffer, sizeof(buffer), "%.2f mW", ctx->ina->power * 1000.0f);
        draw_line("Power", buffer, COLOR_BLUE);

        draw_separator();

        snprintf(buffer, sizeof(buffer), "%.2f cm", ctx->ultrasonic->distance_cm / 10.0f);
        draw_line("Distance", buffer, COLOR_BLUE);

        draw_bottom();

        // Status bar at bottom
        //printf("\n" COLOR_YELLOW "Press Ctrl+] to exit monitor" COLOR_RESET "\n");

        // Update every 200ms for smooth display
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // Show cursor on exit (won't actually run due to while loop)
    printf(CURSOR_SHOW);
}

void dashboard_task_start(dashboard_context_t *ctx) {
    xTaskCreate(dashboard_task, "dashboard", 4096, ctx, 6, NULL);
    ESP_LOGI("DASHBOARD", "Dashboard display started");
}

```

## main/dashboard.h <a name="main-dashboard-h"></a>

```c
// dashboard.h
#ifndef DASHBOARD_H
#define DASHBOARD_H

#include "freertos/FreeRTOS.h"
#include "motors.h"
#include "espnow_sys.h"
#include "temp_sensor.h"
#include "ina219_sensor.h"
#include "ultrasonic_sensor.h"

typedef struct {
    motor_system_t *motors;
    espnow_system_t *espnow;
    temp_sensor_system_t *temp;
    ina219_system_t *ina;
    ultrasonic_system_t *ultrasonic;
} dashboard_context_t;

void dashboard_task_start(dashboard_context_t *ctx);

#endif

```

## main/scheduler.c <a name="main-scheduler-c"></a>

```c
#include "scheduler.h"
#include "esp_log.h"

static const char *TAG = "SCHEDULER";

static void scheduler_task(void *arg) {
    scheduler_t *sched = (scheduler_t *)arg;

    ESP_LOGI(TAG, "Scheduler task started");

    while (1) {
        TickType_t now = xTaskGetTickCount();

        // Update all subsystems
        if (sched->motors && sched->motors->update) {
            sched->motors->update(sched->motors, now);
        }
        if (sched->adc && sched->adc->update) {
            sched->adc->update(sched->adc, now);
        }
        if (sched->temp && sched->temp->update) {
            sched->temp->update(sched->temp, now);
        }
        if (sched->ina && sched->ina->update) {
            sched->ina->update(sched->ina, now);
        }
        if (sched->ultra && sched->ultra->update) {
            sched->ultra->update(sched->ultra, now);
        }
        if (sched->mqtt && sched->mqtt->update) {
            sched->mqtt->update(sched->mqtt, now);
        }
        if (sched->espnow && sched->espnow->update) {
            sched->espnow->update(sched->espnow, now);
        }
        if (sched->ui && sched->ui->update) {
            sched->ui->update(sched->ui, now);
        }

        vTaskDelay(pdMS_TO_TICKS(250));  // 20Hz update rate
    }
}

void scheduler_init(scheduler_t *sched) {
    ESP_LOGI(TAG, "Scheduler initialized");
}

void scheduler_start(scheduler_t *sched) {
    xTaskCreate(scheduler_task, "scheduler", 8192, sched, 10, NULL);
    ESP_LOGI(TAG, "Scheduler started");
}

```

## main/scheduler.h <a name="main-scheduler-h"></a>

```c
#ifndef SCHEDULER_H
#define SCHEDULER_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "motors.h"
#include "adc.h"
#include "temp_sensor.h"
#include "ina219_sensor.h"
#include "ultrasonic_sensor.h"
#include "mqtt_sys.h"
#include "espnow_sys.h"
#include "ui.h"

typedef struct {
    motor_system_t *motors;
    adc_system_t *adc;
    temp_sensor_system_t *temp;
    ina219_system_t *ina;
    ultrasonic_system_t *ultra;
    mqtt_system_t *mqtt;
    espnow_system_t *espnow;
    ui_system_t *ui;
} scheduler_t;

void scheduler_init(scheduler_t *sched);
void scheduler_start(scheduler_t *sched);

#endif

```

## main/system_init.c <a name="main-system-init-c"></a>

```c
#include "system_init.h"
#include "nvs_flash.h"
#include "esp_log.h"

static const char *TAG = "SYSTEM_INIT";

void system_init(void) {
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "System initialization complete");
}

```

## main/system_init.h <a name="main-system-init-h"></a>

```c
#ifndef SYSTEM_INIT_H
#define SYSTEM_INIT_H

void system_init(void);

#endif

```

## subsystems/motors/CMakeLists.txt <a name="subsystems-motors-cmakelists-txt"></a>

```c
idf_component_register(
    SRCS "motors.c"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_ledc esp_driver_gpio
)

```

## subsystems/motors/motors.c <a name="subsystems-motors-motors-c"></a>

```c
/**
* @file motors.c
* @brief Differential Drive Motor Control System for ESP32-C3 Robot
*
* This module implements a 4-channel PWM motor control system using ESP32's LEDC peripheral.
* It controls two DC motors (left and right) with bidirectional control (forward/reverse).
*
* Hardware Architecture:
* - Each motor has 2 PWM channels: one for forward, one for reverse
* - Motor 1 (M1): Left motor forward   - GPIO pin MTR_FRONT_LEFT_IO
* - Motor 2 (M2): Right motor forward  - GPIO pin MTR_FRONT_RIGHT_IO
* - Motor 3 (M3): Left motor reverse   - GPIO pin MTR_FRONT_LEFT_REV_IO
* - Motor 4 (M4): Right motor reverse  - GPIO pin MTR_FRONT_RIGHT_REV_IO
*
* PWM Configuration:
* - Resolution: 13-bit (0-8191 duty cycle range)
* - Frequency: Typically 1-20 kHz (defined by MTR_FREQUENCY)
* - Control: LEDC (LED Control) peripheral used for motor PWM generation
*
* Control Strategy:
* Input: Signed PWM values for left/right motors (-8191 to +8191)
* - Positive value = forward direction
* - Negative value = reverse direction
* - Magnitude = speed (0 = stop, 8191 = full speed)
*
* @author Alexander Bobkov
* @date January 2026
*/

#include "motors.h"
#include "esp_log.h"
#include "esp_err.h"

static const char *TAG = "MOTORS";

/**
 * @brief Initialize LEDC (LED Controller) for PWM motor control
 *
 * The ESP32 LEDC peripheral is designed for LED dimming but works perfectly for motor PWM.
 * This function configures 4 independent PWM channels with 4 timers:
 *
 * Timer/Channel Architecture:
 * - Timer 0 → Channel 0 → Motor 1 (Left Forward)
 * - Timer 1 → Channel 1 → Motor 2 (Right Forward)
 * - Timer 2 → Channel 2 → Motor 3 (Left Reverse)
 * - Timer 3 → Channel 3 → Motor 4 (Right Reverse)
 *
 * Each timer operates independently, allowing:
 * - Different frequencies per motor (if needed)
 * - Phase-shifted PWM (reduces power supply noise)
 * - Independent duty cycle control
 *
 * Configuration Details:
 * - Speed Mode: Low-speed mode (sufficient for motor control)
 * - Duty Resolution: 13-bit = 8192 steps (0-8191)
 * - Clock Source: APB clock (80 MHz on ESP32-C3)
 * - Interrupt: Disabled (motors don't need interrupt-driven updates)
 * - H-Point: 0 (PWM starts at beginning of period)
 *
 * @note Initial duty cycle is 0 (motors stopped)
 */

// Initialize LEDC timers and channels
static void ledc_init(void) {
    // Motor 1 - Left Forward
    ledc_timer_config_t timer1 = {
        .speed_mode = MTR_MODE,
        .duty_resolution = MTR_DUTY_RES,
        .timer_num = MTR_FRONT_LEFT_TMR,
        .freq_hz = MTR_FREQUENCY,
        .clk_cfg = LEDC_APB_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer1));

    ledc_channel_config_t channel1 = {
        .speed_mode = MTR_MODE,
        .channel = MTR_FRONT_LEFT,
        .timer_sel = MTR_FRONT_LEFT_TMR,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = MTR_FRONT_LEFT_IO,
        .duty = 0,
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel1));

    // Motor 2 - Right Forward
    ledc_timer_config_t timer2 = {
        .speed_mode = MTR_MODE,
        .duty_resolution = MTR_DUTY_RES,
        .timer_num = MTR_FRONT_RIGHT_TMR,
        .freq_hz = MTR_FREQUENCY,
        .clk_cfg = LEDC_APB_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer2));

    ledc_channel_config_t channel2 = {
        .speed_mode = MTR_MODE,
        .channel = MTR_FRONT_RIGHT,
        .timer_sel = MTR_FRONT_RIGHT_TMR,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = MTR_FRONT_RIGHT_IO,
        .duty = 0,
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel2));

    // Motor 3 - Left Reverse
    ledc_timer_config_t timer3 = {
        .speed_mode = MTR_MODE,
        .duty_resolution = MTR_DUTY_RES,
        .timer_num = MTR_FRONT_LEFT_REV_TMR,
        .freq_hz = MTR_FREQUENCY,
        .clk_cfg = LEDC_APB_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer3));

    ledc_channel_config_t channel3 = {
        .speed_mode = MTR_MODE,
        .channel = MTR_FRONT_LEFT_REV,
        .timer_sel = MTR_FRONT_LEFT_REV_TMR,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = MTR_FRONT_LEFT_REV_IO,
        .duty = 0,
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel3));

    // Motor 4 - Right Reverse
    ledc_timer_config_t timer4 = {
        .speed_mode = MTR_MODE,
        .duty_resolution = MTR_DUTY_RES,
        .timer_num = MTR_FRONT_RIGHT_REV_TMR,
        .freq_hz = MTR_FREQUENCY,
        .clk_cfg = LEDC_APB_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer4));

    ledc_channel_config_t channel4 = {
        .speed_mode = MTR_MODE,
        .channel = MTR_FRONT_RIGHT_REV,
        .timer_sel = MTR_FRONT_RIGHT_REV_TMR,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = MTR_FRONT_RIGHT_REV_IO,
        .duty = 0,
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel4));

    ESP_LOGI(TAG, "LEDC initialized for all 4 motors");
}

// Your proven motor update logic
void update_motors_pwm(motor_system_t *sys, int pwm_motor_1, int pwm_motor_2) {
    /* UPDATED MOTOR LOGIC */
    if (pwm_motor_1 >= 0 && pwm_motor_2 >= 0) {
        sys->motor1_rpm_pcm = pwm_motor_1;
        sys->motor2_rpm_pcm = pwm_motor_2;
        sys->motor3_rpm_pcm = 0;
        sys->motor4_rpm_pcm = 0;
    }
    if (pwm_motor_1 > 0 && pwm_motor_2 < 0) {
        sys->motor1_rpm_pcm = 0;
        sys->motor2_rpm_pcm = pwm_motor_1;
        sys->motor3_rpm_pcm = 0;
        sys->motor4_rpm_pcm = -pwm_motor_2;
    }
    if (pwm_motor_1 < 0 && pwm_motor_2 > 0) {
        sys->motor1_rpm_pcm = -pwm_motor_1;
        sys->motor2_rpm_pcm = 0;
        sys->motor3_rpm_pcm = pwm_motor_2;
        sys->motor4_rpm_pcm = 0;
    }
    if (pwm_motor_1 < 0 && pwm_motor_2 < 0) {
        sys->motor1_rpm_pcm = 0;
        sys->motor2_rpm_pcm = 0;
        sys->motor3_rpm_pcm = -pwm_motor_1;
        sys->motor4_rpm_pcm = -pwm_motor_2;
    }

    // Store the input PWM values
    sys->left_pwm = pwm_motor_1;
    sys->right_pwm = pwm_motor_2;

    // Update hardware
    ledc_set_duty(MTR_MODE, MTR_FRONT_LEFT, sys->motor1_rpm_pcm);
    ledc_update_duty(MTR_MODE, MTR_FRONT_LEFT);

    ledc_set_duty(MTR_MODE, MTR_FRONT_RIGHT, sys->motor2_rpm_pcm);
    ledc_update_duty(MTR_MODE, MTR_FRONT_RIGHT);

    ledc_set_duty(MTR_MODE, MTR_FRONT_LEFT_REV, sys->motor3_rpm_pcm);
    ledc_update_duty(MTR_MODE, MTR_FRONT_LEFT_REV);

    ledc_set_duty(MTR_MODE, MTR_FRONT_RIGHT_REV, sys->motor4_rpm_pcm);
    ledc_update_duty(MTR_MODE, MTR_FRONT_RIGHT_REV);

    ESP_LOGI(TAG, "M1: %d, M2: %d, M3: %d, M4: %d",
        sys->motor1_rpm_pcm,
        sys->motor2_rpm_pcm,
        sys->motor3_rpm_pcm,
        sys->motor4_rpm_pcm);
}

// Update motor PWM values (unused in your case, but kept for compatibility)
static void motor_update_impl(motor_system_t *self, TickType_t now) {
    (void)self;
    (void)now;
    // Nothing to do - motors are updated directly via update_motors_pwm()
}

void motor_system_init(motor_system_t *sys) {
    sys->left_pwm = 0;
    sys->right_pwm = 0;
    sys->motor1_rpm_pcm = 0;
    sys->motor2_rpm_pcm = 0;
    sys->motor3_rpm_pcm = 0;
    sys->motor4_rpm_pcm = 0;
    sys->update = motor_update_impl;

    ledc_init();
    ESP_LOGI(TAG, "Motor system initialized");
}

void motor_set_pwm(motor_system_t *sys, int left_pwm, int right_pwm) {
    // This is just a wrapper for update_motors_pwm
    update_motors_pwm(sys, left_pwm, right_pwm);
}

```

## subsystems/motors/motors.h <a name="subsystems-motors-motors-h"></a>

```c
#ifndef MOTORS_H
#define MOTORS_H

#include "freertos/FreeRTOS.h"
#include "driver/ledc.h"

// Motor configuration
#define MTR_FREQUENCY               7000
#define MTR_MODE                    LEDC_LOW_SPEED_MODE
#define MTR_DUTY_RES                LEDC_TIMER_13_BIT

// Motor GPIO pins
#define MTR_FRONT_LEFT_IO           6
#define MTR_FRONT_LEFT_TMR          LEDC_TIMER_0
#define MTR_FRONT_LEFT              LEDC_CHANNEL_1
#define MTR_FRONT_LEFT_DUTY         3361

#define MTR_FRONT_RIGHT_IO          5
#define MTR_FRONT_RIGHT_TMR         LEDC_TIMER_1
#define MTR_FRONT_RIGHT             LEDC_CHANNEL_0
#define MTR_FRONT_RIGHT_DUTY        3361

#define MTR_FRONT_LEFT_REV_IO       4
#define MTR_FRONT_LEFT_REV_TMR      LEDC_TIMER_2
#define MTR_FRONT_LEFT_REV          LEDC_CHANNEL_2
#define MTR_FRONT_LEFT_REV_DUTY     3361

#define MTR_FRONT_RIGHT_REV_IO      7
#define MTR_FRONT_RIGHT_REV_TMR     LEDC_TIMER_3
#define MTR_FRONT_RIGHT_REV         LEDC_CHANNEL_3
#define MTR_FRONT_RIGHT_REV_DUTY    3361

// Forward declaration
typedef struct motor_system_t motor_system_t;

// Struct definition
struct motor_system_t {
    int left_pwm;   // Signed PWM for left motors (-8191 to +8190)
    int right_pwm;  // Signed PWM for right motors (-8191 to +8190)

    // Internal motor PWM values
    int motor1_rpm_pcm;  // Left forward
    int motor2_rpm_pcm;  // Right forward
    int motor3_rpm_pcm;  // Left reverse
    int motor4_rpm_pcm;  // Right reverse

    void (*update)(motor_system_t *self, TickType_t now);
};

void motor_system_init(motor_system_t *sys);
void motor_set_pwm(motor_system_t *sys, int left_pwm, int right_pwm);
void update_motors_pwm(motor_system_t *sys, int pwm_motor_1, int pwm_motor_2);

#endif

```

## subsystems/i2c_bus/CMakeLists.txt <a name="subsystems-i2c-bus-cmakelists-txt"></a>

```c
idf_component_register(
    SRCS "i2c_bus.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_driver_i2c )

```

## subsystems/i2c_bus/i2c_bus.c <a name="subsystems-i2c-bus-i2c-bus-c"></a>

```c
/**
 * @file i2c_bus.c
 * @brief Centralized I2C Bus Manager for ESP32-C3
 *
 * This module provides a thread-safe, multi-device I2C bus manager that:
 * - Manages a single shared I2C bus accessed by multiple devices
 * - Provides mutex-protected access to prevent bus contention
 * - Tracks registered devices for debugging and management
 * - Offers convenience wrappers for common I2C operations
 *
 * Architecture:
 *
 *     ┌─────────────────────────────────────────┐
 *     │        I2C Bus Manager (This Module)    │
 *     │  ┌────────────────────────────────────┐ │
 *     │  │ Mutex-Protected Shared Bus         │ │
 *     │  └────────────────────────────────────┘ │
 *     │         ▲        ▲        ▲             │
 *     │         │        │        │             │
 *     └─────────┼────────┼────────┼─────────────┘
 *               │        │        │
 *         ┌─────┴──┐ ┌───┴───┐ ┌─┴────────┐
 *         │ INA219 │ │Sensor │ │Ultrasonic│
 *         │ 0x40   │ │ 0x57  │ │  0x??    │
 *         └────────┘ └───────┘ └──────────┘
 *
 * Why a Centralized Manager?
 *
 * Problem: Multiple subsystems (INA219, ultrasonic, future sensors) share
 * one I2C bus. Without coordination:
 * - Bus collisions (two devices transmitting simultaneously)
 * - Data corruption (one device interrupts another's transaction)
 * - Deadlocks (circular wait conditions)
 * - Bus lockup (device holds SDA low indefinitely)
 *
 * Solution: Single manager with mutex protection ensures:
 * - One transaction at a time (serialized access)
 * - Clean transaction boundaries (complete before next starts)
 * - Timeout handling (prevents infinite waits)
 * - Centralized error handling and logging
 *
 * I2C Protocol Primer:
 *
 * I2C uses two wires: SDA (data) and SCL (clock)
 *
 * Write Transaction:
 *   START → [ADDR+W] → ACK → [REG] → ACK → [DATA] → ACK → STOP
 *
 * Read Transaction:
 *   START → [ADDR+W] → ACK → [REG] → ACK →
 *   RESTART → [ADDR+R] → ACK → [DATA] ← ACK → STOP
 *
 * Key Concepts:
 * - Master (ESP32) controls clock and initiates transactions
 * - Slaves (sensors) respond to their addresses
 * - 7-bit addressing: 0x08-0x77 (0x00-0x07 and 0x78-0x7F reserved)
 * - Open-drain bus requires pull-up resistors (4.7kΩ typical)
 * - Bus speed: 100 kHz (Standard) or 400 kHz (Fast Mode)
 *
 * Thread Safety:
 * FreeRTOS mutex ensures atomic transactions. Without it:
 *
 *   Thread A: START → [ADDR]
 *   Thread B: START → [ADDR]  ← BUS COLLISION!
 *
 * With mutex:
 *   Thread A: lock → START → [transaction] → STOP → unlock
 *   Thread B: wait for lock → START → [transaction] → STOP → unlock
 *
 * @author Alexander Bobkov
 * @date January 2026
 */

#include "i2c_bus.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "I2C_BUS";

// ═══════════════════════════════════════════════════════════════
// MODULE-LEVEL STATE (Private, file scope)
// ═══════════════════════════════════════════════════════════════

/**
 * @brief Handle to ESP32 I2C master bus hardware
 *
 * NULL when uninitialized. Once created, this handle represents the
 * physical I2C peripheral (I2C0 on ESP32-C3). All device operations
 * route through this handle.
 */
static i2c_master_bus_handle_t bus_handle = NULL;

/**
 * @brief FreeRTOS mutex for thread-safe bus access
 *
 * Critical for preventing bus contention in multi-threaded systems.
 *
 * Mutex Behavior:
 * - Binary semaphore with ownership (mutex)
 * - Recursive: Same task can take multiple times
 * - Priority inheritance: Prevents priority inversion
 *
 * Usage Pattern:
 *   if (xSemaphoreTake(mutex, timeout) == pdTRUE) {
 *       // Critical section - exclusive bus access
 *       i2c_operation();
 *       xSemaphoreGive(mutex);
 *   }
 *
 * Why 100ms timeout?
 * - Typical I2C transaction: 1-10ms
 * - 100ms allows for slow devices or multi-byte transfers
 * - Prevents infinite blocking if a device malfunctions
 */
static SemaphoreHandle_t bus_mutex = NULL;

/**
 * @brief Registry of all devices on the bus
 *
 * Tracks active devices for:
 * - Debugging (what's on the bus?)
 * - Device management (remove by address/handle)
 * - Error tracking (which device failed?)
 *
 * Structure: Fixed-size array (simple, no dynamic allocation)
 * Alternative designs:
 * - Linked list: More flexible, but requires malloc/free
 * - Hash table: Faster lookup, more complex
 * - Fixed array: Simple, predictable, sufficient for typical use
 */
static i2c_device_t device_registry[I2C_MAX_DEVICES];

/**
 * @brief Count of registered devices
 *
 * Used for:
 * - Array bounds checking
 * - Quick "is bus empty?" test
 * - Logging/debugging
 */
static int device_count = 0;

// ═══════════════════════════════════════════════════════════════
// PUBLIC API FUNCTIONS
// ═══════════════════════════════════════════════════════════════

/**
 * @brief Get the I2C bus handle for direct hardware access
 *
 * Use Case: Advanced users who need direct ESP-IDF I2C API access.
 *
 * Warning: Bypasses mutex protection! Caller is responsible for:
 * - Taking the bus mutex manually
 * - Ensuring transaction atomicity
 * - Proper error handling
 *
 * Most users should use the i2c_bus_read/write functions instead.
 *
 * @return Handle to I2C master bus, or NULL if uninitialized
 */
i2c_master_bus_handle_t i2c_bus_get(void) {
    return bus_handle;
}

/**
 * @brief Initialize the I2C bus manager
 *
 * Initialization Sequence:
 *
 * 1. Check if already initialized (idempotent)
 * 2. Create FreeRTOS mutex for thread safety
 * 3. Configure I2C hardware peripheral
 * 4. Clear device registry
 * 5. Scan bus for existing devices (diagnostic)
 *
 * Hardware Configuration:
 * - Port: I2C_NUM_0 (ESP32-C3 has one I2C controller)
 * - SDA/SCL: Defined by I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO
 * - Clock Source: APB clock (default, 80 MHz on ESP32-C3)
 * - Glitch Filter: 7 cycles (filters noise spikes < 87.5ns @ 80MHz)
 * - Internal Pull-ups: Enabled (weak ~45kΩ, external 4.7kΩ recommended)
 *
 * Glitch Filtering:
 * I2C is sensitive to electrical noise. Glitch filter ignores pulses
 * shorter than N clock cycles, preventing false START/STOP conditions.
 * - Too low (< 3): Noise causes spurious transactions
 * - Too high (> 15): May filter legitimate fast edges
 * - 7 cycles: Good balance for most applications
 *
 * Pull-up Resistors:
 * I2C is open-drain, requires pull-ups on SDA and SCL.
 * - Internal pull-ups: Weak (~45kΩ), sufficient for short traces
 * - External pull-ups: Strong (4.7kΩ), required for:
 *   * Long traces (> 10cm)
 *   * Multiple devices
 *   * High capacitance
 *   * Fast mode (400 kHz)
 *
 * @return ESP_OK on success
 * @return ESP_ERR_NO_MEM if mutex creation fails
 * @return ESP-IDF error codes if bus creation fails
 */
esp_err_t i2c_bus_init(void) {
    // Idempotency check: Allow multiple init calls safely
    if (bus_handle != NULL) {
        ESP_LOGW(TAG, "I2C bus already initialized");
        return ESP_OK;
    }

    // ───────────────────────────────────────────────────────────
    // STEP 1: Create mutex for thread-safe access
    // ───────────────────────────────────────────────────────────
    bus_mutex = xSemaphoreCreateMutex();
    if (!bus_mutex) {
        ESP_LOGE(TAG, "Failed to create I2C mutex");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Initializing I2C bus...");
    ESP_LOGI(TAG, "  SDA: %d", I2C_MASTER_SDA_IO);
    ESP_LOGI(TAG, "  SCL: %d", I2C_MASTER_SCL_IO);

    // ───────────────────────────────────────────────────────────
    // STEP 2: Configure I2C hardware
    // ───────────────────────────────────────────────────────────
    i2c_master_bus_config_t cfg = {
        .i2c_port = I2C_NUM_0,                      // Use I2C controller 0
        .sda_io_num = I2C_MASTER_SDA_IO,            // Data line GPIO
        .scl_io_num = I2C_MASTER_SCL_IO,            // Clock line GPIO
        .clk_source = I2C_CLK_SRC_DEFAULT,          // APB clock (80 MHz)
        .glitch_ignore_cnt = 7,                     // Filter < 87.5ns spikes
        .flags.enable_internal_pullup = true,       // Enable weak pull-ups
    };

    esp_err_t ret = i2c_new_master_bus(&cfg, &bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init I2C bus: %s", esp_err_to_name(ret));
        // Clean up mutex on failure
        vSemaphoreDelete(bus_mutex);
        bus_mutex = NULL;
        return ret;
    }

    // ───────────────────────────────────────────────────────────
    // STEP 3: Initialize device registry
    // ───────────────────────────────────────────────────────────
    memset(device_registry, 0, sizeof(device_registry));
    device_count = 0;

    ESP_LOGI(TAG, "I2C bus initialized");

    // ───────────────────────────────────────────────────────────
    // STEP 4: Scan for existing devices (diagnostic)
    // ───────────────────────────────────────────────────────────
    i2c_bus_scan();

    return ESP_OK;
}

/**
 * @brief Add a device to the I2C bus
 *
 * Registration Process:
 *
 * 1. Validate bus is initialized
 * 2. Check device registry capacity
 * 3. Create ESP-IDF device handle
 * 4. Add to internal registry
 *
 * What is a Device Handle?
 * - Opaque pointer to ESP-IDF's internal device structure
 * - Contains: address, bus reference, timing parameters
 * - Used in all subsequent I2C operations for this device
 *
 * Device Configuration:
 * - Address Length: 7-bit (standard I2C addressing)
 * - Device Address: 0x08-0x77 (0x00-0x07, 0x78-0x7F reserved)
 * - SCL Speed: Defined by I2C_MASTER_FREQ_HZ (typically 100 kHz)
 *
 * Why Register Devices?
 * - Pre-configuration: Set timing parameters once
 * - Error tracking: Know which device caused issues
 * - Resource management: Clean up on deinit
 * - Debugging: List active devices
 *
 * I2C Address Ranges:
 * - 0x00-0x07: Reserved (general call, START byte, etc.)
 * - 0x08-0x77: Valid 7-bit device addresses
 * - 0x78-0x7F: Reserved (10-bit addressing, future use)
 *
 * @param address 7-bit I2C address (0x08-0x77)
 * @param name Human-readable device name (for logging)
 * @param dev_handle Output: Receives created device handle
 *
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_STATE if bus not initialized
 * @return ESP_ERR_NO_MEM if device registry is full
 * @return ESP-IDF error codes on device creation failure
 */
esp_err_t i2c_bus_add_device(uint8_t address, const char *name,
                             i2c_master_dev_handle_t *dev_handle) {
    // Validate bus is initialized
    if (!bus_handle) return ESP_ERR_INVALID_STATE;

    // Check registry capacity
    if (device_count >= I2C_MAX_DEVICES) return ESP_ERR_NO_MEM;

    ESP_LOGI(TAG, "Adding device '%s' at 0x%02X", name, address);

    // ───────────────────────────────────────────────────────────
    // Configure device parameters
    // ───────────────────────────────────────────────────────────
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,     // Standard 7-bit addressing
        .device_address = address,                  // Device's I2C address
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,        // Bus clock speed
    };

    // Create device handle with ESP-IDF
    esp_err_t ret = i2c_master_bus_add_device(bus_handle, &dev_cfg, dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add device: %s", esp_err_to_name(ret));
        return ret;
    }

    // ───────────────────────────────────────────────────────────
    // Register device in our tracking system
    // ───────────────────────────────────────────────────────────
    device_registry[device_count].address = address;
    device_registry[device_count].dev_handle = *dev_handle;
    device_registry[device_count].is_active = true;
    device_registry[device_count].name = name;
    device_count++;

    return ESP_OK;
}

/**
 * @brief Write data to a device register
 *
 * I2C Register Write Protocol:
 *
 *   START → [ADDR+W] → ACK → [REG] → ACK → [DATA...] → ACK → STOP
 *
 *   Example: Write 0x42 to register 0x10 on device 0x40:
 *     START → 0x80 (0x40<<1 | W) → ACK → 0x10 → ACK → 0x42 → ACK → STOP
 *
 * Operation:
 * 1. Take bus mutex (block if another transaction in progress)
 * 2. Build packet: [reg_addr][data bytes]
 * 3. Transmit to device
 * 4. Release mutex
 *
 * Buffer Size Limit:
 * Uses 128-byte stack buffer for efficiency. For larger transfers,
 * consider dynamic allocation or chunked writes.
 *
 * Mutex Timeout:
 * 100ms timeout prevents infinite blocking if another thread hangs.
 * If timeout occurs, returns ESP_ERR_TIMEOUT.
 *
 * @param dev_handle Device handle from i2c_bus_add_device()
 * @param reg_addr Register address to write to
 * @param data Pointer to data bytes
 * @param len Number of bytes to write
 *
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if dev_handle is NULL
 * @return ESP_ERR_TIMEOUT if mutex not available within 100ms
 * @return ESP_ERR_INVALID_SIZE if len + 1 > 128 bytes
 * @return ESP-IDF I2C error codes on transmission failure
 */
esp_err_t i2c_bus_write(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr,
                        const uint8_t *data, size_t len) {
    if (!dev_handle) return ESP_ERR_INVALID_ARG;

    // ───────────────────────────────────────────────────────────
    // Acquire exclusive bus access
    // ───────────────────────────────────────────────────────────
    if (xSemaphoreTake(bus_mutex, pdMS_TO_TICKS(100)) != pdTRUE)
        return ESP_ERR_TIMEOUT;

    // ───────────────────────────────────────────────────────────
    // Build I2C packet: [register][data bytes...]
    // ───────────────────────────────────────────────────────────
    uint8_t buf[128];
    if (len + 1 > sizeof(buf)) {
        xSemaphoreGive(bus_mutex);
        return ESP_ERR_INVALID_SIZE;
    }

    buf[0] = reg_addr;              // First byte is register address
    memcpy(&buf[1], data, len);     // Followed by data bytes

    // ───────────────────────────────────────────────────────────
    // Transmit to device
    // ───────────────────────────────────────────────────────────
    esp_err_t ret = i2c_master_transmit(dev_handle, buf, len + 1,
                                        I2C_MASTER_TIMEOUT_MS);

    // ───────────────────────────────────────────────────────────
    // Release bus for other threads
    // ───────────────────────────────────────────────────────────
    xSemaphoreGive(bus_mutex);

    return ret;
}

/**
 * @brief Write a single byte to a device register
 *
 * Convenience wrapper around i2c_bus_write() for single-byte operations.
 *
 * Common use cases:
 * - Setting configuration registers
 * - Triggering measurements
 * - Writing command bytes
 *
 * @param dev_handle Device handle
 * @param reg_addr Register address
 * @param value Byte value to write
 *
 * @return ESP_OK on success, error codes on failure
 */
esp_err_t i2c_bus_write_byte(i2c_master_dev_handle_t dev_handle,
                             uint8_t reg_addr, uint8_t value) {
    return i2c_bus_write(dev_handle, reg_addr, &value, 1);
}

/**
 * @brief Read data from a device register
 *
 * I2C Register Read Protocol (Combined Transaction):
 *
 *   START → [ADDR+W] → ACK → [REG] → ACK →
 *   RESTART → [ADDR+R] → ACK → [DATA] ← ACK → ... → NACK → STOP
 *
 *   Example: Read 2 bytes from register 0x02 on device 0x40:
 *     START → 0x80 (0x40<<1 | W) → ACK → 0x02 → ACK →
 *     RESTART → 0x81 (0x40<<1 | R) → ACK → [byte1] ← ACK → [byte2] ← NACK → STOP
 *
 * Why RESTART?
 * Combined (write-then-read) transaction prevents other devices from
 * accessing the bus between register address write and data read.
 * This ensures atomicity.
 *
 * Alternative (Separate Transactions):
 *   STOP after register write, START before read
 *   Problem: Another device could access bus in between!
 *
 * Operation:
 * 1. Take bus mutex
 * 2. Transmit register address (write phase)
 * 3. Without releasing bus, receive data (read phase)
 * 4. Release mutex
 *
 * ESP-IDF Implementation:
 * i2c_master_transmit_receive() handles the RESTART automatically.
 * This is cleaner than manually managing two separate transactions.
 *
 * @param dev_handle Device handle
 * @param reg_addr Register address to read from
 * @param data Buffer to receive data
 * @param len Number of bytes to read
 *
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if dev_handle is NULL
 * @return ESP_ERR_TIMEOUT if mutex not available
 * @return ESP-IDF I2C error codes on transaction failure
 */
esp_err_t i2c_bus_read(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr,
                       uint8_t *data, size_t len) {
    if (!dev_handle) return ESP_ERR_INVALID_ARG;

    // ───────────────────────────────────────────────────────────
    // Acquire exclusive bus access
    // ───────────────────────────────────────────────────────────
    if (xSemaphoreTake(bus_mutex, pdMS_TO_TICKS(100)) != pdTRUE)
        return ESP_ERR_TIMEOUT;

    // ───────────────────────────────────────────────────────────
    // Combined transaction: Write register address, then read data
    // ───────────────────────────────────────────────────────────
    // ESP-IDF handles RESTART automatically between write and read phases
    esp_err_t ret = i2c_master_transmit_receive(dev_handle,
                                                &reg_addr, 1,    // Write phase
                                                data, len,       // Read phase
                                                I2C_MASTER_TIMEOUT_MS);

    // ───────────────────────────────────────────────────────────
    // Release bus
    // ───────────────────────────────────────────────────────────
    xSemaphoreGive(bus_mutex);

    return ret;
}

/**
 * @brief Read a single byte from a device register
 *
 * Convenience wrapper around i2c_bus_read() for single-byte reads.
 *
 * Common use cases:
 * - Reading status registers
 * - Reading device ID
 * - Polling for completion flags
 *
 * @param dev_handle Device handle
 * @param reg_addr Register address
 * @param value Output: Receives byte value
 *
 * @return ESP_OK on success, error codes on failure
 */
esp_err_t i2c_bus_read_byte(i2c_master_dev_handle_t dev_handle,
                            uint8_t reg_addr, uint8_t *value) {
    return i2c_bus_read(dev_handle, reg_addr, value, 1);
}

/**
 * @brief Check if a device is present on the bus
 *
 * Detection Method:
 * Send the device address and check for ACK response.
 *
 *   START → [ADDR+W] → ACK? → STOP
 *
 * - If device responds with ACK: Device is present
 * - If no ACK (NACK or timeout): Device is absent
 *
 * Use Cases:
 * - Bus scanning (find all devices)
 * - Hot-plug detection (device removed/inserted?)
 * - Diagnostic checks (is sensor working?)
 *
 * Limitations:
 * - Some devices don't ACK until properly initialized
 * - Bus faults can cause false negatives
 * - Doesn't guarantee device is functional, just addressable
 *
 * @param address 7-bit I2C address to probe
 *
 * @return true if device ACKs, false otherwise
 */
bool i2c_bus_device_present(uint8_t address) {
    if (!bus_handle) return false;

    // ESP-IDF probe function: sends address, checks for ACK
    return i2c_master_probe(bus_handle, address, I2C_MASTER_TIMEOUT_MS) == ESP_OK;
}

/**
 * @brief Scan I2C bus for all present devices
 *
 * Scans the valid 7-bit address range (0x08-0x77) and logs found devices.
 *
 * Address Range Explanation:
 * - 0x00-0x07: Reserved addresses (general call, START byte, etc.)
 * - 0x08-0x77: Valid device addresses (112 addresses)
 * - 0x78-0x7F: Reserved (10-bit addressing)
 *
 * Use Cases:
 * - Initial setup: What devices are on the bus?
 * - Debugging: Did the sensor get detected?
 * - Diagnostics: Bus health check
 *
 * Performance:
 * Scans 112 addresses at ~1ms per probe = ~112ms total.
 * This is acceptable for one-time initialization but too slow for
 * runtime polling.
 *
 * @return Number of devices found
 */
int i2c_bus_scan(void) {
    if (!bus_handle) return 0;

    ESP_LOGI(TAG, "Scanning I2C bus...");

    int found = 0;

    // Scan valid 7-bit address range
    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        if (i2c_bus_device_present(addr)) {
            ESP_LOGI(TAG, "  Found device at 0x%02X", addr);
            found++;
        }
    }

    if (!found) {
        ESP_LOGW(TAG, "No I2C devices found");
    }

    return found;
}

/**
 * @brief Remove a device from the bus
 *
 * Cleanup Process:
 * 1. Remove device from ESP-IDF's internal structures
 * 2. Free associated resources
 *
 * Note: Does NOT update our device_registry. This is intentional -
 * the registry is for debugging and typically isn't modified at runtime.
 *
 * Use Case:
 * Hot-unplugging devices (rare in embedded systems).
 * Most applications initialize devices once and never remove them.
 *
 * @param dev_handle Device handle to remove
 *
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if handle is NULL
 * @return ESP-IDF error codes on failure
 */
esp_err_t i2c_bus_remove_device(i2c_master_dev_handle_t dev_handle) {
    if (!dev_handle) return ESP_ERR_INVALID_ARG;

    return i2c_master_bus_rm_device(dev_handle);
}

/**
 * @brief Shut down the I2C bus manager
 *
 * Cleanup Sequence:
 * 1. Delete I2C master bus (frees hardware resources)
 * 2. Delete mutex (frees FreeRTOS object)
 * 3. Clear state variables
 *
 * Important: Caller must ensure no devices are actively using the bus!
 * Removing the bus while transactions are in progress causes undefined
 * behavior (likely crashes or bus lockup).
 *
 * Proper Shutdown Order:
 * 1. Stop all tasks using I2C
 * 2. Call i2c_bus_remove_device() for all devices
 * 3. Call i2c_bus_deinit()
 *
 * @return ESP_OK on success
 * @return ESP-IDF error codes on bus deletion failure
 */
esp_err_t i2c_bus_deinit(void) {
    if (!bus_handle) return ESP_OK;  // Already deinitialized

    // ───────────────────────────────────────────────────────────
    // Delete I2C master bus (releases hardware)
    // ───────────────────────────────────────────────────────────
    esp_err_t ret = i2c_del_master_bus(bus_handle);
    if (ret != ESP_OK) return ret;

    // ───────────────────────────────────────────────────────────
    // Delete mutex (releases FreeRTOS object)
    // ───────────────────────────────────────────────────────────
    if (bus_mutex) {
        vSemaphoreDelete(bus_mutex);
        bus_mutex = NULL;
    }

    // ───────────────────────────────────────────────────────────
    // Clear state
    // ───────────────────────────────────────────────────────────
    bus_handle = NULL;
    device_count = 0;

    return ESP_OK;
}

```

## subsystems/i2c_bus/i2c_bus.h <a name="subsystems-i2c-bus-i2c-bus-h"></a>

```c
#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include <stdbool.h>

#define I2C_MASTER_SDA_IO 3
#define I2C_MASTER_SCL_IO 2
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_MASTER_TIMEOUT_MS 100

#define I2C_MAX_DEVICES 8

typedef struct {
    uint8_t address;
    i2c_master_dev_handle_t dev_handle;
    bool is_active;
    const char *name;
} i2c_device_t;

esp_err_t i2c_bus_init(void);
i2c_master_bus_handle_t i2c_bus_get(void);

esp_err_t i2c_bus_add_device(uint8_t address, const char *name,
                             i2c_master_dev_handle_t *dev_handle);

esp_err_t i2c_bus_write(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr,
                        const uint8_t *data, size_t len);

esp_err_t i2c_bus_write_byte(i2c_master_dev_handle_t dev_handle,
                             uint8_t reg_addr, uint8_t value);

esp_err_t i2c_bus_read(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr,
                       uint8_t *data, size_t len);

esp_err_t i2c_bus_read_byte(i2c_master_dev_handle_t dev_handle,
                            uint8_t reg_addr, uint8_t *value);

bool i2c_bus_device_present(uint8_t address);
int i2c_bus_scan(void);

esp_err_t i2c_bus_remove_device(i2c_master_dev_handle_t dev_handle);
esp_err_t i2c_bus_deinit(void);

```

## subsystems/sensors/CMakeLists.txt <a name="subsystems-sensors-cmakelists-txt"></a>

```c
idf_component_register(
    SRCS "ina219_sensor.c"
         "temp_sensor.c"
         "ultrasonic_sensor.c"
    INCLUDE_DIRS "."
    REQUIRES driver
             esp_adc
             i2c_bus
    PRIV_REQUIRES i2c_bus
                  esp-idf-lib__ina219
                  esp-idf-lib__i2cdev)

```

## subsystems/sensors/ina219_sensor.c <a name="subsystems-sensors-ina219-sensor-c"></a>

```c
#include "ina219_sensor.h"
#include "i2c_bus.h"
#include "esp_log.h"
#include <string.h>
#include <math.h>

static const char *TAG = "INA219";

// INA219 registers
#define INA219_REG_CONFIG       0x00
#define INA219_REG_SHUNTVOLTAGE 0x01
#define INA219_REG_BUSVOLTAGE   0x02
#define INA219_REG_POWER        0x03
#define INA219_REG_CURRENT      0x04
#define INA219_REG_CALIBRATION  0x05

// Configuration - CHANGE THIS to match your actual shunt resistor!
// Common values: 0.1 (R100), 0.01 (R010), 0.05 (R050)
#define SHUNT_RESISTOR_OHMS 0.1f

static i2c_master_dev_handle_t ina219_handle = NULL;
static float current_lsb = 0.0f;  // Calculated during calibration
static float power_lsb = 0.0f;    // Calculated during calibration

// Read 16-bit register
static esp_err_t ina219_read_reg(uint8_t reg, uint16_t *value) {
    uint8_t data[2];
    esp_err_t ret = i2c_bus_read(ina219_handle, reg, data, 2);
    if (ret == ESP_OK) {
        *value = (data[0] << 8) | data[1];
    }
    return ret;
}

// Write 16-bit register
static esp_err_t ina219_write_reg(uint8_t reg, uint16_t value) {
    uint8_t data[2] = {(value >> 8) & 0xFF, value & 0xFF};
    return i2c_bus_write(ina219_handle, reg, data, 2);
}

static void ina219_update_impl(ina219_system_t *self, TickType_t now) {
    static TickType_t last_read = 0;

    if ((now - last_read) >= pdMS_TO_TICKS(2500)) {
        uint16_t shunt_raw, bus_raw, power_raw, current_raw;

        // Read bus voltage (supply voltage)
        if (ina219_read_reg(INA219_REG_BUSVOLTAGE, &bus_raw) == ESP_OK) {
            self->bus_voltage = (bus_raw >> 3) * 0.004f;  // 4mV per LSB
        }

        // Read shunt voltage (voltage across shunt resistor)
        if (ina219_read_reg(INA219_REG_SHUNTVOLTAGE, &shunt_raw) == ESP_OK) {
            self->shunt_voltage = (int16_t)shunt_raw * 0.00001f;  // 10µV per LSB
        }

        // Read current (uses calibrated current_lsb)
        if (ina219_read_reg(INA219_REG_CURRENT, &current_raw) == ESP_OK) {
            self->current = (int16_t)current_raw * current_lsb;  // Result in Amps
        }

        // Read power (uses calibrated power_lsb)
        if (ina219_read_reg(INA219_REG_POWER, &power_raw) == ESP_OK) {
            self->power = (int16_t)power_raw * power_lsb;  // Result in Watts
        }

        ESP_LOGI(TAG, "VBUS: %.2fV, I: %.2fmA, P: %.2fmW",
                 self->bus_voltage, self->current * 1000.0f, self->power * 1000.0f);

        last_read = now;
    }
}

void ina219_system_init(ina219_system_t *sys) {
    sys->bus_voltage = 0.0f;
    sys->shunt_voltage = 0.0f;
    sys->current = 0.0f;
    sys->power = 0.0f;
    sys->update = ina219_update_impl;

    // Add INA219 to I2C bus
    esp_err_t ret = i2c_bus_add_device(I2C_ADDR, "INA219", &ina219_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add INA219 to I2C bus");
        return;
    }

    // Configure INA219
    // 0x399F = 32V bus range, ±320mV shunt range (gain /8), 12-bit resolution, continuous mode
    uint16_t config = 0x399F;
    ina219_write_reg(INA219_REG_CONFIG, config);

    // Calculate calibration (based on esp-idf-lib algorithm)
    // Max shunt voltage for gain /8: 320mV
    float u_shunt_max = 0.32f;  // For gain /8 (±320mV range)

    // Calculate current LSB
    // Algorithm: round up to nearest 0.1mA for better precision
    current_lsb = u_shunt_max / SHUNT_RESISTOR_OHMS / 32767.0f;
    current_lsb = ceilf(current_lsb / 0.0001f) * 0.0001f;

    // Power LSB is always 20× current LSB (per INA219 datasheet)
    power_lsb = current_lsb * 20.0f;

    // Calculate calibration register value
    // Formula: Cal = 0.04096 / (Current_LSB × R_shunt)
    uint16_t cal = (uint16_t)(0.04096f / (current_lsb * SHUNT_RESISTOR_OHMS));

    ESP_LOGI(TAG, "INA219 Calibration:");
    ESP_LOGI(TAG, "  Shunt Resistor : %.4f Ω", SHUNT_RESISTOR_OHMS);
    ESP_LOGI(TAG, "  Current LSB    : %.6f A (%.3f mA)", current_lsb, current_lsb * 1000.0f);
    ESP_LOGI(TAG, "  Power LSB      : %.6f W (%.3f mW)", power_lsb, power_lsb * 1000.0f);
    ESP_LOGI(TAG, "  Cal Register   : 0x%04X (%u)", cal, cal);

    // Write calibration to device
    ina219_write_reg(INA219_REG_CALIBRATION, cal);

    ESP_LOGI(TAG, "INA219 initialized and calibrated");
}

```

## subsystems/sensors/ina219_sensor.h <a name="subsystems-sensors-ina219-sensor-h"></a>

```c
#ifndef INA219_SENSOR_H
#define INA219_SENSOR_H

#include "freertos/FreeRTOS.h"
#include "ina219.h"

//#define I2C_PORT 0
#define I2C_ADDR 0x40
#define I2C_SDA_GPIO 3
#define I2C_SCL_GPIO 2
#define SHUNT_RESISTOR_MILLI_OHM 100

// Forward declaration
typedef struct ina219_system_t ina219_system_t;

// Struct definition
struct ina219_system_t {
    float bus_voltage;
    float shunt_voltage;
    float current;
    float power;

    ina219_t dev;

    void (*update)(ina219_system_t *self, TickType_t now);
};

void ina219_system_init(ina219_system_t *sys);

#endif

```

## subsystems/sensors/temp_sensor.c <a name="subsystems-sensors-temp-sensor-c"></a>

```c
#include "temp_sensor.h"
#include "esp_log.h"

static const char *TAG = "TEMP_SENSOR";

static void temp_sensor_update_impl(temp_sensor_system_t *self, TickType_t now) {
    static TickType_t last_read = 0;

    // Read every 5 seconds
    if ((now - last_read) >= pdMS_TO_TICKS(5000)) {
        ESP_ERROR_CHECK(temperature_sensor_get_celsius(self->handle, &self->temperature));
        //ESP_LOGI(TAG, "Temperature: %.2f°C", self->temperature);
        // More detailed logging
                ESP_LOGI(TAG, "==========================================");
                ESP_LOGI(TAG, "ESP32-C3 Internal Temperature Sensor");
                ESP_LOGI(TAG, "Current Temperature: %.2f°C", self->temperature);
                ESP_LOGI(TAG, "==========================================");

        last_read = now;
    }
}

void temp_sensor_system_init(temp_sensor_system_t *sys) {
    sys->temperature = 0.0f;
    sys->handle = NULL;
    sys->update = temp_sensor_update_impl;

    temperature_sensor_config_t temp_sensor_config =
        TEMPERATURE_SENSOR_CONFIG_DEFAULT(10, 50);

    ESP_ERROR_CHECK(temperature_sensor_install(&temp_sensor_config, &sys->handle));
    ESP_ERROR_CHECK(temperature_sensor_enable(sys->handle));

    ESP_LOGI(TAG, "Temperature sensor initialized");
}

```

## subsystems/sensors/temp_sensor.h <a name="subsystems-sensors-temp-sensor-h"></a>

```c
#ifndef TEMP_SENSOR_H
#define TEMP_SENSOR_H

#include "freertos/FreeRTOS.h"
#include "driver/temperature_sensor.h"

// Forward declaration
typedef struct temp_sensor_system_t temp_sensor_system_t;

// Struct definition
struct temp_sensor_system_t {
    float temperature;
    temperature_sensor_handle_t handle;

    void (*update)(temp_sensor_system_t *self, TickType_t now);
};

void temp_sensor_system_init(temp_sensor_system_t *sys);

#endif

```

## subsystems/sensors/ultrasonic_sensor.c <a name="subsystems-sensors-ultrasonic-sensor-c"></a>

```c
#include "ultrasonic_sensor.h"
#include "i2c_bus.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "ULTRASONIC";

// I2C address and commands (from working test code)
#define ULTRASONIC_I2C_ADDR         0x57
#define CMD_MEASURE_CM              0x50
#define CMD_MEASURE_INCH            0x51
#define CMD_MEASURE_US              0x52

static i2c_master_dev_handle_t ultrasonic_handle = NULL;

// Measure distance (adapted from working test code)
static esp_err_t ultrasonic_measure_distance(uint16_t *distance) {
    if (ultrasonic_handle == NULL) {
        ESP_LOGE(TAG, "Ultrasonic not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // CRITICAL: Send init commands BEFORE EVERY measurement
    // Without this, only the first measurement works
    uint8_t init_cmds[] = {0x00, 0x01};
    for (int i = 0; i < sizeof(init_cmds); i++) {
        esp_err_t ret = i2c_master_transmit(ultrasonic_handle, &init_cmds[i], 1, 500);
        if (ret != ESP_OK) {
            // Init commands can fail during startup or when sensor is busy - this is normal
            ESP_LOGD(TAG, "Init cmd 0x%02X: %s", init_cmds[i], esp_err_to_name(ret));
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Send measurement command (0x50 = cm)
    uint8_t cmd = CMD_MEASURE_CM;
    esp_err_t ret = i2c_master_transmit(ultrasonic_handle, &cmd, 1, 500);
    if (ret != ESP_OK) {
        // NACK during measurement command usually means sensor can't measure (too close/busy)
        ESP_LOGD(TAG, "Measurement command failed (sensor may be too close or busy)");
        return ESP_ERR_INVALID_RESPONSE;  // Special error code for "too close"
    }

    // Wait for measurement (70ms as in working code)
    vTaskDelay(pdMS_TO_TICKS(70));

    // Read 4 bytes
    uint8_t data[4];
    ret = i2c_master_receive(ultrasonic_handle, data, 4, 500);
    if (ret != ESP_OK) {
        ESP_LOGD(TAG, "Read failed (sensor may be too close or busy)");
        return ESP_ERR_INVALID_RESPONSE;
    }

    // Extract distance from first 2 bytes (LITTLE-ENDIAN: low byte first)
    //*distance = data[0] | (data[1] << 8);
    *distance = (data[0] << 8) | data[1];

    return ESP_OK;
}

static void ultrasonic_update_impl(ultrasonic_system_t *self, TickType_t now) {
    static TickType_t last_read = 0;

    // Read every 100ms
    if ((now - last_read) < pdMS_TO_TICKS(100)) {
        return;
    }
    last_read = now;

    // Measure distance
    uint16_t raw_distance;
    esp_err_t ret = ultrasonic_measure_distance(&raw_distance);

    if (ret == ESP_OK) {
        // Convert from mm to cm (sensor returns millimeters)
        self->distance_cm = (float)raw_distance / 10.0f;

        // Check for valid range
        if (raw_distance == 0) {
            self->measurement_valid = false;
            ESP_LOGW(TAG, "No object detected (distance = 0)");
        } else if (self->distance_cm >= 2.0f && self->distance_cm <= 400.0f) {
            self->measurement_valid = true;
            ESP_LOGI(TAG, "Distance: %.2f cm (raw: %u mm)", self->distance_cm, raw_distance);
        } else {
            self->measurement_valid = false;
            ESP_LOGW(TAG, "Distance out of range: %.2f cm", self->distance_cm);
        }
    } else if (ret == ESP_ERR_INVALID_RESPONSE) {
        // Sensor NACKed - object is likely too close (< 2cm) or sensor is busy
        self->measurement_valid = false;
        self->distance_cm = 1.0f;  // Indicate "too close"
        ESP_LOGW(TAG, "Object too close or sensor busy");
    } else {
        self->measurement_valid = false;
        ESP_LOGW(TAG, "Measurement failed");
    }
}

void ultrasonic_system_init(ultrasonic_system_t *sys) {
    sys->distance_cm = 0.0f;
    sys->measurement_valid = false;
    sys->update = ultrasonic_update_impl;

    // Add device to I2C bus using your bus manager
    esp_err_t ret = i2c_bus_add_device(ULTRASONIC_I2C_ADDR, "ULTRASONIC", &ultrasonic_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add ultrasonic to I2C bus: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "Ultrasonic sensor initialized at 0x%02X", ULTRASONIC_I2C_ADDR);

    // Send initialization sequence (from working test code)
    ESP_LOGI(TAG, "Sending init commands...");
    uint8_t init_cmds[] = {0x00, 0x01, 0x02, 0xFF};
    for (int i = 0; i < sizeof(init_cmds); i++) {
        i2c_master_transmit(ultrasonic_handle, &init_cmds[i], 1, 500);
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // Try config register writes (from working test code)
    uint8_t cfg1[] = {0x00, 0x01};
    i2c_master_transmit(ultrasonic_handle, cfg1, 2, 500);
    vTaskDelay(pdMS_TO_TICKS(100));

    uint8_t cfg2[] = {0x01, 0x51};
    i2c_master_transmit(ultrasonic_handle, cfg2, 2, 500);
    vTaskDelay(pdMS_TO_TICKS(100));

    // Test initial measurement
    uint16_t test_distance;
    ret = ultrasonic_measure_distance(&test_distance);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Initial test: %u cm", test_distance);
    } else {
        ESP_LOGW(TAG, "Initial test failed (normal on startup)");
    }
}

```

## subsystems/sensors/ultrasonic_sensor.h <a name="subsystems-sensors-ultrasonic-sensor-h"></a>

```c
#ifndef ULTRASONIC_SENSOR_H
#define ULTRASONIC_SENSOR_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"

typedef struct ultrasonic_system_s {
    float distance_cm;           // Measured distance in centimeters
    bool measurement_valid;      // True if last measurement was successful
    void (*update)(struct ultrasonic_system_s *self, TickType_t now);
} ultrasonic_system_t;

/**
 * Initialize the I2C ultrasonic sensor
 * Uses I2C address 0x57 with command 0x50 for cm measurements
 */
void ultrasonic_system_init(ultrasonic_system_t *sys);

#endif // ULTRASONIC_SENSOR_H

```

## subsystems/adc/CMakeLists.txt <a name="subsystems-adc-cmakelists-txt"></a>

```c
idf_component_register(
    SRCS "adc.c"
    INCLUDE_DIRS "."
    REQUIRES esp_adc
)

```

## subsystems/adc/adc.c <a name="subsystems-adc-adc-c"></a>

```c
#include "adc.h"
#include "esp_log.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

static const char *TAG = "ADC";

// ADC channels for joystick
#define ADC_CHANNEL_X ADC_CHANNEL_0  // GPIO 0
#define ADC_CHANNEL_Y ADC_CHANNEL_1  // GPIO 1

static adc_oneshot_unit_handle_t adc_handle = NULL;

static void adc_read_impl(adc_system_t *self, int *x, int *y) {
    int x_val, y_val;

    ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, ADC_CHANNEL_X, &x_val));
    ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, ADC_CHANNEL_Y, &y_val));

    *x = x_val;
    *y = y_val;

    self->x_raw = x_val;
    self->y_raw = y_val;
}

static void adc_update_impl(adc_system_t *self, TickType_t now) {
    (void)now;
    // ADC reading happens on demand via read() function
}

void adc_system_init(adc_system_t *sys) {
    sys->last_reading = 0;
    sys->x_raw = 0;
    sys->y_raw = 0;
    sys->update = adc_update_impl;
    sys->read = adc_read_impl;

    // Configure ADC
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle));

    // Configure ADC channels
    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_X, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_Y, &config));

    ESP_LOGI(TAG, "ADC initialized");
}

```

## subsystems/adc/adc.h <a name="subsystems-adc-adc-h"></a>

```c
#ifndef ADC_H
#define ADC_H

#include "freertos/FreeRTOS.h"
#include "esp_adc/adc_oneshot.h"

// Forward declaration
typedef struct adc_system_t adc_system_t;

// Struct definition
struct adc_system_t {
    int last_reading;
    int x_raw;  // Joystick X-axis raw value
    int y_raw;  // Joystick Y-axis raw value

    void (*update)(adc_system_t *self, TickType_t now);
    void (*read)(adc_system_t *self, int *x, int *y);
};

void adc_system_init(adc_system_t *sys);

#endif

```

## subsystems/controls/CMakeLists.txt <a name="subsystems-controls-cmakelists-txt"></a>

```c
idf_component_register(
    SRCS "joystick.c"
    INCLUDE_DIRS "."
    REQUIRES freertos)

```

## subsystems/controls/joystick.c <a name="subsystems-controls-joystick-c"></a>

```c
/**
 * @file joystick.c
 * @brief Advanced Joystick Input Processing and Differential Drive Mixing
 *
 * This module provides sophisticated joystick input handling with:
 * - Automatic calibration and center-point detection
 * - Asymmetric deadband zones for drift elimination
 * - Non-linear response curves for smooth control
 * - Differential drive mixing with arc limiting
 *
 * Control Philosophy:
 * The goal is to create intuitive, predictable robot control where:
 * - Small joystick movements produce gentle arcs (not jerky turns)
 * - Large movements allow aggressive maneuvers (spins, tight turns)
 * - The robot never does unexpected things due to joystick drift
 * - Control feels natural and responsive, not robotic
 *
 * Mathematical Approach:
 * 1. Raw ADC values → Normalized [-1, +1] coordinates
 * 2. Deadband filtering → Eliminates drift and centering issues
 * 3. Non-linear shaping → Creates smooth response curves
 * 4. Differential mixing → Converts X/Y to left/right motor commands
 * 5. Arc limiting → Prevents overly aggressive turning
 * 6. PWM scaling → Converts to motor controller values (0-8191)
 *
 * @author Alexander Bobkov
 * @date January 2026
 */

#include "joystick.h"

/**
 * @brief Apply deadband filter to eliminate drift
 *
 * Joystick hardware is never perfectly centered. Even at "rest", the ADC
 * readings fluctuate (±50 counts is typical). Without a deadband, this
 * causes constant micro-movements and motor whine.
 *
 * The deadband creates a "dead zone" around zero where small values are
 * treated as exactly zero. This eliminates drift while preserving
 * intentional control inputs.
 *
 * Mathematical behavior:
 * - If |v| < d: output = 0 (inside deadband)
 * - If |v| ≥ d: output = v (outside deadband, pass through unchanged)
 *
 * Trade-offs:
 * - Too small (d < 0.03): Drift not eliminated, motors buzz
 * - Too large (d > 0.15): Loss of fine control, "dead spots" feel bad
 * - Sweet spot: 0.05-0.10 (5-10% of full range)
 *
 * @param v Input value (typically -1.0 to +1.0)
 * @param d Deadband threshold (e.g., 0.05 = 5%)
 * @return Filtered value: 0 if within deadband, otherwise unchanged
 *
 * @note This is a simple rectangular deadband. More advanced alternatives:
 *       - Circular deadband (magnitude-based)
 *       - Gradient deadband (smooth transition, not hard cutoff)
 *       - Hysteresis deadband (prevents rapid on/off oscillation)
 */
static float apply_deadband(float v, float d)
{
    return (fabsf(v) < d) ? 0.0f : v;
}

/**
 * @brief Initialize joystick HAL (Hardware Abstraction Layer)
 *
 * Sets up the joystick system with:
 * - Zero initial values (safe state)
 * - Default calibration parameters
 * - Learning mode configuration
 *
 * Calibration Strategy:
 * The system can auto-learn the joystick center point by averaging the
 * first N samples (typically N=200, collected over 2-5 seconds). This
 * compensates for:
 * - Manufacturing tolerances (no two joysticks are identical)
 * - Mechanical wear (center drifts over time)
 * - Electrical noise (ADC reference variations)
 * - Temperature effects (resistance changes with temp)
 *
 * Current Implementation:
 * Uses fixed constants (JS_CENTER_X, JS_RANGE_X) rather than learning.
 * The learning code structure is kept for future enhancement.
 *
 * @param js Pointer to joystick HAL structure to initialize
 */
void joystick_hal_init(joystick_hal_t *js)
{
    // Raw ADC values (12-bit: 0-4095 typical range)
    js->raw_x = 0;
    js->raw_y = 0;

    // Calibration parameters (for auto-learning mode)
    js->center_x = 0;        // Center point X (typically ~2048 for 12-bit ADC)
    js->center_y = 0;        // Center point Y
    js->range_x = 1;         // Full-scale range X (prevents divide-by-zero)
    js->range_y = 1;         // Full-scale range Y

    // Normalized output values [-1.0, +1.0]
    js->norm_x = 0;
    js->norm_y = 0;

    // Deadband threshold (5% = ignore values within ±0.05 of center)
    js->deadband = 0.05f;

    // Auto-calibration parameters
    js->samples_collected = 0;   // Count of samples collected
    js->samples_needed = 200;    // Learn center over first 200 samples (~2-5 sec)

    // Assign update function pointer (enables polymorphism)
    js->update = joystick_hal_update;
}

/**
 * @brief Update joystick state from raw ADC values
 *
 * Processing Pipeline:
 *
 * 1. CENTER CORRECTION
 *    - Subtracts the known center point from raw values
 *    - Converts 0-4095 ADC range to centered -2048 to +2047 range
 *    - Example: If center = 2048 and raw = 3000:
 *      dx = 3000 - 2048 = +952 (joystick pushed right)
 *
 * 2. NORMALIZATION
 *    - Divides by range to get -1.0 to +1.0 scale
 *    - Example: 952 / 2048 = 0.465 (46.5% of full deflection)
 *    - This makes subsequent math independent of ADC resolution
 *
 * 3. CLAMPING
 *    - Prevents values exceeding ±1.0 due to:
 *      * ADC noise spikes
 *      * Mechanical over-travel
 *      * Calibration errors
 *
 * 4. ASYMMETRIC DEADBAND
 *    - X-axis (steering): 15% deadband (reduces twitchy turns)
 *    - Y-axis (throttle): 8% deadband (preserves speed sensitivity)
 *    - Different axes have different ergonomic requirements!
 *
 * Why Asymmetric Deadband?
 * - Steering (X): Users rest thumb on stick, causing drift
 *   Large deadband prevents unwanted turning
 * - Throttle (Y): Users actively control speed
 *   Small deadband preserves fine speed control
 *
 * Fixed Calibration Constants:
 * This implementation uses compile-time constants rather than auto-learning:
 * - JS_CENTER_X, JS_CENTER_Y: Measured joystick center (typically 1020-2048)
 * - JS_RANGE_X, JS_RANGE_Y: Full deflection range (typically 1000-2000)
 *
 * @param js Pointer to joystick HAL structure
 * @param x_raw Raw ADC value from X-axis (typically 0-4095 for 12-bit ADC)
 * @param y_raw Raw ADC value from Y-axis (typically 0-4095 for 12-bit ADC)
 *
 * @note Results stored in js->norm_x and js->norm_y (range: -1.0 to +1.0)
 */
void joystick_hal_update(joystick_hal_t *js, int32_t x_raw, int32_t y_raw)
{
    // Store raw values for debugging/telemetry
    js->raw_x = x_raw;
    js->raw_y = y_raw;

    // ═══════════════════════════════════════════════════════════
    // STEP 1: Center Correction
    // ═══════════════════════════════════════════════════════════
    // Convert raw ADC values to centered coordinates
    // Example: If JS_CENTER_X = 2048 (typical for 12-bit ADC):
    //   - Raw = 2048 → dx = 0 (centered)
    //   - Raw = 4095 → dx = +2047 (full right)
    //   - Raw = 0    → dx = -2048 (full left)
    float dx = (float)x_raw - JS_CENTER_X;
    float dy = (float)y_raw - JS_CENTER_Y;

    // ═══════════════════════════════════════════════════════════
    // STEP 2: Normalization to [-1, +1] Range
    // ═══════════════════════════════════════════════════════════
    // Divide by range to get unit-scale values
    // Example: If JS_RANGE_X = 2048 and dx = 1024:
    //   nx = 1024 / 2048 = 0.5 (50% deflection to right)
    float nx = dx / JS_RANGE_X;
    float ny = dy / JS_RANGE_Y;

    // ═══════════════════════════════════════════════════════════
    // STEP 3: Clamping to Valid Range
    // ═══════════════════════════════════════════════════════════
    // Prevent values outside [-1, +1] due to:
    // - Calibration errors (center/range not perfectly accurate)
    // - ADC noise (random fluctuations)
    // - Mechanical issues (stick pushed past normal limits)
    if (nx > 1.0f) nx = 1.0f;
    if (nx < -1.0f) nx = -1.0f;
    if (ny > 1.0f) ny = 1.0f;
    if (ny < -1.0f) ny = -1.0f;

    // ═══════════════════════════════════════════════════════════
    // STEP 4: Asymmetric Deadband Application
    // ═══════════════════════════════════════════════════════════
    // Different deadband thresholds for X and Y axes

    const float deadband_x = 0.15f;  // 15% steering deadband (wide zone)
    const float deadband_y = 0.08f;  // 8% throttle deadband (narrow zone)

    // Why 15% for X (steering)?
    // - Steering is very sensitive to drift
    // - Users rest their thumb on the stick
    // - Wide deadband prevents unwanted turning
    // - Sacrifice: Slightly delayed turn response (acceptable trade-off)

    // Why 8% for Y (throttle)?
    // - Speed control needs precision
    // - Users actively move stick for speed changes
    // - Narrow deadband preserves fine speed control
    // - Benefit: Smooth acceleration/deceleration

    js->norm_x = (fabsf(nx) < deadband_x) ? 0.0f : nx;
    js->norm_y = (fabsf(ny) < deadband_y) ? 0.0f : ny;

    // Result: Clean, drift-free normalized values ready for mixing
}

/**
 * @brief Clamp a float to a specified range
 *
 * Simple utility for range limiting. Equivalent to:
 *   return fminf(fmaxf(val, min), max);
 *
 * But written explicitly for clarity and to avoid extra function calls.
 *
 * @param val Value to clamp
 * @param min Minimum allowed value
 * @param max Maximum allowed value
 * @return Clamped value within [min, max]
 */
static float clampf(float val, float min, float max) {
    return (val < min) ? min : (val > max) ? max : val;
}

/**
 * @brief Convert joystick X/Y coordinates to differential drive PWM values
 *
 * This is the "magic" function that makes intuitive robot control work.
 * It implements a sophisticated mixing algorithm with:
 *
 * 1. NON-LINEAR RESPONSE CURVE
 *    Uses cubic (x³) shaping for steering input
 *    - Near center: x³ is very small → gentle arcs
 *    - Near edges: x³ approaches ±1 → sharp turns
 *    - Eliminates the "twitchy" feeling of linear steering
 *
 * 2. DIFFERENTIAL MIXING
 *    Classic tank/skid-steer control:
 *    - Y (forward/back) applied equally to both motors
 *    - X (steering) added to one side, subtracted from other
 *    - Result: Natural feeling arcs and spins
 *
 * 3. ARC LIMITING
 *    Prevents overly aggressive turns that:
 *    - Cause tire slippage
 *    - Draw excessive current
 *    - Feel "snappy" and uncontrollable
 *
 * 4. PWM SCALING
 *    Converts normalized [-1, +1] to motor PWM range [0, 8191]
 *
 * Mathematical Breakdown:
 *
 * Given joystick inputs:
 *   x ∈ [-1, +1] (left/right, steering)
 *   y ∈ [-1, +1] (forward/back, throttle)
 *
 * Step 1: Shape steering with cubic curve
 *   x_shaped = x³
 *
 * Step 2: Apply steering gain
 *   x_scaled = k × x_shaped  (where k = 0.9)
 *
 * Step 3: Differential mix
 *   L = y + x_scaled  (left motor)
 *   R = y - x_scaled  (right motor)
 *
 * Step 4: Limit turning aggression
 *   diff = |L - R|
 *   if diff > max_diff: scale both down proportionally
 *
 * Step 5: Clamp to valid range
 *   L, R ∈ [-1, +1]
 *
 * Step 6: Scale to PWM
 *   pwm_left = L × 8190
 *   pwm_right = R × 8190
 *
 * Example Scenarios:
 *
 * A. Full forward (x=0, y=1):
 *    x_shaped = 0, L = 1, R = 1
 *    → Both motors full forward, straight line
 *
 * B. Gentle right arc (x=0.3, y=0.8):
 *    x_shaped = 0.027, x_scaled = 0.024
 *    L = 0.824, R = 0.776
 *    → Right motor slightly slower, smooth arc
 *
 * C. Sharp right turn (x=0.8, y=0.3):
 *    x_shaped = 0.512, x_scaled = 0.461
 *    L = 0.761, R = -0.161
 *    → Right motor reverses, tight turn
 *
 * D. Spin in place (x=1, y=0):
 *    x_shaped = 1, x_scaled = 0.9
 *    L = 0.9, R = -0.9
 *    → Equal opposite speeds, pivot turn
 *
 * @param x Normalized X input [-1.0, +1.0] (left = negative, right = positive)
 * @param y Normalized Y input [-1.0, +1.0] (back = negative, forward = positive)
 * @param pwm_left Output: Left motor PWM [-8190, +8190]
 * @param pwm_right Output: Right motor PWM [-8190, +8190]
 *
 * @note PWM range is ±8190 (not ±8191) to prevent overflow in some motor drivers
 */
void joystick_mix(float x, float y, int *pwm_left, int *pwm_right)
{
    // ═══════════════════════════════════════════════════════════
    // STEP 1: Non-linear Steering Curve (Cubic Expo)
    // ═══════════════════════════════════════════════════════════
    float x_shaped = x * x * x;

    // Cubic (x³) Response Curve Analysis:
    //
    // Input  | Output | Effect
    // -------|--------|----------------------------------
    //  0.0   |  0.000 | No steering (centered)
    //  0.1   |  0.001 | Nearly imperceptible turn
    //  0.2   |  0.008 | Very gentle arc
    //  0.3   |  0.027 | Mild turn
    //  0.5   |  0.125 | Moderate arc
    //  0.7   |  0.343 | Strong turn
    //  0.9   |  0.729 | Sharp turn
    //  1.0   |  1.000 | Maximum steering
    //
    // Why Cubic?
    // - Linear (x): Too sensitive near center, feels "twitchy"
    // - Quadratic (x²): Better, but loses sign information
    // - Cubic (x³): Perfect! Smooth near center, responsive at edges
    // - Higher powers (x⁵, x⁷): Too aggressive, "dead zone" too large
    //
    // Alternative curves (commented out):
    //   x * fabsf(x) = x²·sign(x) → Quadratic expo (softer than cubic)
    //   x³ × 1.2 → Cubic with gain (more aggressive turning)

    // ═══════════════════════════════════════════════════════════
    // STEP 2: Steering Gain
    // ═══════════════════════════════════════════════════════════
    const float k = 0.9f;

    // Steering gain (k) controls turn aggressiveness:
    //
    // k value | Effect
    // --------|--------------------------------------------
    //   0.3   | Very soft, wide arcs only
    //   0.5   | Gentle steering, limited spin capability
    //   0.7   | Moderate, good for racing
    //   0.9   | Aggressive, strong spins (YOUR CHOICE)
    //   1.2   | Very aggressive, can be hard to control
    //
    // Your choice of 0.9 provides:
    // - Strong turning authority (can do 360° spins)
    // - Still controllable (not too "snappy")
    // - Good balance for general-purpose robot

    // ═══════════════════════════════════════════════════════════
    // STEP 3: Differential Mixing
    // ═══════════════════════════════════════════════════════════
    float L0 = y + k * x_shaped;
    float R0 = y - k * x_shaped;

    // Classic Tank Drive Formula:
    // - Both motors get the Y (throttle) component equally
    // - Steering component is ADDED to left, SUBTRACTED from right
    // - This creates the speed differential that causes turning
    //
    // Physics: When left motor spins faster than right,
    // the robot rotates clockwise (turns right)

    // ═══════════════════════════════════════════════════════════
    // STEP 4: Arc Limiting (Differential Limiter)
    // ═══════════════════════════════════════════════════════════
    float diff = fabsf(L0 - R0);
    float max_diff = 1.7f;

    // Why Limit the Differential?
    //
    // Problem: Without limiting, extreme joystick inputs can create
    // huge speed differences. Example:
    //   y = 0.5, x = 1.0 → L = 1.4, R = -0.4
    //   Both values get clamped: L = 1.0, R = -0.4
    //   This is NOT what we wanted! The ratio is destroyed.
    //
    // Solution: Limit |L - R| to max_diff BEFORE clamping
    //   If diff > max_diff: scale both L and R proportionally
    //   This preserves the ratio while preventing excessive turn rates
    //
    // max_diff values:
    //   1.0 → Very limited turning (60% of max)
    //   1.2 → Conservative (used previously, felt too weak)
    //   1.7 → Aggressive (YOUR CHOICE, allows ~85% max turn)
    //   2.0 → No limiting (can cause sharp, jarring movements)
    //
    // Your choice of 1.7 allows strong spins while keeping smooth arcs

    if (diff > max_diff) {
        float scale = max_diff / diff;
        L0 *= scale;
        R0 *= scale;
    }

    // Example: If diff = 2.0 and max_diff = 1.7:
    //   scale = 1.7 / 2.0 = 0.85
    //   Both L0 and R0 are multiplied by 0.85
    //   New diff = 1.7 (exactly at the limit)
    //   The SHAPE of the turn is preserved, just scaled down

    // ═══════════════════════════════════════════════════════════
    // STEP 5: Final Clamping
    // ═══════════════════════════════════════════════════════════
    // Ensure values are in valid range for PWM conversion
    // Note: No normalization here! We already scaled via diff limiting
    if (L0 > 1.0f) L0 = 1.0f;
    if (L0 < -1.0f) L0 = -1.0f;
    if (R0 > 1.0f) R0 = 1.0f;
    if (R0 < -1.0f) R0 = -1.0f;

    // ═══════════════════════════════════════════════════════════
    // STEP 6: Scale to PWM Range
    // ═══════════════════════════════════════════════════════════
    // Convert normalized [-1, +1] to motor controller PWM [0, 8191]
    // Using 8190 instead of 8191 to avoid potential overflow in some drivers
    *pwm_left  = (int)(L0 * 8190.0f);
    *pwm_right = (int)(R0 * 8190.0f);

    // Final output range: -8190 to +8190
    // - Positive = forward
    // - Negative = reverse
    // - Magnitude = speed (0 = stop, 8190 = full speed)
}

```

## subsystems/controls/joystick.h <a name="subsystems-controls-joystick-h"></a>

```c
#ifndef JOYSTICK_H
#define JOYSTICK_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#define JS_CENTER_X  63668224.0f   // average of your two X centers
#define JS_CENTER_Y  66912256.0f
#define JS_RANGE_X   20000000.0f   // temp guess, we’ll refine
#define JS_RANGE_Y   20000000.0f


typedef struct joystick_hal_t joystick_hal_t;

struct joystick_hal_t {
    // Raw values from ESP-NOW or ADC
    int32_t raw_x;
    int32_t raw_y;

    // Learned calibration
    float center_x;
    float center_y;
    float range_x;
    float range_y;

    // Filtered normalized output [-1..+1]
    float norm_x;
    float norm_y;

    // Deadband threshold
    float deadband;

    // Calibration state
    int samples_collected;
    int samples_needed;

    // Update function
    void (*update)(joystick_hal_t *self, int32_t x_raw, int32_t y_raw);
};

// Public API
void joystick_hal_init(joystick_hal_t *js);
void joystick_hal_update(joystick_hal_t *js, int32_t x_raw, int32_t y_raw);
void joystick_mix(float x, float y, int *pwm_left, int *pwm_right);


#endif

```

## subsystems/ui/CMakeLists.txt <a name="subsystems-ui-cmakelists-txt"></a>

```c
idf_component_register(
    SRCS "ui.c"
    INCLUDE_DIRS "."
    REQUIRES freertos esp_driver_gpio)

```

## subsystems/ui/ui.c <a name="subsystems-ui-ui-c"></a>

```c
#include "ui.h"
#include "esp_log.h"
#include "freertos/queue.h"

static const char *TAG = "UI";

#define ESP_INTR_FLAG_DEFAULT 0

static QueueHandle_t gpio_evt_queue = NULL;

static void IRAM_ATTR gpio_isr_handler(void *arg) {
    uint32_t gpio_num = (uint32_t)arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void gpio_task(void *arg) {
    uint32_t io_num;
    while (1) {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            ESP_LOGI(TAG, "Button GPIO[%lu] pressed, val: %d",
                     io_num, gpio_get_level(io_num));
        }
    }
}

static void ui_update_impl(ui_system_t *self, TickType_t now) {
    static TickType_t last_blink = 0;

    // Blink LED every 500ms
    if ((now - last_blink) >= pdMS_TO_TICKS(500)) {
        self->led_state = !self->led_state;
        gpio_set_level(BLINK_GPIO, self->led_state);
        last_blink = now;
    }
}

void ui_system_init(ui_system_t *sys) {
    sys->led_state = 0;
    sys->update = ui_update_impl;

    // Configure LED
    gpio_reset_pin(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

    // Configure buttons
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_NEGEDGE,
        .pin_bit_mask = (1ULL << PUSH_BTN_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = 1,
    };
    gpio_config(&io_conf);

    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    xTaskCreate(gpio_task, "gpio_task", 2048, NULL, 10, NULL);

    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(PUSH_BTN_GPIO, gpio_isr_handler, (void *)PUSH_BTN_GPIO);

    ESP_LOGI(TAG, "UI system initialized");
}

```

## subsystems/ui/ui.h <a name="subsystems-ui-ui-h"></a>

```c
#ifndef UI_H
#define UI_H

#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"

#define BLINK_GPIO 10
#define PUSH_BTN_GPIO 8
#define NAV_BTN_GPIO 8

// Forward declaration
typedef struct ui_system_t ui_system_t;

// Struct definition
struct ui_system_t {
    uint8_t led_state;

    void (*update)(ui_system_t *self, TickType_t now);
};

void ui_system_init(ui_system_t *sys);

#endif

```

## subsystems/connectivity/CMakeLists.txt <a name="subsystems-connectivity-cmakelists-txt"></a>

```c

```

## subsystems/connectivity/mqtt_sys/CMakeLists.txt <a name="subsystems-connectivity-mqtt-sys-cmakelists-txt"></a>

```c
idf_component_register(
    SRCS "mqtt_sys.c"
    INCLUDE_DIRS "."
    REQUIRES mqtt
)

```

## subsystems/connectivity/mqtt_sys/mqtt_sys.c <a name="subsystems-connectivity-mqtt-sys-mqtt-sys-c"></a>

```c
#include "mqtt_sys.h"
#include "esp_log.h"
#include <stdio.h>

static const char *TAG = "MQTT_SYS";

static void mqtt_publish_task(void *arg) {
    mqtt_system_t *sys = (mqtt_system_t *)arg;

    char temp_str[16], volt_str[16], curr_str[16], pwr_str[16];
    char pwm_l_str[16], pwm_r_str[16];
    char prox_str[16];

    while (1) {
        snprintf(temp_str, sizeof(temp_str), "%.2f", sys->temp_value);
        snprintf(volt_str, sizeof(volt_str), "%.2f", sys->battery_voltage);
        snprintf(curr_str, sizeof(curr_str), "%.2f", sys->sys_current);
        snprintf(pwr_str, sizeof(pwr_str), "%.2f", sys->sys_power);
        snprintf(pwm_l_str, sizeof(pwm_l_str), "%d", sys->pwm_left);
        snprintf(pwm_r_str, sizeof(pwm_r_str), "%d", sys->pwm_right);
        snprintf(prox_str, sizeof(prox_str), "%.2f", sys->proximity);

        esp_mqtt_client_publish(sys->client, "/bitrider/temp", temp_str, 0, 1, 0);
        esp_mqtt_client_publish(sys->client, "/bitrider/battery_voltage", volt_str, 0, 1, 0);
        esp_mqtt_client_publish(sys->client, "/bitrider/sys_current", curr_str, 0, 1, 0);
        esp_mqtt_client_publish(sys->client, "/bitrider/sys_power", pwr_str, 0, 1, 0);
        esp_mqtt_client_publish(sys->client, "/bitrider/pwm_left", pwm_l_str, 0, 1, 0);
        esp_mqtt_client_publish(sys->client, "/bitrider/pwm_right", pwm_r_str, 0, 1, 0);
        esp_mqtt_client_publish(sys->client, "/bitrider/proximity", prox_str, 0, 1, 0);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                                int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t event = event_data;
    mqtt_system_t *sys = (mqtt_system_t *)handler_args;

    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT Connected");
            sys->client = event->client;
            xTaskCreate(mqtt_publish_task, "mqtt_pub", 4096, sys, 5, NULL);
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT Disconnected");
            break;
        default:
            break;
    }
}

static void mqtt_update_impl(mqtt_system_t *self, TickType_t now) {
    (void)self;
    (void)now;
    // Publishing happens in the task
}

void mqtt_system_init(mqtt_system_t *sys) {
    sys->temp_value = 0.0f;
    sys->battery_voltage = 0.0f;
    sys->sys_current = 0.0f;
    sys->sys_power = 0.0f;
    sys->pwm_left = 0;
    sys->pwm_right = 0;
    sys->client = NULL;
    sys->proximity = 0.0f;
    sys->update = mqtt_update_impl;

    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTT_BROKER_URI,
    };

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, sys);
    esp_mqtt_client_start(client);

    ESP_LOGI(TAG, "MQTT system initialized");
}

void mqtt_update_temp(mqtt_system_t *sys, float temp) {
    sys->temp_value = temp;
}

void mqtt_update_battery(mqtt_system_t *sys, float voltage) {
    sys->battery_voltage = voltage;
}

void mqtt_update_current(mqtt_system_t *sys, float current) {
    sys->sys_current = current;
}

void mqtt_update_power(mqtt_system_t *sys, float power) {
    sys->sys_power = power;
}

void mqtt_update_pwm(mqtt_system_t *sys, int left, int right) {
    sys->pwm_left = left;
    sys->pwm_right = right;
}

void mqtt_update_proximity(mqtt_system_t *sys, float proximity) {
    sys->proximity = proximity;
}

```

## subsystems/connectivity/mqtt_sys/mqtt_sys.h <a name="subsystems-connectivity-mqtt-sys-mqtt-sys-h"></a>

```c
#ifndef MQTT_SYS_H
#define MQTT_SYS_H

#include "freertos/FreeRTOS.h"
#include "mqtt_client.h"

#define MQTT_BROKER_URI "mqtt://74.14.210.168"
#define WIFI_SSID "IoT_bots"
#define WIFI_PASSWORD "208208208"

// Forward declaration
typedef struct mqtt_system_t mqtt_system_t;

// Struct definition
struct mqtt_system_t {
    float temp_value;
    float battery_voltage;
    float sys_current;
    float sys_power;
    int pwm_left;
    int pwm_right;
    float proximity;

    esp_mqtt_client_handle_t client;

    void (*update)(mqtt_system_t *self, TickType_t now);
};

void mqtt_system_init(mqtt_system_t *sys);
void mqtt_update_temp(mqtt_system_t *sys, float temp);
void mqtt_update_battery(mqtt_system_t *sys, float voltage);
void mqtt_update_current(mqtt_system_t *sys, float current);
void mqtt_update_power(mqtt_system_t *sys, float power);
void mqtt_update_pwm(mqtt_system_t *sys, int left, int right);
void mqtt_update_proximity(mqtt_system_t *sys, float power);

#endif  // <-- This was missing!

```

## subsystems/connectivity/wifi_sys/CMakeLists.txt <a name="subsystems-connectivity-wifi-sys-cmakelists-txt"></a>

```c
idf_component_register(
    SRCS "wifi_sys.c"
    INCLUDE_DIRS "."
    REQUIRES esp_wifi esp_netif
)

```

## subsystems/connectivity/wifi_sys/wifi_sys.c <a name="subsystems-connectivity-wifi-sys-wifi-sys-c"></a>

```c
#include "wifi_sys.h"
#include "esp_log.h"
#include "esp_netif.h"

static const char *TAG = "WIFI_SYS";

void wifi_system_init(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_connect());

    // Give WiFi time to connect
        vTaskDelay(pdMS_TO_TICKS(3000));

        // Log connection info
        uint8_t channel;
        wifi_second_chan_t second;
        esp_wifi_get_channel(&channel, &second);

        uint8_t mac[6];
        esp_wifi_get_mac(WIFI_IF_STA, mac);

        ESP_LOGI(TAG, "WiFi initialized and connecting...");
        ESP_LOGI(TAG, "SSID: %s", WIFI_SSID);
        ESP_LOGW(TAG, "WiFi Channel: %d", channel);
        ESP_LOGW(TAG, "Receiver MAC: %02X:%02X:%02X:%02X:%02X:%02X",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        ESP_LOGW(TAG, "Transmitter will scan channels to find receiver");
}

```

## subsystems/connectivity/wifi_sys/wifi_sys.h <a name="subsystems-connectivity-wifi-sys-wifi-sys-h"></a>

```c
#ifndef WIFI_SYS_H
#define WIFI_SYS_H

#include "esp_wifi.h"
#include "esp_event.h"

#define WIFI_SSID "IoT_bots"
#define WIFI_PASSWORD "208208208"
#define ESPNOW_CHANNEL 1

void wifi_system_init(void);

#endif

```

## subsystems/connectivity/espnow_sys/CMakeLists.txt <a name="subsystems-connectivity-espnow-sys-cmakelists-txt"></a>

```c
idf_component_register(
    SRCS "espnow_sys.c"
    INCLUDE_DIRS "."
    REQUIRES esp_wifi
)

```

## subsystems/connectivity/espnow_sys/espnow_sys.c <a name="subsystems-connectivity-espnow-sys-espnow-sys-c"></a>

```c
#include "espnow_sys.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "ESPNOW_SYS";
static espnow_system_t *g_sys = NULL;

static void espnow_recv_cb(const esp_now_recv_info_t *recv_info,
                           const uint8_t *data,
                           int len)
{
    if (!g_sys || len <= 0 || len > sizeof(sensors_data_t)) {
        ESP_LOGI(TAG, "Invalid data: len=%d (expected %d)", len, sizeof(sensors_data_t));
        return;
    }

    memcpy(&g_sys->last_data, data, len);
    g_sys->last_len = len;

    // Enhanced logging
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "ESP-NOW DATA RECEIVED");
    ESP_LOGI(TAG, "X-axis (rc_x): %d", g_sys->last_data.x_axis);
    ESP_LOGI(TAG, "Y-axis (rc_y): %d", g_sys->last_data.y_axis);
    ESP_LOGI(TAG, "M1: %d, M2: %d, M3: %d, M4: %d",
             g_sys->last_data.motor1_rpm_pcm,
             g_sys->last_data.motor2_rpm_pcm,
             g_sys->last_data.motor3_rpm_pcm,
             g_sys->last_data.motor4_rpm_pcm);
    ESP_LOGI(TAG, "Data length: %d bytes", len);
    ESP_LOGI(TAG, "========================================");
}

static void espnow_send_impl(espnow_system_t *self,
                             const uint8_t *data,
                             int len)
{
    if (!data || len <= 0) return;
    esp_now_send(NULL, data, len);
}

static void espnow_update_impl(espnow_system_t *self, TickType_t now)
{
    (void)self;
    (void)now;
}

void espnow_system_init(espnow_system_t *sys)
{
    memset(&sys->last_data, 0, sizeof(sys->last_data));
    sys->last_len = 0;
    sys->send = espnow_send_impl;
    sys->update = espnow_update_impl;

    g_sys = sys;

    ESP_LOGI(TAG, "Initializing ESP-NOW...");

    esp_err_t ret = esp_now_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ESP-NOW init failed: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "ESP-NOW init success");

    ret = esp_now_register_recv_cb(espnow_recv_cb);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ESP-NOW register callback failed: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "ESP-NOW callback registered");

    // Print MAC address
    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    ESP_LOGW(TAG, "Receiver MAC: %02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    ESP_LOGI(TAG, "ESP-NOW initialized - waiting for data from transmitter...");
    ESP_LOGI(TAG, "Expected data size: %d bytes", sizeof(sensors_data_t));
}

```

## subsystems/connectivity/espnow_sys/espnow_sys.h <a name="subsystems-connectivity-espnow-sys-espnow-sys-h"></a>

```c
#ifndef ESPNOW_SYS_H
#define ESPNOW_SYS_H

#include "freertos/FreeRTOS.h"
#include "esp_now.h"
#include "esp_wifi.h"

// Match your original structure EXACTLY
typedef struct {
    int x_axis;
    int y_axis;
    int motor1_rpm_pcm;
    int motor2_rpm_pcm;
    int motor3_rpm_pcm;
    int motor4_rpm_pcm;
} sensors_data_t;

// Forward declaration
typedef struct espnow_system_t espnow_system_t;

// Struct definition
struct espnow_system_t {
    sensors_data_t last_data;
    int last_len;

    void (*send)(espnow_system_t *self, const uint8_t *data, int len);
    void (*update)(espnow_system_t *self, TickType_t now);
};

void espnow_system_init(espnow_system_t *sys);

#endif

```
