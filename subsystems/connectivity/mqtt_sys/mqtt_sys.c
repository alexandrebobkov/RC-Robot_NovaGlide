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
            ESP_LOGI(TAG, "MQTT Connected successfully");
            ESP_LOGI(TAG, "Session present: %d", event->session_present);
            sys->client = event->client;
            xTaskCreate(mqtt_publish_task, "mqtt_pub", 4096, sys, 5, NULL);
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT Disconnected");
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGE(TAG, "MQTT error type: %d", event->error_handle->error_type);
            if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
                ESP_LOGE(TAG, "  esp-tls error: 0x%x", event->error_handle->esp_tls_last_esp_err);
                ESP_LOGE(TAG, "  tls stack error: 0x%x", event->error_handle->esp_tls_stack_err);
                ESP_LOGE(TAG, "  socket errno: %d (%s)",
                event->error_handle->esp_transport_sock_errno,
                strerror(event->error_handle->esp_transport_sock_errno));
            } else if (event->error_handle->error_type == MQTT_ERROR_TYPE_CONNECTION_REFUSED) {
                ESP_LOGE(TAG, "  connection refused, return code: %d",
                event->error_handle->connect_return_code);
            }
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
        .credentials.username = MQTT_USERNAME,
        .credentials.authentication.password = MQTT_PASSWORD,
    };

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    if (client == NULL) {
        ESP_LOGE(TAG, "Failed to initialize MQTT client");
        return;
    }

    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, sys);
    esp_err_t err = esp_mqtt_client_start(client);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start MQTT client: %s", esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, "MQTT system initialized with authentication");
    ESP_LOGI(TAG, "Broker: %s", MQTT_BROKER_URI);
    ESP_LOGI(TAG, "Username: %s", MQTT_USERNAME);
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
