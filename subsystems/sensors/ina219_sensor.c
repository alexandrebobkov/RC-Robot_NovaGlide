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
