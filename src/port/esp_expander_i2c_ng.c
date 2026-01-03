/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include "port/esp_expander_i2c_ng.h"

#include <stdbool.h>
#include <string.h>

#include "esp_check.h"

// Arduino-ESP32 driver_ng I2C HAL symbols (provided by esp32-hal-i2c-ng.c)
#ifdef __cplusplus
extern "C" {
#endif
bool i2cInit(uint8_t i2c_num, int sda, int scl, uint32_t frequency);
void *i2cBusHandle(uint8_t i2c_num);
#ifdef __cplusplus
}
#endif

static uint32_t s_port_speed_hz[I2C_NUM_MAX] = {0};

// If the host init is skipped (shared bus initialized elsewhere), we may not
// know the actual bus speed here. Default to a conservative value to improve
// reliability on electrically challenging buses.
#ifndef ESP_EXPANDER_I2C_DEFAULT_SPEED_HZ
#define ESP_EXPANDER_I2C_DEFAULT_SPEED_HZ (100000)
#endif

esp_err_t esp_expander_i2c_ng_init_from_legacy_config(i2c_port_t port, const i2c_config_t *cfg)
{
    ESP_RETURN_ON_FALSE(cfg != NULL, ESP_ERR_INVALID_ARG, "exp_i2c_ng", "cfg is null");
    ESP_RETURN_ON_FALSE(port < I2C_NUM_MAX, ESP_ERR_INVALID_ARG, "exp_i2c_ng", "invalid port");

    // Cache speed for later device additions
    if (cfg->mode == I2C_MODE_MASTER) {
        s_port_speed_hz[port] = cfg->master.clk_speed;
    }

    // Initialize via Arduino core (driver_ng)
    bool ok = i2cInit((uint8_t)port, cfg->sda_io_num, cfg->scl_io_num, cfg->master.clk_speed);
    return ok ? ESP_OK : ESP_FAIL;
}

uint32_t esp_expander_i2c_ng_get_speed_hz(i2c_port_t port)
{
    if (port >= I2C_NUM_MAX) {
        return 0;
    }
    return s_port_speed_hz[port];
}

esp_err_t esp_expander_i2c_ng_add_device(i2c_port_t port, uint16_t addr7, i2c_master_dev_handle_t *out_dev)
{
    ESP_RETURN_ON_FALSE(out_dev != NULL, ESP_ERR_INVALID_ARG, "exp_i2c_ng", "out_dev is null");
    ESP_RETURN_ON_FALSE(port < I2C_NUM_MAX, ESP_ERR_INVALID_ARG, "exp_i2c_ng", "invalid port");

    i2c_master_bus_handle_t bus = (i2c_master_bus_handle_t)i2cBusHandle((uint8_t)port);
    ESP_RETURN_ON_FALSE(bus != NULL, ESP_ERR_INVALID_STATE, "exp_i2c_ng", "bus not initialized");

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr7,
        .scl_speed_hz = (int)(s_port_speed_hz[port] ? s_port_speed_hz[port] : ESP_EXPANDER_I2C_DEFAULT_SPEED_HZ),
    };
    return i2c_master_bus_add_device(bus, &dev_cfg, out_dev);
}

esp_err_t esp_expander_i2c_ng_deinit(i2c_port_t port)
{
    // Do not deinit here: buses are typically shared (Wire/other devices).
    // Keep symbol for API compatibility.
    (void)port;
    return ESP_OK;
}
