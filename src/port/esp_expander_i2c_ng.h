/*
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "esp_err.h"
#include "esp_idf_version.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Minimal driver_ng bridge for ESP32_IO_Expander
 *
 * This is used to avoid linking the legacy I2C driver (driver/i2c.h APIs like
 * i2c_driver_install / i2c_master_write_to_device) when building against
 * ESP-IDF versions where Arduino/Wire uses the new I2C driver (driver_ng).
 */
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)

#include "driver/i2c.h"          // for i2c_port_t, i2c_config_t
#include "driver/i2c_master.h"   // driver_ng APIs

esp_err_t esp_expander_i2c_ng_init_from_legacy_config(i2c_port_t port, const i2c_config_t *cfg);
esp_err_t esp_expander_i2c_ng_deinit(i2c_port_t port);

esp_err_t esp_expander_i2c_ng_add_device(i2c_port_t port, uint16_t addr7, i2c_master_dev_handle_t *out_dev);
uint32_t esp_expander_i2c_ng_get_speed_hz(i2c_port_t port);

#else

/* Stubs for older ESP-IDF */
static inline esp_err_t esp_expander_i2c_ng_init_from_legacy_config(int port, const void *cfg) { (void)port; (void)cfg; return ESP_ERR_NOT_SUPPORTED; }
static inline esp_err_t esp_expander_i2c_ng_deinit(int port) { (void)port; return ESP_ERR_NOT_SUPPORTED; }

#endif

#ifdef __cplusplus
}
#endif
