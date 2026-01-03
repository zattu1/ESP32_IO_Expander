/*
 * SPDX-FileCopyrightText: 2015-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <inttypes.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"

#include "driver/i2c_master.h"
#include "esp_idf_version.h"
#include "esp_bit_defs.h"
#include "esp_check.h"
#include "esp_log.h"

#include "esp_io_expander.h"
#include "esp_io_expander_tca95xx_16bit.h"

#include "esp_expander_utils.h"
#include "esp_expander_i2c_ng.h"

/* Timeout of each I2C communication */
#define I2C_TIMEOUT_MS          (150)

// Arduino-ESP32 driver_ng I2C HAL symbol (provided by esp32-hal-i2c-ng.c)
#ifdef __cplusplus
extern "C" {
#endif
void *i2cBusHandle(uint8_t i2c_num);
#ifdef __cplusplus
}
#endif

static inline void reset_i2c_bus_on_timeout(i2c_port_t port)
{
    i2c_master_bus_handle_t bus = (i2c_master_bus_handle_t)i2cBusHandle((uint8_t)port);
    if (bus != NULL) {
        (void)i2c_master_bus_reset(bus);
    }
}

typedef struct {
    i2c_master_dev_handle_t dev;
    const uint8_t *data;
    size_t len;
} i2c_tx_ctx_t;

static esp_err_t i2c_tx_op(void *p)
{
    i2c_tx_ctx_t *c = (i2c_tx_ctx_t *)p;
    return i2c_master_transmit(c->dev, c->data, c->len, I2C_TIMEOUT_MS);
}

typedef struct {
    i2c_master_dev_handle_t dev;
    uint8_t *data;
    size_t len;
} i2c_rx_ctx_t;

static esp_err_t i2c_rx_op(void *p)
{
    i2c_rx_ctx_t *c = (i2c_rx_ctx_t *)p;
    return i2c_master_receive(c->dev, c->data, c->len, I2C_TIMEOUT_MS);
}

static inline esp_err_t retry_if_timeout(esp_err_t err, TickType_t backoff_ticks)
{
    if (err == ESP_ERR_TIMEOUT) {
        vTaskDelay(backoff_ticks);
    }
    return err;
}

static inline esp_err_t retry_i2c_op_if_timeout(i2c_port_t port, esp_err_t (*op)(void *ctx), void *ctx)
{
    // Retry a few times with a small backoff to tolerate bus contention or
    // temporary clock-stretching. Keep it bounded to avoid blocking too long.
    esp_err_t err = op(ctx);
    if (err != ESP_ERR_TIMEOUT) {
        return err;
    }
    reset_i2c_bus_on_timeout(port);
    (void)retry_if_timeout(err, pdMS_TO_TICKS(2));

    err = op(ctx);
    if (err != ESP_ERR_TIMEOUT) {
        return err;
    }
    reset_i2c_bus_on_timeout(port);
    (void)retry_if_timeout(err, pdMS_TO_TICKS(5));

    err = op(ctx);
    if (err != ESP_ERR_TIMEOUT) {
        return err;
    }
    reset_i2c_bus_on_timeout(port);
    (void)retry_if_timeout(err, pdMS_TO_TICKS(10));

    return op(ctx);
}

#define IO_COUNT                (16)

/* Register address */
#define INPUT_REG_ADDR          (0x00)
#define OUTPUT_REG_ADDR         (0x02)
#define DIRECTION_REG_ADDR      (0x06)

/* Default register value on power-up */
#define DIR_REG_DEFAULT_VAL     (0xffff)
#define OUT_REG_DEFAULT_VAL     (0xffff)

/**
 * @brief Device Structure Type
 */
typedef struct {
    esp_io_expander_t base;
    i2c_port_t i2c_num;
    uint32_t i2c_address;
    i2c_master_dev_handle_t i2c_dev;
    struct {
        uint16_t direction;
        uint16_t output;
    } regs;
} esp_io_expander_tca95xx_16bit_t;

static const char *TAG = "tca95xx_16";

static esp_err_t read_input_reg(esp_io_expander_handle_t handle, uint32_t *value);
static esp_err_t write_output_reg(esp_io_expander_handle_t handle, uint32_t value);
static esp_err_t read_output_reg(esp_io_expander_handle_t handle, uint32_t *value);
static esp_err_t write_direction_reg(esp_io_expander_handle_t handle, uint32_t value);
static esp_err_t read_direction_reg(esp_io_expander_handle_t handle, uint32_t *value);
static esp_err_t reset(esp_io_expander_t *handle);
static esp_err_t del(esp_io_expander_t *handle);

esp_err_t esp_io_expander_new_i2c_tca95xx_16bit(i2c_port_t i2c_num, uint32_t i2c_address, esp_io_expander_handle_t *handle)
{
    ESP_LOGI(TAG, "version: %d.%d.%d", ESP_IO_EXPANDER_TCA95XX_16BIT_VER_MAJOR, ESP_IO_EXPANDER_TCA95XX_16BIT_VER_MINOR,
             ESP_IO_EXPANDER_TCA95XX_16BIT_VER_PATCH);
    ESP_RETURN_ON_FALSE(i2c_num < I2C_NUM_MAX, ESP_ERR_INVALID_ARG, TAG, "Invalid i2c num");
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");

    esp_io_expander_tca95xx_16bit_t *tca = (esp_io_expander_tca95xx_16bit_t *)calloc(1, sizeof(esp_io_expander_tca95xx_16bit_t));
    ESP_RETURN_ON_FALSE(tca, ESP_ERR_NO_MEM, TAG, "Malloc failed");

    esp_err_t ret = ESP_OK;

    tca->base.config.io_count = IO_COUNT;
    tca->base.config.flags.dir_out_bit_zero = 1;
    tca->i2c_num = i2c_num;
    tca->i2c_address = i2c_address;
    tca->i2c_dev = NULL;
    tca->base.read_input_reg = read_input_reg;
    tca->base.write_output_reg = write_output_reg;
    tca->base.read_output_reg = read_output_reg;
    tca->base.write_direction_reg = write_direction_reg;
    tca->base.read_direction_reg = read_direction_reg;
    tca->base.del = del;
    tca->base.reset = reset;

    ESP_GOTO_ON_ERROR(
        esp_expander_i2c_ng_add_device(i2c_num, (uint16_t)i2c_address, &tca->i2c_dev),
        err, TAG, "Add i2c device failed"
    );

    /* Reset configuration and register status */
    ESP_GOTO_ON_ERROR(reset(&tca->base), err, TAG, "Reset failed");

    *handle = &tca->base;
    return ESP_OK;
err:
    free(tca);
    return ret;
}

static esp_err_t read_input_reg(esp_io_expander_handle_t handle, uint32_t *value)
{
    esp_io_expander_tca95xx_16bit_t *tca = (esp_io_expander_tca95xx_16bit_t *)__containerof(handle, esp_io_expander_tca95xx_16bit_t, base);

    uint8_t temp[2] = {0, 0};
    uint8_t reg = INPUT_REG_ADDR;

    // Some Arduino-ESP32 / IDF driver_ng combos have been observed to be flaky with
    // transmit+receive (repeated-start) sequences. Use an explicit write(reg) then read(data)
    // sequence to improve compatibility.
    i2c_tx_ctx_t tx = { tca->i2c_dev, &reg, 1 };
    esp_err_t err = retry_i2c_op_if_timeout(tca->i2c_num, i2c_tx_op, &tx);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Read input reg TX failed [%s]", esp_err_to_name(err));
        return err;
    }

    i2c_rx_ctx_t rx = { tca->i2c_dev, (uint8_t *)&temp, 2 };
    err = retry_i2c_op_if_timeout(tca->i2c_num, i2c_rx_op, &rx);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Read input reg RX failed [%s]", esp_err_to_name(err));
        return err;
    }

    *value = (((uint32_t)temp[1]) << 8) | (temp[0]);
    return ESP_OK;
}

static esp_err_t write_output_reg(esp_io_expander_handle_t handle, uint32_t value)
{
    esp_io_expander_tca95xx_16bit_t *tca = (esp_io_expander_tca95xx_16bit_t *)__containerof(handle, esp_io_expander_tca95xx_16bit_t, base);
    value &= 0xffff;

    uint8_t data[] = {OUTPUT_REG_ADDR, value & 0xff, value >> 8};

    i2c_tx_ctx_t tx = { tca->i2c_dev, data, sizeof(data) };
    esp_err_t err = retry_i2c_op_if_timeout(tca->i2c_num, i2c_tx_op, &tx);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Write output reg failed [%s]", esp_err_to_name(err));
        return err;
    }

    tca->regs.output = value;
    return ESP_OK;
}

static esp_err_t read_output_reg(esp_io_expander_handle_t handle, uint32_t *value)
{
    esp_io_expander_tca95xx_16bit_t *tca = (esp_io_expander_tca95xx_16bit_t *)__containerof(handle, esp_io_expander_tca95xx_16bit_t, base);

    *value = tca->regs.output;
    return ESP_OK;
}

static esp_err_t write_direction_reg(esp_io_expander_handle_t handle, uint32_t value)
{
    esp_io_expander_tca95xx_16bit_t *tca = (esp_io_expander_tca95xx_16bit_t *)__containerof(handle, esp_io_expander_tca95xx_16bit_t, base);
    value &= 0xffff;

    uint8_t data[] = {DIRECTION_REG_ADDR, value & 0xff, value >> 8};

    i2c_tx_ctx_t tx = { tca->i2c_dev, data, sizeof(data) };
    esp_err_t err = retry_i2c_op_if_timeout(tca->i2c_num, i2c_tx_op, &tx);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Write direction reg failed [%s]", esp_err_to_name(err));
        return err;
    }

    tca->regs.direction = value;
    return ESP_OK;
}

static esp_err_t read_direction_reg(esp_io_expander_handle_t handle, uint32_t *value)
{
    esp_io_expander_tca95xx_16bit_t *tca = (esp_io_expander_tca95xx_16bit_t *)__containerof(handle, esp_io_expander_tca95xx_16bit_t, base);

    *value = tca->regs.direction;
    return ESP_OK;
}

static esp_err_t reset(esp_io_expander_t *handle)
{
    ESP_RETURN_ON_ERROR(write_direction_reg(handle, DIR_REG_DEFAULT_VAL), TAG, "Write dir reg failed");
    ESP_RETURN_ON_ERROR(write_output_reg(handle, OUT_REG_DEFAULT_VAL), TAG, "Write output reg failed");
    return ESP_OK;
}

static esp_err_t del(esp_io_expander_t *handle)
{
    esp_io_expander_tca95xx_16bit_t *tca = (esp_io_expander_tca95xx_16bit_t *)__containerof(handle, esp_io_expander_tca95xx_16bit_t, base);
    if (tca->i2c_dev) {
        (void)i2c_master_bus_rm_device(tca->i2c_dev);
        tca->i2c_dev = NULL;
    }
    free(tca);
    return ESP_OK;
}
