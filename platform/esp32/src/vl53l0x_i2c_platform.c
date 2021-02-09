/*
 * File : vl53l0x_i2c_platform.c
 * Created: Friday, 05 February 2021
 * Author: yunsik oh (oyster90@naver.com)
 * 
 * Modified: Friday, 05 February 2021
 * 
 */

#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <rom/ets_sys.h>

#include "vl53l0x_i2c_platform.h"
#include "vl53l0x_platform_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/gpio.h"

#ifdef VL53L0X_LOG_ENABLE
#define trace_print(level, ...) trace_print_module_function(TRACE_MODULE_PLATFORM, level, TRACE_FUNCTION_NONE, ##__VA_ARGS__)
#define trace_i2c(...) trace_print_module_function(TRACE_MODULE_NONE, TRACE_LEVEL_NONE, TRACE_FUNCTION_I2C, ##__VA_ARGS__)
#endif

#define STATUS_OK 0x00
#define STATUS_FAIL 0x01

#define ACK_CHECK_EN true
#define I2C_FLUSH_DELAY (2000 / portTICK_PERIOD_MS)

static i2c_port_t s_vl53l0x_i2c_port = CONFIG_VL53L0X_I2C_NUM;
static int s_vl53l0x_sda_pin = CONFIG_VL53L0X_SDA_PIN;
static int s_vl53l0x_scl_pin = CONFIG_VL53L0X_SCL_PIN;

inline VL53L0X_Error esp_to_vl53l0x_error(esp_err_t esp_err)
{
    switch (esp_err)
    {
    case ESP_OK:                return VL53L0X_ERROR_NONE;
    case ESP_ERR_INVALID_ARG:   return VL53L0X_ERROR_INVALID_PARAMS;
    case ESP_FAIL:
    case ESP_ERR_INVALID_STATE: return VL53L0X_ERROR_CONTROL_INTERFACE;
    case ESP_ERR_TIMEOUT:       return VL53L0X_ERROR_TIME_OUT;
    default:
        return VL53L0X_ERROR_UNDEFINED;
    }
}

int32_t VL53L0X_comms_initialise(uint8_t  comms_type,
                                          uint16_t comms_speed_khz)
{
    #define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
    #define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */

    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = s_vl53l0x_sda_pin; 
    conf.sda_pullup_en = GPIO_PULLUP_DISABLE;
    conf.scl_io_num = s_vl53l0x_scl_pin;
    conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
    conf.master.clk_speed = comms_speed_khz * 1000;

    i2c_param_config(s_vl53l0x_i2c_port, &conf);
    esp_err_t err = i2c_driver_install(s_vl53l0x_i2c_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    if (err != ESP_OK)
    {
#ifdef VL53L0X_LOG_ENABLE
        trace_print(TRACE_LEVEL_ERRORS, "i2c_driver_install failed");
#endif
        return esp_to_vl53l0x_error(err);
    }

#ifdef VL53L0X_LOG_ENABLE
    trace_print(TRACE_LEVEL_INFO, "i2c_driver_install master mode Success");
#endif

    return esp_to_vl53l0x_error(err);
}

int32_t VL53L0X_comms_close(void)
{
    int32_t status = STATUS_OK;

    return status;
}

int32_t VL53L0X_cycle_power(void)
{
    int32_t status = STATUS_OK;

    return status;
}

int32_t VL53L0X_write_multi(uint8_t address, uint8_t index, uint8_t *pdata, int32_t count)
{
    int32_t status = STATUS_OK;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));

    // write I2C address
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN));

    // write register
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, index, ACK_CHECK_EN));

    // Data
    // Note: Needed to use i2c_master_write_byte as i2c_master_write will not expect an ack
    // after each byte
    for (int i = 0; i < count; i++)
    {
        ESP_ERROR_CHECK(i2c_master_write_byte(cmd, *(pdata + i), ACK_CHECK_EN));
    }

    ESP_ERROR_CHECK(i2c_master_stop(cmd));
    esp_err_t err = i2c_master_cmd_begin(s_vl53l0x_i2c_port, cmd, I2C_FLUSH_DELAY);
    i2c_cmd_link_delete(cmd);

    return esp_to_vl53l0x_error(err);
}

int32_t VL53L0X_read_multi(uint8_t address, uint8_t index, uint8_t *pdata, int32_t count)
{
    int32_t status = STATUS_OK;

    // I2C write
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    ////// First tell the VL53L0X which register we are reading from
    ESP_ERROR_CHECK(i2c_master_start(cmd));

    // Write I2C address
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN));
    // Write register
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, index, ACK_CHECK_EN));

    ////// Second, read from the register
    ESP_ERROR_CHECK(i2c_master_start(cmd));

    // Write I2C address
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_READ, ACK_CHECK_EN));

    // Read data from register
    ESP_ERROR_CHECK(i2c_master_read(cmd, pdata, count, I2C_MASTER_LAST_NACK));

    ESP_ERROR_CHECK(i2c_master_stop(cmd));
    esp_err_t err = i2c_master_cmd_begin(s_vl53l0x_i2c_port, cmd, I2C_FLUSH_DELAY);
    i2c_cmd_link_delete(cmd);

    return esp_to_vl53l0x_error(err);
}

int32_t VL53L0X_write_byte(uint8_t address, uint8_t index, uint8_t data)
{
    int32_t status = STATUS_OK;
    const int32_t cbyte_count = 1;

#ifdef VL53L0X_LOG_ENABLE
    trace_print(TRACE_LEVEL_INFO, "Write reg : 0x%02X, Val : 0x%02X\n", index, data);
#endif

    status = VL53L0X_write_multi(address, index, &data, cbyte_count);

    return status;
}

int32_t VL53L0X_write_word(uint8_t address, uint8_t index, uint16_t data)
{
    int32_t status = STATUS_OK;

    uint8_t buffer[BYTES_PER_WORD];

    // Split 16-bit word into MS and LS uint8_t
    buffer[0] = (uint8_t)(data >> 8);
    buffer[1] = (uint8_t)(data & 0x00FF);

    if (index % 2 == 1)
    {
        status = VL53L0X_write_multi(address, index, &buffer[0], 1);
        status = VL53L0X_write_multi(address, index + 1, &buffer[1], 1);
        // serial comms cannot handle word writes to non 2-byte aligned registers.
    }
    else
    {
        status = VL53L0X_write_multi(address, index, buffer, BYTES_PER_WORD);
    }

    return status;
}

int32_t VL53L0X_write_dword(uint8_t address, uint8_t index, uint32_t data)
{
    int32_t status = STATUS_OK;
    uint8_t buffer[BYTES_PER_DWORD];

    // Split 32-bit word into MS ... LS bytes
    buffer[0] = (uint8_t)(data >> 24);
    buffer[1] = (uint8_t)((data & 0x00FF0000) >> 16);
    buffer[2] = (uint8_t)((data & 0x0000FF00) >> 8);
    buffer[3] = (uint8_t)(data & 0x000000FF);

    status = VL53L0X_write_multi(address, index, buffer, BYTES_PER_DWORD);

    return status;
}

int32_t VL53L0X_read_byte(uint8_t address, uint8_t index, uint8_t *pdata)
{
    int32_t status = STATUS_OK;
    int32_t cbyte_count = 1;

    status = VL53L0X_read_multi(address, index, pdata, cbyte_count);

#ifdef VL53L0X_LOG_ENABLE
    trace_print(TRACE_LEVEL_INFO, "Read reg : 0x%02X, Val : 0x%02X\n", index, *pdata);
#endif

    return status;
}

int32_t VL53L0X_read_word(uint8_t address, uint8_t index, uint16_t *pdata)
{
    int32_t status = STATUS_OK;
    uint8_t buffer[BYTES_PER_WORD];

    status = VL53L0X_read_multi(address, index, buffer, BYTES_PER_WORD);
    *pdata = ((uint16_t)buffer[0] << 8) + (uint16_t)buffer[1];

    return status;
}

int32_t VL53L0X_read_dword(uint8_t address, uint8_t index, uint32_t *pdata)
{
    int32_t status = STATUS_OK;
    uint8_t buffer[BYTES_PER_DWORD];

    status = VL53L0X_read_multi(address, index, buffer, BYTES_PER_DWORD);
    *pdata = ((uint32_t)buffer[0] << 24) + ((uint32_t)buffer[1] << 16) + ((uint32_t)buffer[2] << 8) + (uint32_t)buffer[3];

    return status;
}

int32_t VL53L0X_platform_wait_us(int32_t wait_us)
{
    ets_delay_us(wait_us);
    return STATUS_OK;
}

int32_t VL53L0X_wait_ms(int32_t wait_ms)
{
    int tick = pdMS_TO_TICKS(wait_ms);
    if (tick > 0)
    {
        vTaskDelay(tick);
    }
    else
    {
        int us = 1000 * wait_ms;
        ets_delay_us(us);
    }

    return STATUS_OK;
}

int32_t VL53L0X_set_gpio(uint8_t  level)
{
    return STATUS_OK;
}

int32_t VL53L0X_get_gpio(uint8_t *plevel)
{
    return STATUS_OK;
}

int32_t VL53L0X_release_gpio(void)
{
    return STATUS_OK;
}

int32_t VL53L0X_get_timer_frequency(int32_t *ptimer_freq_hz)
{
    *ptimer_freq_hz = 0;
    return STATUS_OK;
}

int32_t VL53L0X_get_timer_value(int32_t *ptimer_count)
{
    *ptimer_count = 0;
    return STATUS_OK;
}
