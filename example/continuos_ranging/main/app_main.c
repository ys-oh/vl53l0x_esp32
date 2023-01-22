/*
 * File : app_main.c
 * Created: Tuesday, 09 February 2021
 * Author: yunsik oh (oyster90@naver.com)
 * 
 * Modified: Tuesday, 09 February 2021
 * 
 */
#include "esp_log.h"
#include "nvs_flash.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "vl53l0x.h"

static const char* TAG = "test";

void vl53l0x_task(void* p)
{
    VL53L0X_Dev_t dev;

    VL53L0X_Error err = VL53L0X_Device_init(&dev);
    if (err != VL53L0X_DEVICEERROR_NONE)
    {
        ESP_LOGE(TAG, "device init error (%d)", err);
        vTaskDelete(NULL);
        return;
    }

    while (1)
    {
        uint16_t data = 0;
        if (VL53L0X_Device_getMeasurement(&dev, &data) == VL53L0X_ERROR_NONE)
        {
            ESP_LOGI(TAG, "measured data : %d mm", data);
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void app_main()
{
    ESP_ERROR_CHECK(nvs_flash_init());
    xTaskCreate(&vl53l0x_task, "test", 2049, NULL, 5, NULL);
}