#include <stdio.h>

#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "nvs_flash.h"

#include "task_common.h"
#include "mqtt/mqtt.h"
#include "wifi_custom/wifi_custom.h"



int nvs_init()
{
    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    ESP_ERROR_CHECK(ret);
    if (ESP_OK != ret)
    {
        ESP_LOGE("nvs", "Failed to initialize NVS Flash. Erasing and re-initializing...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        if (nvs_flash_init() != ESP_OK)
        {
            ESP_LOGE("nvs", "Failed to erase and re-initialize NVS Flash. Aborting...");
            return FAILURE;
        }
    }
    ESP_LOGI("nvs", "NVS Flash initialized \r\n");   
    return SUCCESS;
}

void app_main(void)
{
    nvs_init();
    if ( 0 != wifi_custom_init())
    {
        ESP_LOGE("custom_wifi", "Failed to initialize Wi-Fi");
        return;
    }

    // Connect to Wi-Fi
    for(uint8_t retry_count=0; retry_count<2; retry_count++)
    {
        if ( 0 == wifi_custom__power_on())
        {
            ESP_LOGI("custom_wifi", "Connected to Wi-Fi");
            break;
        }
        else
        {
            ESP_LOGE("custom_wifi", "Failed to connect to Wi-Fi. Retrying [%d/%d]...", retry_count+1, 2);
        }
    }

    if(wifi_custom__connected() != true)
    {
        ESP_LOGE("custom_wifi", "Failed to connect to Wi-Fi. Running ESP SmartConfig...");
        wifi_custom__power_off();
        if ( 0 != smartconfig_init())
        {
            ESP_LOGE("custom_wifi", "Failed to run ESP SmartConfig. Aborting...");
            return;
        }
        ESP_LOGI("custom_wifi", "Waiting for Wi-Fi config with ESPTouch...");
        vTaskDelay(10000 / portTICK_PERIOD_MS);
        if ( 0 == wifi_custom__power_on())
        {
            ESP_LOGI("custom_wifi", "Connected to Wi-Fi");
        }
        else
        {
            ESP_LOGE("custom_wifi", "Failed to connect to Wi-Fi. Aborting...");
            return;
        }
    }
    }
}
