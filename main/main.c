#include <stdio.h>

#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"



void app_main(void)
{
    while(1)
    {
        ESP_LOGI("main", "Hello world!");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
