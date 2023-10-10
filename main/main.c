/******************************************************************************
* Includes
*******************************************************************************/
#include <stdio.h>


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "sdkconfig.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "task_common.h"
#include "mqtt/mqtt.h"
#include "wifi_custom/wifi_custom.h"
#include "ble_gatt_client/ble_gattc.h"
/******************************************************************************
* Module Preprocessor Constants
*******************************************************************************/
#define MODULE_NAME                                 "main"

#define APP_CONF_ENABLE_BLE_GATTC                   (1)
#define APP_CONF_ENABLE_WIFI                        (0)
#define APP_CONF_ENABLE_MQTT                        (0)
/******************************************************************************
* Module Preprocessor Macros
*******************************************************************************/

/******************************************************************************
* Module Typedefs
*******************************************************************************/

/******************************************************************************
* Module Variable Definitions
*******************************************************************************/
TaskInitParams_t const TasksTable[] =
{
 // Function pointer,	String Name,	Stack size,		Parameter,	Priority,	Task Handle
#if (APP_CONF_ENABLE_MQTT != 0)
    {&mqtt_task,	"MQTT Task",	MQTT_TASK_STACK_SIZE,  NULL, MQTT_TASK_PRIORITY, &xMQTT_handler},
#endif /* End of (APP_CONF_ENABLE_BLE_GATTC != 0) */

};

/******************************************************************************
* Function Prototypes
*******************************************************************************/

/******************************************************************************
* Function Definitions
*******************************************************************************/
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

void app_ble_data_handling(void* p_data, void* data_len)
{
    ble_client_packet_t* p_ble_packet = (ble_client_packet_t*)p_data;
    if(p_ble_packet == NULL)
    {
        ESP_LOGE(MODULE_NAME, "Invalid BLE packet");
        return;
    }
    
    if(p_ble_packet->p_payload == NULL)
    {
        ESP_LOGE(MODULE_NAME, "Invalid BLE packet payload");
        return;
    }

    if(p_ble_packet->payload_len <= 0)
    {
        ESP_LOGE(MODULE_NAME, "Invalid BLE packet payload length");
        return;
    }

    if(p_ble_packet->p_payload[0] != SENSOR_PAYLOAD_DATA_HEADER)
        return;

    ESP_LOGI(MODULE_NAME, "BLE packet received. Payload length: %d", p_ble_packet->payload_len);
    for(uint16_t idx=0; idx<p_ble_packet->payload_len; idx++)
    {
        printf("%02X ", p_ble_packet->p_payload[idx]);
    }
    printf("\r\n");
    //TODO: Forward data to data handing task

}

int app_ble_client_init()
{
    ble_client_callback_t ble_client_callback = 
    {
        .ble_found_adv_packet_cb = &app_ble_data_handling,
    };
    if( ble_gatt_client_init(&ble_client_callback) != SUCCESS)
    {
        ESP_LOGE(MODULE_NAME, "Failed to initialize BLE GATTC");
        return FAILURE;
    }
    
    if (ble_gatt_client_start_scan(BLE_GATTC_SCAN_DURATION) != SUCCESS)
    {
        ESP_LOGE(MODULE_NAME, "Failed to start BLE GATTC scan");
        return FAILURE;
    }

    return SUCCESS;
}

void app_main(void)
{
    nvs_init();

    #if (APP_CONF_ENABLE_WIFI != 0)
    if ( 0 != wifi_custom_init())
    {
        ESP_LOGE(MODULE_NAME, "Failed to initialize Wi-Fi");
        return;
    }

    // Connect to Wi-Fi
    for(uint8_t retry_count=0; retry_count<2; retry_count++)
    {
        if ( 0 == wifi_custom__power_on())
        {
            ESP_LOGI(MODULE_NAME, "Connected to Wi-Fi");
            break;
        }
        else
        {
            ESP_LOGE(MODULE_NAME, "Failed to connect to Wi-Fi. Retrying [%d/%d]...", retry_count+1, 2);
        }
    }

    if(wifi_custom__connected() != true)
    {
        ESP_LOGE(MODULE_NAME, "Failed to connect to Wi-Fi. Running ESP SmartConfig...");
        wifi_custom__power_off();
        if ( 0 != smartconfig_init())
        {
            ESP_LOGE(MODULE_NAME, "Failed to run ESP SmartConfig. Aborting...");
            return;
        }
        ESP_LOGI(MODULE_NAME, "Waiting for Wi-Fi config with ESPTouch...");
        vTaskDelay(10000 / portTICK_PERIOD_MS);
        if ( 0 == wifi_custom__power_on())
        {
            ESP_LOGI(MODULE_NAME, "Connected to Wi-Fi");
        }
        else
        {
            ESP_LOGE(MODULE_NAME, "Failed to connect to Wi-Fi. Aborting...");
            return;
        }
    }
    #endif /* End of (APP_CONF_ENABLE_WIFI != 0) */

    #if (APP_CONF_ENABLE_BLE_GATTC != 0)
    if ( 0 != app_ble_client_init())
    {
        ESP_LOGE(MODULE_NAME, "Failed to initialize BLE GATTC");
        return;
    }
    #endif /* End of (APP_CONF_ENABLE_BLE_GATTC != 0) */
    
    // Check array size
    for(uint8_t idx=0; idx< sizeof(TasksTable)/sizeof(TasksTable[0]); idx++)
    {
        xTaskCreate(TasksTable[idx].TaskCodePtr,        /* Function pointer*/  
                    TasksTable[idx].TaskName,           /* String Name*/          
                    TasksTable[idx].StackDepth,         /* Stack size*/	
                    TasksTable[idx].ParametersPtr,      /* Parameter*/
                    TasksTable[idx].TaskPriority,       /* Priority*/
                    TasksTable[idx].TaskHandle);        /* Task Handle*/
        assert(NULL != TasksTable[idx].TaskHandle);
    }
}
