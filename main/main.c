/******************************************************************************
* Includes
*******************************************************************************/
#include <stdio.h>


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "sdkconfig.h"
#include "nvs_flash.h"

#include "actor.h"
#include "task_common.h"

#include "data_handler.h"
#include "mqtt/mqtt.h"
#include "wifi_custom/wifi_custom.h"
#include "ble_gatt_client/ble_gattc.h"
/******************************************************************************
* Module Preprocessor Constants
*******************************************************************************/
#define MODULE_NAME                                 "main"

#define APP_CONF_ENABLE_BLE_GATTC                   (1)
#define APP_CONF_WIFI_ENABLE                        (1)
#define APP_CONF_WIFI_AUTO_RECONNECT                (1) // If Wi-Fi is disconnected, automatically reconnect
#define APP_CONF_ENABLE_MQTT_TASK                   (0)
#define APP_CONF_ENABLE_MQTT_ACTOR                  (1)
/******************************************************************************
* Module Preprocessor Macros
*******************************************************************************/

/******************************************************************************
* Module Typedefs
*******************************************************************************/

/******************************************************************************
* Module Variable Definitions
*******************************************************************************/
static Evt wifi_evt = {0};

TaskInitParams_t const TasksTable[] =
{
 // Function pointer,	String Name,	            Stack size,		        Parameter,	Priority,	Task Handle
#if (APP_CONF_ENABLE_MQTT_TASK != 0)
    {&mqtt_task,	        "MQTT Task",	        MQTT_TASK_STACK_SIZE,   NULL,       MQTT_TASK_PRIORITY, &xMQTT_handler},
#endif /* End of (APP_CONF_ENABLE_BLE_GATTC != 0) */
    {&data_handling_task,	"Data Handling Task",	DATA_HANDLING_TASK_STACK_SIZE,  NULL, DATA_HANDLING_TASK_PRIORITY, &xdata_handler},
};

/******************************************************************************
* Function Prototypes
*******************************************************************************/

/******************************************************************************
* App event callback functions
*******************************************************************************/
/* BLE callbacks */
void app_ble_data_handling(void* p_data, void* data_len);

/* Wi-Fi callbacks */
void app_wifi_connected_cb(void* p_data, void* data_len);
void app_wifi_disconnected_cb(void* p_data, void* data_len);
/******************************************************************************
* App init functions
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

int app_wifi_init()
{
    // Initialize Wi-Fi
    wifi_sta_callback_t wifi_sta_callback = 
    {
        .wifi_sta_connected = &app_wifi_connected_cb,
        .wifi_sta_disconnected = &app_wifi_disconnected_cb,
    };

    if( 0 != wifi_custom_init(&wifi_sta_callback))
    {
        ESP_LOGE(MODULE_NAME, "Failed to initialize Wi-Fi");
        return -1;
    }

    if ( 0 != smartconfig_init())
    {
        ESP_LOGE(MODULE_NAME, "Failed to initialize SmartConfig");
    }

    return 0;
}

void app_main(void)
{
    nvs_init();
	Framework_Init();

    #if (APP_CONF_ENABLE_BLE_GATTC != 0)
    if ( 0 != app_ble_client_init())
    {
        ESP_LOGE(MODULE_NAME, "Failed to initialize BLE GATTC");
    }
    #endif /* End of (APP_CONF_ENABLE_BLE_GATTC != 0) */
    
    #if (APP_CONF_ENABLE_MQTT_ACTOR != 0)
    if (mqtt_actor_init() != 0)
    {
        ESP_LOGE(MODULE_NAME, "Failed to initialize MQTT actor");
    }
    #endif /* End of (APP_CONF_ENABLE_MQTT_ACTOR != 0) */

    #if (APP_CONF_WIFI_ENABLE != 0)
    if (0 != app_wifi_init())
    {
        ESP_LOGE(MODULE_NAME, "Failed to initialize Wi-Fi");
    }
    // Connect to Wi-Fi
    if (wifi_custom__power_on() != 0)
    {
        ESP_LOGE(MODULE_NAME, "Failed to connect to Wi-Fi");
    }
    else
    {
        ESP_LOGI(MODULE_NAME, "Connected to Wi-Fi");
    }
    #endif /* End of (APP_CONF_WIFI_ENABLE != 0) */

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



void app_ble_data_handling(void* p_data, void* data_len)
{
    ble_client_packet_t* p_ble_packet = (ble_client_packet_t*)p_data;
    if(p_ble_packet == NULL || p_ble_packet->p_payload == NULL)
    {
        ESP_LOGE(MODULE_NAME, "Invalid BLE packet");
        return;
    }
    uint16_t raw_sensor_data_len = sizeof(p_ble_packet->ble_addr) + p_ble_packet->payload_len;
    uint8_t raw_sensor_data[raw_sensor_data_len];
    memset(raw_sensor_data, 0, raw_sensor_data_len);
    memcpy(raw_sensor_data, p_ble_packet->ble_addr, sizeof(p_ble_packet->ble_addr));
    memcpy(raw_sensor_data + sizeof(p_ble_packet->ble_addr), p_ble_packet->p_payload, p_ble_packet->payload_len);
    sensor_data_sending(raw_sensor_data, raw_sensor_data_len);
}

void app_wifi_connected_cb(void* p_data, void* data_len)
{
    ESP_LOGI("wifi_callback", "Connected to Wi-Fi");
    // Post to MQTT actor
    wifi_evt.sig = WIFI_CONNECTED;
    Active_post(p_mqtt_actor, (Evt*) &wifi_evt);
}

void app_wifi_disconnected_cb(void* p_data, void* data_len)
{
    ESP_LOGI("wifi_callback", "Disconnected from Wi-Fi");
    // Post to MQTT actor
    wifi_evt.sig = WIFI_DISCONNECTED;
    Active_post(p_mqtt_actor, (Evt*) &wifi_evt);
    
    // #if (APP_CONF_WIFI_AUTO_RECONNECT != 0)
    #define WIFI_CONNECT_RETRY_COUNT    (3)
    static uint8_t wifi_connect_retry_count = 0;
    do 
    {
        wifi_custom__power_on();
        wifi_connect_retry_count++;
        ESP_LOGI(MODULE_NAME, "Retry connecting to Wi-Fi before run Smartconfig (%d/%d)...", wifi_connect_retry_count, WIFI_CONNECT_RETRY_COUNT);
    } while( (!wifi_custom__connected() && (wifi_connect_retry_count < WIFI_CONNECT_RETRY_COUNT)) );

    // Check if connected, if still not launch SmartConfig
    if(wifi_custom__connected() != true)
    {
        ESP_LOGE(MODULE_NAME, "Failed to connect to Wi-Fi. Running ESP SmartConfig...");
        wifi_custom__power_off();
        if (smartconfig_start() != 0)
        {
            ESP_LOGE(MODULE_NAME, "Failed to get Wi-Fi credential with smartconfig after timeout");
            smartconfig_stop();
        }
        else
        {
            ESP_LOGI(MODULE_NAME, "Obtained Wi-Fi credential with Smartconfig, trying to connect");
            wifi_custom__power_on();
        }
    }
    else
    {
        wifi_connect_retry_count = 0;
    }
    // #endif /* End of (APP_CONF_WIFI_AUTO_RECONNECT != 0) */
}