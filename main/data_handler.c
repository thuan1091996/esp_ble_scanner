/*******************************************************************************
* Title                 :   Data handling task 
* Filename              :   data_handler.c
* Author                :   ItachiVN
* Origin Date           :   2023/10/10
* Version               :   0.0.0
* Compiler              :   ESP-IDF V5.0.2
* Target                :   ESP32 
* Notes                 :   None
*******************************************************************************/

/******************************************************************************
* Includes
*******************************************************************************/
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "cJSON.h"
#include "task_common.h"

#include "data_handler.h"
/******************************************************************************
* Module Preprocessor Constants
*******************************************************************************/

/******************************************************************************
* Module Preprocessor Macros
*******************************************************************************/
#define MODULE_NAME                     "data_handler"
#define MODULE_DEFAULT_LOG_LEVEL        ESP_LOG_WARN
/******************************************************************************
* Module Typedefs
*******************************************************************************/

/******************************************************************************
* Module Variable Definitions
*******************************************************************************/
static QueueHandle_t sensor_data_queue=NULL;
TaskHandle_t xdata_handler=NULL;

/******************************************************************************
* Function Prototypes
*******************************************************************************/

/******************************************************************************
* Function Definitions
*******************************************************************************/
void data_handling_task(void* param)
{
    // Create data msg queue
    sensor_data_queue = xQueueCreate(10, sizeof(sensor_data_packet_t));
    if (sensor_data_queue == NULL)
    {
        ESP_LOGE(MODULE_NAME, "Create data msg queue failed");
        vTaskDelete(NULL);
    }
    esp_log_level_set(MODULE_NAME, MODULE_DEFAULT_LOG_LEVEL);
    char str_json[SENSOR_DATA_JSON_MAX_LEN];
    while(1)
    {
        sensor_data_packet_t recv_data = {0};
        if(xQueueReceive(sensor_data_queue, &recv_data, portMAX_DELAY) != pdTRUE)
        {
            ESP_LOGE(MODULE_NAME, "Receive data from queue failed");
        }
        else
        {
            memset(str_json, 0, SENSOR_DATA_JSON_MAX_LEN);
            if(sensor_data_format_json(&recv_data, str_json, SENSOR_DATA_JSON_MAX_LEN) != 0)
            {
                ESP_LOGE(MODULE_NAME, "Format sensor data to JSON failed");
            }
            else
            {
                ESP_LOGI(MODULE_NAME, "JSON: %s", str_json);
                sensor_data_evt_t* p_e = (sensor_data_evt_t*)Event_New(SENSOR_DATA_READY, sizeof(sensor_data_evt_t));
                if(p_e == NULL)
                {
                    ESP_LOGE(MODULE_NAME, "Create event failed");
                }
                else
                {
                    p_e->sensor_data_json = str_json;
                    Active_post(p_mqtt_actor, (Evt*)p_e);
                }
            }
            ESP_LOGW(MODULE_NAME, "FCNT: %ld", recv_data.sensor_payload.fcnt);
#if (SENSOR_DATA_CONF_DUMP_RAW != 0 )
            ESP_LOGI(MODULE_NAME, "Received data from sensor");
            ESP_LOGI(MODULE_NAME, "Device addr: %02X:%02X:%02X:%02X:%02X:%02X", 
                                    recv_data.device_addr[0], recv_data.device_addr[1], recv_data.device_addr[2],
                                    recv_data.device_addr[3], recv_data.device_addr[4], recv_data.device_addr[5]);
            ESP_LOGI(MODULE_NAME, "FCNT: %ld", recv_data.sensor_payload.fcnt);
            ESP_LOGI(MODULE_NAME, "ACL_X: %.02f", recv_data.sensor_payload.acl_x);
            ESP_LOGI(MODULE_NAME, "ACL_Y: %.02f", recv_data.sensor_payload.acl_y);
            ESP_LOGI(MODULE_NAME, "ACL_Z: %.02f", recv_data.sensor_payload.acl_z);
            ESP_LOGI(MODULE_NAME, "GYRO_X: %.02f", recv_data.sensor_payload.gyro_x);
            ESP_LOGI(MODULE_NAME, "GYRO_Y: %.02f", recv_data.sensor_payload.gyro_y);
            ESP_LOGI(MODULE_NAME, "GYRO_Z: %.02f", recv_data.sensor_payload.gyro_z);
#endif /* End of (SENSOR_DATA_CONF_DUMP_RAW != 0 ) */
        }

    }
}

/*
 * @brief: Validate & send sensor data based on given format
 * @param data (in): pointer to data buffer
 * @param len (in): length of data buffer
 * @return int: 0 if success, -1 if failed
 * @note: format of raw sensor data:
 * + Device address - 6B
 * + SENSOR_PAYLOAD_DATA_HEADER - 1B - 0xAA
 * + Sensor data  - sizeof(sensor_data_t)
 */
int sensor_data_sending(uint8_t* data, uint16_t len)
{
    // Validate data format
    if(data == NULL)
    {
        ESP_LOGE(MODULE_NAME, "Invalid data");
        return -1;
    }
    if(len != sizeof(sensor_data_t) + 7)
    {
        // ESP_LOGE(MODULE_NAME, "Invalid data length");
        return -1;
    }
    if(data[6] != SENSOR_DATA_HEADER)
    {
        // ESP_LOGE(MODULE_NAME, "Invalid data header");
        return -1;
    }

    if(sensor_data_queue == NULL)
    {
        ESP_LOGE(MODULE_NAME, "Sensor data queue is not ready");
        return -1;
    }
    // Sending data to queue
    sensor_data_packet_t sensor_data_packet = {0};
    memcpy(sensor_data_packet.device_addr, data, 6);
    memcpy(&sensor_data_packet.sensor_payload, data + 7, sizeof(sensor_data_t));
    // Dump 
    if(xQueueSend(sensor_data_queue, &sensor_data_packet, portMAX_DELAY) != pdTRUE)
    {
        ESP_LOGE(MODULE_NAME, "Send data to queue failed");
        return -1;
    }
    return 0;
}

/*
 * @brief: Format sensor data packet to JSON string
 * @param (in): sensor_data 
 * @param (out): json_str
 * @return int: 0 if success, -1 if failed
 * @note
 *          Target format
 *          {
 *              "addr": "XX:XX:XX:XX:XX:XX"
 *              "sensor": 
 *               {
 *                  "fcnt": 0, // Frame counter - 4B integer
 *                  "aclx": 0, // X-axis acceleration - 4B float
 *                  "acly": 0, // Y-axis acceleration - 4B float
 *                  "aclz": 0, // Z-axis acceleration - 4B float
 *                  "gyrx": 0, // X-axis gyroscope - 4B float
 *                  "gyry": 0, // Y-axis gyroscope - 4B float
 *                  "gyrz": 0, // Z-axis gyroscope - 4B float
 *               }
 *          }
 */
int sensor_data_format_json(sensor_data_packet_t* sensor_data_packet, char* json_str, uint16_t json_str_max_len)
{
    int status = 0 ;
    cJSON *root = NULL;
    root = cJSON_CreateObject();
    if(root == NULL)
    {
        ESP_LOGE(MODULE_NAME, "Create JSON object failed");
        return -1;
    }

    do
    {
        // Add sensor data
        cJSON *sensor_data = NULL;
        sensor_data = cJSON_CreateObject();
        if(sensor_data == NULL)
        {
            ESP_LOGE(MODULE_NAME, "Create JSON object failed");
            status = -1;
            break;
        }
        // Add device address
        char addr_str[18] = {0};
        sprintf(addr_str, "%02X:%02X:%02X:%02X:%02X:%02X", 
                            sensor_data_packet->device_addr[0], sensor_data_packet->device_addr[1], sensor_data_packet->device_addr[2],
                            sensor_data_packet->device_addr[3], sensor_data_packet->device_addr[4], sensor_data_packet->device_addr[5]);
        cJSON_AddStringToObject(root, "addr", addr_str);
        cJSON_AddNumberToObject(sensor_data, "fcnt", sensor_data_packet->sensor_payload.fcnt);
        cJSON_AddNumberToObject(sensor_data, "aclx", sensor_data_packet->sensor_payload.acl_x);
        cJSON_AddNumberToObject(sensor_data, "acly", sensor_data_packet->sensor_payload.acl_y);
        cJSON_AddNumberToObject(sensor_data, "aclz", sensor_data_packet->sensor_payload.acl_z);
        cJSON_AddNumberToObject(sensor_data, "gyrx", sensor_data_packet->sensor_payload.gyro_x);
        cJSON_AddNumberToObject(sensor_data, "gyry", sensor_data_packet->sensor_payload.gyro_y);
        cJSON_AddNumberToObject(sensor_data, "gyrz", sensor_data_packet->sensor_payload.gyro_z);
        cJSON_AddItemToObject(root, "sensor", sensor_data);
        char* str_json = cJSON_PrintUnformatted(root);
        if(str_json == NULL)
        {
            ESP_LOGE(MODULE_NAME, "Create JSON string failed");
            status = -1;
            break;        
        }
        if(strlen(str_json) > json_str_max_len)
        {
            ESP_LOGW(MODULE_NAME, "JSON string is too long (%d/%d), data will be truncated", strlen(str_json), json_str_max_len);
        }
        memcpy(json_str, str_json, json_str_max_len);
        free(str_json);
    } while (0);
	cJSON_Delete(root);
    return status;
}

