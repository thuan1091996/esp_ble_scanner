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
#include "task_common.h"

#include "data_handler.h"
/******************************************************************************
* Module Preprocessor Constants
*******************************************************************************/

/******************************************************************************
* Module Preprocessor Macros
*******************************************************************************/
#define MODULE_NAME                     "data_handler"
#define MODULE_DEFAULT_LOG_LEVEL        ESP_LOG_INFO
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
    sensor_data_queue = xQueueCreate(DATA_QUEUE_MAX_LEN, sizeof(ble_sensor_data_packet_t));
    if (sensor_data_queue == NULL)
    {
        ESP_LOGE(MODULE_NAME, "Create data msg queue failed");
        vTaskDelete(NULL);
    }
    esp_log_level_set(MODULE_NAME, MODULE_DEFAULT_LOG_LEVEL);
    char str_json[SENSOR_DATA_JSON_MAX_LEN];
    while(1)
    {
        ble_sensor_data_packet_t recv_data = {0};
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
                ESP_LOGD(MODULE_NAME, "JSON: %s", str_json);
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
            ESP_LOGI(MODULE_NAME, "Received FCNT: %ld", recv_data.sensor_payload.fcnt);
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

int sensor_data_validate(uint8_t* data, uint16_t len)
{
    // Validate data format
    if(data == NULL)
    {
        ESP_LOGE(MODULE_NAME, "Invalid data");
        return -1;
    }
    if(len != sizeof(sensor_data_t))
    {
        ESP_LOGE(MODULE_NAME, "Invalid data length");
        return -1;
    }
    return 0;
}

/*
 * @brief: Validate & send sensor data based on given format
 * @param data (in): pointer to data buffer
 * @param len (in): length of data buffer
 * @return int: 0 if success, -1 if failed
 */
int sensor_data_sending(uint8_t* data, uint16_t len)
{
    // Dumpp data 
    ESP_LOG_BUFFER_HEXDUMP(MODULE_NAME, data, len, ESP_LOG_DEBUG);
    // Validate data format
    if(data == NULL)
    {
        ESP_LOGE(MODULE_NAME, "Invalid data");
        return -1;
    }
    if(len != sizeof(ble_sensor_data_packet_t))
    {
        ESP_LOGE(MODULE_NAME, "Invalid data length");
        return -1;
    }

    if(sensor_data_queue == NULL)
    {
        ESP_LOGE(MODULE_NAME, "Sensor data queue is not ready");
        return -1;
    }
    // Sending data to queue
    ble_sensor_data_packet_t* p_sensor_data_packet = (ble_sensor_data_packet_t*)data;
    ESP_LOGI(MODULE_NAME, "Sending frame: %ld", (long)p_sensor_data_packet->sensor_payload.fcnt);
    if(xQueueSend(sensor_data_queue, p_sensor_data_packet, portMAX_DELAY) != pdTRUE)
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
int sensor_data_format_json(ble_sensor_data_packet_t* sensor_data_packet, char* json_str, uint16_t json_str_max_len)
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
        cJSON *sensor_data_json_obj = NULL;
        sensor_data_json_obj = cJSON_CreateObject();
        if(sensor_data_json_obj == NULL)
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
        cJSON_AddNumberToObject(sensor_data_json_obj, "fcnt", sensor_data_packet->sensor_payload.fcnt);
        cJSON_AddNumberToObject(sensor_data_json_obj, "aclx", sensor_data_packet->sensor_payload.acl_x);
        cJSON_AddNumberToObject(sensor_data_json_obj, "acly", sensor_data_packet->sensor_payload.acl_y);
        cJSON_AddNumberToObject(sensor_data_json_obj, "aclz", sensor_data_packet->sensor_payload.acl_z);
        cJSON_AddNumberToObject(sensor_data_json_obj, "gyrx", sensor_data_packet->sensor_payload.gyro_x);
        cJSON_AddNumberToObject(sensor_data_json_obj, "gyry", sensor_data_packet->sensor_payload.gyro_y);
        cJSON_AddNumberToObject(sensor_data_json_obj, "gyrz", sensor_data_packet->sensor_payload.gyro_z);
        cJSON_AddItemToObject(root, "sensor", sensor_data_json_obj);
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

cJSON * json_sensor_data_format(ble_sensor_data_packet_t* sensor_data_packet)
{
    assert(sensor_data_packet != NULL);
    cJSON *root = NULL;
    root = cJSON_CreateObject();
    if(root == NULL)
    {
        ESP_LOGE(MODULE_NAME, "Create JSON root for json_sensor_data_format failed");
        return NULL;
    }
    // Add device address
    char addr_str[18] = {0};
    sprintf(addr_str, "%02X:%02X:%02X:%02X:%02X:%02X", 
                        sensor_data_packet->device_addr[0], sensor_data_packet->device_addr[1], sensor_data_packet->device_addr[2],
                        sensor_data_packet->device_addr[3], sensor_data_packet->device_addr[4], sensor_data_packet->device_addr[5]);
    cJSON_AddStringToObject(root, "addr", addr_str);
    // Add sensor data
    cJSON *sensor_data_json_obj = NULL;
    sensor_data_json_obj = cJSON_CreateObject();
    if(sensor_data_json_obj == NULL)
    {
        ESP_LOGE(MODULE_NAME, "Create JSON sensor_data_json_obj for json_sensor_data_format failed");
        cJSON_Delete(root);
        return NULL;
    }
    cJSON_AddNumberToObject(sensor_data_json_obj, "fcnt", sensor_data_packet->sensor_payload.fcnt);
    cJSON_AddNumberToObject(sensor_data_json_obj, "aclx", sensor_data_packet->sensor_payload.acl_x);
    cJSON_AddNumberToObject(sensor_data_json_obj, "acly", sensor_data_packet->sensor_payload.acl_y);
    cJSON_AddNumberToObject(sensor_data_json_obj, "aclz", sensor_data_packet->sensor_payload.acl_z);
    cJSON_AddNumberToObject(sensor_data_json_obj, "gyrx", sensor_data_packet->sensor_payload.gyro_x);
    cJSON_AddNumberToObject(sensor_data_json_obj, "gyry", sensor_data_packet->sensor_payload.gyro_y);
    cJSON_AddNumberToObject(sensor_data_json_obj, "gyrz", sensor_data_packet->sensor_payload.gyro_z);
    cJSON_AddItemToObject(root, "sensor", sensor_data_json_obj);
    return root;    
}

extern ble_sensor_data_packet_t* p_sensor_data[];
/*
 * @brief: Format array of sensor data packet to JSON string 
 * @param json_out: pointer to output JSON string
 * @param json_max_length: maximum length of JSON string
 * @param sensor_data: pointer to array of sensor data packet
 * @param sensor_num: number of sensor data packet
 * @return int: 0 if success, -1 if failed
 * @example:
{
    "sensor_data": [
        {
            "addr": "EF:F6:06:47:95:23",
            "sensor": {
                "fcnt": 2777,
                "aclx": -6.08300018310547,
                "acly": -2.244999885559082,
                "aclz": -7.5580000877380371,
                "gyrx": 0.0040000001899898052,
                "gyry": -0.037000000476837158,
                "gyrz": 0
            }
        },
        {
            "addr": "E7:78:D8:21:03:37",
            "sensor": {
                "fcnt": 3321,
                "aclx": 3.8204714345426319e-37,
                "acly": 1.0082513512365273e-34,
                "aclz": 2.65846275898916e-32,
                "gyrx": 7.0036532705607975e-30,
                "gyry": 1.8436203207959919e-27,
                "gyrz": 4.8494218350807544e-25
            }
        }
    ]
}
 */
int sensor_data_msgs_format_json(char* json_out, uint16_t json_max_length, 
                                 ble_sensor_data_packet_t* sensor_data, uint8_t sensor_num)
{
    assert(json_out != NULL);
    assert(json_max_length > 0);
    cJSON *root = NULL;
    root = cJSON_CreateObject();
    if(root == NULL)
    {
        ESP_LOGE(MODULE_NAME, "Create JSON root for sensor_data_msgs_format_json failed");
        return -1;
    }
    // Add sensor data array to root
    cJSON *sensor_data_json_array = cJSON_CreateArray();
    if(sensor_data_json_array == NULL)
    {
        ESP_LOGE(MODULE_NAME, "Create JSON sensor_data_json_array for sensor_data_msgs_format_json failed");
        goto json_failed_cleanup;
    }
    cJSON_AddItemToObject(root, "sensor_data", sensor_data_json_array);

    // Add sensor data
    for(uint8_t idx=0; idx < sensor_num; idx++)
    {
//        cJSON *sensor_data_json_obj = json_sensor_data_format(*((ble_sensor_data_packet_t**)&sensor_data[idx]));
        cJSON *sensor_data_json_obj = json_sensor_data_format(p_sensor_data[idx]);

        if(sensor_data_json_obj == NULL)
        {
            ESP_LOGE(MODULE_NAME, "Create JSON sensor_data_json_obj for sensor_data_json_array failed at idx %d", idx);
            goto json_failed_cleanup;
        }
        cJSON_AddItemToArray(sensor_data_json_array, sensor_data_json_obj);
    }
    
    char* str_json = cJSON_PrintUnformatted(root);
    // char* str_json = cJSON_Print(root);
    if(str_json == NULL)
    {
        ESP_LOGE(MODULE_NAME, "Create JSON string failed");
        goto json_failed_cleanup;
    }
    memset(json_out, 0, json_max_length);
    if(strlen(str_json) > json_max_length)
    {
        ESP_LOGW(MODULE_NAME, "JSON string is too long (%d/%d), data will be truncated", strlen(str_json), json_max_length);
        memcpy(json_out, str_json, json_max_length);
    }
    else
    {   
        strcpy(json_out, str_json);
    	// memcpy(json_out, str_json, strlen(str_json));
    }
    ESP_LOGE(MODULE_NAME, "Out %d / %d", strlen(json_out), strlen(str_json));
    ESP_LOGI(MODULE_NAME, "JSON ===========================: \r\n \r\n %s  \r\n \r\n ===========================: \r\n\r\n", str_json);
    free(str_json);
    cJSON_Delete(root);


    return 0;
json_failed_cleanup:
    cJSON_Delete(root);
    return -1;
}
