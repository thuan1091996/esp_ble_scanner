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
#define MODULE_NAME             "data_handler"
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
    
    while(1)
    {
        sensor_data_packet_t recv_data = {0};
        if(xQueueReceive(sensor_data_queue, &recv_data, portMAX_DELAY) == pdTRUE)
        {
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
        }
        else
        {
            ESP_LOGE(MODULE_NAME, "Receive data from queue failed");
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

