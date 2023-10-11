/****************************************************************************
* Title                 :   Data handler header file
* Filename              :   data_handler.h
* Author                :   ItachiVN
* Origin Date           :   2023/10/10
* Version               :   v0.0.0
* Compiler              :   ESP-IDF V5.0.2
* Target                :   ESP32 
* Notes                 :   None
*****************************************************************************/

/*************** INTERFACE CHANGE LIST **************************************
*
*    Date    	Software Version    Initials   	Description
*  2023/10/10    v0.0.0         	ItachiVN      Interface Created.
*
*****************************************************************************/

/** \file data_handler.h
 *  \brief This module contains .
 *
 *  This is the header file for 
 */
#ifndef _DATA_HANDLER_H_
#define _DATA_HANDLER_H_

/******************************************************************************
* Includes
*******************************************************************************/
#include "stdint.h"

/******************************************************************************
* Preprocessor Constants
*******************************************************************************/
#define SENSOR_DATA_HEADER              (0xAA)
#define SENSOR_DATA_JSON_MAX_LEN        (256) // In bytes
#define SENSOR_DATA_CONF_DUMP_RAW       (0) // 1 -> log raw sensor data
/******************************************************************************
* Configuration Constants
*******************************************************************************/


/******************************************************************************
* Macros
*******************************************************************************/


/******************************************************************************
* Typedefs
*******************************************************************************/
typedef struct {
    uint32_t fcnt;
    float acl_x;
    float acl_y;
    float acl_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
}sensor_data_t;

typedef struct {
    uint8_t device_addr[6];
    sensor_data_t sensor_payload;
}sensor_data_packet_t;

/******************************************************************************
* Variables
*******************************************************************************/


/******************************************************************************
* Function Prototypes
*******************************************************************************/

#ifdef __cplusplus
extern "C"{
#endif
void data_handling_task(void* param);
int sensor_data_sending(uint8_t* data, uint16_t len);
int sensor_data_format_json(sensor_data_packet_t* sensor_data_packet, char* json_str, uint16_t json_str_max_len);
#ifdef __cplusplus
} // extern "C"
#endif

#endif // _DATA_HANDLER_H_

/*** End of File **************************************************************/