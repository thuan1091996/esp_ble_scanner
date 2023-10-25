/*************** INTERFACE CHANGE LIST **************************************    
*    
*    Date       	Software Version    Initials        Description    
*  27 Nov 2022         0.0.1           Itachi      Interface Created.    
*    
*****************************************************************************/    
/* @file:   task_common.h   
 * @brief:  This header contains   
 */ 
#ifndef MAIN_TASK_COMMON_TASK_COMMON_H_
#define MAIN_TASK_COMMON_TASK_COMMON_H_

/******************************************************************************
* Includes
*******************************************************************************/
#include <string.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"

#include "actor.h"
#include "event.h"
/******************************************************************************
* Preprocessor Constants
*******************************************************************************/
#define SUCCESS                                 (0)
#define FAILURE                                 (-1)

#define MQTT_TESTING                            (0)
/******************************************************************************
* Configuration Constants
*******************************************************************************/
#define MQTT_TASK_STACK_SIZE                    (10*1024)
#define MQTT_TASK_PRIORITY	                    (tskIDLE_PRIORITY + 1)

#define DATA_HANDLING_TASK_STACK_SIZE           (5*1024)
#define DATA_HANDLING_TASK_PRIORITY	            (tskIDLE_PRIORITY + 1)
/******************************************************************************
* Macros
*******************************************************************************/


/******************************************************************************
* Typedefs
*******************************************************************************/
enum app_signal
{
    /* Wi-Fi events */
	WIFI_CONNECTED = USER_SIG,
    WIFI_DISCONNECTED,

    /* MQTT events*/
    MQTT_CONNECTED,
    MQTT_DISCONNECTED,

    /* Data event */
    SENSOR_DATA_READY,

    /* GATT device events */
    GATT_TIMER_TIMEOUT,
    GATT_DEVICE_CONNECTED,
    GATT_DEVICE_DISCONNECTED,
    GATT_DEVICE_SUBSCRIBED,
    GATT_DEVICE_UNSUBSCRIBED,
    GATT_DEVICE_DATA_AVAILABLE,
    GATT_SCAN_FOUND_DEVICE,         // Scan found device
    GATT_SCAN_TIMEOUT,              // Scan duration timeout   
	/* .... */
	MAX_SIG
};

/******************************************************************************
* Module Typedefs
*******************************************************************************/
typedef struct
{
    TaskFunction_t const TaskCodePtr;           /*< Pointer to the task function */
    const char * const TaskName;                /*< String task name             */
    const uint16_t StackDepth;                  /*< Stack depth                  */
    void * const ParametersPtr;                 /*< Parameter Pointer            */
    UBaseType_t TaskPriority;                   /*< Task Priority                */
    TaskHandle_t * const TaskHandle;            /*< Pointer to task handle       */
}TaskInitParams_t;

typedef struct
{
    Evt super;
    char* sensor_data_json;
}sensor_data_evt_t;

/******************************************************************************
* Variables
*******************************************************************************/
extern TaskHandle_t xMQTT_handler;
extern TaskHandle_t xdata_handler;

extern TaskInitParams_t const TasksTable[];

extern ActiveId_t p_mqtt_actor;
/******************************************************************************
* Function Prototypes
*******************************************************************************/


#endif /* MAIN_TASK_COMMON_TASK_COMMON_H_ */
