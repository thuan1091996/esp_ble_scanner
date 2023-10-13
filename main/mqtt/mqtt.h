/****************************************************************************
* Title                 :   MQTT client header
* Filename              :   mqtt.h
* Author                :   Itachi
* Origin Date           :   2023/01/26
* Version               :   v0.0.0
* Compiler              :   ESP IDF v5.0
* Target                :   esp32
* Notes                 :   None
*****************************************************************************/

/*************** INTERFACE CHANGE LIST **************************************
*
*    Date    	Software Version    Initials   	Description
*  2023/01/26    v0.0.0         	Itachi      Interface Created.
*
*****************************************************************************/

/** \file mqtt_client.h
 *  \brief This module contains .
 *
 *  This is the header file for 
 */
#ifndef MQTT_CLIENT_H_
#define MQTT_CLIENT_H_

/******************************************************************************
* Includes
*******************************************************************************/


/******************************************************************************
* Preprocessor Constants
*******************************************************************************/



/******************************************************************************
* Configuration Constants
*******************************************************************************/
#define MQTT_BROKER_ADDR                        "mqtt://test.mosquitto.org"
#define MQTT_DEFAULT_SUB_TOPIC                  "app_cmd"
#define MQTT_DEFAULT_STATUS_TOPIC               "mastering_esp32/sensor_status"
#define MQTT_DEFAULT_DATA_TOPIC                 "mastering_esp32/sensor_data"
/******************************************************************************
* Macros
*******************************************************************************/


/******************************************************************************
* Typedefs
*******************************************************************************/


/******************************************************************************
* Variables
*******************************************************************************/


/******************************************************************************
* Function Prototypes
*******************************************************************************/
void mqtt_client_init();
void mqtt_task(void* param);
int mqtt_actor_init();

int mqtt_update_broker_addr(char* addr);
int mqtt_connect();
int mqtt_disconnect();

int mqtt_publish(const char *topic, const char *data, int len, int qos, int retain);
int mqtt_subscribe(const char *topic, int qos);
#endif // MQTT_CLIENT_H_
/*** End of File **************************************************************/