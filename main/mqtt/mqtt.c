/*******************************************************************************
* Title                 :   MQTT client 
* Filename              :   mqtt.c
* Author                :   Itachi
* Origin Date           :   2023/01/26
* Version               :   0.0.0
* Compiler              :   ESP IDF v5.0
* Target                :   esp32
* Notes                 :   None
*******************************************************************************/

/*************** MODULE REVISION LOG ******************************************
*
*    Date       Software Version	Initials	Description
*  2023/01/26       0.0.0	         Itachi      Module Created.
*
*******************************************************************************/

/** \file mqtt.c
 * \brief This module contains the
 */
/******************************************************************************
* Includes
*******************************************************************************/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "mqtt_client.h"

#include "../task_common.h"
#include "mqtt.h"
/******************************************************************************
* Module Preprocessor Constants
*******************************************************************************/
#define MODULE_NAME						                "MQTT_CLIENT:"
#define MODULE_DEFAULT_LOG_LEVEL						ESP_LOG_WARN

#define MQTT_LWT_TESTING                				(1)
#define MQTT_KEEP_ALIVE_PERIOD          				(10)

#if MQTT_TESTING


#endif /* End of MQTT_TESTING */

#define MQTT_CLIENT_EVENT_CONNECTED						(BIT1)
#define MQTT_CLIENT_EVENT_ONDATA						(BIT2)
#define MQTT_CLIENT_EVENT_PUBLISHED						(BIT3)
#define MQTT_CLIENT_EVENT_DISCONNECT					(BIT4)
/******************************************************************************
* Module Preprocessor Macros
*******************************************************************************/

/******************************************************************************
* Module Typedefs
*******************************************************************************/
typedef uint32_t sensor_data_t;

/******************************************************************************
* Module Variable Definitions
*******************************************************************************/
TaskHandle_t xMQTT_handler = NULL;
static esp_mqtt_client_handle_t mqtt_client_handle = NULL;

sensor_data_t simulate_data = 0;
static Evt mqtt_internal_evt = {0};
/******************************************************************************
* Function Prototypes
*******************************************************************************/

/******************************************************************************
* Function Definitions
*******************************************************************************/
void mqtt_event_handler(void* event_handler_arg, esp_event_base_t event_base, int32_t event_id,  void* event_data)
{
	esp_mqtt_event_handle_t p_evt = (esp_mqtt_event_handle_t)event_data;
	switch (p_evt->event_id)
	{
		case MQTT_EVENT_CONNECTED:
			ESP_LOGI(MODULE_NAME, "MQTT_EVENT_CONNECTED");
			#if (MQTT_TESTING != 0)
			xTaskNotify(xMQTT_handler, MQTT_CLIENT_EVENT_CONNECTED, eSetBits);
			#else /* !(MQTT_TESTING != 0) */
			mqtt_internal_evt.sig = MQTT_CONNECTED;
			Active_post(p_mqtt_actor, (EvtHandle_t)&mqtt_internal_evt);
			#endif /* End of (MQTT_TESTING != 0) */
			
			break;

		case MQTT_EVENT_DISCONNECTED:
			ESP_LOGI(MODULE_NAME, "MQTT_EVENT_DISCONNECTED");
			#if (MQTT_TESTING != 0)
			xTaskNotify(xMQTT_handler, MQTT_CLIENT_EVENT_DISCONNECT, eSetValueWithOverwrite);
			#else /* !(MQTT_TESTING != 0) */
			mqtt_internal_evt.sig = MQTT_DISCONNECTED;
			Active_post(p_mqtt_actor, (EvtHandle_t) &mqtt_internal_evt);
			#endif /* End of (MQTT_TESTING != 0) */
			break;
			
		case MQTT_EVENT_SUBSCRIBED:
			ESP_LOGI(MODULE_NAME, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", p_evt->msg_id);
			break;

		case MQTT_EVENT_UNSUBSCRIBED:
			ESP_LOGI(MODULE_NAME, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", p_evt->msg_id);
			break;

		case MQTT_EVENT_PUBLISHED:
			ESP_LOGI(MODULE_NAME, "MQTT_EVENT_PUBLISHED, msg_id=%d", p_evt->msg_id);
			#if (MQTT_TESTING != 0)
			xTaskNotify(xMQTT_handler, MQTT_CLIENT_EVENT_PUBLISHED, eSetValueWithOverwrite);
			#endif /* End of (MQTT_TESTING != 0) */
			break;

		case MQTT_EVENT_DATA:
			ESP_LOGI(MODULE_NAME, "MQTT_EVENT_DATA");
			ESP_LOGI(MODULE_NAME,"TOPIC=%.*s\r\n", p_evt->topic_len, p_evt->topic);
			ESP_LOGI(MODULE_NAME,"DATA=%.*s\r\n", p_evt->data_len, p_evt->data);
			#if (MQTT_TESTING != 0)
			xTaskNotify(xMQTT_handler, MQTT_CLIENT_EVENT_ONDATA, eSetValueWithOverwrite);
			#endif /* End of (MQTT_TESTING != 0) */
			break;

		case MQTT_EVENT_ERROR:
			ESP_LOGI(MODULE_NAME, "MQTT_EVENT_ERROR");
       		if (p_evt->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) 
			{
				ESP_LOGE(MODULE_NAME,"reported from esp-tls : %s", esp_err_to_name(p_evt->error_handle->esp_tls_last_esp_err));
				ESP_LOGE(MODULE_NAME,"reported from tls stack : %s", esp_err_to_name(p_evt->error_handle->esp_tls_stack_err));
				ESP_LOGE(MODULE_NAME,"captured as transport's socket errno : %s",  esp_err_to_name(p_evt->error_handle->esp_transport_sock_errno));
				ESP_LOGI(MODULE_NAME, "Last errno string (%s)", strerror(p_evt->error_handle->esp_transport_sock_errno));
			}
			break;

		default:
			ESP_LOGI(MODULE_NAME, "Other p_evt id:%d", p_evt->event_id);
			break;
	}
}


void mqtt_task(void* param)
{
#if MQTT_TESTING
	ESP_LOGI(MODULE_NAME, "/************************ MQTT STARTED ************************/\r\n");
	mqtt_client_init();
	while(1)
	{
		uint32_t event_recv = 0;
		char text_temp_buf[5] = {0};

		xTaskNotifyWait(0, ULONG_MAX, &event_recv, portMAX_DELAY); /* Clear all bits when exit*/

		simulate_data = (simulate_data + 1) % 10000;
		sprintf(text_temp_buf, "%04ld", simulate_data);
		ESP_LOGI(MODULE_NAME, "MQTT task is running notify value %ld, sensor value %s", event_recv, text_temp_buf);
		if(event_recv & MQTT_CLIENT_EVENT_CONNECTED)
		{
			ESP_LOGW(MODULE_NAME, "onConnected case");
			esp_mqtt_client_subscribe(mqtt_client_handle, MQTT_DEFAULT_SUB_TOPIC, 0); /* QoS 0 */
			esp_mqtt_client_publish(mqtt_client_handle, MQTT_DEFAULT_STATUS_TOPIC, "on", strlen("on"), 0, 0);
			esp_mqtt_client_publish(mqtt_client_handle, MQTT_DEFAULT_DATA_TOPIC, (const char*)text_temp_buf, sizeof(text_temp_buf), 1, 0);
		}
		if(event_recv & MQTT_CLIENT_EVENT_PUBLISHED)
		{
			ESP_LOGW(MODULE_NAME, "onPublic (QoS 1 or 2 only)");
			esp_mqtt_client_disconnect(mqtt_client_handle);
		}
		if(event_recv & MQTT_CLIENT_EVENT_ONDATA)
		{
			ESP_LOGW(MODULE_NAME, "onData");


		}
		if(event_recv & MQTT_CLIENT_EVENT_DISCONNECT)
		{
			ESP_LOGW(MODULE_NAME, "onDisconnect");
			vTaskDelay(pdMS_TO_TICKS(10000));
			esp_mqtt_client_reconnect(mqtt_client_handle);
		}
	};
#else
	vTaskDelete(NULL);
	return;
#endif /* End of MQTT_TESTING */
	ESP_LOGI(MODULE_NAME, "/************************ MQTT FINISHED ************************/\r\n");
}

#include "actor.h"

#define MQTT_ACTOR_QUEUE_LEN				(20)

Active mqtt_actor = {0};
ActiveId_t p_mqtt_actor = NULL;

// ======================================================================================
static eStatus mqtt_statewait4network(StateMachine_t* const me, const EvtHandle_t p_event);
static eStatus mqtt_statewait4connection(StateMachine_t* const me, const EvtHandle_t p_event);
static eStatus mqtt_state_active(StateMachine_t* const me, const EvtHandle_t p_event);

// MQTT Actor States
static eStatus mqtt_statewait4network(StateMachine_t* const me, const EvtHandle_t p_event)
{
	eStatus status = STATUS_IGNORE;
	ESP_LOGI(MODULE_NAME, "State: mqtt_statewait4network");
	switch (p_event->sig)
	{

		case ENTRY_SIG:
			ESP_LOGI(MODULE_NAME, "Entry: mqtt_statewait4network");
			status = STATUS_HANDLE;
			break;

		case EXIT_SIG:
			ESP_LOGI(MODULE_NAME, "Exit: mqtt_statewait4network");
			status = STATUS_HANDLE;
			break;

		case WIFI_CONNECTED:
			status = TRANSITION(&mqtt_statewait4connection);
			break;

		default:
			break;
	}
	return status;
}

static eStatus mqtt_statewait4connection(StateMachine_t* const me, const EvtHandle_t p_event)
{
	eStatus status = STATUS_IGNORE;
	ESP_LOGI(MODULE_NAME, "State: mqtt_statewait4connection");
	switch (p_event->sig)
	{
		case ENTRY_SIG:
			ESP_LOGI(MODULE_NAME, "Entry: mqtt_statewait4connection");
			for(uint8_t retry=0; retry<3; retry++)
			{
				if( mqtt_connect() == 0)
					break;
				else
				{
					ESP_LOGE(MODULE_NAME, "Failed to connect to MQTT broker, retrying %d/%d", retry+1, 3);
				}
					
			}
			break;	

		case EXIT_SIG:
			ESP_LOGI(MODULE_NAME, "Exit: mqtt_statewait4connection");
			break;

		case WIFI_DISCONNECTED:
			status = TRANSITION(&mqtt_statewait4network);
			break;

		case MQTT_CONNECTED:
			status = TRANSITION(&mqtt_state_active);
			break;

		default:
			break;
	}
	return status;
}

static eStatus mqtt_state_active(StateMachine_t* const me, const EvtHandle_t p_event)
{
	eStatus status = STATUS_IGNORE;
	ESP_LOGI(MODULE_NAME, "State: mqtt_state_active");
	switch (p_event->sig)
	{
		case ENTRY_SIG:
			if( mqtt_publish(MQTT_DEFAULT_STATUS_TOPIC, "on", strlen("on"), 2, 1) != 0)
			{
				ESP_LOGE(MODULE_NAME, "Failed to publish MQTT status");
			}

			ESP_LOGI(MODULE_NAME, "Entry: mqtt_state_active");
			break;

		case EXIT_SIG:
			ESP_LOGI(MODULE_NAME, "Exit: mqtt_state_active");
			break;

		case WIFI_DISCONNECTED:
			status = TRANSITION(&mqtt_statewait4network);
			break;
		
		case MQTT_DISCONNECTED:
			status = TRANSITION(&mqtt_statewait4connection);
			break;

		case SENSOR_DATA_READY:
			sensor_data_evt_t* p_data = (sensor_data_evt_t*)p_event;
			if( mqtt_publish(MQTT_DEFAULT_DATA_TOPIC, p_data->sensor_data_json, strlen(p_data->sensor_data_json), 1, 0) != 0)
			{
				ESP_LOGE(MODULE_NAME, "Failed to publish MQTT data");
			}
			status = STATUS_HANDLE;
			break;

		default:
			break;
	}
	return status;
}

// ======================================================================================
int mqtt_actor_init()
{
	mqtt_client_init();
	Active_Init((Active*)&mqtt_actor, &mqtt_statewait4network, MQTT_TASK_PRIORITY, MQTT_TASK_STACK_SIZE, (void*)0, (void*)0, MQTT_ACTOR_QUEUE_LEN);
	p_mqtt_actor = &mqtt_actor;
	return 0;
}

void mqtt_client_init()
{
	esp_mqtt_client_config_t mqtt_conf = {
        .broker.address.uri = MQTT_BROKER_ADDR,
	};

	#if MQTT_LWT_TESTING
	mqtt_conf.session.keepalive = MQTT_KEEP_ALIVE_PERIOD;
	mqtt_conf.session.last_will = (struct last_will_t){
			.topic = MQTT_DEFAULT_STATUS_TOPIC,
			.msg = "off",
			.msg_len = strlen("off"),
			.qos = 2,
			.retain = true
	};
	#endif /* MQTT_LWT_TESTING */
    mqtt_client_handle = esp_mqtt_client_init(&mqtt_conf);
    assert(mqtt_client_handle);
    esp_mqtt_client_register_event(mqtt_client_handle, ESP_EVENT_ANY_ID, mqtt_event_handler, mqtt_client_handle);
	esp_log_level_set(MODULE_NAME, MODULE_DEFAULT_LOG_LEVEL);
}

int mqtt_connect()
{
	return esp_mqtt_client_start(mqtt_client_handle);
}

int mqtt_disconnect()
{
	return esp_mqtt_client_disconnect(mqtt_client_handle);
}

int mqtt_publish(const char *topic, const char *data, int len, int qos, int retain)
{
	// Check current state
	if (mqtt_state_active != (StateHandler)mqtt_actor.sm.statehandler)
	{
		ESP_LOGE(MODULE_NAME, "MQTT is not connected");
		return -1;
	}

	if (esp_mqtt_client_publish(mqtt_client_handle, topic, data, len, qos, retain) < 0)
		return -1;
	else
		return 0;
}

int mqtt_subscribe(const char *topic, int qos)
{
	if (mqtt_state_active != (StateHandler)mqtt_actor.sm.statehandler)
	{
		ESP_LOGE(MODULE_NAME, "MQTT is not connected");
		return -1;
	}
	if (esp_mqtt_client_subscribe(mqtt_client_handle, topic, qos) < 0)
		return -1;
	else
		return 0;
}