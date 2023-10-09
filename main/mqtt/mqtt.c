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


#if MQTT_TESTING

#define MQTT_KEEP_ALIVE_PERIOD          				(10)
#define MQTT_LWT_TESTING                				(1)

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
static esp_mqtt_client_handle_t client = NULL;

sensor_data_t simulate_data = 0;
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
			xTaskNotify(xMQTT_handler, MQTT_CLIENT_EVENT_CONNECTED, eSetBits);
			break;

		case MQTT_EVENT_DISCONNECTED:
			ESP_LOGI(MODULE_NAME, "MQTT_EVENT_DISCONNECTED");
			xTaskNotify(xMQTT_handler, MQTT_CLIENT_EVENT_DISCONNECT, eSetValueWithOverwrite);
			break;
			
		case MQTT_EVENT_SUBSCRIBED:
			ESP_LOGI(MODULE_NAME, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", p_evt->msg_id);
			break;

		case MQTT_EVENT_UNSUBSCRIBED:
			ESP_LOGI(MODULE_NAME, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", p_evt->msg_id);
			break;

		case MQTT_EVENT_PUBLISHED:
			ESP_LOGI(MODULE_NAME, "MQTT_EVENT_PUBLISHED, msg_id=%d", p_evt->msg_id);
			xTaskNotify(xMQTT_handler, MQTT_CLIENT_EVENT_PUBLISHED, eSetValueWithOverwrite);
			break;

		case MQTT_EVENT_DATA:
			ESP_LOGI(MODULE_NAME, "MQTT_EVENT_DATA");
			ESP_LOGI(MODULE_NAME,"TOPIC=%.*s\r\n", p_evt->topic_len, p_evt->topic);
			ESP_LOGI(MODULE_NAME,"DATA=%.*s\r\n", p_evt->data_len, p_evt->data);
			xTaskNotify(xMQTT_handler, MQTT_CLIENT_EVENT_ONDATA, eSetValueWithOverwrite);
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

void mqtt_client_init()
{
	esp_mqtt_client_config_t mqtt_conf = {
        .broker.address.uri = MQTT_BROKER_ADDR,
		.session.disable_clean_session = true,
	};


	#if MQTT_LWT_TESTING
	mqtt_conf.session.keepalive = MQTT_KEEP_ALIVE_PERIOD;
	mqtt_conf.session.last_will = (struct last_will_t){
			.topic = MQTT_STATUS_TOPIC,
			.msg = "off",
			.msg_len = strlen("off"),
			.qos = 1,
			.retain = true
		};
	#endif /* MQTT_LWT_TESTING */
    client = esp_mqtt_client_init(&mqtt_conf);
    assert(client);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, client);
    esp_mqtt_client_start(client);

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
			esp_mqtt_client_subscribe(client, MQTT_SUB_TOPIC, 0); /* QoS 0 */
			esp_mqtt_client_publish(client, MQTT_STATUS_TOPIC, "on", strlen("on"), 0, 0);
			esp_mqtt_client_publish(client, MQTT_DATA_TOPIC, (const char*)text_temp_buf, sizeof(text_temp_buf), 1, 0);
		}
		if(event_recv & MQTT_CLIENT_EVENT_PUBLISHED)
		{
			ESP_LOGW(MODULE_NAME, "onPublic (QoS 1 or 2 only)");
			esp_mqtt_client_disconnect(client);
		}
		if(event_recv & MQTT_CLIENT_EVENT_ONDATA)
		{
			ESP_LOGW(MODULE_NAME, "onData");


		}
		if(event_recv & MQTT_CLIENT_EVENT_DISCONNECT)
		{
			ESP_LOGW(MODULE_NAME, "onDisconnect");
			vTaskDelay(pdMS_TO_TICKS(10000));
			esp_mqtt_client_reconnect(client);
		}
	};
#else
	vTaskDelete(NULL);
	return;
#endif /* End of MQTT_TESTING */
	ESP_LOGI(MODULE_NAME, "/************************ MQTT FINISHED ************************/\r\n");
}

