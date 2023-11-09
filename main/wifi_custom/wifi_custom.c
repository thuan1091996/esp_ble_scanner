/*******************************************************************************
* Title                 :   
* Filename              :   wifi_custom.c
* Author                :   ItachiVN
* Origin Date           :   2023/25/05
* Version               :   0.0.0
* Compiler              :   
* Target                :   
* Notes                 :   None
*******************************************************************************/

/*************** MODULE REVISION LOG ******************************************
*
*    Date       Software Version	Initials	Description
*  2023/08/06       0.0.0	         ItachiVN      Module Created.
*
*******************************************************************************/

/** \file wifi_custom.c
 *  \brief This module contains the
 */
/******************************************************************************
* Includes
*******************************************************************************/
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "nvs_flash.h"

#include "esp_system.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_event_base.h"
#include "esp_sntp.h"
#include "esp_smartconfig.h"
#include "esp_sntp.h"


/* User libs */
#include "wifi_custom.h"
#include "../task_common.h"
/******************************************************************************
* Module Preprocessor Constants
*******************************************************************************/
#define WIFI_HTTPS_DEFAULT_TIMEOUT_MS   (10000)
#define WIFI_CONN_TIMEOUT_MS            (5000) /* In ms */
#define WIFI_SSID_MAX_LEN               (32)
#define WIFI_PASS_MAX_LEN               (64)
#define WIFI_CERT_MAX_LEN               (2048)


#define MODULE_DEFAULT_LOG_LEVEL        ESP_LOG_WARN
#define WIFI_CONNECTED_BIT 			    BIT0

#define ESPTOUCH_GOT_CREDENTIAL         BIT1
#define ESPTOUCH_TIMEOUT_MS             (60000) /* In ms */

#define EXAMPLE_ESP_WIFI_SSID           "thuantm5"
#define EXAMPLE_ESP_WIFI_PASS           "123456789"

/******************************************************************************
* Module Preprocessor Macros
*******************************************************************************/

/******************************************************************************
* Module Typedefs
*******************************************************************************/

/******************************************************************************
* Module Variable Definitions
*******************************************************************************/
static uint8_t ssid[WIFI_SSID_MAX_LEN] = { 0 };
static uint8_t password[WIFI_PASS_MAX_LEN] = { 0 };
static char wifi_cert[WIFI_CERT_MAX_LEN] = {0};  
/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group = NULL;
static EventGroupHandle_t smartconf_event_group = NULL;

wifi_sta_callback_t wifi_station_cb = {0};
/******************************************************************************
* Function Prototypes
*******************************************************************************/
void sntp_got_time_cb(struct timeval *tv)
{
    struct tm *time_now = localtime(&tv->tv_sec);
    char time_str[50]={0};
    strftime(time_str, sizeof(time_str), "%c", time_now);
    ESP_LOGW("SNTP", "Curtime: %s \r\n",time_str);
}

void sntp_time_init()
{
    /* SNTP */
    ESP_LOGI("SNTP", "Initializing SNTP");
    esp_sntp_setoperatingmode(ESP_SNTP_OPMODE_POLL);
    sntp_set_sync_mode(SNTP_SYNC_MODE_IMMED);
    sntp_setservername(0, "pool.ntp.org");
    sntp_set_time_sync_notification_cb(&sntp_got_time_cb);
    sntp_init();
}

void smartconfig_event_handler(void* event_handler_arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
	ESP_LOGI("esptouch", "Event handler invoked \r\n");
    if (event_base == SC_EVENT)
    {
        if (event_id == SC_EVENT_SCAN_DONE) 
        {
            ESP_LOGI("esptouch", "Scan done");
        }
        else if (event_id == SC_EVENT_FOUND_CHANNEL) 
        {
            ESP_LOGI("esptouch", "Found channel");
        }
        else if (event_id == SC_EVENT_GOT_SSID_PSWD) 
        {
            ESP_LOGI("esptouch", "Got SSID and password with Smartconfig");

            smartconfig_event_got_ssid_pswd_t *evt = (smartconfig_event_got_ssid_pswd_t *)event_data;
            wifi_config_t wifi_config;
            uint8_t rvd_data[33] = { 0 };

            bzero(&wifi_config, sizeof(wifi_config_t));
            memcpy(wifi_config.sta.ssid, evt->ssid, sizeof(wifi_config.sta.ssid));
            memcpy(wifi_config.sta.password, evt->password, sizeof(wifi_config.sta.password));
            wifi_config.sta.bssid_set = evt->bssid_set;
            if (wifi_config.sta.bssid_set == true) {
                memcpy(wifi_config.sta.bssid, evt->bssid, sizeof(wifi_config.sta.bssid));
            }

            memcpy(ssid, evt->ssid, sizeof(evt->ssid));
            memcpy(password, evt->password, sizeof(evt->password));
            ESP_LOGI("esptouch", "SSID:%s", ssid);
            ESP_LOGI("esptouch", "PASSWORD:%s", password);
            if (evt->type == SC_TYPE_ESPTOUCH_V2) {
                ESP_ERROR_CHECK( esp_smartconfig_get_rvd_data(rvd_data, sizeof(rvd_data)) );
                ESP_LOGI("esptouch", "RVD_DATA:");
                for (int i=0; i<33; i++) {
                    printf("%02x ", rvd_data[i]);
                }
                printf("\n");
            }

            // Store SSID and password in NVS
            ESP_LOGI("esptouch", "Storing SSID: %s, password: %s in NVS", wifi_config.sta.ssid, wifi_config.sta.password);
            ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
            xEventGroupSetBits(smartconf_event_group, ESPTOUCH_GOT_CREDENTIAL);
        }
        else if (event_id == SC_EVENT_SEND_ACK_DONE) 
        {
            ESP_LOGI("esptouch", "Smartconfig finished send ACK");
        }
        else
        {
            ESP_LOGW("esptouch", "Unknown SC_EVENT %ld", event_id);
        }
    }
    else
    {
        ESP_LOGW("esptouch", "Unknown event base %s, event id %ld", event_base, event_id);
    }
}

/*
 * @brief: Init smartconfig
 * 
 * @return int: 0 if success, -1 if error 
 */
int smartconfig_init()
{
    esp_err_t err = esp_smartconfig_set_type(SC_TYPE_ESPTOUCH);
    if (err != ESP_OK)
    {
        ESP_LOGE("esptouch", "Smartconfig type set failed with err %d", err);
        return -1;
    }

    if(ESP_OK != esp_event_handler_register(SC_EVENT, ESP_EVENT_ANY_ID, smartconfig_event_handler, NULL) )
    {
        ESP_LOGE("esptouch", "Failed to register Smartconfig event handler");
        return -1;
    }

    smartconf_event_group = xEventGroupCreate();
    if(smartconf_event_group == NULL)
    {
        ESP_LOGE("esptouch", "Failed to create Smartconfig event group");
        return -1;
    }
    return 0;
}

/*
 * @brief: Start smartconfig
 * 
 * @return int: 0 if able to obtain credential, -1 if failed after timeout
 */
int smartconfig_start()
{
    int err;
    smartconfig_start_config_t cfg = SMARTCONFIG_START_CONFIG_DEFAULT();
    err = esp_smartconfig_start(&cfg);
    if (err != ESP_OK)
    {
        ESP_LOGE("esptouch", "Smartconfig start failed with err %d", err);
        return -1;
    }


    /* Waiting for smartconfig to finish */
    EventBits_t bits = xEventGroupWaitBits(smartconf_event_group, ESPTOUCH_GOT_CREDENTIAL,
                                           pdFALSE,
                                           pdFALSE,
                                           pdMS_TO_TICKS(ESPTOUCH_TIMEOUT_MS));
    if (bits & ESPTOUCH_GOT_CREDENTIAL)
    {
        ESP_LOGI("esptouch", "Obtained SSID:%s password:%s", ssid, password);
        esp_smartconfig_stop();
        return 0;
    }
    else
    {
        ESP_LOGE("esptouch", "Smartconfig timeout");
        return -1;
    }
}

int smartconfig_stop()
{
    return esp_smartconfig_stop();
}

void wifi_event_handler(void* event_handler_arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
	ESP_LOGI("wifi_custom", "Event handler invoked \r\n");
    /* Handle IP_EVENT */
    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
	{
		ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
		ESP_LOGI("wifi_custom", "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        if(wifi_station_cb.wifi_sta_connected != NULL)
        {
            wifi_station_cb.wifi_sta_connected(NULL, NULL);
        }
		xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
	}
    /* Handle Wi-Fi event */
    else if(event_base == WIFI_EVENT)
    {
        if (event_id == WIFI_EVENT_STA_START)
        {
            ESP_LOGI("wifi_custom", "WIFI_EVENT_STA_START");
            ESP_LOGI("wifi_custom", "Wi-Fi STATION started successfully \r\n");
        }
        else if (event_id == WIFI_EVENT_STA_DISCONNECTED)
        {
            wifi_event_sta_disconnected_t* event = (wifi_event_sta_disconnected_t*) event_data;
            ESP_LOGW("wifi_custom", "Wi-Fi disconnected, reason: %d", event->reason);        
            switch (event->reason)
            {

                case WIFI_REASON_ASSOC_LEAVE:
                {
                    ESP_LOGE("wifi_custom", "STA left");
                    break;
                }

                case WIFI_REASON_AUTH_FAIL:
                {
                    ESP_LOGE("wifi_custom", "Authentication failed. Wrong credentials provided.");
                    break;
                }

                case WIFI_REASON_NO_AP_FOUND:
                {
                    ESP_LOGE("wifi_custom", "STA AP Not found");
                    break;
                }

                default:
                {

                }
                break;
            }
            xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
            if (esp_wifi_disconnect() != 0)
            {
                ESP_LOGE("wifi_custom", "Failed to disconnect Wi-Fi");
            }
            if (esp_wifi_stop() != ESP_OK)
            {
                ESP_LOGE("wifi_custom", "Failed to stop Wi-Fi");
            }
            if(wifi_station_cb.wifi_sta_disconnected != NULL)
            {
                wifi_station_cb.wifi_sta_disconnected(NULL, NULL);
            }
#if (WIFI_CONF_AUTO_RECONNECT != 0)
            if (0!= wifi_custom__power_on())
            {
                ESP_LOGE("wifi_custom", "Failed to reconnect to Wi-Fi");
            }
            else
            {
                ESP_LOGI("wifi_custom", "Connected to Wi-Fi");
            }
#endif /* End of (WIFI_CONF_AUTO_RECONNECT != 0) */
            if(wifi_station_cb.wifi_sta_failed != NULL)
            {
                wifi_station_cb.wifi_sta_failed(NULL, NULL);
            }
        }
        else
        {
            ESP_LOGW("wifi_custom", "Unknown WIFI_EVENT %ld", event_id);
        }
    }
    else
    {
        ESP_LOGW("wifi_custom", "Unknown event base %s, event id %ld", event_base, event_id);
    }
}

int wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();
    if(s_wifi_event_group == NULL)
    {
        ESP_LOGE("wifi_custom", "Failed to create Wi-Fi event group");
        return -1;
    }
    do
    {
        /* Create and init lwIP related stuffs */
        if(ESP_OK !=esp_netif_init())
            break;

        /* Create default event loop */
        if(ESP_OK !=esp_event_loop_create_default())
            break;

        esp_netif_create_default_wifi_sta();

        /* Create default network interface instance binding to netif */
        if(ESP_OK != esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL) )
            break; 
        if(ESP_OK != esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL) )
            break;

        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        if(ESP_OK !=esp_wifi_init(&cfg))
            break;

        if(ESP_OK != esp_wifi_set_mode(WIFI_MODE_STA))
            break;

        wifi_config_t wifi_config = {
            .sta = {
                .ssid = EXAMPLE_ESP_WIFI_SSID,
                .password = EXAMPLE_ESP_WIFI_PASS,
                /* Setting a password implies station will connect to all security modes including WEP/WPA.
                * However these modes are deprecated and not advisable to be used. Incase your Access point
                * doesn't support WPA2, these mode can be enabled by commenting below line */
                .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            },
        };
        memcpy(ssid, EXAMPLE_ESP_WIFI_SSID, strlen(EXAMPLE_ESP_WIFI_SSID));
        memcpy(password, EXAMPLE_ESP_WIFI_PASS, strlen(EXAMPLE_ESP_WIFI_PASS));

        #if WIFI_CONFIG_LOAD_CREDENTIAL_NVS
            wifi_config_t wifi_config_loaded = {0};
            if (esp_wifi_get_config(WIFI_IF_STA, &wifi_config_loaded) == ESP_OK)
            {
                size_t ssidLen = strlen((char*)wifi_config_loaded.sta.ssid);
                if(ssidLen != 0)
                {
                    memcpy(&wifi_config, &wifi_config_loaded, sizeof(wifi_config_t));
                    memcpy(ssid, wifi_config_loaded.sta.ssid, strlen((char*)wifi_config_loaded.sta.ssid));
                    memcpy(password, wifi_config_loaded.sta.password, strlen((char*)wifi_config_loaded.sta.password));
                    ESP_LOGI("wifi_custom", "SSID loaded from NVS: %s", ssid);
                    ESP_LOGI("wifi_custom", "PASSWORD: %s", password);
                }
                else
                {
                    ESP_LOGE("wifi_custom", "No SSID stored in NVS");
                }
            }
        #endif /* End of WIFI_CONFIG_LOAD_CREDENTIAL_NVS */

            ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
            ESP_LOGI("wifi_custom", "wifi_init_sta finished.");
            sntp_time_init();
            return 0;
    } while (0);
    
    return -1;
}

int wifi_custom_init(wifi_sta_callback_t* station_cb)
{
    esp_log_level_set("wifi_custom", ESP_LOG_INFO);
    if ( wifi_custom__getCA(wifi_cert, WIFI_CERT_MAX_LEN) != 0)
    {
        ESP_LOGE("wifi_http", "Failed to get certificate");
    }

    ESP_LOGI("wifi_custom", "Initializing Wi-Fi station \r\n");
    if ( wifi_init_sta() != 0)
    {
        ESP_LOGE("wifi_custom", "Failed to initialize Wi-Fi station");
        return -1;
    }

#if (WIFI_CONFIG_AUTORUN_SMARTCONFIG != 0)
    if ( 0 != smartconfig_init())
    {
        ESP_LOGE("wifi_custom", "Failed to initialize SmartConfig");
    }
#endif /* End of (WIFI_CONFIG_AUTORUN_SMARTCONFIG != 0) */

    if(station_cb != NULL)
    {
        wifi_station_cb = *station_cb;
    }
    esp_log_level_set("wifi_custom", MODULE_DEFAULT_LOG_LEVEL);
    return 0;
}


/******************************************************************************
* Function Definitions
*******************************************************************************/
//Implements esp_wifi functions to cleanly start up the wifi driver. Should automatically connect to a network if credentials are saved. (Provisioning handled elsewhere) Returns 0 if ok. Returns -1 if error.
int wifi_custom__power_on(void)
{
    if(esp_wifi_start() != ESP_OK)
    {
        ESP_LOGE("wifi_custom", "esp_wifi_start() failed");
        return -1;
    }
    esp_err_t status = esp_wifi_connect();
    if(status != ESP_OK)
    {
        ESP_LOGE("wifi_custom", "esp_wifi_connect() failed with error %d", status);
    }
    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or got credential from smartconfig*/
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | ESPTOUCH_GOT_CREDENTIAL,
            pdFALSE,
            pdFALSE,
            pdMS_TO_TICKS(WIFI_CONN_TIMEOUT_MS));

    if ((bits & WIFI_CONNECTED_BIT) || (bits & ESPTOUCH_GOT_CREDENTIAL))
    {
        if(bits & ESPTOUCH_GOT_CREDENTIAL) 
        {
            ESP_LOGI("wifi_custom", "connected to ap SSID:%s password:%s", ssid, password);
        }
        return 0;
    }
    ESP_LOGE("wifi_custom", "Failed to connect to SSID:%s, password:%s", ssid, password);
#if (WIFI_CONFIG_AUTORUN_SMARTCONFIG != 0)
        ESP_LOGE("wifi_custom", "Failed to connect to Wi-Fi. Running ESP SmartConfig...");
        wifi_custom__power_off();
        if (smartconfig_start() == 0)
        {
            ESP_LOGI("wifi_custom", "Obtained Wi-Fi credential with Smartconfig, trying to connect");
            return wifi_custom__power_on();
        }
        else
        {
            ESP_LOGE("wifi_custom", "Failed to get Wi-Fi credential with smartconfig after timeout");
            smartconfig_stop();
        }
        smartconfig_stop();
#endif /* End of (WIFI_CONFIG_AUTORUN_SMARTCONFIG != 0) */
    return -1;
} 
//Implements esp_wifi functions to cleanly shutdown the wifi driver. (allows for a future call of wifi_custom_power_on() to work as epxected)
int wifi_custom__power_off(void)
{
    return esp_wifi_disconnect();
}

//Implements esp_wifi functions to determine if currently connected to a network.
int wifi_custom__connected(void)
{
	EventBits_t bit_mask = xEventGroupGetBits(s_wifi_event_group);
    if(bit_mask & WIFI_CONNECTED_BIT)
    {
        ESP_LOGI("wifi_custom", "Wi-Fi is connected");
        return 1;
    }
    else
    {   
        ESP_LOGI("wifi_custom", "Wi-Fi is not connected");
        return 0;
    }
}

//Implements esp_wifi functions to get the current time.
long wifi_custom__get_time(void)
{
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);
    // Is time set? If not, tm_year will be (1970 - 1900).
    if (timeinfo.tm_year < (2016 - 1900)) 
    {
        ESP_LOGI("SNTP", "Time is not set yet. Getting time info");
        int retry=0, retry_count=10;
        while (sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && ++retry < retry_count) 
        {
            ESP_LOGI("SNTP", "Waiting for system time to be set... (%d/%d)", retry, retry_count);
            vTaskDelay(2000 / portTICK_PERIOD_MS);
            if(retry == (retry_count-1))
            {
                ESP_LOGE("SNTP", "Failed to get time from SNTP server");
                return -1;
            }
        }
            //TODO: Set time zone
        setenv("TZ","UTC+7",1);
        tzset();
        time(&now);
        localtime_r(&now, &timeinfo);
        // update 'now' variable with current time
        time(&now);
    }

    int timestamp = time(NULL);
    ESP_LOGI("SNTP", "Unix timestamp: %d", timestamp);
    return timestamp;
}
//Implements esp_wifi functions to get the RSSI of the current wifi connection.
int wifi_custom__get_rssi(void)
{
    wifi_ap_record_t ap_info;
    esp_err_t status = esp_wifi_sta_get_ap_info(&ap_info);
    if(status == ESP_ERR_WIFI_NOT_CONNECT)
    {
        ESP_LOGE("wifi_custom", "Wi-Fi is not connected");
        return -1;
    }
    else if(status != ESP_OK)
    {
        ESP_LOGE("wifi_custom", "esp_wifi_sta_get_ap_info() failed with error %d", status);
        return -1;
    }
    ESP_LOGI("wifi_custom", "RSSI of SSID %s: %d", ap_info.ssid, ap_info.rssi);
    return ap_info.rssi;
}

int wifi_custom__getCA(char* cert, uint32_t cert_max_len)
{
    // Open the "certs" namespace in read-only mode
    nvs_handle handle;
    if (nvs_open("certs", NVS_READONLY, &handle) != ESP_OK)
    {
        ESP_LOGE("wifi_custom", "Failed to open NVS");
        return -1;
    }

    // Load the certificate
    ESP_LOGI("wifi_custom", "Loading certificate");

    char* value = NULL;
    do
    {
        size_t value_size;

        // Try to get the size of the item
        if(nvs_get_str(handle, "certificate", NULL, &value_size) != ESP_OK){
            ESP_LOGE("wifi_custom", "Failed to get size of key: %s", "certificate");
            break;
        }

        if(value_size > cert_max_len)
        {
            ESP_LOGE("wifi_custom", "Certificate size is too large");
            break;
        }

        value = malloc(value_size);
        if (value == NULL)
        {
            ESP_LOGE("wifi_custom", "Failed to allocate memory for certificate");
            nvs_close(handle);
            return -1;
        }

        if(nvs_get_str(handle, "certificate", value, &value_size) != ESP_OK) {
            ESP_LOGE("wifi_custom", "Failed to load key: %s", "certificate");
            break;
        }

        if(value == NULL){
            ESP_LOGE("wifi_custom", "Certificate could not be loaded");
            break;
        }

        nvs_close(handle);

        // Print the certificate
        ESP_LOGI("wifi_custom", "Certificate: %s", value);
        memcpy(cert, value, value_size);
        free(value);
        return 0;

    }while(0);

    free(value);
    return -1;
}

int wifi_custom__setCA(char* cert)
{
     // Open the "certs" namespace
    nvs_handle handle;
    if(nvs_open("certs", NVS_READWRITE, &handle) != ESP_OK)
    {
        ESP_LOGE("wifi_custom", "Failed to open NVS");
        return -1;
    }

    do
    {
        ESP_LOGI("wifi_custom", "Writing certificate");
        if (nvs_set_str(handle, "certificate", cert) != ESP_OK)
        {
            ESP_LOGE("wifi_custom", "Failed to write key: %s", "certificate");
            break;
        }

        // Commit written value and close
        if (nvs_commit(handle) != ESP_OK)
        {
            ESP_LOGE("wifi_custom", "Failed to commit NVS");
            break;
        }

        ESP_LOGI("wifi_custom", "Certificate written successfully");

#if (WIFI_CONFIG_LOAD_CERT_TO_RAM != 0) 
        memset(wifi_cert, 0, sizeof(wifi_cert));
        strcpy(wifi_cert, cert);
#endif /*(WIFI_CONFIG_LOAD_CERT_TO_RAM != 0) */

        nvs_close(handle);
        return 0;
        
    }while(0);
    // Write the certificate
    nvs_close(handle);
    return -1;
}