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
#define WIFI_RETRY_CONN_MAX             (5)
#define WIFI_SSID_MAX_LEN               (32)
#define WIFI_PASS_MAX_LEN               (64)
#define WIFI_CERT_MAX_LEN               (2048)
#define WIFI_CONFIG_LOAD_CERT_TO_RAM    (1) /* Set to 1 will load cert to "wifi_cert" when write cert*/
#define WIFI_CONFIG_LOAD_CREDENTIAL_NVS (0) /* Set to 1 to load Wi-Fi credential from NVS */


#define WIFI_CONNECTED_BIT 			BIT0
#define WIFI_FAIL_BIT      			BIT1
#define ESPTOUCH_GOT_CREDENTIAL     BIT2


#define EXAMPLE_ESP_WIFI_SSID      "thuantm5"
#define EXAMPLE_ESP_WIFI_PASS      "12345678"

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
static EventGroupHandle_t s_wifi_event_group;
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

int smartconfig_init()
{
    esp_err_t err = esp_smartconfig_set_type(SC_TYPE_ESPTOUCH);
    if (err != ESP_OK)
    {
        ESP_LOGE("wifi_custom", "Smartconfig type set failed with err %d", err);
        return -1;
    }
    smartconfig_start_config_t cfg = SMARTCONFIG_START_CONFIG_DEFAULT();
    err = esp_smartconfig_start(&cfg);
    if (err != ESP_OK)
    {
        ESP_LOGE("wifi_custom", "Smartconfig start failed with err %d", err);
        return -1;
    }
    return 0;
}

void wifi_on_connected_cb(void)
{
    ESP_LOGW("wifi_custom", "On Wi-Fi connected callback");
}

void wifi_event_handler(void* event_handler_arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
	static int s_retry_num = 0;
	ESP_LOGI("wifi_custom", "Event handler invoked \r\n");
	if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
	{
        ESP_LOGI("wifi_custom", "WIFI_EVENT_STA_START");
		ESP_LOGI("wifi_custom", "Wi-Fi STATION started successfully \r\n");
	}
	else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
	{
        ESP_LOGI("wifi_custom", "WIFI_EVENT_STA_DISCONNECTED");
        wifi_event_sta_disconnected_t* event = (wifi_event_sta_disconnected_t*) event_data;
        ESP_LOGW("wifi_custom", "Wi-Fi disconnected, reason: %d", event->reason);        
        switch (event->reason)
        {

            case WIFI_REASON_ASSOC_LEAVE:
            {
                ESP_LOGE("wifi_custom", "STA left");
                xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
                break;
            }

            case WIFI_REASON_AUTH_FAIL:
            {
                ESP_LOGE("wifi_custom", "Authentication failed. Wrong credentials provided.");
                ESP_LOGW("wifi_custom", "Start smartconfig to update credentials");
                xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
                break;
            }
            case WIFI_REASON_NO_AP_FOUND:
            {
                ESP_LOGE("wifi_custom", "STA AP Not found");
            }
            // case WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT:
            // case WIFI_REASON_AUTH_EXPIRE:

            default:
            {
                if (s_retry_num < 3)
                {
                    esp_wifi_connect();
                    s_retry_num++;
                    xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
                    ESP_LOGI("wifi_custom", "retry to connect to the AP");
                }
                else if(s_retry_num < WIFI_RETRY_CONN_MAX)
                {
                    ESP_LOGW("wifi_custom", "Start smartconfig to update credentials");
                    xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
                    s_retry_num++;
                }
                else
                {
                    xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
                }
            }
            break;
        }
	}

	else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
	{
        ESP_LOGI("wifi_custom", "IP_EVENT_STA_GOT_IP");
		ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
		ESP_LOGI("wifi_custom", "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
		s_retry_num = 0;
		xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
	}
    else if (event_base == SC_EVENT && event_id == SC_EVENT_SCAN_DONE) 
    {
        ESP_LOGI("wifi_custom", "Scan done");
    }
    else if (event_base == SC_EVENT && event_id == SC_EVENT_FOUND_CHANNEL) 
    {
        ESP_LOGI("wifi_custom", "Found channel");
    } 
    else if (event_base == SC_EVENT && event_id == SC_EVENT_GOT_SSID_PSWD) 
    {
        ESP_LOGI("wifi_custom", "Got SSID and password");

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
        ESP_LOGI("wifi_custom", "SSID:%s", ssid);
        ESP_LOGI("wifi_custom", "PASSWORD:%s", password);
        if (evt->type == SC_TYPE_ESPTOUCH_V2) {
            ESP_ERROR_CHECK( esp_smartconfig_get_rvd_data(rvd_data, sizeof(rvd_data)) );
            ESP_LOGI("wifi_custom", "RVD_DATA:");
            for (int i=0; i<33; i++) {
                printf("%02x ", rvd_data[i]);
            }
            printf("\n");
        }

        ESP_ERROR_CHECK( esp_wifi_disconnect() );
        // Store SSID and password in NVS
        ESP_LOGI("wifi_custom", "Storing SSID: %s, password: %s in NVS", wifi_config.sta.ssid, wifi_config.sta.password);
        ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
        xEventGroupSetBits(s_wifi_event_group, ESPTOUCH_GOT_CREDENTIAL);
        esp_wifi_connect();
    } 
    else if (event_base == SC_EVENT && event_id == SC_EVENT_SEND_ACK_DONE) 
    {
        ESP_LOGI("wifi_custom", "Smartconfig finished send ACK");
        esp_smartconfig_stop();
    }
}

int wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

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
        if(ESP_OK != esp_event_handler_register(SC_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL) )
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

int wifi_custom_init(void)
{
    esp_log_level_set("wifi_custom", ESP_LOG_INFO);
    if ( wifi_custom__getCA(wifi_cert, WIFI_CERT_MAX_LEN) != 0)
    {
        ESP_LOGE("wifi_http", "Failed to get certificate");
    }

    ESP_LOGI("wifi_custom", "Initializing Wi-Fi station \r\n");

    return wifi_init_sta();
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
    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT | ESPTOUCH_GOT_CREDENTIAL,
            pdFALSE,
            pdFALSE,
            pdMS_TO_TICKS(15000));

    if ((bits & WIFI_CONNECTED_BIT) || (bits & ESPTOUCH_GOT_CREDENTIAL))
    {
        if(bits & ESPTOUCH_GOT_CREDENTIAL) 
        {
            ESP_LOGI("wifi_custom", "connected to ap SSID:%s password:%s", ssid, password);
        }
        wifi_on_connected_cb();
        return 0;
    }
    else if (bits & WIFI_FAIL_BIT)
    {
        ESP_LOGE("wifi_custom", "Failed to connect to SSID:%s, password:%s", ssid, password);
    }
    else
    {
        ESP_LOGE("wifi_custom", "Timeout to connect");
    }
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