//Wifi Driver (Custom)
//Integrates with ESP-IDF HAL (wifi.h)
//runs on the ESP32 - WIFI hardware is on-chip.

/******************************************************************************
* Module Preprocessor Constants
*******************************************************************************/
#define WIFI_CONFIG_LOAD_CERT_TO_RAM                (1) /* Set to 1 will load cert to "wifi_cert" when write cert*/
#define WIFI_CONFIG_LOAD_CREDENTIAL_NVS             (0) /* Set to 1 to load Wi-Fi credential from NVS */
#define WIFI_CONFIG_AUTORUN_SMARTCONFIG             (0) // 1 - If Wi-Fi is not connected, automatically run SmartConfig
#define WIFI_CONF_AUTO_RECONNECT                    (1) // If Wi-Fi is disconnected, automatically reconnect
#define WIFI_CONF_MAX_RETRY                         (3)

/******************************************************************************
* Typedefs
*******************************************************************************/

typedef struct
{
    
    void (*wifi_sta_connected)(void* p_data, void* p_data_len);
    void (*wifi_sta_disconnected)(void* p_data, void* p_data_len); 
    void (*wifi_sta_failed)(void* p_data, void* p_data_len); // Failed to connect to AP after max retry
}wifi_sta_callback_t;

//Public Functions - Meant for direct use - all block for response to return data.
long wifi_custom__get_time(void); //Implements esp_wifi functions to get the current time.

int wifi_custom_init(wifi_sta_callback_t* station_cb); //Implements esp_wifi functions to initialize the wifi driver. (does not connect to a network   
int wifi_custom__power_on(void); //Implements esp_wifi functions to cleanly start up the wifi driver. Should automatically connect to a network if credentials are saved. (Provisioning handled elsewhere) Returns 0 if ok. Returns -1 if error.
int wifi_custom__power_off(void); //Implements esp_wifi functions to cleanly shutdown the wifi driver. (allows for a future call of wifi_custom_power_on() to work as epxected)
int wifi_custom__connected(void); //Implements esp_wifi functions to determine if currently connected to a network.
int wifi_custom__get_rssi(void); //Implements esp_wifi functions to get the RSSI of the current wifi connection.
int wifi_custom__setCA(char* ca); //Implements esp_wifi functions to set the HTTPS CA cert. Returns 0 if ok. Returns -1 if error.
int wifi_custom__getCA(char* ca, uint32_t ca_max_len);
int wifi_custom__httpsGET(char* url, char* response, uint16_t maxlength); //if url = "google.com/myurl"Implements esp_wifi functions to send a GET request via HTTPS. Returns 0 if ok. Returns -1 if error. Returns response in response char array.
int wifi_custom__httpsPOST(char* url, char* JSONdata, char* agent, char* response, uint16_t maxlength); //if url = "google.com/myurl" Implements esp_wifi functions to send a POST request via HTTPS. Returns 0 if ok. Returns -1 if error. Returns response in response char array.

int wifi_custom__getData(char* data, uint16_t maxlength, bool block); //returns number of characters read if ok. if "block" is true, wait for the next HTTP Response. Handles HTTPS Responses. 
int wifi_custom_test_https_get();
int wifi_custom_test_https_post();
int smartconfig_init();
int smartconfig_start();
int smartconfig_stop();



