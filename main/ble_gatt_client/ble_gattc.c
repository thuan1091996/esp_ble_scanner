/*******************************************************************************
 * Title                 :   ESP-IDF BLE GATT Client src file
 * Filename              :   ble_gattc.c
 * Author                :   ItachiVN
 * Origin Date           :   2023/10/09
 * Version               :   0.0.0
 * Compiler              :   ESP-IDF V5.0.2
 * Target                :   ESP32
 * Notes                 :   None
 *******************************************************************************/

/*************** MODULE REVISION LOG ******************************************
 *
 *    Date       Software Version	Initials	Description
 *  2023/10/09       0.0.0	         ItachiVN      Module Created.
 *
 *******************************************************************************/

/** \file ble_gattc.c
 *  \brief This module contains the
 */
/******************************************************************************
 * Includes
 *******************************************************************************/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "nvs_flash.h"

#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"

#include "ble_gattc.h"
#include "task_common.h"

/******************************************************************************
 * Module Preprocessor Constants
 *******************************************************************************/
#define MODULE_NAME                             "BLE_GATTC"
#define MODULE_DEFAULT_LOG_LEVEL                ESP_LOG_DEBUG

#define BLE_GATTC_CONF_WHITE_LIST               1
#define BLE_GATTC_CONF_WHITE_LIST_MAX           4


#define REMOTE_SERVICE_UUID                     0xFF00
#define REMOTE_NOTIFY_CHAR_UUID                 0xFF01
/******************************************************************************
 * Module Preprocessor Macros
 *******************************************************************************/
#define GATTC_INTERFACE_READY(APP_ID)           ((gl_profile_tab[APP_ID].gattc_if != ESP_GATT_IF_NONE)?true:false)

/******************************************************************************
 * Module Typedefs
 *******************************************************************************/

/* GATT attribute typedef */
typedef struct
{
    esp_bt_uuid_t               service_uuid;           /*!< The service uuid */
    uint16_t                    service_start_handle;   /*!< The service start handle */
    uint16_t                    service_end_handle;     /*!< The service end handle */
}service_info_t;

typedef struct
{
    esp_bt_uuid_t               char_uuid;          /*!< The characteristic uuid */
    esp_gatt_char_prop_t        char_properties;    /*!< The characteristic properties */
    uint16_t                    char_handle;        /*!< The characteristic handle */
}gatt_char_info_t;

typedef struct
{
    gatt_char_info_t            char_info;
    uint16_t                    descr_handle;       /*!< The characteristic descriptor handle */
    ble_gatt_ccc_cb             char_ccc_cb;        /*!< The characteristic CCC callback */
}gatt_char_with_ccc_info_t;


// GATT data structure
struct gattc_profile_inst
{
    esp_gattc_cb_t gattc_cb;        // ✔ set
    // Info related to specific connection
    uint16_t gattc_if;              // <-> Relate directly to profile ID // ✔ set
    uint16_t conn_id;               // <-> Relate to virtual connection with server // ✔ set
    esp_bd_addr_t remote_bda;       // <-> Relate to virtual connection with server // ✔ set
    
    // Info related to specific service definition
    uint16_t service_start_handle;  // ✔ set
    uint16_t service_end_handle;    // ✔ set
    // Info related to specific characteristic definition
    uint16_t char_handle;           // ✔ set
    uint16_t descr_handle;          // ✔ set
    // Nofication/Indication callback
    ble_gatt_ccc_cb char_ccc_changed_cb;    // ✔ set
};

/******************************************************************************
 * Function Prototypes
 *******************************************************************************/
void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);


// Test functions
int profile_a_ccc_changed_callback(uint16_t gattc_if, uint16_t ccc_type, void* p_data, void* p_len);
/******************************************************************************
 * Module Variable Definitions
 *******************************************************************************/

// GATT data connected to GATT event handler
static struct gattc_profile_inst gl_profile_tab[PROFILE_NUM_MAX] = {
    [PROFILE_A_APP_ID] = {
        .gattc_cb = gattc_profile_event_handler,
    },
};

static ble_client_callback_t ble_client_callback = {0};

static int g_current_app_id = PROFILE_A_APP_ID;
static esp_bt_uuid_t remote_filter_service_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {.uuid16 = REMOTE_SERVICE_UUID,},
};

static esp_bt_uuid_t remote_filter_char_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {.uuid16 = REMOTE_NOTIFY_CHAR_UUID,},
};

static esp_bt_uuid_t notify_descr_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG,},
};
/******************************************************************************
 * Profile Specific Function Definitions
 *******************************************************************************/

/* APP ID */
int ble_gattc_profile_get_current_app_id(void)
{
    return g_current_app_id;
}

int ble_gattc_profile_set_current_app_id(int app_id)
{
    if( app_id <0 || app_id >= PROFILE_NUM_MAX)
    {
        ESP_LOGE(MODULE_NAME, "%d is an invalid app_id", app_id);
        return -1;
    }
    g_current_app_id = app_id;
    return 0;
}

int ble_gattc_profile_lookup_appid_by_interface(esp_gatt_if_t gattc_if)
{
    for (int idx = 0; idx < PROFILE_NUM_MAX; idx++)
    {
        if (gl_profile_tab[idx].gattc_if == gattc_if)
        {
            return idx;
        }
    }
    return -1;
}

/* GATT interface */
int ble_gattc_profile_set_gattc_interface(int profile_id, esp_gatt_if_t gattc_if)
{
    if( profile_id <0 || profile_id >= PROFILE_NUM_MAX)
    {
        ESP_LOGE(MODULE_NAME, "%d is an invalid profile_id", profile_id);
        return -1;
    }
    gl_profile_tab[profile_id].gattc_if = gattc_if;
    return 0;
}

int ble_gattc_profile_get_connect_id(esp_gatt_if_t gattc_if)
{
    int app_id = ble_gattc_profile_lookup_appid_by_interface(gattc_if);
    assert(app_id >= 0);
    return gl_profile_tab[app_id].conn_id;
}

int ble_gattc_profile_set_connection_id(esp_gatt_if_t gattc_if, uint16_t conn_id)
{
    int app_id = ble_gattc_profile_lookup_appid_by_interface(gattc_if);
    assert(app_id >= 0);
    gl_profile_tab[app_id].conn_id = conn_id;
    return 0;
}

/*
 * @brief Find and set connected peer address for profile with corresponding gattc_if and connect_id
 * 
 * @param gattc_if: GATT interface to look up
 * @param connect_id: Connection ID to look up
 * @param server_addr: Server address to set
 * @return int : 0 if success, -1 if failed
 */
int ble_gattc_profile_set_server_addr(esp_gatt_if_t gattc_if, uint16_t connect_id, esp_bd_addr_t server_addr)
{
    int app_id = ble_gattc_profile_lookup_appid_by_interface(gattc_if);
    assert(app_id >= 0);
    assert(gl_profile_tab[app_id].conn_id == connect_id);
    memcpy(gl_profile_tab[app_id].remote_bda, server_addr, sizeof(esp_bd_addr_t));
    return 0;
}

int ble_gattc_profile_get_server_addr(esp_gatt_if_t gattc_if, uint16_t connect_id, esp_bd_addr_t server_addr)
{
    int app_id = ble_gattc_profile_lookup_appid_by_interface(gattc_if);
    assert(app_id >= 0);
    assert(gl_profile_tab[app_id].conn_id == connect_id);
    memcpy(server_addr, gl_profile_tab[app_id].remote_bda, sizeof(esp_bd_addr_t));
    return 0;
}

/*
 * @brief Find and set service information (of specified UUID) for profile with corresponding gattc_if and connect_id
 * 
 * @param gattc_if: GATT interface to look up
 * @param connect_id: Connection ID to look up
 * @param service_start_handle: Service start handle to set
 * @param service_end_handle: Service end handle to set
 * @return int: 0 if success, -1 if failed
 */
int  ble_gattc_profile_set_service_info(esp_gatt_if_t gattc_if, uint16_t connect_id, uint16_t service_start_handle, uint16_t service_end_handle)
{
    int app_id = ble_gattc_profile_lookup_appid_by_interface(gattc_if);
    assert(app_id >= 0);
    assert(gl_profile_tab[app_id].conn_id == connect_id);
    gl_profile_tab[app_id].service_start_handle = service_start_handle;
    gl_profile_tab[app_id].service_end_handle = service_end_handle;
    return 0;
}

int ble_gattc_profile_set_char_handle(esp_gatt_if_t gattc_if, uint16_t connect_id, uint16_t char_handle)
{
    int app_id = ble_gattc_profile_lookup_appid_by_interface(gattc_if);
    assert(app_id >= 0);
    assert(gl_profile_tab[app_id].conn_id == connect_id);
    gl_profile_tab[app_id].char_handle = char_handle;
    return 0;
}

/*
 * @brief: Find char handle of input char with corresponding gattc_if and connect_id
 * 
 * @param gattc_if: GATT interface to look up
 * @param connect_id: Connection ID to look up
 * @param char_uuid: Characteristic UUID
 * @param[out] char_handle: Characteristic handle of input char
 * @return int: 0 if success, -1 if failed
 */
int ble_gattc_profile_find_char_handle(esp_gatt_if_t gattc_if, uint16_t connect_id, esp_bt_uuid_t char_uuid, uint16_t *char_handle)
{
    int app_id = ble_gattc_profile_lookup_appid_by_interface(gattc_if);
    assert(app_id >= 0);
    assert(char_handle != NULL);
    assert(gl_profile_tab[app_id].conn_id == connect_id);

    esp_gattc_char_elem_t result_char_element = {0};
    uint16_t number_search_char = 1;
    int status = esp_ble_gattc_get_char_by_uuid(gattc_if, 
                                                gl_profile_tab[app_id].conn_id, 
                                                gl_profile_tab[app_id].service_start_handle,
                                                gl_profile_tab[app_id].service_end_handle,
                                                char_uuid, 
                                                &result_char_element,
                                                &number_search_char);
    if (status != ESP_GATT_OK)
    {
        ESP_LOGE(MODULE_NAME, "esp_ble_gattc_get_char_by_uuid() error");
        return -1;
    }
    *char_handle = result_char_element.char_handle;
    return 0;    
}

/*
 * @brief: Find char descriptor handle of input char with corresponding gattc_if and connect_id
 * 
 * @param gattc_if: GATT interface to look up
 * @param connect_id: Connection ID to look up
 * @param char_uuid: Characteristic UUID
 * @param descr_uuid: Descriptor UUID (Ex: ESP_GATT_UUID_CHAR_CLIENT_CONFIG, ESP_GATT_UUID_ENV_SENSING_CONFIG_DESCR,...)
 * @param[out] descr_handle: characteristic descriptor handle
 * @return int: 0 if success, -1 if failed
 */
int ble_gattc_profile_find_char_descr_handle(esp_gatt_if_t gattc_if, uint16_t connect_id, esp_bt_uuid_t char_uuid, esp_bt_uuid_t descr_uuid, uint16_t *descr_handle)
{
    int app_id = ble_gattc_profile_lookup_appid_by_interface(gattc_if);
    assert(app_id >= 0);
    assert(descr_handle != NULL);
    assert(gl_profile_tab[app_id].conn_id == connect_id);
    esp_gattc_descr_elem_t notify_descr_elem_result = {0};
    uint16_t count = 1;
    int status = esp_ble_gattc_get_descr_by_uuid(gattc_if, 
                                                 gl_profile_tab[app_id].conn_id, 
                                                 gl_profile_tab[app_id].service_start_handle,
                                                 gl_profile_tab[app_id].service_end_handle,
                                                 char_uuid,
                                                 descr_uuid,
                                                &notify_descr_elem_result,
                                                &count);
    if(0 != status)
    {
        ESP_LOGE(MODULE_NAME, "esp_ble_gattc_get_descr_by_uuid() error");
        return -1;
    }
    *descr_handle = notify_descr_elem_result.handle;
    return 0;    
}

/*
 * @brief: Set characteristic descriptor handle of input char with corresponding gattc_if and connect_id
 * @param gattc_if: GATT interface to look up
 * @param connect_id: Connection ID to look up
 * @param descr_handle: characteristic descriptor handle
 * @return int: 0 if success, -1 if failed
 */
int ble_gattc_profile_set_char_descr_handle(esp_gatt_if_t gattc_if,  uint16_t connect_id, uint16_t descr_handle)
{
    int app_id = ble_gattc_profile_lookup_appid_by_interface(gattc_if);
    assert(app_id >= 0);
    assert(gl_profile_tab[app_id].conn_id == connect_id);
    gl_profile_tab[app_id].descr_handle = descr_handle;
    return 0;
}

int ble_gatt_profile_set_ccc_descriptor_value(esp_gatt_if_t gattc_if,  uint16_t connect_id, uint16_t descr_handle, gatt_char_evt_type_value value)
{
    int app_id = ble_gattc_profile_lookup_appid_by_interface(gattc_if);
    assert(app_id >= 0);
    assert(gl_profile_tab[app_id].conn_id == connect_id);
    uint16_t ccc_value = value;
    int status = esp_ble_gattc_write_char_descr( gattc_if,  gl_profile_tab[app_id].conn_id,
                                                descr_handle,
                                                sizeof(ccc_value),
                                                (uint8_t *)&ccc_value,
                                                ESP_GATT_WRITE_TYPE_RSP,
                                                ESP_GATT_AUTH_REQ_NONE);
    if(0 != status)
    {
        ESP_LOGE(MODULE_NAME, "esp_ble_gattc_write_char_descr() error");
        return -1;
    }
    return 0;
}

int on_gatt_ccc_changed_cb(esp_gatt_if_t gattc_if,  uint16_t connect_id, uint16_t descr_handle, 
                            gatt_char_evt_type_value evt_type, uint8_t* p_data, uint16_t data_len)
{
    // Get appID based on gattc_if
    int app_id = ble_gattc_profile_lookup_appid_by_interface(gattc_if);
    assert(app_id >= 0);
    if(gl_profile_tab[app_id].char_handle != descr_handle)
    {
        ESP_LOGE(MODULE_NAME, "Invalid char handle");
        return -1;
    }
    if(gl_profile_tab[app_id].char_ccc_changed_cb == NULL)
    {
        ESP_LOGE(MODULE_NAME, "Invalid Char CCC changed callback");
        return -1;  
    }
    
    ble_client_packet_t ble_client_packet = {0};
    uint16_t ble_client_packet_len = sizeof(ble_client_packet_t);
    ble_client_packet.evt_type = evt_type;
    //Get address of the device
    memcpy(ble_client_packet.ble_addr, gl_profile_tab[app_id].remote_bda, BLE_ADDR_LEN);
    ble_client_packet.p_payload = p_data;
    ble_client_packet.payload_len = data_len;
    gl_profile_tab[app_id].char_ccc_changed_cb(&ble_client_packet, &ble_client_packet_len);
    return 0;
}



void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    esp_ble_gattc_cb_param_t *p_data = (esp_ble_gattc_cb_param_t *)param;

    switch (event)
    {
        case ESP_GATTC_REG_EVT:
            // Scan parameters
            esp_ble_scan_params_t ble_scan_params = {
                .scan_type = BLE_SCAN_TYPE_PASSIVE,
                .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
                .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
                .scan_interval = BLE_GATTC_SCAN_INTERVAL,
                .scan_window = BLE_GATTC_SCAN_WINDOW,
                .scan_duplicate = BLE_SCAN_DUPLICATE_DISABLE
            };
            ESP_LOGI(MODULE_NAME, "1. Registered app profile -> setting scan params");
            esp_err_t scan_ret = esp_ble_gap_set_scan_params(&ble_scan_params);
            if (scan_ret)
            {
                ESP_LOGE(MODULE_NAME, "set scan params error, error code = %x", scan_ret);
            }
        break;

        case ESP_GATTC_CONNECT_EVT:
        {
            ESP_LOGI(MODULE_NAME, "8. Connection established");
            break;
        }

        case ESP_GATTC_OPEN_EVT:
            ESP_LOGI(MODULE_NAME, "ESP_GATTC_OPEN_EVT conn_id %d, if %d, status %d, mtu %d", p_data->open.conn_id, gattc_if, p_data->open.status, p_data->open.mtu);
            if (param->open.status != ESP_GATT_OK)
            {
                ESP_LOGE(MODULE_NAME, "open failed, status %d", p_data->open.status);
                break;
            }

            ESP_LOGI(MODULE_NAME, "8a. Opening of the connection was done successfully");
            // Store connection info into profile table
            if (0 != ble_gattc_profile_set_connection_id(gattc_if, p_data->open.conn_id))
            {
                ESP_LOGE(MODULE_NAME, "Failed to set connection id");
            }

            if (0 != ble_gattc_profile_set_server_addr(gattc_if, p_data->open.conn_id, p_data->open.remote_bda))
            {
                ESP_LOGE(MODULE_NAME, "Failed to set server address");
            }

            // Get current profile id with corresponding gattc_if
            int src_app_id = ble_gattc_profile_lookup_appid_by_interface(gattc_if);

            // Get address string
            char addr_str[18] = {0};
            // TODO - TMT: Refactor to a function that print profile info
            sprintf(addr_str,"%02X:%02X:%02X:%02X:%02X:%02X", 
                            gl_profile_tab[src_app_id].remote_bda[0], gl_profile_tab[src_app_id].remote_bda[1],
                            gl_profile_tab[src_app_id].remote_bda[2], gl_profile_tab[src_app_id].remote_bda[3],
                            gl_profile_tab[src_app_id].remote_bda[4], gl_profile_tab[src_app_id].remote_bda[5]);
            ESP_LOGI(MODULE_NAME,"Profile ID: %d, GATTC IF: %d, ConnID: %d, BDA: %s", src_app_id, gattc_if, gl_profile_tab[src_app_id].conn_id, addr_str);

            ESP_LOGI(MODULE_NAME, "8b. Send configuring MTU");
            esp_err_t mtu_ret = esp_ble_gattc_send_mtu_req(gattc_if, p_data->connect.conn_id);
            if (mtu_ret)
            {
                ESP_LOGE(MODULE_NAME, "config MTU error, error code = %x", mtu_ret);
            }

            #if (BLE_CONF_UPDATE_CONN_PARAM != 0)
            esp_ble_conn_update_params_t new_config;
            new_config.min_int = 10;  // x 1.25ms
            new_config.max_int = 100; // x 1.25ms
            new_config.latency = 0;   // number of skippable connection events
            new_config.timeout = 500; // x 6.25ms, time before peripheral will assume connection is dropped.
            // memcpy(new_config.bda,  &whitelist_addr[0], 6);
            // esp_ble_gap_update_conn_params(&new_config);
            #endif /* End of (BLE_CONF_UPDATE_CONN_PARAM != 0) */

            break;
        case ESP_GATTC_DIS_SRVC_CMPL_EVT:
            if (param->dis_srvc_cmpl.status != ESP_GATT_OK)
            {
                ESP_LOGE(MODULE_NAME, "9. Discover service failed, status: %d", param->dis_srvc_cmpl.status);
                break;
            }

            ESP_LOGI(MODULE_NAME, "9. Discover service complete conn_id: %d", param->dis_srvc_cmpl.conn_id);

            break;
        case ESP_GATTC_CFG_MTU_EVT:
            if (param->cfg_mtu.status != ESP_GATT_OK)
            {
                ESP_LOGE(MODULE_NAME, "config mtu failed, error status = %x", param->cfg_mtu.status);
            }
            ESP_LOGI(MODULE_NAME, "10. Configuring MTU was done successfully");
            ESP_LOGI(MODULE_NAME, "ESP_GATTC_CFG_MTU_EVT, Status %d, MTU %dB, conn_id %d", param->cfg_mtu.status, param->cfg_mtu.mtu, param->cfg_mtu.conn_id);

            ESP_LOGI(MODULE_NAME, "11. Start GATT service discovery");
            esp_ble_gattc_search_service(gattc_if, param->cfg_mtu.conn_id, &remote_filter_service_uuid);
            break;
        case ESP_GATTC_SEARCH_RES_EVT:
        {
            ESP_LOGI(MODULE_NAME, "12. Found service with conn_id = %d is %s service", p_data->search_res.conn_id, p_data->search_res.is_primary?"primary":"secondary");
            ESP_LOGI(MODULE_NAME, "Start handle: %d - End handle: %d - Current handle value: %d",  p_data->search_res.start_handle, p_data->search_res.end_handle, 
                                                                                            p_data->search_res.srvc_id.inst_id);
            if (p_data->search_res.srvc_id.uuid.len == ESP_UUID_LEN_16 && p_data->search_res.srvc_id.uuid.uuid.uuid16 == remote_filter_service_uuid.uuid.uuid16)
            {
                ESP_LOGI(MODULE_NAME, "12a. Found service 0x%X on server with connection id: %d ", remote_filter_service_uuid.uuid.uuid16, p_data->search_res.conn_id);
                if (ble_gattc_profile_set_service_info(gattc_if, p_data->search_res.conn_id , p_data->search_res.start_handle, p_data->search_res.end_handle) != 0)
                {
                    ESP_LOGE(MODULE_NAME, "Failed to set service info");
                }
            }
            break;
        }
        case ESP_GATTC_SEARCH_CMPL_EVT:
            if (p_data->search_cmpl.status != ESP_GATT_OK)
            {
                ESP_LOGE(MODULE_NAME, "13. ESP_GATTC_SEARCH_CMPL_EVT failed, error status = %x", p_data->search_cmpl.status);
                break;
            }
            ESP_LOGI(MODULE_NAME, "13a. ESP_GATTC_SEARCH_CMPL_EVT - service discovery sucessfully");
            ESP_LOGI(MODULE_NAME, "13b. Start finding char handle for char with UUID 0x%X in conn_id %x", remote_filter_char_uuid.uuid.uuid16, p_data->search_cmpl.conn_id);
            uint16_t char_handle = 0;
            int status =  ble_gattc_profile_find_char_handle(gattc_if, p_data->search_cmpl.conn_id, remote_filter_char_uuid, &char_handle);
            if (status != 0)
            {
                ESP_LOGE(MODULE_NAME, "ble_gattc_profile_find_char_handle() error");
                break;
            }
            ESP_LOGI(MODULE_NAME, "13b. Found char handle %d", char_handle);
            // Update char handle in profile table
            if (0 != ble_gattc_profile_set_char_handle(gattc_if, p_data->search_cmpl.conn_id, char_handle))
            {
                ESP_LOGE(MODULE_NAME, "Failed to set char handle");
                break;
            }
            ESP_LOGI(MODULE_NAME, "13c. Try to register for notification");
            
            // Get address corresponding to gattc_if and conn_id
            esp_bd_addr_t target_addr = {0};
            if(ble_gattc_profile_get_server_addr(gattc_if, p_data->search_cmpl.conn_id, target_addr) != 0 )
            {
                ESP_LOGE(MODULE_NAME, "ble_gattc_profile_get_server_addr() failed");
            }
            if (0 != esp_ble_gattc_register_for_notify(gattc_if, target_addr, char_handle))
            {
                ESP_LOGE(MODULE_NAME, "Failed to register for notify with conn_id %x", p_data->search_cmpl.conn_id);
            }
            break;

        case ESP_GATTC_REG_FOR_NOTIFY_EVT:
        {
            if (p_data->reg_for_notify.status != ESP_GATT_OK)
            {
                ESP_LOGE(MODULE_NAME, "ESP_GATTC_REG_FOR_NOTIFY_EVT failed: error status = %d", p_data->reg_for_notify.status);
                break;
            }
            ESP_LOGI(MODULE_NAME, "14. Register for notify with char handle %d success, start writing to CCC descriptor", p_data->reg_for_notify.handle);

            // Get connection ID base on gattc_if
            int connection_id = ble_gattc_profile_get_connect_id(gattc_if);
            if( connection_id < 0)
            {
                ESP_LOGE(MODULE_NAME, "ble_gattc_profile_get_connect_id() failed with err %d", connection_id);
                break;
            }
            uint16_t char_description_handle = 0;
            int status = ble_gattc_profile_find_char_descr_handle(gattc_if, connection_id,
                                                                 remote_filter_char_uuid, 
                                                                 notify_descr_uuid,
                                                                 &char_description_handle);
            if( status != 0)
            {
                ESP_LOGE(MODULE_NAME, "ble_gattc_profile_find_char_descr_handle() failed with err %d", status);
                break;
            }
            ESP_LOGI(MODULE_NAME, "14a. Found descriptor handle %d for char UUID 0x%X", char_description_handle, remote_filter_char_uuid.uuid.uuid16);
            // Update descriptor handle in profile table
            status = ble_gattc_profile_set_char_descr_handle(gattc_if, connection_id, char_description_handle);
            if( status != 0)
            {
                ESP_LOGE(MODULE_NAME, "ble_gattc_profile_set_char_descr_handle() failed with err %d", status);
                break;
            }
            // Enable notification
            status = ble_gatt_profile_set_ccc_descriptor_value(gattc_if, connection_id, char_description_handle, BT_GATT_CCC_INDICATE);
            if( status != 0)
            {
                ESP_LOGE(MODULE_NAME, "ble_gatt_profile_set_ccc_descriptor_value() failed with err %d", status);
                break;
            }
            
            break;
        }

        case ESP_GATTC_WRITE_DESCR_EVT:
            if (p_data->write.status != ESP_GATT_OK)
            {
                ESP_LOGE(MODULE_NAME, "ESP_GATTC_WRITE_DESCR_EVT for handle %d failed, error status = %x", p_data->write.handle, p_data->write.status);
                break;
            }
            ESP_LOGI(MODULE_NAME, "ESP_GATTC_WRITE_DESCR_EVT for handle %d success", p_data->write.handle);
            break;
        
        case ESP_GATTC_WRITE_CHAR_EVT:
            if (p_data->write.status != ESP_GATT_OK)
            {
                ESP_LOGE(MODULE_NAME, "ESP_GATTC_WRITE_CHAR_EVT for handle %d failed, error status = %x", p_data->write.handle, p_data->write.status);
                break;
            }
            ESP_LOGI(MODULE_NAME, "ESP_GATTC_WRITE_CHAR_EVT for handle %d success", p_data->write.handle);
            break;


        case ESP_GATTC_NOTIFY_EVT:
            ESP_LOGI(MODULE_NAME, "ESP_GATTC_NOTIFY_EVT, connection ID %d, char handle %d, evt: %s", 
                                    p_data->notify.conn_id, p_data->notify.handle, p_data->notify.is_notify?"notify":"indicate");
            // esp_log_buffer_hex(MODULE_NAME, p_data->notify.value, p_data->notify.value_len);
            on_gatt_ccc_changed_cb(gattc_if, p_data->notify.conn_id, p_data->notify.handle,
                                    p_data->notify.is_notify? BT_GATT_CCC_NOTIFY : BT_GATT_CCC_INDICATE,
                                    p_data->notify.value, p_data->notify.value_len);
#if 0

            // Get appID based on gattc_if
            int app_id = ble_gattc_profile_lookup_appid_by_interface(gattc_if);
            assert(app_id >= 0);
            if(gl_profile_tab[app_id].char_handle == p_data->notify.handle)
            {
                if(gl_profile_tab[app_id].char_ccc_changed_cb != NULL)
                {
                    uint16_t ccc_value = p_data->notify.is_notify? BT_GATT_CCC_NOTIFY : BT_GATT_CCC_INDICATE;
                    gl_profile_tab[app_id].char_ccc_changed_cb(gattc_if, ccc_value, p_data->notify.value, &p_data->notify.value_len);
                }
            }
#endif /* End of 0 */
            break;
        
        case ESP_GATTC_SRVC_CHG_EVT:
        {
            esp_bd_addr_t bda;
            memcpy(bda, p_data->srvc_chg.remote_bda, sizeof(esp_bd_addr_t));
            ESP_LOGI(MODULE_NAME, "ESP_GATTC_SRVC_CHG_EVT, bd_addr:");
            esp_log_buffer_hex(MODULE_NAME, bda, sizeof(esp_bd_addr_t));
            break;
        }

        case ESP_GATTC_DISCONNECT_EVT:
            ESP_LOGI(MODULE_NAME, "ESP_GATTC_DISCONNECT_EVT, reason = %d", p_data->disconnect.reason);
            if(BLE_CONF_AUTO_RESCAN != 0)
            {
                ble_gatt_client_start_scan(BLE_GATTC_SCAN_DURATION);
            }
            break;
        default:
            break;
    }
}

// GAP callback function - search BT connections
void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event)
    {
        case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
            ESP_LOGI(MODULE_NAME, "2. Finish set scan parameters, start scan");
            break;

        case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
            // scan start complete event to indicate scan start successfully or failed
            if (param->scan_start_cmpl.status != ESP_BT_STATUS_SUCCESS)
            {
                ESP_LOGE(MODULE_NAME, "3. Scan started failed, error status = %x", param->scan_start_cmpl.status);
            }
            else
            {
                ESP_LOGI(MODULE_NAME, "3. Scan started successfully");
            }
            break;

        case ESP_GAP_BLE_SCAN_RESULT_EVT: 
        {
            esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
            switch (scan_result->scan_rst.search_evt) 
            {
                case ESP_GAP_SEARCH_INQ_RES_EVT:
                    ESP_LOGD(MODULE_NAME, "4. Found a device");
                    uint8_t adv_payload_len;
                    uint8_t* p_adv_payload = esp_ble_resolve_adv_data(scan_result->scan_rst.ble_adv, ESP_BLE_AD_TYPE_NAME_CMPL, &adv_payload_len);
                    // if ( (p_adv_payload != NULL) && (NULL != strstr((const char*)p_adv_payload, "IMU")) )
                    if ( (p_adv_payload != NULL) && (NULL != strstr((const char*)p_adv_payload, "XIAO")) )
                    {   //5. Find the interested device to connect if required (based on adv name for example)
                        
                        //6. Stop the scan process
                        if (esp_ble_gap_stop_scanning() != ESP_OK)
                        {
                            ESP_LOGE(MODULE_NAME, "Failed to stop scanning");
                        }
                        ESP_LOGI(MODULE_NAME, "6. Sucessfully stop scanning to connect to the device");
                        ESP_LOGI(MODULE_NAME, "7. Tries to open a connection to the device");
                        int current_profile_id = ble_gattc_profile_get_current_app_id();
                        esp_ble_gattc_open(gl_profile_tab[current_profile_id].gattc_if, scan_result->scan_rst.bda, scan_result->scan_rst.ble_addr_type, true);
                        #if 0
                        if(ble_client_callback.ble_found_adv_packet_cb != NULL)
                        {
                            ble_client_packet_t ble_client_packet;
                            memcpy(ble_client_packet.ble_addr, scan_result->scan_rst.bda, BLE_ADDR_LEN);
                            ble_client_packet.addr_type = scan_result->scan_rst.ble_addr_type;
                            ble_client_packet.rssi = scan_result->scan_rst.rssi;
                            ble_client_packet.p_payload = p_adv_payload;
                            ble_client_packet.payload_len = adv_payload_len;
                            ble_client_callback.ble_found_adv_packet_cb(&ble_client_packet, NULL);
                        }
                        ESP_LOGW(MODULE_NAME, "======= New device is found =======");
                        esp_log_buffer_hex(MODULE_NAME, scan_result->scan_rst.bda, 6);
                        esp_log_buffer_hex(MODULE_NAME, p_adv_payload, adv_payload_len);
                        ESP_LOGW(MODULE_NAME, "=================================== \r\n");
                        #endif

                    }
                    break;


                case ESP_GAP_SEARCH_INQ_CMPL_EVT:
                    if(BLE_CONF_AUTO_RESCAN != 0)
                    {
                        ESP_LOGI(MODULE_NAME, "4.1. Scan complete, restart scanning");
                        ble_gatt_client_start_scan(BLE_GATTC_SCAN_DURATION);
                    }
                    break;

                default:
                    break;
            }
            break;
        }
        case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
            if (param->scan_stop_cmpl.status != ESP_BT_STATUS_SUCCESS)
            {
                ESP_LOGE(MODULE_NAME, "scan stop failed, error status = %x", param->scan_stop_cmpl.status);
                break;
            }
            ESP_LOGI(MODULE_NAME, "stop scan successfully");
            break;

        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
            if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS)
            {
                ESP_LOGE(MODULE_NAME, "adv stop failed, error status = %x", param->adv_stop_cmpl.status);
                break;
            }
            ESP_LOGI(MODULE_NAME, "stop adv successfully");
            break;

        case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
            ESP_LOGI(MODULE_NAME, "update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
                    param->update_conn_params.status,
                    param->update_conn_params.min_int,
                    param->update_conn_params.max_int,
                    param->update_conn_params.conn_int,
                    param->update_conn_params.latency,
                    param->update_conn_params.timeout);
            break;
            
        default:
            break;
    }
}

// GATTC callback function for gl_profile_tab structure initialization
// esp_gattc_cb_event_t - GATT Client callback function events
// esp_gatt_if_t - GATT interface type
// esp_ble_gattc_cb_param_t - GATT client callback parameters union
void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    /* If event is register event, store the gattc_if for each profile */
    if (event == ESP_GATTC_REG_EVT)
    {
        // TODO - TMT: Update while-list
        // ESP_LOGI("BLE_CUSTOM", "UPDATING WHITELIST");
        // esp_ble_gap_update_whitelist(true, whitelist_addr, BLE_WL_ADDR_TYPE_RANDOM);

        if (param->reg.status == ESP_GATT_OK)
        {
            if (0 != ble_gattc_profile_set_gattc_interface(param->reg.app_id, gattc_if))
            {
                ESP_LOGE(MODULE_NAME, "Failed to set gattc interface");
                return;
            }
        }
        else
        {
            ESP_LOGI(MODULE_NAME, "reg app failed, app_id %d, status %d", param->reg.app_id, param->reg.status);
            return;
        }
    }
    /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
    if( gattc_if == ESP_GATT_IF_NONE)
    {
        for (int idx = 0; idx < PROFILE_NUM_MAX; idx++)
        {
            if (gl_profile_tab[idx].gattc_cb)
            {
                gl_profile_tab[idx].gattc_cb(event, gattc_if, param);
            }
        }
    }
    else
    {
        /* If the gattc_if equal to profile A, call profile A cb handler with gattc_if as 2nd param */
        do
        {
            for (int idx = 0; idx < PROFILE_NUM_MAX; idx++)
            {
                if (gattc_if == gl_profile_tab[idx].gattc_if)
                {
                    if (gl_profile_tab[idx].gattc_cb)
                    {
                        gl_profile_tab[idx].gattc_cb(event, gattc_if, param);
                    }
                }
            }
        } while (0);
    }
}

int ble_gatt_client_init(ble_client_callback_t* ble_app_cb)
{
    int ret;
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    if(ble_app_cb != NULL)
    {
        ble_client_callback = *ble_app_cb;
    }
    // Initialize the profile table with NULL values
    for(uint8_t idx=0; idx < PROFILE_NUM_MAX; idx++)
    {
        gl_profile_tab[idx].gattc_if = ESP_GATT_IF_NONE;
        gl_profile_tab[idx].service_start_handle = BLE_GATTC_ATT_INVALID_HANDLE;
        gl_profile_tab[idx].service_end_handle = BLE_GATTC_ATT_INVALID_HANDLE;
        gl_profile_tab[idx].char_handle = BLE_GATTC_ATT_INVALID_HANDLE;
        gl_profile_tab[idx].descr_handle = BLE_GATTC_ATT_INVALID_HANDLE;
        gl_profile_tab[idx].conn_id = 0;
        gl_profile_tab[idx].char_ccc_changed_cb = ble_client_callback.ble_gatt_ccc_cb[idx];
        memset(gl_profile_tab[idx].remote_bda, 0, sizeof(esp_bd_addr_t));

    }


    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret)
    {
        ESP_LOGE(MODULE_NAME, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
        return -1;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret)
    {
        ESP_LOGE(MODULE_NAME, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return -1;
    }

    ret = esp_bluedroid_init();
    if (ret)
    {
        ESP_LOGE(MODULE_NAME, "%s init bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return -1;
    }

    ret = esp_bluedroid_enable();
    if (ret)
    {
        ESP_LOGE(MODULE_NAME, "%s enable bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return -1;
    }

    // register the  callback function to the gap module
    ret = esp_ble_gap_register_callback(esp_gap_cb);
    if (ret)
    {
        ESP_LOGE(MODULE_NAME, "%s gap register failed, error code = %x\n", __func__, ret);
        return -1;
    }

    // register the callback function to the gattc module
    ret = esp_ble_gattc_register_callback(esp_gattc_cb);
    if (ret)
    {
        ESP_LOGE(MODULE_NAME, "%s gattc register failed, error code = %x\n", __func__, ret);
        return -1;
    }

    ret = esp_ble_gattc_app_register(PROFILE_A_APP_ID);
    if (ret)
    {
        ESP_LOGE(MODULE_NAME, "%s gattc app register failed, error code = %x\n", __func__, ret);
        return -1;
    }

    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(BLE_MTU_CONFIG_SIZE);
    if (local_mtu_ret)
    {
        ESP_LOGE(MODULE_NAME, "set local  MTU failed, error code = %x", local_mtu_ret);
        return -1;
    }

    ESP_LOGI(MODULE_NAME, "BLE GATTC initialized");
    esp_log_level_set(MODULE_NAME, MODULE_DEFAULT_LOG_LEVEL);

    return 0;
}

int ble_gatt_client_start_scan(uint32_t scan_duration_s)
{
    int status;
    status = esp_ble_gap_start_scanning(scan_duration_s);
    if(status != 0)
    {
        ESP_LOGE(MODULE_NAME, "Failed to start scan with err %d", status);
    }
    return status;
}

int ble_gatt_client_stop_scan()
{
    int status;
    status = esp_ble_gap_stop_scanning();
    if(status != 0)
    {
        ESP_LOGE(MODULE_NAME, "Failed to stop scan with err %d", status);
    }
    return status;
}