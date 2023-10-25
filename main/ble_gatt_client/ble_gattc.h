/****************************************************************************
* Title                 :   ESP-IDF BLE GATT Client header file
* Filename              :   ble_gattc.h
* Author                :   ItachiVN
* Origin Date           :   2023/10/09
* Version               :   v0.0.0
* Compiler              :   ESP-IDF V5.0.2
* Target                :   ESP32 
* Notes                 :   None
*****************************************************************************/

/*************** INTERFACE CHANGE LIST **************************************
*
*    Date    	Software Version    Initials   	Description
*  2023/10/09    v0.0.0         	ItachiVN      Interface Created.
*
*****************************************************************************/

/** \file ble_gattc.h
 *  \brief This module contains .
 *
 *  This is the header file for 
 */
#ifndef _BLE_GATT_CLIENT_BLE_GATTC_H_
#define _BLE_GATT_CLIENT_BLE_GATTC_H_

/******************************************************************************
* Includes
*******************************************************************************/
#include <stdint.h>

/******************************************************************************
* Preprocessor Constants
*******************************************************************************/
#define BLE_GATTC_ATT_INVALID_HANDLE            (0xFFFF)
#define BLE_GATTC_ATT_INVALID_CONN_ID           (0xFFFF)
#define BLE_GATTC_ATT_INVALID_ADDR_TYPE         (0xFF)
/******************************************************************************
* Configuration Constants
*******************************************************************************/

// BLE configs
#define BLE_GATTC_SCAN_DURATION                 ((uint32_t)0xFFFFFFFF) // In seconds
#define BLE_GATTC_SCAN_INTERVAL                 ((uint16_t)0x0010) // N * 0.625 msec ~ 10ms
#define BLE_GATTC_SCAN_WINDOW                   ((uint16_t)0x0010) // N * 0.625 msec ~ 10ms
#define BLE_MTU_CONFIG_SIZE                     (200)
#define BLE_ADDR_LEN                            (6)

#define BLE_CONF_AUTO_RESCAN                    (0) // 1-> automatically rescan after scan duration timeout or disconnect

/******************************************************************************
* Macros
*******************************************************************************/


/******************************************************************************
* Typedefs
*******************************************************************************/
typedef enum
{
    PROFILE_A_APP_ID = 0,
    PROFILE_B_APP_ID,
    PROFILE_C_APP_ID,
    PROFILE_D_APP_ID,
    PROFILE_NUM_MAX
}app_profile_id;

typedef enum
{
    BT_GATT_CCC_DISABLE = 0,
    BT_GATT_CCC_NOTIFY = 1,
    BT_GATT_CCC_INDICATE = 2,
    BT_GATT_ATT_READ = 3,
}gatt_char_evt_type_value;

typedef struct
{
    uint8_t evt_type;
    uint8_t ble_addr[BLE_ADDR_LEN];
    uint8_t* p_payload;
    uint16_t payload_len;
}ble_client_packet_t;

typedef struct
{
    uint8_t ble_addr[BLE_ADDR_LEN];
    uint8_t addr_type;
    int rssi;
    uint8_t* p_payload;
    uint16_t payload_len;
}ble_client_adv_packet_t;


typedef struct
{
    void (*ble_connected_cb)(void);
    void (*ble_disconnected_cb)(void);
}ble_gap_callback_t;

/* 
 * @brief: Client Characteristic Configuration callback
 * @param [in] p_data: pointer to data from GATT client
 * @param [in] p_len: pointer that contain receive length from GATT client
 * @return: (reserved)
*/
typedef void (*ble_gatt_ccc_cb)(void* p_data, void* p_len); /* BLE GATT callbacks custom service*/

typedef struct
{
    ble_gap_callback_t ble_gap_cb;
    //When found an advertising packet with appropriate filters, this callback will be called
    void (*ble_found_adv_packet_cb)(void* p_data, void* p_data_len);
    ble_gatt_ccc_cb ble_gatt_ccc_cb[PROFILE_NUM_MAX];
}ble_client_callback_t;

/******************************************************************************
* Variables
*******************************************************************************/


/******************************************************************************
* Function Prototypes
*******************************************************************************/

#ifdef __cplusplus
extern "C"{
#endif
int ble_gatt_client_init(ble_client_callback_t* ble_app_cb);
int ble_gatt_client_start_scan(uint32_t scan_duration_s);
int ble_gatt_client_stop_scan();
#ifdef __cplusplus
} // extern "C"
#endif

#endif // _BLE_GATT_CLIENT_BLE_GATTC_H_

/*** End of File **************************************************************/