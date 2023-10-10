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


/******************************************************************************
* Configuration Constants
*******************************************************************************/

// BLE configs
#define BLE_GATTC_SCAN_DURATION                 ((uint32_t)0xFFFFFFFF) // In seconds
#define BLE_GATTC_SCAN_INTERVAL                 ((uint16_t)0x0010) // N * 0.625 msec ~ 10ms
#define BLE_GATTC_SCAN_WINDOW                   ((uint16_t)0x0010) // N * 0.625 msec ~ 10ms
#define BLE_MTU_CONFIG_SIZE                     (200)

#define BLE_CONF_AUTO_RESCAN                    (1) // 1-> automatically rescan after scan duration timeout or disconnect

/******************************************************************************
* Macros
*******************************************************************************/


/******************************************************************************
* Typedefs
*******************************************************************************/
typedef struct
{
    void (*ble_connected_cb)(void);
    void (*ble_disconnected_cb)(void);
    void (*ble_adv_started_cb)(void);
    void (*ble_adv_stopped_cb)(void);
}ble_server_callback_t;


typedef struct
{
    //When found an advertising packet with appropriate filters, this callback will be called
    void (*ble_found_adv_packet_cb)(uint8_t* p_data, uint16_t data_len); 
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
int ble_gatt_client_init();
int ble_gatt_client_start_scan(uint32_t scan_duration_s);
int ble_gatt_client_stop_scan();
#ifdef __cplusplus
} // extern "C"
#endif

#endif // _BLE_GATT_CLIENT_BLE_GATTC_H_

/*** End of File **************************************************************/