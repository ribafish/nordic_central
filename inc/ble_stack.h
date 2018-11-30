/*
 * ble_stack.h
 *
 *  Created on: 6 May 2018
 *      Author: gaspe
 */

#ifndef BLE_STACK_H_
#define BLE_STACK_H_

#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include "ble_advertising.h"
#include "ble_gap.h"
#include "nrf_ble_gatt.h"

#define DEVICE_NAME                     "TestCentral"                           /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME               "GK Solutions"                       /**< Manufacturer. Will be passed to Device Information Service. */
#define FOOLOGRAPHY_COMPANY_ID			0x03EF									/**< BLE company identifier */


#define APP_FEATURE_NOT_SUPPORTED       BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2    /**< Reply when unsupported features are requested. */

/** @brief The maximum number of peripheral and central links combined. */
#define NRF_BLE_LINK_COUNT              (NRF_BLE_PERIPHERAL_LINK_COUNT + NRF_BLE_CENTRAL_LINK_COUNT)

#define CONN_INTERVAL_MIN               MSEC_TO_UNITS(7.5, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (7.5ms seconds). */
#define CONN_INTERVAL_MAX               MSEC_TO_UNITS(100, UNIT_1_25_MS)        /**< Maximum acceptable connection interval (0.4 second). */
#define SLAVE_LATENCY                   0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(8100, UNIT_10_MS)         /**< Connection supervisory timeout (4 seconds). */

#define SCAN_INTERVAL					160											/**< Scanning interval, determines scan interval in units of 0.625 millisecond. */
#define SCAN_TIMEOUT					0											/**< The scan timeout in untis of seconds (0 means no timeout).*/
#define SCAN_WINDOW						80											/**< Scanning window, determines scan window in units of 0.625 millisecond. */

#define APP_CONN_CFG_TAG				1											/**< A tag that refers to the BLE stack configuration we set with @ref sd_ble_cfg_set. Default tag is @ref BLE_CONN_CFG_TAG_DEFAULT. */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(1000)                   /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                  /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    10                                       /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                  1                                       /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                       /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                  0                                       /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS              0                                       /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                    /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                       /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                       /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                      /**< Maximum encryption key size. */

#define CUSTOM_UUID_COUNT				1
// Advertising intervals should be within the range of 20 ms and 10.24 s
#define APP_FAST_ADV_INTERVAL				300										/**< The advertising interval (in units of 0.625 ms. This value corresponds to 187.5 ms). */
#define APP_FAST_ADV_TIMEOUT_IN_SECONDS		120										/**< The advertising timeout in units of seconds. */

#define APP_SLOW_ADV_INTERVAL				2400									/**< The advertising interval (in units of 0.625 ms. This value corresponds to 1.5 seconds). */
#define APP_SLOW_ADV_TIMEOUT_IN_SECONDS		BLE_GAP_ADV_TIMEOUT_GENERAL_UNLIMITED	/**< The advertising timeout in units of seconds. */

extern nrf_ble_gatt_t		m_gatt;

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
void ble_stack_init(void);

/**@brief Function for initializing the GATT module.
 */
void gatt_init(void);

/**
 * @brief Database discovery initialization.
 */
void db_discovery_init(void);

/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
void gap_params_init(void);

/**@brief Function for initializing the Connection Parameters module.
 */
void conn_params_init(void);

/**@brief Function for initializing services that will be used by the application.
 */
void services_init(void);

/**@brief Function for the Peer Manager initialization.
 */
void peer_manager_init(void);

/**@brief Function for initializing the Advertising functionality.
 */
void advertising_init();

/**@brief Function for starting advertising.
 */
void advertising_start(ble_adv_mode_t mode, bool erase_bonds);

/**@brief Function for initiating scanning.
 */
void scan_start(void);

/**@brief Function for stopping scanning.
 */
void scan_stop(void);

void advertising_name_update(char * name);

void ble_set_max_data_length(uint16_t new_att_mtu);

uint8_t ble_get_max_data_length();


uint32_t ble_stack_set_conn_param(ble_gap_conn_params_t *p_conn_params);

uint32_t ble_stack_set_phy(uint8_t phy);

uint32_t ble_stack_set_preferred_phy(uint32_t phy);

#endif /* BLE_STACK_H_ */
