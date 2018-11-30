/*
 * central_ble.h
 *
 *  Created on: Jul 26, 2018
 *      Author: gksolutions
 */

#ifndef CENTRAL_BLE_H_
#define CENTRAL_BLE_H_

#include "ble.h"
#include "sdk_config.h"
#include "ble_db_discovery.h"

#define TEST_CHAR_HANDLE_CONTROL_IDX  	0
#define TEST_CHAR_HANDLE_DATA_IDX 		1

#define OPCODE_LENGTH				1
#define HANDLE_LENGTH				2

// Maximum length of data (in bytes) that can be transmitted to the peer by the thumbnail service
#if defined(NRF_BLE_GATT_MAX_MTU_SIZE) && (NRF_BLE_GATT_MAX_MTU_SIZE != 0)
    #define BLE_THUMBNAIL_TX_CHAR_MAX_LEN (NRF_BLE_GATT_MAX_MTU_SIZE - OPCODE_LENGTH - HANDLE_LENGTH)
#else
    #define BLE_THUMBNAIL_TX_CHAR_MAX_LEN (BLE_GATT_ATT_MTU_DEFAULT - OPCODE_LENGTH - HANDLE_LENGTH)
    #warning NRF_SDH_BLE_GATT_MAX_MTU_SIZE is not defined.
#endif

void central_ble_init();
void central_on_ble_evt(ble_evt_t * p_ble_evt);

void central_ble_on_db_disc_evt(const ble_db_discovery_evt_t * p_evt);

uint32_t central_ble_set_conn_param(ble_gap_conn_params_t const *p_conn_params);

uint32_t write_to_test_char(uint8_t char_handle_idx, uint8_t len, uint8_t * data);
uint32_t write_no_response_to_test_char(uint8_t char_handle_idx, uint8_t len, uint8_t * data);
uint32_t read_test_char(uint8_t char_handle_idx);


uint8_t get_test_handle_index(uint8_t handle);
uint16_t get_test_handle_uuid(uint8_t handle);

#endif /* CENTRAL_BLE_H_ */
