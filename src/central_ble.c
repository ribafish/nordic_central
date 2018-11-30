/*
 * central_ble.c
 *
 *  Created on: Jul 26, 2018
 *      Author: gksolutions
 */

#include "central_ble.h"

#include <stdint.h>
#include "ble_abstraction.h"
#include "nrf_log.h"
#include "app_error.h"
#include "ble.h"
#include "ble_srv_common.h"
#include "ble_dis.h"	// Device information service
#include "ble_uuid.h"
#include "debug.h"
#include "central_core.h"
#include "ble_stack.h"


#define DEBUG	1
#define debug_line(...)  do { if (DEBUG>0) { debug_line_global(__VA_ARGS__); debug_global("\n"); }} while (0)
#define debug_error(...)  do { if (DEBUG>0) { debug_errorline_global(__VA_ARGS__); debug_global("\n"); }} while (0)
#define debugL2(...)  do { if (DEBUG>1) { debug_line_global(__VA_ARGS__); debug_global("\n"); }} while (0)
#define debug_data(...)  do { if (DEBUG>2) { debug_global(__VA_ARGS__); }} while (0)

static ble_service_t				test_service;
static ble_gatts_char_handles_t		test_char[BLE_TEST_SERVICE_CHARA_NUM];

static ble_uuid_t test_service_uuid;
static ble_uuid_t test_c_control_uuid;
static ble_uuid_t test_c_data_uuid;

static uint8_t cccd_msg[BLE_CCCD_VALUE_LEN];

uint8_t request_data[255];


// Private function forward declarations
static void update_connection_handles(uint16_t conn_handle);
static void enable_notifications(bool enable, uint16_t conn_handle, uint16_t handle_cccd);
static void on_hvx(const ble_evt_t * p_ble_evt);


// Function bodies

void central_ble_init() {
	ble_uuid128_t   base_uuid = BLE_UUID_PERIPHERAL_BASE;
	ret_code_t		err_code;

	err_code = sd_ble_uuid_vs_add(&base_uuid, &test_service_uuid.type);
	if (err_code != NRF_SUCCESS)
	{
		debug_error("Failed to add VS UUID (0x%02X)", err_code);
		return;
	}

	err_code = sd_ble_uuid_vs_add(&base_uuid, &test_c_control_uuid.type);
	if (err_code != NRF_SUCCESS)
	{
		debug_error("Failed to add VS UUID (0x%02X)", err_code);
		return;
	}

	err_code = sd_ble_uuid_vs_add(&base_uuid, &test_c_data_uuid.type);
	if (err_code != NRF_SUCCESS)
	{
		debug_error("Failed to add VS UUID (0x%02X)", err_code);
		return;
	}


	test_service_uuid.uuid = BLE_UUID_SERVICE_TEST;

	memset(&test_service, 0, sizeof(test_service));
	test_service.conn_handle	= BLE_CONN_HANDLE_INVALID;
	test_service.char_handles	= test_char;

	err_code = ble_db_discovery_evt_register(&test_service_uuid);

	if (err_code != NRF_SUCCESS) {
		debug_error("Failed to register UUID in discovery module (0x%02X)", err_code);
	}

}

// Event handlers -----------------------------------------------------------------------------

static void on_connect(ble_evt_t * p_ble_evt) {
	update_connection_handles(p_ble_evt->evt.gap_evt.conn_handle);

}

static void on_disconnect(ble_evt_t * p_ble_evt) {
	update_connection_handles(BLE_CONN_HANDLE_INVALID);
}

void central_on_ble_evt(ble_evt_t * p_ble_evt) {
	central_core_event_t evt;
	switch(p_ble_evt->header.evt_id) {
		case BLE_GAP_EVT_CONNECTED:
			on_connect(p_ble_evt);
			evt.type = CENTRAL_CORE_EVT_CONNECTED;
			central_core_event_handler(evt);
			break;
		case BLE_GAP_EVT_DISCONNECTED:
			on_disconnect(p_ble_evt);
			evt.type = CENTRAL_CORE_EVT_DISCONNECTED;
			central_core_event_handler(evt);
			break;
		case BLE_GAP_EVT_PHY_UPDATE:
			evt.type = CENTRAL_CORE_EVT_PHY_UPDATED;
			memcpy(&evt.phy_update, &p_ble_evt->evt.gap_evt.params.phy_update, sizeof &evt.phy_update);
			central_core_event_handler(evt);
			break;
		case BLE_GAP_EVT_CONN_PARAM_UPDATE:
			evt.type = CENTRAL_CORE_EVT_CONN_PARAM_UPDATED;
			memcpy(&evt.conn_params, &p_ble_evt->evt.gap_evt.params.conn_param_update.conn_params, sizeof &evt.conn_params);
			central_core_event_handler(evt);
			break;

			// GATT Client stuff
		case BLE_GATTC_EVT_HVX:
			evt.type = CENTRAL_CORE_EVT_NOTIFY_RECEIVED;
			evt.re_wr_nt.data = p_ble_evt->evt.gattc_evt.params.hvx.data;
			evt.re_wr_nt.datalen = p_ble_evt->evt.gattc_evt.params.hvx.len;
			evt.re_wr_nt.char_handle_id = get_test_handle_index(p_ble_evt->evt.gattc_evt.params.hvx.handle);
			evt.re_wr_nt.char_uuid = get_test_handle_uuid(p_ble_evt->evt.gattc_evt.params.hvx.handle);
			if (evt.re_wr_nt.char_handle_id != 0xFF) {
				central_core_event_handler(evt);
			} else {
		    	debug_error("Unknown handle for notification! 0x%04x", p_ble_evt->evt.gattc_evt.params.hvx.handle);
			}
			on_hvx(p_ble_evt);
			break;
		case BLE_GATTC_EVT_WRITE_RSP:	// with response
			evt.type = CENTRAL_CORE_EVT_WRITE_DONE;
			evt.re_wr_nt.data = p_ble_evt->evt.gattc_evt.params.write_rsp.data;
			evt.re_wr_nt.datalen = p_ble_evt->evt.gattc_evt.params.write_rsp.len;
			evt.re_wr_nt.char_handle_id = get_test_handle_index(p_ble_evt->evt.gattc_evt.params.write_rsp.handle);
			evt.re_wr_nt.char_uuid = get_test_handle_uuid(p_ble_evt->evt.gattc_evt.params.write_rsp.handle);
			if (evt.re_wr_nt.char_handle_id != 0xFF) {
				central_core_event_handler(evt);
			} else {
		    	debug_error("Unknown handle for write! 0x%04x", p_ble_evt->evt.gattc_evt.params.write_rsp.handle);
			}
			break;

		case BLE_GATTC_EVT_WRITE_CMD_TX_COMPLETE:  // without response
			evt.type = CENTRAL_CORE_EVT_WRITE_NO_RSP_DONE;
			evt.wr_no_rsp_count = p_ble_evt->evt.gattc_evt.params.write_cmd_tx_complete.count;
			central_core_event_handler(evt);
//			debug_line("Write no resp done. handle %04x", p_ble_evt->evt.gattc_evt.params.write_rsp.handle);
			break;
        case BLE_GATTC_EVT_READ_RSP:
			evt.type = CENTRAL_CORE_EVT_READ_DONE;
			evt.re_wr_nt.data = p_ble_evt->evt.gattc_evt.params.read_rsp.data;
			evt.re_wr_nt.datalen = p_ble_evt->evt.gattc_evt.params.read_rsp.len;
			evt.re_wr_nt.char_handle_id = get_test_handle_index(p_ble_evt->evt.gattc_evt.params.read_rsp.handle);
			evt.re_wr_nt.char_uuid = get_test_handle_uuid(p_ble_evt->evt.gattc_evt.params.read_rsp.handle);
			if (evt.re_wr_nt.char_handle_id != 0xFF) {
				central_core_event_handler(evt);
			} else {
		    	debug_error("Unknown handle for read! 0x%04x", p_ble_evt->evt.gattc_evt.params.read_rsp.handle);
			}
//        	debug_line("Read RSP len %d: %s", p_ble_evt->evt.gattc_evt.params.read_rsp.len, p_ble_evt->evt.gattc_evt.params.read_rsp.data);
//        	for (uint8_t i =0; i<p_ble_evt->evt.gattc_evt.params.read_rsp.len; i++) {
//        		debug_data("%02x ", p_ble_evt->evt.gattc_evt.params.read_rsp.data[i]);
//        	}
//    		debug_data("\n");
            break;
	}
}

void central_ble_on_db_disc_evt(const ble_db_discovery_evt_t * p_evt) {
	// Check if the barcode scanner service was discovered
	debug_line("Discovering service UUID 0x%04x complete. Num of chars %d",
			p_evt->params.discovered_db.srv_uuid.uuid,
			p_evt->params.discovered_db.char_count);
	if (p_evt->evt_type == BLE_DB_DISCOVERY_COMPLETE &&
	p_evt->params.discovered_db.srv_uuid.uuid == BLE_UUID_SERVICE_TEST &&
	p_evt->params.discovered_db.srv_uuid.type == test_service_uuid.type)
	{
		test_service.conn_handle = p_evt->conn_handle;

		// Find the CCCD Handles of the scanner data characteristic
		for (uint8_t i = 0; i < p_evt->params.discovered_db.char_count; i++)
		{
			debug_line("Discovered characteristic UUID: 0x%04X handle 0x%04X",
					p_evt->params.discovered_db.charateristics[i].characteristic.uuid.uuid,
					p_evt->params.discovered_db.charateristics[i].characteristic.handle_value);
			if ((p_evt->params.discovered_db.charateristics[i].characteristic.uuid.uuid == BLE_UUID_CHARA_CONTROL)
				&&(p_evt->params.discovered_db.charateristics[i].characteristic.uuid.type==test_c_control_uuid.type))
			{
				// Found data characteristic, so we store its handles
				test_service.char_handles[TEST_CHAR_HANDLE_CONTROL_IDX].cccd_handle  = p_evt->params.discovered_db.charateristics[i].cccd_handle;
				test_service.char_handles[TEST_CHAR_HANDLE_CONTROL_IDX].value_handle = p_evt->params.discovered_db.charateristics[i].characteristic.handle_value;
				test_service.char_lookup_table[TEST_CHAR_HANDLE_CONTROL_IDX] = p_evt->params.discovered_db.charateristics[i].characteristic.uuid.uuid;

			} else if ((p_evt->params.discovered_db.charateristics[i].characteristic.uuid.uuid == BLE_UUID_CHARA_DATA)
				&&(p_evt->params.discovered_db.charateristics[i].characteristic.uuid.type==test_c_data_uuid.type))
			{
				// Found data characteristic, so we store its handles
				test_service.char_handles[TEST_CHAR_HANDLE_DATA_IDX].cccd_handle  = p_evt->params.discovered_db.charateristics[i].cccd_handle;
				test_service.char_handles[TEST_CHAR_HANDLE_DATA_IDX].value_handle = p_evt->params.discovered_db.charateristics[i].characteristic.handle_value;
				test_service.char_lookup_table[TEST_CHAR_HANDLE_DATA_IDX] = p_evt->params.discovered_db.charateristics[i].characteristic.uuid.uuid;

				enable_notifications(true, test_service.conn_handle, test_service.char_handles[TEST_CHAR_HANDLE_DATA_IDX].cccd_handle);
			} else {
				debug_line("Unknown characteristic UUID 0x%04x type 0x%04x",
						p_evt->params.discovered_db.charateristics[i].characteristic.uuid.uuid,
						p_evt->params.discovered_db.charateristics[i].characteristic.uuid.type);
			}
		}
		central_core_event_t evt;
		evt.type = CENTRAL_CORE_EVT_DISCOVERY_DONE;
		central_core_event_handler(evt);
	}
}

static void on_hvx(const ble_evt_t * p_ble_evt)
{
    // Check if this is a data or control notification
    if (p_ble_evt->evt.gattc_evt.params.hvx.handle == test_service.char_handles[TEST_CHAR_HANDLE_DATA_IDX].value_handle) {

//    	debug_data("Received DATA notification with data (len %d): ", p_ble_evt->evt.gattc_evt.params.hvx.len);
//
//    	for (uint8_t i = 0; i < p_ble_evt->evt.gattc_evt.params.hvx.len; i++) {
//    		debug_data("%02x ", p_ble_evt->evt.gattc_evt.params.hvx.data[i]);
//    	}
//    	debug_data("\n");
    } else if (p_ble_evt->evt.gattc_evt.params.hvx.handle == test_service.char_handles[TEST_CHAR_HANDLE_DATA_IDX].value_handle) {
//		debug_data("Received CONTROL notification with data (len %d): ", p_ble_evt->evt.gattc_evt.params.hvx.len);
//
//		for (uint8_t i = 0; i < p_ble_evt->evt.gattc_evt.params.hvx.len; i++) {
//			debug_data("%02x ", p_ble_evt->evt.gattc_evt.params.hvx.data[i]);
//		}
//		debug_data("\n");
	}
}

// End of event handlers ----------------------------------------------------------------------

// Helper functions ---------------------------------------------------------------------------

static void update_connection_handles(uint16_t conn_handle) {
	test_service.conn_handle = conn_handle;
}


static void enable_notifications(bool enable, uint16_t conn_handle, uint16_t handle_cccd)
{
    debug_line("Configuring CCCD for handle %04x", handle_cccd);

    uint16_t       cccd_val = enable ? BLE_GATT_HVX_NOTIFICATION : 0;

    ble_gattc_write_params_t gattc_params;

    cccd_msg[0]				= cccd_val;
    cccd_msg[1]				= 0;

    gattc_params.handle		= handle_cccd;
    gattc_params.len		= BLE_CCCD_VALUE_LEN;
    gattc_params.p_value	= cccd_msg;
    gattc_params.offset		= 0;
    gattc_params.write_op	= BLE_GATT_OP_WRITE_REQ;

    ret_code_t err_code = sd_ble_gattc_write(conn_handle, &gattc_params);

    if (err_code != NRF_SUCCESS) {
    	debug_error("Write to CCCD failed (0x%02X)", err_code);
    }
}

uint32_t write_to_test_char(uint8_t char_handle_idx, uint8_t len, uint8_t * data) {

	uint16_t chara_value_handle = test_service.char_handles[char_handle_idx].value_handle;

    VERIFY_PARAM_NOT_NULL(chara_value_handle);

    if (len > ble_get_max_data_length())
    {
        debug_error("Data length too long: %d", len);
        return NRF_ERROR_INVALID_PARAM;
    }
    if (test_service.conn_handle == BLE_CONN_HANDLE_INVALID)
    {
    	debug_error("Connection handle invalid");
        return NRF_ERROR_INVALID_STATE;
    }

    ble_gattc_write_params_t const write_params = {
        .write_op = BLE_GATT_OP_WRITE_REQ,
        .flags    = BLE_GATT_EXEC_WRITE_FLAG_PREPARED_WRITE,
        .handle   = chara_value_handle,
        .offset   = 0,
        .len      = len,
        .p_value  = data
    };


    debugL2("Writing data to conn %x char %x UUID %04x. Len %d",
    		test_service.conn_handle,
			chara_value_handle,
			get_test_handle_uuid(chara_value_handle),
			len);

    ret_code_t err_code = sd_ble_gattc_write(test_service.conn_handle, &write_params);
    return err_code;
}

uint32_t write_no_response_to_test_char(uint8_t char_handle_idx, uint8_t len, uint8_t * data) {

	uint16_t chara_value_handle = test_service.char_handles[char_handle_idx].value_handle;

    VERIFY_PARAM_NOT_NULL(chara_value_handle);

    if (len > ble_get_max_data_length())
    {
        debug_error("Data length too long: %d", len);
        return NRF_ERROR_INVALID_PARAM;
    }
    if (test_service.conn_handle == BLE_CONN_HANDLE_INVALID)
    {
    	debug_error("Connection handle invalid");
        return NRF_ERROR_INVALID_STATE;
    }

    ble_gattc_write_params_t const write_params = {
        .write_op = BLE_GATT_OP_WRITE_CMD,
        .flags    = BLE_GATT_EXEC_WRITE_FLAG_PREPARED_WRITE,
        .handle   = chara_value_handle,
        .offset   = 0,
        .len      = len,
        .p_value  = data
    };


    debugL2("Writing data to conn %x char %x UUID %04x. Len %d",
    		test_service.conn_handle,
			chara_value_handle,
			get_test_handle_uuid(chara_value_handle),
			len);

    ret_code_t err_code = sd_ble_gattc_write(test_service.conn_handle, &write_params);
    return err_code;
}

uint32_t central_ble_set_conn_param(ble_gap_conn_params_t const *p_conn_params) {
	return sd_ble_gap_conn_param_update(test_service.conn_handle, p_conn_params);
}

uint32_t read_test_char(uint8_t char_handle_idx) {

	uint16_t chara_value_handle = test_service.char_handles[char_handle_idx].value_handle;

    VERIFY_PARAM_NOT_NULL(chara_value_handle);

    if (test_service.conn_handle == BLE_CONN_HANDLE_INVALID)
    {
    	debug_error("Connection handle invalid");
        return NRF_ERROR_INVALID_STATE;
    }

//    debug_line("Reading data from conn %x char %x.",
//    		test_service.conn_handle,
//			chara_value_handle);

    ret_code_t err_code = sd_ble_gattc_read(test_service.conn_handle, chara_value_handle, 0);
    return err_code;
}


uint8_t get_test_handle_index(uint8_t handle) {
	for(uint8_t i=0; i<BLE_TEST_SERVICE_CHARA_NUM; i++) {
		if (handle == test_service.char_handles[i].value_handle) {
			return i;
		}
	}
	for(uint8_t i=0; i<BLE_TEST_SERVICE_CHARA_NUM; i++) {
		if (handle == test_service.char_handles[i].cccd_handle) {
			return 0x80 | i;
		}
	}
	return 0xFF;
}

uint16_t get_test_handle_uuid(uint8_t handle) {
	uint8_t index = get_test_handle_index(handle);
	if (index != 0xFF) {
		return test_service.char_lookup_table[index];
	} else {
		return 0xFFFF;
	}
}

