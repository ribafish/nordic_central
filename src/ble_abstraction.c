/*
 * ble_abstraction.c
 *
 *  Created on: 6 May 2018
 *      Author: gaspe
 *
 *
 *  This abstraction layer makes it easier to add services and characteristics
 */


#include "ble_abstraction.h"
#include "ble.h"
#include "nrf_log.h"
#include "app_error.h"
#include "ble_types.h"
#include "debug.h"

#define IS_OFFICIAL_SERVICE(uuid) ((uuid&0xFF00) == 0x1800)
#define IS_OFFICIAL_CHARACTERISTIC(uuid) ((uuid&0xFF00) == 0x2A00)

#define DEBUG	1
#define debug(...)  do { if (DEBUG>1) { debug_global(__VA_ARGS__); }} while (0)
#define debug_line(...)  do { if (DEBUG>1) { debug_line_global(__VA_ARGS__); debug_global("\n"); }} while (0)
#define debug_error(...)  do { if (DEBUG>0) { debug_errorline_global(__VA_ARGS__); debug_global("\n"); }} while (0)

void ble_abstraction_service_init(ble_service_t 			* p_custom_service,
								  uint16_t 					uuid,
								  uint8_t 					characteristic_num,
								  ble_gatts_char_handles_t	* char_handles,
								  uint16_t 					* char_lookup_table,
								  bool						* char_notify) {

    // Declare 16-bit service and 128-bit base UUIDs and add them to BLE stack table
    ret_code_t	err_code;
    ble_uuid_t	service_uuid;
    BLE_UUID_BLE_ASSIGN(service_uuid, uuid);

    // If its not an official service we have to add its UUID
    if (!IS_OFFICIAL_SERVICE(uuid)) {
		ble_uuid128_t base_uuid = BLE_UUID_PERIPHERAL_BASE;

		// Add the service to the BLE stack, vs_add is because we add the vendor UUID
		err_code = sd_ble_uuid_vs_add(&base_uuid, &service_uuid.type);
		if (err_code != NRF_SUCCESS) {
			debug_error("Service initialization error: sd_ble_uuid_vs_add finished with error code 0x%02X", err_code);
			APP_ERROR_CHECK(err_code);
		}
    }

    // Add our service
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &service_uuid,
                                        &p_custom_service->service_handle);
    if (err_code != NRF_SUCCESS) {
    	debug_error("Service initialization error: sd_ble_gatts_service_add finished with error code 0x%02X", err_code);
		APP_ERROR_CHECK(err_code);
	}

    p_custom_service->conn_handle = BLE_CONN_HANDLE_INVALID;
    p_custom_service->char_num = characteristic_num;
    p_custom_service->char_handles = char_handles;
    p_custom_service->char_lookup_table = char_lookup_table;
    p_custom_service->char_notify = char_notify;
    p_custom_service->uuid = uuid;
    p_custom_service->pending_notify = false;

    debug("Service UUID: %#06x\n", service_uuid.uuid);
}


uint32_t ble_abstraction_chara_add(ble_service_t * p_custom_service,
								   uint16_t uuid,
								   uint8_t char_index,
								   uint8_t flags,
								   uint8_t max_len,
								   uint8_t init_len,
								   uint8_t * value) {

    // Add a custom characteristic UUID
    ret_code_t err_code;
    ble_uuid_t char_uuid;
    BLE_UUID_BLE_ASSIGN(char_uuid, uuid);

    // If its not an official service we have to add its UUID
    if (!IS_OFFICIAL_CHARACTERISTIC(uuid)) {
		ble_uuid128_t base_uuid = BLE_UUID_PERIPHERAL_BASE;
		err_code = sd_ble_uuid_vs_add(&base_uuid, &char_uuid.type);

		if (err_code != NRF_SUCCESS) {
			debug_error("Characteristic declaration error: sd_ble_uuid_vs_add finished with error code 0x%02X\r\n", err_code);
			APP_ERROR_CHECK(err_code);
		}
    }
    // Save the UUID to index in service
    p_custom_service->char_lookup_table[char_index] = uuid;

    // Add read/write properties to our characteristic
    ble_gatts_char_md_t char_md;
    memset(&char_md, 0, sizeof(char_md));
    char_md.p_char_user_desc	= NULL;
	char_md.p_char_pf			= NULL;
	char_md.p_user_desc_md		= NULL;
	char_md.p_sccd_md			= NULL;
	//char_md.p_cccd_md			= NULL;
    char_md.char_props.read  = (flags & BLE_READ)  ? 1 : 0;
    char_md.char_props.write = (flags & BLE_WRITE) ? 1 : 0;

	// Configure Client Characteristic Configuration Descriptor metadata and add to char_md structure
	ble_gatts_attr_md_t cccd_md;
	memset(&cccd_md, 0, sizeof(cccd_md));

	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);

	if (flags & BLE_SECURE) {
		BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&cccd_md.write_perm);
	} else {
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
	}

	cccd_md.vloc                = BLE_GATTS_VLOC_STACK;		// CCCD should always be on the stack
	char_md.p_cccd_md           = &cccd_md;

	if (flags & BLE_NOTIFY) {
		char_md.char_props.notify   = 1;
    }

    // Configure the attribute metadata
    ble_gatts_attr_md_t attr_md;
    memset(&attr_md, 0, sizeof(attr_md));
    attr_md.vloc        = BLE_GATTS_VLOC_STACK; //IS_OFFICIAL_CHARACTERISTIC(uuid) ? BLE_GATTS_VLOC_STACK : BLE_GATTS_VLOC_USER;	// put official ones to stack, custom ones to user space
    attr_md.rd_auth		= ((flags & BLE_READ)  && ((flags & BLE_CONST) == 0)) ? 1 : 0;
    attr_md.wr_auth		= ((flags & BLE_WRITE) && ((flags & BLE_CONST) == 0)) ? 1 : 0;
    attr_md.vlen 		= (flags & BLE_VARIABLE_LEN) ? 1 : 0;

    // Set read/write security levels to our characteristic
    if (flags & BLE_SECURE) {
    	BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&attr_md.read_perm);
    	BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&attr_md.write_perm);
    } else{
    	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    }

    // Configure the characteristic value attribute
    ble_gatts_attr_t    attr_char_value;
    memset(&attr_char_value, 0, sizeof(attr_char_value));
    attr_char_value.p_uuid      = &char_uuid;
    attr_char_value.p_attr_md   = &attr_md;

    // Set characteristic length in number of bytes
    attr_char_value.max_len     = max_len;
	attr_char_value.init_len    = init_len;
    if (init_len>0 && value != (uint8_t *)0) {
		attr_char_value.p_value     = value;
    }

    // Add our new characteristic to the service
    err_code = sd_ble_gatts_characteristic_add(p_custom_service->service_handle,
                                               &char_md,
                                               &attr_char_value,
                                               &p_custom_service->char_handles[char_index]);
    if (err_code != NRF_SUCCESS) {
		debug_error("Characteristic declaration error: sd_ble_gatts_characteristic_add finished with error code 0x%02X\r\n", err_code);
		APP_ERROR_CHECK(err_code);
	}

    ble_gatts_char_handles_t * char_handle = &p_custom_service->char_handles[char_index];

    debug("CharUUID 0x%04X - Handles: val 0x%04X | cccd 0x%04X | sccd 0x%04X | user 0x%04X\r\n",
    		uuid, char_handle->value_handle, char_handle->cccd_handle, char_handle->sccd_handle, char_handle->user_desc_handle);

    return NRF_SUCCESS;
}

uint32_t ble_abstraction_chara_notify(ble_service_t 			* p_custom_service,
									  ble_gatts_char_handles_t	* char_handle,
									  uint8_t 					length,
									  uint8_t 					* val) {

	if (p_custom_service->conn_handle != BLE_CONN_HANDLE_INVALID) {
		uint16_t len = length;
		ble_gatts_hvx_params_t hvx_params;
		memset(&hvx_params, 0, sizeof(hvx_params));

		hvx_params.handle = char_handle->value_handle;
		hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
		hvx_params.offset = 0;
		hvx_params.p_len  = &len;
		hvx_params.p_data = val;

		return sd_ble_gatts_hvx(p_custom_service->conn_handle, &hvx_params);
	}
	return BLE_ERROR_INVALID_CONN_HANDLE;
}

uint8_t ble_abstraction_find_chara_by_uuid(ble_service_t *p_custom_service, uint16_t uuid) {
	for (uint8_t i = 0; i < p_custom_service->char_num; i++) {
		if (p_custom_service->char_lookup_table[i] == uuid) {
			return i;
		}
	}
	debug_error("ERROR: Can't find characteristic with UUID 0x%04X", uuid);
	return 0xFF;
}
