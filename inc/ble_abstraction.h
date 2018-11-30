/*
 * ble_abstraction.h
 *
 *  Created on: 6 May 2018
 *      Author: gaspe
 */

#ifndef BLE_ABSTRACTION_H_
#define BLE_ABSTRACTION_H_

#include "stdint.h"
#include "stdbool.h"
#include "ble_gatts.h"

// 128-bit base UUID (little endian)
#define BLE_UUID_PERIPHERAL_BASE		{{0x01, 0xF0, 0x01, 0xF0, 0x01, 0xF0, 0x01, 0xF0, 0x01, 0xF0, 0x01, 0xF0, 0x00, 0xF0, 0x01, 0xF0}}


// Defines for setting the different characteristics flags / permissions
#define BLE_READ 			0x01
#define BLE_WRITE 			0x02
#define BLE_NOTIFY 			0x04
#define BLE_VARIABLE_LEN	0x08
#define BLE_SECURE			0x10
#define BLE_CONST 			0x80	// Will be handled by the stack directly / no authorization from us needed (no read event generated)


typedef struct {
    uint16_t                    conn_handle;
    uint16_t                    service_handle; 	// Handle of service (as provided by the BLE stack)
    uint16_t					uuid;				// UUID of the service
    uint8_t 					char_num;			// Number of characteristics in this service
	ble_gatts_char_handles_t * 	char_handles;		// Pointer to an array of handles for characteristics of this service. def is in components/softdevice/s132/headers/ble_gatts.h
	bool *						char_notify;		// Pointer to an array of booleans which holds flags for missing notifications on subscription in the same index as the lookup table
	uint16_t *					char_lookup_table;	// Pointer to an array which holds the characteristic UUIDs in the same index as they are in handles array
	bool						pending_notify;		// Flag that tells whether there are pending subscription notifications
} ble_service_t;

void ble_abstraction_service_init(ble_service_t 				* p_custom_service,
								  uint16_t 						uuid,
								  uint8_t 						characteristic_num,
								  ble_gatts_char_handles_t		* char_handles,
								  uint16_t 						* char_lookup_table,
								  bool							* char_notify);

uint32_t ble_abstraction_chara_add(ble_service_t				* p_custom_service,
								   uint16_t						uuid,
								   uint8_t 						char_index,
								   uint8_t 						flags,
								   uint8_t 						max_len,
								   uint8_t 						init_len,
								   uint8_t 						* value);

uint32_t ble_abstraction_chara_notify(ble_service_t				* p_custom_service,
									  ble_gatts_char_handles_t	* char_handle,
									  uint8_t 					length,
									  uint8_t					* val);

uint8_t ble_abstraction_find_chara_by_uuid(ble_service_t 		*p_custom_service,
										   uint16_t 			uuid);



#endif /* BLE_ABSTRACTION_H_ */
