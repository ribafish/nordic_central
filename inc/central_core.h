/*
 * central_core.h
 *
 *  Created on: Jul 26, 2018
 *      Author: gksolutions
 */

#ifndef CENTRAL_CORE_H_
#define CENTRAL_CORE_H_


#include <stdint.h>
#include "ble_gap.h"

typedef enum {
	CENTRAL_CORE_STATE_INIT,
	CENTRAL_CORE_STATE_IDLE,
	CENTRAL_CORE_WRITE_SELFTEST,
	CENTRAL_CORE_READ_SELFTEST,
	CENTRAL_CORE_TEST_INIT,
	CENTRAL_CORE_TEST_INIT2,
	CENTRAL_CORE_TEST_START,
	CENTRAL_CORE_TEST_RUN,
	CENTRAL_CORE_WRITE_WAIT,
	CENTRAL_CORE_READ_WAIT,
	CENTRAL_CORE_NOTIFY_WAIT,
	CENTRAL_CORE_DELAY,
	CENTRAL_CORE_TEST_COMPLETE,
	CENTRAL_CORE_TEST_TERMINATE,
	CENTRAL_CORE_TEST_WAIT_PARAMS,
} central_core_state_t;


typedef enum {
	CENTRAL_CORE_EVT_CONNECTED,
	CENTRAL_CORE_EVT_DISCONNECTED,
	CENTRAL_CORE_EVT_DISCOVERY_DONE,	// done discovery for test service
	CENTRAL_CORE_EVT_WRITE_DONE,
	CENTRAL_CORE_EVT_WRITE_NO_RSP_DONE,
	CENTRAL_CORE_EVT_READ_DONE,
	CENTRAL_CORE_EVT_NOTIFY_RECEIVED,
	CENTRAL_CORE_EVT_CONN_PARAM_UPDATED,
	CENTRAL_CORE_EVT_PHY_UPDATED,
} central_core_event_type_t;


typedef struct {
	central_core_event_type_t type;
	union {
		struct {
			uint8_t * data;
			uint8_t datalen;
			uint16_t char_handle_id;
			uint16_t char_uuid;
		} re_wr_nt;
		uint16_t wr_no_rsp_count;
		ble_gap_conn_params_t conn_params;
		ble_gap_evt_phy_update_t phy_update;
	};
} central_core_event_t;

void central_core_init();
void central_core_update();

void central_core_event_handler(central_core_event_t evt);

#endif /* CENTRAL_CORE_H_ */
