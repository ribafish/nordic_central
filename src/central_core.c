/*
 * central_core.c
 *
 *  Created on: Jul 26, 2018
 *      Author: gksolutions
 */

#include "central_core.h"
#include "utils.h"
#include "control_commands.h"
#include "boards.h"
#include "bsp.h"
#include "bsp_btn_ble.h"

// BLE header files
#include "ble_stack.h"
#include "central_ble.h"

#include "app_timer.h"

// Debug header files
#include "debug.h"
#include "app_error.h"

#include "test_params.h"

#ifdef DEBUG
#undef DEBUG
#endif

#define DEBUG	1
#define debug_line(...)  do { if (DEBUG>0) { debug_line_global(__VA_ARGS__); debug_global("\n"); }} while (0)
#define debug_error(...)  do { if (DEBUG>0) { debug_errorline_global(__VA_ARGS__); debug_global("\n"); }} while (0)
#define debug_L2(...)  do { if (DEBUG>1) { debug_line_global(__VA_ARGS__); debug_global("\n"); }} while (0)
#define debug_data(...)  do { if (DEBUG>0) { debug_global(__VA_ARGS__); }} while (0)

#define MAX_QUEUED_TESTS 100


// Variables
static central_core_state_t state = CENTRAL_CORE_STATE_INIT;
RINGBUF_U16_DECLARE_INIT(state_core_next, 16);

bool write_done = true;
bool read_done = true;

uint8_t data[255];
uint8_t datalen = 0;

test_params_t current_test;
uint32_t current_test_bytes_done = 0;
uint32_t test_started_timestamp = 0;
uint32_t output_counter = 0;

test_params_t test_queue[MAX_QUEUED_TESTS];
uint8_t test_queue_head = 1;
RINGBUF_U8_DECLARE_INIT(test_queue_index, MAX_QUEUED_TESTS);

struct {
	uint8_t connected:1;
	uint8_t test_running:1;
	uint8_t conn_param_updated:1;
	uint8_t phy_updated;
} central_core_flags;

// Forward function declarations
static void timers_init();
void central_core_delay(uint32_t ms);
static central_core_state_t get_next_state();
static void queue_state(central_core_state_t next_state);
static void inject_state(central_core_state_t next_state);


void bsp_evt_handler(bsp_event_t evt);


void central_core_init() {
	state = CENTRAL_CORE_STATE_INIT;
}

struct {
	uint32_t delay_timestamp;
	uint32_t delay_ms;
} central_core_timer;


void central_core_update() {
	ret_code_t err_code;
	switch (state) {
	case CENTRAL_CORE_STATE_INIT:
		memset(&central_core_flags, 0, sizeof central_core_flags);

		// Initialize timer module
		timers_init();

		// Initialize BLE stack
		ble_stack_init();
		debug_line("Softdevice initialized");
		gap_params_init();
		debug_line("GAP params initialized");
		gatt_init();
		debug_line("GATT initialized");

		conn_params_init();
		db_discovery_init();
		peer_manager_init();
		debug_line("BLE stack completely initialized\n");

		// Initialize timestamping clock
		clock_timer_init();

		// Initialize the central
		central_ble_init();

		err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS, bsp_evt_handler);
	    APP_ERROR_CHECK(err_code);

		debug_error("CENTRAL completely initialized\n");

		// Start scanning for the peripheral part
		scan_start();

		state = CENTRAL_CORE_STATE_IDLE;
		break;
	case CENTRAL_CORE_WRITE_TEST:
		debug_line("Test write");
		uint8_t write[] = {CTRL_CMD_TEST_NOTIF,1,2,3,4};

		err_code = write_to_test_char(TEST_CHAR_HANDLE_CONTROL_IDX, 5, write);
	    if (err_code != NRF_SUCCESS) {
	    	debug_error("Write to control failed (0x%02X)", err_code);
			state = get_next_state();
	    } else {
	    	state = CENTRAL_CORE_WRITE_WAIT;
	    	write_done = false;
	    }
		break;
	case CENTRAL_CORE_READ_TEST:
		debug_line("Test read");
		err_code = read_test_char(TEST_CHAR_HANDLE_DATA_IDX);
	    if (err_code != NRF_SUCCESS) {
	    	debug_error("Read data failed (0x%02X)", err_code);
			state = get_next_state();
	    } else {
	    	state = CENTRAL_CORE_READ_WAIT;
	    	read_done = false;
	    }
		break;
	case CENTRAL_CORE_STATE_IDLE:
		if (ringbuf_u8_get_length(&test_queue_index) > 0 && central_core_flags.test_running != 1) {
			uint8_t idx = ringbuf_u8_pop(&test_queue_index);
			current_test = test_queue[idx];
			debug_line("test_queue index %d", idx);
			state = CENTRAL_CORE_TEST_INIT;
		} else {
			state = get_next_state();
		}
		break;
	case CENTRAL_CORE_TEST_INIT:
		if (current_test.test_case == TEST_NULL) {
			debug_error("Tried to init NULL test");
			state = get_next_state();
		} else {
			debug_line("Init test:");
			test_params_print(&current_test);

			test_params_set_all(&current_test);

			ble_stack_set_preferred_phy(current_test.rxtx_phy);
			ble_stack_set_phy(current_test.rxtx_phy);

			debug_line("Waiting for params...");
			central_core_flags.conn_param_updated = 0;
			central_core_flags.phy_updated = 0;


			state = CENTRAL_CORE_TEST_WAIT_PARAMS;
		}
		break;
	case CENTRAL_CORE_TEST_WAIT_PARAMS:
		if (central_core_flags.conn_param_updated && central_core_flags.phy_updated) {
			state = CENTRAL_CORE_TEST_INIT2;
			central_core_delay(50);
			central_core_flags.conn_param_updated = 0;
			central_core_flags.phy_updated = 0;
		}
		break;
	case CENTRAL_CORE_TEST_INIT2:
		data[0] = CTRL_CMD_WRITE_TEST_PARAMS;
		test_params_serialize(&current_test, &data[1], &datalen);
		current_test_bytes_done = 0;
		output_counter = 0;

		err_code = write_to_test_char(TEST_CHAR_HANDLE_CONTROL_IDX, datalen+1, data);
		if (err_code == NRF_SUCCESS) {
			state = CENTRAL_CORE_WRITE_WAIT;
			inject_state(CENTRAL_CORE_DELAY);
			inject_state(CENTRAL_CORE_TEST_START);
			write_done = false;
			central_core_timer.delay_ms = 2000;
			central_core_timer.delay_timestamp = clock_get_ms();
		} else if (err_code == NRF_ERROR_BUSY) {
			central_core_delay(10);
		} else {
			debug_error("Write to control failed (0x%02X)", err_code);
			state = get_next_state();
		}
		break;
	case CENTRAL_CORE_TEST_START:	// we'll just wait for the write to finish before changing all the settings

		data[0] = CTRL_CMD_START_TEST;
		err_code = write_to_test_char(TEST_CHAR_HANDLE_CONTROL_IDX, 1, data);
	    if (err_code == NRF_SUCCESS) {
	    	state = CENTRAL_CORE_WRITE_WAIT;
	    	inject_state(CENTRAL_CORE_TEST_RUN);
	    	write_done = false;
	    	central_core_flags.test_running = 1;
	    	debug_line("Started %s test", test_case_str[current_test.test_case]);
	    	test_started_timestamp = clock_get_ms();
	    } else if (err_code == NRF_ERROR_BUSY) {
	    	central_core_delay(10);
	    } else {
	    	debug_error("Write to control failed (0x%02X)", err_code);
			state = get_next_state();
	    }
		break;
	case CENTRAL_CORE_TEST_RUN:
    	if (current_test_bytes_done >= current_test.transfer_data_size) {
    		state = CENTRAL_CORE_TEST_COMPLETE;
    	} else {	// we've still got data to transmit
			switch(current_test.test_case) {
			case TEST_NULL:
				debug_line("NULL test case, exiting testing");
				state = get_next_state();
				break;
			case TEST_BLE_WRITE:
				test_params_build_data(&current_test, current_test_bytes_done, data, &datalen);
				err_code = write_to_test_char(TEST_CHAR_HANDLE_DATA_IDX, datalen, data);
				if (err_code == NRF_SUCCESS) {
					current_test_bytes_done += datalen;
					state = CENTRAL_CORE_TEST_RUN;
					if (current_test_bytes_done - output_counter >= current_test.transfer_data_size / 10) {
						debug_line("Wrote %d/%d KB)", current_test_bytes_done/1024, current_test.transfer_data_size/1024);
						output_counter = current_test_bytes_done;
					}
				} else if (err_code == NRF_ERROR_BUSY) {
					state = CENTRAL_CORE_WRITE_WAIT;
					inject_state(CENTRAL_CORE_TEST_RUN);
					write_done = false;
//					debug_error("Write busy @ byte %d", current_test_bytes_done);
//					central_core_delay(10);
				} else {
					debug_error("Write to data failed (0x%02X)", err_code);
					state = get_next_state();
				}
				break;
			case TEST_BLE_WRITE_NO_RSP:
				test_params_build_data(&current_test, current_test_bytes_done, data, &datalen);
				err_code = write_no_response_to_test_char(TEST_CHAR_HANDLE_DATA_IDX, datalen, data);
				if (err_code == NRF_SUCCESS) {
					current_test_bytes_done += datalen;	// this will get sent
					if (current_test_bytes_done - output_counter >= current_test.transfer_data_size / 10) {
						debug_line("Wrote %d/%d KB)", current_test_bytes_done/1024, current_test.transfer_data_size/1024);
						output_counter = current_test_bytes_done;
					}
				} else if (err_code == NRF_ERROR_RESOURCES) {
					state = CENTRAL_CORE_WRITE_WAIT;
					inject_state(CENTRAL_CORE_TEST_RUN);
					write_done = false;
//					debug_error("Write busy @ byte %d", current_test_bytes_done);
//					central_core_delay(10);
				} else {
					debug_error("Write no rsp to data failed (0x%02X)", err_code);
					state = get_next_state();
				}
				break;
			case TEST_BLE_READ:
				err_code = read_test_char(TEST_CHAR_HANDLE_DATA_IDX);
				if (err_code == NRF_SUCCESS) {
					state = CENTRAL_CORE_READ_WAIT;
					read_done = false;
					inject_state(CENTRAL_CORE_TEST_RUN);
				} else if (err_code == NRF_ERROR_BUSY) {
					debug_error("Read busy @ byte %d", current_test_bytes_done);
					central_core_delay(10);
				} else {
					debug_error("Read data failed (0x%02X)", err_code);
					state = get_next_state();
				}
				break;
			case TEST_BLE_NOTIFY: // else remain here and wait
				break;
			default:
				debug_error("Unknown test case %d, exiting testing", current_test.test_case);
				state = get_next_state();
				break;
			}
    	}
		break;
	case CENTRAL_CORE_TEST_COMPLETE:;
		float throughput =	8.0f * (float)current_test.transfer_data_size / ((float)clock_get_ms_since(test_started_timestamp) / 1000.0f) / 1024.0f; // Kbits per second
		float time = (float)clock_get_ms_since(test_started_timestamp) / 1000.0;
		debug_line("Finished test: %s of %d bytes", test_case_str[current_test.test_case], current_test.transfer_data_size);
		debug_line("Time: "NRF_LOG_FLOAT_MARKER"s", NRF_LOG_FLOAT(time));
		debug_line("Speed: "NRF_LOG_FLOAT_MARKER" Kbits/s", NRF_LOG_FLOAT(throughput));
		central_core_flags.test_running = 0;
		test_params_load(&current_test, BLE_4_2, TEST_NULL);
		current_test.conn_interval = 999.9f;
		test_params_set_all(&current_test);
		state = get_next_state();
		test_started_timestamp = 0;
		break;
	case CENTRAL_CORE_TEST_TERMINATE:
		debug_error("Terminate test. Done %d / %d KB", current_test_bytes_done, current_test.transfer_data_size);
		test_params_print(&current_test);
		central_core_flags.test_running = 0;
		test_params_load(&current_test, BLE_4_2, TEST_NULL);
		test_started_timestamp = 0;

		//empty the queue
		while(ringbuf_u16_get_length(&state_core_next)) {
			ringbuf_u16_pop(&state_core_next);
		}

		data[0] = CTRL_CMD_TERMINATE_TEST;
		err_code = write_to_test_char(TEST_CHAR_HANDLE_CONTROL_IDX, 1, data);
	    if (err_code == NRF_SUCCESS) {
	    	state = CENTRAL_CORE_WRITE_WAIT;
	    	write_done = false;
	    } else if (err_code == NRF_ERROR_BUSY) {
	    	central_core_delay(10);
	    } else {
	    	debug_error("Write to control failed (0x%02X)", err_code);
			state = get_next_state();
	    }
		break;
	case CENTRAL_CORE_WRITE_WAIT:
		if (write_done) {
			state = get_next_state();
		}
		break;
	case CENTRAL_CORE_READ_WAIT:
		if (read_done) {
			state = get_next_state();
		}
		break;
	case CENTRAL_CORE_DELAY:
		if (clock_get_ms_since(central_core_timer.delay_timestamp) > central_core_timer.delay_ms) {
			state = get_next_state();
		}
		break;
	default:
		debug_error("Unknown central state %d", state);
		state = CENTRAL_CORE_STATE_IDLE;
		break;
	}

}


void central_core_delay(uint32_t ms) {
	inject_state(state);
	state = CENTRAL_CORE_DELAY;
	central_core_timer.delay_ms = ms;
	central_core_timer.delay_timestamp = clock_get_ms();
}

void central_core_event_handler(central_core_event_t evt) {
	switch(evt.type) {
	case CENTRAL_CORE_EVT_CONNECTED:
		debug_line("Connected");
		central_core_flags.connected = 1;
		test_params_load(&current_test, BLE_4_2, TEST_NULL);
		current_test.conn_interval = 999.9f;
		test_params_set_all(&current_test);
		break;
	case CENTRAL_CORE_EVT_DISCONNECTED:
		debug_line("Disconnected -> resetting the core");
		central_core_flags.connected = 0;

		central_core_flags.test_running = 0;
		test_params_load(&current_test, BLE_4_1, TEST_NULL);
		test_started_timestamp = 0;

		//empty the state queue
		while(ringbuf_u16_get_length(&state_core_next)) {
			ringbuf_u16_pop(&state_core_next);
		}
		// empty the test queue
		while(ringbuf_u8_get_length(&test_queue_index)) {
			ringbuf_u8_pop(&test_queue_index);
		}

		ble_gap_conn_params_t default_connection_param = {
			.min_conn_interval	= CONN_INTERVAL_MIN,
			.max_conn_interval	= CONN_INTERVAL_MAX,
			.slave_latency		= SLAVE_LATENCY,
			.conn_sup_timeout	= CONN_SUP_TIMEOUT
		};
		ble_stack_set_conn_param(&default_connection_param);

		state = CENTRAL_CORE_STATE_IDLE;
		break;
	case CENTRAL_CORE_EVT_DISCOVERY_DONE:
		// wait for a write to CCCD (notif subscription) to finish, then test normal write and read
		queue_state(CENTRAL_CORE_WRITE_WAIT);
    	write_done = false;

		queue_state(CENTRAL_CORE_WRITE_TEST);
		queue_state(CENTRAL_CORE_READ_TEST);
		break;
	case CENTRAL_CORE_EVT_WRITE_DONE:
//		debug_line("Write done");
		write_done = true;
		if ( evt.re_wr_nt.char_handle_id & 0x80) {
			debug_line("Wrote to CCCD for char id %d", evt.re_wr_nt.char_handle_id & 0x7f);
		} else {
			if (evt.re_wr_nt.datalen > 0) {
				debug_line("Write RSP len %d", evt.re_wr_nt.datalen);
				for (uint8_t i =0; i<evt.re_wr_nt.datalen; i++) {
					debug_data("%02x ", evt.re_wr_nt.data[i]);
				}
				if (evt.re_wr_nt.datalen > 0) {
					debug_data("\n");
				}
			}
		}
		break;
	case CENTRAL_CORE_EVT_WRITE_NO_RSP_DONE:
		write_done = true;
		debug_L2("Wrote %d packets without response", evt.wr_no_rsp_count);
		break;
	case CENTRAL_CORE_EVT_READ_DONE:
//		debug_line("Read done");
		read_done = true;
		if (central_core_flags.test_running == 1 && evt.re_wr_nt.char_handle_id == TEST_CHAR_HANDLE_DATA_IDX) {
			if (evt.re_wr_nt.datalen == strlen(TEST_READ_NOTIFY_STRING) &&
				strncmp((char *) evt.re_wr_nt.data, TEST_READ_NOTIFY_STRING, evt.re_wr_nt.datalen) == 0) {
				debug_error("Read RSP bogus data: '%s'", TEST_READ_NOTIFY_STRING);
			} else {
				test_params_confirm_data(&current_test, current_test_bytes_done, evt.re_wr_nt.data, evt.re_wr_nt.datalen);
			}
			current_test_bytes_done += evt.re_wr_nt.datalen;
			if (current_test_bytes_done - output_counter >= current_test.transfer_data_size / 10) {
				debug_line("Read %d/%d KB)", current_test_bytes_done/1024, current_test.transfer_data_size/1024);
				output_counter = current_test_bytes_done;
			}
		} else if (evt.re_wr_nt.datalen == strlen(TEST_READ_NOTIFY_STRING) &&
			strncmp((char *) evt.re_wr_nt.data, TEST_READ_NOTIFY_STRING, evt.re_wr_nt.datalen) == 0) {
			debug_error("Read RSP bogus data: '%s'", TEST_READ_NOTIFY_STRING);
		} else {
			debug_line("Read RSP UUID %04x len %d:", evt.re_wr_nt.char_uuid, evt.re_wr_nt.datalen);
			for (uint8_t i =0; i<evt.re_wr_nt.datalen; i++) {
				debug_data("%02x ", evt.re_wr_nt.data[i]);
			}
			if (evt.re_wr_nt.datalen > 0) {
				debug_data("\n");
			}
		}
		break;
	case CENTRAL_CORE_EVT_NOTIFY_RECEIVED:
		if (central_core_flags.test_running == 1 && evt.re_wr_nt.char_handle_id == TEST_CHAR_HANDLE_DATA_IDX) {
			if (evt.re_wr_nt.datalen == strlen(TEST_READ_NOTIFY_STRING) &&
				strncmp((char *) evt.re_wr_nt.data, TEST_READ_NOTIFY_STRING, evt.re_wr_nt.datalen) == 0) {
				debug_error("Notif received bogus data: '%s'", TEST_READ_NOTIFY_STRING);
			} else {
				test_params_confirm_data(&current_test, current_test_bytes_done, evt.re_wr_nt.data, evt.re_wr_nt.datalen);
			}
			current_test_bytes_done += evt.re_wr_nt.datalen;
			if (current_test_bytes_done - output_counter >= current_test.transfer_data_size / 10) {
				debug_line("Notify rx %d/%d KB)", current_test_bytes_done/1024, current_test.transfer_data_size/1024);
				output_counter = current_test_bytes_done;
			}
		} else if (evt.re_wr_nt.datalen == strlen(TEST_READ_NOTIFY_STRING) &&
			strncmp((char *) evt.re_wr_nt.data, TEST_READ_NOTIFY_STRING,evt.re_wr_nt.datalen) == 0) {
			debug_error("Notif received bogus data: '%s'", TEST_READ_NOTIFY_STRING);
		} else {
			debug_line("Notif received UUID %04x len %d:", evt.re_wr_nt.char_uuid, evt.re_wr_nt.datalen);
			for (uint8_t i =0; i<evt.re_wr_nt.datalen; i++) {
				debug_data("%02x ", evt.re_wr_nt.data[i]);
			}
			if (evt.re_wr_nt.datalen > 0) {
				debug_data("\n");
			}
		}
		break;
	case CENTRAL_CORE_EVT_CONN_PARAM_UPDATED:
		if (evt.conn_params.max_conn_interval == MSEC_TO_UNITS(current_test.conn_interval, UNIT_1_25_MS)) {
			central_core_flags.conn_param_updated = 1;
		}
		break;
	case CENTRAL_CORE_EVT_PHY_UPDATED:
		if (current_test.rxtx_phy == evt.phy_update.rx_phy && current_test.rxtx_phy == evt.phy_update.tx_phy) {
			central_core_flags.phy_updated = 1;
		}
		break;
	default:
		debug_error("Unknown central event %d", evt.type);
		break;
	}
}

void bsp_evt_handler(bsp_event_t evt) {
	debug_line("Pressed button %d", evt-BSP_EVENT_KEY_0);

	test_case_t test_case = TEST_BLE_READ;
	test_ble_version_t test_ble_version = BLE_5_HS;
	uint32_t datasize = 1024*1024;

	switch(evt) {
	case BSP_EVENT_KEY_0:
		if (central_core_flags.test_running == 0 && central_core_flags.connected == 1) {
			test_case = TEST_BLE_NOTIFY;
			test_ble_version = BLE_4_1;
			if (ringbuf_u8_space_available(&test_queue_index) > 0) {
				test_params_load(&test_queue[test_queue_head], test_ble_version, test_case);
				test_queue[test_queue_head].transfer_data_size = datasize;
				test_queue[test_queue_head].conn_interval = 7.5f;
				test_queue[test_queue_head].conn_evt_len_ext_enabled = 1;
				ringbuf_u8_push(&test_queue_index, test_queue_head);
				test_queue_head = (test_queue_head + 1) % MAX_QUEUED_TESTS;
			}
			if (ringbuf_u8_space_available(&test_queue_index) > 0) {
				test_params_load(&test_queue[test_queue_head], test_ble_version, test_case);
				test_queue[test_queue_head].transfer_data_size = datasize;
				test_queue[test_queue_head].conn_interval = 30.0f;
				test_queue[test_queue_head].conn_evt_len_ext_enabled = 1;
				ringbuf_u8_push(&test_queue_index, test_queue_head);
				test_queue_head = (test_queue_head + 1) % MAX_QUEUED_TESTS;
			}
			if (ringbuf_u8_space_available(&test_queue_index) > 0) {
				test_params_load(&test_queue[test_queue_head], test_ble_version, test_case);
				test_queue[test_queue_head].transfer_data_size = datasize;
				test_queue[test_queue_head].conn_interval = 75.0f;
				test_queue[test_queue_head].conn_evt_len_ext_enabled = 1;
				ringbuf_u8_push(&test_queue_index, test_queue_head);
				test_queue_head = (test_queue_head + 1) % MAX_QUEUED_TESTS;
			}
			if (ringbuf_u8_space_available(&test_queue_index) > 0) {
				test_params_load(&test_queue[test_queue_head], test_ble_version, test_case);
				test_queue[test_queue_head].transfer_data_size = datasize;
				test_queue[test_queue_head].conn_interval = 150.0f;
				test_queue[test_queue_head].conn_evt_len_ext_enabled = 1;
				ringbuf_u8_push(&test_queue_index, test_queue_head);
				test_queue_head = (test_queue_head + 1) % MAX_QUEUED_TESTS;
			}
			if (ringbuf_u8_space_available(&test_queue_index) > 0) {
				test_params_load(&test_queue[test_queue_head], test_ble_version, test_case);
				test_queue[test_queue_head].transfer_data_size = datasize;
				test_queue[test_queue_head].conn_interval = 400.0f;
				test_queue[test_queue_head].conn_evt_len_ext_enabled = 1;
				ringbuf_u8_push(&test_queue_index, test_queue_head);
				test_queue_head = (test_queue_head + 1) % MAX_QUEUED_TESTS;
			}
			if (ringbuf_u8_space_available(&test_queue_index) > 0) {
				test_params_load(&test_queue[test_queue_head], test_ble_version, test_case);
				test_queue[test_queue_head].transfer_data_size = datasize;
				test_queue[test_queue_head].conn_interval = 1000.0f;
				test_queue[test_queue_head].conn_evt_len_ext_enabled = 1;
				ringbuf_u8_push(&test_queue_index, test_queue_head);
				test_queue_head = (test_queue_head + 1) % MAX_QUEUED_TESTS;
			}
		}
		break;
	case BSP_EVENT_KEY_1:
		if (central_core_flags.test_running == 0 && central_core_flags.connected == 1) {
			test_case = TEST_BLE_WRITE_NO_RSP;
			test_ble_version = BLE_4_1;
			if (ringbuf_u8_space_available(&test_queue_index) > 0) {
				test_params_load(&test_queue[test_queue_head], test_ble_version, test_case);
				test_queue[test_queue_head].transfer_data_size = datasize;
				test_queue[test_queue_head].conn_interval = 7.5f;
				test_queue[test_queue_head].conn_evt_len_ext_enabled = 1;
				ringbuf_u8_push(&test_queue_index, test_queue_head);
				test_queue_head = (test_queue_head + 1) % MAX_QUEUED_TESTS;
			}
			if (ringbuf_u8_space_available(&test_queue_index) > 0) {
				test_params_load(&test_queue[test_queue_head], test_ble_version, test_case);
				test_queue[test_queue_head].transfer_data_size = datasize;
				test_queue[test_queue_head].conn_interval = 30.0f;
				test_queue[test_queue_head].conn_evt_len_ext_enabled = 1;
				ringbuf_u8_push(&test_queue_index, test_queue_head);
				test_queue_head = (test_queue_head + 1) % MAX_QUEUED_TESTS;
			}
			if (ringbuf_u8_space_available(&test_queue_index) > 0) {
				test_params_load(&test_queue[test_queue_head], test_ble_version, test_case);
				test_queue[test_queue_head].transfer_data_size = datasize;
				test_queue[test_queue_head].conn_interval = 75.0f;
				test_queue[test_queue_head].conn_evt_len_ext_enabled = 1;
				ringbuf_u8_push(&test_queue_index, test_queue_head);
				test_queue_head = (test_queue_head + 1) % MAX_QUEUED_TESTS;
			}
			if (ringbuf_u8_space_available(&test_queue_index) > 0) {
				test_params_load(&test_queue[test_queue_head], test_ble_version, test_case);
				test_queue[test_queue_head].transfer_data_size = datasize;
				test_queue[test_queue_head].conn_interval = 150.0f;
				test_queue[test_queue_head].conn_evt_len_ext_enabled = 1;
				ringbuf_u8_push(&test_queue_index, test_queue_head);
				test_queue_head = (test_queue_head + 1) % MAX_QUEUED_TESTS;
			}
			if (ringbuf_u8_space_available(&test_queue_index) > 0) {
				test_params_load(&test_queue[test_queue_head], test_ble_version, test_case);
				test_queue[test_queue_head].transfer_data_size = datasize;
				test_queue[test_queue_head].conn_interval = 400.0f;
				test_queue[test_queue_head].conn_evt_len_ext_enabled = 1;
				ringbuf_u8_push(&test_queue_index, test_queue_head);
				test_queue_head = (test_queue_head + 1) % MAX_QUEUED_TESTS;
			}
			if (ringbuf_u8_space_available(&test_queue_index) > 0) {
				test_params_load(&test_queue[test_queue_head], test_ble_version, test_case);
				test_queue[test_queue_head].transfer_data_size = datasize;
				test_queue[test_queue_head].conn_interval = 1000.0f;
				test_queue[test_queue_head].conn_evt_len_ext_enabled = 1;
				ringbuf_u8_push(&test_queue_index, test_queue_head);
				test_queue_head = (test_queue_head + 1) % MAX_QUEUED_TESTS;
			}
		}
		break;
	case BSP_EVENT_KEY_2:
		if (central_core_flags.test_running == 0 && central_core_flags.connected == 1) {
			test_case = TEST_BLE_NOTIFY;
			test_ble_version = BLE_4_2;
			if (ringbuf_u8_space_available(&test_queue_index) > 0) {
				test_params_load(&test_queue[test_queue_head], test_ble_version, test_case);
				test_queue[test_queue_head].transfer_data_size = datasize;
				test_queue[test_queue_head].conn_interval = 7.5f;
				test_queue[test_queue_head].conn_evt_len_ext_enabled = 1;
				ringbuf_u8_push(&test_queue_index, test_queue_head);
				test_queue_head = (test_queue_head + 1) % MAX_QUEUED_TESTS;
			}
			if (ringbuf_u8_space_available(&test_queue_index) > 0) {
				test_params_load(&test_queue[test_queue_head], test_ble_version, test_case);
				test_queue[test_queue_head].transfer_data_size = datasize;
				test_queue[test_queue_head].conn_interval = 30.0f;
				test_queue[test_queue_head].conn_evt_len_ext_enabled = 1;
				ringbuf_u8_push(&test_queue_index, test_queue_head);
				test_queue_head = (test_queue_head + 1) % MAX_QUEUED_TESTS;
			}
			if (ringbuf_u8_space_available(&test_queue_index) > 0) {
				test_params_load(&test_queue[test_queue_head], test_ble_version, test_case);
				test_queue[test_queue_head].transfer_data_size = datasize;
				test_queue[test_queue_head].conn_interval = 75.0f;
				test_queue[test_queue_head].conn_evt_len_ext_enabled = 1;
				ringbuf_u8_push(&test_queue_index, test_queue_head);
				test_queue_head = (test_queue_head + 1) % MAX_QUEUED_TESTS;
			}
			if (ringbuf_u8_space_available(&test_queue_index) > 0) {
				test_params_load(&test_queue[test_queue_head], test_ble_version, test_case);
				test_queue[test_queue_head].transfer_data_size = datasize;
				test_queue[test_queue_head].conn_interval = 150.0f;
				test_queue[test_queue_head].conn_evt_len_ext_enabled = 1;
				ringbuf_u8_push(&test_queue_index, test_queue_head);
				test_queue_head = (test_queue_head + 1) % MAX_QUEUED_TESTS;
			}
			if (ringbuf_u8_space_available(&test_queue_index) > 0) {
				test_params_load(&test_queue[test_queue_head], test_ble_version, test_case);
				test_queue[test_queue_head].transfer_data_size = datasize;
				test_queue[test_queue_head].conn_interval = 400.0f;
				test_queue[test_queue_head].conn_evt_len_ext_enabled = 1;
				ringbuf_u8_push(&test_queue_index, test_queue_head);
				test_queue_head = (test_queue_head + 1) % MAX_QUEUED_TESTS;
			}
			if (ringbuf_u8_space_available(&test_queue_index) > 0) {
				test_params_load(&test_queue[test_queue_head], test_ble_version, test_case);
				test_queue[test_queue_head].transfer_data_size = datasize;
				test_queue[test_queue_head].conn_interval = 1000.0f;
				test_queue[test_queue_head].conn_evt_len_ext_enabled = 1;
				ringbuf_u8_push(&test_queue_index, test_queue_head);
				test_queue_head = (test_queue_head + 1) % MAX_QUEUED_TESTS;
			}
			test_case = TEST_BLE_WRITE_NO_RSP;
			test_ble_version = BLE_4_2;
			if (ringbuf_u8_space_available(&test_queue_index) > 0) {
				test_params_load(&test_queue[test_queue_head], test_ble_version, test_case);
				test_queue[test_queue_head].transfer_data_size = datasize;
				test_queue[test_queue_head].conn_interval = 7.5f;
				test_queue[test_queue_head].conn_evt_len_ext_enabled = 1;
				ringbuf_u8_push(&test_queue_index, test_queue_head);
				test_queue_head = (test_queue_head + 1) % MAX_QUEUED_TESTS;
			}
			if (ringbuf_u8_space_available(&test_queue_index) > 0) {
				test_params_load(&test_queue[test_queue_head], test_ble_version, test_case);
				test_queue[test_queue_head].transfer_data_size = datasize;
				test_queue[test_queue_head].conn_interval = 30.0f;
				test_queue[test_queue_head].conn_evt_len_ext_enabled = 1;
				ringbuf_u8_push(&test_queue_index, test_queue_head);
				test_queue_head = (test_queue_head + 1) % MAX_QUEUED_TESTS;
			}
			if (ringbuf_u8_space_available(&test_queue_index) > 0) {
				test_params_load(&test_queue[test_queue_head], test_ble_version, test_case);
				test_queue[test_queue_head].transfer_data_size = datasize;
				test_queue[test_queue_head].conn_interval = 75.0f;
				test_queue[test_queue_head].conn_evt_len_ext_enabled = 1;
				ringbuf_u8_push(&test_queue_index, test_queue_head);
				test_queue_head = (test_queue_head + 1) % MAX_QUEUED_TESTS;
			}
			if (ringbuf_u8_space_available(&test_queue_index) > 0) {
				test_params_load(&test_queue[test_queue_head], test_ble_version, test_case);
				test_queue[test_queue_head].transfer_data_size = datasize;
				test_queue[test_queue_head].conn_interval = 150.0f;
				test_queue[test_queue_head].conn_evt_len_ext_enabled = 1;
				ringbuf_u8_push(&test_queue_index, test_queue_head);
				test_queue_head = (test_queue_head + 1) % MAX_QUEUED_TESTS;
			}
			if (ringbuf_u8_space_available(&test_queue_index) > 0) {
				test_params_load(&test_queue[test_queue_head], test_ble_version, test_case);
				test_queue[test_queue_head].transfer_data_size = datasize;
				test_queue[test_queue_head].conn_interval = 400.0f;
				test_queue[test_queue_head].conn_evt_len_ext_enabled = 1;
				ringbuf_u8_push(&test_queue_index, test_queue_head);
				test_queue_head = (test_queue_head + 1) % MAX_QUEUED_TESTS;
			}
			if (ringbuf_u8_space_available(&test_queue_index) > 0) {
				test_params_load(&test_queue[test_queue_head], test_ble_version, test_case);
				test_queue[test_queue_head].transfer_data_size = datasize;
				test_queue[test_queue_head].conn_interval = 1000.0f;
				test_queue[test_queue_head].conn_evt_len_ext_enabled = 1;
				ringbuf_u8_push(&test_queue_index, test_queue_head);
				test_queue_head = (test_queue_head + 1) % MAX_QUEUED_TESTS;
			}
		}
		break;
	case BSP_EVENT_KEY_3:
		if (central_core_flags.test_running == 0 && central_core_flags.connected == 1) {
			test_case = TEST_BLE_NOTIFY;
			test_ble_version = BLE_5_HS;
			if (ringbuf_u8_space_available(&test_queue_index) > 0) {
				test_params_load(&test_queue[test_queue_head], test_ble_version, test_case);
				test_queue[test_queue_head].transfer_data_size = datasize;
				test_queue[test_queue_head].conn_interval = 7.5f;
				test_queue[test_queue_head].conn_evt_len_ext_enabled = 1;
				ringbuf_u8_push(&test_queue_index, test_queue_head);
				test_queue_head = (test_queue_head + 1) % MAX_QUEUED_TESTS;
			}
			if (ringbuf_u8_space_available(&test_queue_index) > 0) {
				test_params_load(&test_queue[test_queue_head], test_ble_version, test_case);
				test_queue[test_queue_head].transfer_data_size = datasize;
				test_queue[test_queue_head].conn_interval = 30.0f;
				test_queue[test_queue_head].conn_evt_len_ext_enabled = 1;
				ringbuf_u8_push(&test_queue_index, test_queue_head);
				test_queue_head = (test_queue_head + 1) % MAX_QUEUED_TESTS;
			}
			if (ringbuf_u8_space_available(&test_queue_index) > 0) {
				test_params_load(&test_queue[test_queue_head], test_ble_version, test_case);
				test_queue[test_queue_head].transfer_data_size = datasize;
				test_queue[test_queue_head].conn_interval = 75.0f;
				test_queue[test_queue_head].conn_evt_len_ext_enabled = 1;
				ringbuf_u8_push(&test_queue_index, test_queue_head);
				test_queue_head = (test_queue_head + 1) % MAX_QUEUED_TESTS;
			}
			if (ringbuf_u8_space_available(&test_queue_index) > 0) {
				test_params_load(&test_queue[test_queue_head], test_ble_version, test_case);
				test_queue[test_queue_head].transfer_data_size = datasize;
				test_queue[test_queue_head].conn_interval = 150.0f;
				test_queue[test_queue_head].conn_evt_len_ext_enabled = 1;
				ringbuf_u8_push(&test_queue_index, test_queue_head);
				test_queue_head = (test_queue_head + 1) % MAX_QUEUED_TESTS;
			}
			if (ringbuf_u8_space_available(&test_queue_index) > 0) {
				test_params_load(&test_queue[test_queue_head], test_ble_version, test_case);
				test_queue[test_queue_head].transfer_data_size = datasize;
				test_queue[test_queue_head].conn_interval = 400.0f;
				test_queue[test_queue_head].conn_evt_len_ext_enabled = 1;
				ringbuf_u8_push(&test_queue_index, test_queue_head);
				test_queue_head = (test_queue_head + 1) % MAX_QUEUED_TESTS;
			}
			if (ringbuf_u8_space_available(&test_queue_index) > 0) {
				test_params_load(&test_queue[test_queue_head], test_ble_version, test_case);
				test_queue[test_queue_head].transfer_data_size = datasize;
				test_queue[test_queue_head].conn_interval = 1000.0f;
				test_queue[test_queue_head].conn_evt_len_ext_enabled = 1;
				ringbuf_u8_push(&test_queue_index, test_queue_head);
				test_queue_head = (test_queue_head + 1) % MAX_QUEUED_TESTS;
			}
			test_case = TEST_BLE_WRITE_NO_RSP;
			test_ble_version = BLE_5_HS;
			if (ringbuf_u8_space_available(&test_queue_index) > 0) {
				test_params_load(&test_queue[test_queue_head], test_ble_version, test_case);
				test_queue[test_queue_head].transfer_data_size = datasize;
				test_queue[test_queue_head].conn_interval = 7.5f;
				test_queue[test_queue_head].conn_evt_len_ext_enabled = 1;
				ringbuf_u8_push(&test_queue_index, test_queue_head);
				test_queue_head = (test_queue_head + 1) % MAX_QUEUED_TESTS;
			}
			if (ringbuf_u8_space_available(&test_queue_index) > 0) {
				test_params_load(&test_queue[test_queue_head], test_ble_version, test_case);
				test_queue[test_queue_head].transfer_data_size = datasize;
				test_queue[test_queue_head].conn_interval = 30.0f;
				test_queue[test_queue_head].conn_evt_len_ext_enabled = 1;
				ringbuf_u8_push(&test_queue_index, test_queue_head);
				test_queue_head = (test_queue_head + 1) % MAX_QUEUED_TESTS;
			}
			if (ringbuf_u8_space_available(&test_queue_index) > 0) {
				test_params_load(&test_queue[test_queue_head], test_ble_version, test_case);
				test_queue[test_queue_head].transfer_data_size = datasize;
				test_queue[test_queue_head].conn_interval = 75.0f;
				test_queue[test_queue_head].conn_evt_len_ext_enabled = 1;
				ringbuf_u8_push(&test_queue_index, test_queue_head);
				test_queue_head = (test_queue_head + 1) % MAX_QUEUED_TESTS;
			}
			if (ringbuf_u8_space_available(&test_queue_index) > 0) {
				test_params_load(&test_queue[test_queue_head], test_ble_version, test_case);
				test_queue[test_queue_head].transfer_data_size = datasize;
				test_queue[test_queue_head].conn_interval = 150.0f;
				test_queue[test_queue_head].conn_evt_len_ext_enabled = 1;
				ringbuf_u8_push(&test_queue_index, test_queue_head);
				test_queue_head = (test_queue_head + 1) % MAX_QUEUED_TESTS;
			}
			if (ringbuf_u8_space_available(&test_queue_index) > 0) {
				test_params_load(&test_queue[test_queue_head], test_ble_version, test_case);
				test_queue[test_queue_head].transfer_data_size = datasize;
				test_queue[test_queue_head].conn_interval = 400.0f;
				test_queue[test_queue_head].conn_evt_len_ext_enabled = 1;
				ringbuf_u8_push(&test_queue_index, test_queue_head);
				test_queue_head = (test_queue_head + 1) % MAX_QUEUED_TESTS;
			}
			if (ringbuf_u8_space_available(&test_queue_index) > 0) {
				test_params_load(&test_queue[test_queue_head], test_ble_version, test_case);
				test_queue[test_queue_head].transfer_data_size = datasize;
				test_queue[test_queue_head].conn_interval = 1000.0f;
				test_queue[test_queue_head].conn_evt_len_ext_enabled = 1;
				ringbuf_u8_push(&test_queue_index, test_queue_head);
				test_queue_head = (test_queue_head + 1) % MAX_QUEUED_TESTS;
			}
		}
		break;
	default:
		break;
	}

	/*
	switch(evt) {
		case BSP_EVENT_KEY_0:
			if (central_core_flags.test_running == 0 && central_core_flags.connected == 1) {
				test_case = TEST_BLE_READ;
				datasize = 12800; //=12.5f*1024;
				if (ringbuf_u8_space_available(&test_queue_index) > 0) {
					test_params_load(&test_queue[test_queue_head], test_ble_version, test_case);
					test_queue[test_queue_head].transfer_data_size = datasize;
					test_queue[test_queue_head].conn_interval = 7.5f;
					test_queue[test_queue_head].conn_evt_len_ext_enabled = 1;
					ringbuf_u8_push(&test_queue_index, test_queue_head);
					test_queue_head = (test_queue_head + 1) % MAX_QUEUED_TESTS;
				}
				if (ringbuf_u8_space_available(&test_queue_index) > 0) {
					test_params_load(&test_queue[test_queue_head], test_ble_version, test_case);
					test_queue[test_queue_head].transfer_data_size = datasize;
					test_queue[test_queue_head].conn_interval = 30.0f;
					test_queue[test_queue_head].conn_evt_len_ext_enabled = 1;
					ringbuf_u8_push(&test_queue_index, test_queue_head);
					test_queue_head = (test_queue_head + 1) % MAX_QUEUED_TESTS;
				}
				if (ringbuf_u8_space_available(&test_queue_index) > 0) {
					test_params_load(&test_queue[test_queue_head], test_ble_version, test_case);
					test_queue[test_queue_head].transfer_data_size = datasize;
					test_queue[test_queue_head].conn_interval = 75.0f;
					test_queue[test_queue_head].conn_evt_len_ext_enabled = 1;
					ringbuf_u8_push(&test_queue_index, test_queue_head);
					test_queue_head = (test_queue_head + 1) % MAX_QUEUED_TESTS;
				}
				if (ringbuf_u8_space_available(&test_queue_index) > 0) {
					test_params_load(&test_queue[test_queue_head], test_ble_version, test_case);
					test_queue[test_queue_head].transfer_data_size = datasize;
					test_queue[test_queue_head].conn_interval = 150.0f;
					test_queue[test_queue_head].conn_evt_len_ext_enabled = 1;
					ringbuf_u8_push(&test_queue_index, test_queue_head);
					test_queue_head = (test_queue_head + 1) % MAX_QUEUED_TESTS;
				}
				if (ringbuf_u8_space_available(&test_queue_index) > 0) {
					test_params_load(&test_queue[test_queue_head], test_ble_version, test_case);
					test_queue[test_queue_head].transfer_data_size = datasize;
					test_queue[test_queue_head].conn_interval = 400.0f;
					test_queue[test_queue_head].conn_evt_len_ext_enabled = 1;
					ringbuf_u8_push(&test_queue_index, test_queue_head);
					test_queue_head = (test_queue_head + 1) % MAX_QUEUED_TESTS;
				}
				if (ringbuf_u8_space_available(&test_queue_index) > 0) {
					test_params_load(&test_queue[test_queue_head], test_ble_version, test_case);
					test_queue[test_queue_head].transfer_data_size = datasize;
					test_queue[test_queue_head].conn_interval = 1000.0f;
					test_queue[test_queue_head].conn_evt_len_ext_enabled = 1;
					ringbuf_u8_push(&test_queue_index, test_queue_head);
					test_queue_head = (test_queue_head + 1) % MAX_QUEUED_TESTS;
				}
			}
			break;
		case BSP_EVENT_KEY_1:
			if (central_core_flags.test_running == 0 && central_core_flags.connected == 1) {
				test_case = TEST_BLE_WRITE;
				datasize = 12800; //=12.5f*1024;
				if (ringbuf_u8_space_available(&test_queue_index) > 0) {
					test_params_load(&test_queue[test_queue_head], test_ble_version, test_case);
					test_queue[test_queue_head].transfer_data_size = datasize;
					test_queue[test_queue_head].conn_interval = 7.5f;
					test_queue[test_queue_head].conn_evt_len_ext_enabled = 1;
					ringbuf_u8_push(&test_queue_index, test_queue_head);
					test_queue_head = (test_queue_head + 1) % MAX_QUEUED_TESTS;
				}
				if (ringbuf_u8_space_available(&test_queue_index) > 0) {
					test_params_load(&test_queue[test_queue_head], test_ble_version, test_case);
					test_queue[test_queue_head].transfer_data_size = datasize;
					test_queue[test_queue_head].conn_interval = 30.0f;
					test_queue[test_queue_head].conn_evt_len_ext_enabled = 1;
					ringbuf_u8_push(&test_queue_index, test_queue_head);
					test_queue_head = (test_queue_head + 1) % MAX_QUEUED_TESTS;
				}
				if (ringbuf_u8_space_available(&test_queue_index) > 0) {
					test_params_load(&test_queue[test_queue_head], test_ble_version, test_case);
					test_queue[test_queue_head].transfer_data_size = datasize;
					test_queue[test_queue_head].conn_interval = 75.0f;
					test_queue[test_queue_head].conn_evt_len_ext_enabled = 1;
					ringbuf_u8_push(&test_queue_index, test_queue_head);
					test_queue_head = (test_queue_head + 1) % MAX_QUEUED_TESTS;
				}
				if (ringbuf_u8_space_available(&test_queue_index) > 0) {
					test_params_load(&test_queue[test_queue_head], test_ble_version, test_case);
					test_queue[test_queue_head].transfer_data_size = datasize;
					test_queue[test_queue_head].conn_interval = 150.0f;
					test_queue[test_queue_head].conn_evt_len_ext_enabled = 1;
					ringbuf_u8_push(&test_queue_index, test_queue_head);
					test_queue_head = (test_queue_head + 1) % MAX_QUEUED_TESTS;
				}
				if (ringbuf_u8_space_available(&test_queue_index) > 0) {
					test_params_load(&test_queue[test_queue_head], test_ble_version, test_case);
					test_queue[test_queue_head].transfer_data_size = datasize;
					test_queue[test_queue_head].conn_interval = 400.0f;
					test_queue[test_queue_head].conn_evt_len_ext_enabled = 1;
					ringbuf_u8_push(&test_queue_index, test_queue_head);
					test_queue_head = (test_queue_head + 1) % MAX_QUEUED_TESTS;
				}
				if (ringbuf_u8_space_available(&test_queue_index) > 0) {
					test_params_load(&test_queue[test_queue_head], test_ble_version, test_case);
					test_queue[test_queue_head].transfer_data_size = datasize;
					test_queue[test_queue_head].conn_interval = 1000.0f;
					test_queue[test_queue_head].conn_evt_len_ext_enabled = 1;
					ringbuf_u8_push(&test_queue_index, test_queue_head);
					test_queue_head = (test_queue_head + 1) % MAX_QUEUED_TESTS;
				}
			}
			break;
		case BSP_EVENT_KEY_2:
			if (central_core_flags.test_running == 0 && central_core_flags.connected == 1) {
				test_case = TEST_BLE_NOTIFY;
				datasize = 100*1024;
				if (ringbuf_u8_space_available(&test_queue_index) > 0) {
					test_params_load(&test_queue[test_queue_head], test_ble_version, test_case);
					test_queue[test_queue_head].transfer_data_size = datasize;
					test_queue[test_queue_head].conn_interval = 7.5f;
					test_queue[test_queue_head].conn_evt_len_ext_enabled = 1;
					ringbuf_u8_push(&test_queue_index, test_queue_head);
					test_queue_head = (test_queue_head + 1) % MAX_QUEUED_TESTS;
				}
				if (ringbuf_u8_space_available(&test_queue_index) > 0) {
					test_params_load(&test_queue[test_queue_head], test_ble_version, test_case);
					test_queue[test_queue_head].transfer_data_size = datasize;
					test_queue[test_queue_head].conn_interval = 30.0f;
					test_queue[test_queue_head].conn_evt_len_ext_enabled = 1;
					ringbuf_u8_push(&test_queue_index, test_queue_head);
					test_queue_head = (test_queue_head + 1) % MAX_QUEUED_TESTS;
				}
				if (ringbuf_u8_space_available(&test_queue_index) > 0) {
					test_params_load(&test_queue[test_queue_head], test_ble_version, test_case);
					test_queue[test_queue_head].transfer_data_size = datasize;
					test_queue[test_queue_head].conn_interval = 75.0f;
					test_queue[test_queue_head].conn_evt_len_ext_enabled = 1;
					ringbuf_u8_push(&test_queue_index, test_queue_head);
					test_queue_head = (test_queue_head + 1) % MAX_QUEUED_TESTS;
				}
				if (ringbuf_u8_space_available(&test_queue_index) > 0) {
					test_params_load(&test_queue[test_queue_head], test_ble_version, test_case);
					test_queue[test_queue_head].transfer_data_size = datasize;
					test_queue[test_queue_head].conn_interval = 150.0f;
					test_queue[test_queue_head].conn_evt_len_ext_enabled = 1;
					ringbuf_u8_push(&test_queue_index, test_queue_head);
					test_queue_head = (test_queue_head + 1) % MAX_QUEUED_TESTS;
				}
				if (ringbuf_u8_space_available(&test_queue_index) > 0) {
					test_params_load(&test_queue[test_queue_head], test_ble_version, test_case);
					test_queue[test_queue_head].transfer_data_size = datasize;
					test_queue[test_queue_head].conn_interval = 400.0f;
					test_queue[test_queue_head].conn_evt_len_ext_enabled = 1;
					ringbuf_u8_push(&test_queue_index, test_queue_head);
					test_queue_head = (test_queue_head + 1) % MAX_QUEUED_TESTS;
				}
				if (ringbuf_u8_space_available(&test_queue_index) > 0) {
					test_params_load(&test_queue[test_queue_head], test_ble_version, test_case);
					test_queue[test_queue_head].transfer_data_size = datasize;
					test_queue[test_queue_head].conn_interval = 1000.0f;
					test_queue[test_queue_head].conn_evt_len_ext_enabled = 1;
					ringbuf_u8_push(&test_queue_index, test_queue_head);
					test_queue_head = (test_queue_head + 1) % MAX_QUEUED_TESTS;
				}
			}
			break;
		case BSP_EVENT_KEY_3:
			if (central_core_flags.test_running == 0 && central_core_flags.connected == 1) {
				test_case = TEST_BLE_WRITE_NO_RSP;
				datasize = 100*1024;
				if (ringbuf_u8_space_available(&test_queue_index) > 0) {
					test_params_load(&test_queue[test_queue_head], test_ble_version, test_case);
					test_queue[test_queue_head].transfer_data_size = datasize;
					test_queue[test_queue_head].conn_interval = 7.5f;
					test_queue[test_queue_head].conn_evt_len_ext_enabled = 1;
					ringbuf_u8_push(&test_queue_index, test_queue_head);
					test_queue_head = (test_queue_head + 1) % MAX_QUEUED_TESTS;
				}
				if (ringbuf_u8_space_available(&test_queue_index) > 0) {
					test_params_load(&test_queue[test_queue_head], test_ble_version, test_case);
					test_queue[test_queue_head].transfer_data_size = datasize;
					test_queue[test_queue_head].conn_interval = 30.0f;
					test_queue[test_queue_head].conn_evt_len_ext_enabled = 1;
					ringbuf_u8_push(&test_queue_index, test_queue_head);
					test_queue_head = (test_queue_head + 1) % MAX_QUEUED_TESTS;
				}
				if (ringbuf_u8_space_available(&test_queue_index) > 0) {
					test_params_load(&test_queue[test_queue_head], test_ble_version, test_case);
					test_queue[test_queue_head].transfer_data_size = datasize;
					test_queue[test_queue_head].conn_interval = 75.0f;
					test_queue[test_queue_head].conn_evt_len_ext_enabled = 1;
					ringbuf_u8_push(&test_queue_index, test_queue_head);
					test_queue_head = (test_queue_head + 1) % MAX_QUEUED_TESTS;
				}
				if (ringbuf_u8_space_available(&test_queue_index) > 0) {
					test_params_load(&test_queue[test_queue_head], test_ble_version, test_case);
					test_queue[test_queue_head].transfer_data_size = datasize;
					test_queue[test_queue_head].conn_interval = 150.0f;
					test_queue[test_queue_head].conn_evt_len_ext_enabled = 1;
					ringbuf_u8_push(&test_queue_index, test_queue_head);
					test_queue_head = (test_queue_head + 1) % MAX_QUEUED_TESTS;
				}
				if (ringbuf_u8_space_available(&test_queue_index) > 0) {
					test_params_load(&test_queue[test_queue_head], test_ble_version, test_case);
					test_queue[test_queue_head].transfer_data_size = datasize;
					test_queue[test_queue_head].conn_interval = 400.0f;
					test_queue[test_queue_head].conn_evt_len_ext_enabled = 1;
					ringbuf_u8_push(&test_queue_index, test_queue_head);
					test_queue_head = (test_queue_head + 1) % MAX_QUEUED_TESTS;
				}
				if (ringbuf_u8_space_available(&test_queue_index) > 0) {
					test_params_load(&test_queue[test_queue_head], test_ble_version, test_case);
					test_queue[test_queue_head].transfer_data_size = datasize;
					test_queue[test_queue_head].conn_interval = 1000.0f;
					test_queue[test_queue_head].conn_evt_len_ext_enabled = 1;
					ringbuf_u8_push(&test_queue_index, test_queue_head);
					test_queue_head = (test_queue_head + 1) % MAX_QUEUED_TESTS;
				}
			}
			break;
		default:
			break;
	}
	*/

	/*
	// Tests for BLE 4.1
	float conn_interval = 7.5f;

	switch(evt) {
	case BSP_EVENT_KEY_0:
		if (central_core_flags.test_running == 0 && central_core_flags.connected == 1) {
//			conn_interval = 50.0f;
			if (ringbuf_u8_space_available(&test_queue_index) > 0) {
				test_params_load(&test_queue[test_queue_head], test_ble_version, test_case);
				test_queue[test_queue_head].transfer_data_size = 100;
				test_queue[test_queue_head].conn_interval = conn_interval;
				test_queue[test_queue_head].conn_evt_len_ext_enabled = 1;
//				test_params_print(&test_queue[test_queue_head]);
				ringbuf_u8_push(&test_queue_index, test_queue_head);
				test_queue_head = (test_queue_head + 1) % MAX_QUEUED_TESTS;
			}
			if (ringbuf_u8_space_available(&test_queue_index) > 0) {
				test_params_load(&test_queue[test_queue_head], test_ble_version, test_case);
				test_queue[test_queue_head].transfer_data_size = 400;
				test_queue[test_queue_head].conn_interval = conn_interval;
				test_queue[test_queue_head].conn_evt_len_ext_enabled = 1;
//				test_params_print(&test_queue[test_queue_head]);
				ringbuf_u8_push(&test_queue_index, test_queue_head);
				test_queue_head = (test_queue_head + 1) % MAX_QUEUED_TESTS;
			}
			if (ringbuf_u8_space_available(&test_queue_index) > 0) {
				test_params_load(&test_queue[test_queue_head], test_ble_version, test_case);
				test_queue[test_queue_head].transfer_data_size = 1000;
				test_queue[test_queue_head].conn_interval = conn_interval;
				test_queue[test_queue_head].conn_evt_len_ext_enabled = 1;
//				test_params_print(&test_queue[test_queue_head]);
				ringbuf_u8_push(&test_queue_index, test_queue_head);
				test_queue_head = (test_queue_head + 1) % MAX_QUEUED_TESTS;
			}

			if (ringbuf_u8_space_available(&test_queue_index) > 0) {
				test_params_load(&test_queue[test_queue_head], test_ble_version, test_case);
				test_queue[test_queue_head].transfer_data_size = 10 * 1000;
				test_queue[test_queue_head].conn_interval = conn_interval;
				test_queue[test_queue_head].conn_evt_len_ext_enabled = 1;
//				test_params_print(&test_queue[test_queue_head]);
				ringbuf_u8_push(&test_queue_index, test_queue_head);
				test_queue_head = (test_queue_head + 1) % MAX_QUEUED_TESTS;
			}

			if (ringbuf_u8_space_available(&test_queue_index) > 0) {
				test_params_load(&test_queue[test_queue_head], test_ble_version, test_case);
				test_queue[test_queue_head].transfer_data_size = 100 * 1000;
				test_queue[test_queue_head].conn_interval = conn_interval;
				test_queue[test_queue_head].conn_evt_len_ext_enabled = 1;
//				test_params_print(&test_queue[test_queue_head]);
				ringbuf_u8_push(&test_queue_index, test_queue_head);
				test_queue_head = (test_queue_head + 1) % MAX_QUEUED_TESTS;
			}
		}
		break;
	case BSP_EVENT_KEY_1:
		if (central_core_flags.test_running == 0 && central_core_flags.connected == 1) {
			conn_interval = 400.0f;
			if (ringbuf_u8_space_available(&test_queue_index) > 0) {
				test_params_load(&test_queue[test_queue_head], test_ble_version, test_case);
				test_queue[test_queue_head].transfer_data_size = 100;
				test_queue[test_queue_head].conn_interval = conn_interval;
				test_queue[test_queue_head].conn_evt_len_ext_enabled = 1;
//				test_params_print(&test_queue[test_queue_head]);
				ringbuf_u8_push(&test_queue_index, test_queue_head);
				test_queue_head = (test_queue_head + 1) % MAX_QUEUED_TESTS;
			}
			if (ringbuf_u8_space_available(&test_queue_index) > 0) {
				test_params_load(&test_queue[test_queue_head], test_ble_version, test_case);
				test_queue[test_queue_head].transfer_data_size = 400;
				test_queue[test_queue_head].conn_interval = conn_interval;
				test_queue[test_queue_head].conn_evt_len_ext_enabled = 1;
//				test_params_print(&test_queue[test_queue_head]);
				ringbuf_u8_push(&test_queue_index, test_queue_head);
				test_queue_head = (test_queue_head + 1) % MAX_QUEUED_TESTS;
			}
			if (ringbuf_u8_space_available(&test_queue_index) > 0) {
				test_params_load(&test_queue[test_queue_head], test_ble_version, test_case);
				test_queue[test_queue_head].transfer_data_size = 1000;
				test_queue[test_queue_head].conn_interval = conn_interval;
				test_queue[test_queue_head].conn_evt_len_ext_enabled = 1;
//				test_params_print(&test_queue[test_queue_head]);
				ringbuf_u8_push(&test_queue_index, test_queue_head);
				test_queue_head = (test_queue_head + 1) % MAX_QUEUED_TESTS;
			}

			if (ringbuf_u8_space_available(&test_queue_index) > 0) {
				test_params_load(&test_queue[test_queue_head], test_ble_version, test_case);
				test_queue[test_queue_head].transfer_data_size = 10 * 1000;
				test_queue[test_queue_head].conn_interval = conn_interval;
				test_queue[test_queue_head].conn_evt_len_ext_enabled = 1;
//				test_params_print(&test_queue[test_queue_head]);
				ringbuf_u8_push(&test_queue_index, test_queue_head);
				test_queue_head = (test_queue_head + 1) % MAX_QUEUED_TESTS;
			}
			if (ringbuf_u8_space_available(&test_queue_index) > 0) {
				test_params_load(&test_queue[test_queue_head], test_ble_version, test_case);
				test_queue[test_queue_head].transfer_data_size = 100 * 1000;
				test_queue[test_queue_head].conn_interval = conn_interval;
				test_queue[test_queue_head].conn_evt_len_ext_enabled = 1;
//				test_params_print(&test_queue[test_queue_head]);
				ringbuf_u8_push(&test_queue_index, test_queue_head);
				test_queue_head = (test_queue_head + 1) % MAX_QUEUED_TESTS;
			}
		}
		break;
	case BSP_EVENT_KEY_2:
		if (central_core_flags.test_running == 0 && central_core_flags.connected == 1) {
			conn_interval = 1000.0f;
			if (ringbuf_u8_space_available(&test_queue_index) > 0) {
				test_params_load(&test_queue[test_queue_head], test_ble_version, test_case);
				test_queue[test_queue_head].transfer_data_size = 100;
				test_queue[test_queue_head].conn_interval = conn_interval;
				test_queue[test_queue_head].conn_evt_len_ext_enabled = 1;
//				test_params_print(&test_queue[test_queue_head]);
				ringbuf_u8_push(&test_queue_index, test_queue_head);
				test_queue_head = (test_queue_head + 1) % MAX_QUEUED_TESTS;
			}
			if (ringbuf_u8_space_available(&test_queue_index) > 0) {
				test_params_load(&test_queue[test_queue_head], test_ble_version, test_case);
				test_queue[test_queue_head].transfer_data_size = 400;
				test_queue[test_queue_head].conn_interval = conn_interval;
				test_queue[test_queue_head].conn_evt_len_ext_enabled = 1;
//				test_params_print(&test_queue[test_queue_head]);
				ringbuf_u8_push(&test_queue_index, test_queue_head);
				test_queue_head = (test_queue_head + 1) % MAX_QUEUED_TESTS;
			}
			if (ringbuf_u8_space_available(&test_queue_index) > 0) {
				test_params_load(&test_queue[test_queue_head], test_ble_version, test_case);
				test_queue[test_queue_head].transfer_data_size = 1000;
				test_queue[test_queue_head].conn_interval = conn_interval;
				test_queue[test_queue_head].conn_evt_len_ext_enabled = 1;
//				test_params_print(&test_queue[test_queue_head]);
				ringbuf_u8_push(&test_queue_index, test_queue_head);
				test_queue_head = (test_queue_head + 1) % MAX_QUEUED_TESTS;
			}

			if (ringbuf_u8_space_available(&test_queue_index) > 0) {
				test_params_load(&test_queue[test_queue_head], test_ble_version, test_case);
				test_queue[test_queue_head].transfer_data_size = 10 * 1000;
				test_queue[test_queue_head].conn_interval = conn_interval;
				test_queue[test_queue_head].conn_evt_len_ext_enabled = 1;
//				test_params_print(&test_queue[test_queue_head]);
				ringbuf_u8_push(&test_queue_index, test_queue_head);
				test_queue_head = (test_queue_head + 1) % MAX_QUEUED_TESTS;
			}
			if (ringbuf_u8_space_available(&test_queue_index) > 0) {
				test_params_load(&test_queue[test_queue_head], test_ble_version, test_case);
				test_queue[test_queue_head].transfer_data_size = 100 * 1000;
				test_queue[test_queue_head].conn_interval = conn_interval;
				test_queue[test_queue_head].conn_evt_len_ext_enabled = 1;
//				test_params_print(&test_queue[test_queue_head]);
				ringbuf_u8_push(&test_queue_index, test_queue_head);
				test_queue_head = (test_queue_head + 1) % MAX_QUEUED_TESTS;
			}
		}
		break;
	case BSP_EVENT_KEY_3:
		if (central_core_flags.test_running == 0 && central_core_flags.connected == 1) {
			conn_interval = 4000.0f;
			if (ringbuf_u8_space_available(&test_queue_index) > 0) {
				test_params_load(&test_queue[test_queue_head], test_ble_version, test_case);
				test_queue[test_queue_head].transfer_data_size = 100;
				test_queue[test_queue_head].conn_interval = conn_interval;
				test_queue[test_queue_head].conn_evt_len_ext_enabled = 1;
//				test_params_print(&test_queue[test_queue_head]);
				ringbuf_u8_push(&test_queue_index, test_queue_head);
				test_queue_head = (test_queue_head + 1) % MAX_QUEUED_TESTS;
			}
			if (ringbuf_u8_space_available(&test_queue_index) > 0) {
				test_params_load(&test_queue[test_queue_head], test_ble_version, test_case);
				test_queue[test_queue_head].transfer_data_size = 400;
				test_queue[test_queue_head].conn_interval = conn_interval;
				test_queue[test_queue_head].conn_evt_len_ext_enabled = 1;
//				test_params_print(&test_queue[test_queue_head]);
				ringbuf_u8_push(&test_queue_index, test_queue_head);
				test_queue_head = (test_queue_head + 1) % MAX_QUEUED_TESTS;
			}
			if (ringbuf_u8_space_available(&test_queue_index) > 0) {
				test_params_load(&test_queue[test_queue_head], test_ble_version, test_case);
				test_queue[test_queue_head].transfer_data_size = 1000;
				test_queue[test_queue_head].conn_interval = conn_interval;
				test_queue[test_queue_head].conn_evt_len_ext_enabled = 1;
//				test_params_print(&test_queue[test_queue_head]);
				ringbuf_u8_push(&test_queue_index, test_queue_head);
				test_queue_head = (test_queue_head + 1) % MAX_QUEUED_TESTS;
			}

			if (ringbuf_u8_space_available(&test_queue_index) > 0) {
				test_params_load(&test_queue[test_queue_head], test_ble_version, test_case);
				test_queue[test_queue_head].transfer_data_size = 10 * 1000;
				test_queue[test_queue_head].conn_interval = conn_interval;
				test_queue[test_queue_head].conn_evt_len_ext_enabled = 1;
//				test_params_print(&test_queue[test_queue_head]);
				ringbuf_u8_push(&test_queue_index, test_queue_head);
				test_queue_head = (test_queue_head + 1) % MAX_QUEUED_TESTS;
			}
			if (ringbuf_u8_space_available(&test_queue_index) > 0) {
				test_params_load(&test_queue[test_queue_head], test_ble_version, test_case);
				test_queue[test_queue_head].transfer_data_size = 100 * 1000;
				test_queue[test_queue_head].conn_interval = conn_interval;
				test_queue[test_queue_head].conn_evt_len_ext_enabled = 1;
//				test_params_print(&test_queue[test_queue_head]);
				ringbuf_u8_push(&test_queue_index, test_queue_head);
				test_queue_head = (test_queue_head + 1) % MAX_QUEUED_TESTS;
			}
		}
		break;
	default:
		break;
	}
	*/


	/*
	switch(evt) {
	case BSP_EVENT_KEY_0:
		if (central_core_flags.test_running == 0 && central_core_flags.connected == 1) {
			debug_line("Queueing BLE 4.1 tests");
			if (ringbuf_u8_space_available(&test_queue_index) > 0) {
				test_params_load(&test_queue[test_queue_head], BLE_4_1, TEST_BLE_WRITE);
				test_queue[test_queue_head].transfer_data_size = 10 * 1024;
				debug_line("test_queue_head %d", test_queue_head);
				test_params_print(&test_queue[test_queue_head]);
				ringbuf_u8_push(&test_queue_index, test_queue_head);
				test_queue_head = (test_queue_head + 1) % MAX_QUEUED_TESTS;
			}
			if (ringbuf_u8_space_available(&test_queue_index) > 0) {
				test_params_load(&test_queue[test_queue_head], BLE_4_1, TEST_BLE_READ);
				test_queue[test_queue_head].transfer_data_size = 10 * 1024;
				debug_line("test_queue_head %d", test_queue_head);
				test_params_print(&test_queue[test_queue_head]);
				ringbuf_u8_push(&test_queue_index, test_queue_head);
				test_queue_head = (test_queue_head + 1) % MAX_QUEUED_TESTS;
			}
			if (ringbuf_u8_space_available(&test_queue_index) > 0) {
				test_params_load(&test_queue[test_queue_head], BLE_4_1, TEST_BLE_NOTIFY);
				test_queue[test_queue_head].transfer_data_size = 10 * 1024;
				debug_line("test_queue_head %d", test_queue_head);
				test_params_print(&test_queue[test_queue_head]);
				ringbuf_u8_push(&test_queue_index, test_queue_head);
				test_queue_head = (test_queue_head + 1) % MAX_QUEUED_TESTS;
			}
		}
		break;
	case BSP_EVENT_KEY_1:
		if (central_core_flags.test_running == 0 && central_core_flags.connected == 1) {
			debug_line("Queueing BLE 4.2 tests");
			if (ringbuf_u8_space_available(&test_queue_index) > 0) {
				test_params_load(&test_queue[test_queue_head], BLE_4_2, TEST_BLE_WRITE);
				test_queue[test_queue_head].transfer_data_size = 10 * 1024;
				ringbuf_u8_push(&test_queue_index, test_queue_head);
//				debug_line("test_queue_head %d", test_queue_head);
				test_queue_head = (test_queue_head + 1) % MAX_QUEUED_TESTS;
			}
			if (ringbuf_u8_space_available(&test_queue_index) > 0) {
				test_params_load(&test_queue[test_queue_head], BLE_4_2, TEST_BLE_READ);
				test_queue[test_queue_head].transfer_data_size = 10 * 1024;
				ringbuf_u8_push(&test_queue_index, test_queue_head);
//				debug_line("test_queue_head %d", test_queue_head);
				test_queue_head = (test_queue_head + 1) % MAX_QUEUED_TESTS;
			}
			if (ringbuf_u8_space_available(&test_queue_index) > 0) {
				test_params_load(&test_queue[test_queue_head], BLE_4_2, TEST_BLE_NOTIFY);
				test_queue[test_queue_head].transfer_data_size = 100 * 1024;
				ringbuf_u8_push(&test_queue_index, test_queue_head);
//				debug_line("test_queue_head %d", test_queue_head);
				test_queue_head = (test_queue_head + 1) % MAX_QUEUED_TESTS;
			}
		}
		break;
	case BSP_EVENT_KEY_2:
		if (central_core_flags.test_running == 0 && central_core_flags.connected == 1) {
			debug_line("Queueing BLE 5.0 HS tests");
			if (ringbuf_u8_space_available(&test_queue_index) > 0) {
				test_params_load(&test_queue[test_queue_head], BLE_5_HS, TEST_BLE_WRITE_NO_RSP);
				test_queue[test_queue_head].transfer_data_size = 100 * 1024;
				debug_line("test_queue_head %d", test_queue_head);
				ringbuf_u8_push(&test_queue_index, test_queue_head);
				test_queue_head = (test_queue_head + 1) % MAX_QUEUED_TESTS;
			}
			if (ringbuf_u8_space_available(&test_queue_index) > 0) {
				test_params_load(&test_queue[test_queue_head], BLE_5_HS, TEST_BLE_WRITE);
				test_queue[test_queue_head].transfer_data_size = 10 * 1024;
				debug_line("test_queue_head %d", test_queue_head);
				ringbuf_u8_push(&test_queue_index, test_queue_head);
				test_queue_head = (test_queue_head + 1) % MAX_QUEUED_TESTS;
			}
			if (ringbuf_u8_space_available(&test_queue_index) > 0) {
				test_params_load(&test_queue[test_queue_head], BLE_5_HS, TEST_BLE_READ);
				test_queue[test_queue_head].transfer_data_size = 10 * 1024;
				debug_line("test_queue_head %d", test_queue_head);
				ringbuf_u8_push(&test_queue_index, test_queue_head);
				test_queue_head = (test_queue_head + 1) % MAX_QUEUED_TESTS;
			}
			if (ringbuf_u8_space_available(&test_queue_index) > 0) {
				test_params_load(&test_queue[test_queue_head], BLE_5_HS, TEST_BLE_NOTIFY);
				test_queue[test_queue_head].transfer_data_size = 100 * 1024;
				debug_line("test_queue_head %d", test_queue_head);
				ringbuf_u8_push(&test_queue_index, test_queue_head);
				test_queue_head = (test_queue_head + 1) % MAX_QUEUED_TESTS;
			}
		}
		break;
	case BSP_EVENT_KEY_3:
		inject_state(CENTRAL_CORE_TEST_TERMINATE);
		break;
	default:
		break;
	}
	*/
}


static central_core_state_t get_next_state() {
	if (ringbuf_u16_get_length(&state_core_next) > 0) {
		return ringbuf_u16_pop(&state_core_next);
	} else {
		return CENTRAL_CORE_STATE_IDLE;
	}
}

static void queue_state(central_core_state_t next_state) {
	ringbuf_u16_push(&state_core_next, next_state);
}

static void inject_state(central_core_state_t next_state) {
	ringbuf_u16_push_first(&state_core_next, next_state);
}


// Helper functions ---------------------------------------------------------------------------

static void timers_init() {
	ret_code_t err_code = app_timer_init();
	APP_ERROR_CHECK(err_code);
}
