/*
 * ble_stack.c
 *
 *  Created on: Jan 15, 2018
 *      Author: gaspe
 */

#include "stdint.h"
#include "ble_stack.h"

#include "app_error.h"
#include "app_timer.h"
#include "ble.h"
#include "ble_dfu.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_db_discovery.h"
#include "softdevice_handler.h"
#include "fstorage.h"
#include "fds.h"
#include "peer_manager.h"
#include "ble_conn_state.h"
#include "nrf_ble_gatt.h"

#include "central_ble.h"


#include "debug.h"

#ifdef DEBUG
#undef DEBUG
#endif

#define DEBUG	1
#define debug_line(...)  do { if (DEBUG>0) { debug_line_global(__VA_ARGS__); debug_global("\n"); }} while (0)
#define debug_error(...)  do { if (DEBUG>0) { debug_errorline_global(__VA_ARGS__); debug_global("\n"); }} while (0)
#define debug_L2(...)  do { if (DEBUG>1) { debug_line_global(__VA_ARGS__); debug_global("\n"); }} while (0)
#define debug_data(...)  do { if (DEBUG>0) { debug_global(__VA_ARGS__); }} while (0)

// Low frequency clock source to be used by the SoftDevice
#define NRF_CLOCK_LFCLKSRC      {.source        = NRF_CLOCK_LF_SRC_XTAL,            \
                                 .rc_ctiv       = 0,                                \
                                 .rc_temp_ctiv  = 0,                                \
                                 .xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM}


static char * roles_str[] = {
    "INVALID_ROLE",
    "CENTRAL",
    "PERIPHERAL",
};

static uint8_t max_data_length = BLE_GATT_ATT_MTU_DEFAULT - OPCODE_LENGTH - HANDLE_LENGTH;

/**@brief Variable length data encapsulation in terms of length and pointer to data.
 */
typedef struct {
	uint8_t  * p_data;    /**< Pointer to data. */
	uint16_t   data_len;  /**< Length of data. */
} data_t;

typedef struct {
    bool           is_connected;
    ble_gap_addr_t address;
} conn_peer_t;

/** @brief Parameters used when scanning.
 */
static const ble_gap_scan_params_t m_scan_params = {
    .active   = 1,
    .interval = SCAN_INTERVAL,
    .window   = SCAN_WINDOW,
    .timeout  = SCAN_TIMEOUT,
	.use_whitelist = 0,
};

/**@brief Connection parameters requested for connection.
 */
ble_gap_conn_params_t m_connection_param = {
	.min_conn_interval	= CONN_INTERVAL_MIN,
	.max_conn_interval	= CONN_INTERVAL_MAX,
	.slave_latency		= SLAVE_LATENCY,
	.conn_sup_timeout	= CONN_SUP_TIMEOUT
};

static uint16_t				m_conn_handle_central = BLE_CONN_HANDLE_INVALID;				/**< Handle of the current central connection. */
static uint16_t				m_conn_handle_peripheral = BLE_CONN_HANDLE_INVALID;			/**< Handle of the current peripheral connection. */
static ble_dfu_t				m_dfus;														/**< Structure used to identify the DFU service. */
nrf_ble_gatt_t		m_gatt;														/**< GATT module instance. */
static ble_db_discovery_t	m_ble_db_discovery[NRF_BLE_LINK_COUNT];						/**< DB structures used by the database discovery module. */

static conn_peer_t			m_connected_peers[NRF_BLE_LINK_COUNT];

static const uint8_t m_target_periph_addr[BLE_GAP_ADDR_LEN] = {0};	/**< Address of the device the central will try to connect to. */
static const char m_target_periph_name[] = "TestPeripheral";							/**< Name of the device the central will try to connect to. */

static pm_peer_id_t	m_whitelist_peers[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];		/**< List of peers currently in the whitelist. */
static uint32_t		m_whitelist_peer_cnt;									/**< Number of peers currently in the whitelist. */
static bool			m_whitelist_changed;										/**< Indicates if the whitelist has been changed since last time it has been updated in the Peer Manager. */


// Helper functions ---------------------------------------------------------------------------

/**
 * @brief Parses advertisement data, providing length and location of the field in case
 *        matching data is found.
 *
 * @param[in]  Type of data to be looked for in advertisement data.
 * @param[in]  Advertisement report length and pointer to report.
 * @param[out] If data type requested is found in the data report, type data length and
 *             pointer to data will be populated here.
 *
 * @retval NRF_SUCCESS if the data type is found in the report.
 * @retval NRF_ERROR_NOT_FOUND if the data type could not be found.
 */
static uint32_t adv_report_parse(uint8_t type, data_t * p_advdata, data_t * p_typedata) {
	uint32_t   index = 0;
	uint8_t  * p_data;

	p_data = p_advdata->p_data;

	while (index < p_advdata->data_len) {
		uint8_t field_length = p_data[index];
		uint8_t field_type   = p_data[index + 1];

		if (field_type == type) {
			p_typedata->p_data   = &p_data[index + 2];
			p_typedata->data_len = field_length - 1;
			return NRF_SUCCESS;
		}
		index += field_length + 1;
	}
	return NRF_ERROR_NOT_FOUND;
}

/**@brief Function for searching a given name in the advertisement packets.
 *
 * @details Use this function to parse received advertising data and to find a given
 * name in them either as 'complete_local_name' or as 'short_local_name'.
 *
 * @param[in]   p_adv_report   advertising data to parse.
 * @param[in]   name_to_find   name to search.
 * @return   true if the given name was found, false otherwise.
 */
static bool find_adv_name(ble_gap_evt_adv_report_t const * p_adv_report, char const * name_to_find) {
	ret_code_t err_code;
	data_t     adv_data;
	data_t     dev_name;

	// Initialize advertisement report for parsing
	adv_data.p_data   = (uint8_t *)p_adv_report->data;
	adv_data.data_len = p_adv_report->dlen;

	// Search for advertising names
	err_code = adv_report_parse(BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME, &adv_data, &dev_name);
	if (err_code == NRF_SUCCESS) {
		debug_L2("Found complete name: %s", dev_name.p_data);
		if (memcmp(name_to_find, dev_name.p_data, strlen(name_to_find)) == 0) {
			return true;
		}
	} else {
		// Look for the short local name if it was not found as complete
		err_code = adv_report_parse(BLE_GAP_AD_TYPE_SHORT_LOCAL_NAME, &adv_data, &dev_name);
		debug_L2("Found short name: %s", dev_name.p_data);
		if (err_code != NRF_SUCCESS) {
			return false;
		}
		if (memcmp(name_to_find, dev_name.p_data, strlen(name_to_find)) == 0) {
			return true;
		}
	}
	return false;
}

/**@brief Function for searching a UUID in the advertisement packets.
 *
 * @details Use this function to parse received advertising data and to find a given
 * UUID in them.
 *
 * @param[in]   p_adv_report   advertising data to parse.
 * @param[in]   uuid_to_find   UUIID to search.
 * @return   true if the given UUID was found, false otherwise.
 */
static bool find_adv_uuid(ble_gap_evt_adv_report_t const * p_adv_report, uint16_t uuid_to_find) {
	ret_code_t err_code;
	data_t     adv_data;
	data_t     type_data;

	// Initialize advertisement report for parsing.
	adv_data.p_data   = (uint8_t *)p_adv_report->data;
	adv_data.data_len = p_adv_report->dlen;

	err_code = adv_report_parse(BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_MORE_AVAILABLE,
								&adv_data,
								&type_data);

	if (err_code != NRF_SUCCESS) {
		// Look for the services in 'complete' if it was not found in 'more available'.
		err_code = adv_report_parse(BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_COMPLETE,
									&adv_data,
									&type_data);

		if (err_code != NRF_SUCCESS) {
			// If we can't parse the data, then exit.
			return false;
		}
	}

	// Verify if any UUID match the given UUID.
	for (uint32_t i = 0; i < (type_data.data_len / sizeof(uint16_t)); i++) {
		uint16_t extracted_uuid = uint16_decode(&type_data.p_data[i * sizeof(uint16_t)]);
		if (extracted_uuid == uuid_to_find) {
			return true;
		}
	}
	return false;
}

/**@brief Function for checking if a link already exists with a new connected peer.
 *
 * @details This function checks if a newly connected device is already connected
 *
 * @param[in]   p_connected_evt Bluetooth connected event.
 * @return                      True if the peer's address is found in the list of connected peers,
 *                              false otherwise.
 */
static bool is_already_connected(ble_gap_addr_t const * p_connected_adr) {
	for (uint32_t i = 0; i < NRF_BLE_LINK_COUNT; i++) {
		if (m_connected_peers[i].is_connected) {
			if (m_connected_peers[i].address.addr_type == p_connected_adr->addr_type) {
				if (memcmp(m_connected_peers[i].address.addr,
				p_connected_adr->addr,
				sizeof(m_connected_peers[i].address.addr)) == 0) {
					return true;
				}
			}
		}
	}
	return false;
}

/**@brief Clear bond information from persistent storage.
 */
static void delete_bonds(void) {
	ret_code_t err_code;
	err_code = pm_peers_delete();
	APP_ERROR_CHECK(err_code);
	debug_line("Bonds erased");
}

/**@brief Fetch the list of peer manager peer IDs.
 *
 * @param[inout] p_peers   The buffer where to store the list of peer IDs.
 * @param[inout] p_size    In: The size of the @p p_peers buffer.
 *                         Out: The number of peers copied in the buffer.
 */
static void peer_list_get(pm_peer_id_t * p_peers, uint32_t * p_size) {
	pm_peer_id_t peer_id;
	uint32_t     peers_to_copy;

	peers_to_copy = (*p_size < BLE_GAP_WHITELIST_ADDR_MAX_COUNT) ? *p_size : BLE_GAP_WHITELIST_ADDR_MAX_COUNT;

	peer_id = pm_next_peer_id_get(PM_PEER_ID_INVALID);
	*p_size = 0;

	while ((peer_id != PM_PEER_ID_INVALID) && (peers_to_copy--)) {
		p_peers[(*p_size)++] = peer_id;
		peer_id = pm_next_peer_id_get(peer_id);
	}
}

// End of helper functions --------------------------------------------------------------------
// Event handlers -----------------------------------------------------------------------------

static void ble_dfu_evt_handler(ble_dfu_t * p_dfu, ble_dfu_evt_t * p_evt) {
	switch (p_evt->type) {
		case BLE_DFU_EVT_INDICATION_DISABLED:
			debug_line("Indication for BLE DFU is disabled");
			break;
		case BLE_DFU_EVT_INDICATION_ENABLED:
			debug_line("Indication for BLE DFU is enabled");
			break;
		case BLE_DFU_EVT_ENTERING_BOOTLOADER:
			debug_line("Device is requested to enter bootloader mode");
			break;
		default:
			debug_error("Unknown event from BLE DFU");
			break;
	}
}

/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt) {
	ret_code_t err_code;

	switch (p_evt->evt_id) {
		case PM_EVT_BONDED_PEER_CONNECTED:
			debug_line("Connected to a previously bonded device");
			break;

		case PM_EVT_CONN_SEC_SUCCEEDED:
			debug_line("Connection secured");

			if (p_evt->params.conn_sec_succeeded.procedure == PM_LINK_SECURED_PROCEDURE_BONDING) {
				if (m_whitelist_peer_cnt < BLE_GAP_WHITELIST_ADDR_MAX_COUNT) {
					debug_line("New bond - Adding peer to whitelist");

					// Bonded to a new peer, add it to the whitelist
					m_whitelist_peers[m_whitelist_peer_cnt++] = p_evt->peer_id;
					m_whitelist_changed = true;
				}
			}

			break;

		case PM_EVT_CONN_SEC_FAILED:
			/* Often, when securing fails, it shouldn't be restarted, for security reasons.
			* Other times, it can be restarted directly.
			* Sometimes it can be restarted, but only after changing some Security Parameters.
			* Sometimes, it cannot be restarted until the link is disconnected and reconnected.
			* Sometimes it is impossible, to secure the link, or the peer device does not support it.
			* How to handle this error is highly application dependent. */
			debug_error("Secure connection failed");
			break;

		case PM_EVT_CONN_SEC_CONFIG_REQ:; // Intentional empty statement
			// Reject pairing request from an already bonded peer.
			pm_conn_sec_config_t conn_sec_config = { .allow_repairing = false };
			pm_conn_sec_config_reply(p_evt->conn_handle, &conn_sec_config);
			break;

		case PM_EVT_STORAGE_FULL:
			// Run garbage collection on the flash.
			err_code = fds_gc();

			if (err_code == FDS_ERR_BUSY || err_code == FDS_ERR_NO_SPACE_IN_QUEUES) {
				// Retry.
			} else {
				APP_ERROR_CHECK(err_code);
			}
			break;

		case PM_EVT_PEERS_DELETE_SUCCEEDED:
			advertising_start(BLE_ADV_MODE_FAST, false);
			break;

		case PM_EVT_LOCAL_DB_CACHE_APPLY_FAILED:
			// The local database has likely changed, send service changed indications.
			pm_local_database_has_changed();
			break;

		case PM_EVT_PEER_DATA_UPDATE_FAILED:
			// Assert.
			APP_ERROR_CHECK(p_evt->params.peer_data_update_failed.error);
			break;

		case PM_EVT_PEER_DELETE_FAILED:
			// Assert.
			APP_ERROR_CHECK(p_evt->params.peer_delete_failed.error);
			break;

		case PM_EVT_PEERS_DELETE_FAILED:
			// Assert.
			APP_ERROR_CHECK(p_evt->params.peers_delete_failed_evt.error);
			break;

		case PM_EVT_ERROR_UNEXPECTED:
			// Assert.
			APP_ERROR_CHECK(p_evt->params.error_unexpected.error);
			break;

		case PM_EVT_CONN_SEC_START:
		case PM_EVT_PEER_DATA_UPDATE_SUCCEEDED:
		case PM_EVT_PEER_DELETE_SUCCEEDED:
		case PM_EVT_LOCAL_DB_CACHE_APPLIED:
		case PM_EVT_SERVICE_CHANGED_IND_SENT:
		case PM_EVT_SERVICE_CHANGED_IND_CONFIRMED:
		default:
			break;
	}
}

/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt) {
	ret_code_t err_code;

	if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED) {
		err_code = sd_ble_gap_disconnect(m_conn_handle_peripheral, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
		APP_ERROR_CHECK(err_code);
	}
}

/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt) {
	switch (ble_adv_evt) {
		case BLE_ADV_EVT_FAST:
			debug_line("Fast advertising\n");
			break;
		case BLE_ADV_EVT_FAST_WHITELIST:
			debug_line("Fast advertising with whitelist\n");
			break;
		case BLE_ADV_EVT_SLOW:
			debug_line("Slow advertising\n");
			break;
		case BLE_ADV_EVT_SLOW_WHITELIST:
			debug_line("Slow advertising with whitelist\n");
			break;
		case BLE_ADV_EVT_IDLE:
			debug_line("Advertising timed out");
			advertising_start(BLE_ADV_MODE_SLOW, false);
			break;
		case BLE_ADV_EVT_WHITELIST_REQUEST:
			debug_line("Requesting whitelist");
			ble_gap_addr_t whitelist_addrs[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];
			ble_gap_irk_t  whitelist_irks[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];
			uint32_t       addr_cnt = BLE_GAP_WHITELIST_ADDR_MAX_COUNT;
			uint32_t       irk_cnt  = BLE_GAP_WHITELIST_ADDR_MAX_COUNT;

			ret_code_t err_code = pm_whitelist_get(whitelist_addrs, &addr_cnt, whitelist_irks,  &irk_cnt);
			APP_ERROR_CHECK(err_code);

			debug_line("%d addresses and %d IRK in whitelist", addr_cnt, irk_cnt);

			// Apply the whitelist
			err_code = ble_advertising_whitelist_reply(whitelist_addrs, addr_cnt, whitelist_irks,  irk_cnt);
			APP_ERROR_CHECK(err_code);
			break;
		default:
			break;
	}
}

/**@brief Function for handling BLE Stack events common to both the central and peripheral roles.
 * @param[in] conn_handle Connection Handle.
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void on_ble_evt(uint16_t conn_handle, ble_evt_t const * p_ble_evt) {
	uint32_t err_code;
	uint16_t role = ble_conn_state_role(conn_handle);
    ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;

	switch (p_ble_evt->header.evt_id)
	{
		case BLE_GAP_EVT_CONNECTED:
			if (is_already_connected(&p_ble_evt->evt.gap_evt.params.connected.peer_addr)) {
				debug_line("%s: ALREADY connected, disconnecting", roles_str[role]);
				sd_ble_gap_disconnect(conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
			} else {
				m_connected_peers[conn_handle].is_connected = true;
				m_connected_peers[conn_handle].address = p_ble_evt->evt.gap_evt.params.connected.peer_addr;
			}
			break;
		case BLE_GAP_EVT_DISCONNECTED:
			m_connected_peers[conn_handle].is_connected = false;
			break;
		case BLE_GAP_EVT_PHY_UPDATE:
			debug_line("PHY updated: %d Mbps RX, %d Mbps TX - Status: %d",
							p_ble_evt->evt.gap_evt.params.phy_update.rx_phy,
							p_ble_evt->evt.gap_evt.params.phy_update.tx_phy,
							p_ble_evt->evt.gap_evt.params.phy_update.status);
			break;
        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:;
        {
            ble_gap_conn_params_t params;
            // Accept parameters requested by the peer.
            params = p_gap_evt->params.conn_param_update_request.conn_params;


        	debug_line("BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST : \n"
        			"\tmin: %d max: %d slave %d timeout %d",
					params.min_conn_interval,
					params.max_conn_interval,
					params.slave_latency,
					params.conn_sup_timeout);

            err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle, &m_connection_param);
            APP_ERROR_CHECK(err_code);

//            if (err_code != NRF_SUCCESS) {
//            	debug_error("Failed setting connection parameters: %d", retval);
//            }
        } break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE:;
            ble_gap_conn_params_t params;
            params = p_gap_evt->params.conn_param_update.conn_params;

        	debug_line("BLE_GAP_EVT_CONN_PARAM_UPDATE : \n"
        			"\tmin: %d max: %d slave %d timeout %d",
					params.min_conn_interval,
					params.max_conn_interval,
					params.slave_latency,
					params.conn_sup_timeout);
        	break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
        	debug_line("BLE_GATTS_EVT_SYS_ATTR_MISSING");
            err_code = sd_ble_gatts_sys_attr_set(p_gap_evt->conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
        case BLE_GATTS_EVT_TIMEOUT:
        	debug_line("GATT timeout, disconnecting.");
            err_code = sd_ble_gap_disconnect(conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_EVT_USER_MEM_REQUEST:
            err_code = sd_ble_user_mem_reply(p_ble_evt->evt.common_evt.conn_handle, NULL);
            APP_ERROR_CHECK(err_code);
            break;
		case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
			// Implement when needed
			break;
		case BLE_GAP_EVT_PASSKEY_DISPLAY:
			// Implement when needed
			break;
		case BLE_GAP_EVT_AUTH_KEY_REQUEST:
			// Implement when needed
			break;
		case BLE_GAP_EVT_LESC_DHKEY_REQUEST:
			// Implement when needed
			break;
		case BLE_GAP_EVT_AUTH_STATUS:
			// Implement when needed
			break;
		default:
			// No implementation needed.
			break;
	}
}

/**@brief Function for handling BLE Stack events concerning central applications.
 *
 * @details This function keeps the connection handles of central applications up-to-date. It
 * parses scanning reports, initiating a connection attempt to peripherals when a target UUID
 * is found, and manages connection parameter update requests. Additionally, it updates the status
 * of LEDs used to report central applications activity.
 *
 * @note        Since this function updates connection handles, @ref BLE_GAP_EVT_DISCONNECTED events
 *              should be dispatched to the target application before invoking this function.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void on_ble_central_evt(ble_evt_t const * p_ble_evt) {
	ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;
	ret_code_t            err_code;
	uint32_t timestamp = 0;

	switch (p_ble_evt->header.evt_id) {

		// Upon connection, check which peripheral has connected (HR or RSC), initiate DB
		// discovery and resume scanning if necessary.
		case BLE_GAP_EVT_CONNECTED:
			debug_line("CENTRAL: Connected");

		    // Stop scanning and advertising.
		    (void) sd_ble_gap_scan_stop();
		    (void) sd_ble_gap_adv_stop();

			// If there is no peer currently connected, try to find the scanner service on this peripheral.
			if (m_conn_handle_central == BLE_CONN_HANDLE_INVALID) {
				m_conn_handle_central = p_gap_evt->conn_handle;
				debug_line("CENTRAL: Searching for test service...", p_gap_evt->conn_handle);
				memset(&m_ble_db_discovery[p_gap_evt->conn_handle], 0, sizeof(ble_db_discovery_t));
				err_code = ble_db_discovery_start(&m_ble_db_discovery[p_gap_evt->conn_handle], p_gap_evt->conn_handle);
				APP_ERROR_CHECK(err_code);
			}
			break; // BLE_GAP_EVT_CONNECTED

		// Upon disconnection, reset the connection handle of the peer which disconnected
		// and start scanning again.
		case BLE_GAP_EVT_DISCONNECTED:
			debug_line("CENTRAL: Disconnected - Reason [0x%02X]\n", p_ble_evt->evt.gap_evt.params.disconnected.reason);
			if (p_gap_evt->conn_handle == m_conn_handle_central) {
				m_conn_handle_central = BLE_CONN_HANDLE_INVALID;
			}
			if (m_conn_handle_central == BLE_CONN_HANDLE_INVALID) {
				// Start scanning
				scan_start();
			}
			break; // BLE_GAP_EVT_DISCONNECTED

		case BLE_GAP_EVT_ADV_REPORT:
			if (is_already_connected(&p_gap_evt->params.adv_report.peer_addr)) {
				break;
			}

			bool do_connect = false;

			if (memcmp(m_target_periph_addr, p_gap_evt->params.connected.peer_addr.addr, BLE_GAP_ADDR_LEN) == 0) {
				debug_line("Found wanted BLE address");
				do_connect = true;
			}
			//TODO: Uncomment when whitelisting is implemented
			else if (strlen(m_target_periph_name) != 0) {
				if (find_adv_name(&p_gap_evt->params.adv_report, m_target_periph_name)) {
					do_connect = true;
				}
			} else {
				// We do not want to connect to two peripherals offering the same service, so when
				// a UUID is matched, we check that we are not already connected to a peer which
				// offers the same service.
				if (find_adv_uuid(&p_gap_evt->params.adv_report, BLE_UUID_GATT) &&
				(m_conn_handle_central == BLE_CONN_HANDLE_INVALID)) {
					do_connect = true;
				}
			}

			if (do_connect) {
				// Initiate connection.
				debug_line("CENTRAL: Connecting...");
				debug_L2("Peer address: %02X %02X %02X %02X %02X %02X",
						p_gap_evt->params.connected.peer_addr.addr[0],
						p_gap_evt->params.connected.peer_addr.addr[1],
						p_gap_evt->params.connected.peer_addr.addr[2],
						p_gap_evt->params.connected.peer_addr.addr[3],
						p_gap_evt->params.connected.peer_addr.addr[4],
						p_gap_evt->params.connected.peer_addr.addr[5]);
				err_code = sd_ble_gap_connect(&p_gap_evt->params.adv_report.peer_addr,
				&m_scan_params,
				&m_connection_param,
				APP_CONN_CFG_TAG);

				if (err_code != NRF_SUCCESS) {
					debug_error("Connection request failed with error code 0x%02X", err_code);
				}
			}
			break; // BLE_GAP_ADV_REPORT

		case BLE_GAP_EVT_TIMEOUT:
			// We have not specified a timeout for scanning, so only connection attempts can timeout.
			if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN) {
				debug_line("CENTRAL: Connection request timed out");
			}
			break; // BLE_GAP_EVT_TIMEOUT
//		no need for this here, as it's already implemented in on_ble_evt, which handles events common to both central and peripheral
//		case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
//			// Accept parameters requested by peer.
//			err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
//			&p_gap_evt->params.conn_param_update_request.conn_params);
//			APP_ERROR_CHECK(err_code);
//			break; // BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST

		case BLE_GATTC_EVT_TIMEOUT:
			// Disconnect on GATT Client timeout event.
			NRF_LOG_DEBUG("CENTRAL: GATT client timeout");
			err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
			BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
			APP_ERROR_CHECK(err_code);
			break; // BLE_GATTC_EVT_TIMEOUT

		case BLE_GATTS_EVT_TIMEOUT:
			// Disconnect on GATT Server timeout event.
			NRF_LOG_DEBUG("CENTRAL: GATT server timeout");
			err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
			BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
			APP_ERROR_CHECK(err_code);
			break; // BLE_GATTS_EVT_TIMEOUT

		default:
			// No implementation needed.
			break;
	}
}

/**@brief Function for handling BLE Stack events involving peripheral applications.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void on_ble_peripheral_evt(ble_evt_t * p_ble_evt) {
	ret_code_t err_code = NRF_SUCCESS;

	switch (p_ble_evt->header.evt_id) {
		case BLE_GAP_EVT_CONNECTED:
			debug_line("PERIPHERAL: Connected");
			APP_ERROR_CHECK(err_code);
			m_conn_handle_peripheral = p_ble_evt->evt.gap_evt.conn_handle;
			break; // BLE_GAP_EVT_CONNECTED

		case BLE_GAP_EVT_DISCONNECTED:
			debug_line("PERIPHERAL: Disconnected");
			switch (p_ble_evt->evt.gap_evt.params.disconnected.reason) {
				case BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION:
					debug_line("- REMOTE_USER_TERMINATED_CONNECTION");
					break;
				case BLE_HCI_LOCAL_HOST_TERMINATED_CONNECTION:
					debug_line("- LOCAL_HOST_TERMINATED_CONNECTION");
					break;
				case BLE_HCI_REMOTE_DEV_TERMINATION_DUE_TO_LOW_RESOURCES:
					debug_line("- REMOTE_DEV_TERMINATION_DUE_TO_LOW_RESOURCES");
					break;
				case BLE_HCI_REMOTE_DEV_TERMINATION_DUE_TO_POWER_OFF:
					debug_line("- REMOTE_DEV_TERMINATION_DUE_TO_POWER_OFF");
					break;
				default:
					debug_line("- UNKNOWN REASON [0x%02X]",p_ble_evt->evt.gap_evt.params.disconnected.reason);
					break;
			}
			m_conn_handle_peripheral = BLE_CONN_HANDLE_INVALID;

			if (m_whitelist_changed) {
				// The whitelist has been modified, update it in the Peer Manager
				err_code = pm_whitelist_set(m_whitelist_peers, m_whitelist_peer_cnt);
				APP_ERROR_CHECK(err_code);

				err_code = pm_device_identities_list_set(m_whitelist_peers, m_whitelist_peer_cnt);
				if (err_code != NRF_ERROR_NOT_SUPPORTED) {
					APP_ERROR_CHECK(err_code);
				}

				m_whitelist_changed = false;
			}

			advertising_start(BLE_ADV_MODE_FAST, false);
			break; // BLE_GAP_EVT_DISCONNECTED

		case BLE_GATTC_EVT_TIMEOUT:
			// Disconnect on GATT Client timeout event.
			debug_error("PERIPHERAL: GATT client timeout");
			err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
					 BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
			APP_ERROR_CHECK(err_code);
			break; // BLE_GATTC_EVT_TIMEOUT

		case BLE_GATTS_EVT_TIMEOUT:
			// Disconnect on GATT Server timeout event.
			debug_error("PERIPHERAL: GATT server timeout");
			err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
					 BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
			APP_ERROR_CHECK(err_code);
			break; // BLE_GATTS_EVT_TIMEOUT

		case BLE_EVT_USER_MEM_REQUEST:
			err_code = sd_ble_user_mem_reply(p_ble_evt->evt.gattc_evt.conn_handle, NULL);
			APP_ERROR_CHECK(err_code);
			break; // BLE_EVT_USER_MEM_REQUEST

		default:
			// No implementation needed.
			break;
	}
}

/**@brief Function for handling database discovery events.
 *
 * @details This function is callback function to handle events from the database discovery module.
 *          Depending on the UUIDs that are discovered, this function should forward the events
 *          to their respective services.
 *
 * @param[in] p_event  Pointer to the database discovery event.
 */
static void db_disc_handler(ble_db_discovery_evt_t * p_evt) {
	central_ble_on_db_disc_evt(p_evt);
}

/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in] sys_evt  System stack event.
 */
static void sys_evt_dispatch(uint32_t sys_evt) {
	// Dispatch the system event to the fstorage module, where it will be
	// dispatched to the Flash Data Storage (FDS) module.
	fs_sys_event_handler(sys_evt);

	// Dispatch to the Advertising module last, since it will check if there are any
	// pending flash operations in fstorage. Let fstorage process system events first,
	// so that it can report correctly to the Advertising module.
	ble_advertising_on_sys_evt(sys_evt);
}

/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the BLE Stack event interrupt handler after a BLE stack
 *          event has been received.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt) {
	uint16_t conn_handle;
	uint16_t role;

	debug_L2("BLE event: %d", p_ble_evt->header.evt_id);

	// The Connection state module has to be fed BLE events in order to function correctly
	// Remember to call ble_conn_state_on_ble_evt before calling any ble_conns_state_* functions.
	ble_conn_state_on_ble_evt(p_ble_evt);
	pm_on_ble_evt(p_ble_evt);

	// The connection handle should really be retrievable for any event type.
	conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
	role        = ble_conn_state_role(conn_handle);

	on_ble_evt(conn_handle, p_ble_evt);

	// Based on the role this device plays in the connection, dispatch to the right applications.
	// Forward advertising timeout events to the advertising module
	if ((role == BLE_GAP_ROLE_PERIPH) ||
		((p_ble_evt->header.evt_id == BLE_GAP_EVT_TIMEOUT) && (p_ble_evt->evt.gap_evt.params.timeout.src == BLE_GAP_TIMEOUT_SRC_ADVERTISING))) {
		on_ble_peripheral_evt(p_ble_evt);

		ble_advertising_on_ble_evt(p_ble_evt);
		ble_conn_params_on_ble_evt(p_ble_evt);

		// Dispatch to peripheral applications.
//		central_on_ble_evt(p_ble_evt);
	}
	else if ((role == BLE_GAP_ROLE_CENTRAL) || (p_ble_evt->header.evt_id == BLE_GAP_EVT_ADV_REPORT)) {
		// on_ble_central_evt() will update the connection handles, so we want to execute it
		// after dispatching to the central applications upon disconnection.
		if (p_ble_evt->header.evt_id != BLE_GAP_EVT_DISCONNECTED) {
			on_ble_central_evt(p_ble_evt);
		}

		if (conn_handle < NRF_BLE_LINK_COUNT) {
			ble_db_discovery_on_ble_evt(&m_ble_db_discovery[conn_handle], p_ble_evt);
		}

		// Dispatch to central applications.
		central_on_ble_evt(p_ble_evt);

		// If the peer disconnected, we update the connection handles last.
		if (p_ble_evt->header.evt_id == BLE_GAP_EVT_DISCONNECTED) {
			on_ble_central_evt(p_ble_evt);
		}
	}
	nrf_ble_gatt_on_ble_evt(&m_gatt, p_ble_evt);
}

/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error) {
	APP_ERROR_HANDLER(nrf_error);
}

/**@brief GATT module event handler.
 */
static void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt) {
	switch (p_evt->evt_id) {
	case NRF_BLE_GATT_EVT_ATT_MTU_UPDATED:
		debug_line("ATT MTU %d", p_evt->params.att_mtu_effective);
		ble_set_max_data_length(p_evt->params.att_mtu_effective);
		break;
	default:
		debug_line("GATT event %04x", p_evt->evt_id);
		break;
	}
}

// To update the maximum data length we can send
void ble_set_max_data_length(uint16_t new_att_mtu) {
	max_data_length = new_att_mtu - OPCODE_LENGTH - HANDLE_LENGTH;
	debug_line("set max data length to %d", max_data_length);
}


uint8_t ble_get_max_data_length() {
	return max_data_length;
}

uint32_t ble_stack_set_conn_param(ble_gap_conn_params_t *p_conn_params) {
	memcpy(&m_connection_param, p_conn_params, sizeof m_connection_param);
	if (m_conn_handle_central != BLE_CONN_HANDLE_INVALID) {
		return sd_ble_gap_conn_param_update(m_conn_handle_central, p_conn_params);
	} else if (m_conn_handle_peripheral != BLE_CONN_HANDLE_INVALID) {
		return sd_ble_gap_conn_param_update(m_conn_handle_peripheral, p_conn_params);
	} else {
		return NRF_ERROR_INVALID_STATE;
	}
//	return ble_conn_params_change_conn_params(p_conn_params);
}

uint32_t ble_stack_set_phy(uint8_t phy) {
	ble_gap_phys_t phys;
	phys.rx_phys = phy;
	phys.tx_phys = phy;

	if (m_conn_handle_central != BLE_CONN_HANDLE_INVALID) {
		return sd_ble_gap_phy_request(m_conn_handle_central, &phys);
	} else if (m_conn_handle_peripheral != BLE_CONN_HANDLE_INVALID) {
		return sd_ble_gap_phy_request(m_conn_handle_peripheral, &phys);
	} else {
		return NRF_ERROR_INVALID_STATE;
	}
}

uint32_t ble_stack_set_preferred_phy(uint32_t phy) {
    ble_opt_t opts =
    {
        .gap_opt.preferred_phys.tx_phys = phy,
        .gap_opt.preferred_phys.rx_phys = phy,
    };

    ret_code_t err_code = sd_ble_opt_set(BLE_GAP_OPT_PREFERRED_PHYS_SET, &opts);
    debug_line("Setting preferred phy (rxtx) to %u: 0x%x", phy, err_code);
    APP_ERROR_CHECK(err_code);
    return err_code;
}

// End of event handlers ----------------------------------------------------------------------
// Initializers -------------------------------------------------------------------------------

void ble_stack_init(void) {
	ret_code_t err_code;

	nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;

	// Initialize the SoftDevice handler module.
	SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);

	// Fetch the start address of the application RAM.
	uint32_t ram_start = 0;
	err_code = softdevice_app_ram_start_get(&ram_start);
	APP_ERROR_CHECK(err_code);

	// Overwrite some of the default configurations for the BLE stack.
	ble_cfg_t ble_cfg;

	memset(&ble_cfg, 0, sizeof(ble_cfg));
	ble_cfg.common_cfg.vs_uuid_cfg.vs_uuid_count = CUSTOM_UUID_COUNT;

	err_code = sd_ble_cfg_set(BLE_COMMON_CFG_VS_UUID, &ble_cfg, ram_start);
	APP_ERROR_CHECK(err_code);

	memset(&ble_cfg, 0, sizeof(ble_cfg));
	// TODO: change this value if you get the NRF_ERROR_NO_MEM
	// If you change the size, you need to change the RAM settings in the linker file
	ble_cfg.gatts_cfg.attr_tab_size.attr_tab_size = BLE_GATTS_ATTR_TAB_SIZE_DEFAULT + BLE_GATTS_ATTR_TAB_SIZE_MIN + 0xE00;
	err_code = sd_ble_cfg_set(BLE_GATTS_CFG_ATTR_TAB_SIZE, &ble_cfg, ram_start);
	APP_ERROR_CHECK(err_code);

	// Configure the maximum number of connections.
	memset(&ble_cfg, 0, sizeof(ble_cfg));
	ble_cfg.gap_cfg.role_count_cfg.periph_role_count  = NRF_BLE_PERIPHERAL_LINK_COUNT;
	ble_cfg.gap_cfg.role_count_cfg.central_role_count = NRF_BLE_CENTRAL_LINK_COUNT;
	ble_cfg.gap_cfg.role_count_cfg.central_sec_count  = NRF_BLE_CENTRAL_LINK_COUNT;
	err_code = sd_ble_cfg_set(BLE_GAP_CFG_ROLE_COUNT, &ble_cfg, ram_start);
	APP_ERROR_CHECK(err_code);

	// Configure the maximum ATT MTU.
	memset(&ble_cfg, 0x00, sizeof(ble_cfg));
	ble_cfg.conn_cfg.params.gatt_conn_cfg.att_mtu = NRF_BLE_GATT_MAX_MTU_SIZE;
	ble_cfg.conn_cfg.conn_cfg_tag                 = APP_CONN_CFG_TAG;
	err_code = sd_ble_cfg_set(BLE_CONN_CFG_GATT, &ble_cfg, ram_start);
	APP_ERROR_CHECK(err_code);

	// Configure the maximum event length.
	memset(&ble_cfg, 0, sizeof(ble_cfg));
	ble_cfg.conn_cfg.params.gap_conn_cfg.conn_count     = NRF_BLE_LINK_COUNT;
	ble_cfg.conn_cfg.params.gap_conn_cfg.event_length   = 3200;					// basically sets the event length, which is then capped by connection interval. Since we've got 1 link, we can give all the time to the singe link
	ble_cfg.conn_cfg.conn_cfg_tag                       = APP_CONN_CFG_TAG;
	err_code = sd_ble_cfg_set(BLE_CONN_CFG_GAP, &ble_cfg, ram_start);
	APP_ERROR_CHECK(err_code);

	// Enable BLE stack.
	err_code = softdevice_enable(&ram_start);
	APP_ERROR_CHECK(err_code);

	// Register with the SoftDevice handler module for BLE events.
	err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
	APP_ERROR_CHECK(err_code);

	// Register with the SoftDevice handler module for BLE events.
	err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
	APP_ERROR_CHECK(err_code);
}

void gatt_init(void) {
	ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
	APP_ERROR_CHECK(err_code);
}

void db_discovery_init(void) {
	ret_code_t err_code = ble_db_discovery_init(db_disc_handler);
	APP_ERROR_CHECK(err_code);
}

void gap_params_init(void) {
	ret_code_t              err_code;
	ble_gap_conn_sec_mode_t sec_mode;

	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

	err_code = sd_ble_gap_device_name_set(&sec_mode,
										  (const uint8_t *)DEVICE_NAME,
										  strlen(DEVICE_NAME));
	APP_ERROR_CHECK(err_code);

	// We don't need an appearance value
	//err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_);
	//APP_ERROR_CHECK(err_code);

	err_code = sd_ble_gap_ppcp_set(&m_connection_param);
	APP_ERROR_CHECK(err_code);
}

void conn_params_init(void) {
	ret_code_t             err_code;
	ble_conn_params_init_t cp_init;

	memset(&cp_init, 0, sizeof(cp_init));

	cp_init.p_conn_params                  = NULL;
	cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
	cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
	cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
	cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
	cp_init.disconnect_on_fail             = false;
	cp_init.evt_handler                    = on_conn_params_evt;
	cp_init.error_handler                  = conn_params_error_handler;

	err_code = ble_conn_params_init(&cp_init);
	APP_ERROR_CHECK(err_code);
}

void services_init(void) {
	uint32_t err_code;
	ble_dfu_init_t dfus_init;

	// Initialize the Device Firmware Update Service.
	memset(&dfus_init, 0, sizeof(dfus_init));

	dfus_init.evt_handler                               = ble_dfu_evt_handler;
	dfus_init.ctrl_point_security_req_write_perm        = SEC_SIGNED;
	dfus_init.ctrl_point_security_req_cccd_write_perm   = SEC_SIGNED;

//	err_code = ble_dfu_init(&m_dfus, &dfus_init);
	APP_ERROR_CHECK(err_code);

}

void peer_manager_init(void) {
	ble_gap_sec_params_t sec_param;
	ret_code_t           err_code;

	err_code = pm_init();
	APP_ERROR_CHECK(err_code);

	memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

	// Security parameters to be used for all security procedures.
	sec_param.bond           = SEC_PARAM_BOND;
	sec_param.mitm           = SEC_PARAM_MITM;
	sec_param.lesc           = SEC_PARAM_LESC;
	sec_param.keypress       = SEC_PARAM_KEYPRESS;
	sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
	sec_param.oob            = SEC_PARAM_OOB;
	sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
	sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
	sec_param.kdist_own.enc  = 1;
	sec_param.kdist_own.id   = 1;
	sec_param.kdist_peer.enc = 1;
	sec_param.kdist_peer.id  = 1;

	err_code = pm_sec_params_set(&sec_param);
	APP_ERROR_CHECK(err_code);

	err_code = pm_register(pm_evt_handler);
	APP_ERROR_CHECK(err_code);
}

void advertising_init() {
	ret_code_t             err_code;
	ble_advdata_t          advdata;
	ble_adv_modes_config_t options;

	// This is the device information service
	//ble_uuid_t m_adv_uuids[] = {{BLE_UUID_CAMERA_INFO_SERVICE, BLE_UUID_TYPE_VENDOR_BEGIN}};

	// Build advertising data struct to pass into @ref ble_advertising_init.
	memset(&advdata, 0, sizeof(advdata));

	advdata.name_type               = BLE_ADVDATA_FULL_NAME;
	advdata.include_appearance      = false;
	advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

	memset(&options, 0, sizeof(options));

	options.ble_adv_fast_enabled  = true;
	options.ble_adv_fast_interval = APP_FAST_ADV_INTERVAL;
	options.ble_adv_fast_timeout  = APP_FAST_ADV_TIMEOUT_IN_SECONDS;

	options.ble_adv_slow_enabled  = true;
	options.ble_adv_slow_interval = APP_SLOW_ADV_INTERVAL;
	options.ble_adv_slow_timeout  = APP_SLOW_ADV_TIMEOUT_IN_SECONDS;

	options.ble_adv_whitelist_enabled = false;

	ble_advdata_manuf_data_t manufacturer_data;

	uint8_t data_array[4];

	memset(&manufacturer_data, 0, sizeof(manufacturer_data));
	manufacturer_data.company_identifier = FOOLOGRAPHY_COMPANY_ID;
	manufacturer_data.data.size = 0;
	manufacturer_data.data.p_data = data_array;

	// Declare and instantiate the scan response
	ble_advdata_t srdata;
	memset(&srdata, 0, sizeof(srdata));
	srdata.uuids_complete.uuid_cnt = 1;
//	srdata.uuids_complete.p_uuids = &main_service_uuid;
	srdata.p_manuf_specific_data = &manufacturer_data;

	//Include scan response packet in advertising

	err_code = ble_advertising_init(&advdata, &srdata, &options, on_adv_evt, NULL);
	if (err_code != NRF_SUCCESS) {
		debug_error("Advertising initialisation failed with error code 0x%02X", err_code);
		APP_ERROR_CHECK(err_code);
	}

	ble_advertising_conn_cfg_tag_set(APP_CONN_CFG_TAG);

	debug_line("Advertising initialized");
}


// End of initializers ------------------------------------------------------------------------
// Advertising and scanning -------------------------------------------------------------------

void advertising_start(ble_adv_mode_t mode, bool erase_bonds) {
	if (erase_bonds == true) {
		delete_bonds();
		// Advertising is started by PM_EVT_PEERS_DELETED_SUCEEDED event
	} else {

		ret_code_t ret;

		memset(m_whitelist_peers, PM_PEER_ID_INVALID, sizeof(m_whitelist_peers));
		m_whitelist_peer_cnt = (sizeof(m_whitelist_peers) / sizeof(pm_peer_id_t));

		peer_list_get(m_whitelist_peers, &m_whitelist_peer_cnt);

		ret = pm_whitelist_set(m_whitelist_peers, m_whitelist_peer_cnt);
		APP_ERROR_CHECK(ret);

		// Setup the device identies list.
		// Some SoftDevices do not support this feature.
		ret = pm_device_identities_list_set(m_whitelist_peers, m_whitelist_peer_cnt);
		if (ret != NRF_ERROR_NOT_SUPPORTED) {
			APP_ERROR_CHECK(ret);
		}

		m_whitelist_changed = false;

		ret_code_t err_code = ble_advertising_start(mode);
		APP_ERROR_CHECK(err_code);
	}
}

void scan_start(void) {
	ret_code_t err_code;

	sd_ble_gap_scan_stop();

	err_code = sd_ble_gap_scan_start(&m_scan_params);

	// It is okay to ignore this error since we are stopping the scan anyway.
	if (err_code != NRF_ERROR_INVALID_STATE) {
		APP_ERROR_CHECK(err_code);
	}
	debug_line("Scanning started\n");
}

void scan_stop(void) {
	sd_ble_gap_scan_stop();
	debug_line("Scanning stopped\n");
}

void advertising_name_update(char * name) {
	ret_code_t              err_code;
	ble_gap_conn_sec_mode_t sec_mode;

	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

	err_code = sd_ble_gap_device_name_set(&sec_mode,
										  (const uint8_t *)name,
										  strlen(name));
	if (err_code != NRF_SUCCESS) {
		debug_error("Failed to set device name");
	}
}


// End of advertising and scanning ------------------------------------------------------------


