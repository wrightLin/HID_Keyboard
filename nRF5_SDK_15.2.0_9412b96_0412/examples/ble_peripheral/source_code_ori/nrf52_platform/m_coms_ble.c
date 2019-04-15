/**
 * Copyright (c) 2012 - 2018, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 *
 * @defgroup ble_sdk_app_hids_keyboard_main main.c
 * @{
 * @ingroup ble_sdk_app_hids_keyboard
 * @brief HID Keyboard Sample Application main file.
 *
 * This file contains is the source code for a sample application using the HID, Battery and Device
 * Information Services for implementing a simple keyboard functionality.
 * Pressing Button 0 will send text 'hello' to the connected peer. On receiving output report,
 * it toggles the state of LED 2 on the mother board based on whether or not Caps Lock is on.
 * This application uses the @ref app_scheduler.
 *
 * Also it would accept pairing requests from any peer device.
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_assert.h"
#include "app_error.h"
#include "ble.h"
#include "ble_err.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "m_coms_ble_adv.h"
#include "ble_advdata.h"
#include "ble_bas.h"
#include "ble_dis.h"
#include "ble_conn_params.h"
#include "sensorsim.h"
#include "bsp_btn_ble.h"

#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "app_timer.h"

#include "fds.h"
#include "ble_conn_state.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"
#include "peer_manager_handler.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

//wright : add for hid module
//#include "m_coms_ble_addr.h"
#include "m_coms_ble_adv.h"
#include "m_coms_ble_hid.h"


#include "m_coms_ble.h"

#include "project_params.h"





#define APP_BLE_OBSERVER_PRIO               3                                          /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG                1                                          /**< A tag identifying the SoftDevice BLE configuration. */

#define BATTERY_LEVEL_MEAS_INTERVAL         APP_TIMER_TICKS(2000)                      /**< Battery level measurement interval (ticks). */
#define MIN_BATTERY_LEVEL                   81                                         /**< Minimum simulated battery level. */
#define MAX_BATTERY_LEVEL                   100                                        /**< Maximum simulated battery level. */
#define BATTERY_LEVEL_INCREMENT             1                                          /**< Increment between each simulated battery level measurement. */


#define MAX_BUFFER_ENTRIES                  5                                          /**< Number of elements that can be enqueued */

#define DEAD_BEEF                           0xDEADBEEF                                 /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define SCHED_MAX_EVENT_DATA_SIZE           APP_TIMER_SCHED_EVENT_DATA_SIZE            /**< Maximum size of scheduler events. */


BLE_BAS_DEF(m_bas);                                                 /**< Structure used to identify the battery service. */
NRF_BLE_GATT_DEF(m_gatt);                                           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                 /**< Advertising module instance. */

static app_sched_event_handler_t     m_event_callback           = 0;
static bool                          m_in_boot_mode             = false;   /**< Current protocol mode. */

static uint16_t          m_conn_handle  = BLE_CONN_HANDLE_INVALID;  /**< Handle of the current connection. */
static pm_peer_id_t      m_peer_id;                                 /**< Device reference handle to the current bonded central. */


static void on_hids_evt(ble_hids_t * p_hids, ble_hids_evt_t * p_evt);


static m_coms_ble_evt_t coms_evt;

static bool is_adv_started = false;

/*wright : add dt2 need part */
#define QUARTER_PAGE_FLASH_ERASE_VALUES \
    0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF, \
    0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF, \
    0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF, \
    0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF, \

// Flash databases: These needs to be initialized to the flash erased state (0xFF) so they can be written once without erasing first
static const uint32_t s_ble_params_db[] = {QUARTER_PAGE_FLASH_ERASE_VALUES}; // 1/4 flash page
static const uint32_t s_hid_params_db[] = {QUARTER_PAGE_FLASH_ERASE_VALUES QUARTER_PAGE_FLASH_ERASE_VALUES 
                                           QUARTER_PAGE_FLASH_ERASE_VALUES QUARTER_PAGE_FLASH_ERASE_VALUES};

																					 // Static variables
static m_coms_ble_params_t *         s_ble_params_dt2     = 0; /** BLE parameters */
static app_sched_event_handler_t     s_event_callback = 0; /** Event callback used to notify m_coms */
static app_timer_id_t                s_encryption_timer;   /** Timer used to request encryption if Central doesnt enable it in a timely manner */
static app_timer_id_t                s_flash_access_timer; /** Timer used while waiting for flash access. */
static app_timer_id_t                s_notify_usage_timer;  /** Timer used to delay m_coms_ble_addr_notify_usage function */
static ble_gap_sec_params_t          s_sec_params;         /** Security requirements for this application. */
static ble_bas_t                     s_bas;                /** Structure used to identify the battery service. */
//static ble_dfu_t                     s_dfu;                /** Structure used to identify the dfu service. */
static uint8_t                       s_num_inp_reports;
static uint8_t                       s_hid_notif_enabled_count = 0;
static int8_t                        s_tx_buf_cnt;         /** Number of packets in SoftDevice TX buffer. Used to limit amount of buffered packets */
static bool                          s_encrypted;          /** Link encryption status */
static bool                          s_sec_params_requested; /** Auth key requested */
static bool                          s_boot_mode_active;   /** In Boot or Report mode? */
static ble_srv_report_ref_t          s_bas_report_ref;     /** Battery service report reference mapping to HID service */
static uint16_t                      s_conn_handle;        /** Handle value of current connection */
static m_coms_ble_boot_mode_callback s_boot_mode_callback; /** Callback used to handle Boot mode reports */
// 1 flash page
/*wright : add dt2 need part end*/

/**@brief Function for setting filtered whitelist.
 *
 * @param[in] skip  Filter passed to @ref pm_peer_id_list.
 */
static void whitelist_set(pm_peer_id_list_skip_t skip)
{
    pm_peer_id_t peer_ids[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];
    uint32_t     peer_id_count = BLE_GAP_WHITELIST_ADDR_MAX_COUNT;

    ret_code_t err_code = pm_peer_id_list(peer_ids, &peer_id_count, PM_PEER_ID_INVALID, skip);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("\tm_whitelist_peer_cnt %d, MAX_PEERS_WLIST %d",
                   peer_id_count + 1,
                   BLE_GAP_WHITELIST_ADDR_MAX_COUNT);

    err_code = pm_whitelist_set(peer_ids, peer_id_count);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for setting filtered device identities.
 *
 * @param[in] skip  Filter passed to @ref pm_peer_id_list.
 */
static void identities_set(pm_peer_id_list_skip_t skip)
{
    pm_peer_id_t peer_ids[BLE_GAP_DEVICE_IDENTITIES_MAX_COUNT];
    uint32_t     peer_id_count = BLE_GAP_DEVICE_IDENTITIES_MAX_COUNT;

    ret_code_t err_code = pm_peer_id_list(peer_ids, &peer_id_count, PM_PEER_ID_INVALID, skip);
    APP_ERROR_CHECK(err_code);

    err_code = pm_device_identities_list_set(peer_ids, peer_id_count);
    APP_ERROR_CHECK(err_code);
}


/**@brief Clear bond information from persistent storage.
 */
static void delete_bonds(void)
{
    ret_code_t err_code;

    NRF_LOG_INFO("Erase bonds!");

    err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
	      bool is_bonded_before = (pm_peer_count()!=0);
	
	ble_adv_mode_t adv_mode = (is_bonded_before)?BLE_ADV_MODE_DIRECTED_HIGH_DUTY:BLE_ADV_MODE_FAST;
			
	      if(is_bonded_before)  m_advertising.adv_modes_config.ble_adv_fast_timeout =300;
				     
        whitelist_set(PM_PEER_ID_LIST_SKIP_NO_ID_ADDR);
       
        ret_code_t ret = ble_advertising_start(&m_advertising, adv_mode);
        APP_ERROR_CHECK(ret);	
	
	      is_adv_started = true;
}


/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    pm_handler_on_pm_evt(p_evt);
    pm_handler_flash_clean(p_evt);

    switch (p_evt->evt_id)
    {
        
		    case PM_EVT_BONDED_PEER_CONNECTED:
					
			  case PM_EVT_CONN_SEC_SUCCEEDED:
             coms_evt.type = M_COMS_BLE_EVT_BONDED;
				     m_event_callback(&coms_evt,sizeof(coms_evt));
				     break;
			
			  case PM_EVT_PEERS_DELETE_SUCCEEDED:
             advertising_start();
             break;

        case PM_EVT_PEER_DATA_UPDATE_SUCCEEDED:
            if (     p_evt->params.peer_data_update_succeeded.flash_changed
                 && (p_evt->params.peer_data_update_succeeded.data_id == PM_PEER_DATA_ID_BONDING))
            {
                NRF_LOG_INFO("New Bond, add the peer to the whitelist if possible");
                // Note: You should check on what kind of white list policy your application should use.

                whitelist_set(PM_PEER_ID_LIST_SKIP_NO_ID_ADDR);
            }
            break;

				
        default:
            break;
    }
}


/**@brief Function for handling Service errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void service_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for handling advertising errors.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void ble_advertising_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}




/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)s_ble_params.device_info.device_name,
                                          strlen(s_ble_params.device_info.device_name));
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_appearance_set(s_ble_params.appearance);
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = s_ble_params.conn_params.min_conn_interval;
    gap_conn_params.max_conn_interval = s_ble_params.conn_params.max_conn_interval;
    gap_conn_params.slave_latency     = s_ble_params.conn_params.slave_latency;
    gap_conn_params.conn_sup_timeout  = s_ble_params.conn_params.conn_sup_timeout;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Queued Write Module.
 */
static void qwr_init(void)
{
    ret_code_t         err_code;
    nrf_ble_qwr_init_t qwr_init_obj = {0};

    qwr_init_obj.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init_obj);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing Device Information Service.
 */
static void dis_init(void)
{
    ret_code_t       err_code;
    ble_dis_init_t   dis_init_obj;
    ble_dis_pnp_id_t pnp_id;

    pnp_id.vendor_id_source = s_ble_params.device_info.pnp_vendor_id_src;
    pnp_id.vendor_id        = s_ble_params.device_info.pnp_vendor_id;
    pnp_id.product_id       = s_ble_params.device_info.pnp_product_id;
    pnp_id.product_version  = s_ble_params.device_info.pnp_product_version;

    memset(&dis_init_obj, 0, sizeof(dis_init_obj));

    ble_srv_ascii_to_utf8(&dis_init_obj.manufact_name_str, s_ble_params.device_info.manufacturer_name);
    dis_init_obj.p_pnp_id = &pnp_id;

    dis_init_obj.dis_char_rd_sec = SEC_JUST_WORKS;

    err_code = ble_dis_init(&dis_init_obj);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing Battery Service.
 */
static void bas_init(void)
{
    ret_code_t     err_code;
    ble_bas_init_t bas_init_obj;

    memset(&bas_init_obj, 0, sizeof(bas_init_obj));

    bas_init_obj.evt_handler          = NULL;
    bas_init_obj.support_notification = true;
    bas_init_obj.p_report_ref         = NULL;
    bas_init_obj.initial_batt_level   = 100;

    bas_init_obj.bl_rd_sec        = SEC_JUST_WORKS;
    bas_init_obj.bl_cccd_wr_sec   = SEC_JUST_WORKS;
    bas_init_obj.bl_report_rd_sec = SEC_JUST_WORKS;

    err_code = ble_bas_init(&m_bas, &bas_init_obj);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing HID Service.
 */
static void hids_init(void)
{
    ret_code_t                    err_code;
    ble_hids_init_t               hids_init_obj;
    


    uint8_t                       hid_info_flags;
	  uint8_t                      report_map[sizeof(s_hid_descriptor)];
	  uint8_t                      i;
  
	  memcpy(report_map, s_hid_descriptor, sizeof(s_hid_descriptor));

#if ( NUM_OF_INPUT_REPORTS != 0 )
	
  ble_hids_inp_rep_init_t     * p_input_report;
  static ble_hids_inp_rep_init_t     input_report_array[NUM_OF_INPUT_REPORTS];
  memset((void *)input_report_array, 0, sizeof(ble_hids_inp_rep_init_t));

	for (i = 0; i < NUM_OF_INPUT_REPORTS; i++)
	{
    // Initialize HID Service
    p_input_report                      = &input_report_array[i];
    p_input_report->max_len             = s_report_input_max_lens[i];
    p_input_report->rep_ref.report_id   = s_report_input_ref_id[i];
    p_input_report->rep_ref.report_type = BLE_HIDS_REP_TYPE_INPUT;

    p_input_report->sec.cccd_wr = SEC_JUST_WORKS;
    p_input_report->sec.wr      = SEC_JUST_WORKS;
    p_input_report->sec.rd      = SEC_JUST_WORKS;
	
  }
	
#endif
	
#if ( NUM_OF_OUTPUT_REPORTS != 0)
	
	ble_hids_outp_rep_init_t    * p_output_report;
	static ble_hids_outp_rep_init_t    output_report_array[NUM_OF_OUTPUT_REPORTS];
	memset((void *)output_report_array, 0, sizeof(ble_hids_outp_rep_init_t));
	
	for (i = 0; i < NUM_OF_OUTPUT_REPORTS; i++)
	{
    p_output_report                      = &output_report_array[i];
    p_output_report->max_len             = s_report_output_max_lens[i];
    p_output_report->rep_ref.report_id   = s_report_output_ref_id[i];
    p_output_report->rep_ref.report_type = BLE_HIDS_REP_TYPE_OUTPUT;

    p_output_report->sec.wr = SEC_JUST_WORKS;
    p_output_report->sec.rd = SEC_JUST_WORKS;
	}
	
#endif


#if ( NUM_OF_FEATURE_REPORTS != 0)
  
	ble_hids_feature_rep_init_t * p_feature_report;
	static ble_hids_feature_rep_init_t feature_report_array[NUM_OF_FEATURE_REPORTS];
	memset((void *)feature_report_array, 0, sizeof(ble_hids_feature_rep_init_t));
	
	for (i = 0; i < NUM_OF_FEATURE_REPORTS; i++)
	{
    p_feature_report                      = &feature_report_array[i];
    p_feature_report->max_len             = s_report_feature_max_lens[i];
    p_feature_report->rep_ref.report_id   = s_report_feature_ref_id[i];
    p_feature_report->rep_ref.report_type = BLE_HIDS_REP_TYPE_FEATURE;

    p_feature_report->sec.rd              = SEC_JUST_WORKS;
    p_feature_report->sec.wr              = SEC_JUST_WORKS;
	}
	
#endif
	
	
    hid_info_flags = HID_INFO_FLAG_REMOTE_WAKE_MSK | HID_INFO_FLAG_NORMALLY_CONNECTABLE_MSK;

    memset(&hids_init_obj, 0, sizeof(hids_init_obj));

    hids_init_obj.evt_handler                    = on_hids_evt;
    hids_init_obj.error_handler                  = service_error_handler;
    hids_init_obj.is_kb                          = true;
    hids_init_obj.is_mouse                       = false;
    hids_init_obj.inp_rep_count                  = NUM_OF_INPUT_REPORTS;
#if ( NUM_OF_INPUT_REPORTS != 0 )	
    hids_init_obj.p_inp_rep_array                = input_report_array;
#endif	
    hids_init_obj.outp_rep_count                 = NUM_OF_OUTPUT_REPORTS;
#if ( NUM_OF_OUTPUT_REPORTS != 0 )		
    hids_init_obj.p_outp_rep_array               = output_report_array;
#endif	
    hids_init_obj.feature_rep_count              = NUM_OF_FEATURE_REPORTS;
#if ( NUM_OF_FEATURE_REPORTS != 0 )		
    hids_init_obj.p_feature_rep_array            = feature_report_array;
#endif		
    hids_init_obj.rep_map.data_len               = sizeof(s_hid_descriptor);
    hids_init_obj.rep_map.p_data                 = report_map;
    hids_init_obj.hid_information.bcd_hid        = s_ble_params.base_hid_version;
    hids_init_obj.hid_information.b_country_code = 0;
    hids_init_obj.hid_information.flags          = hid_info_flags;
    hids_init_obj.included_services_count        = 0;
    hids_init_obj.p_included_services_array      = NULL;

    hids_init_obj.rep_map.rd_sec         = SEC_JUST_WORKS;
    hids_init_obj.hid_information.rd_sec = SEC_JUST_WORKS;

    hids_init_obj.boot_kb_inp_rep_sec.cccd_wr = SEC_JUST_WORKS;
    hids_init_obj.boot_kb_inp_rep_sec.rd      = SEC_JUST_WORKS;

    hids_init_obj.boot_kb_outp_rep_sec.rd = SEC_JUST_WORKS;
    hids_init_obj.boot_kb_outp_rep_sec.wr = SEC_JUST_WORKS;

    hids_init_obj.protocol_mode_rd_sec = SEC_JUST_WORKS;
    hids_init_obj.protocol_mode_wr_sec = SEC_JUST_WORKS;
    hids_init_obj.ctrl_point_wr_sec    = SEC_JUST_WORKS;

    err_code = ble_hids_init(&m_hids, &hids_init_obj);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    qwr_init();
    dis_init();	
    bas_init();
    hids_init();
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    ret_code_t             err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = s_ble_params.conn_params.first_conn_params_update_delay;
    cp_init.next_conn_params_update_delay  = s_ble_params.conn_params.next_conn_params_update_delay;
    cp_init.max_conn_params_update_count   = s_ble_params.conn_params.max_conn_params_update_count;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = NULL;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling HID events.
 *
 * @details This function will be called for all HID events which are passed to the application.
 *
 * @param[in]   p_hids  HID service structure.
 * @param[in]   p_evt   Event received from the HID service.
 */
static void on_hids_evt(ble_hids_t * p_hids, ble_hids_evt_t * p_evt)
{
#if (NUM_OF_OUTPUT_REPORTS !=0)	
	  uint8_t    report_val, report_index;
#endif	
								
    switch (p_evt->evt_type)
    {
        case BLE_HIDS_EVT_BOOT_MODE_ENTERED:
            m_in_boot_mode = true;
            break;

        case BLE_HIDS_EVT_REPORT_MODE_ENTERED:
            m_in_boot_mode = false;
            break;

        case BLE_HIDS_EVT_REP_CHAR_WRITE:
					
#if (NUM_OF_OUTPUT_REPORTS !=0)
				
				   report_index = p_evt->params.char_write.char_id.rep_index;
					
				   if (p_evt->params.char_write.char_id.rep_type == BLE_HIDS_REP_TYPE_OUTPUT  && 
						   report_index == OUTPUT_REPORT_INDEX )
					 {
						 
						 // This code assumes that the output report is one byte long. Hence the following
            // static assert is made.
            STATIC_ASSERT(OUTPUT_REPORT_MAX_LEN == 1);	
				
						ret_code_t err_code = ble_hids_outp_rep_get(&m_hids,
                                             report_index,
                                             OUTPUT_REPORT_MAX_LEN,
                                             0,
                                             m_conn_handle,
                                             &report_val);
						APP_ERROR_CHECK(err_code); 
            coms_evt.type               = M_COMS_BLE_EVT_DATA_RECEIVED;
						coms_evt.data_received.len  = OUTPUT_REPORT_MAX_LEN;
						*coms_evt.data_received.data = report_val;
						//memcpy(coms_evt.data_received.data, &report_val, OUTPUT_REPORT_MAX_LEN);            
				    m_event_callback(&coms_evt,sizeof(coms_evt));
					 } 	 
#endif					 
            break;

        case BLE_HIDS_EVT_NOTIF_ENABLED:
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    ret_code_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_DIRECTED_HIGH_DUTY:
             coms_evt.type = M_COMS_BLE_EVT_ADV_DIRECTED_HIGH_DUTY;
				     m_event_callback(&coms_evt, sizeof(coms_evt));
             break;

        case BLE_ADV_EVT_DIRECTED:
					   coms_evt.type = M_COMS_BLE_EVT_ADV_DIRECTED;
				     m_event_callback(&coms_evt, sizeof(coms_evt));
             break;

        case BLE_ADV_EVT_FAST:
					   coms_evt.type = M_COMS_BLE_EVT_ADV_FAST;
				     m_event_callback(&coms_evt, sizeof(coms_evt));
             break;

        case BLE_ADV_EVT_SLOW:
             coms_evt.type = M_COMS_BLE_EVT_ADV_SLOW;
				     m_event_callback(&coms_evt, sizeof(coms_evt));
            break;

        case BLE_ADV_EVT_FAST_WHITELIST:
             coms_evt.type = M_COMS_BLE_EVT_ADV_FAST_WHITELIST;
				     m_event_callback(&coms_evt, sizeof(coms_evt));
             break;

        case BLE_ADV_EVT_SLOW_WHITELIST:
             coms_evt.type = M_COMS_BLE_EVT_ADV_SLOW_WHITELIST;
				     m_event_callback(&coms_evt, sizeof(coms_evt));
             break;
	
        case BLE_ADV_EVT_IDLE:
					  is_adv_started = false;
					  coms_evt.type = M_COMS_BLE_EVT_ADV_IDLE;
				     m_event_callback(&coms_evt, sizeof(coms_evt));
            break;

        case BLE_ADV_EVT_WHITELIST_REQUEST:
        {
            ble_gap_addr_t whitelist_addrs[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];
            ble_gap_irk_t  whitelist_irks[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];
            uint32_t       addr_cnt = BLE_GAP_WHITELIST_ADDR_MAX_COUNT;
            uint32_t       irk_cnt  = BLE_GAP_WHITELIST_ADDR_MAX_COUNT;

            err_code = pm_whitelist_get(whitelist_addrs, &addr_cnt,
                                        whitelist_irks,  &irk_cnt);
            APP_ERROR_CHECK(err_code);
            NRF_LOG_DEBUG("pm_whitelist_get returns %d addr in whitelist and %d irk whitelist",
                          addr_cnt, irk_cnt);

            // Set the correct identities list (no excluding peers with no Central Address Resolution).
            identities_set(PM_PEER_ID_LIST_SKIP_NO_IRK);

            // Apply the whitelist.
            err_code = ble_advertising_whitelist_reply(&m_advertising,
                                                       whitelist_addrs,
                                                       addr_cnt,
                                                       whitelist_irks,
                                                       irk_cnt);
            APP_ERROR_CHECK(err_code);
        } break; //BLE_ADV_EVT_WHITELIST_REQUEST

        case BLE_ADV_EVT_PEER_ADDR_REQUEST:
        {
            pm_peer_data_bonding_t peer_bonding_data;

            // Only Give peer address if we have a handle to the bonded peer.
            if (m_peer_id != PM_PEER_ID_INVALID)
            {
                err_code = pm_peer_data_bonding_load(m_peer_id, &peer_bonding_data);
                if (err_code != NRF_ERROR_NOT_FOUND)
                {
                    APP_ERROR_CHECK(err_code);

                    // Manipulate identities to exclude peers with no Central Address Resolution.
                    identities_set(PM_PEER_ID_LIST_SKIP_ALL);

                    ble_gap_addr_t * p_peer_addr = &(peer_bonding_data.peer_ble_id.id_addr_info);
                    err_code = ble_advertising_peer_addr_reply(&m_advertising, p_peer_addr);
                    APP_ERROR_CHECK(err_code);
                }
            }
        } break; //BLE_ADV_EVT_PEER_ADDR_REQUEST

        default:
            break;
    }
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
				     // Notify m_coms of event
				     is_adv_started = false;
             coms_evt.type = M_COMS_BLE_EVT_CONNECTED;
             m_event_callback(&coms_evt, sizeof(coms_evt));
				     m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
				                err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
             APP_ERROR_CHECK(err_code);
				     
				     							 
 				    break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected");          
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
				    // Notify m_coms of event
            coms_evt.type = M_COMS_BLE_EVT_DISCONNECTED;
            m_event_callback(&coms_evt, sizeof(coms_evt));
            break; // BLE_GAP_EVT_DISCONNECTED

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTS_EVT_HVN_TX_COMPLETE:
          
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;
				

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}




/**@brief Function for the Peer Manager initialization.
 */
static void peer_manager_init(void)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = s_ble_params.sec_params.bond;
    sec_param.mitm           = s_ble_params.sec_params.mitm;
    sec_param.lesc           = s_ble_params.sec_params.lesc;
    sec_param.keypress       = s_ble_params.sec_params.keypress;
    sec_param.io_caps        = s_ble_params.sec_params.io_capabilities;
    sec_param.oob            = s_ble_params.sec_params.oob_data_available;
    sec_param.min_key_size   = s_ble_params.sec_params.min_key_size;
    sec_param.max_key_size   = s_ble_params.sec_params.max_key_size;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}



/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t               err_code;
    ble_advertising_init_t init;

	  ble_uuid_t m_adv_uuids[] = {0, 0};
	  m_adv_uuids->uuid = s_ble_params.adv_params.uuid;
	  m_adv_uuids->type = s_ble_params.adv_params.uuid_type;	
		
    memset(&init, 0, sizeof(init));

    init.advdata.flags                             = s_ble_params.adv_params.adv_flags;
    init.advdata.name_type                         = s_ble_params.adv_params.name_type;
    init.advdata.include_appearance                = true;
    init.advdata.uuids_complete.uuid_cnt           = 1; 
    init.advdata.uuids_complete.p_uuids            = m_adv_uuids;	
    init.config.ble_adv_whitelist_enabled          = true;
    init.config.ble_adv_directed_high_duty_enabled = true;
    init.config.ble_adv_directed_enabled           = false;
    init.config.ble_adv_directed_interval          = 0;
    init.config.ble_adv_directed_timeout           = 0;
    init.config.ble_adv_fast_enabled               = true;
    init.config.ble_adv_fast_interval              = s_ble_params.adv_params.adv_fast_interval;
    init.config.ble_adv_fast_timeout               = s_ble_params.adv_params.adv_fast_timeout;
    init.config.ble_adv_slow_enabled               = false;  //<-- disable slow advertising
  //init.config.ble_adv_slow_interval              = s_ble_params.adv_params.adv_slow_interval;
  //init.config.ble_adv_slow_timeout               = s_ble_params.adv_params.adv_slow_timeout;

    init.evt_handler   = on_adv_evt;
    init.error_handler = ble_advertising_error_handler;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}


//wright : Merge dt2 code ;
//void m_coms_ble_init( const app_sched_event_handler_t p_event_callback)
ret_code_t m_coms_ble_init( const app_sched_event_handler_t p_event_callback, const m_coms_ble_params_t * p_ble_params)
{
	 ret_code_t err_code;


   if (p_event_callback == 0)
   {
       err_code = NRF_ERROR_INVALID_PARAM;
	 }
   else
   {
       err_code = NRF_SUCCESS;
   }		 
	 APP_ERROR_CHECK(err_code);
	 
	 m_event_callback = p_event_callback;
	 
	 ble_stack_init();	 

	 
	 /*wright : dt2 part */
    m_coms_ble_hid_init_t   hid_params;
    uint32_t                flash_access_cnt = 0;
    
    if (p_event_callback == 0 || p_event_callback == 0)
    {
        return NRF_ERROR_INVALID_PARAM;
    }
    
    if (sizeof(m_coms_ble_params_t) > sizeof(s_ble_params_db))
    {
        // m_coms_ble_params_t has grown beyond flash storage size
        return NRF_ERROR_NO_MEM;
    }
    
    s_ble_params_dt2 = (m_coms_ble_params_t *) s_ble_params_db;
     
		//wright : It needs to be checked if it's nessrary to write flash in 52 ...
		
//    if (memcmp(s_ble_params_dt2, p_ble_params, sizeof(m_coms_ble_params_t)) != 0)
//    {
//        // If this is the first time the program has run, the parameters are written to flash. 
//        // This is to avoid putting parameter persistance requirements on the above application layers.
//        // Assert triggers if the database exceeds flash memory size
//        APP_ERROR_CHECK_BOOL((((uint32_t) s_ble_params_db) + CEIL_DIV(sizeof(m_coms_ble_params_t),4)) <= (NRF_FICR->CODEPAGESIZE * BLE_FLASH_PAGE_END));
//        ble_flash_block_write((uint32_t *)s_ble_params_db, (uint32_t *)p_ble_params, CEIL_DIV(sizeof(m_coms_ble_params_t),4));
//    }
    
    // Initializing static variables
    s_event_callback     = p_event_callback != 0 ? p_event_callback : s_event_callback;
    s_boot_mode_callback = s_ble_params_dt2->boot_mode_callback;
    s_encrypted          = false;
    s_sec_params_requested = false;
    s_boot_mode_active   = false;
    s_conn_handle        = BLE_CONN_HANDLE_INVALID;
    s_tx_buf_cnt         = 0;
    s_num_inp_reports    = 0;
    memset(&s_sec_params, 0, sizeof(s_sec_params));
    memset(&s_bas, 0, sizeof(s_bas));
    memset(&s_bas_report_ref, 0, sizeof(s_bas_report_ref));
    
    s_waiting_for_flash      = false;
    s_waiting_for_disconnect = false;
    s_waiting_reason     = 0;
    
    err_code = app_timer_create(&s_encryption_timer, APP_TIMER_MODE_SINGLE_SHOT, on_encrypt_timeout);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    // HID module initialization
    hid_params.base_hid_version = s_ble_params_dt2->base_hid_version;
    hid_params.b_country_code   = s_ble_params_dt2->hid_country_code;
    hid_params.flags            = s_ble_params_dt2->hids_flags;
    hid_params.io_capabilities  = s_ble_params_dt2->sec_params.io_capabilities;
    hid_params.db_loc           = (uint32_t *) s_hid_params_db;
    hid_params.db_size          = sizeof(s_hid_params_db);
    hid_params.evt_handler      = ble_hids_evt_handler;
    hid_params.error_handler    = ble_hids_error_handler;
    
    err_code = m_coms_ble_hid_init(&hid_params);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }    
    
    // SoftDevice and event handling initialization
    err_code = ble_stack_init();
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }      
    
    // Initialize persistent storage module.
    err_code = pstorage_init();
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    err_code = sec_params_init(s_ble_params);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }    
    
    // Device manager initialization
    err_code = device_manager_init(p_ble_params->bond_params.delete_bonds);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    err_code = m_coms_ble_addr_init(&m_app_instance_id);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    err_code = app_timer_create(&s_flash_access_timer, APP_TIMER_MODE_SINGLE_SHOT, on_flash_access_timeout);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    err_code = app_timer_create(&s_notify_usage_timer, APP_TIMER_MODE_SINGLE_SHOT, on_addr_notify_usage_timeout);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    err_code = pstorage_access_status_get(&flash_access_cnt);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    SEGGER_RTT_printf(0, "flash_access_cnt: %d\r\n", flash_access_cnt);

    if (flash_access_cnt > 0)
    {
        // Wait for flash access to finish before sending init finished event.
        s_waiting_for_flash = true;
        s_waiting_reason |= WAITING_REASON_INIT;
    }
    else
    {
        m_coms_ble_evt_t coms_evt;
        
        coms_evt.type = M_COMS_BLE_EVT_INIT_FINISHED;
        err_code = app_sched_event_put(&coms_evt, sizeof(coms_evt), ble_stack_disable);
        APP_ERROR_CHECK(err_code);        
    }

    return err_code;
	 
	/*wright : dt2 part ---end */
}


void m_coms_ble_enable(bool erase_bonds)
{
   gap_params_init();
	
	 gatt_init();
	
	 advertising_init();
	
	 services_init();
	
	 conn_params_init();
	
	 peer_manager_init();

	 if (erase_bonds == true)
   {
        delete_bonds();
        // Advertising is started by PM_EVT_PEERS_DELETE_SUCCEEDED event.
   }
	 else
	 {	 
        advertising_start();
   }

		 

}
  //wright: try to duplicate the same logic like DT2 
uint32_t m_coms_ble_disable(void)
{

	uint32_t         err_code;	
	//if connected => disconnect
	if(m_conn_handle  != BLE_CONN_HANDLE_INVALID)
	{
		// Disconnect
		m_coms_ble_diconnect();
		if (err_code != NRF_SUCCESS)
		{
				return err_code;
		}
		m_conn_handle = BLE_CONN_HANDLE_INVALID;
	}
	
	//stop adv
	err_code = sd_ble_gap_adv_stop(1);

	return err_code;
}	



void m_coms_ble_hid_report_send(uint8_t report_id, uint8_t  * p_data )
{ 
	  ret_code_t err_code;
    
    if(m_conn_handle == BLE_CONN_HANDLE_INVALID )  
		{
       if (!is_adv_started)
			 {	 
			    advertising_start();    //Can only call once	
			 }				 
		}			
    else
		{	
	     if(m_in_boot_mode)

		   err_code = ble_hids_boot_kb_inp_rep_send(&m_hids, s_report_input_ref_index[report_id], p_data, m_conn_handle);
		
		   else
		
	     err_code = ble_hids_inp_rep_send(&m_hids, s_report_input_ref_index[report_id], s_report_input_max_lens[report_id], p_data, m_conn_handle);
  
       APP_ERROR_CHECK(err_code);
		}
}

void m_coms_ble_bas_send(uint8_t battery_level)
{	
	  ret_code_t err_code;
	
	  err_code = ble_bas_battery_level_update(&m_bas, battery_level, BLE_CONN_HANDLE_ALL);
	  if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_BUSY) &&
        (err_code != NRF_ERROR_RESOURCES) &&
        (err_code != NRF_ERROR_FORBIDDEN) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
       )
    {
        APP_ERROR_HANDLER(err_code);
    }
}	

void m_coms_ble_diconnect(void)
{
	
  	 ret_code_t err_code;
	
	
     err_code = sd_ble_gap_disconnect(m_conn_handle,
                                      BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
     if (err_code != NRF_ERROR_INVALID_STATE)
     {
                APP_ERROR_CHECK(err_code);
     }

}

void m_coms_ble_whitelist_off(void)
{
	ret_code_t err_code;
	
   if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
   {
       err_code = ble_advertising_restart_without_whitelist(&m_advertising);
       if (err_code != NRF_ERROR_INVALID_STATE)
       {
          APP_ERROR_CHECK(err_code);
       }
   }
}

void m_coms_ble_sleep_mode_enter(void)
{
	  ret_code_t err_code;
	
	  // Go to system-off mode (this function will not return; wakeup will cause a reset).
	
    err_code = sd_power_system_off();
	  if( err_code != NRF_ERROR_SOC_POWER_OFF_SHOULD_NOT_RETURN)
	  { 
      APP_ERROR_CHECK(err_code);
    } 
}	

//wright : add for check if bond data exist
bool m_coms_ble_adv_bond_stored(void)    
{
	// wright : check peer count if bigger than 0 => there are bond data. 
	uint32_t peer_num =0;
	peer_num = pm_peer_count();
	return (peer_num>0);
}






/**
 * @}
 */
