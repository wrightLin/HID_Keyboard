/* @brief BLE communications sub-module.
 *
 * @details This is a communication sub-module used to deal with BTLE and HID over GATT-specifics.
 *          
 */
#ifndef __M_COMS_BLE_H__
#define __M_COMS_BLE_H__
 
#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#include "app_scheduler.h"
#include "ble_advdata.h"
#include "ble_hids.h"
#include "ble_types.h"
#include "peer_manager.h"



typedef struct
{	  
    bool         oob_data_available;
	  uint8_t      bond             :1; 
	  uint8_t      mitm             :1;
	  uint8_t      lesc             :1;
	  uint8_t      keypress         :1;	
	  uint8_t      io_capabilities  :3; /** See @ref BLE_GAP_IO_CAPS. */
	  uint8_t      min_key_size;
	  uint8_t      max_key_size;
} m_coms_ble_sec_params_t;



/**@brief Device Information Service parameters
 * (see http://developer.bluetooth.org/gatt/services/Pages/ServiceViewer.aspx?u=org.bluetooth.service.device_information.xml)
 * @note All strings should be of UTF-8 format.
 * @note NULL termination works for all UTF-8 strings, including Chinese, Korean, and Japanese.
 */
typedef struct
{
    char *   device_name;       /** Null-terminated device name */
    char *   manufacturer_name; /** Null-terminated manufacturer name */
    char *   hw_revision;       /** Null-terminated hardware revision string */
    char *   fw_revision;       /** Null-terminated firmware revision string */
    char *   serial_number;     /** Null-terminated serial number string */
    uint16_t pnp_vendor_id;     /** Vendor ID */
    uint8_t  pnp_vendor_id_src; /** Vendor ID source (@ref BLE_DIS_VENDOR_ID_SRC_BLUETOOTH_SIG or @ref BLE_DIS_VENDOR_ID_SRC_USB_IMPL_FORUM) */
    uint16_t pnp_product_id;    /** Product ID */
    uint16_t pnp_product_version;/** Product version */
} m_coms_ble_device_info_t;


typedef struct
{
  uint8_t                    adv_flags;                  /**< Advertising data Flags field. */
	ble_advdata_name_type_t    name_type;                  /**< Type of device name. */
	uint8_t                    short_name_len;             /**< Length of short device name (if short type is specified). */
  uint32_t adv_fast_interval;                            /**< Advertising interval for fast advertising. */
  uint32_t adv_fast_timeout;                             /**< Time-out (in units of 10ms) for fast advertising. */
  uint32_t adv_slow_interval;                            /**< Advertising interval for slow advertising. */
  uint32_t adv_slow_timeout;                             /**< Time-out (in units of 10ms) for slow advertising. */
	uint16_t uuid;                                         /**< UUID to be advertised  */
	uint8_t  uuid_type;                                    /**< Correpondinf UUID type */
	

}m_coms_ble_adv_params_t;	


typedef struct
{
  uint16_t min_conn_interval;              /**< Minimum Connection Interval in 1.25 ms units, see @ref BLE_GAP_CP_LIMITS.*/
  uint16_t max_conn_interval;              /**< Maximum Connection Interval in 1.25 ms units, see @ref BLE_GAP_CP_LIMITS.*/
  uint16_t slave_latency;                  /**< Slave Latency in number of connection events, see @ref BLE_GAP_CP_LIMITS.*/
  uint16_t conn_sup_timeout;               /**< Connection Supervision Timeout in 10 ms units, see @ref BLE_GAP_CP_LIMITS.*/
	uint32_t first_conn_params_update_delay; /**<Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called */
	uint32_t next_conn_params_update_delay;  /**< Time between each call to sd_ble_gap_conn_param_update after the first call */
	uint8_t  max_conn_params_update_count;   /**< Number of attempts before giving up the connection parameter negotiation. */
	
} m_coms_ble_conn_params_t;

/**@brief BLE parameters. See @ref M_COMS_BLE_PARAMS_FILL for to initialize default values. */
typedef struct
{
    m_coms_ble_device_info_t   device_info;  /** Device information parameters */	
    m_coms_ble_adv_params_t    adv_params;   /** Advertising parameters  */ 
    m_coms_ble_sec_params_t    sec_params;   /** BLE security parameters */
    bool                       delete_bond;  /** Delete Bond parameters */
    uint16_t                   appearance;   /** Device appearance. See @ref BLE_APPEARANCES */
    m_coms_ble_conn_params_t   conn_params;  /** Preferred connection parameters */
    
    uint16_t base_hid_version; /** This states which HID usage table version that this USB descriptor uses */
    uint8_t  hid_country_code; /** HID country code. Set to 0 to indicate no specific country */
    uint8_t  hids_flags;       /** HID Service flags. See section 4.10 (HID Information Behavior) of HOGP spec. */
    uint8_t  max_tx_buf_cnt;   /** Max number of packets put in SoftDevice TX buffer at a time */
    
   //m_coms_ble_boot_mode_callback boot_mode_callback; /** Callback function used in HID Boot mode */
   //m_coms_ble_tx_complete_cb_t   tx_complete_callback; /** Callback function used to get BLE_EVT_TX_COMPLETE event to the application in interrupt context */
} m_coms_ble_params_t;



/**@brief Events types generated by this module */
typedef enum
{
  M_COMS_BLE_EVT_ADV_DIRECTED_HIGH_DUTY,
	M_COMS_BLE_EVT_ADV_DIRECTED,
	M_COMS_BLE_EVT_ADV_FAST,
	M_COMS_BLE_EVT_ADV_SLOW,
	M_COMS_BLE_EVT_ADV_FAST_WHITELIST,
	M_COMS_BLE_EVT_ADV_SLOW_WHITELIST,
	M_COMS_BLE_EVT_ADV_IDLE,                     /** Advertising has timed out */
	M_COMS_BLE_EVT_CONNECTED,                    /** Connected */
  M_COMS_BLE_EVT_DISCONNECTED,                 /** Disconnected */
	M_COMS_BLE_EVT_DATA_RECEIVED,                /** Data received */
	M_COMS_BLE_EVT_BONDED,
	
}m_coms_ble_evt_type_t;


/**@brief Data received event details */
typedef struct
{
//    uint8_t   interface_idx; /** Which interface data was received on */
//    uint8_t   report_type;   /** Which type of report is it. Input, output or feature */
//    uint8_t   report_idx;    /** Which report index data was received on */ 
      uint8_t   len;           /** Length of received data */
      uint8_t * data;          /** Received data */
} m_coms_ble_evt_data_recv_t;

/**@brief Read request event details */
typedef struct
{
    uint8_t   interface_idx; /** Which interface read report is in */
    uint8_t   report_idx;    /** Which report index is read */ 
} m_coms_ble_evt_read_req_t;

/**@Brief Connection update event details */
typedef struct
{
    uint16_t min_conn_interval;   /** Min Connection interval [1.25 ms units] */
    uint16_t max_conn_interval;   /** Max Connection interval [1.25 ms units] */
    uint16_t slave_latency;       /** Slave latency */
    uint16_t supervision_timeout; /** Link timeout [10 ms units] */
} m_coms_ble_evt_conn_update_t;


/**@brief Event structs generated by this module */
typedef struct
{
    m_coms_ble_evt_type_t  type;
	
	  m_coms_ble_evt_data_recv_t data_received;
//    union
//    {
//        m_coms_ble_evt_data_recv_t    data_received;
//        m_coms_ble_evt_read_req_t     read_req;
//        m_coms_ble_evt_conn_update_t  conn_update;
//    } data;
} m_coms_ble_evt_t;


void m_coms_ble_init( const app_sched_event_handler_t p_event_callback); 

void m_coms_ble_enable(bool erase_bonds);

void m_coms_ble_disable(void);

void m_coms_ble_hid_report_send(uint8_t report_id ,uint8_t * p_data);

void m_coms_ble_bas_send(uint8_t battery_level);

void m_coms_ble_diconnect(void);

void m_coms_ble_whitelist_off(void);

void m_coms_ble_sleep_mode_enter(void);


#endif /*  __M_COMS_BLE_H__ */

/** @} */

