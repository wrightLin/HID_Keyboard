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


/*wright add missing part of m_coms_ble.h */
#define BOOT_KEYBOARD_LEN 8 /** See HID 1.11 spec Appendix B.1 (Boot keyboard descriptor) */
/**@brief Packet definition used for boot reports. 
 * @details Format is specified in USB HID spec, Appendix B: Boot Interface Descriptors.
 */
typedef union
{
    struct
    {
        uint8_t keys[BOOT_KEYBOARD_LEN]; /** Keyboard keycodes */
    } keyboard_data;
    struct
    {
        uint8_t buttons; /** Mouse buttons */
        int8_t  x_delta; /** Mouse cursor X delta */
        int8_t  y_delta; /** Mouse cursor Y delta */
    } mouse_data;
} m_coms_hid_boot_pkt_t;

/**@brief When in BLE HID (HID over GATT) boot mode, only keyboard and mouse packets of a specified format can be sent. 
 * 
 * @note Bitmask type enum: valid values are powers of 2
 */
typedef enum
{
    ble_boot_pkt_keyboard = 0x01, /** Keyboard boot report */
    ble_boot_pkt_mouse    = 0x02, /** Mouse boot report */
    ble_boot_pkt_none     = 0x80  /** Not keyboard/mouse boot report. Will not be sent in boot mode. */
} m_coms_ble_hid_boot_type_t;
/**@brief When in boot mode (BLE HID) a specific format is used for mouse and keyboard packets regardless of HID descriptor.
 *        THe application will be tasked to re-assemble keyboard and mouse packets to fit the specified format 
 *        (http://developer.bluetooth.org/gatt/services/Pages/ServiceViewer.aspx?u=org.bluetooth.service.human_interface_device.xml).
 *        Non-keyboard and mouse packets will be discarded in boot mode.
 *
 * @param[in\out] p_boot_pkt      Resulting boot packet
 * @param[in]     p_pkt_type      Boot packet type
 * @param[in]     p_data          Original packet
 * @param[in]     p_len           Length of original packet
 * @param[in]     p_hid_interface Interface index packet was sent on
 * @param[in]     p_report_idx    Report index used
 */
typedef void (*m_coms_ble_boot_mode_callback)(m_coms_hid_boot_pkt_t *      p_boot_pkt, 
                                              m_coms_ble_hid_boot_type_t * p_pkt_type, 
                                              uint8_t *                    p_data, 
                                              uint8_t                      p_len, 
                                              uint8_t                      p_hid_interface, 
                                              uint8_t                      p_report_idx);

/**@brief Callback to get the BLE_EVT_TX_COMPLETE event to the application.
 *
 * @param[in]     p_ble_evt    Pointer to the BLE event structure.
 */
typedef void (*m_coms_ble_tx_complete_cb_t)(ble_evt_t * p_ble_evt);

/*wright add missing part of m_coms_ble.h --end*/


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
    
	//wright : open it for m_coms_ble_init();
   m_coms_ble_boot_mode_callback boot_mode_callback; /** Callback function used in HID Boot mode */
   m_coms_ble_tx_complete_cb_t   tx_complete_callback; /** Callback function used to get BLE_EVT_TX_COMPLETE event to the application in interrupt context */
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
	
	
	//wright : try to add DT2 Event types
		M_COMS_BLE_EVT_INIT_FINISHED, /** Initialization finished, ready to enable */
    M_COMS_BLE_EVT_READ_REQ,      /** Read request received */
    M_COMS_BLE_EVT_CONN_UPDATE,   /** Connection parameter update */
    M_COMS_BLE_EVT_ADV_TIMEOUT,   /** Advertising has timed out */
    M_COMS_BLE_EVT_ADV_BONDABLE,  /** Bondable advertising is running */
    M_COMS_BLE_EVT_ADDR_CHANGED,  /** Address has changed */
    M_COMS_BLE_EVT_PASSKEY_REQ,   /** Passkey requested */
    M_COMS_BLE_EVT_OOBKEY_REQ,    /** Passkey requested */    
    M_COMS_BLE_EVT_KEY_SENT,      /** Pass or OOB key sent */
    M_COMS_BLE_EVT_DISABLED
	
	
}m_coms_ble_evt_type_t;


/**@brief Data received event details */
typedef struct
{
	
			//wright : fix for dt2 data struct
			uint8_t   interface_idx; /** Which interface data was received on */
			uint8_t   report_type;   /** Which type of report is it. Input, output or feature */
			uint8_t   report_idx;    /** Which report index data was received on */ 
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

//wright : Merge dt2 code 
//void m_coms_ble_init( const app_sched_event_handler_t p_event_callback); 
ret_code_t m_coms_ble_init( const app_sched_event_handler_t p_event_callback, const m_coms_ble_params_t * p_ble_params);


void m_coms_ble_enable(bool erase_bonds);

//wright: try to duplicate the same logic like DT2
uint32_t m_coms_ble_disable(void);

void m_coms_ble_hid_report_send(uint8_t report_id ,uint8_t * p_data);

void m_coms_ble_bas_send(uint8_t battery_level);

void m_coms_ble_diconnect(void);

void m_coms_ble_whitelist_off(void);

void m_coms_ble_sleep_mode_enter(void);

//wright : add to check if bond data exist
bool m_coms_ble_adv_bond_stored(void);   


/*wright add missing part of m_coms_ble.h*/

/**@brief HID Report types */
typedef enum
{
    hid_report_type_input   = BLE_HIDS_REP_TYPE_INPUT,
    hid_report_type_output  = BLE_HIDS_REP_TYPE_OUTPUT,
    hid_report_type_feature = BLE_HIDS_REP_TYPE_FEATURE
} m_coms_hid_report_type_t;


/*wright add missing part of m_coms_ble.h --end*/


#endif /*  __M_COMS_BLE_H__ */

/** @} */

