/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 * $LastChangedRevision$
 */
/** @file 
 *  @brief Project-specific parameters and helper functions.
 *
 */
#ifndef __PROJECT_PARAMS_H__
#define __PROJECT_PARAMS_H__

#include <string.h>


#include "ble_types.h"
#include "m_coms_ble.h"
#include "m_coms_gzll.h"
#include "ble_dis.h"
#include "app_util.h"
#include "ble_types.h"
#include "app_timer.h"

#define NUM_OF_INPUT_REPORTS            3
#define NUM_OF_OUTPUT_REPORTS           0
#define NUM_OF_FEATURE_REPORTS          0

#define INPUT_REP_BUTTONS_INDEX         0                       /**< Index of Mouse Input Report containing button data. */
#define INPUT_REP_REF_BUTTONS_ID        1                       /**< Id of reference to Mouse Input Report containing button data. */
#define INPUT_REP_BUTTONS_LEN           3                       /**< Length of Mouse Input Report containing button data. */

#define INPUT_REP_MOVEMENT_INDEX        1                       /**< Index of Mouse Input Report containing movement data. */
#define INPUT_REP_REF_MOVEMENT_ID       2                       /**< Id of reference to Mouse Input Report containing movement data. */
#define INPUT_REP_MOVEMENT_LEN          3                       /**< Length of Mouse Input Report containing movement data. */

#define INPUT_REP_MPLAYER_INDEX         2                       /**< Index of Mouse Input Report containing media player data. */
#define INPUT_REP_REF_MPLAYER_ID        3                       /**< Id of reference to Mouse Input Report containing media player data. */
#define INPUT_REP_MEDIA_PLAYER_LEN      1                       /**< Length of Mouse Input Report containing media player data. */

BLE_HIDS_DEF(m_hids,                                                                /**< HID service instance. */
             NRF_SDH_BLE_TOTAL_LINK_COUNT,
             INPUT_REP_BUTTONS_LEN,
             INPUT_REP_MOVEMENT_LEN,
             INPUT_REP_MEDIA_PLAYER_LEN);

static const uint8_t s_report_input_max_lens[] = {INPUT_REP_BUTTONS_LEN, INPUT_REP_MOVEMENT_LEN, INPUT_REP_MEDIA_PLAYER_LEN};

static const uint8_t s_report_input_ref_id[] =  {INPUT_REP_REF_BUTTONS_ID, INPUT_REP_REF_MOVEMENT_ID, INPUT_REP_REF_MPLAYER_ID};

static const uint8_t s_report_input_ref_index[] =  {INPUT_REP_BUTTONS_INDEX, INPUT_REP_MOVEMENT_INDEX, INPUT_REP_MPLAYER_INDEX};

static const uint8_t s_hid_descriptor[] =
{
    0x05, 0x01,                     // Usage Page (Generic Desktop)
    0x09, 0x02,                     // Usage (Mouse)

    0xA1, 0x01,                     // Collection (Application)

    // Report ID 1: Mouse buttons + scroll/pan
    0x85, 0x01,                     //     Report Id 1 
    0x09, 0x01,                     //     Usage (Pointer)
    0xA1, 0x00,                     //     Collection (Physical)
    0x95, 0x05,                     //         Report Count (5)
    0x75, 0x01,                     //         Report Size (1)
    0x05, 0x09,                     //         Usage Page (Buttons)
    0x19, 0x01,                     //             Usage Minimum (01)
    0x29, 0x05,                     //             Usage Maximum (05)
    0x15, 0x00,                     //             Logical Minimum (0)
    0x25, 0x01,                     //             Logical Maximum (1)
    0x81, 0x02,                     //             Input (Data, Variable, Absolute)
    0x95, 0x01,                     //             Report Count (1)
    0x75, 0x03,                     //             Report Size (3)
    0x81, 0x01,                     //             Input (Constant) for padding
    0x75, 0x08,                     //             Report Size (8)
    0x95, 0x01,                     //             Report Count (1)
    0x05, 0x01,                     //         Usage Page (Generic Desktop)
    0x09, 0x38,                     //             Usage (Wheel)
    0x15, 0x81,                     //             Logical Minimum (-127)
    0x25, 0x7F,                     //             Logical Maximum (127)
    0x81, 0x06,                     //             Input (Data, Variable, Relative) 
    0x05, 0x0C,                     //         Usage Page (Consumer)
    0x0A, 0x38, 0x02,               //             Usage (AC Pan) 
    0x95, 0x01,                     //             Report Count (1)
    0x81, 0x06,                     //             Input (Data,Value,Relative,Bit Field)
    0xC0,                           //     End Collection (Physical)

    // Report ID 2: Mouse motion
    0x85, 0x02,                     //     Report Id 2 
    0x09, 0x01,                     //     Usage (Pointer)
    0xA1, 0x00,                     //     Collection (Physical)
    0x75, 0x0C,                     //         Report Size (12)
    0x95, 0x02,                     //         Report Count (2)
    0x05, 0x01,                     //         Usage Page (Generic Desktop)
    0x09, 0x30,                     //             Usage (X)                                                  
    0x09, 0x31,                     //             Usage (Y)
    0x16, 0x01, 0xF8,               //             Logical maximum (2047) 
    0x26, 0xFF, 0x07,               //             Logical minimum (-2047) 
    0x81, 0x06,                     //             Input (Data, Variable, Relative) 
    0xC0,                           //     End Collection (Physical) 
    0xC0,                           // End Collection (Application)

    // Report ID 3: Advanced buttons
    0x05, 0x0C,                     // Usage Page (Consumer)
    0x09, 0x01,                     // Usage (Consumer Control) 
    0xA1, 0x01,                     // Collection (Application)
    0x85, 0x03,                     //     Report Id (3) 
    0x15, 0x00,                     //     Logical minimum (0)
    0x25, 0x01,                     //     Logical maximum (1) 
    0x75, 0x01,                     //     Report Size (1)
    0x95, 0x01,                     //     Report Count (1)

    0x09, 0xCD,                     //     Usage (Play/Pause) 
    0x81, 0x06,                     //     Input (Data,Value,Relative,Bit Field)
    0x0A, 0x83, 0x01,               //     Usage (AL Consumer Control Configuration) 
    0x81, 0x06,                     //     Input (Data,Value,Relative,Bit Field)
    0x09, 0xB5,                     //     Usage (Scan Next Track)
    0x81, 0x06,                     //     Input (Data,Value,Relative,Bit Field)   
    0x09, 0xB6,                     //     Usage (Scan Previous Track)
    0x81, 0x06,                     //     Input (Data,Value,Relative,Bit Field) 

    0x09, 0xEA,                     //     Usage (Volume Down)
    0x81, 0x06,                     //     Input (Data,Value,Relative,Bit Field) 
    0x09, 0xE9,                     //     Usage (Volume Up) 
    0x81, 0x06,                     //     Input (Data,Value,Relative,Bit Field) 
    0x0A, 0x25, 0x02,               //     Usage (AC Forward) 
    0x81, 0x06,                     //     Input (Data,Value,Relative,Bit Field) 
    0x0A, 0x24, 0x02,               //     Usage (AC Back) 
    0x81, 0x06,                     //     Input (Data,Value,Relative,Bit Field) 
    
    //2 byte feature report
    0x09, 0x05,                 //     Usage (Vendor Defined)
    0x15, 0x00,                 //     Logical Minimum (0)
    0x26, 0xFF, 0x00,           //     Logical Maximum (255)
    0x75, 0x08,                 //     Report Count (2)
    0x95, 0x02,                 //     Report Size (8 bit)
    0xB1, 0x02,                 //     Feature (Data, Variable, Absolute)
        
    0xC0                            // End Collection
};

/*
 *  BLE Parameters 
 */	
static const m_coms_ble_params_t s_ble_params =
{
	 // Device information
  .device_info.device_name         = "m_coms_mu",       
	.device_info.manufacturer_name   = "Nordic Semiconductor",  
  .device_info.fw_revision         = "1.0.0",                
	.device_info.hw_revision         = 0,                                                         // Don't have a specific hardware revision yet
  .device_info.serial_number       = 0,                                                         // TODO: Use NRF_FICR->DEVICEID
	.device_info.pnp_product_id      = 0xEEEE,                                                    // PNP ID product ID
  .device_info.pnp_product_version = 0x0001,                                                    // PNP ID product version
	.device_info.pnp_vendor_id       = 0x1915,
  .device_info.pnp_vendor_id_src   = BLE_DIS_VENDOR_ID_SRC_USB_IMPL_FORUM,
	
	//Advertising parameters
	.adv_params.adv_flags            = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE,
	.adv_params.name_type            = BLE_ADVDATA_FULL_NAME,
	.adv_params.adv_fast_interval    = 0x0028,                                                  // Fast advertising interval (in units of 0.625 ms. This value corresponds to 25 ms.). */
	.adv_params.adv_slow_interval    = 0x0C80,                                                  // Slow advertising interval (in units of 0.625 ms. This value corrsponds to 2 seconds). */
  .adv_params.adv_fast_timeout     = 18000,                                                   // The advertising duration of fast advertising in units of 10 milliseconds. */
  .adv_params.adv_slow_timeout     = 0,                                                       // The advertising duration of slow advertising in units of 10 milliseconds. */
  .adv_params.uuid                 = BLE_UUID_HUMAN_INTERFACE_DEVICE_SERVICE,
	.adv_params.uuid_type            = BLE_UUID_TYPE_BLE,
	
  // Preferred connection parameters
  .conn_params.min_conn_interval                  = MSEC_TO_UNITS(7.5, UNIT_1_25_MS),         // Minimum connection interval. In units of 1.25 ms. 
  .conn_params.max_conn_interval                  = MSEC_TO_UNITS(15, UNIT_1_25_MS),          // Maximum connection interval. In units of 1.25 ms.
  .conn_params.slave_latency                      = 6,                                        // Number of connection events the Peripheral can skip. 
                                                                                              // Note that this introduces delay in Central -> Peripheral communication
  .conn_params.conn_sup_timeout                   = MSEC_TO_UNITS(430, UNIT_10_MS),           // Connection timeout in units of 10 ms. If link members i.e. loses power it takes this long
  .conn_params.first_conn_params_update_delay     = APP_TIMER_TICKS(5000),                    // (5 secs)                                                                         
  .conn_params.next_conn_params_update_delay      = APP_TIMER_TICKS(30000),                   // (30 secs)
	.conn_params.max_conn_params_update_count       = 3,
	
	// Security parameters
  .sec_params.bond               = 1,
	.sec_params.mitm               = 0,
	.sec_params.lesc               = 0,
	.sec_params.io_capabilities    = BLE_GAP_IO_CAPS_NONE,                                         // See @ref BLE_GAP_IO_CAPS for valid numbers.
  .sec_params.oob_data_available = false,
  .sec_params.keypress           = 0,		
	.sec_params.min_key_size       = 7,
	.sec_params.max_key_size       = 16,
	
  // Misc parameters
  .appearance       = BLE_APPEARANCE_HID_MOUSE,
  .base_hid_version = 0x010,                                                                  // Version number of base USB HID Specification implemented by this device
  .hid_country_code = 0,                                                                      // Country code can be used to specify which country the hardware is localized for. Most hardware is not localized (value 0).
  .max_tx_buf_cnt   = 2,                                                                      // Only allow 2 packets to be in SoftDevice TX buffer at a time to reduce max interrupt latency
	
}; 	

/*
 *  GZLL Parameters 
 */
static const m_coms_gzll_params_t  s_gzll_params =
{	 
    // Device information
    .encrypt          = false,                                                      // Make encrypted pipe available 
    .tx_attempts      = 400,
    .timeslot_period  = 504,                                                       //Should be the the LU1 dongle RX_PERIOD / 2
    .sync_lifetime    = 1000,
};

#endif /* __PROJECT_PARAMS_H__ */
