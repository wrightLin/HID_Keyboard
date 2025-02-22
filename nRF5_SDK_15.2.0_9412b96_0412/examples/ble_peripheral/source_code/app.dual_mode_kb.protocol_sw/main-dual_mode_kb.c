

/**
 * Copyright (c) 2014 - 2018, Nordic Semiconductor ASA
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
 * @defgroup ble_sdk_app_template_main main.c
 * @{
 * @ingroup ble_sdk_app_template
 * @brief Template project main file.
 *
 * This file contains a template for creating a new application. It has the code necessary to wakeup
 * from button, advertise, get a connection restart advertising on disconnect and if no new
 * connection created go back to system-off mode.
 * It can easily be used as a starting point for creating a new application, the comments identified
 * with 'YOUR_JOB' indicates where and how you can customize.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"

//wright : Fix for not to infect GPIO output
//#include "bsp_btn_ble.h"

#include "app_scheduler.h"

#include "app_timer.h"

#include "sensorsim.h"

#include "nrf_pwr_mgmt.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "m_coms_ble.h"

#include "m_coms_gzll.h"

#include "bsp_config.h"

#include "peer_manager.h"

#include "nrf_sdh.h"

#include "nrf_drv_clock.h"

#include "nrf_drv_gpiote.h"

#include "nrf_delay.h"

#include "nrf_sdm.h"

#include "project_params.h"

#define DEAD_BEEF                          0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define SHIFT_BUTTON_ID                     1                                          /**< Button used as 'SHIFT' Key. */

#define BATTERY_LEVEL_MEAS_INTERVAL          APP_TIMER_TICKS(2000)                      /**< Battery level measurement interval (ticks). */
#define MIN_BATTERY_LEVEL                    81                                         /**< Minimum simulated battery level. */
#define MAX_BATTERY_LEVEL                    100                                        /**< Maximum simulated battery level. */
#define BATTERY_LEVEL_INCREMENT              1                                          /**< Increment between each simulated battery level measurement. */


#define SCHED_QUEUE_SIZE                     30
#define SCHED_MAX_EVENT_DATA_SIZE            MAX(sizeof(ble_evt_t), MAX(sizeof(app_timer_event_t) \
                                               , sizeof(m_coms_ble_evt_t)))              /**< Maximum size of scheduler events. */

#define MODIFIER_KEY_POS                     0                                          /**< Position of the modifier byte in the Input Report. */
#define SCAN_CODE_POS                        2                                          /**< The start position of the key scan code in a HID Report. */
#define SHIFT_KEY_CODE                       0x02                                       /**< Key code indicating the press of the Shift Key. */

#define MAX_KEYS_IN_ONE_REPORT               (INPUT_REPORT_KEYS_MAX_LEN - SCAN_CODE_POS) /**< Maximum number of key presses that can be sent in one Input Report. */

#define OUTPUT_REPORT_BIT_MASK_CAPS_LOCK     0x02                                       /**< CAPS LOCK bit in Output Report (based on 'LED Page (0x08)' of the Universal Serial Bus HID Usage Tables). */

//wright : change for test															
#define M_COMS_GZLL_MODE                     0																							 
#define M_COMS_BLE_MODE_PAIR_1               1
#define M_COMS_BLE_MODE_PAIR_2               2
																							 
																							 
#define PROTOCOL_DETECT_PIN                  3

																							 
#define APP_GZLL_SEARCH_INTERVAL             APP_TIMER_TICKS(1000)
																							 
																							 
																				 
																							 

/********************************* wright : header and defines ******************************/

#include "m_keyboard.h"
#include "m_pwr_and_clk_mgmt.h"


/********************************* wright : fds part *************************************/

//wright: mode selection 
#include "fds.h"
#define LAST_TRANSMIT_MODE_FILE_ID 0x1111
#define LAST_TRANSMIT_MODE_RECORD_KEY 0x2222
#define DELETE_RECORD_NOT_FOUND 0xEEEE
#define REC_DEFAULT_VALUE 0xFFFF
#define DEFAULT_TRANSMIT_MODE 0x0000 //Gazell

static void fds_evt_handler(fds_evt_t const * p_evt);
static ret_code_t mode_record_delete(void);
static ret_code_t mode_record_write(uint32_t mode_value);
static ret_code_t mode_record_update(uint32_t mode_value);
static ret_code_t mode_record_read(void);


/* Flag to check fds initialization. */
static bool volatile m_fds_initialized;
bool fds_busy_flag = false;
static uint32_t current_transmit_mode_data[1] = {DEFAULT_TRANSMIT_MODE};

static struct
{
    bool delete_next;   //!< Delete next record.
    bool pending;       //!< Waiting for an fds FDS_EVT_DEL_RECORD event, to delete the next record.
} m_delete_all;

/********************************* wright : fds part *************************************/



/********************************* wright change mode start ******************************/

volatile bool change_mode_is_start = false;

#define DEFAULT_TRANSMIT_MODE 0x0000 //Gazell
uint8_t is_softdevice_enabled = false;
/****************************** wright change mode end ******************************/



/****************************** wright for swipe **************************/
static m_keyboard_data_t  s_gzll_keepalive_keyboard_pkt;
APP_TIMER_DEF(s_gzll_keep_alive_id);
static void m_keyboard_handler(void * p_event_data, uint16_t event_size);
static void gzll_keep_alive_handler();

/****************************** wright for swipe end**************************/

/**Buffer queue access macros
 *
 * @{ */

//#define MAX_BUFFER_ENTRIES                  5                                          /**< Number of elements that can be enqueued */

///** Initialization of buffer list */
//#define BUFFER_LIST_INIT()     \
//    do                         \
//    {                          \
//        buffer_list.rp    = 0; \
//        buffer_list.wp    = 0; \
//        buffer_list.count = 0; \
//    } while (0)

///** Provide status of data list is full or not */
//#define BUFFER_LIST_FULL() \
//    ((MAX_BUFFER_ENTRIES == buffer_list.count - 1) ? true : false)

///** Provides status of buffer list is empty or not */
//#define BUFFER_LIST_EMPTY() \
//    ((0 == buffer_list.count) ? true : false)

//#define BUFFER_ELEMENT_INIT(i)                 \
//    do                                         \
//    {                                          \
//        buffer_list.buffer[(i)].p_data = NULL; \
//    } while (0)
//		
///** @} */

///** Abstracts buffer element */
//typedef struct hid_key_buffer
//{
//    uint8_t      data_offset; /**< Max Data that can be buffered for all entries */
//    uint8_t      data_len;    /**< Total length of data */
//    uint8_t    * p_data;      /**< Scanned key pattern */
//} buffer_entry_t;

//STATIC_ASSERT(sizeof(buffer_entry_t) % 4 == 0);

///** Circular buffer list */
//typedef struct
//{
//    buffer_entry_t buffer[MAX_BUFFER_ENTRIES]; /**< Maximum number of entries that can enqueued in the list */
//    uint8_t        rp;                         /**< Index to the read location */
//    uint8_t        wp;                         /**< Index to write location */
//    uint8_t        count;                      /**< Number of elements in the list */
//} buffer_list_t;

//STATIC_ASSERT(sizeof(buffer_list_t) % 4 == 0);

//static buffer_list_t     buffer_list;                               /**< List to enqueue not just data to be sent, but also related information like the handle, connection handle etc */


APP_TIMER_DEF(m_app_gzll_search_timer_id);

APP_TIMER_DEF(m_battery_timer_id);                                  /**< Battery timer. */

static uint8_t m_sample_key_press_scan_str[] = /**< Key pattern to be sent when the key press button has been pushed. */
{
    0x0b,       /* Key h */
    0x08,       /* Key e */
    0x0f,       /* Key l */
    0x0f,       /* Key l */
    0x12,       /* Key o */
    0x28        /* Key Return */
};

static uint8_t m_caps_on_key_scan_str[] = /**< Key pattern to be sent when the output report has been written with the CAPS LOCK bit set. */
{
    0x06,       /* Key C */
    0x04,       /* Key a */
    0x13,       /* Key p */
    0x16,       /* Key s */
    0x12,       /* Key o */
    0x11,       /* Key n */
};

static uint8_t m_caps_off_key_scan_str[] = /**< Key pattern to be sent when the output report has been written with the CAPS LOCK bit cleared. */
{
    0x06,       /* Key C */
    0x04,       /* Key a */
    0x13,       /* Key p */
    0x16,       /* Key s */
    0x12,       /* Key o */
    0x09,       /* Key f */
};


static uint32_t              m_protocol_mode;
static bool              m_caps_on = false;                         /**< Variable to indicate if Caps Lock is turned on. */
//static bool              is_bonded = false;

static sensorsim_cfg_t   m_battery_sim_cfg;                         /**< Battery Level sensor simulator configuration. */
static sensorsim_state_t m_battery_sim_state;                       /**< Battery Level sensor simulator state. */

static nrf_drv_gpiote_in_config_t		protocol_detect_pin_cfg;

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for initializing the battery sensor simulator.
 */
static void sensor_simulator_init(void)
{
    m_battery_sim_cfg.min          = MIN_BATTERY_LEVEL;
    m_battery_sim_cfg.max          = MAX_BATTERY_LEVEL;
    m_battery_sim_cfg.incr         = BATTERY_LEVEL_INCREMENT;
    m_battery_sim_cfg.start_at_max = true;

    sensorsim_init(&m_battery_sim_state, &m_battery_sim_cfg);
}



/**@brief Function for performing a battery measurement, and update the Battery Level characteristic in the Battery Service.
 */
static void battery_level_update(void)
{
    uint8_t  battery_level;

    battery_level = (uint8_t)sensorsim_measure(&m_battery_sim_state, &m_battery_sim_cfg);

    m_coms_ble_bas_send(battery_level);
}


/**@brief Function for handling the Battery measurement timer timeout.
 *
 * @details This function will be called each time the battery level measurement timer expires.
 *
 * @param[in]   p_context   Pointer used for passing some arbitrary information (context) from the
 *                          app_start_timer() call to the timeout handler.
 */
static void battery_level_meas_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    battery_level_update();
}

static void app_gzll_search_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    m_coms_gzll_search_for_host();
}	


/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    // Initialize timer module.
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
    if(m_protocol_mode == M_COMS_BLE_MODE_PAIR_1 || m_protocol_mode == M_COMS_BLE_MODE_PAIR_2)
    {	
        err_code = app_timer_create(&m_battery_timer_id,
                                   APP_TIMER_MODE_REPEATED,
                                   battery_level_meas_timeout_handler);
      
		}	
		else  //M_COMS_GZLL_MODE
		{
//        err_code = app_timer_create(&m_app_gzll_search_timer_id,
//                                   APP_TIMER_MODE_SINGLE_SHOT,
//                                   app_gzll_search_timeout_handler);
			
			
			//wright for swipe problem
			app_timer_create(&s_gzll_keep_alive_id, APP_TIMER_MODE_REPEATED, gzll_keep_alive_handler);
			
    }		



    APP_ERROR_CHECK(err_code);
}

static void clock_start( void )
{   
    uint32_t err_code = nrf_drv_clock_init();
    
		APP_ERROR_CHECK(err_code);
	
    nrf_drv_clock_lfclk_request(NULL);
	
}


/**@brief Function for starting timers.
 */
static void application_timers_start(void)
{
   if(m_protocol_mode == M_COMS_BLE_MODE_PAIR_1 || m_protocol_mode == M_COMS_BLE_MODE_PAIR_2)
   {	    
	     /* YOUR_JOB: Start your timers. below is an example of how to start a timer.*/
       ret_code_t err_code;	
       err_code = app_timer_start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL, NULL);
       APP_ERROR_CHECK(err_code);
   }
}


/**@brief   Function for transmitting a key scan Press & Release Notification.
 *
 * @warning This handler is an example only. You need to analyze how you wish to send the key
 *          release.
 *
 * @param[in]  p_key_pattern  Pointer to key pattern.
 * @param[in]  pattern_len    Length of key pattern. 0 < pattern_len < 7.
 * @param[in]  pattern_offset Offset applied to Key Pattern for transmission.
 * @param[out] actual_len     Provides actual length of Key Pattern transmitted, making buffering of
 *                            rest possible if needed.
 * @return     NRF_SUCCESS on success, NRF_ERROR_RESOURCES in case transmission could not be
 *             completed due to lack of transmission buffer or other error codes indicating reason
 *             for failure.
 *
 * @note       In case of NRF_ERROR_RESOURCES, remaining pattern that could not be transmitted
 *             can be enqueued \ref buffer_enqueue function.
 *             In case a pattern of 'cofFEe' is the p_key_pattern, with pattern_len as 6 and
 *             pattern_offset as 0, the notifications as observed on the peer side would be
 *             1>    'c', 'o', 'f', 'F', 'E', 'e'
 *             2>    -  , 'o', 'f', 'F', 'E', 'e'
 *             3>    -  ,   -, 'f', 'F', 'E', 'e'
 *             4>    -  ,   -,   -, 'F', 'E', 'e'
 *             5>    -  ,   -,   -,   -, 'E', 'e'
 *             6>    -  ,   -,   -,   -,   -, 'e'
 *             7>    -  ,   -,   -,   -,   -,  -
 *             Here, '-' refers to release, 'c' refers to the key character being transmitted.
 *             Therefore 7 notifications will be sent.
 *             In case an offset of 4 was provided, the pattern notifications sent will be from 5-7
 *             will be transmitted.
 */
static uint32_t send_key_scan_press_release(
                                            uint8_t    * p_key_pattern,
                                            uint16_t     pattern_len,
                                            uint16_t     pattern_offset,
                                            uint16_t   * p_actual_len)
{
    ret_code_t err_code = NRF_SUCCESS;
    uint16_t offset;
    uint16_t data_len;
    uint8_t  data[INPUT_REPORT_KEYS_MAX_LEN];

    // HID Report Descriptor enumerates an array of size 6, the pattern hence shall not be any
    // longer than this.
    STATIC_ASSERT((INPUT_REPORT_KEYS_MAX_LEN - 2) == 6);

    ASSERT(pattern_len <= (INPUT_REPORT_KEYS_MAX_LEN - 2));

    offset   = pattern_offset;
    data_len = pattern_len;

    do
    {
        // Reset the data buffer.
        memset(data, 0, sizeof(data));

        // Copy the scan code.
        memcpy(data + SCAN_CODE_POS + offset, p_key_pattern + offset, data_len - offset);
				//wright : Fix for not to infect GPIO output
				//wright : TODO : knowing how to add SHIFT key vaule into data befor sending. 
//        if (bsp_button_is_pressed(SHIFT_BUTTON_ID))
//        {
//            data[MODIFIER_KEY_POS] |= SHIFT_KEY_CODE;
//        }

				if (m_protocol_mode == M_COMS_BLE_MODE_PAIR_1 || m_protocol_mode == M_COMS_BLE_MODE_PAIR_2 )
				{
					m_coms_ble_hid_report_send(0, data);
        }
				else
				{
          m_coms_gzll_send(1 ,data, sizeof(data));
        }									
        offset++;
    }
    while (offset <= data_len);

    *p_actual_len = offset;

    return err_code;
}


//wright : fix for not detecting modifier keys.
static uint32_t send_key_scan_press_release_fix(
                                            uint8_t    * p_key_pattern,
                                            uint16_t     pattern_len,
                                            uint16_t     pattern_offset,
                                            uint16_t   * p_actual_len,
																						uint8_t     p_modifer
)
{

    ret_code_t err_code = NRF_SUCCESS;
    uint16_t offset;
    uint16_t data_len;
    uint8_t  data[INPUT_REPORT_KEYS_MAX_LEN];

    // HID Report Descriptor enumerates an array of size 6, the pattern hence shall not be any
    // longer than this.
    STATIC_ASSERT((INPUT_REPORT_KEYS_MAX_LEN - 2) == 6);

    ASSERT(pattern_len <= (INPUT_REPORT_KEYS_MAX_LEN - 2));

    offset   = pattern_offset;
    data_len = pattern_len;


        // Reset the data buffer.
        memset(data, 0, sizeof(data));

        // Copy the scan code.
        memcpy(data + SCAN_CODE_POS + offset, p_key_pattern + offset, data_len - offset);


				//****** wright : before sending keypacket, merge it with modifier keys. *******//
				if (p_modifer != 0x00)
            data[MODIFIER_KEY_POS] |= p_modifer;
				
				
				if (m_protocol_mode == M_COMS_BLE_MODE_PAIR_1 || m_protocol_mode == M_COMS_BLE_MODE_PAIR_2)
				{
					m_coms_ble_hid_report_send(0, data);
        }
				else
				{
					
					//wright 0503 check if there is any empty packet
					
					bool is_data_empty = true;
					for(int i = 0 ; i < INPUT_REPORT_KEYS_MAX_LEN;i++)
					{
						if(data[i] != 0x00)
						{
							is_data_empty = false;
						}
					}
					if(is_data_empty)
					{
						NRF_LOG_INFO("there is an empty data going to be sent......data: \r\n");
						NRF_LOG_HEXDUMP_INFO(data,INPUT_REPORT_KEYS_MAX_LEN);
					}
					
					
          m_coms_gzll_send(1 ,data, sizeof(data));

        }									


    *p_actual_len = offset;

    return err_code;
}


#if 0
/**@brief   Function for initializing the buffer queue used to key events that could not be
 *          transmitted
 *
 * @warning This handler is an example only. You need to analyze how you wish to buffer or buffer at
 *          all.
 *
 * @note    In case of HID keyboard, a temporary buffering could be employed to handle scenarios
 *          where encryption is not yet enabled or there was a momentary link loss or there were no
 *          Transmit buffers.
 */
static void buffer_init(void)
{
    uint32_t buffer_count;

    BUFFER_LIST_INIT();

    for (buffer_count = 0; buffer_count < MAX_BUFFER_ENTRIES; buffer_count++)
    {
        BUFFER_ELEMENT_INIT(buffer_count);
    }
}


/**@brief Function for enqueuing key scan patterns that could not be transmitted either completely
 *        or partially.
 *
 * @warning This handler is an example only. You need to analyze how you wish to send the key
 *          release.
 *
 * @param[in]  p_hids         Identifies the service for which Key Notifications are buffered.
 * @param[in]  p_key_pattern  Pointer to key pattern.
 * @param[in]  pattern_len    Length of key pattern.
 * @param[in]  offset         Offset applied to Key Pattern when requesting a transmission on
 *                            dequeue, @ref buffer_dequeue.
 * @return     NRF_SUCCESS on success, else an error code indicating reason for failure.
 */
static uint32_t buffer_enqueue(uint8_t    * p_key_pattern,
                               uint16_t     pattern_len,
                               uint16_t     offset)
{
    buffer_entry_t * element;
    uint32_t         err_code = NRF_SUCCESS;

    if (BUFFER_LIST_FULL())
    {
        // Element cannot be buffered.
        err_code = NRF_ERROR_NO_MEM;
    }
    else
    {
        // Make entry of buffer element and copy data.
        element              = &buffer_list.buffer[(buffer_list.wp)];
        element->p_data      = p_key_pattern;
        element->data_offset = offset;
        element->data_len    = pattern_len;

        buffer_list.count++;
        buffer_list.wp++;

        if (buffer_list.wp == MAX_BUFFER_ENTRIES)
        {
            buffer_list.wp = 0;
        }
    }

    return err_code;
}


/**@brief   Function to dequeue key scan patterns that could not be transmitted either completely of
 *          partially.
 *
 * @warning This handler is an example only. You need to analyze how you wish to send the key
 *          release.
 *
 * @param[in]  tx_flag   Indicative of whether the dequeue should result in transmission or not.
 * @note       A typical example when all keys are dequeued with transmission is when link is
 *             disconnected.
 *
 * @return     NRF_SUCCESS on success, else an error code indicating reason for failure.
 */
static uint32_t buffer_dequeue(bool tx_flag)
{
    buffer_entry_t * p_element;
    uint32_t         err_code = NRF_SUCCESS;
    uint16_t         actual_len;

    if (BUFFER_LIST_EMPTY())
    {
        err_code = NRF_ERROR_NOT_FOUND;
    }
    else
    {
        bool remove_element = true;

        p_element = &buffer_list.buffer[(buffer_list.rp)];

        if (tx_flag)
        {
            err_code = send_key_scan_press_release(p_element->p_data,
                                                   p_element->data_len,
                                                   p_element->data_offset,
                                                   &actual_len);
            // An additional notification is needed for release of all keys, therefore check
            // is for actual_len <= element->data_len and not actual_len < element->data_len
            if ((err_code == NRF_ERROR_RESOURCES) && (actual_len <= p_element->data_len))
            {
                // Transmission could not be completed, do not remove the entry, adjust next data to
                // be transmitted
                p_element->data_offset = actual_len;
                remove_element         = false;
            }
        }

        if (remove_element)
        {
            BUFFER_ELEMENT_INIT(buffer_list.rp);

            buffer_list.rp++;
            buffer_list.count--;

            if (buffer_list.rp == MAX_BUFFER_ENTRIES)
            {
                buffer_list.rp = 0;
            }
        }
    }

    return err_code;
}
#endif

/**@brief Function for sending sample key presses to the peer.
 *
 * @param[in]   key_pattern_len   Pattern length.
 * @param[in]   p_key_pattern     Pattern to be sent.
 */
static void keys_send(uint8_t key_pattern_len, uint8_t * p_key_pattern)
{
	//wright : 0415 
		NRF_LOG_INFO("----- ENTER keys_send -----");
		NRF_LOG_INFO("Num_Key: %d",key_pattern_len);
		NRF_LOG_HEXDUMP_INFO(p_key_pattern, 6);
	
	
    ret_code_t err_code;
    uint16_t actual_len;

    err_code = send_key_scan_press_release(p_key_pattern,
                                           key_pattern_len,
                                           0,
                                           &actual_len);
    // An additional notification is needed for release of all keys, therefore check
    // is for actual_len <= key_pattern_len and not actual_len < key_pattern_len.
/*	
    if ((err_code == NRF_ERROR_RESOURCES) && (actual_len <= key_pattern_len))		
    {
        // Buffer enqueue routine return value is not intentionally checked.
        // Rationale: Its better to have a a few keys missing than have a system
        // reset. Recommendation is to work out most optimal value for
        // MAX_BUFFER_ENTRIES to minimize chances of buffer queue full condition
        UNUSED_VARIABLE(buffer_enqueue(p_key_pattern, key_pattern_len, actual_len));
    }

*/
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != NRF_ERROR_RESOURCES) &&
        (err_code != NRF_ERROR_BUSY) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
       )
    {
        APP_ERROR_HANDLER(err_code);
    }
		
}

//wright : fix for not detecting modifier keys.
static void keys_send_fix(uint8_t key_pattern_len, uint8_t * p_key_pattern,uint8_t p_modifier)
{
		//wright : 0415
		NRF_LOG_INFO("----- ENTER keys_send_fix -----");
		NRF_LOG_INFO("Num_Key: %d",key_pattern_len);
		NRF_LOG_HEXDUMP_INFO(p_key_pattern, 6);
	
	
    ret_code_t err_code;
    uint16_t actual_len;

    err_code = send_key_scan_press_release_fix(p_key_pattern,
                                           key_pattern_len,
                                           0,
                                           &actual_len,
																						p_modifier);

    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != NRF_ERROR_RESOURCES) &&
        (err_code != NRF_ERROR_BUSY) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
       )
    {
        APP_ERROR_HANDLER(err_code);
    }
		
}

/**@brief Function for handling the HID Report Characteristic Write event.
 *
 * @param[in]   p_evt   HID service event.
 */
static void on_hid_rep_char_write(uint8_t report_val, uint8_t length)
{
	 ret_code_t err_code;
	
            if (!m_caps_on && ((report_val & OUTPUT_REPORT_BIT_MASK_CAPS_LOCK) != 0))
            {
                // Caps Lock is turned On.
                NRF_LOG_INFO("Caps Lock is turned On!");
                
								//wright : fix for not to infect GPIO output
								//err_code = bsp_indication_set(BSP_INDICATE_ALERT_3);
							
                APP_ERROR_CHECK(err_code);
								
                keys_send(sizeof(m_caps_on_key_scan_str), m_caps_on_key_scan_str);
                m_caps_on = true;
            }
            else if (m_caps_on && ((report_val & OUTPUT_REPORT_BIT_MASK_CAPS_LOCK) == 0))
            {
                // Caps Lock is turned Off .
                NRF_LOG_INFO("Caps Lock is turned Off!");
							
								//wright : fix for not to infect GPIO output
								//err_code = bsp_indication_set(BSP_INDICATE_ALERT_OFF);
							
                APP_ERROR_CHECK(err_code);

                keys_send(sizeof(m_caps_off_key_scan_str), m_caps_off_key_scan_str);
                m_caps_on = false;
            }
            else
            {
                // The report received is not supported by this application. Do nothing.
            }
}

/**@brief Function for the Event Scheduler initialization.
 */
static void scheduler_init(void)
{
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    ret_code_t err_code;
		//wright : fix for not to infect GPIO output
    //err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
		//wright : fix for not to infect GPIO output
    //err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);
	
	  NRF_LOG_INFO("Go to sleep zzzzz");	
	       
		 m_coms_ble_sleep_mode_enter();		 
}



/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    app_sched_execute();
	
	  if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}


static void m_coms_ble_evt_handler(void * p_event_data, uint16_t event_size)
{
	 m_coms_ble_evt_t * p_evt = (m_coms_ble_evt_t *) p_event_data;;
   uint32_t           err_code;
   uint8_t            report;
	

	 switch (p_evt->type)
   {
		 
		 case M_COMS_BLE_EVT_ADV_DIRECTED_HIGH_DUTY:			 
		      NRF_LOG_INFO("High Duty Directed advertising.");
          //wright : fix for not to infect GPIO output
					//err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_DIRECTED);
          //APP_ERROR_CHECK(err_code);
          break;
		 
		 case M_COMS_BLE_EVT_ADV_DIRECTED:
			    NRF_LOG_INFO("Directed advertising.");
          //wright : fix for not to infect GPIO output
					//err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_DIRECTED);
          //APP_ERROR_CHECK(err_code);	 
		      break;
		 
		 case M_COMS_BLE_EVT_ADV_FAST:			   
          NRF_LOG_INFO("Fast advertising.");
          //wright : fix for not to infect GPIO output
					//err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
          //APP_ERROR_CHECK(err_code);
		      break;
		 
		 case M_COMS_BLE_EVT_ADV_SLOW:
			    NRF_LOG_INFO("Slow advertising.");
					//wright : fix for not to infect GPIO output
          //err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_SLOW);
          //APP_ERROR_CHECK(err_code);
		      break;
		 
		 case M_COMS_BLE_EVT_ADV_FAST_WHITELIST:
			    NRF_LOG_INFO("Fast advertising with whitelist.");
          //wright : fix for not to infect GPIO output
					//err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_WHITELIST);
          //APP_ERROR_CHECK(err_code);
		      break;
		 
		 case M_COMS_BLE_EVT_ADV_SLOW_WHITELIST:
			    NRF_LOG_INFO("Slow advertising with whitelist.");
          //wright : fix for not to infect GPIO output
					//err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_WHITELIST);
          //APP_ERROR_CHECK(err_code);
		      break;
			
		 case M_COMS_BLE_EVT_ADV_IDLE:
			 //wright : debug		
		 NRF_LOG_INFO("-----M_COMS_BLE_EVT_ADV_IDLE-----");
			    //wright : fix for not to infect GPIO output
					//err_code = bsp_indication_set(BSP_INDICATE_IDLE);
          //APP_ERROR_CHECK(err_code);
		      sleep_mode_enter();
          break;
         
		 
		 case M_COMS_BLE_EVT_CONNECTED:
			    NRF_LOG_INFO("Connected");
          //wright : fix for not to infect GPIO output
					//err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
          //APP_ERROR_CHECK(err_code);	
		      break;
		 
		 case M_COMS_BLE_EVT_DISCONNECTED:
			 
					//wright : TODO find the reason about disconection.
		      NRF_LOG_INFO("Disconnected");
          // Dequeue all keys without transmission.
          //(void) buffer_dequeue(false);	        
          // Reset m_caps_on variable. Upon reconnect, the HID host will re-send the Output
          // report containing the Caps lock state.
          m_caps_on = false;
          // disabling alert 3. signal - used for capslock ON
		 
          //wright : fix for not to infect GPIO output
					//err_code = bsp_indication_set(BSP_INDICATE_ALERT_OFF);
          //APP_ERROR_CHECK(err_code);	
		      //bsp_board_led_off(BSP_LED_INDICATE_USER_LED1);
		      //bsp_board_led_off(BSP_LED_INDICATE_USER_LED2);
		 
          break;
		 
		 case M_COMS_BLE_EVT_DATA_RECEIVED:			 
		      report = *p_evt->data_received.data;
			    on_hid_rep_char_write(report, p_evt->data_received.len); 
		      break;
		 
		 case M_COMS_BLE_EVT_BONDED:
			    NRF_LOG_INFO("Bonded");
			    //is_bonded = true;
		 
		 
		      //wright : fix for not to infect GPIO output
					//bsp_board_led_on(BSP_LED_INDICATE_USER_LED2);		
		 
		      break;
		 
		 default:
		 break;
		 
	 }	 
}   


static void m_coms_gzll_evt_handler(void * p_event_data, uint16_t event_size)
{
	  gzll_event_t * p_evt = (gzll_event_t *) p_event_data;
	
	
	  switch (*p_evt)
    {
        case gzll_event_search_for_host:
	           NRF_LOG_INFO("Search for Host.");
//             bsp_board_led_invert(BSP_LED_INDICATE_USER_LED3); 
				     app_timer_start(m_app_gzll_search_timer_id, APP_GZLL_SEARCH_INTERVAL, NULL);
             break;				
						  
			  case gzll_event_sysaddr_exchanged:
					   NRF_LOG_INFO("Paired with Host");
//             bsp_board_led_on(BSP_LED_INDICATE_USER_LED3);
				     break;
				
				case gzll_event_hostid_exchanged:
					   NRF_LOG_INFO("Bonded with Host");
			       //is_bonded = true;
//		         bsp_board_led_on(BSP_LED_INDICATE_USER_LED4);		
				     break;
				
				case gzll_event_tx_failed:
//					    NRF_LOG_INFO("GZLL transmit failed !!");
//				      bsp_board_led_off(BSP_LED_INDICATE_USER_LED3);	
//				      bsp_board_led_off(BSP_LED_INDICATE_USER_LED4);	
				     break;
				
				default:
					   break;
				
		}		
}	

/**@brief Function for initializing 
 */
static void modules_init(void)
{		
   //buffer_init();
	 sensor_simulator_init();
   power_management_init();

	 
	if(m_protocol_mode == M_COMS_BLE_MODE_PAIR_1 || m_protocol_mode == M_COMS_BLE_MODE_PAIR_2)
	{	
     m_coms_ble_init(m_coms_ble_evt_handler);
  }
  else
	{	
	  clock_start();
	  m_coms_gzll_init(m_coms_gzll_evt_handler);
		
	}	

}	


static void modules_enable(bool erase_bonds)
{

  if(m_protocol_mode == M_COMS_BLE_MODE_PAIR_1 || m_protocol_mode == M_COMS_BLE_MODE_PAIR_2)
	{	
	
		m_coms_ble_enable(erase_bonds);
		 NRF_LOG_INFO("BLE mode start.");
	}	
	else
	{				
	   m_coms_gzll_enable(erase_bonds);
		 m_coms_gzll_search_for_host();
     NRF_LOG_INFO("GZLL mode start.");		
	}	
		
}

static void protocol_det_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
	uint8_t level = nrf_gpio_pin_read(PROTOCOL_DETECT_PIN);
	uint8_t is_softdevice_enabled;
	
  if (pin == PROTOCOL_DETECT_PIN)
	{
	
    NRF_LOG_INFO("Reset!!");
	
    nrf_delay_ms(100);
		
		sd_softdevice_is_enabled(&is_softdevice_enabled);
		
		if (is_softdevice_enabled==1)
		{	
		  sd_nvic_SystemReset();
		}
		else
		{
      NVIC_SystemReset();
    }			

  }		
	
	
}	




/************************************* wright : add keyboard function **************************************/


/**@brief Handler function for keyboard module events
 */
//wright : for mode selection

static uint8_t KEY_CODE_GARZLL_MODE = 0xAA;
static uint8_t KEY_CODE_BLE_MODE_PAIR_1 = 0xAB;
static uint8_t KEY_CODE_BLE_MODE_PAIR_2 = 0xAC;
bool IS_KEY_CODE_MODE_PRESSED = false;

static void mode_change_key_check(m_keyboard_data_t * keyboard_pkt)
{

		//check if keypackage contain change mode key code
		if( keyboard_pkt->keys[0]== KEY_CODE_GARZLL_MODE)
		{
			NRF_LOG_INFO("---PRESS KEY_CODE_GARZLL_MODE !---");
			IS_KEY_CODE_MODE_PRESSED = true;
			current_transmit_mode_data[0] = M_COMS_GZLL_MODE;
		}
			
		if(keyboard_pkt->keys[0] == KEY_CODE_BLE_MODE_PAIR_1)	
		{
			NRF_LOG_INFO("---PRESS KEY_CODE_BLE_MODE_PAIR_1 !---");
			IS_KEY_CODE_MODE_PRESSED = true;
			current_transmit_mode_data[0] = M_COMS_BLE_MODE_PAIR_1;
		}
		if(keyboard_pkt->keys[0] == KEY_CODE_BLE_MODE_PAIR_2)	
		{
			NRF_LOG_INFO("---PRESS KEY_CODE_BLE_MODE_PAIR_2 !---");
			IS_KEY_CODE_MODE_PRESSED = true;
			current_transmit_mode_data[0] = M_COMS_BLE_MODE_PAIR_2;
		}		
		
		//if change key code pressed ; write it to NRF_POWER->GPREGRET 
		if(IS_KEY_CODE_MODE_PRESSED)
		{
			m_keyboard_polling_disable();
			
			change_mode_is_start = true;
			
			NRF_POWER->GPREGRET = current_transmit_mode_data[0];
			
			NRF_LOG_INFO("--- Write current_transmit_mode_data: %d to NRF_POWER->GPREGRET !---",
			current_transmit_mode_data[0]);
		
			//reset after save the mode info
			sd_softdevice_is_enabled(&is_softdevice_enabled);
			
			if (is_softdevice_enabled==1)
			{	
				sd_nvic_SystemReset();
			}
			else
			{
				NVIC_SystemReset();
			}		
			
		}
			
}

//wright 0424: for key swipe problem  
static void gzll_keep_alive_handler()
{
	// Put the previous keyboard packet in buffer.
	//(differ from dt2 :  buffer_keys(&s_gzll_keepalive_keyboard_pkt);  )

		keys_send_fix(s_gzll_keepalive_keyboard_pkt.num_keys,
									s_gzll_keepalive_keyboard_pkt.keys,
									s_gzll_keepalive_keyboard_pkt.modifier_keys);	
}


static void m_keyboard_handler(void * p_event_data, uint16_t event_size)
{
    uint32_t            err_code;
    m_keyboard_data_t * keyboard_pkt;

    ASSERT(event_size == sizeof(m_keyboard_data_t));
    
    keyboard_pkt = (m_keyboard_data_t *) p_event_data;
	
		NRF_LOG_INFO("---Enter m_keyboard_handler!---");

	
		if (keyboard_pkt->pairing_button)
    {
			//do nothing
		}
		else
		{
			//wright : change mode
				if(!change_mode_is_start)
				{
					mode_change_key_check(keyboard_pkt);
				}
				
				keys_send_fix(keyboard_pkt->num_keys,keyboard_pkt->keys,keyboard_pkt->modifier_keys);
				
				//wright 0424: for key swipe problem  
				if(current_transmit_mode_data[0] == M_COMS_GZLL_MODE)
				{
					memcpy(&s_gzll_keepalive_keyboard_pkt, keyboard_pkt, sizeof(m_keyboard_data_t));
					if (m_keyboard_packet_is_empty(keyboard_pkt))
					{
							app_timer_stop(s_gzll_keep_alive_id);
					}
					else
					{
							app_timer_start(s_gzll_keep_alive_id, APP_TIMER_TICKS(500),0);
					}
				}
		}

    m_pwr_mgmt_feed();
}





void keyboard_module_enable(void)
{
    uint32_t err_code;

	
    err_code = m_keyboard_init(keyboard_format_usbhid, m_keyboard_handler);    
    APP_ERROR_CHECK(err_code); 
    m_keyboard_matrix_enable();
	
	return;


}


/************************************* wright : add keyboard function end**************************************/








//wright : get current mode (ble/gazll) for power manager module
bool get_current_mode()
{
    return m_protocol_mode;
}







/********************************* wright : FDS part ******************************/



static void fds_evt_handler(fds_evt_t const * p_evt)
{
    switch (p_evt->id)
    {
        case FDS_EVT_INIT:
            if (p_evt->result == FDS_SUCCESS)
            {
								NRF_LOG_INFO("FDS_EVT_INIT Success:");
                m_fds_initialized = true;
            }
            break;

        case FDS_EVT_WRITE:
        {
            if (p_evt->result == FDS_SUCCESS)
            {		
								NRF_LOG_INFO("FDS_EVT_WRITE Success:");
                NRF_LOG_INFO("Record ID:\t0x%04x",  p_evt->write.record_id);
                NRF_LOG_INFO("File ID:\t0x%04x",    p_evt->write.file_id);
                NRF_LOG_INFO("Record key:\t0x%04x", p_evt->write.record_key);
								fds_busy_flag = false;
            }
        } break;

        case FDS_EVT_DEL_RECORD:
        {
            if (p_evt->result == FDS_SUCCESS)
            {
								NRF_LOG_INFO("FDS_EVT_DEL_RECORD Success:");
                NRF_LOG_INFO("Record ID:\t0x%04x",  p_evt->del.record_id);
                NRF_LOG_INFO("File ID:\t0x%04x",    p_evt->del.file_id);
                NRF_LOG_INFO("Record key:\t0x%04x", p_evt->del.record_key);
								fds_busy_flag = false;
            }
        } break;
				
				case FDS_EVT_UPDATE:
				{
            if (p_evt->result == FDS_SUCCESS)
            {
								NRF_LOG_INFO("FDS_EVT_UPDATE Success:");
                NRF_LOG_INFO("Record ID:\t0x%04x",  p_evt->del.record_id);
                NRF_LOG_INFO("File ID:\t0x%04x",    p_evt->del.file_id);
                NRF_LOG_INFO("Record key:\t0x%04x", p_evt->del.record_key);
								fds_busy_flag = false;
								//wright: 0418
								NRF_LOG_INFO("====Read file data after \"FDS_EVT_UPDATE Success:\"====");
								//mode_record_read();
            }
        } break;

        default:
            break;
    }
}



static ret_code_t mode_record_delete(void)
{
		fds_busy_flag = true;
    fds_find_token_t tok   = {0};
    fds_record_desc_t desc = {0};
		ret_code_t rc = REC_DEFAULT_VALUE;

    if (fds_record_find(LAST_TRANSMIT_MODE_FILE_ID, LAST_TRANSMIT_MODE_RECORD_KEY, &desc, &tok) == FDS_SUCCESS)
    {
        rc = fds_record_delete(&desc);
        if (rc != FDS_SUCCESS)
        {
						NRF_LOG_INFO("error: fds_record_delete %x",rc);

            return rc;
        }

//        NRF_LOG_INFO("SUCCESS: fds_record_delete -- fid = %X,  rk= %X ",
//				LAST_TRANSMIT_MODE_FILE_ID,
//				LAST_TRANSMIT_MODE_RECORD_KEY);
    }
    else
    {
        NRF_LOG_INFO("record not found!\r\n");
				rc = DELETE_RECORD_NOT_FOUND;
				fds_busy_flag = false;
    }
		
		return rc;
}

static ret_code_t mode_record_write(uint32_t mode_value)
{
		fds_busy_flag = true;
    fds_record_t const rec =
    {
        .file_id           = LAST_TRANSMIT_MODE_FILE_ID,
        .key               = LAST_TRANSMIT_MODE_RECORD_KEY,
        .data.p_data       = &mode_value,
        .data.length_words = 1
    };


    ret_code_t rc = fds_record_write(NULL, &rec);
    if (rc != FDS_SUCCESS)
    {
			NRF_LOG_INFO("fds_record_write failed... error code %d",rc);
    }
		return rc;
}

static ret_code_t mode_record_update(uint32_t mode_value)
{
		//record find
		fds_record_desc_t desc = {0};
		fds_find_token_t  tok  = {0};
		ret_code_t rc;
		fds_flash_record_t frec = {0};
			
		rc=fds_record_find(LAST_TRANSMIT_MODE_FILE_ID,LAST_TRANSMIT_MODE_RECORD_KEY,&desc,&tok);
	
		if (rc != FDS_SUCCESS)
    {
			NRF_LOG_INFO("fds_record_find failed... error code %d",rc);
			return rc;
    }
		
		
		//record update
		fds_busy_flag = true;
    fds_record_t const rec =
    {
        .file_id           = LAST_TRANSMIT_MODE_FILE_ID,
        .key               = LAST_TRANSMIT_MODE_RECORD_KEY,
        .data.p_data       = &mode_value,
        .data.length_words = 1//(2 + 3) / sizeof(uint32_t)
    };
		
		
		NRF_LOG_INFO("==== mode data before update ==== : %d",mode_value);
	
		rc = fds_record_update(&desc, &rec);
		
    if (rc != FDS_SUCCESS)
    {
			NRF_LOG_INFO("fds_record_update failed... error code %d",rc);
			return rc;
    }
		
		
		return rc;
}




static ret_code_t mode_record_read(void)
{

    fds_record_desc_t desc = {0};
    fds_find_token_t  tok  = {0};
		ret_code_t rc;
		fds_flash_record_t frec = {0};
		//uint32_t data_read[1] ={0};
			
		rc=fds_record_find(LAST_TRANSMIT_MODE_FILE_ID,LAST_TRANSMIT_MODE_RECORD_KEY,&desc,&tok);
		

		
		if (rc == FDS_SUCCESS)
		{
				rc = fds_record_open(&desc, &frec);
				if(rc == FDS_SUCCESS)
				{
					current_transmit_mode_data[0] = *(uint32_t *)frec.p_data; 
					NRF_LOG_INFO("Hex dump the contents of p_data\n");
					NRF_LOG_HEXDUMP_INFO(frec.p_data, 4);
					NRF_LOG_INFO("current_transmit_mode_data = %x",current_transmit_mode_data[0]);
					rc = fds_record_close(&desc);			
				}
				else
				{
					NRF_LOG_INFO("open failed!");
				}
			
		}
		//wright : if file not find, create one (Default mode).
		else if (rc == FDS_ERR_NOT_FOUND)
		{
			NRF_LOG_INFO("record not found! , write Default mode into flash...");
			mode_record_write(M_COMS_GZLL_MODE);
		}

		
		return rc;
}


/**@brief   Wait for fds to initialize. */
static void wait_for_fds_ready(void)
{
    while (!m_fds_initialized)
    {
        idle_state_handle();
    }
}





//wright: mode selection 
static uint32_t mode_selection()
{
		uint32_t mode_result = 0;
		ret_code_t ret;

		NRF_LOG_INFO("--- record read ---");
		//read last transmitted mode from gpregret register
		mode_result = NRF_POWER->GPREGRET;
	
		switch(mode_result)
		{
			case 0:
				NRF_LOG_INFO("--- Default Mode : Gazll ---");
				m_protocol_mode= M_COMS_GZLL_MODE;
				break;
			case 1:
				NRF_LOG_INFO("--- BLE Mode : Pairing Device 1 ---");
				m_protocol_mode= M_COMS_BLE_MODE_PAIR_1;
				break;
			case 2:
				NRF_LOG_INFO("--- BLE Mode : Pairing Device 2 ---");
				m_protocol_mode= M_COMS_BLE_MODE_PAIR_2;
				break;
			default :
				NRF_LOG_INFO("--- Default Mode : Gazll ---");
				break;		
		}
		
		return mode_result;
}
/********************************* wright : FDS part end ******************************/







/**@brief Function for application main entry.
 */
int main(void)
{
	//wright : 0422 keep bonding information 
    bool erase_bonds = false;  //bool erase_bonds;  

    // Initialize.
    log_init();

		//wright : Mode selection
		mode_selection();

    timers_init();	

    scheduler_init();
		
    modules_init();

    NRF_LOG_INFO("Dual mode keyboard example started.");

    modules_enable(erase_bonds);

    application_timers_start();
		
		

    //wright : enable keyboard module.
    keyboard_module_enable();

   

    // Enter main loop.
    for (;;)
    {
    idle_state_handle();
    }
}



















