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
//#define M_COMS_BLE_MODE                      0
//#define M_COMS_GZLL_MODE                     1																							 
#define M_COMS_BLE_MODE                      1
#define M_COMS_GZLL_MODE                     0
																							 
																							 
#define PROTOCOL_DETECT_PIN                  3

																							 
#define APP_GZLL_SEARCH_INTERVAL             APP_TIMER_TICKS(1000)
																							 
																							 
																				 
																							 

/********************************* wright : header and defines ******************************/

#include "m_keyboard.h"
#include "m_pwr_and_clk_mgmt.h"



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


static bool              m_protocol_mode;
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
    if(m_protocol_mode == M_COMS_BLE_MODE)
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
   if(m_protocol_mode == M_COMS_BLE_MODE)
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

				if (m_protocol_mode == M_COMS_BLE_MODE)
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

    do
    {
        // Reset the data buffer.
        memset(data, 0, sizeof(data));

        // Copy the scan code.
        memcpy(data + SCAN_CODE_POS + offset, p_key_pattern + offset, data_len - offset);


				//****** wright : before sending keypacket, merge it with modifier keys. *******//
				if (p_modifer != 0x00)
            data[MODIFIER_KEY_POS] |= p_modifer;
				
				
				if (m_protocol_mode == M_COMS_BLE_MODE)
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
    ret_code_t err_code;
    uint16_t actual_len;

    err_code = send_key_scan_press_release_fix(p_key_pattern,
                                           key_pattern_len,
                                           0,
                                           &actual_len,
																						p_modifier);
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


//wright : fix for not to infect GPIO output

/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated when button is pressed.
 */
//static void bsp_event_handler(bsp_event_t event)
//{
// //ret_code_t err_code;
//	static uint8_t * p_key = m_sample_key_press_scan_str;
//  static uint8_t   size  = 0;
//  
//  switch (event)
//    {
//       case BSP_EVENT_SLEEP:
//            //sleep_mode_enter();
//            break;
//			 
//			 case BSP_EVENT_DISCONNECT:
//            m_coms_ble_diconnect();
//			    
//            break;
//			 
//			 case BSP_EVENT_WHITELIST_OFF:
//			      m_coms_ble_whitelist_off();
//			 
//			      break;
//			 
//			 case BSP_EVENT_KEY_0:
//            //if (is_bonded)
//            //{
//                keys_send(2, p_key);
//                p_key++;
//                size++;
//                if (size == MAX_KEYS_IN_ONE_REPORT)
//                {
//                    p_key = m_sample_key_press_scan_str;
//                    size  = 0;
//                }
//            //}
//            break;
//        default:
//            break;
//    }
//}


//wright : fix for not to infect GPIO output
///**@brief Function for initializing buttons and leds.
// *
// * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
// */
//static void buttons_leds_init(bool * p_erase_bonds)
//{
//    ret_code_t err_code;
//    bsp_event_t startup_event;

//    err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
//    APP_ERROR_CHECK(err_code);

//    err_code = bsp_btn_ble_init(NULL, &startup_event);
//    APP_ERROR_CHECK(err_code);

//    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
//}


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

	 
	if(m_protocol_mode == M_COMS_BLE_MODE)
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

  if(m_protocol_mode == M_COMS_BLE_MODE)
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




/**@brief Initialization of unused I/Os
 */
static void misc_io_init(void)
{
    ret_code_t err_code; 
	  int i;
    uint32_t unused_pins[] = {0, 1, 2, 4, 5, 6, 7, 8, 9, 10, 11, 12, 26, 27, 28, 29, 30, 31};  
    uint8_t  level;   

    for (i = 0; i < (sizeof(unused_pins) / sizeof(unused_pins[0])); ++i)
    {
        uint32_t pin = unused_pins[i];
        
        NRF_GPIO->PIN_CNF[pin] =
            (GPIO_PIN_CNF_DIR_Input        << GPIO_PIN_CNF_DIR_Pos)    |
            (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos)  |
            (GPIO_PIN_CNF_PULL_Disabled    << GPIO_PIN_CNF_PULL_Pos)   |
            (GPIO_PIN_CNF_DRIVE_S0S1       << GPIO_PIN_CNF_DRIVE_Pos)  |
            (GPIO_PIN_CNF_SENSE_Disabled   << GPIO_PIN_CNF_SENSE_Pos);           
    }     
		
				//Set P0.03 as a protocol detect input pin, default GZLL, GND to BLE
		   NRF_GPIO->PIN_CNF[PROTOCOL_DETECT_PIN] =
		        (GPIO_PIN_CNF_DIR_Input        << GPIO_PIN_CNF_DIR_Pos)    |
            (GPIO_PIN_CNF_INPUT_Connect    << GPIO_PIN_CNF_INPUT_Pos)  |
            (GPIO_PIN_CNF_PULL_Pullup      << GPIO_PIN_CNF_PULL_Pos)   |
            (GPIO_PIN_CNF_DRIVE_S0S1       << GPIO_PIN_CNF_DRIVE_Pos)  |
            (GPIO_PIN_CNF_SENSE_Disabled   << GPIO_PIN_CNF_SENSE_Pos);
		
		level = nrf_gpio_pin_read(PROTOCOL_DETECT_PIN);
		
		if (!nrf_drv_gpiote_is_init())
    {
		   err_code = nrf_drv_gpiote_init();
       APP_ERROR_CHECK(err_code);
		}
		//Set P0.03 as a protocol detect input pin, VDD to GZLL, GND to BLE
		memset(&protocol_detect_pin_cfg, 0, sizeof(protocol_detect_pin_cfg));
		protocol_detect_pin_cfg.hi_accuracy = true;
	  
		protocol_detect_pin_cfg.sense = (level==M_COMS_BLE_MODE)? NRF_GPIOTE_POLARITY_LOTOHI : NRF_GPIOTE_POLARITY_HITOLO  ;
		protocol_detect_pin_cfg.pull  = (level==M_COMS_BLE_MODE)? NRF_GPIO_PIN_PULLDOWN : NRF_GPIO_PIN_PULLUP ;
		
		NRF_LOG_INFO("protocol detect pin toggled, level = %d", nrf_gpio_pin_read(PROTOCOL_DETECT_PIN) );
		
		err_code = nrf_drv_gpiote_in_init(PROTOCOL_DETECT_PIN, &protocol_detect_pin_cfg, protocol_det_pin_handler);
		APP_ERROR_CHECK(err_code);
		
		nrf_drv_gpiote_in_event_enable(PROTOCOL_DETECT_PIN, true);
	
}			


/************************************* wright : add keyboard function **************************************/


/**@brief Handler function for keyboard module events
 */
static void m_keyboard_handler(void * p_event_data, uint16_t event_size)
{
    uint32_t            err_code;
    m_keyboard_data_t * keyboard_pkt;
    
    ASSERT(event_size == sizeof(m_keyboard_data_t));
    
    keyboard_pkt = (m_keyboard_data_t *) p_event_data;
	
	//wright : Try to send key value by 52 series way
	// 1.Try to get data from m_keyborad_handler (Before HW ready, Use Fake Data)
	// 2.figure out the format of data
	// 3.try to reproduce the sending proccess of 52 series
		NRF_LOG_INFO("---Enter m_keyboard_handler!----");
	
	//wright : send a fake key package 
//		m_keyboard_data_t new_key_data;
//		m_keyboard_data_t * fake_key_data = &new_key_data; 

		//fake data : wright 
//		uint8_t fake_str[M_KEYBOARD_MAX_NUM_KEYS] = /**< Key pattern to be sent when the key press button has been pushed. */
//		{
//				0xE1,       //shift key code
//				0x00,       
//				0x00,       
//				0x00,       
//				0x00,       
//				0x00        
//		};
//		fake_key_data->modifier_keys = 0x02;
//		memcpy(fake_key_data->keys,fake_str,M_KEYBOARD_MAX_NUM_KEYS);
//		fake_key_data->num_keys=0x01;
//		fake_key_data->pairing_button=0x00;
//	
		if (keyboard_pkt->pairing_button)
    {
			//do nothing
		}
		else
		{
	
				
				//wright : fake key package	
//				NRF_LOG_INFO("---Keys_send---");
//				NRF_LOG_INFO("Num_Key: %d",fake_key_data->num_keys);
//				NRF_LOG_HEXDUMP_INFO(fake_key_data->keys, 6);
//				keys_send(fake_key_data->num_keys,fake_key_data->keys);
			
				//wright : actual key package
				
				NRF_LOG_INFO("Num_Key: %d",keyboard_pkt->num_keys);
				NRF_LOG_HEXDUMP_INFO(keyboard_pkt->keys, 6);
			

				NRF_LOG_INFO("---Keys_send---");
				keys_send_fix(keyboard_pkt->num_keys,keyboard_pkt->keys,keyboard_pkt->modifier_keys);
				
		}

    
//    if (keyboard_pkt->pairing_button)
//    {
//        uint8_t release_report[8] = {0};
//        
//        app_timer_stop(s_gzll_keep_alive_id);
//        
//        // Stop buffer timer and flush
//        if (s_buffer_timer_running)
//        {
//            err_code = app_timer_stop(s_packet_buffer_id);
//            APP_ERROR_CHECK(err_code);
//            s_buffer_timer_running = false;
//        }
//        
//        s_packet_buffer.start_idx = 0;
//        s_packet_buffer.end_idx = 0;
//        
//        if(s_connection_state == state_connected)
//        {
//            // Send release packet
//            m_coms_hid_report_send(release_report, 
//                                   sizeof(release_report), 
//                                   0,
//                                   s_hid_reports.keyboard_rep_idx);     
//       }                               
//        // Start bonding
//        m_coms_bonding_start();
//    }
//    else
//    {
//        if (s_waiting_for_passkey)
//        {
//            if (keyboard_pkt->num_keys > 0)
//            {
//                memcpy(&s_passkey_buf[s_passkey_idx], keyboard_pkt->keys, keyboard_pkt->num_keys);
//                s_passkey_idx += keyboard_pkt->num_keys;
//                
//                if (s_passkey_idx >= 6)
//                {
//                    err_code = m_coms_ble_passkey_set(s_passkey_buf);
//                    APP_ERROR_CHECK(err_code);
//                    s_passkey_idx = 0;
//                }
//            }
//        }
//        else
//        {
//            buffer_keys(keyboard_pkt);    
//            memcpy(&s_gzll_keepalive_keyboard_pkt, keyboard_pkt, sizeof(m_keyboard_data_t));
//        
//            if (s_protocol_mode == protocol_mode_gzll)
//            {
//                if (m_keyboard_packet_is_empty(keyboard_pkt))
//                {
//                    app_timer_stop(s_gzll_keep_alive_id);
//                }
//                else
//                {
//                    app_timer_start(s_gzll_keep_alive_id, APP_TIMER_TICKS(500, APP_TIMER_PRESCALER), 0);
//                }
//            }
//        }
//    }
    // Notifying the power manager of activity
    m_pwr_mgmt_feed();
}





void keyboard_module_enable(void)
{
    uint32_t err_code;

	//wright : waiting for Jennifer's reply
	
//    err_code = m_coms_ble_report_descriptor_add(s_keyboard_hid_descriptor, sizeof(s_keyboard_hid_descriptor), ble_boot_pkt_keyboard, &s_hid_interface_idx);
//    APP_ERROR_CHECK(err_code);
//    
//    err_code = m_coms_ble_report_id_map(s_hid_interface_idx, hid_report_type_input, false, 0, 8, &s_hid_reports.keyboard_rep_idx);
//    APP_ERROR_CHECK(err_code);
//    
//    err_code = m_coms_ble_report_id_map(s_hid_interface_idx, hid_report_type_output, false, 0, 1, &s_hid_reports.output_rep_idx);
//    APP_ERROR_CHECK(err_code);
//    
//    err_code = m_coms_ble_report_id_map(s_hid_interface_idx, hid_report_type_feature, false, 0, 2, &s_hid_reports.feature_rep_idx);
//    APP_ERROR_CHECK(err_code);
        // Keyboard module. 
	
	
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



//wright: mode selection 


/**@brief Function for application main entry.
 */
int main(void)
{
    bool erase_bonds;  

    // Initialize.
    log_init();

   // misc_io_init();    //For protocol mode setting & unused pins	
	
	
	//wright : Mode selection
	//mode_selection();
	
	

    m_protocol_mode = M_COMS_GZLL_MODE;//((nrf_gpio_pin_read(PROTOCOL_DETECT_PIN) == 0)?M_COMS_BLE_MODE:M_COMS_GZLL_MODE) ;

    timers_init();	

    scheduler_init();
		
		//wright : fix for not to infect GPIO output
    //buttons_leds_init(&erase_bonds);

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


/**
 * @}
 */
