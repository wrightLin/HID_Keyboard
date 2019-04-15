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

#include "bsp_btn_ble.h"

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


#define SCHED_QUEUE_SIZE                     15
#define SCHED_MAX_EVENT_DATA_SIZE            MAX(sizeof(ble_evt_t), MAX(sizeof(app_timer_event_t) \
                                               , sizeof(m_coms_ble_evt_t)))              /**< Maximum size of scheduler events. */

#define MODIFIER_KEY_POS                     0                                          /**< Position of the modifier byte in the Input Report. */
#define SCAN_CODE_POS                        2                                          /**< The start position of the key scan code in a HID Report. */
#define SHIFT_KEY_CODE                       0x02                                       /**< Key code indicating the press of the Shift Key. */

#define MAX_KEYS_IN_ONE_REPORT               (INPUT_REPORT_KEYS_MAX_LEN - SCAN_CODE_POS) /**< Maximum number of key presses that can be sent in one Input Report. */

#define OUTPUT_REPORT_BIT_MASK_CAPS_LOCK     0x02                                       /**< CAPS LOCK bit in Output Report (based on 'LED Page (0x08)' of the Universal Serial Bus HID Usage Tables). */

#define M_COMS_BLE_MODE                      0
#define M_COMS_GZLL_MODE                     1																							 

#define PROTOCOL_DETECT_PIN                  3
																							 
#define APP_GZLL_SEARCH_INTERVAL             APP_TIMER_TICKS(1000)

#define TX_INTERVAL                          APP_TIMER_TICKS(8) 
																							 
#define NRFR_MOUSE_PIPE         2       																							 
																							 
APP_TIMER_DEF(m_app_gzll_search_timer_id);																							 
APP_TIMER_DEF(m_battery_timer_id);                                  /**< Battery timer. */																							 
APP_TIMER_DEF(m_pkt_tx_timer_id);                                   /**< Packet sim timer */																							 

  
																							 
static bool              m_protocol_mode;

static nrf_drv_gpiote_in_config_t		protocol_detect_pin_cfg;


static sensorsim_cfg_t   m_battery_sim_cfg;                         /**< Battery Level sensor simulator configuration. */
static sensorsim_state_t m_battery_sim_state;                       /**< Battery Level sensor simulator state. */

static int32_t s_delta_multiplier = 1;
static uint8_t  m_pkt_tx_state = 0;

   

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

static void pkt_tx_handler(void * p_event_data, uint16_t event_size)
{
	static const int16_t s_delta_x[] = {0, -1, -3, -6, -8, -10, -12, -13, -14, -15, -15, -15, -15, -14, -13, -11, -9, -7, -5, -3, 0, 1, 4, 6, 8, 10, 12, 13, 15, 15, 15, 15, 15, 14, 12, 11, 9, 7, 4, 2, 0, -2, -5, -7, -9, -11, -13, -14, -15, -15, -15, -15, -14, -13, -12, -10, -8, -6, -4, -1, 0, 3, 5, 7, 10, 11, 13, 14, 15, 15, 15, 15, 14, 13, 12, 10, 8, 6, 3, 1, -1, -3, -6, -8, -10, -12, -13, -14, -15, -15, -15, -15, -14, -13, -11, -9, -7, -5, -3, 0, 2, 4, 6, 9, 11, 12, 14, 15, 15, 15, 15, 15, 14, 12, 11, 9, 7, 4, 2, 0, -2, -5, -7, -9, -11, -13, -14, -15, -15, -15, -15, -14, -13, -12, -10, -8, -6, -4, -1, 0, 3, 5, 8, 10, 11, 13, 14, 15, 15, 15, 15, 14, 13, 12, 10, 8, 5, 3, 1, -1, -3, -6, -8, -10, -12, -13, -14, -15, -15, -15, -15, -14, -13, -11, -9, -7, -5, -2, 0, 2, 4, 6, 9, 11, 12, 14, 15, 15, 15, 15, 15, 14, 12, 11, 9, 7, 4, 2, 0, -2, -5, -7, -9, -11, -13, -14, -15, -15, -15, -15, -14, -13, -12, -10, -8, -6, -4, -1, 0, 3, 5, 8, 10, 11, 13, 14, 15, 15, 15, 15, 14, 13, 11, 10, 8, 5, 3, 0, -1, -4, -6, -8, -10, -12, -13, -14, -15, -15, -15, -15, -14, -13, -11, -9, -7, -5, -2, 0, 2, 4, 7, 9, 11, 12, 14, 15, 15, 15, 15, 15, 14, 12, 11, 9, 6, 4, 2, 0, -2, -5, -7, -9, -11, -13, -14, -15, -15, -15, -15, -14, -13, -12, -10, -8, -6, -3, -1, 1, 3, 5, 8, 10, 12, 13, 14, 15, 15, 15, 15, 14, 13, 11, 10, 8, 5, 3, 0, -1, -4, -6, -8, -10, -12, -13, -14, -15, -15, -15, -15, -14, -13, -11, -9, -7, -5, -2, 0, 2, 4, 7, 9, 11, 12, 14, 15, 15, 15, 15, 15, 14, 12, 11, 9, 6, 4, 2, 0, -3, -5, -7};
  static const int16_t s_delta_y[] = {0, 15, 15, 14, 13, 11, 10, 8, 5, 3, 0, -1, -4, -6, -8, -10, -12, -13, -14, -15, -15, -15, -15, -14, -13, -11, -9, -7, -5, -2, 0, 2, 4, 7, 9, 11, 12, 14, 15, 15, 15, 15, 15, 14, 12, 11, 9, 6, 4, 2, 0, -2, -5, -7, -9, -11, -13, -14, -15, -15, -15, -15, -14, -13, -12, -10, -8, -6, -3, -1, 1, 3, 5, 8, 10, 12, 13, 14, 15, 15, 15, 15, 14, 13, 11, 10, 8, 5, 3, 0, -1, -4, -6, -8, -10, -12, -13, -14, -15, -15, -15, -15, -14, -13, -11, -9, -7, -5, -2, 0, 2, 4, 7, 9, 11, 12, 14, 15, 15, 15, 15, 15, 14, 12, 11, 9, 6, 4, 2, 0, -3, -5, -7, -9, -11, -13, -14, -15, -15, -15, -15, -14, -13, -12, -10, -8, -6, -3, -1, 1, 3, 6, 8, 10, 12, 13, 14, 15, 15, 15, 15, 14, 13, 11, 10, 7, 5, 3, 0, -1, -4, -6, -8, -10, -12, -13, -14, -15, -15, -15, -15, -14, -13, -11, -9, -7, -5, -2, 0, 2, 4, 7, 9, 11, 12, 14, 15, 15, 15, 15, 15, 13, 12, 10, 8, 6, 4, 1, 0, -3, -5, -7, -9, -11, -13, -14, -15, -15, -15, -15, -14, -13, -12, -10, -8, -6, -3, -1, 1, 3, 6, 8, 10, 12, 13, 14, 15, 15, 15, 15, 14, 13, 11, 9, 7, 5, 3, 0, -1, -4, -6, -8, -10, -12, -13, -15, -15, -15, -15, -15, -14, -12, -11, -9, -7, -4, -2, 0, 2, 5, 7, 9, 11, 13, 14, 15, 15, 15, 15, 14, 13, 12, 10, 8, 6, 4, 1, 0, -3, -5, -7, -10, -11, -13, -14, -15, -15, -15, -15, -14, -13, -12, -10, -8, -6, -3, -1, 1, 3, 6, 8, 10, 12, 13, 14, 15, 15, 15, 15, 14, 13, 11, 9, 7, 5, 3, 0, -2, -4, -6, -9, -11, -12, -14, -15, -15, -15, -15, -15, -14, -12, -11, -9, -7, -4, -2, 0, 2, 5, 7, 9, 11, 13, 14, 15, 15, 15, 15, 14, 13};
  static int32_t s_idx = 0;
	
   uint8_t  tx_buffer[4];			
			
    if (m_pkt_tx_state == 0)
    {
        return;
    }	

         tx_buffer[0] = INPUT_REP_REF_MOVEMENT_ID;
 
         int16_t x_delta = MIN((s_delta_x[s_idx] * s_delta_multiplier), 0x0fff);
         int16_t y_delta = MIN((s_delta_y[s_idx] * s_delta_multiplier), 0x0fff);

         tx_buffer[1] =  x_delta & 0x00ff;
         tx_buffer[2] = ((y_delta & 0x000f) << 4) | ((x_delta & 0x0f00) >> 8);
         tx_buffer[3] = (y_delta & 0x0ff0) >> 4;

         s_idx++;
		
	
        if (s_idx > (sizeof(s_delta_x) / sizeof(s_delta_x[0])))
        {
            s_idx = 0;
        }
        else if (s_idx < 0)
        {
            s_idx = (sizeof(s_delta_x) / sizeof(s_delta_x[0]));
        }
   
			 if(m_protocol_mode == M_COMS_BLE_MODE)
	     {	    
				 m_coms_ble_hid_report_send(1,&tx_buffer[1]);   //JSModify:
			 }
			 else 
			 {
				 nrf_gpio_pin_toggle(4);
         m_coms_gzll_send(NRFR_MOUSE_PIPE, tx_buffer, sizeof(tx_buffer));
       }				 
		
			
}	

static void pkt_tx_timeout_handler(void * p_context)
{
    uint32_t err_code;
		
    err_code = app_sched_event_put(0, 0, pkt_tx_handler);
    APP_ERROR_CHECK(err_code);
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
      err_code = app_timer_create(&m_app_gzll_search_timer_id,
                                   APP_TIMER_MODE_SINGLE_SHOT,
                                   app_gzll_search_timeout_handler);
			
		
    }			
		APP_ERROR_CHECK(err_code);
				
		// Create packet timer.
    err_code = app_timer_create(&m_pkt_tx_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                pkt_tx_timeout_handler);
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
static void batt_timer_start(void)
{
   if(m_protocol_mode == M_COMS_BLE_MODE)
   {	    
	     /* YOUR_JOB: Start your timers. below is an example of how to start a timer.*/
       ret_code_t err_code;	
       err_code = app_timer_start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL, NULL);
       APP_ERROR_CHECK(err_code);
   }
}


/**@brief Function for start timer.
 */
static void pkt_timer_start(uint32_t pkt_interval)
{
    uint32_t err_code;

    err_code = app_timer_start(m_pkt_tx_timer_id, pkt_interval, NULL);    //30.5 us per tick
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for stop timers.
 */
static void pkt_timer_stop()
{
    uint32_t err_code;

	  err_code = app_timer_stop(m_pkt_tx_timer_id);    
    APP_ERROR_CHECK(err_code);
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

    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);
	
	  NRF_LOG_INFO("Go to sleep zzzzz");	
	       
		 m_coms_ble_sleep_mode_enter();		 
}

/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated when button is pressed.
 */
static void bsp_event_handler(bsp_event_t event)
{
  
  switch (event)
    {
       case BSP_EVENT_SLEEP:
            //sleep_mode_enter();
            break;
			 
			 case BSP_EVENT_DISCONNECT:
            m_coms_ble_diconnect();
			    
            break;
			 
			 case BSP_EVENT_WHITELIST_OFF:
			      m_coms_ble_whitelist_off();
			 
			      break;
			 
			 case BSP_EVENT_KEY_0:
            
			      	pkt_timer_stop();				
              if (m_pkt_tx_state ==1)
              {							
                 nrf_gpio_pin_set(BSP_LED_0);   //Turn off 
                 m_pkt_tx_state = 0;
              }													
              else
              {
                 nrf_delay_ms(10);	
                 pkt_timer_start(TX_INTERVAL);
                 nrf_gpio_pin_clear(BSP_LED_0);   //Turn on  LED1
                 nrf_gpio_pin_set(BSP_LED_1);     //Turn off LED2
                 nrf_gpio_pin_set(BSP_LED_2);     //Turn off LED3
                 nrf_gpio_pin_set(BSP_LED_3);     //Turn off LED4
                 m_pkt_tx_state = 1;												
	            }						
			 
			 
            break;
        default:
            break;
    }
}



/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    ret_code_t err_code;
    bsp_event_t startup_event;

    err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
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

	

	 switch (p_evt->type)
   {
		 
		 case M_COMS_BLE_EVT_ADV_DIRECTED_HIGH_DUTY:			 
		      NRF_LOG_INFO("High Duty Directed advertising.");
          err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_DIRECTED);
          APP_ERROR_CHECK(err_code);
          break;
		 
		 case M_COMS_BLE_EVT_ADV_DIRECTED:
			    NRF_LOG_INFO("Directed advertising.");
          err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_DIRECTED);
          APP_ERROR_CHECK(err_code);	 
		      break;
		 
		 case M_COMS_BLE_EVT_ADV_FAST:			   
          NRF_LOG_INFO("Fast advertising.");
          err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
          APP_ERROR_CHECK(err_code);
		      break;
		 
		 case M_COMS_BLE_EVT_ADV_SLOW:
			    NRF_LOG_INFO("Slow advertising.");
          err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_SLOW);
          APP_ERROR_CHECK(err_code);
		      break;
		 
		 case M_COMS_BLE_EVT_ADV_FAST_WHITELIST:
			    NRF_LOG_INFO("Fast advertising with whitelist.");
          err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_WHITELIST);
          APP_ERROR_CHECK(err_code);
		      break;
		 
		 case M_COMS_BLE_EVT_ADV_SLOW_WHITELIST:
			    NRF_LOG_INFO("Slow advertising with whitelist.");
          err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_WHITELIST);
          APP_ERROR_CHECK(err_code);
		      break;
			
		 case M_COMS_BLE_EVT_ADV_IDLE:
			    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
          APP_ERROR_CHECK(err_code);
		      sleep_mode_enter();
          break;
         
		 
		 case M_COMS_BLE_EVT_CONNECTED:
			    NRF_LOG_INFO("Connected");
          err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
          APP_ERROR_CHECK(err_code);	
		      break;
		 
		 case M_COMS_BLE_EVT_DISCONNECTED:
		      NRF_LOG_INFO("Disconnected");
     
          err_code = bsp_indication_set(BSP_INDICATE_ALERT_OFF);
          APP_ERROR_CHECK(err_code);	
		      bsp_board_led_off(BSP_LED_INDICATE_USER_LED1);
		      bsp_board_led_off(BSP_LED_INDICATE_USER_LED2);
          break;
		 
		 case M_COMS_BLE_EVT_DATA_RECEIVED:			 
	 
		      break;
		 
		 case M_COMS_BLE_EVT_BONDED:
			    NRF_LOG_INFO("Bonded");
		      bsp_board_led_on(BSP_LED_INDICATE_USER_LED2);		    
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
             bsp_board_led_invert(BSP_LED_INDICATE_USER_LED3); 
				     app_timer_start(m_app_gzll_search_timer_id, APP_GZLL_SEARCH_INTERVAL, NULL);
             break;				
						  
			  case gzll_event_sysaddr_exchanged:
					   NRF_LOG_INFO("Paired with Host");
             bsp_board_led_on(BSP_LED_INDICATE_USER_LED3);
				     break;
				
				case gzll_event_hostid_exchanged:
					   NRF_LOG_INFO("Bonded with Host");
			       //is_bonded = true;
		         bsp_board_led_on(BSP_LED_INDICATE_USER_LED4);		
				     break;
				
				case gzll_event_tx_failed:
					    NRF_LOG_INFO("GZLL transmit failed !!");
				      bsp_board_led_off(BSP_LED_INDICATE_USER_LED3);	
				      bsp_board_led_off(BSP_LED_INDICATE_USER_LED4);	
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
    uint32_t unused_pins[] = {0, 1, 2, 5, 6, 7, 8, 9, 10, 11, 12, 26, 27, 28, 29, 30, 31};  
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
		
		   nrf_gpio_cfg_output(4);
		   nrf_gpio_pin_clear(4);
		
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


/**@brief Function for application main entry.
 */
int main(void)
{
   bool erase_bonds;  
	
	  // Initialize.
    log_init();
	
	  misc_io_init();    //For protocol mode setting & unused pins	
	
	  m_protocol_mode = ((nrf_gpio_pin_read(PROTOCOL_DETECT_PIN) == 0)?M_COMS_BLE_MODE:M_COMS_GZLL_MODE) ;

	
  	timers_init();	
	
    scheduler_init();

	  buttons_leds_init(&erase_bonds);

    modules_init();

    NRF_LOG_INFO("Dual mode mouse example with dual protocol started.");
   	
	  modules_enable(erase_bonds);
	
	  batt_timer_start();
		
		
	  

    // Enter main loop.
    for (;;)
    {
        idle_state_handle();
    }
}


/**
 * @}
 */
