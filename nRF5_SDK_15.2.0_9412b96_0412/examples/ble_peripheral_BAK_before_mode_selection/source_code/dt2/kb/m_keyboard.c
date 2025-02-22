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
 
#include "m_keyboard.h"

#include <string.h>

#include "app_scheduler.h"
#include "app_timer.h"
#include "app_util.h"
#include "common_params.h"
#include "drv_keyboard_btn.h"
#include "drv_keyboard_matrix.h"
#include "hal_gpiote.h"
#include "io_cfg.h"
#include "nrf_assert.h"
#include "nrf_error.h"

//wright : add for debug 
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

//wright : add for long push 
#define KEY_LONG_PUSH_COUNTS 30   //If the times of a key be detected over this count in serail , the key value should be send.   
// wright : add for long push 
static int pressed_count =0;


#define KEY_PACKET_MODIFIER_KEY_INDEX (0) //!< Index in the key packet where modifier keys such as ALT and Control are stored
#define KEY_PACKET_RESERVED_INDEX     (1) //!< Index in the key packet where OEMs can store information
#define KEY_PACKET_KEY_INDEX          (2) //!< Start index in the key packet where pressed keys are stored
#define KEY_PACKET_MAX_KEYS           (6) //!< Maximum number of keys that can be stored into the key packet                                                                  
#define KEY_PACKET_SIZE               (KEY_PACKET_KEY_INDEX+KEY_PACKET_MAX_KEYS) //!< Total size of the key packet in bytes
#define KEY_PACKET_NO_KEY             (0) //!< Value to be stored to key index to indicate no key is pressed

#define MODIFIER_HID_START         0xE0
#define MODIFIER_HID_END           0xE7
#define MODIFIER_LEFT_CONTROL_HID  0xE0
#define MODIFER_RIGHT_CONTROL_HID  0xE4

static uint8_t                   m_currently_pressed_keys[KEYBOARD_MAX_NUM_OF_PRESSED_KEYS]; //!< Array holding currently pressed keys. Filled up from index 0. Values are 
static uint8_t                   m_transmitted_keys[KEYBOARD_MAX_NUM_OF_PRESSED_KEYS];       //!< Array holding the keys that have already been transmitted.
static uint8_t                   m_num_of_currently_pressed_keys;                            //!< Number of keys in m_currently_pressed_keys
static uint8_t                   m_number_of_transmitted_keys;                               //!< Number of keys in m_transmitted_keys
static m_keyboard_data_t         m_key_packet;                              //!< Stores last created key packet. One byte is used for modifier keys, one for OEMs. Key values are USB HID keycodes.
static m_keyboard_data_t         m_empty_key_packet;                        //!< Empty key report kept for comparison
static bool                      m_fn_key_set = false;
static bool                      m_keyboard_matrix_enabled = false;
static m_keyboard_format_t       s_format;
static bool                      m_keyboard_initialized = false;
//wright debug
//static app_timer_id_t            m_keyboard_timer_id;
static bool                      m_keyboard_timer_running = false;
static app_sched_event_handler_t m_keyboard_event_callback = 0;

static void gpio_int_handler(void);

//wright debug
APP_TIMER_DEF(m_keyboard_timer_id);



bool m_keyboard_packet_is_empty(m_keyboard_data_t* packet)
{
    return memcmp(packet, &m_empty_key_packet, sizeof(m_empty_key_packet)) == 0;
}

static void polling_enable_handler(void * p_event_data, uint16_t p_event_size) 
{   
		NRF_LOG_INFO("---- enter polling_enable_handler -----");
    m_keyboard_polling_enable(M_KEYBOARD_DEFAULT_POLLRATE); 
}

static void m_keyboard_hid_to_ascii(m_keyboard_data_t *data)
{
    uint8_t i;
    uint32_t keys_idx = 0;
    
    for(i = 0; i < data->num_keys; i++)
    {
        uint8_t key = data->keys[i];
        
        if ((key >= 0x1E) && (key <= 0x27))
        {
            if(key == 0x27)
            {
                data->keys[keys_idx++] = 0x30;         // '0'
            }
            else
            {
                data->keys[keys_idx++] = key + 0x13;
            }    
        }
        else if ((key >= 0x04) && (key <= 0x1D))
        {
            data->keys[keys_idx++] = key + 0x5D;
        }
        else if (key == 0x2A)                 // Backspace
        {
            data->keys[keys_idx++] = 0x08;         // 'BS'
        }
    }
    data->num_keys = keys_idx;
}


//#define WRIGHT_HW_NOT_READY


static void matrix_scan_timeout_handler(void* p_context)
{
	
//************************** Wright : HW Ready *************************//
#ifndef WRIGHT_HW_NOT_READY
	
   m_keyboard_data_t* key_data;  
	 static bool pairing_button_is_set=false;
	 //NRF_LOG_INFO("------ enter matrix_scan_timeout_handler ------");
	
	 if (m_keyboard_matrix_enabled && m_keyboard_timer_running)
   {
			if (m_keyboard_new_packet(&key_data))
      {   
				if (s_format == keyboard_format_ascii)
				{
						m_keyboard_hid_to_ascii(key_data);                
				}
			
				m_keyboard_event_callback(key_data, sizeof(m_keyboard_data_t));
				
				//wright : 
				//Keypacket is empty & keychanged means user release the key button from a long push
				if(m_keyboard_packet_is_empty(key_data) && (m_fn_key_set == false))
				{
						NRF_LOG_INFO("------ m_keyboard_packet_is_empty ------");

						// Keys are no longer held: stop polling and enable interrupts
						m_keyboard_polling_disable();

						drv_keyboard_sense_enable(true);
						drv_keyboard_btn_sense_enable(true);
					
						// wright : for long push
						pressed_count = 0;
            
         }
			}
    }
	 
		if (drv_keyboard_btn_read_pin_state() != 0 && !pairing_button_is_set)
    {
        uint32_t err_code;
        m_keyboard_data_t data;
        pairing_button_is_set = true;
        m_keyboard_polling_disable();
       // while (drv_keyboard_btn_read_pin_state() != 0);
        data.pairing_button = true;
				NRF_LOG_INFO("------ matrix_scan_timeout_handler B part ------");
        err_code = app_sched_event_put(&data, sizeof(m_keyboard_data_t), m_keyboard_event_callback);
        APP_ERROR_CHECK(err_code);
        
        drv_keyboard_btn_sense_enable(true);
        drv_keyboard_sense_enable(true);
    }
    else
    {
        if(drv_keyboard_btn_read_pin_state()==0)
        { 
            pairing_button_is_set=false;
        }
    }       
		
//************************** Wright : HW Not Ready *************************//
		
#else
		m_keyboard_data_t new_key_data;
		m_keyboard_data_t * key_data = &new_key_data; 
		static bool pairing_button_is_set=false;
		
		//*************wright : Add fake data and send to peer every second.******************//
		//fake data : wright 
		uint8_t fake_str[M_KEYBOARD_MAX_NUM_KEYS] = /**< Key pattern to be sent when the key press button has been pushed. */
		{
				0x1a,       /* Key w */
				0x15,       /* Key r */
				0x0c,       /* Key i */
				0x0a,       /* Key g */
				0x0b,       /* Key h */
				0x17        /* Key t*/
		};

		key_data->modifier_keys = 0x00;
		memcpy(key_data->keys,fake_str,M_KEYBOARD_MAX_NUM_KEYS);
		key_data->num_keys=0x01;
		key_data->pairing_button=0x00;
		
		//wright : debug
		NRF_LOG_INFO("------ enter matrix_scan_timeout_handler ------");
	
    if (m_keyboard_matrix_enabled && m_keyboard_timer_running)
    {
				//wright : pass the proccess of getting key val from HW
				if(1)
					{   
            if (s_format == keyboard_format_ascii)
            {
                m_keyboard_hid_to_ascii(key_data);                
            }
          
            m_keyboard_event_callback(key_data, sizeof(m_keyboard_data_t));
            
            if(m_keyboard_packet_is_empty(key_data) && (m_fn_key_set == false))
            {

                // Keys are no longer held: stop polling and enable interrupts
                m_keyboard_polling_disable();
							          
            }
        }
    }
		
#endif 


}
     


static void gpio_int_handler(void)
{
    uint32_t err_code;
    if (m_keyboard_matrix_enabled)
    {
        drv_keyboard_sense_enable(false);
    }
    err_code = app_sched_event_put(0,0, polling_enable_handler);
    APP_ERROR_CHECK(err_code);
}

static void m_keyboard_keypacket_addkey(uint8_t key)
{
    int i;
    for (i = 0; i < M_KEYBOARD_MAX_NUM_KEYS; i++)
    {
        if (m_key_packet.keys[i] == key)
        {
            return;
        }
    }

    for (i = 0; i < M_KEYBOARD_MAX_NUM_KEYS; i++)
    {
        if (m_key_packet.keys[i] == KEY_PACKET_NO_KEY)
        {
            m_key_packet.keys[i] = key;
            m_key_packet.num_keys = i + 1;
            return;
        }
    }
}

static void m_keyboard_keypacket_create(m_keyboard_data_t *key_packet)
{
    int i,j;
    
    // Clear key_packet contents
    for (i = 0; i < M_KEYBOARD_MAX_NUM_KEYS; i++)
    {
        key_packet->keys[i] = KEY_PACKET_NO_KEY;
    }
    key_packet->modifier_keys = 0;
    key_packet->num_keys = 0;
    
    // Give priority to keys that were already pressed when we transmitted them the last time.
    for (i = 0 ; i < m_number_of_transmitted_keys; i++)
    {
        for (j = 0; j < m_num_of_currently_pressed_keys; j++)
        {
            if (m_transmitted_keys[i] == m_currently_pressed_keys[j])
            {
                if ((m_currently_pressed_keys[i] < MODIFIER_HID_START) && (m_currently_pressed_keys[i] > MODIFIER_HID_END)) // Detect and set modifier keys
                {
                    m_keyboard_keypacket_addkey(m_currently_pressed_keys[j]);
                }
                break;
            }
        }
    }

    // Detect modifier keys, and add rest of the keys to the packet
    for (i = 0; i < m_num_of_currently_pressed_keys; i++)
    {
        if ((m_currently_pressed_keys[i] >= MODIFIER_HID_START) && (m_currently_pressed_keys[i] <= MODIFIER_HID_END)) // Detect and set modifier keys
        {
            key_packet->modifier_keys |= (uint8_t)(1U << (m_currently_pressed_keys[i] - MODIFIER_HID_START));

        }
        else if (m_currently_pressed_keys[i] != 0)
        {
            m_keyboard_keypacket_addkey(m_currently_pressed_keys[i]);
        }
    }
    
		

    // Remapping of special keys (if they exist)
    m_fn_key_set = drv_keyboard_remap_special_keys(m_key_packet.keys, m_key_packet.modifier_keys, m_key_packet.num_keys);    
		

}

static bool m_keyboard_have_keys_changed(const uint8_t *state_now, uint8_t number_of_now_pressed_keys, const uint8_t *state_before, uint8_t number_of_before_pressed_keys)
{
    int i;
    
	
		//wright : debug 
		//NRF_LOG_INFO("number_of_now_pressed_keys : %d ; number_of_before_pressed_keys %d",number_of_now_pressed_keys,number_of_before_pressed_keys);
	
    if (number_of_now_pressed_keys != number_of_before_pressed_keys)
    {
        return true;
    }
    else
    {
        for (i = number_of_now_pressed_keys; i--;)
        {
            if (state_now[i] != state_before[i])
            {
                return true;
            }
        } 
    }
    return false;
}
//wright : add for setting GPIOTE handler into port sense event
#include <nrf_drv_gpiote.h>

void in_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    gpio_int_handler();
}

void gpiote_port_event_set()
{
	ret_code_t err_code;
	APP_ERROR_CHECK(nrf_drv_gpiote_init());
	
	nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(false);
	in_config.pull = NRF_GPIO_PIN_PULLDOWN;
	
	//make all ROW gpios sense low to high 
	APP_ERROR_CHECK(nrf_drv_gpiote_in_init(21, &in_config, in_pin_handler));
	APP_ERROR_CHECK(nrf_drv_gpiote_in_init(22, &in_config, in_pin_handler));
	APP_ERROR_CHECK(nrf_drv_gpiote_in_init(23, &in_config, in_pin_handler));
	APP_ERROR_CHECK(nrf_drv_gpiote_in_init(24, &in_config, in_pin_handler));
	APP_ERROR_CHECK(nrf_drv_gpiote_in_init(25, &in_config, in_pin_handler));
	APP_ERROR_CHECK(nrf_drv_gpiote_in_init(28, &in_config, in_pin_handler));
	APP_ERROR_CHECK(nrf_drv_gpiote_in_init(6, &in_config, in_pin_handler));
	APP_ERROR_CHECK(nrf_drv_gpiote_in_init(13, &in_config, in_pin_handler));


	nrf_drv_gpiote_in_event_enable(21, true);
	nrf_drv_gpiote_in_event_enable(22, true);
	nrf_drv_gpiote_in_event_enable(23, true);
	nrf_drv_gpiote_in_event_enable(24, true);
	nrf_drv_gpiote_in_event_enable(25, true);
	nrf_drv_gpiote_in_event_enable(28, true);
	nrf_drv_gpiote_in_event_enable(6, true);
	nrf_drv_gpiote_in_event_enable(13, true);
	
	APP_ERROR_CHECK(err_code);
	return ;
}



uint32_t m_keyboard_init(m_keyboard_format_t p_format, app_sched_event_handler_t event_handler)
{
    uint32_t err_code;

    if (p_format != keyboard_format_ascii &&
        p_format != keyboard_format_usbhid)
    {
        return NRF_ERROR_INVALID_PARAM;
    }        
    
    if (event_handler == 0)
    {
        return NRF_ERROR_INVALID_PARAM;
    }
    
    if (m_keyboard_initialized)
    {
        return NRF_ERROR_INVALID_STATE;
    }   
    
    s_format         = p_format;
    m_keyboard_event_callback = event_handler;          
    m_num_of_currently_pressed_keys = 0;
    m_number_of_transmitted_keys = 0;    
    memset(m_currently_pressed_keys, 0, sizeof(m_currently_pressed_keys));
    memset(m_transmitted_keys, 0, sizeof(m_transmitted_keys));
    memset(&m_empty_key_packet, KEY_PACKET_NO_KEY, sizeof(m_empty_key_packet));
    
#ifndef WRIGHT_HW_NOT_READY
    drv_keyboard_btn_init();
#endif 
    
    err_code = app_timer_create(&m_keyboard_timer_id,
                                  APP_TIMER_MODE_REPEATED,
                                  matrix_scan_timeout_handler);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }


    err_code = hal_gpiote_init();
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
#ifndef WRIGHT_HW_NOT_READY
		
    drv_keyboard_btn_sense_enable(true);
		
		//wright: fix bug (DT2 callback handler is not properly set)
		//err_code = hal_gpiote_cb_set(hal_gpiote_evt_port, gpio_int_handler);
		gpiote_port_event_set();
		

		
		
#endif   
		
		//wright: debug
    // Trigger polling right away
    polling_enable_handler(0, 0);
    
    return err_code;
}


uint32_t m_keyboard_format_set(m_keyboard_format_t p_format)
{
    if (p_format != keyboard_format_ascii &&
        p_format != keyboard_format_usbhid)
    {
        return NRF_ERROR_INVALID_PARAM;
    }
    
    s_format = p_format;
    
    return NRF_SUCCESS;    
}

uint32_t m_keyboard_matrix_enable(void)
{    
    if(m_keyboard_matrix_enabled == true)
    {
        return NRF_ERROR_INVALID_STATE;
    }
#ifndef WRIGHT_HW_NOT_READY
    drv_keyboard_init();
#endif 
		
    m_keyboard_matrix_enabled = true;
		
		
#ifndef WRIGHT_HW_NOT_READY
    drv_keyboard_sense_enable(true);
#endif    
		
		
    return NRF_SUCCESS;
}

uint32_t m_keyboard_matrix_disable(void)
{   
    if(m_keyboard_matrix_enabled == false)
    {
        return NRF_ERROR_INVALID_STATE;
    }
    
    m_keyboard_matrix_enabled = false;
    
    return NRF_SUCCESS;
}

uint32_t m_keyboard_polling_enable(uint32_t pollrate_ms)
{
    uint32_t err_code;
    
    if (!m_keyboard_timer_running)
    {
				//wright
        err_code = app_timer_start(m_keyboard_timer_id, APP_TIMER_TICKS(pollrate_ms), 0);//APP_TIMER_TICKS(pollrate_ms, APP_TIMER_PRESCALER), 0);
        APP_ERROR_CHECK(err_code);
        m_keyboard_timer_running = true;
        matrix_scan_timeout_handler(NULL);
    }
    
    return NRF_SUCCESS;
}
 
void m_keyboard_polling_disable(void)
{  
	
    if (m_keyboard_timer_running)
    {
				NRF_LOG_INFO("---- enter m_keyboard_polling_disable() -----");
        app_timer_stop(m_keyboard_timer_id);
        m_keyboard_timer_running = false;
    }
}
 
bool m_keyboard_new_packet(m_keyboard_data_t ** p_key_packet)
{
	

    bool new_packet_prepared;
    int  i;

    // Save currently pressed keys
    for (i = KEYBOARD_MAX_NUM_OF_PRESSED_KEYS; i--;)  
    {
        m_transmitted_keys[i] = m_currently_pressed_keys[i];
    }

		m_number_of_transmitted_keys = m_num_of_currently_pressed_keys;
		
    // Create a new packet if key states have changed and there are no keys blocking each other (ghosting/phantom keys)
    if (drv_keyboard_keymatrix_read(m_currently_pressed_keys, &m_num_of_currently_pressed_keys))
    {
				//NRF_LOG_HEXDUMP_INFO(m_currently_pressed_keys,KEYBOARD_MAX_NUM_OF_PRESSED_KEYS);
			
       if (m_keyboard_have_keys_changed(m_currently_pressed_keys, m_num_of_currently_pressed_keys, m_transmitted_keys, m_number_of_transmitted_keys)
				//wright : set for long push
			 || pressed_count>KEY_LONG_PUSH_COUNTS
			 )
       {

					NRF_LOG_INFO("----- m_keyboard_have_keys_changed -----");
					NRF_LOG_INFO("m_num_of_currently_pressed_keys : %d",m_num_of_currently_pressed_keys);
					NRF_LOG_INFO("m_number_of_transmitted_keys : %d",m_number_of_transmitted_keys);
					NRF_LOG_INFO("m_currently_pressed_keys :");
					NRF_LOG_HEXDUMP_INFO(m_currently_pressed_keys,KEYBOARD_MAX_NUM_OF_PRESSED_KEYS);
					NRF_LOG_INFO("m_transmitted_keys :");
					NRF_LOG_HEXDUMP_INFO(m_transmitted_keys,KEYBOARD_MAX_NUM_OF_PRESSED_KEYS);
				 

				 
					m_keyboard_keypacket_create(&m_key_packet);
					*p_key_packet = &m_key_packet;
					new_packet_prepared = true;
				 
        }
        else
        {
            // The same keys are still pressed, no need to create a new packet
            new_packet_prepared = false;
					
						//wright : set for long push
						pressed_count++;
        }
    }
    else
    {
        // Ghosting detected. Don't create a packet.
        new_packet_prepared = false;
    }

    return new_packet_prepared;    
}
 
bool m_keyboard_pairing_btn_pressed(void)
{
    return drv_keyboard_btn_read_pin_state() != 0;
}

#include "nrf_sdm.h"
//wright
#include "nrf_nvic.h"

bool m_keyboard_wakeup_prepare(bool wakeup)
{
        uint8_t     softdevice_enabled     = false;
    uint32_t    err_code;
    #ifdef SOFTDEVICE_PRESENT
    err_code = sd_softdevice_is_enabled(&softdevice_enabled);
    APP_ERROR_CHECK(err_code);
#else
#ifndef SOFTDEVICE_NOT_PRESENT
#error "Please define either SOFTDEVICE_PRESENT or SOFTDEVICE_NOT_PRESENT to state if a Softdevice is present or not."
#endif /* SOFTDEVICE_NOT_PRESENT */
#endif /* SOFTDEVICE_PRESENT */ 

    if (softdevice_enabled == 1)
    {
        sd_nvic_critical_region_enter(0);
        m_keyboard_polling_disable();
      
        sd_nvic_DisableIRQ(GPIOTE_IRQn);
        sd_nvic_ClearPendingIRQ(GPIOTE_IRQn);
        sd_nvic_critical_region_exit(0);
    }
    else
    {       
        NVIC_DisableIRQ(GPIOTE_IRQn);
        NVIC_ClearPendingIRQ(GPIOTE_IRQn);
        
    }
    drv_keyboard_wakeup_prepare();
    drv_keyboard_btn_wakeup_prepare();
    return true;
     
}
