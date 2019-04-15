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
 *
 * @defgroup modules_coms_gzll
 * @{
 * @ingroup nrfready_modules
 * @brief Gazell communications sub-module.
 *
 * @details 
 */
#ifndef __M_COMS_GZLL_H__
#define __M_COMS_GZLL_H__
 
#include <stdbool.h>
#include <stdint.h>

#include "app_scheduler.h"


#define M_COMS_GZLL_DEFAULT_SYNC_LIFETIME 1000


/**@brief Gazell parameters. See @ref M_COMS_GZLL_PARAMS_FILL for to initialize default values. */
typedef struct
{
    bool     encrypt;         /** Make encrypted pipe available */
    uint32_t tx_attempts;     /** Number of retransmission attempts per packet */
    uint32_t timeslot_period; /** Gazell timing parameter. Needs to match Host side */
    uint32_t sync_lifetime;   /** Number of timeslot periods to stay in sync */
} m_coms_gzll_params_t;


typedef enum
{
    gzll_pairing_status_no_pairing,
    gzll_pairing_status_system_address_present,
    gzll_pairing_status_host_id_present
} gzll_pairing_status_t;

typedef enum
{
    gzll_event_search_for_host    = 1,
	  gzll_event_sysaddr_exchanged  = 2, /** System address exchanged */
    gzll_event_sysaddr_timeout    = 3, /** System address exchange has timed out (failed) */
    gzll_event_hostid_exchanged   = 4, /** Host ID has been exchanged */
    gzll_event_hostid_rejected    = 5, /** Host ID exchange has failed (rejected by the host or timed out) */
    gzll_event_tx_failed          = 6, /** Transmission attempt has timed out (failed) */
    gzll_event_disabled           = 7  /** Event to indicate that gzll is disabled */
} gzll_event_t;

/**@brief Gazell initialization
 *
 * @param[in] event_callback 
 * @param[in] tx_attempts    
 * @return
 * @retval NRF_SUCCESS
 */
uint32_t m_coms_gzll_init(const app_sched_event_handler_t event_callback);

 
/**@brief Sends data to the host/central device. Starts reconnect if disconnected.
 *
 * @param[in] type        Type of data
 * @param[in] packet      The packet to send
 * @param[in] packet_size Size of the packet
 * @return
 * @retval NRF_SUCCESS
 * @retval NRF_PARAM_ERROR 
 */
uint32_t m_coms_gzll_send(uint8_t p_pipe, uint8_t * p_packet, uint8_t p_packet_size);

/**@brief Enables gazell. Must have been initialized prior to this.
 *
 * @return
 * @retval NRF_SUCCESS
 * @retval NRF_ERROR_INTERNAL
 */
uint32_t m_coms_gzll_enable(bool erase_bonds);

/**@brief Stop all radio activity, disable Gazell.
 *
 * @return
 * @retval NRF_SUCCESS
 * @retval NRF_ERROR_INTERNAL
 */
uint32_t m_coms_gzll_disable(void);

/**@brief Prepares for sleep and subsequent wakeup.
 */
bool m_coms_gzll_wakeup_prepare(void);

/**@brief Clears existing Gazell bonds and search for a new host to bond with
 *
 * @return
 * @retval NRF_SUCCESS
 * @retval NRF_ERROR_TIMEOUT
 */
uint32_t m_coms_gzll_bonding_start(void);

/**@brief Check if keys are stored in flash.
 *
 * @return True if keys from previous connection is stored in flash
 */
bool m_coms_gzll_keys_stored(void);

/**@brief Delete all keys stored in flash.
 *
 * @return
 */
void m_coms_gzll_clear_keys(void);
 
 
uint32_t m_coms_gzll_search_for_host(void);


#endif /*  __M_COMS_GZLL_H__ */

/** @} */
