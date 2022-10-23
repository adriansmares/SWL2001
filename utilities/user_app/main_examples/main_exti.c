/*!
 * \file      main_exti.c
 *
 * \brief     main program for exti example
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2021. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Semtech corporation nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT
 * NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL SEMTECH CORPORATION BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */
#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type

#include "main.h"

#include "smtc_modem_api.h"
#include "smtc_modem_utilities.h"

#include "smtc_modem_hal.h"
#include "smtc_hal_dbg_trace.h"

#include "example_options.h"

#include "smtc_hal_mcu.h"
#include "smtc_hal_gpio.h"

#if defined( SX128X )
#include "ralf_sx128x.h"
#elif defined( SX126X )
#include "ralf_sx126x.h"
#elif defined( LR11XX )
#include "ralf_lr11xx.h"
#endif

#include <string.h>

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/**
 * Stack id value (multistacks modem is not yet available)
 */
#define STACK_ID 0

#if defined( SX128X )
const ralf_t modem_radio = RALF_SX128X_INSTANTIATE( NULL );
#elif defined( SX126X )
const ralf_t modem_radio = RALF_SX126X_INSTANTIATE( NULL );
#elif defined( LR11XX )
const ralf_t modem_radio = RALF_LR11XX_INSTANTIATE( NULL );
#else
#error "Please select radio board.."
#endif
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */
static uint32_t alarm_rate           = 0x08;   // Periodic alarm rate in seconds
static bool     led_state            = false;  // Simulated LED state
static uint8_t  temperature_state    = 0x12;   // Simulated temperature sensor state

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */
static void get_event( void );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

/**
 * @brief Example to send a user payload on an external event
 *
 */
void main_exti( void )
{
    // Disable IRQ to avoid unwanted behaviour during init
    hal_mcu_disable_irq( );

    // Configure all the µC periph (clock, gpio, timer, ...)
    hal_mcu_init( );

    // Init the modem and use get_event as event callback, please note that the callback will be
    // called immediatly after the first call to smtc_modem_run_engine because of the reset detection
    smtc_modem_init( &modem_radio, &get_event );

    // Re-enable IRQ
    hal_mcu_enable_irq( );

    SMTC_HAL_TRACE_INFO( "EXTI example is starting \n" );

    smtc_modem_return_code_t ret = smtc_modem_alarm_start_timer( alarm_rate );
    if( ret != SMTC_MODEM_RC_OK )
    {
        SMTC_HAL_TRACE_ERROR ( "Failed to start alarm timer: %d\n", ret );
    }

    ret = smtc_modem_dm_set_info_interval( SMTC_MODEM_DM_INFO_INTERVAL_IN_HOUR, 1 );
    if( ret != SMTC_MODEM_RC_OK )
    {
        SMTC_HAL_TRACE_ERROR( "Failed to set periodic info interval: %d\n", ret );
    }

    const uint8_t info_fields[] = {
        SMTC_MODEM_DM_FIELD_STATUS,
        SMTC_MODEM_DM_FIELD_CHARGE,
        SMTC_MODEM_DM_FIELD_VOLTAGE,
        SMTC_MODEM_DM_FIELD_TEMPERATURE,
        SMTC_MODEM_DM_FIELD_SIGNAL,
        SMTC_MODEM_DM_FIELD_UP_TIME,
        SMTC_MODEM_DM_FIELD_RX_TIME,
        SMTC_MODEM_DM_FIELD_ALMANAC_STATUS,
    };
    ret = smtc_modem_dm_set_info_fields( info_fields, sizeof( info_fields ) );
    if( ret != SMTC_MODEM_RC_OK )
    {
        SMTC_HAL_TRACE_ERROR( "Failed to set periodic info fields: %d\n", ret );
    }

    while( 1 )
    {
        // Execute modem runtime, this function must be recalled in sleep_time_ms (max value, can be recalled sooner)
        hal_mcu_set_sleep_for_ms( smtc_modem_run_engine( ) );
    }
}

typedef enum application_f_port_e {
    application_f_port_uplink   = 102,
    application_f_port_downlink = 102,
} application_f_port_t;

typedef enum application_op_code_e {
    application_op_code_alarm_rate        = 0x01,
    application_op_code_led_state         = 0x02,
    application_op_code_temperature_state = 0x03,
} application_op_code_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/**
 * @brief User callback for modem event
 *
 *  This callback is called every time an event ( see smtc_modem_event_t ) appears in the modem.
 *  Several events may have to be read from the modem when this callback is called.
 */
static void get_event( void )
{
    SMTC_HAL_TRACE_MSG_COLOR( "get_event () callback\n", HAL_DBG_TRACE_COLOR_BLUE );

    smtc_modem_event_t current_event;
    uint8_t            event_pending_count;
    uint8_t            stack_id = STACK_ID;

    // Continue to read modem event until all event has been processed
    do
    {
        // Read modem event
        smtc_modem_get_event( &current_event, &event_pending_count );

        switch( current_event.event_type )
        {
        case SMTC_MODEM_EVENT_RESET:
            SMTC_HAL_TRACE_INFO( "Event received: RESET\n" );
            smtc_modem_join_network( stack_id );
            break;

        case SMTC_MODEM_EVENT_ALARM:
        {
            SMTC_HAL_TRACE_INFO( "Event received: ALARM\n" );

            smtc_modem_return_code_t ret = smtc_modem_alarm_start_timer( alarm_rate );
            if( ret != SMTC_MODEM_RC_OK )
            {
                SMTC_HAL_TRACE_ERROR ( "Failed to start alarm timer: %d\n", ret );
                break;
            }

            uint32_t status = 0;
            ret = smtc_modem_get_status( stack_id, &status );
            if( ret != SMTC_MODEM_RC_OK )
            {
                SMTC_HAL_TRACE_ERROR( "Failed to retrieve status: %d\n", ret );
                break;
            }
            if( ( status & SMTC_MODEM_STATUS_JOINED ) != SMTC_MODEM_STATUS_JOINED )
            {
                SMTC_HAL_TRACE_INFO( "Not joined yet\n" );
                break;
            }

            const uint8_t mcu_temperature = ( uint8_t ) smtc_modem_hal_get_temperature( );
            SMTC_HAL_TRACE_INFO( "MCU temperature : %d\n", mcu_temperature );

            // Send MCU temperature on port 102
            const uint8_t buffer[] = { mcu_temperature, led_state, temperature_state, alarm_rate };
            ret = smtc_modem_request_uplink( stack_id, application_f_port_uplink, false, buffer, sizeof(buffer) );
            if( ret != SMTC_MODEM_RC_OK )
            {
                SMTC_HAL_TRACE_ERROR( "Failed to enqueue uplink: %d\n", ret );
                break;
            }

            break;
        }

        case SMTC_MODEM_EVENT_JOINED:
        {
            SMTC_HAL_TRACE_INFO( "Event received: JOINED\n" );
            SMTC_HAL_TRACE_INFO( "Modem is now joined \n" );

            smtc_modem_return_code_t ret = smtc_modem_time_start_sync_service( stack_id, SMTC_MODEM_TIME_MAC_SYNC );
            if( ret != SMTC_MODEM_RC_OK )
            {
                SMTC_HAL_TRACE_ERROR( "Failed to start time sync: %d\n", ret );
                break;
            }
            ret = smtc_modem_lorawan_class_b_request_ping_slot_info ( stack_id );
            if( ret != SMTC_MODEM_RC_OK )
            {
                SMTC_HAL_TRACE_ERROR( "Failed to request ping slot info: %d\n", ret );
                break;
            }

            break;
        }

        case SMTC_MODEM_EVENT_TXDONE:
            SMTC_HAL_TRACE_INFO( "Event received: TXDONE\n" );
            SMTC_HAL_TRACE_INFO( "Transmission done: %d\n", current_event.event_data.txdone.status );
            break;

        case SMTC_MODEM_EVENT_DOWNDATA:
        {
            SMTC_HAL_TRACE_INFO( "Event received: DOWNDATA\n" );

            const uint8_t f_port = current_event.event_data.downdata.fport;
            const uint16_t rx_payload_size = current_event.event_data.downdata.length;
            const uint8_t* rx_payload = current_event.event_data.downdata.data;

            SMTC_HAL_TRACE_PRINTF( "Data received on port %u\n", f_port );
            SMTC_HAL_TRACE_ARRAY( "Received payload", rx_payload, rx_payload_size );

            switch( current_event.event_data.downdata.fport )
            {
                case application_f_port_downlink:
                    for( uint8_t i = 0; i + 1 < rx_payload_size; i += 2 )
                    {
                        switch( rx_payload[i] )
                        {
                            case application_op_code_alarm_rate:
                                alarm_rate = rx_payload[i + 1];
                                SMTC_HAL_TRACE_INFO( "Alarm rate updated: %d\n", alarm_rate );
                                break;
                            case application_op_code_led_state:
                                led_state = rx_payload[i + 1] != 0;
                                SMTC_HAL_TRACE_INFO( "LED updated: %d\n", led_state );
                                break;
                            case application_op_code_temperature_state:
                                temperature_state = rx_payload[i + 1];
                                SMTC_HAL_TRACE_INFO( "Temperature updated: %d\n", temperature_state );
                                break;
                            default:
                                SMTC_HAL_TRACE_ERROR( "Unknown operation code: %d\n", rx_payload[i] );
                                break;
                        }
                    }

                    break;
                default:
                    break;
            }

            break;
        }

        case SMTC_MODEM_EVENT_UPLOADDONE:
            SMTC_HAL_TRACE_INFO( "Event received: UPLOADDONE\n" );

            break;

        case SMTC_MODEM_EVENT_SETCONF:
            SMTC_HAL_TRACE_INFO( "Event received: SETCONF\n" );

            break;

        case SMTC_MODEM_EVENT_MUTE:
            SMTC_HAL_TRACE_INFO( "Event received: MUTE\n" );

            break;

        case SMTC_MODEM_EVENT_STREAMDONE:
            SMTC_HAL_TRACE_INFO( "Event received: STREAMDONE\n" );

            break;

        case SMTC_MODEM_EVENT_JOINFAIL:
            SMTC_HAL_TRACE_INFO( "Event received: JOINFAIL\n" );
            SMTC_HAL_TRACE_WARNING( "Join request failed \n" );
            break;

        case SMTC_MODEM_EVENT_TIME:
            SMTC_HAL_TRACE_INFO( "Event received: TIME\n" );

            if( current_event.event_data.time.status != SMTC_MODEM_EVENT_TIME_VALID )
            {
                break;
            }

            smtc_modem_return_code_t ret = smtc_modem_set_class( stack_id, SMTC_MODEM_CLASS_B );
            if( ret != SMTC_MODEM_RC_OK )
            {
                SMTC_HAL_TRACE_ERROR( "Failed to switch to class B: %d\n", ret );
                break;
            }

            break;

        case SMTC_MODEM_EVENT_TIMEOUT_ADR_CHANGED:
            SMTC_HAL_TRACE_INFO( "Event received: TIMEOUT_ADR_CHANGED\n" );
            break;

        case SMTC_MODEM_EVENT_NEW_LINK_ADR:
            SMTC_HAL_TRACE_INFO( "Event received: NEW_LINK_ADR\n" );
            break;

        case SMTC_MODEM_EVENT_LINK_CHECK:
            SMTC_HAL_TRACE_INFO( "Event received: LINK_CHECK\n" );
            break;

        case SMTC_MODEM_EVENT_ALMANAC_UPDATE:
        {
            SMTC_HAL_TRACE_INFO( "Event received: ALMANAC_UPDATE\n" );

            if( current_event.event_data.almanac_update.status != SMTC_MODEM_EVENT_ALMANAC_UPDATE_STATUS_REQUESTED )
            {
                break;
            }

            const uint8_t info_fields[] = { SMTC_MODEM_DM_FIELD_ALMANAC_STATUS };
            smtc_modem_return_code_t ret = smtc_modem_dm_request_single_uplink( info_fields, sizeof( info_fields ) );
            if( ret != SMTC_MODEM_RC_OK )
            {
                SMTC_HAL_TRACE_ERROR( "Failed to send single DM request: %d\n", ret );
                break;
            }

            break;
        }

        case SMTC_MODEM_EVENT_USER_RADIO_ACCESS:
            SMTC_HAL_TRACE_INFO( "Event received: USER_RADIO_ACCESS\n" );
            break;

        case SMTC_MODEM_EVENT_CLASS_B_PING_SLOT_INFO:
            SMTC_HAL_TRACE_INFO( "Event received: CLASS_B_PING_SLOT_INFO\n" );
            break;

        case SMTC_MODEM_EVENT_CLASS_B_STATUS:
            SMTC_HAL_TRACE_INFO( "Event received: CLASS_B_STATUS\n" );
            break;

        default:
            SMTC_HAL_TRACE_ERROR( "Unknown event %u\n", current_event.event_type );
            break;
        }
    } while( event_pending_count > 0 );
}
