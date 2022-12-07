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

#include "smtc_modem_middleware_advanced_api.h"

#include "gnss_middleware.h"
#include "wifi_middleware.h"

#include "modem_pinout.h"
#include "ral_lr11xx_bsp.h"

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
static const smtc_modem_stream_cipher_mode_t stream_cipher_mode      = SMTC_MODEM_STREAM_NO_CIPHER; // Location stream cipher mode
static const uint8_t                         stream_redundancy_ratio = 128;                         // Location stream redundancy ratio

static gnss_mw_mode_t gnss_scan_mode = GNSS_MW_MODE_STATIC; // Periodic GNSS scan mode (static/mobile)
static uint32_t       gnss_scan_rate = 60 * 60;             // Periodic GNSS scan rate in seconds
static uint32_t       wifi_scan_rate = 60 * 60;             // Periodic WiFi scan rate in seconds

static smtc_modem_dm_info_interval_format_t dm_report_unit = SMTC_MODEM_DM_INFO_INTERVAL_IN_MINUTE; // Device Management reporting interval unit
static uint8_t                              dm_report_rate = 5;                                     // Device Management reporting interval

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

    SMTC_HAL_TRACE_INFO( "EXTI example is starting \n" );

    mw_version_t mw_version = { 0 };
    mw_return_code_t mw_ret = wifi_mw_get_version( &mw_version );
    if( mw_ret != MW_RC_OK )
    {
        SMTC_HAL_TRACE_ERROR( "Failed to get WiFi middleware version: %d\n", mw_ret );
        return;
    }
    SMTC_HAL_TRACE_INFO( "WiFi middleware version: %d.%d.%d\n", mw_version.major, mw_version.minor, mw_version.patch );

    mw_ret = wifi_mw_init( &modem_radio, STACK_ID );
    if( mw_ret != MW_RC_OK )
    {
        SMTC_HAL_TRACE_ERROR( "Failed to initialize wifi middleware: %d\n", mw_ret );
        return;
    }
    wifi_mw_send_bypass( true );

    mw_ret = gnss_mw_get_version( &mw_version );
    if( mw_ret != MW_RC_OK )
    {
        SMTC_HAL_TRACE_ERROR( "Failed to get GNSS middleware version: %d\n", mw_ret );
        return;
    }
    SMTC_HAL_TRACE_INFO( "GNSS middleware version: %d.%d.%d\n", mw_version.major, mw_version.minor, mw_version.patch );

    mw_ret = gnss_mw_init( &modem_radio, STACK_ID );
    if( mw_ret != MW_RC_OK )
    {
        SMTC_HAL_TRACE_ERROR( "Failed to initialize GNSS middleware: %d\n", mw_ret );
        return;
    }
    gnss_mw_send_bypass( true );

    // Re-enable IRQ
    hal_mcu_enable_irq( );

    while( 1 )
    {
        // Execute modem runtime, this function must be recalled in sleep_time_ms (max value, can be recalled sooner)
        hal_mcu_set_sleep_for_ms( smtc_modem_run_engine( ) );
    }
}

typedef enum application_f_port_e {
    application_f_port_uplink   = 102,
    application_f_port_downlink = 102,
    application_f_port_solver   = 150,
    application_f_port_stream   = 199,
} application_f_port_t;

typedef enum tlv_record_e {
    tlv_record_gnss_any_antenna   = 0x05,
    tlv_record_gnss_pcb_antenna   = 0x06,
    tlv_record_gnss_patch_antenna = 0x07,
    tlv_record_wifi               = 0x08,
} tlv_record_t;

typedef enum application_op_code_e {
    application_op_code_get_parameters     = 0x01,

    application_op_code_set_gnss_scan_mode = 0x10,
    application_op_code_set_gnss_scan_rate = 0x11,
    application_op_code_set_wifi_scan_rate = 0x12,
    application_op_code_set_dm_report_rate = 0x13,

    application_op_code_request_gnss_scan  = 0x20,
    application_op_code_request_wifi_scan  = 0x21,
} application_op_code_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static void on_reset( const uint8_t stack_id )
{
    smtc_modem_return_code_t ret = smtc_modem_dm_set_info_interval( dm_report_unit, dm_report_rate );
    if( ret != SMTC_MODEM_RC_OK )
    {
        SMTC_HAL_TRACE_ERROR( "Failed to set DM periodic rate: %d\n", ret );
        return;
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
    ret = smtc_modem_dm_set_info_fields( info_fields, sizeof info_fields );
    if( ret != SMTC_MODEM_RC_OK )
    {
        SMTC_HAL_TRACE_ERROR( "Failed to set DM periodic fields: %d\n", ret );
        return;
    }

    ret = smtc_modem_time_start_sync_service( stack_id, SMTC_MODEM_TIME_MAC_SYNC );
    if( ret != SMTC_MODEM_RC_OK )
    {
        SMTC_HAL_TRACE_ERROR( "Failed to start time sync: %d\n", ret );
        return;
    }

    ret = smtc_modem_stream_init( stack_id, application_f_port_stream, stream_cipher_mode, stream_redundancy_ratio );
    if( ret != SMTC_MODEM_RC_OK )
    {
        SMTC_HAL_TRACE_ERROR( "Failed to start streaming: %d\n", ret );
        return;
    }

    ret = smtc_modem_join_network( stack_id );
    if( ret != SMTC_MODEM_RC_OK )
    {
        SMTC_HAL_TRACE_ERROR( "Failed to join network: %d\n", ret );
        return;
    }
}

static void on_join( const uint8_t stack_id )
{
    smtc_modem_return_code_t ret = smtc_modem_lorawan_class_b_request_ping_slot_info( stack_id );
    if( ret != SMTC_MODEM_RC_OK )
    {
        SMTC_HAL_TRACE_ERROR( "Failed to request ping slot info: %d\n", ret );
        return;
    }
}

#define ENCODE_4BYTES(x) \
        ( x       ) & 0xFF, \
        ( x >> 8  ) & 0xFF, \
        ( x >> 16 ) & 0xFF, \
        ( x >> 24 ) & 0xFF  \

#define DECODE_4BYTES(x, i) (    \
        ( ( x )[( i )    ]       ) | \
        ( ( x )[( i ) + 1] << 8  ) | \
        ( ( x )[( i ) + 2] << 16 ) | \
        ( ( x )[( i ) + 3] << 24 )   \
    )

static void on_down( const uint8_t stack_id, const smtc_modem_event_t* current_event )
{
    const uint8_t f_port = current_event->event_data.downdata.fport;
    const uint16_t rx_payload_size = current_event->event_data.downdata.length;
    const uint8_t* rx_payload = current_event->event_data.downdata.data;

    SMTC_HAL_TRACE_PRINTF( "Data received on port %u\n", f_port );
    SMTC_HAL_TRACE_ARRAY( "Received payload", rx_payload, rx_payload_size );

    switch( f_port )
    {
        case application_f_port_downlink:
        {
            for( uint8_t i = 0; i < rx_payload_size; )
            {
                switch( rx_payload[i] )
                {
                    case application_op_code_get_parameters:
                    {
                        const uint8_t buffer[] = {
                            application_op_code_get_parameters,
                            gnss_scan_mode,
                            ENCODE_4BYTES(gnss_scan_rate),
                            ENCODE_4BYTES(wifi_scan_rate),
                            ( uint8_t ) smtc_modem_hal_get_temperature(),
                            dm_report_unit,
                            dm_report_rate,
                        };
                        smtc_modem_return_code_t ret = smtc_modem_request_uplink(
                            stack_id,
                            application_f_port_uplink,
                            false,
                            buffer,
                            sizeof buffer
                        );
                        if( ret != SMTC_MODEM_RC_OK )
                        {
                            SMTC_HAL_TRACE_ERROR( "Failed to request uplink: %d\n", ret );
                        }
                        i += 1;
                        break;
                    }

                    case application_op_code_set_gnss_scan_mode:
                    {
                        if ( i + 1 >= rx_payload_size )
                        {
                            SMTC_HAL_TRACE_ERROR( "Not enough bytes: %d / %d\n", i + 1, rx_payload_size );
                            i = rx_payload_size;
                            break;
                        }
                        switch( rx_payload[i + 1] )
                        {
                            case GNSS_MW_MODE_STATIC:
                            case GNSS_MW_MODE_MOBILE:
                                gnss_scan_mode = rx_payload[i + 1];
                        }
                        i += 2;
                        break;
                    }

                    case application_op_code_set_gnss_scan_rate:
                    {
                        if ( i + 4 >= rx_payload_size )
                        {
                            SMTC_HAL_TRACE_ERROR( "Not enough bytes: %d / %d\n", i + 4, rx_payload_size );
                            i = rx_payload_size;
                            break;
                        }
                        gnss_scan_rate = DECODE_4BYTES( rx_payload, i + 1 );
                        i += 5;
                        break;
                    }

                    case application_op_code_set_wifi_scan_rate:
                    {
                        if ( i + 4 >= rx_payload_size )
                        {
                            SMTC_HAL_TRACE_ERROR( "Not enough bytes: %d / %d\n", i + 4, rx_payload_size );
                            i = rx_payload_size;
                            break;
                        }
                        wifi_scan_rate = DECODE_4BYTES( rx_payload, i + 1 );
                        i += 5;
                        break;
                    }

                    case application_op_code_set_dm_report_rate:
                    {
                        if ( i + 2 >= rx_payload_size )
                        {
                            SMTC_HAL_TRACE_ERROR( "Not enough bytes: %d / %d\n", i + 4, rx_payload_size );
                            i = rx_payload_size;
                            break;
                        }
                        switch( rx_payload[i + 1] )
                        {
                            case SMTC_MODEM_DM_INFO_INTERVAL_IN_SECOND:
                            case SMTC_MODEM_DM_INFO_INTERVAL_IN_DAY:
                            case SMTC_MODEM_DM_INFO_INTERVAL_IN_HOUR:
                            case SMTC_MODEM_DM_INFO_INTERVAL_IN_MINUTE:
                            {
                                dm_report_unit = ( smtc_modem_dm_info_interval_format_t ) rx_payload[i + 1];
                                dm_report_rate = rx_payload[i + 2];
                                smtc_modem_return_code_t ret = smtc_modem_dm_set_info_interval( dm_report_unit, dm_report_rate );
                                if( ret != SMTC_MODEM_RC_OK )
                                {
                                    SMTC_HAL_TRACE_ERROR( "Failed to set DM periodic rate: %d\n", ret );
                                }
                                break;
                            }
                        }
                        i += 3;
                        break;
                    }

                    case application_op_code_request_gnss_scan:
                    {
                        if ( i + 4 >= rx_payload_size )
                        {
                            SMTC_HAL_TRACE_ERROR( "Not enough bytes: %d / %d\n", i + 4, rx_payload_size );
                            i = rx_payload_size;
                            break;
                        }
                        const uint32_t delay = DECODE_4BYTES( rx_payload, i + 1 );
                        mw_return_code_t mw_ret = gnss_mw_scan_start( gnss_scan_mode, delay );
                        if( mw_ret != MW_RC_OK )
                        {
                            SMTC_HAL_TRACE_ERROR( "Failed to start GNSS scan: %d\n", mw_ret );
                        }
                        i += 5;
                        break;
                    }

                    case application_op_code_request_wifi_scan:
                    {
                        if ( i + 4 >= rx_payload_size )
                        {
                            SMTC_HAL_TRACE_ERROR( "Not enough bytes: %d / %d\n", i + 4, rx_payload_size );
                            i = rx_payload_size;
                            break;
                        }
                        const uint32_t delay = DECODE_4BYTES( rx_payload, i + 1 );

                        mw_return_code_t mw_ret = wifi_mw_scan_start( delay );
                        if( mw_ret != MW_RC_OK )
                        {
                            SMTC_HAL_TRACE_ERROR( "Failed to start WiFi scan: %d\n", mw_ret );
                        }
                        i += 5;
                        break;
                    }

                    default:
                        SMTC_HAL_TRACE_ERROR( "Unknown operation code: %d\n", rx_payload[i] );
                        i = rx_payload_size;
                        break;
                }
            }
            break;
        }

        case application_f_port_solver:
        {
            mw_return_code_t mw_ret = gnss_mw_set_solver_aiding_position( rx_payload, rx_payload_size );
            if( mw_ret != MW_RC_OK )
            {
                SMTC_HAL_TRACE_ERROR( "Failed to push solver aid: %d\n", mw_ret );
                break;
            }
            break;
        }
    }
}

static void on_time( const uint8_t stack_id, const smtc_modem_event_t* current_event )
{
    if( current_event->event_data.time.status != SMTC_MODEM_EVENT_TIME_VALID )
    {
        return;
    }

    smtc_modem_return_code_t ret = smtc_modem_set_class( stack_id, SMTC_MODEM_CLASS_B );
    if( ret != SMTC_MODEM_RC_OK )
    {
        SMTC_HAL_TRACE_ERROR( "Failed to set modem class: %d\n", ret );
        return;
    }

    mw_return_code_t mw_ret = wifi_mw_scan_start( 0 );
    if( mw_ret != MW_RC_OK )
    {
        SMTC_HAL_TRACE_ERROR( "Failed to start WiFi scan: %d\n", mw_ret );
        return;
    }

    mw_ret = gnss_mw_scan_start( gnss_scan_mode, 0 );
    if( mw_ret != MW_RC_OK )
    {
        SMTC_HAL_TRACE_ERROR( "Failed to start GNSS scan: %d\n", mw_ret );
        return;
    }
}

static void on_almanac_update( const uint8_t stack_id, const smtc_modem_event_t* current_event )
{
    if( current_event->event_data.almanac_update.status != SMTC_MODEM_EVENT_ALMANAC_UPDATE_STATUS_REQUESTED )
    {
        return;
    }

    const uint8_t info_fields[] = { SMTC_MODEM_DM_FIELD_ALMANAC_STATUS };
    smtc_modem_return_code_t ret = smtc_modem_dm_request_single_uplink( info_fields, sizeof info_fields );
    if( ret != SMTC_MODEM_RC_OK )
    {
        SMTC_HAL_TRACE_ERROR( "Failed to send single DM request: %d\n", ret );
        return;
    }
}

static void on_ping_slot_info( const uint8_t stack_id, const smtc_modem_event_t* current_event )
{
    if( current_event->event_data.class_b_ping_slot_info.status == SMTC_MODEM_EVENT_CLASS_B_PING_SLOT_ANSWERED )
    {
        return;
    }

    smtc_modem_return_code_t ret = smtc_modem_lorawan_class_b_request_ping_slot_info( stack_id );
    if( ret != SMTC_MODEM_RC_OK )
    {
        SMTC_HAL_TRACE_ERROR( "Failed to request ping slot info: %d\n", ret );
        return;
    }
}

static void on_gnss_middleware( const uint8_t stack_id, const smtc_modem_event_t* current_event )
{
    const uint8_t pending_events = current_event->event_data.middleware_event_status.status;
    const bool scan_done = gnss_mw_has_event( pending_events, GNSS_MW_EVENT_SCAN_DONE );
    bool repeat_scan = true;
    if( scan_done )
    {
        SMTC_HAL_TRACE_INFO( "GNSS middleware event - SCAN DONE\n" );

        gnss_mw_event_data_scan_done_t event_data = { 0 };
        mw_return_code_t mw_ret = gnss_mw_get_event_data_scan_done( &event_data );
        if( mw_ret != MW_RC_OK )
        {
            SMTC_HAL_TRACE_ERROR( "Failed to get GNSS event data: %d\n", mw_ret );
            return;
        }
        gnss_mw_display_results( event_data );

        if( !event_data.is_valid )
        {
            goto gnss_continue;
        }

        const uint8_t header_size = 2 * sizeof( uint8_t );
        uint16_t buffer_size = 0;
        for( int i = 0; i < event_data.nb_scan_valid; i++ )
        {
            if( !event_data.scan[i].nav_valid )
            {
                continue;
            }
            buffer_size += header_size + event_data.scan[i].nav_size;
        }

        if( buffer_size == 0 )
        {
            goto gnss_continue;
        }

        uint16_t pending = 0, free = 0;
        smtc_modem_return_code_t ret = smtc_modem_stream_status( stack_id, &pending, &free );
        if( ret != SMTC_MODEM_RC_OK )
        {
            SMTC_HAL_TRACE_ERROR( "Failed to get stream status: %d\n", ret );
            return;
        }
        if( free < buffer_size )
        {
            goto gnss_continue;
        }

        uint8_t buffer[256] = { 0 };
        uint8_t buffer_offset = 0;
        for( int i = 0; i < event_data.nb_scan_valid; i++ )
        {
            if( !event_data.scan[i].nav_valid )
            {
                continue;
            }

            buffer[buffer_offset    ] = tlv_record_gnss_any_antenna;
            buffer[buffer_offset + 1] = event_data.scan[i].nav_size;
            memcpy( &buffer[buffer_offset + 2], event_data.scan[i].nav, event_data.scan[i].nav_size );

            buffer_offset += header_size + event_data.scan[i].nav_size;
        }

        ret = smtc_modem_stream_add_data( stack_id, buffer, buffer_size );
        if( ret != SMTC_MODEM_RC_OK )
        {
            SMTC_HAL_TRACE_ERROR( "Failed to add stream data: %d\n", ret );
            return;
        }

        repeat_scan = false;
    }

gnss_continue:
    if( gnss_mw_has_event( pending_events, GNSS_MW_EVENT_TERMINATED ) )
    {
        SMTC_HAL_TRACE_INFO( "GNSS middleware event - TERMINATED\n" );

        gnss_mw_event_data_terminated_t event_data = { 0 };
        mw_return_code_t mw_ret =  gnss_mw_get_event_data_terminated( &event_data );
        if( mw_ret != MW_RC_OK )
        {
            SMTC_HAL_TRACE_ERROR( "Failed to get GNSS event data: %d\n", mw_ret );
            return;
        }
        SMTC_HAL_TRACE_PRINTF( "TERMINATED info:\n" );
        SMTC_HAL_TRACE_PRINTF( "-- number of scans sent: %u\n", event_data.nb_scan_sent );
    }

    if( gnss_mw_has_event( pending_events, GNSS_MW_EVENT_SCAN_CANCELLED ) )
    {
        SMTC_HAL_TRACE_INFO( "GNSS middleware event - SCAN CANCELLED\n" );
    }

    if( gnss_mw_has_event( pending_events, GNSS_MW_EVENT_ERROR_NO_TIME ) )
    {
        SMTC_HAL_TRACE_ERROR( "GNSS middleware event - ERROR NO TIME\n" );
    }

    if( gnss_mw_has_event( pending_events, GNSS_MW_EVENT_ERROR_ALMANAC_UPDATE ) )
    {
        SMTC_HAL_TRACE_ERROR( "GNSS middleware event - ALMANAC UPDATE REQUIRED\n" );
    }

    if( gnss_mw_has_event( pending_events, GNSS_MW_EVENT_ERROR_NO_AIDING_POSITION ) )
    {
        SMTC_HAL_TRACE_ERROR( "GNSS middleware event - ERROR NO AIDING POSITION set\n" );
    }

    if( gnss_mw_has_event( pending_events, GNSS_MW_EVENT_ERROR_UNKNOWN ) )
    {
        SMTC_HAL_TRACE_ERROR( "GNSS middleware event - UNEXPECTED ERROR\n" );
    }

    gnss_mw_clear_pending_events( );

    mw_return_code_t mw_ret = gnss_mw_scan_start( gnss_scan_mode, repeat_scan ? 0 : gnss_scan_rate );
    if( mw_ret != MW_RC_OK )
    {
        SMTC_HAL_TRACE_ERROR( "Failed to start GNSS scan: %d\n", mw_ret );
        return;
    }
}

static void on_wifi_middleware( const uint8_t stack_id, const smtc_modem_event_t* current_event )
{
    const uint8_t pending_events = current_event->event_data.middleware_event_status.status;
    const bool scan_done = wifi_mw_has_event( pending_events, WIFI_MW_EVENT_SCAN_DONE );
    bool repeat_scan = true;
    if( scan_done )
    {
        SMTC_HAL_TRACE_INFO( "WiFi middleware event - SCAN DONE\n" );

        wifi_mw_event_data_scan_done_t event_data = { 0 };
        mw_return_code_t mw_ret = wifi_mw_get_event_data_scan_done( &event_data );
        if( mw_ret != MW_RC_OK )
        {
            SMTC_HAL_TRACE_ERROR( "Failed to get WiFi event data: %d\n", mw_ret );
            return;
        }
        wifi_mw_display_results( event_data );

        if( event_data.nbr_results == 0 )
        {
            goto wifi_continue;
        }

        const uint8_t wifi_record_size = 1 + sizeof( lr11xx_wifi_mac_address_t );
        const uint8_t header_size = 2 * sizeof( uint8_t );
        const uint16_t buffer_size = header_size + event_data.nbr_results * wifi_record_size;

        uint16_t pending = 0, free = 0;
        smtc_modem_return_code_t ret = smtc_modem_stream_status( stack_id, &pending, &free );
        if( ret != SMTC_MODEM_RC_OK )
        {
            SMTC_HAL_TRACE_ERROR( "Failed to get stream status: %d\n", ret );
            return;
        }
        if( free < buffer_size )
        {
            goto wifi_continue;
        }

        uint8_t buffer[256] = { 0 };
        buffer[0] = tlv_record_wifi;
        buffer[1] = buffer_size - header_size;
        for( int i = 0; i < event_data.nbr_results; i++ )
        {
            const uint16_t offset = header_size + i * wifi_record_size;
            buffer[offset] = event_data.results[i].rssi;
            memcpy( &buffer[offset + 1], event_data.results[i].mac_address, sizeof( lr11xx_wifi_mac_address_t ) );
        }

        ret = smtc_modem_stream_add_data( stack_id, buffer, buffer_size );
        if( ret != SMTC_MODEM_RC_OK )
        {
            SMTC_HAL_TRACE_ERROR( "Failed to add stream data: %d\n", ret );
            return;
        }

        repeat_scan = false;
    }

wifi_continue:
    if( wifi_mw_has_event( pending_events, WIFI_MW_EVENT_TERMINATED ) )
    {
        SMTC_HAL_TRACE_INFO( "WiFi middleware event - TERMINATED\n" );

        wifi_mw_event_data_terminated_t event_data = { 0 };
        mw_return_code_t mw_ret =  wifi_mw_get_event_data_terminated( &event_data );
        if( mw_ret != MW_RC_OK )
        {
            SMTC_HAL_TRACE_ERROR( "Failed to get WiFi event data: %d\n", mw_ret );
            return;
        }
        SMTC_HAL_TRACE_PRINTF( "TERMINATED info:\n" );
        SMTC_HAL_TRACE_PRINTF( "-- number of scans sent: %u\n", event_data.nb_scan_sent );
    }

    if( wifi_mw_has_event( pending_events, WIFI_MW_EVENT_SCAN_CANCELLED ) )
    {
        SMTC_HAL_TRACE_INFO( "WiFi middleware event - SCAN CANCELLED\n" );
    }

    if( wifi_mw_has_event( pending_events, WIFI_MW_EVENT_ERROR_UNKNOWN ) )
    {
        SMTC_HAL_TRACE_ERROR( "WiFi middleware event - UNEXPECTED ERROR\n" );
    }

    wifi_mw_clear_pending_events( );

    mw_return_code_t mw_ret = wifi_mw_scan_start( repeat_scan ? 0 : wifi_scan_rate );
    if( mw_ret != MW_RC_OK )
    {
        SMTC_HAL_TRACE_ERROR( "Failed to start WiFi scan: %d\n", mw_ret );
        return;
    }
}

/**
 * @brief User callback for modem event
 *
 *  This callback is called every time an event ( see smtc_modem_event_t ) appears in the modem.
 *  Several events may have to be read from the modem when this callback is called.
 */
static void get_event( void )
{
    SMTC_HAL_TRACE_MSG_COLOR( "get_event () callback\n", HAL_DBG_TRACE_COLOR_BLUE );

    smtc_modem_event_t current_event = { 0 };
    uint8_t            event_pending_count = 0;
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
            on_reset( stack_id );
            break;

        case SMTC_MODEM_EVENT_ALARM:
            SMTC_HAL_TRACE_INFO( "Event received: ALARM\n" );
            break;

        case SMTC_MODEM_EVENT_JOINED:
            SMTC_HAL_TRACE_INFO( "Event received: JOINED\n" );
            SMTC_HAL_TRACE_INFO( "Modem is now joined \n" );
            on_join( stack_id );
            break;

        case SMTC_MODEM_EVENT_TXDONE:
            SMTC_HAL_TRACE_INFO( "Event received: TXDONE\n" );
            SMTC_HAL_TRACE_INFO( "Transmission done\n" );
            break;

        case SMTC_MODEM_EVENT_DOWNDATA:
            SMTC_HAL_TRACE_INFO( "Event received: DOWNDATA\n" );
            on_down( stack_id, &current_event );
            break;

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
            on_time( stack_id, &current_event );
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
            SMTC_HAL_TRACE_INFO( "Event received: ALMANAC_UPDATE\n" );
            on_almanac_update( stack_id, &current_event );
            break;

        case SMTC_MODEM_EVENT_USER_RADIO_ACCESS:
            SMTC_HAL_TRACE_INFO( "Event received: USER_RADIO_ACCESS\n" );
            break;

        case SMTC_MODEM_EVENT_CLASS_B_PING_SLOT_INFO:
            SMTC_HAL_TRACE_INFO( "Event received: CLASS_B_PING_SLOT_INFO\n" );
            on_ping_slot_info( stack_id, &current_event );
            break;

        case SMTC_MODEM_EVENT_CLASS_B_STATUS:
            SMTC_HAL_TRACE_INFO( "Event received: CLASS_B_STATUS\n" );
            break;

        case SMTC_MODEM_EVENT_MIDDLEWARE_1:
            SMTC_HAL_TRACE_INFO( "Event received: MIDDLEWARE_1\n" );
            on_gnss_middleware( stack_id, &current_event );
            break;

        case SMTC_MODEM_EVENT_MIDDLEWARE_2:
            SMTC_HAL_TRACE_INFO( "Event received: MIDDLEWARE_2\n" );
            on_wifi_middleware( stack_id, &current_event );
            break;

        case SMTC_MODEM_EVENT_MIDDLEWARE_3:
            SMTC_HAL_TRACE_INFO( "Event received: MIDDLEWARE_3\n" );
            break;

        default:
            SMTC_HAL_TRACE_ERROR( "Unknown event %u\n", current_event.event_type );
            break;
        }
    } while( event_pending_count > 0 );
}

void smtc_board_lna_on( void ) { hal_gpio_set_value( RADIO_LNA_CTRL, 1 ); }

void smtc_board_lna_off( void ) { hal_gpio_set_value( RADIO_LNA_CTRL, 0 ); }

void smtc_board_wifi_prescan( void ) { }

void smtc_board_wifi_postscan( void ) { }

void smtc_board_gnss_prescan( void ) { smtc_board_lna_on( ); }

void smtc_board_gnss_postscan( void ) { smtc_board_lna_off( ); }

void mw_bsp_gnss_prescan_actions( void ) { smtc_board_gnss_prescan( ); }

void mw_bsp_gnss_postscan_actions( void ) { smtc_board_gnss_postscan( ); }

void mw_bsp_wifi_prescan_actions( void ) { smtc_board_wifi_prescan( ); }

void mw_bsp_wifi_postscan_actions( void ) { smtc_board_wifi_postscan( ); }

lr11xx_system_lfclk_cfg_t mw_bsp_get_lr11xx_lf_clock_cfg( void ) { return LR11XX_SYSTEM_LFCLK_XTAL; }

void mw_bsp_get_lr11xx_reg_mode( const void* context, lr11xx_system_reg_mode_t* reg_mode )
{
    ral_lr11xx_bsp_get_reg_mode( context, reg_mode );
}
