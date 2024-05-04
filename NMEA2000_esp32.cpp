//@formatter:off
/*
NMEA2000_esp32.cpp

Copyright (c) 2015-2020 Timo Lappalainen, Kave Oy, www.kave.fi

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to use,
copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

Inherited NMEA2000 object for ESP32 modules. See also NMEA2000 library.

Thanks to Thomas Barth, barth-dev.de, who has written ESP32 CAN code. To avoid extra
libraries, I implemented his code directly to the NMEA2000_esp32 to avoid extra
can.h library, which may cause even naming problem.
*/

#include <driver/twai.h>
#include "freertos/timers.h"
#include <hal/twai_hal.h>
#include "sdkconfig.h"
#include "soc/dport_access.h"
#include "soc/system_reg.h"
#include "esp_log.h"
#include "NMEA2000_esp32.h"

static const char *LOG = "n2k_esp32";

bool tNMEA2000_esp32::CanInUse = false;
int tNMEA2000_esp32::receive_timeout_secs = 30;
static tNMEA2000_esp32 *instance;

//*****************************************************************************
tNMEA2000_esp32::tNMEA2000_esp32( gpio_num_t txPin, gpio_num_t rxPin, gpio_num_t standbyPin ) :
        tNMEA2000(),
        IsOpen( false ),
        speed( CAN_SPEED_250KBPS ),
        TxPin( txPin ),
        RxPin( rxPin ),
        StandbyPin( standbyPin ),
        RxQueue( nullptr ),
        TxQueue( nullptr ) {
    ESP_LOGI( "tNMEA2000_esp32", "tNMEA2000_esp32 tx=%d rx=%d stdby=%d", TxPin, RxPin, StandbyPin );
    instance = this;
}

void tNMEA2000_esp32::InitCANFrameBuffers() {
    if ( MaxCANReceiveFrames < 10 ) MaxCANReceiveFrames = 50; // ESP32 has plenty of RAM
    if ( MaxCANSendFrames < 10 ) MaxCANSendFrames = 40;
    //CANGlobalBufSize = MaxCANSendFrames;
    //MaxCANSendFrames = 0;
    uint16_t CANGlobalBufSize = MaxCANSendFrames - 4;
    MaxCANSendFrames = 4;  // we do not need much library internal buffer since driver has them.
    RxQueue = xQueueCreate( MaxCANReceiveFrames, sizeof( tCANFrame ));
    TxQueue = xQueueCreate( CANGlobalBufSize, sizeof( twai_message_t ));

    tNMEA2000::InitCANFrameBuffers(); // call main initialization
}

bool tNMEA2000_esp32::CANOpen() {
    if ( IsOpen ) return true;
    if ( CanInUse ) return false;

    IsOpen = true;
    CAN_init();

    CanInUse = IsOpen;

    return IsOpen;
}

void tNMEA2000_esp32::CAN_init() {
    gpio_config_t io_conf = {};

    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT_OD;
    io_conf.pin_bit_mask = BIT( StandbyPin );
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config( &io_conf );
    gpio_set_level( StandbyPin, 0 );

    //Initialize configuration structures using macro initializers
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT( TxPin, RxPin, TWAI_MODE_NO_ACK );
    g_config.alerts_enabled = TWAI_ALERT_ALL; //TWAI_ALERT_TX_IDLE;
    g_config.intr_flags = ESP_INTR_FLAG_IRAM;
    g_config.tx_queue_len = 20;
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    ESP_ERROR_CHECK( twai_driver_install( &g_config, &t_config, &f_config ));

    ESP_ERROR_CHECK( twai_start());

    create_event_tasks();
}

//*****************************************************************************
bool
tNMEA2000_esp32::CANSendFrame( unsigned long id, unsigned char len, const unsigned char *buf, bool /*wait_sent*/ ) {
    if ( uxQueueSpacesAvailable(TxQueue)==0 ) {
        // can not send to queue
        return false;
    }

    twai_message_t twai_frame;
    twai_frame.flags = 0;
    twai_frame.extd = true;
    twai_frame.identifier = id;
    if ( len > sizeof( twai_frame.data )) {
        len = sizeof( twai_frame.data );
    }
    twai_frame.data_length_code = len;
    memcpy( twai_frame.data, buf, len );

    return xQueueSendToBack(TxQueue,&twai_frame,0) == pdTRUE;
}

void tNMEA2000_esp32::CAN_send_frame( void *frame ) {

    twai_message_t *twai_frame = (twai_message_t *) frame;
    for ( ;; ) {
        esp_err_t result = twai_transmit( twai_frame, 0 );
        if ( result == ESP_OK ) {
            return;
        }
        if ( result == ESP_ERR_INVALID_STATE ) {
            twai_status_info_t status_info;
            twai_get_status_info( &status_info );
            if ( status_info.state == TWAI_STATE_BUS_OFF ) {
                ESP_LOGW( LOG, "Driver is in state %s, result is %04d, initiate recovery, dropping package",
                          "TWAI_STATE_BUS_OFF", result );
                twai_initiate_recovery();
            } else if ( status_info.state == TWAI_STATE_STOPPED ) {
                ESP_LOGW( LOG, "Driver is in state %s, result is %04d, start driver, retrying package",
                          "TWAI_STATE_STOPPED", result );
                twai_start();
                continue;
            } else if ( status_info.state == TWAI_STATE_RECOVERING ) {
                ESP_LOGW( LOG, "Driver is in state %s, result is %04d", "TWAI_STATE_RECOVERING", result );
            } else if ( status_info.state == TWAI_STATE_RUNNING ) {
                ESP_LOGW( LOG, "Driver is in state %s, result is %04d", "TWAI_STATE_RUNNING", result );
            } else {
                ESP_LOGW( LOG, "Driver is in state %d, result is %04d", status_info.state, result );
            }
        } else if ( ESP_OK != result ) {
            ESP_LOGW( LOG, "twai_transmit resulted in %d", result );
            // TODO handle result, restart driver for example
        }
        return;
    }
}

void tNMEA2000_esp32::send_task( void *arg ) {
    (void) arg;

    ESP_LOGI( LOG, "send_task starting" );
    for ( ;; ) {
        twai_message_t frame;

        if ( !xQueueReceive( TxQueue, &frame, portMAX_DELAY )) {
            continue;
        }
        instance->CAN_send_frame( &frame );
    }
}

//*****************************************************************************
bool tNMEA2000_esp32::CANGetFrame( unsigned long &id, unsigned char &len, unsigned char *buf ) {

    bool hasFrame = false;
    tCANFrame frame;

    //receive next CAN frame from queue
    if ( xQueueReceive( RxQueue, &frame, 0 ) == pdTRUE) {
        hasFrame = true;
        id = frame.id;
        len = frame.len;
        memcpy( buf, frame.buf, frame.len );
    }

    return hasFrame;
}

void tNMEA2000_esp32::receive_task( void *arg ) {
    (void) arg;

    ESP_LOGI( LOG, "receive_task starting" );
    twai_message_t message;
    for ( ;; ) {
        esp_err_t result = twai_receive( &message, pdMS_TO_TICKS( receive_timeout_secs * 1000 ));
        if ( result == ESP_OK ) {
            CAN_read_frame( &message );
            continue;
        }
        if ( result == ESP_ERR_TIMEOUT ) {
            ESP_LOGI( LOG, "nothing received in %d secs ", receive_timeout_secs );
            // TODO maybe restart driver?
            continue;
        }
        ESP_LOGW( LOG, "Unhandled result %04x %s from twai_receive",
                  result,
                  decodeEspResult( result ));
    }
}

const char *tNMEA2000_esp32::decodeEspResult( esp_err_t result ) {
    switch ( result ) {
        case ESP_ERR_INVALID_STATE:
            return "ESP_ERR_INVALID_STATE";
        default:
            return "unknown";
    }
}

void tNMEA2000_esp32::CAN_read_frame( void *frame ) {
    twai_message_t *twai_message = (twai_message_t *) frame;
    uint8_t len = twai_message->data_length_code;
    if ( len > sizeof( tCANFrame::buf )) {
        len = sizeof( tCANFrame::buf );
    }
    tCANFrame canFrame{
            .id=twai_message->identifier,
            .len = len,
            .buf = {}
    };
    memcpy( canFrame.buf, twai_message->data, len );
    // FromISR
    xQueueSendToBack( RxQueue, &canFrame, 0 );
}

void tNMEA2000_esp32::alert_task( void *arg ) {
    (void) arg;

    ESP_LOGI( LOG, "alert_task starting" );
    for ( ;; ) {
        uint32_t alerts = 0;
        esp_err_t result = twai_read_alerts( &alerts, portMAX_DELAY );
        if ( result == ESP_ERR_TIMEOUT || alerts == 0 ) {
            continue;
        }
//        ESP_LOGI( LOG, "alert got %08lx", alerts );
        // TWAI_ALERT_TX_IDLE 0x00000001
        // TWAI_ALERT_TX_SUCCESS 0x00000002
        // TWAI_ALERT_RX_DATA 0x00000004
        // TWAI_ALERT_ERR_ACTIVE 0x00000010
        // TWAI_ALERT_BUS_ERROR 0x00000200
        // twai_get_status_info(twai_status_info_t *status_info)
//        if ( alerts & TWAI_ALERT_TX_IDLE ) {
//            gpio_set_level( N2K_GPIO_NUM_STANDBY, 1 );
//            ESP_LOGI( LOG, "transmit end" );
//        }
    }
}

extern void n2k_save_address( uint8_t address );

static void address_changed_callback( TimerHandle_t timer ) {
    (void) timer;
    bool changed = instance->ReadResetAddressChanged();
    if ( changed ) {
        n2k_save_address( instance->GetN2kSource());
    }
}

static void receive_task_main( void *arg ) {
    instance->receive_task( arg );
}

static void alert_task_main( void *arg ) {
    instance->alert_task( arg );
}

static void send_task_main( void *arg ) {
    instance->send_task( arg );
}

void tNMEA2000_esp32::create_event_tasks() {
    xTaskCreate( receive_task_main, "twai_rx", 3072, nullptr, 5, nullptr );
    xTaskCreate( alert_task_main, "twai_idle", 2048, nullptr, 5, nullptr );
    xTaskCreate( send_task_main, "twai_tx", 3072, nullptr, 5, nullptr );

    TimerHandle_t timer = xTimerCreate(
            "address_changed",
            pdMS_TO_TICKS( 1 * 1000 ),
            1,
            nullptr,
            address_changed_callback );
    xTimerStart( timer, portMAX_DELAY );
}

bool tNMEA2000_esp32::isAbleToSendFrame() {
    return uxQueueSpacesAvailable(TxQueue) > 0;
}

bool tNMEA2000_esp32::isAbleToReceiveFrame() {
    return uxQueueMessagesWaiting( RxQueue ) > 0;
}
