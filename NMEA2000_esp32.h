#ifndef NMEA2000_ESP32_H_
#define NMEA2000_ESP32_H_

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "NMEA2000.h"
#include "N2kMsg.h"
#include "ESP32_CAN_def.h"

#ifndef ESP32_CAN_TX_PIN
#define ESP32_CAN_TX_PIN GPIO_NUM_16
#endif
#ifndef ESP32_CAN_RX_PIN
#define ESP32_CAN_RX_PIN GPIO_NUM_4
#endif
#ifndef ESP32_CAN_STANDBY_PIN
#define ESP32_CAN_STANDBY_PIN GPIO_NUM_1
#endif

class tNMEA2000_esp32 : public tNMEA2000 {
private:
    bool IsOpen;
    static bool CanInUse;

protected:
    struct tCANFrame {
        uint32_t id; // can identifier
        uint8_t len; // length of data
        uint8_t buf[8];
    };

protected:
    CAN_speed_t speed;
    gpio_num_t TxPin;
    gpio_num_t RxPin;
    gpio_num_t StandbyPin;
    QueueHandle_t RxQueue;
    QueueHandle_t TxQueue;
    static int receive_timeout_secs;

protected:
    void CAN_read_frame( void *frame);

    void CAN_init();
    void create_event_tasks();
    const char* decodeEspResult( esp_err_t result );

protected:
    bool CANOpen() override;
    bool CANSendFrame( unsigned long id, unsigned char len, const unsigned char *buf, bool wait_sent ) override;
    bool CANGetFrame( unsigned long &id, unsigned char &len, unsigned char *buf ) override;

    void InitCANFrameBuffers() override;
public:
    explicit tNMEA2000_esp32( gpio_num_t txPin = ESP32_CAN_TX_PIN,
                              gpio_num_t rxPin = ESP32_CAN_RX_PIN,
                              gpio_num_t standbyPin = ESP32_CAN_STANDBY_PIN );
    _Noreturn void receive_task( void *arg );

    _Noreturn void alert_task( void *arg );

    _Noreturn void send_task( void *arg );

    void CAN_send_frame( void *frame );
};

#endif
