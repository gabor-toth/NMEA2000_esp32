#ifndef NMEA2000_ESP32_STREAM_H_
#define NMEA2000_ESP32_STREAM_H_

#include "N2kStream.h"

#define HEX 16

class EspN2kStream : public N2kStream {
public:
    int read() override;
    int peek() override;
    size_t write(const uint8_t* data, size_t size) override;

    // make Arduino developers happy

    static void print( const char *format, ... );
    static void print( int value, int radix = 10 );
    static void println( const char *format, ... );
    static void println( int value );
    static void println();
};

extern EspN2kStream Serial;

#endif
