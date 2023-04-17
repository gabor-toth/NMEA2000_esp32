#include "NMEA2000_esp32_stream.h"
#include <cstdio>
#include <iostream>
#include <cstdarg>

const char* TAG = "n2k";

EspN2kStream Serial;

int EspN2kStream::read() {
    return 0;
}

int EspN2kStream::peek() {
    return 0;
}

size_t EspN2kStream::write( const uint8_t *data, size_t size ) {
    return fwrite(data, size, 1, stdout);
}

void EspN2kStream::print( const char *format, ... ) {
    va_list ap;

    va_start(ap, format);
    vprintf(format, ap);
    va_end(ap);
}

void EspN2kStream::print( int value, int radix ) {
    if ( radix == 10) {
        printf("%d", value);
    } else if ( radix == 16) {
        printf("%02x", value);
    } else {
        throw std::invalid_argument("unsupported radix ");
    }
}

void EspN2kStream::println( const char *format, ... ) {
    va_list ap;

    va_start(ap, format);
    vprintf(format, ap);
    println();

    va_end(ap);
}

void EspN2kStream::println( int value ) {
    std::cout << value << std::endl;
}

void EspN2kStream::println() {
    std::cout << std::endl;
}
