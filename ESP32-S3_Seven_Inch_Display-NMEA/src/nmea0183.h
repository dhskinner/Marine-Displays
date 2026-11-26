// nmea0183.h — NMEA 0183 sentences from a serial port
//
// The board's RS485 terminal (A/B) takes an NMEA 0183 talker directly:
// NMEA 0183 is RS422, which an RS485 receiver reads. The transceiver switches
// direction by itself, and this code only ever receives.
//
// A task on core 0 reads the UART, splits the stream into sentences, checks
// each checksum, and keeps the most recent ones for the web page's traffic
// window. Settings apply immediately (no reboot). "Demonstration" feeds made-up
// sentences from nmea_demo through the same path instead of a serial port.

#ifndef NMEA0183_H
#define NMEA0183_H

#include <Arduino.h>

enum Nmea0183Port : uint8_t {
    NMEA0183_PORT_OFF   = 0,
    NMEA0183_PORT_RS485 = 1,   // RS485 terminal: UART1, RX GPIO15, TX GPIO16
    NMEA0183_PORT_DEMO  = 2,   // made-up sentences from nmea_demo (no hardware)
};

struct Nmea0183Settings {
    uint8_t  port = NMEA0183_PORT_OFF;
    uint32_t baud = 4800;      // NMEA 0183 standard; 38400 for "high speed"/AIS
    bool auto_reconnect = true; // reopen the port when data stops (see nmea0183.cpp)
};

void nmea0183_load(Nmea0183Settings& s);
void nmea0183_save(const Nmea0183Settings& s);
const Nmea0183Settings& nmea0183_settings();   // current (running) settings

bool nmea0183_valid_baud(uint32_t baud);
const char* nmea0183_port_name(uint8_t port);

void nmea0183_begin();                          // at boot: load and start
void nmea0183_apply(const Nmea0183Settings& s); // save, then restart the reader

String nmea0183_status();                       // human-readable state
// Traffic log as JSON: sentences with sequence number > since, plus counters.
String nmea0183_log_json(uint32_t since);

#endif // NMEA0183_H
