// nmea_demo.h — "Demonstration" data for NMEA 2000 and NMEA 0183
//
// Simulates a twin-engine motor boat cruising in slow loops, and turns it into
//   - NMEA 2000 messages (built with the NMEA2000 library's SetN2k...
//     functions) for every PGN the display decodes, and
//   - NMEA 0183 sentences (GGA, RMC, GLL, VTG, GSA, GSV, ZDA, HDG, HDM, HDT,
//     DPT, DBT, MWV, VWR, VHW, MTW).
//
// Each output is passed to the same handler its real source uses (the CAN
// port's message handler, the serial port's line handler), so everything
// downstream runs exactly as with real data. Both outputs come from the one
// simulated boat, so they agree with each other when both are on.
//
// Everything that sets what the demo shows (location, speeds, engine figures,
// tank sizes, AIS targets, satellites, send rates) is grouped at the top of
// nmea_demo.cpp.

#ifndef NMEA_DEMO_H
#define NMEA_DEMO_H

#include <Arduino.h>
#include <N2kMsg.h>

typedef void (*NmeaDemoN2kHandler)(const tN2kMsg& msg);
// One sentence, starting with '$' and ending with the "*hh" checksum (no CR/LF).
typedef void (*NmeaDemo0183Handler)(const char* sentence, size_t len);

// NMEA 2000 output
void     nmea_demo_start_n2k(NmeaDemoN2kHandler handler);
void     nmea_demo_stop_n2k();
bool     nmea_demo_n2k_running();
int      nmea_demo_pgn_count();          // different PGNs generated
uint32_t nmea_demo_n2k_msg_count();      // messages since the output started
String   nmea_demo_n2k_status();         // for the Data Setup page

// NMEA 0183 output
void     nmea_demo_start_0183(NmeaDemo0183Handler handler);
void     nmea_demo_stop_0183();
bool     nmea_demo_0183_running();
int      nmea_demo_sentence_count();     // different sentence types generated

#endif // NMEA_DEMO_H
