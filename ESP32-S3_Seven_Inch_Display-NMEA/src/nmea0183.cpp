// nmea0183.cpp — see nmea0183.h

#include "nmea0183.h"

#include <Preferences.h>
#include <HardwareSerial.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include "esp_heap_caps.h"
#include "nmea_demo.h"

static const char* PREFS_NS = "nmea0183";

// RS485 terminal on the Waveshare ESP32-S3-Touch-LCD-7. Pin directions are
// from Waveshare's Arduino RS485 demo (UART RX = GPIO15, TX = GPIO16); their
// pin table names the nets from the transceiver side, which reads backwards.
// TX is configured even though nothing is sent: it idles HIGH, which keeps the
// auto-direction transceiver in receive mode. A floating TX could drive the bus.
static const int RS485_RX_PIN = 15;
static const int RS485_TX_PIN = 16;

// UART driver receive buffer. 38400 baud is ~3.8 KB/s, so this covers about
// half a second of the reader task not running.
static const size_t N0183_UART_RX_BUF = 2048;

// Recent sentences kept for the web page's traffic window.
static const int N0183_LOG_LINES = 64;
// An NMEA sentence is at most 82 characters including "$" and CR/LF. Longer
// "lines" are noise (usually a wrong baud rate); they're cut at this length.
static const int N0183_LINE_MAX = 96;

enum : uint8_t {
    N0183_CK_NONE = 0,   // valid start, no checksum (allowed for some sentences)
    N0183_CK_OK   = 1,   // checksum matches
    N0183_CK_BAD  = 2,   // checksum present but wrong
    N0183_CK_JUNK = 3,   // not a sentence: wrong baud, swapped A/B, or noise
    N0183_CK_INFO = 4,   // a note from the display itself (port opened etc.)
};

struct LogEntry {
    uint32_t seq;
    uint8_t  kind;
    char     text[N0183_LINE_MAX];
};

static Nmea0183Settings settings;
static bool settings_loaded = false;

static LogEntry*         g_log = nullptr;      // ring of N0183_LOG_LINES, in PSRAM
static uint32_t          g_seq = 0;            // sequence number of the newest entry
static SemaphoreHandle_t g_log_mutex = nullptr;

static volatile uint32_t st_sentences = 0;
static volatile uint32_t st_bad = 0;
static volatile uint32_t st_junk = 0;
static volatile uint32_t st_bytes = 0;
static volatile uint32_t st_last_sentence_ms = 0;
static volatile uint32_t st_last_junk_ms = 0;
static volatile uint32_t st_opened_ms = 0;

static TaskHandle_t  task_handle = nullptr;
static volatile bool task_run = false;
static bool          uart_open = false;

// ── Settings (own NVS namespace) ────────────────────────────────────────
bool nmea0183_valid_baud(uint32_t baud) {
    switch (baud) {
        case 4800: case 9600: case 19200: case 38400: case 57600: case 115200: return true;
        default: return false;
    }
}

const char* nmea0183_port_name(uint8_t port) {
    switch (port) {
        case NMEA0183_PORT_RS485: return "RS485 terminal";
        case NMEA0183_PORT_DEMO:  return "Demonstration";
        default:                  return "Off";
    }
}

void nmea0183_load(Nmea0183Settings& s) {
    s = Nmea0183Settings();
    Preferences p;
    if (p.begin(PREFS_NS, true)) {
        s.port = p.getUChar("port", NMEA0183_PORT_OFF);
        s.baud = p.getULong("baud", 4800);
        s.auto_reconnect = p.getBool("reconnect", true);
        p.end();
    }
    if (s.port > NMEA0183_PORT_DEMO) s.port = NMEA0183_PORT_OFF;
    if (!nmea0183_valid_baud(s.baud)) s.baud = 4800;
}

void nmea0183_save(const Nmea0183Settings& s) {
    Preferences p;
    if (!p.begin(PREFS_NS, false)) {
        Serial.println("[0183] ERROR: could not open settings");
        return;
    }
    p.putUChar("port", s.port);
    p.putULong("baud", s.baud);
    p.putBool("reconnect", s.auto_reconnect);
    p.end();
}

const Nmea0183Settings& nmea0183_settings() {
    if (!settings_loaded) {
        nmea0183_load(settings);
        settings_loaded = true;
    }
    return settings;
}

// ── Traffic log ─────────────────────────────────────────────────────────
static void log_add(uint8_t kind, const char* text, size_t len) {
    if (!g_log || !g_log_mutex) return;
    if (len > N0183_LINE_MAX - 1) len = N0183_LINE_MAX - 1;
    xSemaphoreTake(g_log_mutex, portMAX_DELAY);
    uint32_t seq = g_seq + 1;
    LogEntry& e = g_log[seq % N0183_LOG_LINES];
    e.seq = seq;
    e.kind = kind;
    memcpy(e.text, text, len);
    e.text[len] = '\0';
    g_seq = seq;
    xSemaphoreGive(g_log_mutex);
}

static void log_info(const char* fmt, ...) {
    char tmp[N0183_LINE_MAX];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(tmp, sizeof(tmp), fmt, ap);
    va_end(ap);
    if (n < 0) return;
    log_add(N0183_CK_INFO, tmp, strlen(tmp));
    Serial.printf("[0183] %s\n", tmp);
}

static int hex_val(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

// Different sentence types seen since the port opened ("GGA", "MWV"...), for
// the status line. Only the active source's task writes these.
static const int N0183_MAX_TYPES = 48;
static char             seen_types[N0183_MAX_TYPES][4];
static volatile int     seen_count = 0;

static void note_type(const char* line, size_t len) {
    // Address field: "$GPGGA," -> talker "GP" + type "GGA". Proprietary
    // sentences ("$PGRME,") have no talker, so keep their first 3 letters.
    const char* comma = (const char*)memchr(line, ',', len);
    size_t alen = comma ? (size_t)(comma - line) - 1 : len - 1;
    if (alen < 3) return;
    const char* t = (line[1] == 'P') ? line + 2 : line + 1 + alen - 3;
    for (int i = 0; i < seen_count; ++i)
        if (memcmp(seen_types[i], t, 3) == 0) return;
    if (seen_count >= N0183_MAX_TYPES) return;
    memcpy(seen_types[seen_count], t, 3);
    seen_types[seen_count][3] = '\0';
    seen_count = seen_count + 1;
}

// Classify one received line, update the counters and log it.
static void finish_line(const char* line, size_t len, bool had_garbage) {
    uint8_t kind;
    if (had_garbage || len < 6 || (line[0] != '$' && line[0] != '!')) {
        kind = N0183_CK_JUNK;
    } else {
        kind = N0183_CK_NONE;
        const char* star = (const char*)memchr(line, '*', len);
        if (star) {
            size_t after = len - (size_t)(star - line) - 1;   // characters after '*'
            int hi = after >= 1 ? hex_val(star[1]) : -1;
            int lo = after >= 2 ? hex_val(star[2]) : -1;
            if (hi >= 0 && lo >= 0) {
                uint8_t sum = 0;
                for (const char* p = line + 1; p < star; ++p) sum ^= (uint8_t)*p;
                kind = (sum == (uint8_t)((hi << 4) | lo)) ? N0183_CK_OK : N0183_CK_BAD;
            } else {
                kind = N0183_CK_BAD;   // "*" with no valid hex after it
            }
        }
    }

    uint32_t now = millis();
    if (kind == N0183_CK_JUNK) {
        st_junk = st_junk + 1;
        st_last_junk_ms = now;
    } else {
        st_sentences = st_sentences + 1;
        if (kind == N0183_CK_BAD) st_bad = st_bad + 1;
        else { st_last_sentence_ms = now; note_type(line, len); }
    }
    log_add(kind, line, len);
}

// ── Reader task (core 0, like the NMEA 2000 tasks) ──────────────────────
// Auto-reconnect: NMEA talkers send at least once a second, so a port that has
// been silent for RECONNECT_SILENCE_MS is treated as dropped (talker rebooted,
// cable pulled, UART stuck after a line fault). The port is then closed and
// reopened, waiting longer after each attempt that brings nothing back:
// 5 s, 10 s, 20 s, 40 s, then every 60 s. The first byte that arrives resets
// the wait. Unreadable data (wrong baud, swapped A/B) still counts as data:
// reopening can't fix that, so the port is left alone.
static const uint32_t RECONNECT_SILENCE_MS = 10000;
static const uint32_t RECONNECT_FIRST_MS   = 5000;
static const uint32_t RECONNECT_MAX_MS     = 60000;

static volatile uint32_t st_last_byte_ms = 0;
static volatile uint32_t rc_attempts = 0;      // reopen attempts since data stopped
static volatile uint32_t rc_next_ms = 0;       // when the next attempt is due
static bool demo_active = false;

static void uart_begin() {
    Serial1.setRxBufferSize(N0183_UART_RX_BUF);   // must come before begin()
    Serial1.begin(settings.baud, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
    uart_open = true;
}

static void reader_task(void*) {
    char line[N0183_LINE_MAX];
    size_t len = 0;
    bool garbage = false;
    uint8_t buf[128];
    uint32_t wait_ms = RECONNECT_FIRST_MS;

    while (task_run) {
        uint32_t now = millis();
        int avail = Serial1.available();

        if (avail <= 0) {
            // Silent port: reopen it, with a growing wait between attempts
            uint32_t since = st_last_byte_ms ? st_last_byte_ms : st_opened_ms;
            if (settings.auto_reconnect && now - since >= RECONNECT_SILENCE_MS &&
                (rc_attempts == 0 || (int32_t)(now - rc_next_ms) >= 0)) {
                Serial1.end();
                uart_begin();
                len = 0;
                garbage = false;
                rc_attempts = rc_attempts + 1;
                rc_next_ms = now + wait_ms;
                log_info("No data for %lu s: reopened the port (attempt %lu, next in %lu s)",
                         (unsigned long)((now - since) / 1000), (unsigned long)rc_attempts,
                         (unsigned long)(wait_ms / 1000));
                wait_ms = wait_ms * 2 > RECONNECT_MAX_MS ? RECONNECT_MAX_MS : wait_ms * 2;
            }
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        size_t n = Serial1.read(buf, avail < (int)sizeof(buf) ? (size_t)avail : sizeof(buf));
        st_bytes = st_bytes + n;
        st_last_byte_ms = now;
        if (rc_attempts) {
            log_info("Data is back after %lu reconnect attempt%s", (unsigned long)rc_attempts, rc_attempts == 1 ? "" : "s");
            rc_attempts = 0;
            wait_ms = RECONNECT_FIRST_MS;
        }
        for (size_t i = 0; i < n; ++i) {
            char c = (char)buf[i];
            if (c == '\r' || c == '\n') {
                if (len) finish_line(line, len, garbage);
                len = 0;
                garbage = false;
                continue;
            }
            if (len >= (size_t)N0183_LINE_MAX - 1) {   // too long to be a sentence
                finish_line(line, len, true);
                len = 0;
                garbage = false;
            }
            if ((uint8_t)c < 0x20 || (uint8_t)c > 0x7E) {
                garbage = true;
                c = '?';
            }
            line[len++] = c;
        }
        vTaskDelay(pdMS_TO_TICKS(1));   // always yield so core 0 stays responsive
    }

    task_handle = nullptr;
    vTaskDelete(nullptr);
}

// Demonstration: sentences arrive whole from nmea_demo, on its task.
static void demo_line(const char* sentence, size_t len) {
    st_bytes = st_bytes + len + 2;          // + CR/LF, as on a real port
    st_last_byte_ms = millis();
    finish_line(sentence, len, false);
}

static void stop_reader() {
    if (demo_active) {
        nmea_demo_stop_0183();
        demo_active = false;
        log_info("Demonstration data stopped");
    }
    if (task_handle) {
        task_run = false;
        uint32_t t0 = millis();
        while (task_handle && millis() - t0 < 500) vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (uart_open) {
        Serial1.end();
        uart_open = false;
        log_info("Port closed");
    }
}

static void start_reader() {
    if (settings.port == NMEA0183_PORT_OFF) return;

    st_sentences = st_bad = st_junk = st_bytes = 0;
    st_last_sentence_ms = st_last_junk_ms = st_last_byte_ms = 0;
    seen_count = 0;
    rc_attempts = 0;
    st_opened_ms = millis();

    if (settings.port == NMEA0183_PORT_DEMO) {
        log_info("Demonstration data: %d sentence types", nmea_demo_sentence_count());
        demo_active = true;
        nmea_demo_start_0183(demo_line);
        return;
    }

    uart_begin();
    log_info("Listening on %s at %lu baud%s", nmea0183_port_name(settings.port),
             (unsigned long)settings.baud, settings.auto_reconnect ? ", reconnecting automatically" : "");

    task_run = true;
    if (xTaskCreatePinnedToCore(reader_task, "NMEA0183", 4096, nullptr, 2, &task_handle, 0) != pdPASS) {
        task_run = false;
        task_handle = nullptr;
        Serial1.end();
        uart_open = false;
        log_info("Couldn't start the reader (out of memory)");
    }
}

void nmea0183_begin() {
    if (!g_log_mutex) g_log_mutex = xSemaphoreCreateMutex();
    if (!g_log) {
        size_t bytes = sizeof(LogEntry) * N0183_LOG_LINES;
        g_log = (LogEntry*)heap_caps_calloc(1, bytes, MALLOC_CAP_SPIRAM);
        if (!g_log) g_log = (LogEntry*)calloc(1, bytes);
    }
    nmea0183_settings();   // load
    start_reader();
}

void nmea0183_apply(const Nmea0183Settings& s) {
    nmea0183_save(s);
    stop_reader();
    settings = s;
    settings_loaded = true;
    start_reader();
}

// ── Status and traffic for the web page ─────────────────────────────────
String nmea0183_status() {
    if (settings.port == NMEA0183_PORT_OFF) return "Off";

    uint32_t now = millis();
    bool recent_ok   = st_last_sentence_ms && now - st_last_sentence_ms < 5000;
    bool recent_junk = st_last_junk_ms && now - st_last_junk_ms < 5000;

    if (settings.port == NMEA0183_PORT_DEMO) {
        if (!nmea_demo_0183_running()) return "Demonstration data not running";
        return "Demonstration data: " + String(nmea_demo_sentence_count()) + " sentence types, " +
               String(st_sentences) + " sentences generated";
    }

    if (!uart_open || !task_handle) return "Port not open";
    String where = String(nmea0183_port_name(settings.port)) + " at " + String(settings.baud) + " baud";

    if (recent_ok)
        return "Receiving on " + where + ": " + String(seen_count) + " sentence type" + (seen_count == 1 ? "" : "s") +
               ", " + String(st_sentences) + " sentences received";
    if (recent_junk)
        return "Receiving data on " + where + " that isn't NMEA 0183: check the baud rate, or swap the A and B wires";
    if (rc_attempts) {
        int32_t next = (int32_t)(rc_next_ms - now);
        return "No data on " + where + ": reconnecting (attempt " + String(rc_attempts) +
               (next > 0 ? ", next in " + String((next + 999) / 1000) + " s)" : ")");
    }
    if (st_bytes == 0) return "Listening on " + where + ", no data yet";
    uint32_t last = st_last_byte_ms ? st_last_byte_ms : st_opened_ms;
    return "No data on " + where + " for " + String((now - last) / 1000) + " s";
}

String nmea0183_log_json(uint32_t since) {
    String j;
    j.reserve(512);
    j += "{\"st\":\"";
    String st = nmea0183_status();
    for (size_t i = 0; i < st.length(); ++i) {
        char c = st[i];
        if (c == '"' || c == '\\') j += '\\';
        j += c;
    }
    j += "\",\"n\":";   j += String(st_sentences);
    j += ",\"bad\":";   j += String(st_bad);
    j += ",\"junk\":";  j += String(st_junk);
    j += ",\"bytes\":"; j += String(st_bytes);

    if (!g_log || !g_log_mutex) {
        j += ",\"seq\":0,\"l\":[]}";
        return j;
    }

    xSemaphoreTake(g_log_mutex, portMAX_DELAY);
    uint32_t newest = g_seq;
    if (since > newest) since = 0;                          // display restarted
    uint32_t oldest = newest > (uint32_t)N0183_LOG_LINES ? newest - N0183_LOG_LINES + 1 : 1;
    uint32_t from = since + 1 > oldest ? since + 1 : oldest;
    j += ",\"seq\":";
    j += String(newest);
    j += ",\"l\":[";
    bool first = true;
    for (uint32_t s = from; s <= newest && newest > 0; ++s) {
        const LogEntry& e = g_log[s % N0183_LOG_LINES];
        if (!first) j += ',';
        first = false;
        j += '[';
        j += String(e.kind);
        j += ",\"";
        for (const char* p = e.text; *p; ++p) {   // printable ASCII only (see reader)
            if (*p == '"' || *p == '\\') j += '\\';
            j += *p;
        }
        j += "\"]";
    }
    xSemaphoreGive(g_log_mutex);
    j += "]}";
    return j;
}
