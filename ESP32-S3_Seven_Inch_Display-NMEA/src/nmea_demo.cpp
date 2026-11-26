// nmea_demo.cpp — see nmea_demo.h
//
// Layout:
//   1. Scenario: the numbers to change to change what the demo shows
//   2. Helpers: random numbers, smooth "wander" values, unit conversions
//   3. Simulation: one boat, its GPS satellites and a few AIS targets,
//      stepped 10 times a second
//   4. NMEA 2000: one small function per PGN, reading the simulation
//   5. NMEA 0183: one small function per sentence, reading the simulation
//   6. Schedules: which builder runs how often
//   7. Task, start/stop and status
//
// NMEA 2000 and NMEA 0183 builders read the same simulation values, never
// their own random numbers, so the same quantity (depth, wind, heading,
// position, speed...) shows the same on both. Any jitter belongs in sim_step().

#include "nmea_demo.h"

#include <N2kMessages.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>      // esp_random()
#include <math.h>
#include <stdarg.h>
#include <time.h>

// ═════════════════════════════════════════════════════════════════════════
// 1. Scenario
// ═════════════════════════════════════════════════════════════════════════
namespace scenario {
    // Where the boat starts, and how its course turns. It sails one slow loop
    // every LOOP_MINUTES so it stays in the same area however long it runs.
    const double START_LAT        = 50.790;    // the Solent, off Cowes
    const double START_LON        = -1.290;
    const float  LOOP_MINUTES     = 45.0f;
    const float  COURSE_WANDER    = 25.0f;     // ± degrees either side of the loop course
    const double WAYPOINT_LAT     = 50.815;    // destination shown by 129284
    const double WAYPOINT_LON     = -1.240;
    const float  MAG_VARIATION    = -1.0f;     // degrees (negative = west)
    const float  COMPASS_DEVIATION = 0.5f;     // degrees

    // Engines (twin diesels) and boat speed
    const float  IDLE_RPM         = 700.0f;
    const float  MAX_RPM          = 3400.0f;
    const float  TOP_SPEED_KN     = 18.0f;     // speed through water at full throttle
    const float  MAX_FUEL_LPH     = 45.0f;     // per engine, at full throttle
    const float  START_ENGINE_HOURS = 1234.5f;
    const float  START_COOLANT_C  = 45.0f;     // engines start part-warm and warm up

    // Weather and sea
    const float  WIND_DIR_TRUE    = 235.0f;    // degrees, direction the wind comes from
    const float  WIND_DIR_WANDER  = 20.0f;
    const float  WIND_SPEED_MIN   = 9.0f;      // knots; it gusts between these
    const float  WIND_SPEED_MAX   = 17.0f;
    const float  CURRENT_SET      = 70.0f;     // degrees, direction the tide flows towards
    const float  CURRENT_MIN_KN   = 0.3f;
    const float  CURRENT_MAX_KN   = 1.2f;
    const float  AIR_TEMP_MIN     = 17.0f,  AIR_TEMP_MAX = 23.0f;   // °C
    const float  SEA_TEMP_MIN     = 15.5f,  SEA_TEMP_MAX = 16.8f;
    const float  PRESSURE_MIN_HPA = 1006.0f, PRESSURE_MAX_HPA = 1021.0f;
    const float  HUMIDITY_MIN     = 55.0f,  HUMIDITY_MAX = 82.0f;   // %
    const float  DEPTH_MIN_M      = 6.0f,   DEPTH_MAX_M = 38.0f;
    const float  TRANSDUCER_TO_KEEL_M = 0.9f;

    // Tanks: capacity (litres) and starting level (%)
    const float  FUEL_CAPACITY_L  = 800.0f,  FUEL_START   = 68.0f;
    const float  WATER_CAPACITY_L = 300.0f,  WATER_START  = 74.0f;
    const float  GREY_CAPACITY_L  = 100.0f,  GREY_START   = 28.0f;
    const float  BLACK_CAPACITY_L = 80.0f,   BLACK_START  = 21.0f;
    const float  OIL_CAPACITY_L   = 20.0f,   OIL_START    = 88.0f;
    const float  LIVEWELL_CAPACITY_L = 40.0f, LIVEWELL_START = 55.0f;

    // Batteries
    const float  HOUSE_AH         = 400.0f;    // house bank capacity
    const float  HOUSE_SOC_START  = 72.0f;     // %
    const float  HOUSE_LOAD_MIN_A = 5.0f,  HOUSE_LOAD_MAX_A = 16.0f;
    const float  ALTERNATOR_MAX_A = 60.0f;
    const float  SOLAR_MAX_A      = 12.0f;

    // Log
    const double START_LOG_NM     = 12345.0;

    // AIS targets around the boat
    struct AisDef {
        uint32_t mmsi; const char* name; const char* callsign; uint8_t type;
        bool class_a; float range_nm; float bearing; float cog; float sog_kn;
        tN2kAISNavStatus status; float length_m; float beam_m;
    };
    const AisDef AIS[] = {
        // mmsi       name              call     type A?    nm  brg  cog   kn   status                          len   beam
        {235987001, "NORTHERN STAR",  "MNST1",  70, true,  2.5,  40, 250, 11.0, N2kaisns_Under_Way_Motoring,    120,  18},
        {235987002, "SOLENT EXPRESS", "MSEX2",  60, true,  1.8, 300, 110, 14.0, N2kaisns_Under_Way_Motoring,     45,  11},
        {235987003, "KESTREL",        "MKES3",  36, false, 0.9, 160,  20,  5.5, N2kaisns_Under_Way_Sailing,      11,   3.6},
        {235987004, "BLUE HORIZON",   "MBLH4",  37, false, 1.4, 220, 330,  8.0, N2kaisns_Under_Way_Motoring,     14,   4.2},
        {235987005, "GANNET",         "MGAN5",  30, true,  3.2, 100, 200,  3.0, N2kaisns_Fishing,                18,   6},
        {235987006, "ISLAND SPIRIT",  "MISP6",  37, true,  0.7,  10,   0,  0.0, N2kaisns_At_Anchor,              22,   5.5},
    };
    const float AIS_MAX_RANGE_NM  = 6.0f;      // targets further than this turn back

    // Send rates (ms). Real devices send the fast ones at 10 Hz; 5 Hz is plenty
    // for the display and keeps the load down.
    const uint32_t RATE_FAST_MS   = 200;
    const uint32_t RATE_WIND_MS   = 500;
    const uint32_t RATE_NORMAL_MS = 1000;
    const uint32_t RATE_SLOW_MS   = 2500;
    const uint32_t AIS_CLASS_A_MS = 4000;
    const uint32_t AIS_CLASS_B_MS = 10000;
    const uint32_t AIS_STATIC_MS  = 30000;

    // GPS satellites in view (PRN numbers). The receiver uses the highest
    // ones, up to the satellite count shown in GGA / PGN 129029.
    const uint8_t SAT_PRNS[] = {2, 5, 7, 9, 11, 13, 15, 18, 20, 23, 26, 29, 30, 31};
    const float   GEOID_SEPARATION_M = 47.3f;  // shown in GGA / PGN 129029

    // NMEA 0183: local time zone in ZDA (hours, minutes) and send rates (ms)
    const int      ZDA_ZONE_HOURS   = 0;
    const int      ZDA_ZONE_MINUTES = 0;
    const uint32_t RATE_0183_GPS_MS     = 1000;   // GGA RMC GLL VTG ZDA
    const uint32_t RATE_0183_GSA_MS     = 2000;
    const uint32_t RATE_0183_GSV_MS     = 5000;
    const uint32_t RATE_0183_HEADING_MS = 500;    // HDG HDM HDT
    const uint32_t RATE_0183_WIND_MS    = 500;    // MWV VWR
    const uint32_t RATE_0183_NORMAL_MS  = 1000;   // DPT DBT VHW
    const uint32_t RATE_0183_SLOW_MS    = 2000;   // MTW
}

// If the display's clock hasn't been set, the demo's date/time starts here
// (2026-06-01 10:00 UTC) and counts up from power-on.
static const time_t FALLBACK_EPOCH = 1780308000;

// ═════════════════════════════════════════════════════════════════════════
// 2. Helpers
// ═════════════════════════════════════════════════════════════════════════
static float frand(float lo, float hi) {
    return lo + (hi - lo) * ((esp_random() & 0xFFFFFF) / 16777215.0f);
}

static float norm360(float d) {
    d = fmodf(d, 360.0f);
    return d < 0 ? d + 360.0f : d;
}

static float norm180(float d) {
    d = norm360(d);
    return d > 180.0f ? d - 360.0f : d;
}

// Moves smoothly towards a target, like a real quantity with inertia.
static float approach(float cur, float target, float tau_s, float dt) {
    return cur + (target - cur) * (1.0f - expf(-dt / tau_s));
}

// A value that drifts around a range: it picks a new random target every few
// seconds (between hold_min and hold_max) and eases towards it.
struct Wander {
    float lo, hi, hold_min, hold_max, tau;
    float value, target, hold_left;

    void init(float lo_, float hi_, float hold_min_, float hold_max_, float tau_, float start) {
        lo = lo_; hi = hi_; hold_min = hold_min_; hold_max = hold_max_; tau = tau_;
        value = target = start;
        hold_left = frand(hold_min, hold_max);
    }
    float step(float dt) {
        hold_left -= dt;
        if (hold_left <= 0) {
            target = frand(lo, hi);
            hold_left = frand(hold_min, hold_max);
        }
        value = approach(value, target, tau, dt);
        return value;
    }
};

static inline double K(double c)       { return c + 273.15; }        // °C -> K
static inline double RAD(double deg)   { return deg * DEG_TO_RAD; }
static inline double MS(double knots)  { return knots * 1852.0 / 3600.0; }
static inline double PA(double hpa)    { return hpa * 100.0; }

static void utc_now(uint16_t& days, double& secs) {
    time_t now = time(nullptr);
    if (now < 1700000000) now = FALLBACK_EPOCH + (time_t)(millis() / 1000);
    days = (uint16_t)(now / 86400);
    secs = (double)(now % 86400) + (millis() % 1000) / 1000.0;
}

// Flat-earth distance and bearing: accurate enough over a few miles.
static void dist_bearing(double lat1, double lon1, double lat2, double lon2, double& dist_m, float& brg_deg) {
    double dn = (lat2 - lat1) * 111320.0;
    double de = (lon2 - lon1) * 111320.0 * cos(lat1 * DEG_TO_RAD);
    dist_m = sqrt(dn * dn + de * de);
    brg_deg = norm360((float)(atan2(de, dn) * RAD_TO_DEG));
}

static void move(double& lat, double& lon, float course_deg, float speed_kn, float dt) {
    double m = MS(speed_kn) * dt;
    lat += m * cos(course_deg * DEG_TO_RAD) / 111320.0;
    lon += m * sin(course_deg * DEG_TO_RAD) / (111320.0 * cos(lat * DEG_TO_RAD));
}

// ═════════════════════════════════════════════════════════════════════════
// 3. Simulation
// ═════════════════════════════════════════════════════════════════════════
struct Engine {
    float rpm, boost_kpa, coolant_c, oil_c, oil_kpa, alt_v, fuel_lph, exhaust_c, hours_s;
    float trans_oil_c, trans_oil_kpa;
};

struct AisTarget {
    const scenario::AisDef* def;
    double lat, lon;
    float cog, sog, heading, rot;
    uint32_t next_pos_ms, next_static_ms;
};

static struct Sim {
    float t;                             // seconds since start
    Wander throttle, course_offset, rpm_split, tilt, tab_split;
    Wander current_set, current_drift, leeway;
    Wander wind_dir, wind_speed, air, sea, pressure, humidity, cabin, depth;
    Wander house_load, solar, gnss_alt, gnss_hdop, gnss_sats, xte, livewell, list;

    Engine eng[2];
    float throttle_now;                  // 0..1 after the lever has moved
    float load;                          // 0..1
    float stw_kn, sog_kn, cog, heading, prev_heading, rot_dps, rudder;
    float loop_course;
    float roll, pitch;
    double lat, lon;
    double trip_m;
    float tws_kn, twd, twa, aws_kn, awa;
    float trip_fuel_l, trip_hours;
    float fuel_l, water_l, grey_l, black_l, oil_l;
    float house_soc, house_v, house_a, start_v, start_a, batt_c;
    float fridge_c, fridge_dir;
    float heating_c;
    double wpt_dist_m; float wpt_brg, vmg_kn;
    float depth_m;                       // depth below transducer, with sounder jitter
    uint8_t sid;

    AisTarget ais[sizeof(scenario::AIS) / sizeof(scenario::AIS[0])];

    struct Sat { uint8_t prn; float elev, az, snr; } sats[sizeof(scenario::SAT_PRNS)];
} S;

static const int SAT_COUNT = sizeof(scenario::SAT_PRNS);

static const int AIS_COUNT = sizeof(scenario::AIS) / sizeof(scenario::AIS[0]);

static void sim_init() {
    using namespace scenario;
    memset(&S, 0, sizeof(S));

    S.throttle.init(0.0f, 1.0f, 25, 90, 4, 0.45f);
    S.course_offset.init(-COURSE_WANDER, COURSE_WANDER, 20, 60, 15, 0);
    S.rpm_split.init(-30, 30, 10, 40, 5, 0);             // twin engines never quite match
    S.tilt.init(-5, 20, 30, 90, 6, 5);                   // drive trim, %
    S.tab_split.init(-4, 4, 15, 45, 8, 0);
    S.current_set.init(CURRENT_SET - 20, CURRENT_SET + 20, 60, 180, 60, CURRENT_SET);
    S.current_drift.init(CURRENT_MIN_KN, CURRENT_MAX_KN, 60, 180, 60, (CURRENT_MIN_KN + CURRENT_MAX_KN) / 2);
    S.leeway.init(-2.5f, 2.5f, 20, 60, 15, 0);
    S.wind_dir.init(WIND_DIR_TRUE - WIND_DIR_WANDER, WIND_DIR_TRUE + WIND_DIR_WANDER, 30, 90, 20, WIND_DIR_TRUE);
    S.wind_speed.init(WIND_SPEED_MIN, WIND_SPEED_MAX, 4, 15, 4, (WIND_SPEED_MIN + WIND_SPEED_MAX) / 2);
    S.air.init(AIR_TEMP_MIN, AIR_TEMP_MAX, 300, 900, 300, (AIR_TEMP_MIN + AIR_TEMP_MAX) / 2);
    S.sea.init(SEA_TEMP_MIN, SEA_TEMP_MAX, 600, 1800, 600, (SEA_TEMP_MIN + SEA_TEMP_MAX) / 2);
    S.pressure.init(PRESSURE_MIN_HPA, PRESSURE_MAX_HPA, 900, 2400, 1200, 1013.0f);
    S.humidity.init(HUMIDITY_MIN, HUMIDITY_MAX, 300, 900, 300, 68.0f);
    S.cabin.init(20.0f, 23.0f, 300, 900, 300, 21.5f);
    S.depth.init(DEPTH_MIN_M, DEPTH_MAX_M, 20, 60, 25, 14.0f);
    S.house_load.init(HOUSE_LOAD_MIN_A, HOUSE_LOAD_MAX_A, 20, 120, 5, 9.0f);
    S.solar.init(0.0f, SOLAR_MAX_A, 60, 300, 40, SOLAR_MAX_A * 0.5f);
    S.gnss_alt.init(0.8f, 2.6f, 10, 40, 10, 1.6f);
    S.gnss_hdop.init(0.7f, 1.3f, 20, 90, 20, 0.9f);
    S.gnss_sats.init(9.0f, 13.4f, 30, 120, 10, 11.0f);
    S.xte.init(-60.0f, 60.0f, 20, 60, 20, 0);             // metres
    S.livewell.init(LIVEWELL_START - 10, LIVEWELL_START + 10, 60, 240, 60, LIVEWELL_START);
    S.list.init(-1.5f, 1.5f, 60, 240, 60, 0.5f);          // static heel from loading, degrees

    S.lat = START_LAT;
    S.lon = START_LON;
    S.loop_course = 0;
    S.heading = S.prev_heading = S.cog = 0;
    S.throttle_now = 0.45f;

    for (int i = 0; i < 2; ++i) {
        Engine& e = S.eng[i];
        e.rpm = IDLE_RPM;
        e.coolant_c = START_COOLANT_C;
        e.oil_c = START_COOLANT_C + 5;
        e.trans_oil_c = START_COOLANT_C - 5;
        e.exhaust_c = 150;
        e.alt_v = 14.1f;
        e.hours_s = START_ENGINE_HOURS * 3600.0f + i * 7.3f * 3600.0f;
    }

    S.fuel_l  = FUEL_CAPACITY_L  * FUEL_START  / 100.0f;
    S.water_l = WATER_CAPACITY_L * WATER_START / 100.0f;
    S.grey_l  = GREY_CAPACITY_L  * GREY_START  / 100.0f;
    S.black_l = BLACK_CAPACITY_L * BLACK_START / 100.0f;
    S.oil_l   = OIL_CAPACITY_L   * OIL_START   / 100.0f;
    S.house_soc = HOUSE_SOC_START;
    S.batt_c = 24.0f;
    S.fridge_c = 4.0f;
    S.fridge_dir = 1;
    S.heating_c = 60.0f;

    for (int i = 0; i < SAT_COUNT; ++i) {
        S.sats[i].prn  = SAT_PRNS[i];
        S.sats[i].elev = frand(5, 85);
        S.sats[i].az   = frand(0, 360);
        S.sats[i].snr  = 20 + S.sats[i].elev * 0.3f;
    }

    for (int i = 0; i < AIS_COUNT; ++i) {
        const AisDef& d = AIS[i];
        AisTarget& a = S.ais[i];
        a.def = &d;
        a.lat = START_LAT;
        a.lon = START_LON;
        move(a.lat, a.lon, d.bearing, d.range_nm * 3600.0f, 1.0f);   // (range x 3600) knots for 1 s = range in miles
        a.cog = d.cog;
        a.sog = d.sog_kn;
        a.heading = d.cog;
        a.next_pos_ms = millis() + 500 + i * 300;
        a.next_static_ms = millis() + 1000 + i * 400;   // names show up straight away
    }
}

static void sim_step(float dt) {
    using namespace scenario;
    S.t += dt;

    // ── Throttle: mostly cruising, sometimes slowing to idle ──────────────
    float thr = S.throttle.step(dt);
    if (thr < 0.15f) thr = 0.0f;                         // the low end of the range means "idle"
    S.throttle_now = approach(S.throttle_now, thr, 3.0f, dt);
    float f = S.throttle_now;
    S.load = 0.12f + 0.85f * powf(f, 1.4f);

    // ── Engines ───────────────────────────────────────────────────────────
    float split = S.rpm_split.step(dt);
    for (int i = 0; i < 2; ++i) {
        Engine& e = S.eng[i];
        float target_rpm = IDLE_RPM + f * (MAX_RPM - IDLE_RPM) + (i ? split : 0);
        e.rpm = approach(e.rpm, target_rpm, 2.0f, dt) + frand(-4, 4);
        float fr = (e.rpm - IDLE_RPM) / (MAX_RPM - IDLE_RPM);
        if (fr < 0) fr = 0;
        float boost_target = fr > 0.25f ? 120.0f * powf((fr - 0.25f) / 0.75f, 1.5f) : 0.0f;
        e.boost_kpa = approach(e.boost_kpa, boost_target, 1.5f, dt);
        e.fuel_lph  = 1.8f + (MAX_FUEL_LPH - 1.8f) * powf(fr, 2.4f) + frand(-0.1f, 0.1f);
        e.coolant_c = approach(e.coolant_c, 78.0f + 8.0f * S.load, 90.0f, dt);
        e.oil_c     = approach(e.oil_c, 84.0f + 18.0f * S.load, 150.0f, dt);
        e.exhaust_c = approach(e.exhaust_c, 170.0f + 330.0f * S.load, 20.0f, dt);
        // Oil pressure rises with rpm and drops a little as the oil thins when hot
        float thin = 1.0f - 0.15f * fmaxf(0.0f, (e.oil_c - 85.0f) / 20.0f);
        e.oil_kpa   = (160.0f + 360.0f * fr) * thin + frand(-3, 3);
        e.alt_v     = 14.15f + frand(-0.04f, 0.04f);
        e.hours_s  += dt;
        e.trans_oil_c   = approach(e.trans_oil_c, 55.0f + 22.0f * S.load, 120.0f, dt);
        e.trans_oil_kpa = 1200.0f + 900.0f * fr + frand(-10, 10);
    }

    // ── Speed, course and position ────────────────────────────────────────
    float stw_target = f > 0 ? 1.2f + (TOP_SPEED_KN - 1.2f) * powf(f, 1.4f) : 1.0f;
    S.stw_kn = approach(S.stw_kn, stw_target, 10.0f, dt);

    S.loop_course = norm360(S.loop_course + 360.0f / (LOOP_MINUTES * 60.0f) * dt);
    S.prev_heading = S.heading;
    S.heading = norm360(S.loop_course + S.course_offset.step(dt));
    float dh = norm180(S.heading - S.prev_heading);
    S.rot_dps = approach(S.rot_dps, dh / dt, 1.5f, dt);
    S.rudder = approach(S.rudder, fmaxf(-30.0f, fminf(30.0f, S.rot_dps * 8.0f)) + frand(-0.3f, 0.3f), 1.0f, dt);

    float lw = S.leeway.step(dt);
    float set = S.current_set.step(dt), drift = S.current_drift.step(dt);
    // Ground velocity = boat through the water (+ leeway) + tidal current
    float vn = S.stw_kn * cosf(RAD(S.heading + lw)) + drift * cosf(RAD(set));
    float ve = S.stw_kn * sinf(RAD(S.heading + lw)) + drift * sinf(RAD(set));
    S.sog_kn = sqrtf(vn * vn + ve * ve);
    S.cog = norm360(atan2f(ve, vn) * RAD_TO_DEG);
    move(S.lat, S.lon, S.cog, S.sog_kn, dt);
    S.trip_m += MS(S.sog_kn) * dt;

    // ── Attitude: rolling and pitching in a short sea ─────────────────────
    float list = S.list.step(dt);
    S.roll  = list + 4.0f * sinf(2 * PI * S.t / 6.5f) + 1.5f * sinf(2 * PI * S.t / 11.0f + 1.0f) + frand(-0.2f, 0.2f);
    S.pitch = 1.0f + 2.5f * f + 1.2f * sinf(2 * PI * S.t / 4.3f) + frand(-0.1f, 0.1f);

    // ── Wind: true from the scenario, apparent from the boat's motion ─────
    S.twd = norm360(S.wind_dir.step(dt));
    S.tws_kn = S.wind_speed.step(dt) + frand(-0.3f, 0.3f);
    S.twa = norm360(S.twd - S.heading);
    // Air moves towards (twd + 180); apparent = true wind - boat velocity
    float wn = -S.tws_kn * cosf(RAD(S.twd)), we = -S.tws_kn * sinf(RAD(S.twd));
    float bn = S.stw_kn * cosf(RAD(S.heading)), be = S.stw_kn * sinf(RAD(S.heading));
    float an = wn - bn, ae = we - be;
    S.aws_kn = sqrtf(an * an + ae * ae);
    float from = norm360(atan2f(-ae, -an) * RAD_TO_DEG);
    S.awa = norm360(from - S.heading);

    // ── Fuel, tanks and trip ──────────────────────────────────────────────
    float burn = (S.eng[0].fuel_lph + S.eng[1].fuel_lph) / 3600.0f * dt;
    S.fuel_l = fmaxf(0.0f, S.fuel_l - burn);
    S.trip_fuel_l += burn;
    S.trip_hours += dt / 3600.0f;
    S.water_l = fmaxf(0.0f, S.water_l - 0.0015f * dt);   // showers and washing up
    S.grey_l  = fminf(GREY_CAPACITY_L, S.grey_l + 0.0010f * dt);
    S.black_l = fminf(BLACK_CAPACITY_L, S.black_l + 0.0003f * dt);
    S.oil_l   = fmaxf(0.0f, S.oil_l - 0.000005f * dt);

    // ── Batteries ─────────────────────────────────────────────────────────
    float load_a = S.house_load.step(dt);
    float solar_a = S.solar.step(dt);
    // Alternator charge tapers off as the bank fills
    float charge = ALTERNATOR_MAX_A * fminf(1.0f, S.throttle_now * 3 + 0.3f) * fmaxf(0.05f, (100.0f - S.house_soc) / 40.0f);
    S.house_a = charge + solar_a - load_a + frand(-0.3f, 0.3f);
    S.house_soc = fminf(100.0f, fmaxf(5.0f, S.house_soc + S.house_a * dt / 3600.0f / HOUSE_AH * 100.0f));
    S.house_v = 12.2f + 0.012f * S.house_soc + 0.018f * S.house_a;
    S.house_v = fminf(14.4f, fmaxf(11.8f, S.house_v));
    S.start_a = 1.5f + frand(-0.2f, 0.2f);
    S.start_v = 13.95f + frand(-0.03f, 0.03f);
    S.batt_c = approach(S.batt_c, 22.0f + 0.12f * fabsf(S.house_a), 600.0f, dt);

    // ── Fridge cycles between 3 and 5 °C; heating holds around 60 °C ──────
    S.fridge_c += S.fridge_dir * 0.004f * dt;
    if (S.fridge_c > 5.0f) S.fridge_dir = -1;
    if (S.fridge_c < 3.0f) S.fridge_dir = 1;
    S.heating_c = approach(S.heating_c, 60.0f + 4.0f * sinf(2 * PI * S.t / 900.0f), 60.0f, dt);

    // ── Depth: jitter added here, once, so NMEA 2000 and 0183 show the same ─
    S.depth_m = S.depth.step(dt) + frand(-0.05f, 0.05f);

    // ── Waypoint ──────────────────────────────────────────────────────────
    dist_bearing(S.lat, S.lon, WAYPOINT_LAT, WAYPOINT_LON, S.wpt_dist_m, S.wpt_brg);
    S.vmg_kn = S.sog_kn * cosf(RAD(S.cog - S.wpt_brg));

    // ── Everything else that just drifts ──────────────────────────────────
    S.air.step(dt); S.sea.step(dt); S.pressure.step(dt); S.humidity.step(dt);
    S.cabin.step(dt); S.gnss_alt.step(dt); S.gnss_hdop.step(dt);
    S.gnss_sats.step(dt); S.xte.step(dt); S.livewell.step(dt); S.tilt.step(dt); S.tab_split.step(dt);

    // ── Satellites drift slowly across the sky; signal follows elevation ──
    for (int i = 0; i < SAT_COUNT; ++i) {
        auto& sat = S.sats[i];
        sat.az = norm360(sat.az + 0.008f * dt);
        sat.elev = fminf(88.0f, fmaxf(3.0f, sat.elev + ((i & 1) ? 0.004f : -0.004f) * dt));
        sat.snr = approach(sat.snr, 22.0f + sat.elev * 0.28f + frand(-3, 3), 5.0f, dt);
    }

    // ── AIS targets: straight lines, turning back if they get too far away ─
    for (int i = 0; i < AIS_COUNT; ++i) {
        AisTarget& a = S.ais[i];
        if (a.sog <= 0.05f) {                            // at anchor: swing to the wind
            a.heading = approach(a.heading, S.twd, 60.0f, dt);
            continue;
        }
        double d; float brg;
        dist_bearing(S.lat, S.lon, a.lat, a.lon, d, brg);
        float want = a.cog;
        if (d > AIS_MAX_RANGE_NM * 1852.0) want = norm360(brg + 180.0f + frand(-30, 30));
        float turn = norm180(want - a.cog);
        float step = fmaxf(-0.5f * dt, fminf(0.5f * dt, turn));   // 0.5 °/s at most
        a.rot = step / dt;
        a.cog = norm360(a.cog + step);
        a.heading = norm360(a.cog + frand(-1.5f, 1.5f));
        move(a.lat, a.lon, a.cog, a.sog, dt);
    }
}

// ═════════════════════════════════════════════════════════════════════════
// 4. NMEA 2000
// ═════════════════════════════════════════════════════════════════════════
static NmeaDemoN2kHandler volatile g_n2k = nullptr;
static volatile uint32_t g_n2k_msgs = 0;

static void emit(const tN2kMsg& m) {
    NmeaDemoN2kHandler h = g_n2k;
    if (h) h(m);
    g_n2k_msgs = g_n2k_msgs + 1;
}

static void pgn_126992() {                               // System time
    uint16_t days; double secs; utc_now(days, secs);
    tN2kMsg m; SetN2kSystemTime(m, S.sid, days, secs, N2ktimes_GPS); emit(m);
}

static void pgn_127245() {                               // Rudder
    tN2kMsg m; SetN2kRudder(m, RAD(S.rudder)); emit(m);
}

static void pgn_127250() {                               // Heading (true)
    tN2kMsg m;
    SetN2kPGN127250(m, S.sid, RAD(S.heading), RAD(scenario::COMPASS_DEVIATION), RAD(scenario::MAG_VARIATION), N2khr_true);
    emit(m);
}

static void pgn_127251() {                               // Rate of turn
    tN2kMsg m; SetN2kRateOfTurn(m, S.sid, RAD(S.rot_dps)); emit(m);
}

static void pgn_127257() {                               // Attitude
    tN2kMsg m; SetN2kAttitude(m, S.sid, RAD(S.heading), RAD(S.pitch), RAD(S.roll)); emit(m);
}

static void pgn_127258() {                               // Magnetic variation
    uint16_t days; double secs; utc_now(days, secs);
    tN2kMsg m; SetN2kMagneticVariation(m, S.sid, N2kmagvar_WMM2025, days, RAD(scenario::MAG_VARIATION)); emit(m);
}

// The display's handlers keep separate engine 1 / engine 2 values only for
// rpm, oil, coolant, alternator, fuel rate and hours. Boost, trim, load,
// torque, gearbox and exhaust have one field each, so engine 2 sends those as
// "not available"; otherwise the gauges would flick between the two engines.
static void pgn_127488() {                               // Engine rapid, both engines
    for (int i = 0; i < 2; ++i) {
        const Engine& e = S.eng[i];
        tN2kMsg m;
        if (i == 0) SetN2kEngineParamRapid(m, i, e.rpm, e.boost_kpa * 1000.0, (int8_t)S.tilt.value);
        else        SetN2kEngineParamRapid(m, i, e.rpm);
        emit(m);
    }
}

static void pgn_127489() {                               // Engine dynamic, both engines
    for (int i = 0; i < 2; ++i) {
        const Engine& e = S.eng[i];
        tN2kMsg m;
        SetN2kEngineDynamicParam(m, (unsigned char)i, e.oil_kpa * 1000.0, K(e.oil_c), K(e.coolant_c), e.alt_v,
                                 e.fuel_lph, e.hours_s, N2kDoubleNA, N2kDoubleNA,
                                 i == 0 ? (int8_t)(S.load * 100.0f) : N2kInt8NA,
                                 i == 0 ? (int8_t)(S.load * 92.0f) : N2kInt8NA,
                                 tN2kEngineDiscreteStatus1(), tN2kEngineDiscreteStatus2());
        emit(m);
    }
}

static void pgn_127493() {                               // Transmission (engine 1; see above)
    const Engine& e = S.eng[0];
    tN2kMsg m;
    SetN2kTransmissionParameters(m, 0, S.throttle_now > 0.02f ? N2kTG_Forward : N2kTG_Neutral,
                                 e.trans_oil_kpa * 1000.0, K(e.trans_oil_c));
    emit(m);
}

static void pgn_127497() {                               // Engine trip
    float rate_now = S.eng[0].fuel_lph + S.eng[1].fuel_lph;
    float avg = S.trip_hours > 0.001f ? S.trip_fuel_l / S.trip_hours : rate_now;
    tN2kMsg m; SetN2kEngineTripParameters(m, 0, S.trip_fuel_l, avg, avg, rate_now); emit(m);
}

static void pgn_127505() {                               // Fluid levels, one per tank
    using namespace scenario;
    struct { tN2kFluidType type; float litres, capacity; } tanks[] = {
        {N2kft_Fuel,       S.fuel_l,  FUEL_CAPACITY_L},
        {N2kft_Water,      S.water_l, WATER_CAPACITY_L},
        {N2kft_GrayWater,  S.grey_l,  GREY_CAPACITY_L},
        {N2kft_BlackWater, S.black_l, BLACK_CAPACITY_L},
        {N2kft_Oil,        S.oil_l,   OIL_CAPACITY_L},
        {N2kft_LiveWell,   S.livewell.value * LIVEWELL_CAPACITY_L / 100.0f, LIVEWELL_CAPACITY_L},
    };
    for (auto& t : tanks) {
        tN2kMsg m; SetN2kFluidLevel(m, 0, t.type, t.litres / t.capacity * 100.0, t.capacity); emit(m);
    }
}

static void pgn_127506() {                               // DC detailed status (house bank)
    float hours = S.house_a < -0.5f ? S.house_soc / 100.0f * scenario::HOUSE_AH / -S.house_a
                                    : S.house_soc / 100.0f * scenario::HOUSE_AH / fmaxf(1.0f, S.house_load.value);
    tN2kMsg m;
    SetN2kDCStatus(m, S.sid, 0, N2kDCt_Battery, (unsigned char)S.house_soc, 97, hours * 3600.0, 0.02);
    emit(m);
}

static void pgn_127508() {                               // Battery status, house (0) and start (1)
    tN2kMsg m;
    SetN2kDCBatStatus(m, 0, S.house_v, S.house_a, K(S.batt_c), S.sid); emit(m);
    // One battery temperature field on the display, so the start bank sends none
    SetN2kDCBatStatus(m, 1, S.start_v, S.start_a, N2kDoubleNA, S.sid); emit(m);
}

static void pgn_127751() {                               // DC voltage/current (solar controller)
    tN2kMsg m; SetN2kDCVoltageCurrent(m, 0, S.house_v + 0.15f, S.solar.value, S.sid); emit(m);
}

static void pgn_128000() {                               // Leeway
    tN2kMsg m; SetN2kLeeway(m, S.sid, RAD(S.leeway.value)); emit(m);
}

static void pgn_128259() {                               // Boat speed
    tN2kMsg m; SetN2kBoatSpeed(m, S.sid, MS(S.stw_kn), MS(S.sog_kn), N2kSWRT_Paddle_wheel); emit(m);
}

static void pgn_128267() {                               // Depth
    tN2kMsg m;
    SetN2kWaterDepth(m, S.sid, S.depth_m, -scenario::TRANSDUCER_TO_KEEL_M, 100.0);
    emit(m);
}

static void pgn_128275() {                               // Distance log
    uint16_t days; double secs; utc_now(days, secs);
    uint32_t total = (uint32_t)(scenario::START_LOG_NM * 1852.0 + S.trip_m);
    tN2kMsg m; SetN2kDistanceLog(m, days, secs, total, (uint32_t)S.trip_m); emit(m);
}

static void pgn_129025() {                               // Position rapid
    tN2kMsg m; SetN2kLatLonRapid(m, S.lat, S.lon); emit(m);
}

static void pgn_129026() {                               // COG & SOG rapid
    tN2kMsg m; SetN2kCOGSOGRapid(m, S.sid, N2khr_true, RAD(S.cog), MS(S.sog_kn)); emit(m);
}

static void pgn_129029() {                               // GNSS position data
    uint16_t days; double secs; utc_now(days, secs);
    float hdop = S.gnss_hdop.value;
    tN2kMsg m;
    SetN2kGNSS(m, S.sid, days, secs, S.lat, S.lon, S.gnss_alt.value,
               N2kGNSSt_GPSGLONASS, N2kGNSSm_GNSSfix, (unsigned char)S.gnss_sats.value,
               hdop, hdop * 1.6, scenario::GEOID_SEPARATION_M);
    emit(m);
}

static void pgn_129283() {                               // Cross-track error
    tN2kMsg m; SetN2kXTE(m, S.sid, N2kxtem_Autonomous, false, S.xte.value); emit(m);
}

static void pgn_129284() {                               // Navigation info (to the waypoint)
    uint16_t days; double secs; utc_now(days, secs);
    double eta_s = N2kDoubleNA;
    int16_t eta_d = N2kInt16NA;
    if (S.vmg_kn > 0.3f) {
        double t = secs + S.wpt_dist_m / MS(S.vmg_kn);
        eta_d = (int16_t)(days + (int)(t / 86400.0));
        eta_s = fmod(t, 86400.0);
    }
    tN2kMsg m;
    SetN2kNavigationInfo(m, S.sid, S.wpt_dist_m, N2khr_true, false, false, N2kdct_RhumbLine,
                         eta_s, eta_d, RAD(S.wpt_brg), RAD(S.wpt_brg), 1, 2,
                         scenario::WAYPOINT_LAT, scenario::WAYPOINT_LON, MS(S.vmg_kn));
    emit(m);
}

static void pgn_130306() {                               // Wind: apparent, true (boat), true (north)
    tN2kMsg m;
    SetN2kWindSpeed(m, S.sid, MS(S.aws_kn), RAD(S.awa), N2kWind_Apparent);   emit(m);
    SetN2kWindSpeed(m, S.sid, MS(S.tws_kn), RAD(S.twa), N2kWind_True_boat);  emit(m);
    SetN2kWindSpeed(m, S.sid, MS(S.tws_kn), RAD(S.twd), N2kWind_True_North); emit(m);
}

static void pgn_130310() {                               // Outside environment
    tN2kMsg m;
    SetN2kOutsideEnvironmentalParameters(m, S.sid, K(S.sea.value), K(S.air.value), PA(S.pressure.value));
    emit(m);
}

static void pgn_130311() {                               // Environmental parameters
    tN2kMsg m;
    SetN2kEnvironmentalParameters(m, S.sid, N2kts_OutsideTemperature, K(S.air.value),
                                  N2khs_OutsideHumidity, S.humidity.value, PA(S.pressure.value));
    emit(m);
}

static void pgn_130312() {                               // Temperature: exhaust, sea, cabin (with set point)
    tN2kMsg m;
    SetN2kTemperature(m, S.sid, 0, N2kts_ExhaustGasTemperature, K(S.eng[0].exhaust_c)); emit(m);
    SetN2kTemperature(m, S.sid, 1, N2kts_SeaTemperature, K(S.sea.value));              emit(m);
    SetN2kTemperature(m, S.sid, 2, N2kts_MainCabinTemperature, K(S.cabin.value), K(22.0)); emit(m);
}

static void pgn_130314() {                               // Pressure (atmospheric)
    tN2kMsg m; SetN2kPressure(m, S.sid, 0, N2kps_Atmospheric, PA(S.pressure.value)); emit(m);
}

static void pgn_130316() {                               // Temperature, extended range: one per source
    float air = S.air.value, rh = S.humidity.value;
    // Dew point (Magnus formula)
    float g = logf(rh / 100.0f) + 17.62f * air / (243.12f + air);
    float dew = 243.12f * g / (17.62f - g);
    // Wind chill (Environment Canada formula, wind in km/h)
    auto chill = [air](float kn) {
        float v = powf(fmaxf(kn * 1.852f, 5.0f), 0.16f);
        return 13.12f + 0.6215f * air - 11.37f * v + 0.3965f * air * v;
    };
    struct { tN2kTempSource src; float c; } t[] = {
        {N2kts_SeaTemperature,                  S.sea.value},
        {N2kts_OutsideTemperature,              air},
        {N2kts_InsideTemperature,               S.cabin.value + 0.6f},
        {N2kts_MainCabinTemperature,            S.cabin.value},
        {N2kts_RefridgerationTemperature,       S.fridge_c},
        {N2kts_FreezerTemperature,              -18.0f + 1.5f * sinf(2 * PI * S.t / 1500.0f)},
        {N2kts_HeatingSystemTemperature,        S.heating_c},
        {N2kts_DewPointTemperature,             dew},
        {N2kts_ApparentWindChillTemperature,    chill(S.aws_kn)},
        {N2kts_TheoreticalWindChillTemperature, chill(S.tws_kn)},
        {N2kts_ExhaustGasTemperature,           S.eng[0].exhaust_c},
    };
    for (unsigned i = 0; i < sizeof(t) / sizeof(t[0]); ++i) {
        tN2kMsg m; SetN2kTemperatureExt(m, S.sid, i, t[i].src, K(t[i].c)); emit(m);
    }
}

static void pgn_130576() {                               // Trim tabs: down at mid speeds to get on the plane
    float f = S.throttle_now;
    float base = 8.0f + 30.0f * expf(-powf((f - 0.45f) / 0.2f, 2));
    float d = S.tab_split.value;
    tN2kMsg m; SetN2kTrimTab(m, (int8_t)(base + d), (int8_t)(base - d)); emit(m);
}

static void pgn_130577() {                               // Direction data (incl. tidal set & drift)
    tN2kMsg m;
    SetN2kDirectionData(m, N2kDD025_Autonomous, N2khr_true, S.sid, RAD(S.cog), MS(S.sog_kn),
                        RAD(S.heading), MS(S.stw_kn), RAD(S.current_set.value), MS(S.current_drift.value));
    emit(m);
}

// AIS: positions and static data for each target on their own timers
static void ais_send(uint32_t now) {
    for (int i = 0; i < AIS_COUNT; ++i) {
        AisTarget& a = S.ais[i];
        const scenario::AisDef& d = *a.def;
        uint8_t secs = (uint8_t)((now / 1000) % 60);
        if ((int32_t)(now - a.next_pos_ms) >= 0) {
            tN2kMsg m;
            if (d.class_a) {
                SetN2kAISClassAPosition(m, 1, N2kaisr_Initial, d.mmsi, a.lat, a.lon, true, false, secs,
                                        RAD(a.cog), MS(a.sog), N2kaischannel_A_VDL_reception,
                                        RAD(a.heading), RAD(a.rot), d.status);
                a.next_pos_ms = now + (a.sog > 0.05f ? scenario::AIS_CLASS_A_MS : 3 * scenario::AIS_CLASS_A_MS);
            } else {
                SetN2kAISClassBPosition(m, 18, N2kaisr_Initial, d.mmsi, a.lat, a.lon, true, false, secs,
                                        RAD(a.cog), MS(a.sog), N2kaischannel_B_VDL_reception,
                                        RAD(a.heading), N2kaisunit_ClassB_CS, false, false, true, false,
                                        N2kaismode_Autonomous, false);
                a.next_pos_ms = now + scenario::AIS_CLASS_B_MS;
            }
            emit(m);
        }
        if ((int32_t)(now - a.next_static_ms) >= 0) {
            tN2kMsg m;
            if (d.class_a) {
                uint16_t days; double s; utc_now(days, s);
                SetN2kAISClassAStatic(m, 5, N2kaisr_Initial, d.mmsi, 9000000 + (d.mmsi % 1000000), d.callsign, d.name,
                                      d.type, d.length_m, d.beam_m, d.beam_m / 2, d.length_m * 0.8,
                                      days + 1, 36000, d.length_m > 40 ? 6.5 : 2.0, "SOUTHAMPTON",
                                      N2kaisv_ITU_R_M_1371_3, N2kGNSSt_GPS, N2kaisdte_Ready);
                emit(m);
            } else {
                SetN2kAISClassBStaticPartA(m, 24, N2kaisr_Initial, d.mmsi, d.name);
                emit(m);
                SetN2kAISClassBStaticPartB(m, 24, N2kaisr_Initial, d.mmsi, d.type, "DEMO", d.callsign,
                                           d.length_m, d.beam_m, d.beam_m / 2, d.length_m * 0.7, 0);
                emit(m);
            }
            a.next_static_ms = now + scenario::AIS_STATIC_MS;
        }
    }
}

// ═════════════════════════════════════════════════════════════════════════
// 5. NMEA 0183
// ═════════════════════════════════════════════════════════════════════════
static NmeaDemo0183Handler volatile g_0183 = nullptr;
static volatile uint32_t g_0183_msgs = 0;

// Formats the part between '$' and '*', adds the checksum and sends it.
static void out0183(const char* fmt, ...) {
    char body[96];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(body, sizeof(body), fmt, ap);
    va_end(ap);
    if (n <= 0) return;
    uint8_t ck = 0;
    for (const char* p = body; *p; ++p) ck ^= (uint8_t)*p;
    char line[104];
    int len = snprintf(line, sizeof(line), "$%s*%02X", body, ck);
    NmeaDemo0183Handler h = g_0183;
    if (h && len > 0) h(line, (size_t)len);
    g_0183_msgs = g_0183_msgs + 1;
}

struct Utc { int hh, mi, dd, mo, yyyy; double ss; };

static Utc utc_parts() {
    uint16_t days; double secs; utc_now(days, secs);
    time_t t = (time_t)days * 86400 + (time_t)secs;
    struct tm tm; gmtime_r(&t, &tm);
    Utc u;
    u.hh = tm.tm_hour; u.mi = tm.tm_min; u.ss = tm.tm_sec + fmod(secs, 1.0);
    u.dd = tm.tm_mday; u.mo = tm.tm_mon + 1; u.yyyy = tm.tm_year + 1900;
    return u;
}

static String hhmmss(const Utc& u) {
    char b[16]; snprintf(b, sizeof(b), "%02d%02d%05.2f", u.hh, u.mi, u.ss); return String(b);
}

// "ddmm.mmmm,N" and "dddmm.mmmm,W"
static String lat0183(double v) {
    char h = v >= 0 ? 'N' : 'S'; v = fabs(v);
    int d = (int)v; char b[20]; snprintf(b, sizeof(b), "%02d%07.4f,%c", d, (v - d) * 60.0, h); return String(b);
}
static String lon0183(double v) {
    char h = v >= 0 ? 'E' : 'W'; v = fabs(v);
    int d = (int)v; char b[20]; snprintf(b, sizeof(b), "%03d%07.4f,%c", d, (v - d) * 60.0, h); return String(b);
}

static float mag_heading()  { return norm360(S.heading - scenario::MAG_VARIATION); }
static float mag_cog()      { return norm360(S.cog - scenario::MAG_VARIATION); }
static char  ew(float v)    { return v < 0 ? 'W' : 'E'; }
static int   sats_used()    { int n = (int)S.gnss_sats.value; return n > SAT_COUNT ? SAT_COUNT : n; }

// Satellites in use: the highest ones in the sky
static int used_prns(uint8_t* out, int max) {
    int idx[SAT_COUNT];
    for (int i = 0; i < SAT_COUNT; ++i) idx[i] = i;
    for (int i = 1; i < SAT_COUNT; ++i)                  // sort by elevation, highest first
        for (int j = i; j > 0 && S.sats[idx[j]].elev > S.sats[idx[j - 1]].elev; --j) {
            int t = idx[j]; idx[j] = idx[j - 1]; idx[j - 1] = t;
        }
    int n = sats_used(); if (n > max) n = max;
    for (int i = 0; i < n; ++i) out[i] = S.sats[idx[i]].prn;
    return n;
}

static void nmea_gga() {                                 // Fix data
    Utc u = utc_parts();
    out0183("GPGGA,%s,%s,%s,1,%02d,%.1f,%.1f,M,%.1f,M,,", hhmmss(u).c_str(),
            lat0183(S.lat).c_str(), lon0183(S.lon).c_str(), sats_used(),
            S.gnss_hdop.value, S.gnss_alt.value, scenario::GEOID_SEPARATION_M);
}

static void nmea_rmc() {                                 // Recommended minimum
    Utc u = utc_parts();
    out0183("GPRMC,%s,A,%s,%s,%.1f,%.1f,%02d%02d%02d,%.1f,%c,A", hhmmss(u).c_str(),
            lat0183(S.lat).c_str(), lon0183(S.lon).c_str(), S.sog_kn, S.cog,
            u.dd, u.mo, u.yyyy % 100, fabsf(scenario::MAG_VARIATION), ew(scenario::MAG_VARIATION));
}

static void nmea_gll() {                                 // Geographic position
    Utc u = utc_parts();
    out0183("GPGLL,%s,%s,%s,A,A", lat0183(S.lat).c_str(), lon0183(S.lon).c_str(), hhmmss(u).c_str());
}

static void nmea_vtg() {                                 // Course and speed over ground
    out0183("GPVTG,%.1f,T,%.1f,M,%.1f,N,%.1f,K,A", S.cog, mag_cog(), S.sog_kn, S.sog_kn * 1.852f);
}

static void nmea_gsa() {                                 // Satellites in use, DOP
    uint8_t prn[12];
    int n = used_prns(prn, 12);
    String f;
    for (int i = 0; i < 12; ++i) {
        f += ',';
        if (i < n) { char b[4]; snprintf(b, sizeof(b), "%02u", prn[i]); f += b; }
    }
    float h = S.gnss_hdop.value, v = h * 1.3f, p = sqrtf(h * h + v * v);
    out0183("GPGSA,A,3%s,%.1f,%.1f,%.1f", f.c_str(), p, h, v);
}

static void nmea_gsv() {                                 // Satellites in view, 4 per sentence
    int total = (SAT_COUNT + 3) / 4;
    for (int m = 0; m < total; ++m) {
        String f;
        for (int k = m * 4; k < m * 4 + 4 && k < SAT_COUNT; ++k) {
            const auto& sat = S.sats[k];
            char b[24];
            snprintf(b, sizeof(b), ",%02u,%02d,%03d,%02d", sat.prn, (int)sat.elev, (int)sat.az, (int)sat.snr);
            f += b;
        }
        out0183("GPGSV,%d,%d,%02d%s", total, m + 1, SAT_COUNT, f.c_str());
    }
}

static void nmea_zda() {                                 // Date and time
    Utc u = utc_parts();
    int zh = scenario::ZDA_ZONE_HOURS;
    char zone[8];
    snprintf(zone, sizeof(zone), zh < 0 ? "-%02d" : "%02d", abs(zh));
    out0183("GPZDA,%s,%02d,%02d,%04d,%s,%02d", hhmmss(u).c_str(), u.dd, u.mo, u.yyyy, zone, scenario::ZDA_ZONE_MINUTES);
}

static void nmea_hdg() {                                 // Compass: sensor heading, deviation, variation
    float dev = scenario::COMPASS_DEVIATION, var = scenario::MAG_VARIATION;
    float sensor = norm360(mag_heading() - dev);
    out0183("HCHDG,%.1f,%.1f,%c,%.1f,%c", sensor, fabsf(dev), ew(dev), fabsf(var), ew(var));
}

static void nmea_hdm() { out0183("HCHDM,%.1f,M", mag_heading()); }   // Magnetic heading
static void nmea_hdt() { out0183("HEHDT,%.1f,T", S.heading); }       // True heading

static void nmea_dpt() {                                 // Depth below transducer, offset to keel
    out0183("SDDPT,%.1f,%.1f,100.0", S.depth_m, -scenario::TRANSDUCER_TO_KEEL_M);
}

static void nmea_dbt() {                                 // Depth below transducer in feet, metres, fathoms
    float d = S.depth_m;
    out0183("SDDBT,%.1f,f,%.1f,M,%.1f,F", d * 3.28084f, d, d * 0.546807f);
}

static void nmea_mwv() {                                 // Wind: apparent (R) and true (T), off the bow
    out0183("WIMWV,%.1f,R,%.1f,N,A", S.awa, S.aws_kn);
    out0183("WIMWV,%.1f,T,%.1f,N,A", S.twa, S.tws_kn);
}

static void nmea_vwr() {                                 // Apparent wind, 0-180 left or right
    bool right = S.awa <= 180.0f;
    float a = right ? S.awa : 360.0f - S.awa;
    out0183("WIVWR,%.1f,%c,%.1f,N,%.1f,M,%.1f,K", a, right ? 'R' : 'L',
            S.aws_kn, S.aws_kn * 0.514444f, S.aws_kn * 1.852f);
}

static void nmea_vhw() {                                 // Speed through water and heading
    out0183("VWVHW,%.1f,T,%.1f,M,%.1f,N,%.1f,K", S.heading, mag_heading(), S.stw_kn, S.stw_kn * 1.852f);
}

static void nmea_mtw() { out0183("YXMTW,%.1f,C", S.sea.value); }     // Water temperature

// ═════════════════════════════════════════════════════════════════════════
// 6. Schedules
// ═════════════════════════════════════════════════════════════════════════
struct Sched { const char* name; uint32_t period_ms; void (*fn)(); uint32_t next_ms; };

static Sched n2k_schedule[] = {
    // fast (5 Hz)
    {"127245", scenario::RATE_FAST_MS,   pgn_127245, 0},
    {"127250", scenario::RATE_FAST_MS,   pgn_127250, 0},
    {"127251", scenario::RATE_FAST_MS,   pgn_127251, 0},
    {"127257", scenario::RATE_FAST_MS,   pgn_127257, 0},
    {"127488", scenario::RATE_FAST_MS,   pgn_127488, 0},
    {"129025", scenario::RATE_FAST_MS,   pgn_129025, 0},
    {"129026", scenario::RATE_FAST_MS,   pgn_129026, 0},
    // wind and speed (2 Hz)
    {"130306", scenario::RATE_WIND_MS,   pgn_130306, 0},
    {"128259", scenario::RATE_WIND_MS,   pgn_128259, 0},
    // once a second
    {"126992", scenario::RATE_NORMAL_MS, pgn_126992, 0},
    {"127489", scenario::RATE_NORMAL_MS, pgn_127489, 0},
    {"127493", scenario::RATE_NORMAL_MS, pgn_127493, 0},
    {"127506", scenario::RATE_NORMAL_MS, pgn_127506, 0},
    {"127508", scenario::RATE_NORMAL_MS, pgn_127508, 0},
    {"127751", scenario::RATE_NORMAL_MS, pgn_127751, 0},
    {"128000", scenario::RATE_NORMAL_MS, pgn_128000, 0},
    {"128267", scenario::RATE_NORMAL_MS, pgn_128267, 0},
    {"128275", scenario::RATE_NORMAL_MS, pgn_128275, 0},
    {"129029", scenario::RATE_NORMAL_MS, pgn_129029, 0},
    {"129283", scenario::RATE_NORMAL_MS, pgn_129283, 0},
    {"129284", scenario::RATE_NORMAL_MS, pgn_129284, 0},
    {"130576", scenario::RATE_NORMAL_MS, pgn_130576, 0},
    {"130577", scenario::RATE_NORMAL_MS, pgn_130577, 0},
    // slow
    {"127258", scenario::RATE_SLOW_MS,   pgn_127258, 0},
    {"127497", scenario::RATE_SLOW_MS,   pgn_127497, 0},
    {"127505", scenario::RATE_SLOW_MS,   pgn_127505, 0},
    {"130310", scenario::RATE_SLOW_MS,   pgn_130310, 0},
    {"130311", scenario::RATE_SLOW_MS,   pgn_130311, 0},
    {"130312", scenario::RATE_SLOW_MS,   pgn_130312, 0},
    {"130314", scenario::RATE_SLOW_MS,   pgn_130314, 0},
    {"130316", scenario::RATE_SLOW_MS,   pgn_130316, 0},
};
static const int N2K_SCHED_COUNT = sizeof(n2k_schedule) / sizeof(n2k_schedule[0]);

// AIS PGNs sent by ais_send(): 129038, 129039, 129794, 129809, 129810
static const int AIS_PGN_COUNT = 5;

static Sched nmea0183_schedule[] = {
    {"GGA", scenario::RATE_0183_GPS_MS,     nmea_gga, 0},
    {"RMC", scenario::RATE_0183_GPS_MS,     nmea_rmc, 0},
    {"GLL", scenario::RATE_0183_GPS_MS,     nmea_gll, 0},
    {"VTG", scenario::RATE_0183_GPS_MS,     nmea_vtg, 0},
    {"ZDA", scenario::RATE_0183_GPS_MS,     nmea_zda, 0},
    {"GSA", scenario::RATE_0183_GSA_MS,     nmea_gsa, 0},
    {"GSV", scenario::RATE_0183_GSV_MS,     nmea_gsv, 0},
    {"HDG", scenario::RATE_0183_HEADING_MS, nmea_hdg, 0},
    {"HDM", scenario::RATE_0183_HEADING_MS, nmea_hdm, 0},
    {"HDT", scenario::RATE_0183_HEADING_MS, nmea_hdt, 0},
    {"DPT", scenario::RATE_0183_NORMAL_MS,  nmea_dpt, 0},
    {"DBT", scenario::RATE_0183_NORMAL_MS,  nmea_dbt, 0},
    {"MWV", scenario::RATE_0183_WIND_MS,    nmea_mwv, 0},
    {"VWR", scenario::RATE_0183_WIND_MS,    nmea_vwr, 0},
    {"VHW", scenario::RATE_0183_NORMAL_MS,  nmea_vhw, 0},
    {"MTW", scenario::RATE_0183_SLOW_MS,    nmea_mtw, 0},
};
static const int NMEA0183_SCHED_COUNT = sizeof(nmea0183_schedule) / sizeof(nmea0183_schedule[0]);

static void schedule_reset(Sched* sch, int count, uint32_t now) {
    for (int i = 0; i < count; ++i) sch[i].next_ms = now + (i * 37) % sch[i].period_ms;
}

static void schedule_run(Sched* sch, int count, uint32_t now) {
    for (int i = 0; i < count; ++i) {
        Sched& s = sch[i];
        if ((int32_t)(now - s.next_ms) >= 0) {
            s.fn();
            s.next_ms += s.period_ms;
            if ((int32_t)(now - s.next_ms) >= 0) s.next_ms = now + s.period_ms;   // fell behind
        }
    }
}

// ═════════════════════════════════════════════════════════════════════════
// 7. Task, start/stop and status
// ═════════════════════════════════════════════════════════════════════════
// One task runs the simulation while either output is on, so both outputs
// describe the same boat.
static TaskHandle_t demo_task_handle = nullptr;
static volatile bool demo_run = false;
static volatile bool n2k_restart = false, nmea0183_restart = false;

static void demo_task(void*) {
    const uint32_t STEP_MS = 100;
    sim_init();
    uint32_t last = millis();

    while (demo_run) {
        uint32_t now = millis();
        float dt = (now - last) / 1000.0f;
        last = now;
        if (dt > 0.0f) sim_step(fminf(dt, 1.0f));
        S.sid = (uint8_t)((S.sid + 1) % 253);

        if (g_n2k) {
            if (n2k_restart) { schedule_reset(n2k_schedule, N2K_SCHED_COUNT, now); n2k_restart = false; }
            schedule_run(n2k_schedule, N2K_SCHED_COUNT, now);
            ais_send(now);
        }
        if (g_0183) {
            if (nmea0183_restart) { schedule_reset(nmea0183_schedule, NMEA0183_SCHED_COUNT, now); nmea0183_restart = false; }
            schedule_run(nmea0183_schedule, NMEA0183_SCHED_COUNT, now);
        }

        uint32_t spent = millis() - now;
        vTaskDelay(pdMS_TO_TICKS(spent < STEP_MS ? STEP_MS - spent : 1));
    }

    demo_task_handle = nullptr;
    vTaskDelete(nullptr);
}

static void ensure_task() {
    if (demo_run && demo_task_handle) return;
    // A previous run may still be finishing its last step
    uint32_t t0 = millis();
    while (demo_task_handle && millis() - t0 < 500) vTaskDelay(pdMS_TO_TICKS(10));
    demo_run = true;
    if (xTaskCreatePinnedToCore(demo_task, "NMEA-Demo", 8192, nullptr, 3, &demo_task_handle, 0) != pdPASS) {
        demo_run = false;
        demo_task_handle = nullptr;
        Serial.println("[Demo] ERROR: could not start the demo task");
    }
}

static void stop_if_unused() {
    if (!g_n2k && !g_0183) demo_run = false;
}

void nmea_demo_start_n2k(NmeaDemoN2kHandler handler) {
    g_n2k_msgs = 0;
    n2k_restart = true;
    g_n2k = handler;
    ensure_task();
    Serial.printf("[Demo] NMEA 2000 demonstration data started (%d PGNs)\n", nmea_demo_pgn_count());
}

void nmea_demo_stop_n2k() {
    g_n2k = nullptr;
    stop_if_unused();
}

void nmea_demo_start_0183(NmeaDemo0183Handler handler) {
    g_0183_msgs = 0;
    nmea0183_restart = true;
    g_0183 = handler;
    ensure_task();
    Serial.printf("[Demo] NMEA 0183 demonstration data started (%d sentence types)\n", nmea_demo_sentence_count());
}

void nmea_demo_stop_0183() {
    g_0183 = nullptr;
    stop_if_unused();
}

bool nmea_demo_n2k_running()  { return g_n2k && demo_run && demo_task_handle; }
bool nmea_demo_0183_running() { return g_0183 && demo_run && demo_task_handle; }

int nmea_demo_pgn_count()      { return N2K_SCHED_COUNT + AIS_PGN_COUNT; }
int nmea_demo_sentence_count() { return NMEA0183_SCHED_COUNT; }
uint32_t nmea_demo_n2k_msg_count() { return g_n2k_msgs; }

String nmea_demo_n2k_status() {
    if (!nmea_demo_n2k_running()) return "Demonstration data not running";
    return "Demonstration data: " + String(nmea_demo_pgn_count()) + " PGNs, " +
           String(g_n2k_msgs) + " messages generated";
}
