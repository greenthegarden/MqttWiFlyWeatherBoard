#ifndef __CONFIG_H__
#define __CONFIG_H__


//#define DEBUG                   false    // turn on DEBUG in libraries

#define ENABLE_TEMP             true
#define ENABLE_HUMIDITY         true
#define ENABLE_PRESSURE         true
#define ENABLE_LIGHT            true
#define ENABLE_WEATHER_METERS   true

#define ENABLE_POWER_MONITOR    true    // for use with SwitchDoc Lab SunAirPlus

#if ENABLE_POWER_MONITOR
// the three channels of the INA3221 named for SunAirPlus Solar Power Controller channels (www.switchdoc.com)
#define LIPO_BATTERY_CHANNEL    1
#define SOLAR_CELL_CHANNEL      2
#define OUTPUT_CHANNEL          3
#endif


// Watchdog timer
#define ENABLE_WDT              false


// status LED to show when WiFLy is connecting (consider turning off to save power)
#define USE_STATUS_LED          true


// Serial parameters
#define BAUD_RATE               9600


// Network configuration
#include "networkConfig.h"


// MQTT parameters
//byte mqtt_server_addr[]     = { 192, 168, 1, 30 };    // Airology
byte mqtt_server_addr[]       = { 192, 168, 1, 55 };    // Pi
int mqtt_port                 = 1883;
char mqtt_client_id[]         = "weather";
#define MQTT_MAX_PACKET_SIZE    168
#define MQTT_KEEPALIVE          300
boolean wifly_connected       = false;


const byte BUFFER_SIZE        = 42;
char prog_buffer[BUFFER_SIZE];


// MQTT topic definitions

// status topics

const char wifly_topic[]              PROGMEM = "weather/status/wifly";
const char sensor_topic[]             PROGMEM = "weather/status/sensor";
const char battery_topic[]            PROGMEM = "weather/status/battery";
const char memory_topic[]             PROGMEM = "weather/status/memory";

PGM_P const status_topics[]           PROGMEM = { wifly_topic,      // idx = 0
                                                  sensor_topic,     // idx = 1
                                                  battery_topic,    // idx = 2
                                                  memory_topic,     // idx = 3
                                                };

// sunairplus measurement topics 

#if ENABLE_POWER_MONITOR
const char battery_voltage_topic[]    PROGMEM = "weather/sunairplus/battery_voltage";
const char battery_current_topic[]    PROGMEM = "weather/sunairplus/battery_current";
const char solar_voltage_topic[]      PROGMEM = "weather/sunairplus/solar_voltage";
const char solar_current_topic[]      PROGMEM = "weather/sunairplus/solar_current";
const char output_voltage_topic[]     PROGMEM = "weather/sunairplus/output_voltage";
const char output_current_topic[]     PROGMEM = "weather/sunairplus/output_current";

//tables to refer to strings
PGM_P const sunairplus_topics[]       PROGMEM = { battery_voltage_topic,     // idx = 0
                                                  battery_current_topic,     // idx = 1
                                                  solar_voltage_topic,       // idx = 2
                                                  solar_current_topic,       // idx = 3
                                                  output_voltage_topic,      // idx = 4
                                                  output_current_topic,      // idx = 5
                                                };
#endif

// measurement topics

//#if ENABLE_TEMP
const char SHT15_temp_topic[]         PROGMEM = "weather/measurement/SHT15_temp";
//#endif
//#if ENABLE_HUMIDITY
const char SHT15_humidity_topic[]     PROGMEM = "weather/measurement/SHT15_humidity";
//#endif
//#if ENABLE_PRESSURE
const char BMP085_temp_topic[]        PROGMEM = "weather/measurement/BMP085_temp";
const char BMP085_pressure_topic[]    PROGMEM = "weather/measurement/BMP085_pressure";
//#endif
//#if ENABLE_LIGHT
const char TEMT6000_light_raw_topic[] PROGMEM = "weather/measurement/TEMT6000_light_raw";
const char TEMT6000_light_topic[]     PROGMEM = "weather/measurement/TEMT6000_light";
//#endif
//#if ENABLE_WEATHER_METERS
const char wind_spd_topic[]           PROGMEM = "weather/measurement/wind_spd";
const char wind_spd_max_topic[]       PROGMEM = "weather/measurement/wind_spd_max";
const char wind_dir_topic[]           PROGMEM = "weather/measurement/wind_dir";
const char rainfall_topic[]           PROGMEM = "weather/measurement/rain";
//#endif

//tables to refer to strings
PGM_P const measurment_topics[]       PROGMEM = { SHT15_temp_topic,          // idx = 0
                                                  SHT15_humidity_topic,      // idx = 1
                                                  BMP085_temp_topic,         // idx = 2
                                                  BMP085_pressure_topic,     // idx = 3
                                                  TEMT6000_light_raw_topic,  // idx = 4
                                                  TEMT6000_light_topic,      // idx = 5
                                                  wind_spd_topic,            // idx = 6
                                                  wind_dir_topic,            // idx = 7
                                                  rainfall_topic,            // idx = 8
                                                  wind_spd_max_topic,        // idx = 9
                                                };


// program constants

#define FLOAT_DECIMAL_PLACES    1
//#define MEASUREMENT_INTERVAL    300000    // 5 minutes = 5 * 60 * 1000 miliiseconds
#define MEASUREMENT_INTERVAL    120000    // 2 minutes = 2 * 60 * 1000 miliiseconds
#define AFTER_ERROR_DELAY       5000


// constant conversion factors

#if ENABLE_WEATHER_METERS
//const float WIND_RPM_TO_MPH  = 22.686745;         // divide RPM by this for velocity
const float WIND_RPM_TO_MPS    = 50.748803;         // divide RPM by this for meters per second
// 1 m/s = 1.943844492 knots
const float WIND_RPM_TO_KNOTS  = WIND_RPM_TO_MPS / 1.943844492;
//const float RAIN_BUCKETS_TO_INCHES = 0.014815;    // multiply bucket tips by this for inches
const float RAIN_BUCKETS_TO_MM = 0.376296;          // multiply bucket tips by this for mm
const unsigned int ZERODELAY   = 4000;              // ms, zero RPM if no result for this time period (see irq below)
#endif


// Weather Board Digital I/O pin definitions

#define   RAIN         2
#define   WSPEED       3
#define   STATUS_LED   4
#define   RF_CTS       5
#define   RF_RTS       6
#define   EOC          8
#define   XCLR         9
// (the following three are predefined)
// #define MOSI        11
// #define MISO        12
// #define SCK         13

// analog I/O pins
#define   WDIR         A0
#define   SHT1x_DATA   A4
#define   SHT1x_CLOCK  A5
#define   BATT_LVL     A6
#define   LIGHT        A7


#endif

