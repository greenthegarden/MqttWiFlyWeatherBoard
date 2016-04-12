#ifndef __CONFIG_H__
#define __CONFIG_H__


//#define DEBUG                   false    // turn on DEBUG in libraries

// Define use of sensors
#define ENABLE_SHT15            true
#define ENABLE_DHT22            true
#define ENABLE_BMP085           true
#define ENABLE_EXTERNAL_LIGHT   true
#define ENABLE_WEATHER_METERS   true
#define ENABLE_WIND_DIR_AVERAGING true

// Define measurements to take
#define ENABLE_TEMP             true
#define ENABLE_HUMIDITY         true
#define ENABLE_PRESSURE         true
#define ENABLE_LIGHT            true

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
IPAddress mqttServerAddr(192, 168, 1, 55);    // Pi
const int MQTT_PORT           = 1883;
char mqttClientId[]         = "weather";
#define MQTT_MAX_PACKET_SIZE    168
#define MQTT_KEEPALIVE          300
boolean wiflyConnected       = false;
byte measurementCount        = 0;


const byte BUFFER_SIZE        = 42;
char progBuffer[BUFFER_SIZE];
char messBuffer[BUFFER_SIZE];


// MQTT topic definitions

// status topics

const char WIFLY_STATUS[]             PROGMEM = "weather/status/wifly";
const char SENSOR_STATUS[]            PROGMEM = "weather/status/sensor";
const char BATTERY_STATUS[]           PROGMEM = "weather/status/battery";
const char MEMORY_STATUS[]            PROGMEM = "weather/status/memory";
const char SHT15_STATUS[]             PROGMEM = "weather/status/sht15";
const char DHT22_STATUS[]             PROGMEM = "weather/status/dht22";
const char BMP085_STATUS[]            PROGMEM = "weather/status/bmp085";

PGM_P const STATUS_TOPICS[]           PROGMEM = { WIFLY_STATUS,     // idx = 0
                                                  SENSOR_STATUS,    // idx = 1
                                                  BATTERY_STATUS,   // idx = 2
                                                  MEMORY_STATUS,    // idx = 3
                                                  SHT15_STATUS,     // idx = 4
                                                  DHT22_STATUS,     // idx = 5
                                                  BMP085_STATUS,    // idx = 6
                                                };

// MQTT payloads
const char MQTT_PAYLOAD_CONNECTED[]          PROGMEM = "Connected";
const char MQTT_PAYLOAD_WM_ERROR[]           PROGMEM = "Weather Meter ERROR";

PGM_P const MQTT_PAYLOADS[]    PROGMEM = { MQTT_PAYLOAD_CONNECTED,   // idx = 0
                                           MQTT_PAYLOAD_WM_ERROR,    // idx = 1
                                               };

// BMP085 status messages

const char BMP085_INIT_SUCCESS[]          PROGMEM = "BMP085: Init success";
const char BMP085_INIT_FAILURE[]          PROGMEM = "BMP085: init failure";
const char BMP085_ERROR_PRESSURE_START[]  PROGMEM = "BMP085: Pressure Start error";
const char BMP085_ERROR_PRESSURE_GET[]    PROGMEM = "BMP085: Pressure Get error";
const char BMP085_ERROR_TEMP_START[]      PROGMEM = "BMP085: Temperature Start error";
const char BMP085_ERROR_TEMP_GET[]        PROGMEM = "BMP085: Temperature Get error";

PGM_P const BMP085_STATUS_MESSAGES[]      PROGMEM = { BMP085_INIT_SUCCESS,          // idx = 0
                                                      BMP085_INIT_FAILURE,          // idx = 1
                                                      BMP085_ERROR_PRESSURE_START,  // idx = 2
                                                      BMP085_ERROR_PRESSURE_GET,    // idx = 3
                                                      BMP085_ERROR_TEMP_START,      // idx = 4
                                                      BMP085_ERROR_TEMP_GET,        // idx = 5
                                                    };

                                                
// sunairplus measurement topics 

#if ENABLE_POWER_MONITOR
const char BATTERY_VOLTAGE_TOPIC[]    PROGMEM = "weather/sunairplus/battery_voltage";
const char BATTERY_CURRENT_TOPIC[]    PROGMEM = "weather/sunairplus/battery_current";
const char SOLAR_VOLTAGE_TOPIC[]      PROGMEM = "weather/sunairplus/solar_voltage";
const char SOLAR_CURRENT_TOPIC[]      PROGMEM = "weather/sunairplus/solar_current";
const char OUTPUT_VOLTAGE_TOPIC[]     PROGMEM = "weather/sunairplus/output_voltage";
const char OUTPUT_CURRENT_TOPIC[]     PROGMEM = "weather/sunairplus/output_current";

//tables to refer to strings
PGM_P const SUNAIRPLUS_TOPICS[]       PROGMEM = { BATTERY_VOLTAGE_TOPIC,     // idx = 0
                                                  BATTERY_CURRENT_TOPIC,     // idx = 1
                                                  SOLAR_VOLTAGE_TOPIC,       // idx = 2
                                                  SOLAR_CURRENT_TOPIC,       // idx = 3
                                                  OUTPUT_VOLTAGE_TOPIC,      // idx = 4
                                                  OUTPUT_CURRENT_TOPIC,      // idx = 5
                                                };
#endif

// measurement topics

const char SHT15_TEMP_TOPIC[]         PROGMEM = "weather/measurement/SHT15_temp";
const char SHT15_HUMIDITY_TOPIC[]     PROGMEM = "weather/measurement/SHT15_humidity";
const char BMP085_TEMP_TOPIC[]        PROGMEM = "weather/measurement/BMP085_temp";
const char BMP085_PRESSURE_TOPIC[]    PROGMEM = "weather/measurement/BMP085_pressure";
const char TEMT6000_LIGHT_RAW_TOPIC[] PROGMEM = "weather/measurement/TEMT6000_light_raw";
const char TEMT6000_LIGHT_TOPIC[]     PROGMEM = "weather/measurement/TEMT6000_light";
const char WIND_SPEED_TOPIC[]         PROGMEM = "weather/measurement/wind_spd";
const char WIND_SPEED_MAX_TOPIC[]     PROGMEM = "weather/measurement/wind_spd_max";
const char WIND_DIRECTION_TOPIC[]     PROGMEM = "weather/measurement/wind_dir";
const char RAINFALL_TOPIC[]           PROGMEM = "weather/measurement/rain";

const char DHT22_TEMP_TOPIC[]         PROGMEM = "weather/measurement/DHT22_temp";
const char DHT22_HUMIDITY_TOPIC[]     PROGMEM = "weather/measurement/DHT22_humidity";

const char MEASUREMENTS_START[]       PROGMEM = "weather/measurement/START";
const char MEASUREMENTS_END[]         PROGMEM = "weather/measurement/END";

//tables to refer to strings
PGM_P const MEASUREMENT_TOPICS[]      PROGMEM = { SHT15_TEMP_TOPIC,          // idx = 0
                                                  SHT15_HUMIDITY_TOPIC,          // idx = 1
                                                  BMP085_TEMP_TOPIC,      // idx = 2
                                                  BMP085_PRESSURE_TOPIC,      // idx = 3
                                                  TEMT6000_LIGHT_RAW_TOPIC,         // idx = 4
                                                  TEMT6000_LIGHT_TOPIC,     // idx = 5
                                                  WIND_SPEED_TOPIC,  // idx = 6
                                                  WIND_SPEED_MAX_TOPIC,      // idx = 7
                                                  WIND_DIRECTION_TOPIC,            // idx = 8
                                                  RAINFALL_TOPIC,            // idx = 9
                                                  DHT22_TEMP_TOPIC,        // idx = 10
                                                  DHT22_HUMIDITY_TOPIC,            // idx = 11
                                                  MEASUREMENTS_START,        // idx = 12
                                                  MEASUREMENTS_END,          // idx = 13
                                                };


// program constants

#define FLOAT_DECIMAL_PLACES    1
#define MEASUREMENT_INTERVAL    300000    // 5 minutes = 5 * 60 * 1000 miliiseconds
//#define MEASUREMENT_INTERVAL    120000    // 2 minutes = 2 * 60 * 1000 miliiseconds
#define AFTER_ERROR_DELAY       60000


#if ENABLE_WIND_DIR_AVERAGING
// Wind direction averaging
byte WIND_DIR_AVERAGING_SIZE    = 10;
unsigned long WIND_DIR_INTERVAL = 1000;
#endif


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


#if ENABLE_DHT22
byte dht22MeasurementOk = false;
#endif


// Weather Board Digital I/O pin definitions

#define   RAIN         2
#define   WSPEED       3
#define   STATUS_LED   4
#define   RF_CTS       5
#define   RF_RTS       6
#define   EOC          8
#define   XCLR         9
#define   DHT22_PIN    11
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

