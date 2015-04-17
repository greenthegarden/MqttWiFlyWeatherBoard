#ifndef __CONFIG_H__
#define __CONFIG_H__


//#define DEBUG                   false    // turn on DEBUG in libraries

#define ENABLE_TEMP             true
#define ENABLE_HUMIDITY         true
#define ENABLE_PRESSURE         true
#define ENABLE_LIGHT            true
#define ENABLE_WEATHER_METERS   false

// Watchdog timer
#define ENABLE_WDT              false
//#define
// Serial parameters
#define BAUD_RATE               9600


// Wifi parameters
const char ssid[]             = "ssid";
const char passphrase[]       = "password";
boolean    mode               = true; //or WEP_MODE


// MQTT parameters
//byte mqtt_server_addr[]     = { 192, 168, 1, 30 };    // Airology
byte mqtt_server_addr[]       = { 192, 168, 1, 55 };    // Pi
int mqtt_port                 = 1883;
char mqtt_client_id[]         = "weather";
#define MQTT_MAX_PACKET_SIZE    168
#define MQTT_KEEPALIVE          300
boolean wifly_connected       = false;

//topics
char wifly_topic[]            = "weather/status/wifly";
char sensor_topic[]           = "weather/status/sensor";
char battery_topic[]          = "weather/status/battery";
char memory_topic[]           = "weather/status/memory";

char SHT15_temp_topic[]       = "weather/measurement/SHT15_temp";
char SHT15_humidity_topic[]   = "weather/measurement/SHT15_humidity";
char BMP085_temp_topic[]      = "weather/measurement/BMP085_temp";
char BMP085_pressure_topic[]  = "weather/measurement/BMP085_pressure";
char TEMT6000_light_raw_topic[] = "weather/measurement/TEMT6000_light_raw";
char TEMT6000_light_topic[]   = "weather/measurement/TEMT6000_light";
char wind_dir_topic[]         = "weather/measurement/wind_dir";
char wind_spd_topic[]         = "weather/measurement/wind_spd";
char rainfall_topic[]         = "weather/measurement/rain";


#define FLOAT_DECIMAL_PLACES    1
#define MEASUREMENT_INTERVAL    300000    // 5 minutes = 5 * 60 * 1000 miliiseconds
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


// digital I/O pins
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

