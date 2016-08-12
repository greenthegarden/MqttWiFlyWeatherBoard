#ifndef MQTTWIFLYWEATHERBOARD_CONFIG_H_
#define MQTTWIFLYWEATHERBOARD_CONFIG_H_


#include "debug.h"

const byte BUFFER_SIZE                        = 42;
char progBuffer[BUFFER_SIZE];
char messBuffer[BUFFER_SIZE];

const byte FLOAT_DECIMAL_PLACES               = 1;

// character buffer to support conversion of floats to char
char buf[12];

// Serial parameters
const int BAUD_RATE                           = 9600;

// program constants
const unsigned long MEASUREMENT_INTERVAL_SECS = 15UL * 60UL;
const unsigned long MEASUREMENT_INTERVAL      = MEASUREMENT_INTERVAL_SECS * 1000UL;   // 5 minutes = 5 * 60 * 1000 milliseconds

// global variable definitions
unsigned long previousMeasurementMillis       = 0;
unsigned long previousWindMeasurementMillis   = 0;

#include "wiFlyConfig.h"

#include "mqttConfig.h"

// Define use of weather board sensors
#define ENABLE_WEATHER_METERS     true
#define ENABLE_WIND_DIR_AVERAGING true
#define ENABLE_EXTERNAL_LIGHT     true

#include "weatherBoardConfig.h"

// status LED to show when WiFLy is connecting (consider turning off to save power)
#define USE_STATUS_LED            false

// Define user of external sensors
#define ENABLE_POWER_MONITOR      true    // for use with SwitchDoc Lab SunAirPlus
#define ENABLE_DHT22              false

#if ENABLE_POWER_MONITOR
#include "sunAirPlusConfig.h"
#endif

#if ENABLE_DHT22
#include "dht22Config.h"
#endif


#endif  /* MQTTWIFLYWEATHERBOARD_CONFIG_H_ */

