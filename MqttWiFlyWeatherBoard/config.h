#ifndef MQTTWIFLYWEATHERBOARD_CONFIG_H_
#define MQTTWIFLYWEATHERBOARD_CONFIG_H_

#include "debug.h"

const byte BUFFER_SIZE = 42;
char progBuffer[BUFFER_SIZE];
char messBuffer[BUFFER_SIZE];

const byte FLOAT_DECIMAL_PLACES = 1;

// character buffer to support conversion of floats to char
char buf[12];

// Serial parameters
const int BAUD_RATE = 9600;

// program constants
const unsigned long MEASUREMENT_INTERVAL_SECS = 15UL * 60UL;
const unsigned long MEASUREMENT_INTERVAL =
    MEASUREMENT_INTERVAL_SECS * 1000UL; // conversion to milliseconds

// global variable definitions
unsigned long previousMeasurementMillis = 0UL;
unsigned long previousWindMeasurementMillis = 0UL;

#include "wiFlyConfig.h"

#ifndef ENABLE_JSON_OUTPUT
#define ENABLE_JSON_OUTPUT false
#endif

#include "mqttConfig.h"

// Define use of weather board sensors
#ifndef ENABLE_WEATHER_METERS
#define ENABLE_WEATHER_METERS true
#endif
#ifndef ENABLE_WIND_MEASUREMENT_AVERAGING
#define ENABLE_WIND_MEASUREMENT_AVERAGING true
#endif
#ifndef ENABLE_EXTERNAL_LIGHT
#define ENABLE_EXTERNAL_LIGHT true
#endif

#include "weatherBoardConfig.h"

// status LED to show when WiFLy is connecting (consider turning off to save
// power)
#ifndef USE_STATUS_LED
#define USE_STATUS_LED false
#endif

// Define user of external sensors
#ifndef ENABLE_POWER_MONITOR
#define ENABLE_POWER_MONITOR true // for use with SwitchDoc Lab SunAirPlus
#endif
#ifndef ENABLE_DHT22
#define ENABLE_DHT22 false
#endif

#if ENABLE_POWER_MONITOR
#include "sunAirPlusConfig.h"
#endif

#if ENABLE_DHT22
#include "dht22Config.h"
#endif

#endif /* MQTTWIFLYWEATHERBOARD_CONFIG_H_ */
