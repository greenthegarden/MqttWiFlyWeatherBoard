#ifndef MQTTWIFLYWEATHERBOARD_CONFIG_H_
#define MQTTWIFLYWEATHERBOARD_CONFIG_H_


//#define DEBUG                   false    // turn on DEBUG in libraries

#include "debug.h"


const byte BUFFER_SIZE        = 42;
char progBuffer[BUFFER_SIZE];
char messBuffer[BUFFER_SIZE];

#define FLOAT_DECIMAL_PLACES    1

// character buffer to support conversion of floats to char
char buf[12];

// Serial parameters
const int BAUD_RATE           = 9600;


// program constants

const unsigned long MEASUREMENT_INTERVAL  =  5UL * 60UL * 1000UL;   // 5 minutes = 5 * 60 * 1000 miliiseconds
//#define MEASUREMENT_INTERVAL    120000    // 2 minutes = 2 * 60 * 1000 miliiseconds
const unsigned long AFTER_ERROR_DELAY      = 60UL * 1000UL;


#include "wiFlyConfig.h"
#include "mqttConfig.h"


// Define use of weather board sensors
#define ENABLE_EXTERNAL_LIGHT   true
#define ENABLE_WEATHER_METERS   true
#define ENABLE_WIND_DIR_AVERAGING true

#include "weatherBoardConfig.h"

// Define user of external sensors
#define ENABLE_DHT22            false
#define ENABLE_POWER_MONITOR    true    // for use with SwitchDoc Lab SunAirPlus

#if ENABLE_POWER_MONITOR
#include "sunAirPlusConfig.h"
#endif


// status LED to show when WiFLy is connecting (consider turning off to save power)
#define USE_STATUS_LED          true


#if ENABLE_DHT22
#include "dht22Config.h"
byte dht22MeasurementOk = false;
#endif










#endif  /* MQTTWIFLYWEATHERBOARD_CONFIG_H_ */

