#ifndef MQTTWIFLYWEATHERBOARD_CONFIG_H_
#define MQTTWIFLYWEATHERBOARD_CONFIG_H_

//#include "debug.h"

const byte BUFFER_SIZE = 42;
char topicBuffer[BUFFER_SIZE];
char payloadBuffer[BUFFER_SIZE];

const byte FLOAT_DECIMAL_PLACES = 1;

// Serial parameters
const int BAUD_RATE = 9600;

// program constants
const unsigned long MEASUREMENT_INTERVAL_SECS = 10UL;
const unsigned long MEASUREMENT_INTERVAL =
    MEASUREMENT_INTERVAL_SECS * 1000UL; // conversion to milliseconds

// global variable definitions
unsigned long previousMeasurementMillis = 0UL;
unsigned long previousWindMeasurementMillis = 0UL;

typedef enum {
  COMM_SERIAL = 0,
  COMM_WIFLY = 1,
} communication_methods;

#ifndef COMMUNICATION_METHOD
#define COMMUNICATION_METHOD COMM_SERIAL
#endif

#if COMMUNICATION_METHOD==COMM_WIFLY
//  #include "wiFlyConfig.h"
//  #include "mqttConfig.h"
  #define ENABLE_MQTT true
#else
  #define ENABLE_MQTT false
#endif

void printTopicPayloadPair(const char* topic, const char* payload) {
  Serial.print(topic);
  Serial.print(": ");
  Serial.print(payload);
  Serial.println();
}

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
//#include "sunAirPlusConfig.h"
#endif

#if ENABLE_DHT22
//#include "dht22Config.h"
#endif

// PROGMEM_STRINGS
const char PROGMEM_STRING_CONNECTED[] PROGMEM = "CONNECTED";
const char PROGMEM_STRING_OK[] PROGMEM = "OK";
const char PROGMEM_STRING_ERROR[] PROGMEM = "ERROR";
const char PROGMEM_STRING_START[] PROGMEM = "START";
const char PROGMEM_STRING_END[] PROGMEM = "END";
const char PROGMEM_STRING_SLEEP[] PROGMEM = "SLEEP";

PGM_P const PROGMEM_STRINGS[] PROGMEM = {
    PROGMEM_STRING_CONNECTED, // idx = 0
    PROGMEM_STRING_OK,        // idx = 1
    PROGMEM_STRING_ERROR,     // idx = 2
    PROGMEM_STRING_START,     // idx = 3
    PROGMEM_STRING_END,       // idx = 4
    PROGMEM_STRING_SLEEP,     // idx = 5
};

/* PROGMEM_STRINGS indices, must match table above */
typedef enum {
  PROGMEM_STRING_CONNECTED_IDX = 0,
  PROGMEM_STRING_OK_IDX = 1,
  PROGMEM_STRING_ERROR_IDX = 2,
  PROGMEM_STRING_START_IDX = 3,
  PROGMEM_STRING_END_IDX = 4,
  PROGMEM_STRING_SLEEP_IDX = 5,
} progmem_strings;

#endif /* MQTTWIFLYWEATHERBOARD_CONFIG_H_ */
