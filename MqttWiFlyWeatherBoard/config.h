#ifndef MQTTWIFLYWEATHERBOARD_CONFIG_H_
#define MQTTWIFLYWEATHERBOARD_CONFIG_H_

//#include "debug.h"

// define buffers
#if ENABLE_JSON
#include <ArduinoJson.h>
const byte JSON_BUFFER_SIZE = 200;
//StaticJsonBuffer<JSON_BUFFER_SIZE> jsonBuffer;
#endif
const byte TOPIC_BUFFER_SIZE = 42;
// char topicBuffer[TOPIC_BUFFER_SIZE];
const byte PAYLOAD_BUFFER_SIZE = JSON_BUFFER_SIZE;
// char payloadBuffer[PAYLOAD_BUFFER_SIZE];

const byte FLOAT_DECIMAL_PLACES = 1;

// Serial parameters
const int BAUD_RATE = 9600;

// program constants
const unsigned long MEASUREMENT_INTERVAL_SECS = 30UL;
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

#ifndef COMMUNICATION_METHOD
  #define ENABLE_MQTT 0
#elif COMMUNICATION_METHOD == COMM_SERIAL
  #define ENABLE_MQTT 0
#elif COMMUNICATION_METHOD == COMM_WIFLY
  #include "wiFlyConfig.h"
  #include "mqttConfig.h"
  #define ENABLE_MQTT 1
#else
  #error "Unexpected value of COMMUNICATION_METHOD"
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

#include "progmemStrings.h"

// status topics
const char WIFLY_STATUS[] PROGMEM = "weather/status/wifly";
const char BATTERY_STATUS[] PROGMEM = "weather/status/battery";
const char MEMORY_STATUS[] PROGMEM = "weather/status/memory";
const char REPORT_STATUS[] PROGMEM = "weather/status/report";

PGM_P const STATUS_TOPICS[] PROGMEM = {
    WIFLY_STATUS,          // idx = 0
    BATTERY_STATUS,        // idx = 1
    MEMORY_STATUS,         // idx = 2
    REPORT_STATUS,         // idx = 3
};

#endif /* MQTTWIFLYWEATHERBOARD_CONFIG_H_ */
