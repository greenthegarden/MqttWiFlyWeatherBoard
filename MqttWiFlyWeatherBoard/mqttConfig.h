#ifndef MQTTWIFLYWEATHERBOARD_MQTTCONFIG_H_
#define MQTTWIFLYWEATHERBOARD_MQTTCONFIG_H_

#include <PubSubClient.h>

// MQTT parameters
IPAddress mqttServerAddr(192, 168, 1, 50); // openHAB
char mqttClientId[] = "weather";
const int MQTT_PORT = 1883;

// callback definition for MQTT
void callback(char *topic, uint8_t *payload, unsigned int length) {
  // nothing to do here!!
}

PubSubClient mqttClient(mqttServerAddr, MQTT_PORT, callback, networkClient);

// MQTT topic definitions

// MQTT payloads
const char MQTT_PAYLOAD_CONNECTED[] PROGMEM = "CONNECTED";
const char MQTT_PAYLOAD_ERROR[] PROGMEM = "ERROR";
const char MQTT_PAYLOAD_START[] PROGMEM = "START";
const char MQTT_PAYLOAD_END[] PROGMEM = "END";
const char MQTT_PAYLOAD_SLEEP[] PROGMEM = "SLEEP";
const char MQTT_PAYLOAD_OK[] PROGMEM = "OK";

PGM_P const MQTT_PAYLOADS[] PROGMEM = {
    MQTT_PAYLOAD_CONNECTED, // idx = 0
    MQTT_PAYLOAD_ERROR,     // idx = 1
    MQTT_PAYLOAD_START,     // idx = 2
    MQTT_PAYLOAD_END,       // idx = 3
    MQTT_PAYLOAD_SLEEP,     // idx = 4
    MQTT_PAYLOAD_OK,        // idx = 5
};

/* MQTT_PAYLOADS indices, must match table above */
typedef enum {
  MQTT_PAYLOAD_CONNECTED_IDX = 0,
  MQTT_PAYLOAD_ERROR_IDX = 1,
  MQTT_PAYLOAD_START_IDX = 2,
  MQTT_PAYLOAD_END_IDX = 3,
  MQTT_PAYLOAD_SLEEP_IDX = 4,
  MQTT_PAYLOAD_OK_IDX = 5,
} mqtt_payloads;

// status topics
const char WIFLY_STATUS[] PROGMEM = "weather/status/wifly";
const char BATTERY_STATUS[] PROGMEM = "weather/status/battery";
const char MEMORY_STATUS[] PROGMEM = "weather/status/memory";
const char REPORT_STATUS[] PROGMEM = "weather/status/report";
const char BMP085_STATUS[] PROGMEM = "weather/status/bmp085";
const char WEATHER_METERS_STATUS[] PROGMEM = "weather/status/wm";
const char DHT22_STATUS[] PROGMEM = "weather/status/dht22";

PGM_P const STATUS_TOPICS[] PROGMEM = {
    WIFLY_STATUS,          // idx = 0
    BATTERY_STATUS,        // idx = 1
    MEMORY_STATUS,         // idx = 2
    REPORT_STATUS,         // idx = 3
    BMP085_STATUS,         // idx = 4
    WEATHER_METERS_STATUS, // idx = 5
    DHT22_STATUS,          // idx = 6
};

// STATUS_TOPICS indices, must match table above
typedef enum {
  WIFLY_STATUS_IDX = 0,
  BATTERY_STATUS_IDX = 1,
  MEMORY_STATUS_IDX = 2,
  REPORT_STATUS_IDX = 3,
  BMP085_STATUS_IDX = 4,
  WEATHER_METERS_STATUS_IDX = 5,
  DHT22_STATUS_IDX = 6,
} status_topics;

// measurement topics
const char SHT15_TEMP_TOPIC[] PROGMEM = "weather/measurement/SHT15_temp";
const char SHT15_HUMIDITY_TOPIC[] PROGMEM =
    "weather/measurement/SHT15_humidity";
const char BMP085_TEMP_TOPIC[] PROGMEM = "weather/measurement/BMP085_temp";
const char BMP085_PRESSURE_TOPIC[] PROGMEM =
    "weather/measurement/BMP085_pressure";
const char TEMT6000_LIGHT_RAW_TOPIC[] PROGMEM =
    "weather/measurement/TEMT6000_light_raw";
const char TEMT6000_LIGHT_TOPIC[] PROGMEM =
    "weather/measurement/TEMT6000_light";
const char WIND_SPEED_TOPIC[] PROGMEM = "weather/measurement/wind_spd";
const char WIND_SPEED_MAX_TOPIC[] PROGMEM = "weather/measurement/wind_spd_max";
const char WIND_DIRECTION_TOPIC[] PROGMEM = "weather/measurement/wind_dir";
const char RAINFALL_TOPIC[] PROGMEM = "weather/measurement/rain";

const char DHT22_TEMP_TOPIC[] PROGMEM = "weather/measurement/DHT22_temp";
const char DHT22_HUMIDITY_TOPIC[] PROGMEM =
    "weather/measurement/DHT22_humidity";

PGM_P const MEASUREMENT_TOPICS[] PROGMEM = {
    SHT15_TEMP_TOPIC,         // idx = 0
    SHT15_HUMIDITY_TOPIC,     // idx = 1
    BMP085_TEMP_TOPIC,        // idx = 2
    BMP085_PRESSURE_TOPIC,    // idx = 3
    TEMT6000_LIGHT_RAW_TOPIC, // idx = 4
    TEMT6000_LIGHT_TOPIC,     // idx = 5
    WIND_SPEED_TOPIC,         // idx = 6
    WIND_SPEED_MAX_TOPIC,     // idx = 7
    WIND_DIRECTION_TOPIC,     // idx = 8
    RAINFALL_TOPIC,           // idx = 9
    DHT22_TEMP_TOPIC,         // idx = 10
    DHT22_HUMIDITY_TOPIC,     // idx = 11
};

// MEASUREMENT_TOPICS indices, must match table above
typedef enum {
  SHT15_TEMP_TOPIC_IDX = 0,
  SHT15_HUMIDITY_TOPIC_IDX = 1,
  BMP085_TEMP_TOPIC_IDX = 2,
  BMP085_PRESSURE_TOPIC_IDX = 3,
  TEMT6000_LIGHT_RAW_TOPIC_IDX = 4,
  TEMT6000_LIGHT_TOPIC_IDX = 5,
  WIND_SPEED_TOPIC_IDX = 6,
  WIND_SPEED_MAX_TOPIC_IDX = 7,
  WIND_DIRECTION_TOPIC_IDX = 8,
  RAINFALL_TOPIC_IDX = 9,
  DHT22_TEMP_TOPIC_IDX = 10,
  DHT22_HUMIDITY_TOPIC_IDX = 11,
} measurement_topics;

#endif /* MQTTWIFLYWEATHERBOARD_MQTTCONFIG_H_ */
