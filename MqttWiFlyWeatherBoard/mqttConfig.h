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


#endif /* MQTTWIFLYWEATHERBOARD_MQTTCONFIG_H_ */
