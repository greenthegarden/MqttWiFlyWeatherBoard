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


#endif /* MQTTWIFLYWEATHERBOARD_MQTTCONFIG_H_ */
