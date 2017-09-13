#ifndef MQTTWIFLYWEATHERBOARD_SUNAIRPLUSCONFIG_H_
#define MQTTWIFLYWEATHERBOARD_SUNAIRPLUSCONFIG_H_


#include <Wire.h>
#include <SDL_Arduino_INA3221.h>

SDL_Arduino_INA3221 ina3221;

// the three channels of the INA3221 named for SunAirPlus Solar Power Controller channels (www.switchdoc.com)
#define LIPO_BATTERY_CHANNEL    1
#define SOLAR_CELL_CHANNEL      2
#define OUTPUT_CHANNEL          3

// sunairplus measurement topics

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

/* SUNAIRPLUS_TOPICS indices, must match table above */
typedef enum {
  BATTERY_VOLTAGE_TOPIC_IDX  = 0,
  BATTERY_CURRENT_TOPIC_IDX  = 1,
  SOLAR_VOLTAGE_TOPIC_IDX    = 2,
  SOLAR_CURRENT_TOPIC_IDX    = 3,
  OUTPUT_VOLTAGE_TOPIC_IDX   = 4,
  OUTPUT_CURRENT_TOPIC_IDX   = 5,
} sunairplus_topics;

#if ENABLE_JSON
void publish_sunairplus_measurement()
  JsonObject& root = payloadBuffer.createObject();
  root[F("sensor")] = F("gps");
  root[F("time")] = 1351824120;
}
#else
void publish_sunairplus_measurement()
{
  float measurement = 0.0;

  //LIPO battery measurements
  measurement = ina3221.getBusVoltage_V(LIPO_BATTERY_CHANNEL);
  topicBuffer[0] = '\0';
  strcpy_P(topicBuffer, (char*)pgm_read_word(&(SUNAIRPLUS_TOPICS[BATTERY_VOLTAGE_TOPIC_IDX])));
  payloadBuffer[0] = '\0';
  dtostrf(measurement,1,FLOAT_DECIMAL_PLACES, payloadBuffer);
  #if ENABLE_MQTT
  if (mqttClient.connected()) {
    mqttClient.publish(topicBuffer, payloadBuffer);
  }
  #else
  printTopicPayloadPair(topicBuffer, payloadBuffer);
  #endif
  measurement = ina3221.getCurrent_mA(LIPO_BATTERY_CHANNEL);
  topicBuffer[0] = '\0';
  strcpy_P(topicBuffer, (char*)pgm_read_word(&(SUNAIRPLUS_TOPICS[BATTERY_CURRENT_TOPIC_IDX])));
  payloadBuffer[0] = '\0';
  dtostrf(measurement,1,FLOAT_DECIMAL_PLACES, payloadBuffer);
  #if ENABLE_MQTT
  if (mqttClient.connected()) {
    mqttClient.publish(topicBuffer, payloadBuffer);
  }
  #else
  printTopicPayloadPair(topicBuffer, payloadBuffer);
  #endif

  // Solar cell measurements
  measurement = ina3221.getBusVoltage_V(SOLAR_CELL_CHANNEL);
  topicBuffer[0] = '\0';
  strcpy_P(topicBuffer, (char*)pgm_read_word(&(SUNAIRPLUS_TOPICS[SOLAR_VOLTAGE_TOPIC_IDX])));
  payloadBuffer[0] = '\0';
  dtostrf(measurement,1,FLOAT_DECIMAL_PLACES, payloadBuffer);
  #if ENABLE_MQTT
  if (mqttClient.connected()) {
    mqttClient.publish(topicBuffer, payloadBuffer);
  }
  #else
  printTopicPayloadPair(topicBuffer, payloadBuffer);
  #endif
  measurement = ina3221.getCurrent_mA(SOLAR_CELL_CHANNEL);
  topicBuffer[0] = '\0';
  strcpy_P(topicBuffer, (char*)pgm_read_word(&(SUNAIRPLUS_TOPICS[SOLAR_CURRENT_TOPIC_IDX])));
  payloadBuffer[0] = '\0';
  dtostrf(measurement,1,FLOAT_DECIMAL_PLACES, payloadBuffer);
  #if ENABLE_MQTT
  if (mqttClient.connected()) {
    mqttClient.publish(topicBuffer, payloadBuffer);
  }
  #else
  printTopicPayloadPair(topicBuffer, payloadBuffer);
  #endif

  // SunAirPlus output measurements
  measurement = ina3221.getBusVoltage_V(OUTPUT_CHANNEL);
  topicBuffer[0] = '\0';
  strcpy_P(topicBuffer, (char*)pgm_read_word(&(SUNAIRPLUS_TOPICS[OUTPUT_VOLTAGE_TOPIC_IDX])));
  payloadBuffer[0] = '\0';
  dtostrf(measurement,1,FLOAT_DECIMAL_PLACES, payloadBuffer);
  #if ENABLE_MQTT
  if (mqttClient.connected()) {
    mqttClient.publish(topicBuffer, payloadBuffer);
  }
  #else
  printTopicPayloadPair(topicBuffer, payloadBuffer);
  #endif
  measurement = ina3221.getCurrent_mA(OUTPUT_CHANNEL);
  topicBuffer[0] = '\0';
  strcpy_P(topicBuffer, (char*)pgm_read_word(&(SUNAIRPLUS_TOPICS[OUTPUT_CURRENT_TOPIC_IDX])));
  payloadBuffer[0] = '\0';
  dtostrf(measurement,1,FLOAT_DECIMAL_PLACES, payloadBuffer);
  #if ENABLE_MQTT
  if (mqttClient.connected()) {
    mqttClient.publish(topicBuffer, payloadBuffer);
  }
  #else
  printTopicPayloadPair(topicBuffer, payloadBuffer);
  #endif
}
#endif


#endif  /* MQTTWIFLYWEATHERBOARD_SUNAIRPLUSCONFIG_H_ */
