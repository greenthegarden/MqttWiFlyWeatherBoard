#ifndef MQTTWIFLYWEATHERBOARD_DHT22CONFIG_H_
#define MQTTWIFLYWEATHERBOARD_DHT22CONFIG_H_


#include <DHT22Config.h>


const int DHT22_PIN = 11;

byte dht22_measurement(JsonObject& root) {
  int chk = dht22_reading(DHT22_PIN);

  // progBuffer[0] = '\0';
  // strcpy_P(progBuffer,
  //          (char *)pgm_read_word(&(STATUS_TOPICS[DHT22_STATUS_IDX])));

  JsonObject& sensor = root.createNestedObject("dht22");

  char charBuffer[12];

  switch (chk) {
  case DHTLIB_OK:
//    DEBUG_LOG(1, "OK");
    strcpy_P(charBuffer, (char *)pgm_read_word(
                             &(DHT22_STATUS_MESSAGES[DHT22_STATUS_OK_IDX])));
    // #if ENABLE_MQTT
    // if (mqttClient.connected()) {
    //   mqttClient.publish(progBuffer, messBuffer);
    // }
    // #else
    // printTopicPayloadPair(topicBuffer, payloadBuffer);
    // #endif
    break;
  case DHTLIB_ERROR_CHECKSUM:
//    DEBUG_LOG(1, "Checksum error");
    strcpy_P(charBuffer, (char *)pgm_read_word(&(
                             DHT22_STATUS_MESSAGES[DHT22_CHECKSUM_ERROR_IDX])));
    // #if ENABLE_MQTT
    // if (mqttClient.connected()) {
    //   mqttClient.publish(progBuffer, messBuffer);
    // }
    // #else
    // printTopicPayloadPair(topicBuffer, payloadBuffer);
    // #endif
    break;
  case DHTLIB_ERROR_TIMEOUT:
//    DEBUG_LOG(1, "Time out error");
    strcpy_P(charBuffer, (char *)pgm_read_word(&(
                             DHT22_STATUS_MESSAGES[DHT22_TIMEOUT_ERROR_IDX])));
                            //  #if ENABLE_MQTT
                            //  if (mqttClient.connected()) {
                            //    mqttClient.publish(progBuffer, messBuffer);
                            //  }
                            //  #else
                            //  printTopicPayloadPair(topicBuffer, payloadBuffer);
                            //  #endif
    break;
  case DHTLIB_ERROR_CONNECT:
//    DEBUG_LOG(1, "Connect error");
    strcpy_P(charBuffer, (char *)pgm_read_word(&(
                             DHT22_STATUS_MESSAGES[DHT22_CONNECT_ERROR_IDX])));
                            //  #if ENABLE_MQTT
                            //  if (mqttClient.connected()) {
                            //    mqttClient.publish(progBuffer, messBuffer);
                            //  }
                            //  #else
                            //  printTopicPayloadPair(topicBuffer, payloadBuffer);
                            //  #endif
    break;
  case DHTLIB_ERROR_ACK_L:
//    DEBUG_LOG(1, "Ack Low error");
    strcpy_P(charBuffer, (char *)pgm_read_word(&(
                             DHT22_STATUS_MESSAGES[DHT22_ACK_LOW_ERROR_IDX])));
                            //  #if ENABLE_MQTT
                            //  if (mqttClient.connected()) {
                            //    mqttClient.publish(progBuffer, messBuffer);
                            //  }
                            //  #else
                            //  printTopicPayloadPair(topicBuffer, payloadBuffer);
                            //  #endif
    break;
  case DHTLIB_ERROR_ACK_H:
//    DEBUG_LOG(1, "Ack High error");
    strcpy_P(charBuffer, (char *)pgm_read_word(&(
                             DHT22_STATUS_MESSAGES[DHT22_ACK_HIGH_ERROR_IDX])));
                            //  #if ENABLE_MQTT
                            //  if (mqttClient.connected()) {
                            //    mqttClient.publish(progBuffer, messBuffer);
                            //  }
                            //  #else
                            //  printTopicPayloadPair(topicBuffer, payloadBuffer);
                            //  #endif
    break;
  default:
//    DEBUG_LOG(1, "Unknown error");
    strcpy_P(charBuffer, (char *)pgm_read_word(&(
                             DHT22_STATUS_MESSAGES[DHT22_UNKNOWN_ERROR_IDX])));
                            //  #if ENABLE_MQTT
                            //  if (mqttClient.connected()) {
                            //    mqttClient.publish(progBuffer, messBuffer);
                            //  }
                            //  #else
                            //  printTopicPayloadPair(topicBuffer, payloadBuffer);
                            //  #endif
    break;
  }
  sensor[F("status")] = charBuffer;
  return chk;
}

void publish_dht22_measurement(JsonObject& root) {
  if (dht22_measurement(root) == DHTLIB_OK) {

    JsonObject& sensor = root.createNestedObject("dht22");

    // value is stored in DHT object
    sensor[F("temp")] = dht.temperature;
    sensor[F("hum")] = dht.humidity;
  }
}

// void publish_dht22_humidity_measurement(JsonObject& root) {
//   DEBUG_LOG(3, "DHT22 humidity measurement: ");
//   // value is stored in DHT object
//   DEBUG_LOG(3, dht.humidity);
//   buf[0] = '\0';
//   dtostrf(dht.humidity, 1, FLOAT_DECIMAL_PLACES, buf);
//   progBuffer[0] = '\0';
//   strcpy_P(progBuffer, (char *)pgm_read_word(&(MEASUREMENT_TOPICS[11])));
//   #if ENABLE_MQTT
//   if (mqttClient.connected()) {
//     mqttClient.publish(progBuffer, buf);
//   }
//   #else
//   printTopicPayloadPair(topicBuffer, payloadBuffer);
//   #endif
// }
//
// // void publish_dht22_measurements(JsonObject& root) {
// //   if (dht22_measurement(root) == DHTLIB_OK) {
// //     publish_dht22_measurement(root);
// //   }
// // }


#endif /* MQTTWIFLYWEATHERBOARD_DHT22CONFIG_H_ */
