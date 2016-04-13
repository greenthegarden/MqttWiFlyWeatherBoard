#ifndef MQTTWIFLYWEATHERBOARD_DHT22CONFIG_H_
#define MQTTWIFLYWEATHERBOARD_DHT22CONFIG_H_


#include <DHT22Config.h>       // DHT22 temperature/humidty sensor library


const int DHT22_PIN = 11;

byte dht22_measurement()
{
  byte chk = dht22_reading(DHT22_PIN);

  progBuffer[0] = '\0';
  strcpy_P(progBuffer, (char*)pgm_read_word(&(STATUS_TOPICS[5])));

  switch (chk) {
    case DHTLIB_OK :
      DEBUG_LOG(1, "OK");
      messBuffer[0] = '\0';
      strcpy_P(messBuffer, (char*)pgm_read_word(&(DHT22_STATUS_MESSAGES[0])));
      mqttClient.publish(progBuffer,messBuffer);
      break;
    case DHTLIB_ERROR_CHECKSUM :
      DEBUG_LOG(1, "Checksum error");
      messBuffer[0] = '\0';
      strcpy_P(messBuffer, (char*)pgm_read_word(&(DHT22_STATUS_MESSAGES[1])));
      mqttClient.publish(progBuffer,messBuffer);
      break;
    case DHTLIB_ERROR_TIMEOUT :
      DEBUG_LOG(1, "Time out error");
      messBuffer[0] = '\0';
      strcpy_P(messBuffer, (char*)pgm_read_word(&(DHT22_STATUS_MESSAGES[2])));
      mqttClient.publish(progBuffer,messBuffer);
      break;
    case DHTLIB_ERROR_CONNECT :
      DEBUG_LOG(1, "Connect error");
      messBuffer[0] = '\0';
      strcpy_P(messBuffer, (char*)pgm_read_word(&(DHT22_STATUS_MESSAGES[3])));
      mqttClient.publish(progBuffer,messBuffer);
      break;
    case DHTLIB_ERROR_ACK_L :
      DEBUG_LOG(1, "Ack Low error");
      messBuffer[0] = '\0';
      strcpy_P(messBuffer, (char*)pgm_read_word(&(DHT22_STATUS_MESSAGES[4])));
      mqttClient.publish(progBuffer,messBuffer);
      break;
    case DHTLIB_ERROR_ACK_H :
      DEBUG_LOG(1, "Ack High error");
      messBuffer[0] = '\0';
      strcpy_P(messBuffer, (char*)pgm_read_word(&(DHT22_STATUS_MESSAGES[5])));
      mqttClient.publish(progBuffer,messBuffer);
      break;
    default :
      DEBUG_LOG(1, "Unknown error");
      messBuffer[0] = '\0';
      strcpy_P(messBuffer, (char*)pgm_read_word(&(DHT22_STATUS_MESSAGES[6])));
      mqttClient.publish(progBuffer,messBuffer);
      break;
  }
  return chk;
}

void publish_dht22_temperature_measurement()
{
  DEBUG_LOG(3, "DHT22 temperature measurement: ");
  // value is stored in DHT object
  DEBUG_LOG(3, dht.temperature);
  buf[0] = '\0';
  dtostrf(dht.temperature, 1, FLOAT_DECIMAL_PLACES, buf);
  progBuffer[0] = '\0';
  strcpy_P(progBuffer, (char*)pgm_read_word(&(MEASUREMENT_TOPICS[10])));
  mqttClient.publish(progBuffer, buf);
}

void publish_dht22_humidity_measurement()
{
  DEBUG_LOG(3, "DHT22 humidity measurement: ");
  // value is stored in DHT object
  DEBUG_LOG(3, dht.humidity);
  buf[0] = '\0';
  dtostrf(dht.humidity, 1, FLOAT_DECIMAL_PLACES, buf);
  progBuffer[0] = '\0';
  strcpy_P(progBuffer, (char*)pgm_read_word(&(MEASUREMENT_TOPICS[11])));
  mqttClient.publish(progBuffer, buf);
}

void publish_dht22_measurements()
{
  if (dht22_measurement() == DHTLIB_OK) {
    publish_dht22_temperature_measurement();
    publish_dht22_humidity_measurement();
  }
}


#endif  /* MQTTWIFLYWEATHERBOARD_MQTTCONFIG_H_ */

