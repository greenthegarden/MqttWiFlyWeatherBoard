#ifndef DHT22CONFIG_H_
#define DHT22CONFIG_H_

// DHT22 temperature/humidty sensor library
#include <DHT.h>

DHT dht;

// DHT22 status messages
const char DHT22_STATUS_OK[] PROGMEM = "OK";
const char DHT22_CHECKSUM_ERROR[] PROGMEM = "ERROR: Checksum";
const char DHT22_TIMEOUT_ERROR[] PROGMEM = "ERROR: Time out";
const char DHT22_CONNECT_ERROR[] PROGMEM = "ERROR: Connect";
const char DHT22_ACK_LOW_ERROR[] PROGMEM = "ERROR: Ack Low";
const char DHT22_ACK_HIGH_ERROR[] PROGMEM = "ERROR: Ack High";
const char DHT22_UNKNOWN_ERROR[] PROGMEM = "ERROR: Unknown";

PGM_P const DHT22_STATUS_MESSAGES[] PROGMEM = {
    DHT22_STATUS_OK,      // idx = 0
    DHT22_CHECKSUM_ERROR, // idx = 1
    DHT22_TIMEOUT_ERROR,  // idx = 2
    DHT22_CONNECT_ERROR,  // idx = 3
    DHT22_ACK_LOW_ERROR,  // idx = 4
    DHT22_ACK_HIGH_ERROR, // idx = 5
    DHT22_UNKNOWN_ERROR,  // idx = 6
};
/* DHT22_STATUS_MESSAGES indices, must match table above */
typedef enum {
  DHT22_STATUS_OK_IDX = 0,
  DHT22_CHECKSUM_ERROR_IDX = 1,
  DHT22_TIMEOUT_ERROR_IDX = 2,
  DHT22_CONNECT_ERROR_IDX = 3,
  DHT22_ACK_LOW_ERROR_IDX = 4,
  DHT22_ACK_HIGH_ERROR_IDX = 5,
  DHT22_UNKNOWN_ERROR_IDX = 6,
} dht22_status_messages;

byte dht22_reading(byte pin) { return (dht.read22(pin)); }

#endif /* DHT22CONFIG_H_ */
