#ifndef MQTTWIFLYWEATHERBOARD_JSONCONFIG_H_
#define MQTTWIFLYWEATHERBOARD_JSONCONFIG_H_

#include <ArduinoJson.h>

StaticJsonBuffer<200> jsonBuffer;

// JsonObject& root = jsonBuffer.createObject();
// root["sensor"] = "gps";
// root["time"] = 1351824120;
//
// JsonArray& data = root.createNestedArray("data");
// data.add(48.756080, 6);  // 6 is the number of decimals to print
// data.add(2.302038, 6);   // if not specified, 2 digits are printed
//
// root.printTo(Serial);

#endif /* MQTTWIFLYWEATHERBOARD_JSONCONFIG_H_ */
