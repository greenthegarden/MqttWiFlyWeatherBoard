#ifndef MQTTWIFLYWEATHERBOARD_WEATHERBOARDCONFIG_H_
#define MQTTWIFLYWEATHERBOARD_WEATHERBOARDCONFIG_H_

// external sensor libraries
#include <SFE_BMP085.h> // BMP085 pressure sensor library
#include <SHT1x.h>      // SHT15 temperature/humidity sensor library
#include <Wire.h>       // I2C library (necessary for pressure sensor)

#include "progmemStrings.h"

// Weather Board Digital I/O pin definitions

#define RAIN 2
#define WSPEED 3
#define STATUS_LED 4
#define RF_CTS 5
#define RF_RTS 6
#define EOC 8
#define XCLR 9

// (the following three are predefined)
// #define MOSI        11
// #define MISO        12
// #define SCK         13

// analog I/O pins
#define WDIR A0
#define SHT1x_DATA A4
#define SHT1x_CLOCK A5
#define BATT_LVL A6
#define LIGHT A7

// global variable definitions
boolean pressureSensorStatus = false;

// sensor status topics
const char BMP085_STATUS[] PROGMEM = "weather/status/bmp085";
const char WEATHER_METERS_STATUS[] PROGMEM = "weather/status/wm";
const char DHT22_STATUS[] PROGMEM = "weather/status/dht22";

PGM_P const SENSOR_STATUS_TOPICS[] PROGMEM = {
    BMP085_STATUS,         // idx = 4
    WEATHER_METERS_STATUS, // idx = 5
    DHT22_STATUS,          // idx = 6
};

// SENSOR_STATUS_TOPICS indices, must match table above
typedef enum {
  BMP085_STATUS_IDX = 0,
  WEATHER_METERS_STATUS_IDX = 1,
  DHT22_STATUS_IDX = 2,
} sensor_status_topics;

// sensor topics
const char SENSOR_SHT15_TOPIC[] PROGMEM = "weather/sensor/SHT15";
const char SENSOR_BMP085_TOPIC[] PROGMEM = "weather/sensor/BMP085";
const char SENSOR_TEMT6000_TOPIC[] PROGMEM = "weather/sensor/TEMT6000";
const char SENSOR_WIND_TOPIC[] PROGMEM = "weather/sensor/wind";
const char SENSOR_RAINFALL_TOPIC[] PROGMEM = "weather/sensor/rainfall";

PGM_P const SENSOR_TOPICS[] PROGMEM = {
    SENSOR_SHT15_TOPIC,         // idx = 0
    SENSOR_BMP085_TOPIC,     // idx = 1
    SENSOR_TEMT6000_TOPIC,        // idx = 2
    SENSOR_WIND_TOPIC,    // idx = 3
    SENSOR_RAINFALL_TOPIC, // idx = 4
};

// SENSOR_TOPICS indices, must match table above
typedef enum {
  SENSOR_SHT15_TOPIC_IDX = 0,
  SENSOR_BMP085_TOPIC_IDX = 1,
  SENSOR_TEMT6000_TOPIC_IDX = 2,
  SENSOR_WIND_TOPIC_IDX = 3,
  SENSOR_RAINFALL_TOPIC_IDX = 4,
} sensor_topics;

// BMP085 status messages

const char BMP085_INIT_SUCCESS[] PROGMEM = "Success";
const char BMP085_INIT_FAILURE[] PROGMEM = "Failure";
const char BMP085_ERROR_TEMP_START[] PROGMEM = "ERROR: Temperature Start";
const char BMP085_ERROR_TEMP_GET[] PROGMEM = "ERROR: Temperature Get";
const char BMP085_ERROR_PRESSURE_START[] PROGMEM = "ERROR: Pressure Start";
const char BMP085_ERROR_PRESSURE_GET[] PROGMEM = "ERROR: Pressure Get";

PGM_P const BMP085_STATUS_MESSAGES[] PROGMEM = {
    BMP085_INIT_SUCCESS,         // idx = 0
    BMP085_INIT_FAILURE,         // idx = 1
    BMP085_ERROR_TEMP_START,     // idx = 2
    BMP085_ERROR_TEMP_GET,       // idx = 3
    BMP085_ERROR_PRESSURE_START, // idx = 4
    BMP085_ERROR_PRESSURE_GET,   // idx = 5
};

/* bmp085_status_messages indices, must match table above */
typedef enum {
  BMP085_INIT_SUCCESS_IDX = 0,
  BMP085_INIT_FAILURE_IDX = 1,
  BMP085_ERROR_TEMP_START_IDX = 2,
  BMP085_ERROR_TEMP_GET_IDX = 3,
  BMP085_ERROR_PRESSURE_START_IDX = 4,
  BMP085_ERROR_PRESSURE_GET_IDX = 5,
} bmp085_status_messages;

// instantiation of sensor objects
SHT1x humiditySensor(SHT1x_DATA, SHT1x_CLOCK);
SFE_BMP085 pressureSensor(BMP_ADDR);

// void rfpins_init() {
//   // configure CTS and RTS pins as inputs
//   pinMode(RF_CTS, INPUT);
//   pinMode(RF_RTS, INPUT);
// }

void weatherboard_sensors_init()
{
  // Configure sensors
  // set up inputs and outputs
  pinMode(XCLR, OUTPUT);    // output to BMP085 reset (unused)
  digitalWrite(XCLR, HIGH); // make pin high to turn off reset

  pinMode(EOC, INPUT);    // input from BMP085 end of conversion (unused)
  digitalWrite(EOC, LOW); // turn off pullup

  // Reset the humidity sensor connection so that the I2C bus can be accessed
  TWCR &= ~(_BV(TWEN));    // turn off I2C enable bit so we can access the SHT15
                           // humidity sensor
  digitalWrite(XCLR, LOW); // disable the BMP085 while resetting humidity sensor
  // humiditySensor.connectionReset();     // reset the humidity sensor
  // connection
  TWCR |= _BV(TWEN); // turn on I2C enable bit so we can access the BMP085
                     // pressure sensor
  digitalWrite(XCLR, HIGH); // enable BMP085
  delay(10); // wait for the BMP085 pressure sensor to become ready after reset

  // init BMP085 pressure sensor and publish result
  char topicBuffer[TOPIC_BUFFER_SIZE];
  strcpy_P(topicBuffer, (char *)pgm_read_word(&(SENSOR_TOPICS[SENSOR_BMP085_TOPIC_IDX])));

  StaticJsonBuffer<JSON_BUFFER_SIZE> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();

  char charBuffer[12];
  if (pressureSensor.begin()) { // initialize the BMP085 pressure sensor
                                // (important to get calibration values stored
                                // on the device)
    pressureSensorStatus = true;
    strcpy_P(charBuffer, (char *)pgm_read_word(&(BMP085_STATUS_MESSAGES[BMP085_INIT_SUCCESS_IDX])));
  } else {
    strcpy_P(charBuffer, (char *)pgm_read_word(&(BMP085_STATUS_MESSAGES[BMP085_INIT_FAILURE_IDX])));
  }
  root[F("init")] = charBuffer;

  char payloadBuffer[PAYLOAD_BUFFER_SIZE];
  root.printTo(payloadBuffer);
  #if ENABLE_MQTT
  if (mqttClient.connected()) {
    mqttClient.publish(topicBuffer, payloadBuffer);
  }
  #else
  Serial.println(payloadBuffer);
//  printTopicPayloadPair(topicBuffer, payloadBuffer);
  #endif
}

void publish_sht15_measurements(JsonObject& root)
{
  TWCR &= ~(_BV(TWEN)); // turn off I2C enable bit to allow access to
                        // the SHT15 humidity sensor

  // char topicBuffer[TOPIC_BUFFER_SIZE];
  // strcpy_P(topicBuffer, (char *)pgm_read_word(&(SENSOR_TOPICS[SENSOR_SHT15_TOPIC_IDX])));

//  StaticJsonBuffer<JSON_BUFFER_SIZE> jsonBuffer;
//  JsonObject& root = jsonBuffer.createObject();

  JsonObject& sensor = root.createNestedObject("sht15");

  float measurement = 0.0;

  // humidity reading
  measurement = humiditySensor.readTemperatureC(); // temperature returned in degrees Celcius
  sensor[F("temp")] = measurement;

  measurement = humiditySensor.readHumidity();
  sensor[F("hum")] = measurement;

//   char payloadBuffer[PAYLOAD_BUFFER_SIZE];
//   root.printTo(payloadBuffer);
//   #if ENABLE_MQTT
//   if (mqttClient.connected()) {
//     mqttClient.publish(topicBuffer, payloadBuffer);
//   }
//   #else
//   Serial.println(payloadBuffer);
// //  printTopicPayloadPair(topicBuffer, payloadBuffer);
//   #endif
}

void publish_bmp085_measurements(JsonObject& root)
{
  TWCR |= _BV(TWEN); // turn on I2C enable bit to allow access to
                     // the BMP085 pressure sensor

  // char topicBuffer[TOPIC_BUFFER_SIZE];
  // strcpy_P(topicBuffer, (char *)pgm_read_word(&(SENSOR_TOPICS[SENSOR_BMP085_TOPIC_IDX])));

  // StaticJsonBuffer<JSON_BUFFER_SIZE> jsonBuffer;
  // JsonObject& root = jsonBuffer.createObject();
//  root[F("sensor")] = F("bmp085");

  JsonObject& sensor = root.createNestedObject("bmp085");

  byte status;
  double bmp085Temp = 0.0;
  double bmp085Pressure = 0.0;

  // start BMP085 temperature reading
  // tell the sensor to start a temperature measurement
  // if request is successful, the number of ms to wait is returned
  // if request is unsuccessful, 0 is returned
  status = pressureSensor.startTemperature();
  if (status != 0) {
    // wait for the measurement to complete
    delay(status);

    // retrieve BMP085 temperature reading
    // function returns 1 if successful, 0 if failure
    status = pressureSensor.getTemperature(&bmp085Temp); // temperature returned in degrees Celcius

    if (status != 0) {
      // publish BMP085 temperature measurement
      sensor[F("temp")] = bmp085Temp;

      // tell the sensor to start a pressure measurement
      // the parameter is the oversampling setting, from 0 to 3 (highest res,
      // longest wait)
      // if request is successful, the number of ms to wait is returned
      // if request is unsuccessful, 0 is returned
      status = pressureSensor.startPressure(3);

      if (status != 0) {
        // wait for the measurement to complete
        delay(status);

        // retrieve the BMP085 pressure reading
        // note that the function requires the previous temperature measurement
        // (T)
        // (if temperature is stable, one temperature measurement can be used
        // for a number of pressure measurements)
        // function returns 1 if successful, 0 if failure
        status = pressureSensor.getPressure(&bmp085Pressure,
                                            &bmp085Temp); // mbar, deg C

        if (status != 0) {
          // publish BMP085 pressure measurement
          // pressure in millibars (or hectopascal/hPa)
          // 1 millibar is equivalent to 0.02953 inches of mercury (Hg)
          sensor[F("pres")] = bmp085Pressure;
        } else {
          // publish pressure get error
          char charBuffer[12];
          strcpy_P(charBuffer,
                   (char *)pgm_read_word(&(BMP085_STATUS_MESSAGES[BMP085_ERROR_PRESSURE_GET_IDX])));
          sensor[F("err")] = charBuffer;
        }
      } else {
        // publish pressure start error
        char charBuffer[12];
        strcpy_P(charBuffer,
                 (char *)pgm_read_word(&(BMP085_STATUS_MESSAGES[BMP085_ERROR_PRESSURE_START_IDX])));
        sensor[F("err")] = charBuffer;
       }
    } else {
      // publish temperature get error
      char charBuffer[12];
      strcpy_P(charBuffer, (char *)pgm_read_word(&(BMP085_STATUS_MESSAGES[BMP085_ERROR_TEMP_GET_IDX])));
      sensor[F("err")] = charBuffer;
    }
  } else {
    // publish temperature start error
    char charBuffer[12];
    strcpy_P(charBuffer, (char *)pgm_read_word(&(BMP085_STATUS_MESSAGES[BMP085_ERROR_TEMP_START_IDX])));
    sensor[F("err")] = charBuffer;
  }
//   char payloadBuffer[PAYLOAD_BUFFER_SIZE];
//   root.printTo(payloadBuffer);
//   #if ENABLE_MQTT
//   if (mqttClient.connected()) {
//     mqttClient.publish(topicBuffer, payloadBuffer);
//   }
//   #else
//   Serial.println(payloadBuffer);
// //  printTopicPayloadPair(topicBuffer, payloadBuffer);
//   #endif
}

void publish_temt6000_measurement(JsonObject& root)
{
  // char topicBuffer[TOPIC_BUFFER_SIZE];
  // strcpy_P(topicBuffer, (char *)pgm_read_word(&(SENSOR_TOPICS[SENSOR_TEMT6000_TOPIC_IDX])));

  // StaticJsonBuffer<JSON_BUFFER_SIZE> jsonBuffer;
  // JsonObject& root = jsonBuffer.createObject();
  // root[F("sensor")] = F("temt6000");

  JsonObject& sensor = root.createNestedObject("temt6000");

// get light level
#if ENABLE_EXTERNAL_LIGHT
  // higher reading corresponds to brighter conditions
  int TEMT6000_light_raw = analogRead(LIGHT);
#else
  // lower reading corresponds to brighter conditions
  int TEMT6000_light_raw = 1023 - analogRead(LIGHT);
#endif

  sensor[F("raw")] = TEMT6000_light_raw;

  // convert TEMT6000_light_raw voltage value to percentage
  // map(value, fromLow, fromHigh, toLow, toHigh)
  int TEMT6000_light = map(TEMT6000_light_raw, 0, 1023, 0, 100);
  sensor[F("level")] = TEMT6000_light;

//   char payloadBuffer[PAYLOAD_BUFFER_SIZE];
//   root.printTo(payloadBuffer);
//   #if ENABLE_MQTT
//   if (mqttClient.connected()) {
//     mqttClient.publish(topicBuffer, payloadBuffer);
//   }
//   #else
//   Serial.println(payloadBuffer);
// //  printTopicPayloadPair(topicBuffer, payloadBuffer);
//   #endif
}

#if ENABLE_WEATHER_METERS

// Global variables
const unsigned int ZERODELAY =
    4000; // ms, zero RPM if no result for this time period
unsigned int windRpm = 0;
unsigned int windRpmMax = 0;
unsigned int windStopped = 0;
// volatiles are subject to modification by IRQs
volatile unsigned long tempWindRpm = 0UL;
volatile unsigned long windTime = 0UL;
volatile unsigned long windLast = 0UL;
volatile unsigned long windInterval = 0UL;
volatile unsigned char windIntCount;
volatile boolean gotWindSpeed;
volatile unsigned long rainTime = 0UL;
volatile unsigned long rainLast = 0UL;
volatile unsigned long rainInterval = 0UL;
volatile unsigned long rain = 0UL;

// Constant conversion factors
// const float WIND_RPM_TO_MPH  = 22.686745;         // divide RPM by this for
// velocity
const float WIND_RPM_TO_MPS =
    50.748803; // divide RPM by this for meters per second
// 1 m/s = 1.943844492 knots
const float WIND_RPM_TO_KNOTS = WIND_RPM_TO_MPS / 1.943844492;
// const float RAIN_BUCKETS_TO_INCHES = 0.014815;    // multiply bucket tips by
// this for inches
const float RAIN_BUCKETS_TO_MM =
    0.376296; // multiply bucket tips by this for mm

#if ENABLE_WIND_MEASUREMENT_AVERAGING
#include "runningAverageConfig.h"
unsigned long WIND_MEASUREMENT_INTERVAL = 1000UL; // milliSeconds
#endif

void wind_speed_irq()
// if the Weather Meters are attached, measure anemometer RPM (2 ticks per
// rotation), set flag if RPM is updated
// activated by the magnet in the anemometer (2 ticks per rotation), attached to
// input D3

// this routine measures RPM by measuring the time between anemometer pulses
// windintcount is the number of pulses we've measured - we need two to measure
// one full rotation (eliminates any bias between the position of the two
// magnets)
// when windintcount is 2, we can calculate the RPM based on the total time from
// when we got the first pulse
// note that this routine still needs an outside mechanism to zero the RPM if
// the anemometer is stopped (no pulses occur within a given period of time)
{
  windTime = micros(); // grab current time
  if ((windIntCount == 0) || ((windTime - windLast) > 10000)) {
    // ignore switch-bounce glitches less than 10ms after the reed switch closes
    if (windIntCount == 0) {
      // if we're starting a new measurement, reset the interval
      windInterval = 0;
    } else {
      // otherwise, add current interval to the interval timer
      windInterval += (windTime - windLast);
    }
    if (windIntCount == 2) {
      // we have two measurements (one full rotation), so calculate result and
      // start a new measurement
      tempWindRpm = (60000000UL / windInterval); // calculate RPM (temporary
                                                 // since it may change
                                                 // unexpectedly)
      windIntCount = 0;
      windInterval = 0;
      gotWindSpeed = true; // set flag for main loop
    }

    windIntCount++;
    windLast = windTime; // save the current time so that we can calculate the
                         // interval between now and the next interrupt
  }
}

float get_wind_direction()
// read the wind direction sensor, return heading in degrees
{
  unsigned int adc =
      analogRead(WDIR); // get the current reading from the sensor

  // The following table is ADC readings for the wind direction sensor output,
  // sorted from low to high.
  // Each threshold is the midpoint between adjacent headings. The output is
  // degrees for that ADC reading.
  // Note that these are not in compass degree order!  See Weather Meters
  // datasheet for more information.

  if (adc < 380)
    return (112.5);
  if (adc < 393)
    return (67.5);
  if (adc < 414)
    return (90);
  if (adc < 456)
    return (157.5);
  if (adc < 508)
    return (135);
  if (adc < 551)
    return (202.5);
  if (adc < 615)
    return (180);
  if (adc < 680)
    return (22.5);
  if (adc < 746)
    return (45);
  if (adc < 801)
    return (247.5);
  if (adc < 833)
    return (225);
  if (adc < 878)
    return (337.5);
  if (adc < 913)
    return (0);
  if (adc < 940)
    return (292.5);
  if (adc < 967)
    return (315);
  if (adc < 990)
    return (270);
  return (-1); // error, disconnected?
}

/* From Weather Meters docs and the Weather Board V3 schematic:

 heading         resistance      volts           nominal         midpoint (<)
 112.5  º 0.69  k 1.2 V 372 counts  380
 67.5 º 0.89  k 1.26  V 389 counts  393
 90 º 1 k 1.29  V 398 counts  414
 157.5  º 1.41  k 1.39  V 430 counts  456
 135  º 2.2 k 1.56  V 483 counts  508
 202.5  º 3.14  k 1.72  V 534 counts  551
 180  º 3.9 k 1.84  V 569 counts  615
 22.5 º 6.57  k 2.13  V 661 counts  680
 45 º 8.2 k 2.26  V 700 counts  746
 247.5  º 14.12 k 2.55  V 792 counts  801
 225  º 16  k 2.62  V 811 counts  833
 337.5  º 21.88 k 2.76  V 855 counts  878
 0  º 33  k 2.91  V 902 counts  913
 292.5  º 42.12 k 2.98  V 925 counts  940
 315  º 64.9  k 3.08  V 956 counts  967
 270  º 98.6  k 3.15  V 978 counts  >967
 */

void rain_irq()
// if the Weather Meters are attached, count rain gauge bucket tips as they
// occur
// activated by the magnet and reed switch in the rain gauge, attached to input
// D2
{
  rainTime = micros(); // grab current time
  rainInterval =
      rainTime - rainLast; // calculate interval between this and last event

  if (rainInterval > 100) {
    // ignore switch-bounce glitches less than 100uS after initial edge
    rain++;              // increment bucket counter
    rainLast = rainTime; // set up for next event
  }
}

byte weatherboard_meters_connected()
{
  char topicBuffer[TOPIC_BUFFER_SIZE];
  strcpy_P(topicBuffer, (char *)pgm_read_word(&(SENSOR_TOPICS[SENSOR_WIND_TOPIC_IDX])));

  StaticJsonBuffer<JSON_BUFFER_SIZE> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
  root[F("sensor")] = F("weather");

  if (get_wind_direction() < 0) {
    // likely that weather meters are not conneced
    char charBuffer[12];
    strcpy_P(charBuffer, (char *)pgm_read_word(&(PROGMEM_STRINGS[PROGMEM_STRING_ERROR_IDX])));
    root[F("state")] = charBuffer;

    char payloadBuffer[PAYLOAD_BUFFER_SIZE];
    root.printTo(payloadBuffer);
    #if ENABLE_MQTT
    if (mqttClient.connected()) {
      mqttClient.publish(topicBuffer, payloadBuffer);
    }
    #else
    Serial.println(payloadBuffer);
//    printTopicPayloadPair(topicBuffer, payloadBuffer);
    #endif
    return 0;
  }
  char charBuffer[12];
  strcpy_P(charBuffer, (char *)pgm_read_word(&(PROGMEM_STRINGS[PROGMEM_STRING_OK_IDX])));
  root[F("state")] = charBuffer;

  char payloadBuffer[PAYLOAD_BUFFER_SIZE];
  root.printTo(payloadBuffer);
  #if ENABLE_MQTT
  if (mqttClient.connected()) {
    mqttClient.publish(topicBuffer, payloadBuffer);
  }
  #else
  Serial.println(payloadBuffer);
//  printTopicPayloadPair(topicBuffer, payloadBuffer);
  #endif
  return 1;
}

void weatherboard_meters_init()
{
  // check wind direction measurement which will indicate an error
  // publish an error if not connected
  if(weatherboard_meters_connected()) {

    pinMode(WSPEED, INPUT);     // input from wind meters windspeed sensor
    digitalWrite(WSPEED, HIGH); // turn on pullup

    pinMode(RAIN, INPUT);     // input from wind meters rain gauge sensor
    digitalWrite(RAIN, HIGH); // turn on pullup

    // init wind speed interrupt global variables
    gotWindSpeed = false;
    windRpm = 0;
    windIntCount = 0;

#if ENABLE_WIND_MEASUREMENT_AVERAGING
    // explicitly start clean
    wind_spd_avg.clear();
    wind_dir_avg.clear();
#endif

    // attach external interrupt pins to IRQ functions
    attachInterrupt(0, rain_irq, FALLING);
    attachInterrupt(1, wind_speed_irq, FALLING);
  }
}

void publish_wind_measurement()
{
  char topicBuffer[TOPIC_BUFFER_SIZE];
  strcpy_P(topicBuffer, (char *)pgm_read_word(&(SENSOR_TOPICS[SENSOR_WIND_TOPIC_IDX])));

  StaticJsonBuffer<JSON_BUFFER_SIZE> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
  root[F("sensor")] = F("wind");

  float windSpeedMeasurement = -1.0; // this value should never be published
// publish instantaneous wind speed
#if ENABLE_WIND_MEASUREMENT_AVERAGING
  windSpeedMeasurement = wind_spd_avg.getAverage();
#else
  windSpeedMeasurement = float(windRpm) / WIND_RPM_TO_KNOTS;
#endif

  root[F("speed")] = windSpeedMeasurement;

  // publish maximum wind speed since last report
  windSpeedMeasurement = float(windRpmMax) / WIND_RPM_TO_KNOTS;

  root[F("maxspeed")] = windSpeedMeasurement;

  float WM_wdirection = -1.0;
#if ENABLE_WIND_MEASUREMENT_AVERAGING
  WM_wdirection = wind_dir_avg.getAverage();
#else
  // use instantaneous wind direction
  WM_wdirection = get_wind_direction(); // should return a -1 if disconnected
#endif
  if (WM_wdirection >= 0) {
    root[F("dir")] = WM_wdirection;
  } else {
    root[F("dir")] = F("err");
  }

  char payloadBuffer[PAYLOAD_BUFFER_SIZE];
  root.printTo(payloadBuffer);
  #if ENABLE_MQTT
  if (mqttClient.connected()) {
    mqttClient.publish(topicBuffer, payloadBuffer);
  }
  #else
  Serial.println(payloadBuffer);
//  printTopicPayloadPair(topicBuffer, payloadBuffer);
  #endif
}

// byte publish_wind_direction_measurement()
// {
//   char topicBuffer[TOPIC_BUFFER_SIZE];
//   strcpy_P(topicBuffer, (char *)pgm_read_word(&(SENSOR_TOPICS[WIND_SPEED_TOPIC_IDX])));
//   StaticJsonBuffer<JSON_BUFFER_SIZE> jsonBuffer;
//   JsonObject& root = jsonBuffer.createObject();
//   root[F("sensor")] = F("wind");
//
//   return 0;
// }

void publish_rainfall_measurement()
{
  char topicBuffer[TOPIC_BUFFER_SIZE];
  strcpy_P(topicBuffer, (char *)pgm_read_word(&(SENSOR_TOPICS[SENSOR_RAINFALL_TOPIC_IDX])));

  StaticJsonBuffer<JSON_BUFFER_SIZE> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
  root[F("sensor")] = F("rain");

  // rainfall unit conversion
  float rainfallMeasurement = rain * RAIN_BUCKETS_TO_MM;

  root[F("rainfall")] = rainfallMeasurement;

  char payloadBuffer[PAYLOAD_BUFFER_SIZE];
  root.printTo(payloadBuffer);
  #if ENABLE_MQTT
  if (mqttClient.connected()) {
    mqttClient.publish(topicBuffer, payloadBuffer);
  }
  #else
  Serial.println(payloadBuffer);
//  printTopicPayloadPair(topicBuffer, payloadBuffer);
  #endif
  // reset value of rain to zero
  rain = 0;
}

void publish_weather_meter_measurement()
{
  // take wind-direction measurement first
  // if returns -1 then treat as sensors not connected
  if (weatherboard_meters_connected()) {
    publish_wind_measurement();
    publish_rainfall_measurement();
  }
}

#endif /* ENABLE_WEATHER_METERS */


#endif /* MQTTWIFLYWEATHERBOARD_WEATHERBOARDCONFIG_H_ */
