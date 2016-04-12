/*
  Code based on Sparkfun WeatherShield v1.4 source from
  https://github.com/sparkfun/USB_Weather_Board/
  The libaraies included with the Sparkfun source code,
  for the temperature sensor (SHT1x) and pressure sensor  (SFE_BMP085)
  are required to be installed in the Arduino library folder
  in order to compile the code.

  In addition, the Wifly-MQTT library from
  https://github.com/greenthegarden/WiFly
  is used to provide the MQTT interface.
*/

/*
  To compile Set Tools/Board in the Arduino IDE to
  "Arduino Pro or Pro Mini (3.3V 8MHz) w/ ATmega328"

  While uploading to code to the Weather Board, remove the WiFly module and
  ensure the Comm switch is set to 'USB'

  Once code is uploaded, before powering up ensure the Comm switch is set to 'RF' and
  WiFly module is re-connected
*/

/*
  Function notes

  dtostrf function details
    dtostrf(floatVar, minStringWidthIncDecimalPoint, numVarsAfterDecimal, charBuf);

  itoa function details
    char* itoa (int val, char *buf, int radix)
    where radix is the number base, ie. 10
*/



/*
  Revision history

  1.0 2014/04/15
    Initial development based on Sparkfun WeatherShield v1.4 code
  2.0 2015/01/31
    Rewritten to support MQTT and hopefully improve stability!!
  2.1 2015/02/02
    Modified to improve stability
    based on code from https://github.com/xoseperez/rentalito/blob/master/client/rentalito.ino
  3.0 2015/04/24
    Modified to support SunAirPower measurements
    Moved MQTT topic strings to EEPROM to reduce SRAM usage
*/


#include "debug.h"

#include "config.h"

void publish_measurements()
{
#if USE_STATUS_LED
  digitalWrite(STATUS_LED, HIGH);
#endif

  if (!wiflyConnected)
    wifly_connect();

  if (wiflyConnected) {
    // MQTT client setup
//    mqttClient.disconnect();
    if (mqttClient.connect(mqttClientId)) {

#if USE_STATUS_LED
      digitalWrite(STATUS_LED, LOW);
#endif

      progBuffer[0] = '\0';
      strcpy_P(progBuffer, (char*)pgm_read_word(&(STATUS_TOPICS[0])));
      messBuffer[0] = '\0';
      strcpy_P(messBuffer, (char*)pgm_read_word(&(MQTT_PAYLOADS[0])));
      mqttClient.publish(progBuffer,messBuffer);

      takeMeasurement();

      mqttClient.disconnect();
    }
  }
}

void takeMeasurement(void)
{
#if ENABLE_WDT
  wdt_reset();
#endif

// connect to mqtt server
//  if (!mqttClient.loop())
//  {
//    connect_mqtt();
//  }

  // publish measurement start topic
  progBuffer[0] = '\0';
  strcpy_P(progBuffer, (char*)pgm_read_word(&(MEASUREMENT_TOPICS[12])));
  mqttClient.publish(progBuffer, "");

#if ENABLE_DHT22
  // take measurement as sensor cannot be be sampled at short intervals
  if (dht22_measurement() == DHTLIB_OK) {
    // value is stored in DHT object
    dht22MeasurementOk = true;
  }
#endif

#if ENABLE_TEMP
  temperature_measurement();
#endif
#if ENABLE_HUMIDITY
  humidity_measurement();
#endif
#if ENABLE_PRESSURE
  if ( pressureSensorStatus )
    bmp085_measurement();
#endif
#if ENABLE_LIGHT
  TEMT6000_measurement();
#endif
#if ENABLE_POWER_MONITOR
  sunairplus_measurement();
#endif
#if ENABLE_WEATHER_METERS
//  windspeed_measurement();
//  winddirection_measurement();
//  rainfall_measurement();
  weather_meter_measurement();
#endif

  // publish measurement end topic with message
  // message is number of measurements included

//  buf[0] = '\0';
//  itoa(measurement_count, buf, 10);

  // publish measurement end topic
  progBuffer[0] = '\0';
  strcpy_P(progBuffer, (char*)pgm_read_word(&(MEASUREMENT_TOPICS[13])));
  mqttClient.publish(progBuffer, "");

#if ENABLE_DHT22
  // reset measurements
  dht22MeasurementOk = false;
#endif
}

#if ENABLE_TEMP 
void temperature_measurement()
{
#if ENABLE_SHT15
  TWCR &= ~(_BV(TWEN));  // turn off I2C enable bit so we can access the SHT15 humidity sensor
  float measurement = humiditySensor.readTemperatureC();  // temperature returned in degrees Celcius
  buf[0] = '\0';
  dtostrf(measurement,1,FLOAT_DECIMAL_PLACES, buf);
  progBuffer[0] = '\0';
  strcpy_P(progBuffer, (char*)pgm_read_word(&(MEASUREMENT_TOPICS[0])));
  mqttClient.publish(progBuffer, buf);
#endif
#if ENABLE_DHT22
  if (dht22MeasurementOk) {
    // value is stored in DHT object
    buf[0] = '\0';
    dtostrf(dht.temperature,1,FLOAT_DECIMAL_PLACES, buf);
    progBuffer[0] = '\0';
    strcpy_P(progBuffer, (char*)pgm_read_word(&(MEASUREMENT_TOPICS[1])));
    mqttClient.publish(progBuffer, buf);
  }
#endif
}
#endif

#if ENABLE_HUMIDITY
void humidity_measurement()
{
#if ENABLE_SHT15
  TWCR &= ~(_BV(TWEN));  // turn off I2C enable bit so we can access the SHT15 humidity sensor
  float measurement = humiditySensor.readHumidity();
  buf[0] = '\0';
  dtostrf(measurement,1,FLOAT_DECIMAL_PLACES, buf);
  progBuffer[0] = '\0';
  strcpy_P(progBuffer, (char*)pgm_read_word(&(MEASUREMENT_TOPICS[2])));
  mqttClient.publish(progBuffer, buf);
#endif
#if ENABLE_DHT22
  if (dht22MeasurementOk) {
    // value is stored in DHT object
    buf[0] = '\0';
    dtostrf(dht.humidity,1,FLOAT_DECIMAL_PLACES, buf);
    progBuffer[0] = '\0';
    strcpy_P(progBuffer, (char*)pgm_read_word(&(MEASUREMENT_TOPICS[3])));
    mqttClient.publish(progBuffer, buf);
  }
#endif
}
#endif

#if ENABLE_PRESSURE
void bmp085_measurement()
{
  TWCR |= _BV(TWEN);          // turn on I2C enable bit so we can access the BMP085 pressure sensor

  char   status;
  double bmp085Temp     = 0.0;
  double bmp085Pressure = 0.0;

  // start BMP085 temperature reading
  // tell the sensor to start a temperature measurement
  // if request is successful, the number of ms to wait is returned
  // if request is unsuccessful, 0 is returned
  status = pressureSensor.startTemperature();
  if (status != 0)
  {
    // wait for the measurement to complete
    delay(status);

    progBuffer[0] = '\0';
    strcpy_P(progBuffer, (char*)pgm_read_word(&(MEASUREMENT_TOPICS[4])));

    // retrieve BMP085 temperature reading
    // function returns 1 if successful, 0 if failure
    status = pressureSensor.getTemperature(&bmp085Temp); // temperature returned in degrees Celcius

    if (status != 0) {
      // publish BMP085 temperature measurement
      buf[0] = '\0';
      dtostrf(bmp085Temp,1,FLOAT_DECIMAL_PLACES, buf);
      mqttClient.publish(progBuffer, buf);

      // prepare topic for pressure measurement
      progBuffer[0] = '\0';
      strcpy_P(progBuffer, (char*)pgm_read_word(&(MEASUREMENT_TOPICS[5])));

      // tell the sensor to start a pressure measurement
      // the parameter is the oversampling setting, from 0 to 3 (highest res, longest wait)
      // if request is successful, the number of ms to wait is returned
      // if request is unsuccessful, 0 is returned
      status = pressureSensor.startPressure(3);

      if (status != 0) {
        // wait for the measurement to complete
        delay(status);

        // retrieve the BMP085 pressure reading
        // note that the function requires the previous temperature measurement (T)
        // (if temperature is stable, one temperature measurement can be used for a number of pressure measurements)
        // function returns 1 if successful, 0 if failure
        status = pressureSensor.getPressure(&bmp085Pressure, &bmp085Temp); // mbar, deg C
        if (status != 0 ) {
          // publish BMP085 pressure measurement
          buf[0] = '\0';
          dtostrf(bmp085Pressure,1,FLOAT_DECIMAL_PLACES, buf);
          mqttClient.publish(progBuffer, buf);
        } else {
          messBuffer[0] = '\0';
          strcpy_P(messBuffer, (char*)pgm_read_word(&(BMP085_STATUS_MESSAGES[2])));
          mqttClient.publish(progBuffer, messBuffer);
        }
      } else {
        messBuffer[0] = '\0';
        strcpy_P(messBuffer, (char*)pgm_read_word(&(BMP085_STATUS_MESSAGES[3])));
        mqttClient.publish(progBuffer, messBuffer);
      }
    } else {
      messBuffer[0] = '\0';
      strcpy_P(messBuffer, (char*)pgm_read_word(&(BMP085_STATUS_MESSAGES[4])));
      mqttClient.publish(progBuffer, messBuffer);
    }
  } else {
    messBuffer[0] = '\0';
    strcpy_P(messBuffer, (char*)pgm_read_word(&(BMP085_STATUS_MESSAGES[5])));
    mqttClient.publish(progBuffer, messBuffer);
  }
}
#endif

#if ENABLE_LIGHT
void TEMT6000_measurement()
{
  // get light level
#if ENABLE_EXTERNAL_LIGHT
  // higher reading corresponds to brighter conditions
  int TEMT6000_light_raw = analogRead(LIGHT);
#else
  // lower reading corresponds to brighter conditions
  int TEMT6000_light_raw = 1023 - analogRead(LIGHT);
#endif

  buf[0] = '\0';
  itoa(TEMT6000_light_raw, buf, 10);

  progBuffer[0] = '\0';
  strcpy_P(progBuffer, (char*)pgm_read_word(&(MEASUREMENT_TOPICS[6])));

  mqttClient.publish(progBuffer, buf);

  // convert TEMT6000_light_raw voltage value to percentage
  //map(value, fromLow, fromHigh, toLow, toHigh)
  int TEMT6000_light = map(TEMT6000_light_raw, 0, 1023, 0, 100);

  buf[0] = '\0';
  itoa(TEMT6000_light, buf, 10);

  progBuffer[0] = '\0';
  strcpy_P(progBuffer, (char*)pgm_read_word(&(MEASUREMENT_TOPICS[7])));

  mqttClient.publish(progBuffer, buf);
}
#endif

/*--------------------------------------------------------------------------------------
 setup()
 Called by the Arduino framework once, before the main loop begins
 --------------------------------------------------------------------------------------*/
void setup()
{
#if USE_STATUS_LED
  // Configure status LED
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, LOW);
#endif

  // lots of time for the WiFly to start up
  delay(5000);

  // Configure WiFly
  wifly_configure();

  wifly_connect();

#if USE_STATUS_LED
  digitalWrite(STATUS_LED, HIGH);
#endif

  if (wiflyConnected) {
    if (mqttClient.connect(mqttClientId)) {
#if USE_STATUS_LED
      digitalWrite(STATUS_LED, LOW);
#endif
      progBuffer[0] = '\0';
      strcpy_P(progBuffer, (char*)pgm_read_word(&(STATUS_TOPICS[0])));
      messBuffer[0] = '\0';
      strcpy_P(messBuffer, (char*)pgm_read_word(&(STATUS_TOPICS[0])));
      mqttClient.publish(progBuffer,messBuffer);
    }
  }

  weatherboard_sensors_initialisaton();

#if ENABLE_POWER_MONITOR
  ina3221.begin();
#endif

  if (mqttClient.connected())
    mqttClient.disconnect();

#if ENABLE_WEATHER_METERS
  weatherboard_meters_initialisation();
#endif
}



/*--------------------------------------------------------------------------------------
 loop()
 Arduino main loop
 --------------------------------------------------------------------------------------*/
void loop()
{
  unsigned long currentMillis = millis();

  if (currentMillis - previousMeasurementMillis >= MEASUREMENT_INTERVAL) {
    previousMeasurementMillis = currentMillis;
    publish_measurements();
#if ENABLE_WEATHER_METERS
    windRpmMax = 0.0;    // reset to get strongest gust in each measurement period
#endif
  }

#if ENABLE_WEATHER_METERS && ENABLE_WIND_DIR_AVERAGING
  if (currentMillis - previousWindDirMillis >= WIND_DIR_INTERVAL) {
    previousWindDirMillis = currentMillis;
    wind_dir_avg.addValue(get_wind_direction());
  }
#endif  /* ENABLE_WEATHER_METERS && ENABLE_WIND_DIR_AVERAGING */



  if (wiflyFailedConnections > WIFLY_FAILED_CONNECTIONS_MAX) {
    delay(AFTER_ERROR_DELAY);
    wiflyFailedConnections = 0;
  }

#if ENABLE_WEATHER_METERS
  // handle weather meter interrupts in loop()
  static unsigned long windStopped = 0;

  // an interrupt occurred, handle it now
  if (gotWindSpeed) {
    gotWindSpeed = false;
    windRpm = word(tempWindRpm);
    if (windRpm > windRpmMax) {
      windRpmMax = windRpm;
    }
    windStopped = millis() + ZERODELAY;  // save this timestamp
  }

  // zero wind speed RPM if we don't get a reading in ZERODELAY ms
  if (millis() > windStopped) {
    windRpm = 0;
    windIntCount = 0;
  }
#endif  // ENABLE_WEATHER_METERS

  // require a client.loop in order to receive subscriptions
  //client.loop();
}

/*--------------------------------------------------------------------------------------
 end loop()
 --------------------------------------------------------------------------------------*/
 


