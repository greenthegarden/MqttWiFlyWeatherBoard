#include <Arduino.h>

/*
  Code based on Sparkfun WeatherShield v1.4 source from
  https://github.com/sparkfun/USB_Weather_Board/
  The libaraies included with the Sparkfun source code,
  for the temperature sensor (SHT1x) and pressure sensor  (SFE_BMP085)
  are required to be installed in the Arduino library folder
  in order to compile the code.

  In addition, the Wifly-MQTT library from
  https://github.com/greenthegarden/WiFly
  is used to provide the MQTT interface via WiFly module.
*/

/*
  To compile Set Tools/Board in the Arduino IDE to
  Board: "Arduino Pro or Pro Mini"
  Processor: ATmega328 (3.3V 8MHz)"

  While uploading to code to the Weather Board, remove the WiFly module and
  ensure the Comm switch is set to 'USB'

  Once code is uploaded, before powering up ensure the Comm switch is set to
  'RF' and
  WiFly module is re-connected
*/

/*
  Function notes

  dtostrf function details
    dtostrf(floatVar, minStringWidthIncDecimalPoint, numVarsAfterDecimal,
  charBuf);

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
    based on code from
  https://github.com/xoseperez/rentalito/blob/master/client/rentalito.ino
  3.0 2015/04/24
    Modified to support SunAirPower measurements
    Moved MQTT topic strings to EEPROM to reduce SRAM usage
  4.0 2016/04/19
    Restructured code to improve maintence and align with more recent projects.
  5.0 2016/05/15
    Added capability to 'sleep' WiFLy module
  5.1 2016/08/10
    Fixed reliability of sleep
*/

#include "debug.h"

#include "config.h"

void publish_measurements(void) {
  publish_sht15_measurements();
  if (pressureSensorStatus) {
    publish_bmp085_measurements();
  }
  publish_temt6000_measurement();
#if ENABLE_WEATHER_METERS
  publish_weather_meter_measurement();
#endif
#if ENABLE_DHT22
  publish_dht22_measurements();
#endif
#if ENABLE_POWER_MONITOR
  publish_sunairplus_measurement();
#endif
}

byte publish_report() {
#if USE_STATUS_LED
  digitalWrite(STATUS_LED, HIGH);
#endif

  if (!wiflyConnectedToNetwork) {
    wifly_connect_to_network();
  }

  if (wiflyConnectedToNetwork) {
    if (mqttClient.connect(mqttClientId)) {

#if USE_STATUS_LED
      digitalWrite(STATUS_LED, LOW);
#endif

      messBuffer[0] = '\0';
      strcpy_P(messBuffer, (char *)pgm_read_word(&(MQTT_PAYLOADS[0])));
      progBuffer[0] = '\0';
      strcpy_P(progBuffer, (char *)pgm_read_word(&(STATUS_TOPICS[0])));
      mqttClient.publish(progBuffer, messBuffer);

      // publish report start topic
      messBuffer[0] = '\0';
      strcpy_P(messBuffer, (char *)pgm_read_word(&(MQTT_PAYLOADS[2])));
      progBuffer[0] = '\0';
      strcpy_P(progBuffer, (char *)pgm_read_word(&(STATUS_TOPICS[3])));
      mqttClient.publish(progBuffer, messBuffer);

      publish_measurements();

      // publish report end topic
      messBuffer[0] = '\0';
      strcpy_P(messBuffer, (char *)pgm_read_word(&(MQTT_PAYLOADS[3])));
      progBuffer[0] = '\0';
      strcpy_P(progBuffer, (char *)pgm_read_word(&(STATUS_TOPICS[3])));
      mqttClient.publish(progBuffer, messBuffer);

      mqttClient.disconnect(); // should stop tcp connection

      return 1;
    }
  }
  return 0;
}

void reset_cummulative_measurements() {
#if ENABLE_WEATHER_METERS
  windRpmMax = 0.0; // reset to get strongest gust in each measurement period
#endif
}

/*--------------------------------------------------------------------------------------
  setup()
  Called by the Arduino framework once, before the main loop begins
  --------------------------------------------------------------------------------------*/
void setup() {
#if DEBUG_LEVEL > 0
  Serial.begin(BAUD_RATE); // Start hardware serial interface for debugging
#endif

#if USE_STATUS_LED
  // Configure status LED
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, LOW);
#endif

  rfpins_init(); // configure rf pins on wwatherboard as inputs

  delay(2000); // use a delay to get things settled before configuring WiFly

  // Configure WiFly
  DEBUG_LOG(1, "WiFly");

  wifly_init();

  wifly_connect_to_network();

#if USE_STATUS_LED
  digitalWrite(STATUS_LED, HIGH);
#endif

  if (wiflyConnectedToNetwork) {
    if (mqttClient.connect(mqttClientId)) {
#if USE_STATUS_LED
      digitalWrite(STATUS_LED, LOW);
#endif
      messBuffer[0] = '\0';
      strcpy_P(messBuffer, (char *)pgm_read_word(&(MQTT_PAYLOADS[0])));
      progBuffer[0] = '\0';
      strcpy_P(progBuffer, (char *)pgm_read_word(&(STATUS_TOPICS[0])));
      mqttClient.publish(progBuffer, messBuffer);
    }
  }

  weatherboard_sensors_init();

#if ENABLE_WEATHER_METERS
  weatherboard_meters_init();

  // turn on interrupts
  interrupts();
#endif

#if ENABLE_POWER_MONITOR
  ina3221.begin();
#endif

  if (wiflyConnectedToNetwork) {
    if (mqttClient.connected())
      mqttClient.disconnect();
  }

#if USE_WIFLY_SLEEP
  wifly_sleep();
#endif
}
/*--------------------------------------------------------------------------------------
  end setup()
  --------------------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------------------
  loop()
  Arduino main loop
  --------------------------------------------------------------------------------------*/
void loop() {
  unsigned long currentMillis = millis();

#if USE_WIFLY_SLEEP
  // use WiFly timer to publish reports
  // WiFly wake monitor
  // When the WiFly wakes up, the RTS pin goes high. Once the module is ready,
  // the the RTS pin is driven low.
  if (wiflySleep) {
    // look for RTS pin high
    if (digitalRead(RF_RTS) == HIGH) {
      wiflyAwake = true;
      wiflySleep = false;
    }
  }
  if (wiflyAwake) {
    if (digitalRead(RF_RTS) == LOW) {
      // WiFly now ready
      wifly_after_wake();
      publish_report();
      reset_cummulative_measurements();
      wifly_sleep();
    }
  }
#else
  // use arduino timer to publish reports
  if (currentMillis - previousMeasurementMillis >= MEASUREMENT_INTERVAL) {
    previousMeasurementMillis = currentMillis;
    publish_report();
    reset_cummulative_measurements();
  }
#endif /* USE_WIFLY_SLEEP */

#if ENABLE_WEATHER_METERS && ENABLE_WIND_MEASUREMENT_AVERAGING
  if (currentMillis - previousWindMeasurementMillis >=
      WIND_MEASUREMENT_INTERVAL) {
    previousWindMeasurementMillis = currentMillis;
    wind_spd_avg.addValue(windRpm);
    wind_dir_avg.addValue(get_wind_direction());
  }
#endif /* ENABLE_WEATHER_METERS && ENABLE_WIND_DIR_AVERAGING */

#if ENABLE_WEATHER_METERS
  // handle weather meter interrupts
  static unsigned long windStopped = 0UL;

  // an interrupt occurred, handle it now
  if (gotWindSpeed) {
    gotWindSpeed = false;
    windRpm = word(tempWindRpm);
    if (windRpm > windRpmMax) {
      windRpmMax = windRpm;
    }
    windStopped = millis() + ZERODELAY; // save this timestamp
  }

  // zero wind speed RPM if a reading does not occur in ZERODELAY ms
  if (millis() > windStopped) {
    windRpm = 0;
    windIntCount = 0;
  }
#endif /* ENABLE_WEATHER_METERS */
}
/*--------------------------------------------------------------------------------------
  end loop()
  --------------------------------------------------------------------------------------*/
