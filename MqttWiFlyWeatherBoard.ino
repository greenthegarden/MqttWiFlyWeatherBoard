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
  Board: "Arduino Pro or Pro Mini"
  Processor: ATmega328 (3.3V 8MHz)"

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
  4.0 2016/04/19
    Restructured code to improve maintence and align with more recent projects.
  5.0 2016/05/15
    Added capability to 'sleep' WiFLy module 
*/


#include "debug.h"

#include "config.h"

void publish_measurements(void)
{
  publish_sht15_measurements();
  if (pressureSensorStatus) { publish_bmp085_measurements(); }
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
  wdt_reset();  // reset watchdog timer
}


byte publish_report()
{
#if USE_STATUS_LED
  digitalWrite(STATUS_LED, HIGH);
#endif
  
  if (!wiflyConnected) {
    wifly_connect();
  }

  if (wiflyConnected) {
    // MQTT client setup
//    mqttClient.disconnect();
    if (mqttClient.connect(mqttClientId)) {

#if USE_STATUS_LED
      digitalWrite(STATUS_LED, LOW);
#endif

      messBuffer[0] = '\0';
      strcpy_P(messBuffer, (char*)pgm_read_word(&(MQTT_PAYLOADS[0])));
      progBuffer[0] = '\0';
      strcpy_P(progBuffer, (char*)pgm_read_word(&(STATUS_TOPICS[0])));
      mqttClient.publish(progBuffer, messBuffer);

      // publish report start topic
      messBuffer[0] = '\0';
      strcpy_P(messBuffer, (char*)pgm_read_word(&(MQTT_PAYLOADS[2])));
      progBuffer[0] = '\0';
      strcpy_P(progBuffer, (char*)pgm_read_word(&(MEASUREMENT_TOPICS[12])));
      mqttClient.publish(progBuffer, messBuffer);

      publish_measurements();

      // publish report end topic
      messBuffer[0] = '\0';
      strcpy_P(messBuffer, (char*)pgm_read_word(&(MQTT_PAYLOADS[3])));
      progBuffer[0] = '\0';
      strcpy_P(progBuffer, (char*)pgm_read_word(&(MEASUREMENT_TOPICS[12])));
      mqttClient.publish(progBuffer, messBuffer);

      mqttClient.disconnect();  // should stop tcp connection

      wdt_reset();  // reset watchdog timer

      return 1;
    } else {
      return 0;
    }
  }
}

void reset_cummulative_measurements()
{
#if ENABLE_WEATHER_METERS
  windRpmMax = 0.0;    // reset to get strongest gust in each measurement period
#endif
}


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

  // Configure WiFly
  DEBUG_LOG(1, "configuring WiFly ...");
  wifly_configure();

  DEBUG_LOG(1, "connecting WiFly ...");
  wifly_connect();

#if USE_STATUS_LED
  digitalWrite(STATUS_LED, HIGH);
#endif

  if (wiflyConnected) {
    if (mqttClient.connect(mqttClientId)) {
#if USE_STATUS_LED
      digitalWrite(STATUS_LED, LOW);
#endif
      messBuffer[0] = '\0';
      strcpy_P(messBuffer, (char*)pgm_read_word(&(MQTT_PAYLOADS[0])));
      progBuffer[0] = '\0';
      strcpy_P(progBuffer, (char*)pgm_read_word(&(STATUS_TOPICS[0])));
      mqttClient.publish(progBuffer,messBuffer);
    }
  }

  weatherboard_sensors_initialisaton();

#if ENABLE_WEATHER_METERS
  weatherboard_meters_initialisation();

  // turn on interrupts
  interrupts();
#endif

#if ENABLE_POWER_MONITOR
  ina3221.begin();
#endif

  wifly_configure_sleep();

  if (wiflyConnected) {
    if (mqttClient.connected())
      mqttClient.disconnect();
  }

  wifly_sleep();
  
  wdt_enable(WDTO_8S);    // enable watchdog timer
}
/*--------------------------------------------------------------------------------------
 end setup()
 --------------------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------------------
 loop()
 Arduino main loop
 --------------------------------------------------------------------------------------*/
void loop()
{  
  unsigned long currentMillis = millis();

  if (currentMillis - previousMeasurementMillis >= MEASUREMENT_INTERVAL) {
    previousMeasurementMillis = currentMillis;
    publish_report();
    wifly_sleep();
    reset_cummulative_measurements();
  }

#if ENABLE_WEATHER_METERS && ENABLE_WIND_MEASUREMENT_AVERAGING
  if (currentMillis - previousWindMeasurementMillis >= WIND_MEASUREMENT_INTERVAL) {
    previousWindMeasurementMillis = currentMillis;
    wind_spd_avg.addValue(windRpm);
    wind_dir_avg.addValue(get_wind_direction());
  }
#endif  /* ENABLE_WEATHER_METERS && ENABLE_WIND_DIR_AVERAGING */

#if ENABLE_WEATHER_METERS
  // handle weather meter interrupts
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

  // zero wind speed RPM if a reading does not occur in ZERODELAY ms
  if (millis() > windStopped) {
    windRpm = 0;
    windIntCount = 0;
  }
#endif  // ENABLE_WEATHER_METERS

  wdt_reset();  // reset watchdog timer
}
/*--------------------------------------------------------------------------------------
 end loop()
 --------------------------------------------------------------------------------------*/
 


