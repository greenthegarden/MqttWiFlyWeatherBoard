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
  WiFly module attributes
    These values are specific to the modules I am using,
    and configuration of my router.

  RN-XV WiFly Module - Wire Antenna
    MAC: 00:06:66:50:71:6f
    IP:  192.168.1.52

  RN-XV WiFly Module – SMA
    MAC: 00:06:66:71:68:d5
    IP:  192.168.1.51
*/

/*
  WiFly status based on LED

  Sources:
    http://www.instructables.com/id/WiFly-RN-XV-Module-Wireless-Arduino-Board-Tutorial/
    http://cairohackerspace.blogspot.com.au/2011/05/beginners-guide-to-connecting-and.html

  Green LED:
    Solid:         Connected through TCP
    Slow Blinking: IP address is assigned
    Fast Blinking: No IP address assigned
    None/Off:      NA

  Yellow LED
    Solid:         NA
    Slow Blinking: NA
    Fast Blinking: RX/TX Data Transfer
    None/Off:      No network activity

  Red LED
    Solid:         NA
    Slow Blinking: Associated, No internet detected
    Fast Blinking: Not associated
    None/Off:      Associated, Internet detected
*/

/*
  WiFly configuration

  The following is the sequence of commands I use to
  configure the WiFly module used when running this code.
  (ensure values for ssid and phrase entered in place of xxx)

reboot
$$$
factory RESET

set wlan join 0    // Stop device connecting while we setup

set ip dhcp 3
set wlan ssid xxx
set wlan phrase xxx
set wlan join 1

save
reboot
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


// WiFly libraries
#include <SPI.h>
#include <WiFly.h>
#include <PubSubClient.h>

#include "debug.h"

#include "config.h"


#if ENABLE_WDT
#include <avr/wdt.h>  // required for AVR watchdog timer
#endif


// external sensor libraries
#if ENABLE_SHT15
#include <SHT1x.h>              // SHT15 humidity sensor library
#endif
#if ENABLE_DHT22
#include <dht.h>                // DHT22 temperature/humidty sensor library
#endif
#if ENABLE_BMP085
#include <Wire.h>               // I2C library (necessary for pressure sensor)
#include <SFE_BMP085.h>         // BMP085 pressure sensor library
#endif
#if ENABLE_POWER_MONITOR
#include <Wire.h>
#include <SDL_Arduino_INA3221.h>
#endif

// character buffer to support conversion of floats to char
char buf[12];

// global variable definitions
unsigned long previousMeasurementMillis = 0;
unsigned long previousWindDirMillis     = 0;
boolean  pressureSensorStatus         = false;
#if ENABLE_WEATHER_METERS
unsigned int windRpm                    = 0;
unsigned int windRpmMax                = 0;
unsigned int windStopped                    = 0;
// volatiles are subject to modification by IRQs
volatile unsigned long tempWindRpm      = 0, windTime = 0, windLast = 0, windInterval = 0;
volatile unsigned char windIntCount;
volatile boolean       gotWindSpeed;
volatile unsigned long rainTime         = 0, rainLast = 0, rainInterval = 0, rain = 0;
#endif


// initialisation of sensor objects
#if ENABLE_SHT15
SHT1x humidity_sensor(SHT1x_DATA, SHT1x_CLOCK);
#endif
#if ENABLE_DHT22
dht DHT;
#endif
#if ENABLE_BMP085
SFE_BMP085 pressure_sensor(BMP_ADDR);
#endif
#if ENABLE_POWER_MONITOR
SDL_Arduino_INA3221 ina3221;
#endif

// function declarations
void take_measurement();
#if ENABLE_WEATHER_METERS
float get_wind_direction();
// interrupt routines (these are called by the hardware interrupts, not by the main code)
void rain_irq();
float get_wind_direction();
#if ENABLE_WIND_DIR_AVERAGING
#include "RunningAverage.h"
RunningAverage wind_dir_avg(WIND_DIR_AVERAGING_SIZE);
#endif
#endif


// WiFly setup and connection routines

const byte WIFLY_FAILED_CONNECTIONS_MAX = 2;	// reset wifly after this many failed connections
byte       wiflyFailedConnections       = 0;

void wifly_connect()
{
#if ENABLE_WDT
  wdt_reset();
#endif

#if USE_STATUS_LED
  digitalWrite(STATUS_LED, HIGH);
#endif

  WiFly.begin();

  if (!WiFly.join(SSID, PASSPHRASE, mode)) {
    wiflyConnected = false;
    wiflyFailedConnections++;
#if ENABLE_WDT
    wdt_reset();
#endif
  } else {
    wiflyConnected = true;
    wiflyFailedConnections = 0;
#if USE_STATUS_LED
    digitalWrite(STATUS_LED, LOW);
#endif
  }
}

void wifly_configure()
{
  // Configure WiFly
  Serial.begin(BAUD_RATE);      // Start hardware Serial for the RN-XV
  WiFly.setUart(&Serial);       // Tell the WiFly library that we are not using the SPIUart

//  WiFly.begin();

//  wifly_connect();
}


// MQTT related routines

// callback definition for MQTT
void callback(char* topic, uint8_t* payload, unsigned int length)
{
  // nothing to do here!!
}

WiFlyClient wiflyClient;
PubSubClient mqttClient(mqttServerAddr, MQTT_PORT, callback, wiflyClient);

// dht22 measurement routine
byte dht22_measurement()
{
  int chk = DHT.read22(DHT22_PIN);

  progBuffer[0] = '\0';
  strcpy_P(progBuffer, (char*)pgm_read_word(&(STATUS_TOPICS[5])));

  switch (chk) {
    case DHTLIB_OK :
      DEBUG_LOG(1, "OK");
      messBuffer[0] = '\0';
      strcpy_P(messBuffer, (char*)pgm_read_word(&(dht22_status_messages[0])));
      mqttClient.publish(progBuffer,messBuffer);
      break;
    case DHTLIB_ERROR_CHECKSUM :
      DEBUG_LOG(1, "Checksum error");
      messBuffer[0] = '\0';
      strcpy_P(messBuffer, (char*)pgm_read_word(&(dht22_status_messages[1])));
      mqttClient.publish(progBuffer,messBuffer);
      break;
    case DHTLIB_ERROR_TIMEOUT :
      DEBUG_LOG(1, "Time out error");
      messBuffer[0] = '\0';
      strcpy_P(messBuffer, (char*)pgm_read_word(&(dht22_status_messages[2])));
      mqttClient.publish(progBuffer,messBuffer);
      break;
    case DHTLIB_ERROR_CONNECT :
      DEBUG_LOG(1, "Connect error");
      messBuffer[0] = '\0';
      strcpy_P(messBuffer, (char*)pgm_read_word(&(dht22_status_messages[3])));
      mqttClient.publish(progBuffer,messBuffer);
      break;
    case DHTLIB_ERROR_ACK_L :
      DEBUG_LOG(1, "Ack Low error");
      messBuffer[0] = '\0';
      strcpy_P(messBuffer, (char*)pgm_read_word(&(dht22_status_messages[4])));
      mqttClient.publish(progBuffer,messBuffer);
      break;
    case DHTLIB_ERROR_ACK_H :
      DEBUG_LOG(1, "Ack High error");
      messBuffer[0] = '\0';
      strcpy_P(messBuffer, (char*)pgm_read_word(&(dht22_status_messages[5])));
      mqttClient.publish(progBuffer,messBuffer);
      break;
    default :
      DEBUG_LOG(1, "Unknown error");
      messBuffer[0] = '\0';
      strcpy_P(messBuffer, (char*)pgm_read_word(&(dht22_status_messages[6])));
      mqttClient.publish(progBuffer,messBuffer);
      break;
  }
  return chk;
}

void publish_measurements()
{
#if ENABLE_WDT
  wdt_reset();
#endif

#if USE_STATUS_LED
  digitalWrite(STATUS_LED, HIGH);
#endif

  if (!wiflyConnected)
    wifly_connect();

#if ENABLE_WDT
  wdt_reset();
#endif

  if (wiflyConnected) {
    // MQTT client setup
//    mqttClient.disconnect();
    if (mqttClient.connect(mqttClientId)) {
#if ENABLE_WDT
      wdt_reset();
#endif

#if USE_STATUS_LED
      digitalWrite(STATUS_LED, LOW);
#endif

      progBuffer[0] = '\0';
      strcpy_P(progBuffer, (char*)pgm_read_word(&(STATUS_TOPICS[0])));
      messBuffer[0] = '\0';
      strcpy_P(messBuffer, (char*)pgm_read_word(&(mqtt_status_messages[0])));
      mqttClient.publish(progBuffer,messBuffer);

      takeMeasurement();

      mqttClient.disconnect();
    }
  }
}


/*--------------------------------------------------------------------------------------
 setup()
 Called by the Arduino framework once, before the main loop begins
 --------------------------------------------------------------------------------------*/
void setup()
{
#if ENABLE_WDT
  wdt_disable();
#endif

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

  // Configure sensors
#if ENABLE_BMP085
  // set up inputs and outputs
  pinMode(XCLR, OUTPUT);                // output to BMP085 reset (unused)
  digitalWrite(XCLR, HIGH);             // make pin high to turn off reset

  pinMode(EOC, INPUT);                  // input from BMP085 end of conversion (unused)
  digitalWrite(EOC, LOW);               // turn off pullup

  // Reset the humidity sensor connection so that the I2C bus can be accessed
  TWCR &= ~(_BV(TWEN));                // turn off I2C enable bit so we can access the SHT15 humidity sensor
  digitalWrite(XCLR, LOW);              // disable the BMP085 while resetting humidity sensor
  humidity_sensor.connectionReset();   // reset the humidity sensor connection
  TWCR |= _BV(TWEN);                   // turn on I2C enable bit so we can access the BMP085 pressure sensor
  digitalWrite(XCLR, HIGH);             // enable BMP085
  delay(10);                           // wait for the BMP085 pressure sensor to become ready after reset

  progBuffer[0] = '\0';
  strcpy_P(progBuffer, (char*)pgm_read_word(&(STATUS_TOPICS[6])));
  if (pressure_sensor.begin()) {        // initialize the BMP085 pressure sensor (important to get calibration values stored on the device)
    pressureSensorStatus = true;
    if (mqttClient.connected()) {
      messBuffer[0] = '\0';
      strcpy_P(messBuffer, (char*)pgm_read_word(&(bmp085_status_messages[0])));
      mqttClient.publish(progBuffer, messBuffer);
    }
  } else {
    if (mqttClient.connected()) {
      messBuffer[0] = '\0';
      strcpy_P(messBuffer, (char*)pgm_read_word(&(bmp085_status_messages[1])));
      mqttClient.publish(progBuffer, messBuffer);
    }
  }
#endif

#if ENABLE_POWER_MONITOR
  ina3221.begin();
#endif

  if (mqttClient.connected())
    mqttClient.disconnect();

#if ENABLE_WEATHER_METERS
  pinMode(WSPEED,INPUT);               // input from wind meters windspeed sensor
  digitalWrite(WSPEED,HIGH);           // turn on pullup

  pinMode(RAIN,INPUT);                 // input from wind meters rain gauge sensor
  digitalWrite(RAIN,HIGH);             // turn on pullup

  // init wind speed interrupt global variables
  gotWindSpeed       = false;
  windRpm         = 0;
  windIntCount    = 0;

#if ENABLE_WIND_DIR_AVERAGING
  wind_dir_avg.clear(); // explicitly start clean
#endif

  // attach external interrupt pins to IRQ functions
  attachInterrupt(0, rain_irq,   FALLING);
  attachInterrupt(1, wind_speed_irq, FALLING);

  // turn on interrupts
  interrupts();
#endif

#if ENABLE_WDT
  wdt_enable(WDTO_8S); // Watchdog timer set for eight seconds
#endif
}



/*--------------------------------------------------------------------------------------
 loop()
 Arduino main loop
 --------------------------------------------------------------------------------------*/
void loop()
{
#if ENABLE_WDT
  wdt_reset();
#endif

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
  strcpy_P(progBuffer, (char*)pgm_read_word(&(measurment_topics[12])));
  mqttClient.publish(progBuffer, "");

#if ENABLE_DHT22
  // take measurement as sensor cannot be be sampled at short intervals
  if (dht22_measurement() == DHTLIB_OK) {
    // value is stored in DHT object
    dht22_measurement_ok = true;
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
    BMP085_measurement();
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
  strcpy_P(progBuffer, (char*)pgm_read_word(&(measurment_topics[13])));
  mqttClient.publish(progBuffer, "");

#if ENABLE_DHT22
  // reset measurements
  dht22_measurement_ok = false;
#endif
}

#if ENABLE_TEMP 
void temperature_measurement()
{
#if ENABLE_SHT15
  TWCR &= ~(_BV(TWEN));  // turn off I2C enable bit so we can access the SHT15 humidity sensor
  float SHT15_temp = humidity_sensor.readTemperatureC();  // temperature returned in degrees Celcius
  buf[0] = '\0';
  dtostrf(SHT15_temp,1,FLOAT_DECIMAL_PLACES, buf);
  progBuffer[0] = '\0';
  strcpy_P(progBuffer, (char*)pgm_read_word(&(measurment_topics[0])));
  mqttClient.publish(progBuffer, buf);
#endif
#if ENABLE_DHT22
  if (dht22_measurement_ok) {
    // value is stored in DHT object
    buf[0] = '\0';
    dtostrf(DHT.temperature,1,FLOAT_DECIMAL_PLACES, buf);
    progBuffer[0] = '\0';
    strcpy_P(progBuffer, (char*)pgm_read_word(&(measurment_topics[1])));
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
  float SHT15_humidity = humidity_sensor.readHumidity();
  buf[0] = '\0';
  dtostrf(SHT15_humidity,1,FLOAT_DECIMAL_PLACES, buf);
  progBuffer[0] = '\0';
  strcpy_P(progBuffer, (char*)pgm_read_word(&(measurment_topics[2])));
  mqttClient.publish(progBuffer, buf);
#endif
#if ENABLE_DHT22
  if (dht22_measurement_ok) {
    // value is stored in DHT object
    buf[0] = '\0';
    dtostrf(DHT.humidity,1,FLOAT_DECIMAL_PLACES, buf);
    progBuffer[0] = '\0';
    strcpy_P(progBuffer, (char*)pgm_read_word(&(measurment_topics[3])));
    mqttClient.publish(progBuffer, buf);
  }
#endif
}
#endif

#if ENABLE_PRESSURE
void BMP085_measurement()
{
  TWCR |= _BV(TWEN);          // turn on I2C enable bit so we can access the BMP085 pressure sensor

  char   status;
  double BMP085_temp     = 0.0;
  double BMP085_pressure = 0.0;

  // start BMP085 temperature reading
  // tell the sensor to start a temperature measurement
  // if request is successful, the number of ms to wait is returned
  // if request is unsuccessful, 0 is returned
  status = pressure_sensor.startTemperature();
  if (status != 0)
  {
    // wait for the measurement to complete
    delay(status);

    progBuffer[0] = '\0';
    strcpy_P(progBuffer, (char*)pgm_read_word(&(measurment_topics[4])));

    // retrieve BMP085 temperature reading
    // function returns 1 if successful, 0 if failure
    status = pressure_sensor.getTemperature(&BMP085_temp); // temperature returned in degrees Celcius

    if (status != 0) {
      // publish BMP085 temperature measurement
      buf[0] = '\0';
      dtostrf(BMP085_temp,1,FLOAT_DECIMAL_PLACES, buf);

      mqttClient.publish(progBuffer, buf);

      // prepare topic for pressure measurement
      progBuffer[0] = '\0';
      strcpy_P(progBuffer, (char*)pgm_read_word(&(measurment_topics[5])));

      // tell the sensor to start a pressure measurement
      // the parameter is the oversampling setting, from 0 to 3 (highest res, longest wait)
      // if request is successful, the number of ms to wait is returned
      // if request is unsuccessful, 0 is returned
      status = pressure_sensor.startPressure(3);

      if (status != 0) {
        // wait for the measurement to complete
        delay(status);

        // retrieve the BMP085 pressure reading
        // note that the function requires the previous temperature measurement (T)
        // (if temperature is stable, one temperature measurement can be used for a number of pressure measurements)
        // function returns 1 if successful, 0 if failure
        status = pressure_sensor.getPressure(&BMP085_pressure, &BMP085_temp); // mbar, deg C
        if (status != 0 ) {
          // publish BMP085 pressure measurement
          buf[0] = '\0';
          dtostrf(BMP085_pressure,1,FLOAT_DECIMAL_PLACES, buf);

          mqttClient.publish(progBuffer, buf);
        } else {
          messBuffer[0] = '\0';
          strcpy_P(messBuffer, (char*)pgm_read_word(&(bmp085_status_messages[2])));
          mqttClient.publish(progBuffer, messBuffer);
        }
      } else {
        messBuffer[0] = '\0';
        strcpy_P(messBuffer, (char*)pgm_read_word(&(bmp085_status_messages[3])));
        mqttClient.publish(progBuffer, messBuffer);
      }
    } else {
      messBuffer[0] = '\0';
      strcpy_P(messBuffer, (char*)pgm_read_word(&(bmp085_status_messages[4])));
      mqttClient.publish(progBuffer, messBuffer);
    }
  } else {
    messBuffer[0] = '\0';
    strcpy_P(messBuffer, (char*)pgm_read_word(&(bmp085_status_messages[5])));
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
  strcpy_P(progBuffer, (char*)pgm_read_word(&(measurment_topics[6])));

  mqttClient.publish(progBuffer, buf);

  // convert TEMT6000_light_raw voltage value to percentage
  //map(value, fromLow, fromHigh, toLow, toHigh)
  int TEMT6000_light = map(TEMT6000_light_raw, 0, 1023, 0, 100);

  buf[0] = '\0';
  itoa(TEMT6000_light, buf, 10);

  progBuffer[0] = '\0';
  strcpy_P(progBuffer, (char*)pgm_read_word(&(measurment_topics[7])));

  mqttClient.publish(progBuffer, buf);
}
#endif

#if ENABLE_WEATHER_METERS
void weather_meter_measurement()
{
  // take wind-direction measurement first
  // if returns -1 then treat as sensors not connected
  if (winddirection_measurement()) {
    windspeed_measurement();
    rainfall_measurement();
  } else {
    progBuffer[0] = '\0';
    strcpy_P(progBuffer, (char*)pgm_read_word(&(STATUS_TOPICS[1])));
    messBuffer[0] = '\0';
    strcpy_P(messBuffer, (char*)pgm_read_word(&(weather_meter_messages[0])));
    mqttClient.publish(progBuffer, messBuffer);
  }
}

byte winddirection_measurement()
{
  float WM_wdirection = -1.0;
#if ENABLE_WIND_DIR_AVERAGING
  WM_wdirection = wind_dir_avg.getAverage();
#else
  // use instantaneous wind direction
  WM_wdirection = get_wind_direction();  // should return a -1 if disconnected
#endif
  if (WM_wdirection >= 0) {
    buf[0] = '\0';
    dtostrf(WM_wdirection,1,FLOAT_DECIMAL_PLACES, buf);
    progBuffer[0] = '\0';
    strcpy_P(progBuffer, (char*)pgm_read_word(&(measurment_topics[8])));
    mqttClient.publish(progBuffer, buf);
    return 1;
  } else {
    return 0;
  }
}

void windspeed_measurement()
{
  float windSpeedMeasurement = 0.0;

  // publish instantaneous wind speed  
  windSpeedMeasurement = float(windRpm) / WIND_RPM_TO_KNOTS;
  buf[0] = '\0';
  dtostrf(windSpeedMeasurement,1,FLOAT_DECIMAL_PLACES, buf);
  progBuffer[0] = '\0';
  strcpy_P(progBuffer, (char*)pgm_read_word(&(measurment_topics[9])));
  mqttClient.publish(progBuffer, buf);

  // publish maximum wind speed since last report
  windSpeedMeasurement = float(windRpmMax) / WIND_RPM_TO_KNOTS;
  buf[0] = '\0';
  dtostrf(windSpeedMeasurement,1,FLOAT_DECIMAL_PLACES, buf);
  progBuffer[0] = '\0';
  strcpy_P(progBuffer, (char*)pgm_read_word(&(measurment_topics[10])));
  mqttClient.publish(progBuffer, buf);
}

void rainfall_measurement()
{
  // rainfall unit conversion
  float rainfallMeasurement = rain * RAIN_BUCKETS_TO_MM;
  
  buf[0] = '\0';
  dtostrf(rainfallMeasurement,1,FLOAT_DECIMAL_PLACES, buf);
  progBuffer[0] = '\0';
  strcpy_P(progBuffer, (char*)pgm_read_word(&(measurment_topics[11])));
  mqttClient.publish(progBuffer, buf);

  // reset value of rain to zero
  rain = 0;
}
#endif

#if ENABLE_POWER_MONITOR
void sunairplus_measurement()
{
  float measurement = 0.0;

  //LIPO battery measurements

  measurement = ina3221.getBusVoltage_V(LIPO_BATTERY_CHANNEL);
  buf[0] = '\0';
  dtostrf(measurement,1,FLOAT_DECIMAL_PLACES, buf);
  progBuffer[0] = '\0';
  strcpy_P(progBuffer, (char*)pgm_read_word(&(sunairplus_topics[0])));
  mqttClient.publish(progBuffer, buf);
  
  measurement = ina3221.getCurrent_mA(LIPO_BATTERY_CHANNEL);
  buf[0] = '\0';
  dtostrf(measurement,1,FLOAT_DECIMAL_PLACES, buf);
  progBuffer[0] = '\0';
  strcpy_P(progBuffer, (char*)pgm_read_word(&(sunairplus_topics[1])));
  mqttClient.publish(progBuffer, buf);

  // Solar cell measurements
  
  measurement = ina3221.getBusVoltage_V(SOLAR_CELL_CHANNEL);
  buf[0] = '\0';
  dtostrf(measurement,1,FLOAT_DECIMAL_PLACES, buf);
  progBuffer[0] = '\0';
  strcpy_P(progBuffer, (char*)pgm_read_word(&(sunairplus_topics[2])));
  mqttClient.publish(progBuffer, buf);

  measurement = ina3221.getCurrent_mA(SOLAR_CELL_CHANNEL);
  buf[0] = '\0';
  dtostrf(measurement,1,FLOAT_DECIMAL_PLACES, buf);
  progBuffer[0] = '\0';
  strcpy_P(progBuffer, (char*)pgm_read_word(&(sunairplus_topics[3])));
  mqttClient.publish(progBuffer, buf);

  // SunAirPlus output measurements

  measurement = ina3221.getBusVoltage_V(OUTPUT_CHANNEL);
  buf[0] = '\0';
  dtostrf(measurement,1,FLOAT_DECIMAL_PLACES, buf);
  progBuffer[0] = '\0';
  strcpy_P(progBuffer, (char*)pgm_read_word(&(sunairplus_topics[4])));
  mqttClient.publish(progBuffer, buf);

  measurement = ina3221.getCurrent_mA(OUTPUT_CHANNEL);
  buf[0] = '\0';
  dtostrf(measurement,1,FLOAT_DECIMAL_PLACES, buf);
  progBuffer[0] = '\0';
  strcpy_P(progBuffer, (char*)pgm_read_word(&(sunairplus_topics[5])));
  mqttClient.publish(progBuffer, buf);
}
#endif


#if ENABLE_WEATHER_METERS
void rain_irq()
// if the Weather Meters are attached, count rain gauge bucket tips as they occur
// activated by the magnet and reed switch in the rain gauge, attached to input D2
{
  rainTime     = micros();              // grab current time
  rainInterval = rainTime - rainLast;   // calculate interval between this and last event

  if (rainInterval > 100) {
    // ignore switch-bounce glitches less than 100uS after initial edge
    rain++;                             // increment bucket counter
    rainLast = rainTime;                // set up for next event
  }
}

void wind_speed_irq()
// if the Weather Meters are attached, measure anemometer RPM (2 ticks per rotation), set flag if RPM is updated
// activated by the magnet in the anemometer (2 ticks per rotation), attached to input D3

// this routine measures RPM by measuring the time between anemometer pulses
// windintcount is the number of pulses we've measured - we need two to measure one full rotation (eliminates any bias between the position of the two magnets)
// when windintcount is 2, we can calculate the RPM based on the total time from when we got the first pulse
// note that this routine still needs an outside mechanism to zero the RPM if the anemometer is stopped (no pulses occur within a given period of time)
{
  windTime = micros(); // grab current time
  if ((windIntCount == 0) || ((windTime - windLast) > 10000))  { 
    // ignore switch-bounce glitches less than 10ms after the reed switch closes
    if (windIntCount == 0) {
      // if we're starting a new measurement, reset the interval
      windInterval = 0;
    } else {
      // otherwise, add current interval to the interval timer
      windInterval += (windTime - windLast);
    }
    if (windIntCount == 2) {
      // we have two measurements (one full rotation), so calculate result and start a new measurement
      tempWindRpm = (60000000UL / windInterval); // calculate RPM (temporary since it may change unexpectedly)
      windIntCount = 0;
      windInterval = 0;
      gotWindSpeed = true; // set flag for main loop
    }

    windIntCount++;
    windLast = windTime; // save the current time so that we can calculate the interval between now and the next interrupt
  }
}

float get_wind_direction()
// read the wind direction sensor, return heading in degrees
{
  unsigned int adc = analogRead(WDIR); // get the current reading from the sensor

  // The following table is ADC readings for the wind direction sensor output, sorted from low to high.
  // Each threshold is the midpoint between adjacent headings. The output is degrees for that ADC reading.
  // Note that these are not in compass degree order!  See Weather Meters datasheet for more information.

  if (adc < 380) return (112.5);
  if (adc < 393) return (67.5);
  if (adc < 414) return (90);
  if (adc < 456) return (157.5);
  if (adc < 508) return (135);
  if (adc < 551) return (202.5);
  if (adc < 615) return (180);
  if (adc < 680) return (22.5);
  if (adc < 746) return (45);
  if (adc < 801) return (247.5);
  if (adc < 833) return (225);
  if (adc < 878) return (337.5);
  if (adc < 913) return (0);
  if (adc < 940) return (292.5);
  if (adc < 967) return (315);
  if (adc < 990) return (270);
  return (-1); // error, disconnected?
}

/* From Weather Meters docs and the Weather Board V3 schematic:

 heading         resistance      volts           nominal         midpoint (<)
 112.5	º	0.69	k	1.2	V	372	counts	380
 67.5	º	0.89	k	1.26	V	389	counts	393
 90	º	1	k	1.29	V	398	counts	414
 157.5	º	1.41	k	1.39	V	430	counts	456
 135	º	2.2	k	1.56	V	483	counts	508
 202.5	º	3.14	k	1.72	V	534	counts	551
 180	º	3.9	k	1.84	V	569	counts	615
 22.5	º	6.57	k	2.13	V	661	counts	680
 45	º	8.2	k	2.26	V	700	counts	746
 247.5	º	14.12	k	2.55	V	792	counts	801
 225	º	16	k	2.62	V	811	counts	833
 337.5	º	21.88	k	2.76	V	855	counts	878
 0	º	33	k	2.91	V	902	counts	913
 292.5	º	42.12	k	2.98	V	925	counts	940
 315	º	64.9	k	3.08	V	956	counts	967
 270	º	98.6	k	3.15	V	978	counts	>967
 */
#endif


