
// Set Tools/Board to "Arduino Pro or Pro Mini (3.3V 8MHz) w/ ATmega328"

// Uses the SHT15x library by Jonathan Oxer et.al.  https://github.com/practicalarduino/SHT1x
// A special version is supplied with this software distribution.
// Place in your Arduino sketchbook under "libraries/SHT1x"

// Uses the SFE_BMP085 library by SparkFun with math from http://wmrx00.sourceforge.net/Arduino/BMP085-Calcs.pdf
// Supplied with this distribution; place in your Arduino sketchbook under "libraries/SFE_BMP085"

// Note dtostrf function details
// dtostrf(floatVar, minStringWidthIncDecimalPoint, numVarsAfterDecimal, charBuf);

// Note itoa function details
// char* itoa (	int val, char *buf, int radix)
// where radix is the number base, ie. 10

// Revision history
// 1.0 2014/04/15
//  Initial release based on Sparkfun WeatherShield v1.4 code
// 2.0 2015/01/31
//  Rewritten to support MQTT and hopefully improve stability!!
// 2.1 2015/02/02
//  Modified to improve stability
//  based on code from https://github.com/xoseperez/rentalito/blob/master/client/rentalito.ino

/* WiFly configuration
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

// WiFly libraries
#include <SPI.h>
#include <WiFly.h>

#include "config.h"

#include <avr/wdt.h>

// external sensor libraries
#if ENABLE_TEMP || ENABLE_HUMIDITY
#include <SHT1x.h>              // SHT15 humidity sensor library
#endif
#if ENABLE_PRESSURE
#include <SFE_BMP085.h>         // BMP085 pressure sensor library
#include <Wire.h>               // I2C library (necessary for pressure sensor)
#endif


// character buffer to support conversion of floats to char
char buf[12];


// global variables definitions
unsigned long previousMillis = 0;
#if ENABLE_WEATHER_METERS
unsigned int windRPM     = 0;
unsigned int stopped     = 0;
// volatiles are subject to modification by IRQs
volatile unsigned long tempwindRPM = 0, windtime = 0, windlast = 0, windinterval = 0;
volatile unsigned char windintcount;
volatile boolean       gotwspeed;
volatile unsigned long raintime = 0, rainlast = 0, raininterval = 0, rain = 0;
#endif


// sensor objects
#if ENABLE_TEMP || ENABLE_HUMIDITY
SHT1x humidity_sensor(SHT1x_DATA, SHT1x_CLOCK);
#endif
#if ENABLE_PRESSURE
SFE_BMP085 pressure_sensor(BMP_ADDR);
#endif


// function declarations
void takeMeasurement();
#if ENABLE_WEATHER_METERS
float get_wind_direction();
// interrupt routines (these are called by the hardware interrupts, not by the main code)
void rainIRQ();
float get_wind_direction();
#endif


void callback(char* topic, uint8_t* payload, unsigned int length)
{
  // nothing to do here!!
}

WiFlyClient wiflyClient;
PubSubClient mqttClient(mqtt_server_addr, mqtt_port, callback, wiflyClient);

void connect_wifly()
{
  wdt_reset();
  
  digitalWrite(STATUS_LED, HIGH);
  
  WiFly.begin();

  if (!WiFly.join(ssid, passphrase, mode))
  {
    wifly_connected = false;
    wdt_reset();
    delay(AFTER_ERROR_DELAY);
  } 
  else {
    wifly_connected = true;
    digitalWrite(STATUS_LED, LOW);
  }
}

void publish_measurements()
{
  wdt_reset();
  
  digitalWrite(STATUS_LED, HIGH);
  
  if (!wifly_connected)
    connect_wifly();

  wdt_reset();
  
  if (wifly_connected)
  {
    // MQTT client setup
//    mqttClient.disconnect();
//    debug(F("Connecting"), GREEN);
    if (mqttClient.connect(mqtt_client_id))
    {
      wdt_reset();
      
      digitalWrite(STATUS_LED, LOW);
      
      mqttClient.publish(wifly_topic, "Connected to broker");
      
      takeMeasurement();
      
      mqttClient.disconnect();
    } 
    else
    {
      wdt_reset();
      delay(AFTER_ERROR_DELAY);
    }
  }
}


/*--------------------------------------------------------------------------------------
 setup()
 Called by the Arduino framework once, before the main loop begins
 --------------------------------------------------------------------------------------*/
void setup()
{
  wdt_disable();
  
  // Configure status LED
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, LOW);
  
  // lots of time for the WiFly to start up
  delay(5000);

  // Configure WiFly
  Serial.begin(BAUD_RATE);      // Start hardware Serial for the RN-XV
  WiFly.setUart(&Serial);       // Tell the WiFly library that we are not using the SPIUart

  connect_wifly();
  
  digitalWrite(STATUS_LED, HIGH);
  
  if (wifly_connected)
  {
    if (mqttClient.connect(mqtt_client_id))
    {
      digitalWrite(STATUS_LED, LOW);
      mqttClient.publish(wifly_topic, "Connected to broker");
    } 
    else
    {
      delay(AFTER_ERROR_DELAY);
    }
  }

  // Configure sensors
#if ENABLE_PRESSURE
  // set up inputs and outputs
  pinMode(XCLR,OUTPUT);                // output to BMP085 reset (unused)
  digitalWrite(XCLR,HIGH);             // make pin high to turn off reset 

  pinMode(EOC,INPUT);                  // input from BMP085 end of conversion (unused)
  digitalWrite(EOC,LOW);               // turn off pullup

  // Reset the humidity sensor connection so that the I2C bus can be accessed
  TWCR &= ~(_BV(TWEN));                // turn off I2C enable bit so we can access the SHT15 humidity sensor 
  digitalWrite(XCLR,LOW);              // disable the BMP085 while resetting humidity sensor
  humidity_sensor.connectionReset();   // reset the humidity sensor connection
  TWCR |= _BV(TWEN);                   // turn on I2C enable bit so we can access the BMP085 pressure sensor
  digitalWrite(XCLR,HIGH);             // enable BMP085
  delay(10);                           // wait for the BMP085 pressure sensor to become ready after reset

  if (pressure_sensor.begin())         // initialize the BMP085 pressure sensor (important to get calibration values stored on the device)
    mqttClient.publish(sensor_topic,"BMP085 init success");
  else
    mqttClient.publish(sensor_topic,"BMP085 init failure");
#endif

#if ENABLE_WEATHER_METERS
  pinMode(WSPEED,INPUT);               // input from wind meters windspeed sensor
  digitalWrite(WSPEED,HIGH);           // turn on pullup

  pinMode(RAIN,INPUT);                 // input from wind meters rain gauge sensor
  digitalWrite(RAIN,HIGH);             // turn on pullup

  // init wind speed interrupt global variables
  gotwspeed       = false; 
  windRPM         = 0; 
  windintcount    = 0;

  // attach external interrupt pins to IRQ functions
  attachInterrupt(0, rainIRQ,   FALLING);
  attachInterrupt(1, wspeedIRQ, FALLING);

  // turn on interrupts
  interrupts();
#endif

  mqttClient.disconnect();
  
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
  wdt_reset();

  unsigned long currentMillis = millis();

  if(currentMillis - previousMillis >= MEASUREMENT_INTERVAL)
  {
    previousMillis = currentMillis;   

    publish_measurements();
  }

#if ENABLE_WEATHER_METERS
  // handle weather meter interrupts in loop()
  static unsigned long windstopped = 0;

  // an interrupt occurred, handle it now
  if (gotwspeed)
  {
    gotwspeed = false;
    windRPM = word(tempwindRPM);
    windstopped = millis() + ZERODELAY;  // save this timestamp
  }  

  // zero wind speed RPM if we don't get a reading in ZERODELAY ms
  if (millis() > windstopped)
  {    
    windRPM = 0;
    windintcount = 0;
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
  wdt_reset();
  // connext to mqtt server
//  if (!mqttClient.loop())
//  {
//    connect_mqtt();
//  }
  
#if ENABLE_TEMP
  temperature_measurement();
#endif
#if ENABLE_HUMIDITY
  humidity_measurement();
#endif
#if ENABLE_PRESSURE
  BMP085_measurement();
#endif
#if ENABLE_LIGHT
  TEMT6000_measurement();
#endif
#if ENABLE_WEATHER_METERS
  windspeed_measurement();
  winddirection_measurement();
  rainfall_measurement();
#endif
}

#if ENABLE_TEMP
void temperature_measurement()
{
  TWCR &= ~(_BV(TWEN));  // turn off I2C enable bit so we can access the SHT15 humidity sensor

  float SHT15_temp = humidity_sensor.readTemperatureC();  // temperature returned in degrees Celcius
  buf[0] = '\0';
  dtostrf(SHT15_temp,1,FLOAT_DECIMAL_PLACES, buf);
  mqttClient.publish(SHT15_temp_topic, buf);
}
#endif

#if ENABLE_HUMIDITY
void humidity_measurement()
{
  TWCR &= ~(_BV(TWEN));  // turn off I2C enable bit so we can access the SHT15 humidity sensor

  float SHT15_humidity = humidity_sensor.readHumidity();
  buf[0] = '\0';
  dtostrf(SHT15_humidity,1,FLOAT_DECIMAL_PLACES, buf);
  mqttClient.publish(SHT15_humidity_topic, buf);
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

    // retrieve BMP085 temperature reading
    // function returns 1 if successful, 0 if failure
    status = pressure_sensor.getTemperature(&BMP085_temp); // temperature returned in degrees Celcius
    if (status != 0)
    {
      // publish BMP085 temperature measurement
      buf[0] = '\0';
      dtostrf(BMP085_temp,1,FLOAT_DECIMAL_PLACES, buf);
      mqttClient.publish(BMP085_temp_topic, buf);

      // tell the sensor to start a pressure measurement
      // the parameter is the oversampling setting, from 0 to 3 (highest res, longest wait)
      // if request is successful, the number of ms to wait is returned
      // if request is unsuccessful, 0 is returned
      status = pressure_sensor.startPressure(3);
      if (status != 0)
      {
        // wait for the measurement to complete
        delay(status);

        // retrieve the BMP085 pressure reading
        // note that the function requires the previous temperature measurement (T)
        // (if temperature is stable, one temperature measurement can be used for a number of pressure measurements)
        // function returns 1 if successful, 0 if failure
        status = pressure_sensor.getPressure(&BMP085_pressure, &BMP085_temp); // mbar, deg C
        if (status != 0 )
        {
          // publish BMP085 pressure measurement
          buf[0] = '\0';
          dtostrf(BMP085_pressure,1,FLOAT_DECIMAL_PLACES, buf);
          mqttClient.publish(BMP085_pressure_topic, buf);
        }
        else
          mqttClient.publish(BMP085_pressure_topic, "ERR_BMP085_PRESSURE_GET");
      }    
      else
        mqttClient.publish(BMP085_pressure_topic, "ERR_BMP085_PRESSURE_START");
    }
    else
      mqttClient.publish(BMP085_temp_topic, "ERR_BMP085_TEMP_GET");
  }
  else
    mqttClient.publish(BMP085_temp_topic, "ERR_BMP085_TEMP_START");
}
#endif

#if ENABLE_LIGHT
void TEMT6000_measurement()
{
  // get light level
  int TEMT6000_light_raw = 1023 - analogRead(LIGHT);
  buf[0] = '\0';
  itoa(TEMT6000_light_raw, buf, 10);
  mqttClient.publish(TEMT6000_light_raw_topic, buf);

  // convert TEMT6000_light_raw voltage value to percentage
  //map(value, fromLow, fromHigh, toLow, toHigh)
  int TEMT6000_light = map(TEMT6000_light_raw, 0, 1023, 0, 100);
  itoa(TEMT6000_light, buf, 10);
  mqttClient.publish(TEMT6000_light_topic, buf);
}
#endif

#if ENABLE_WEATHER_METERS
void windspeed_measurement()
{
  // windspeed unit conversion
  float WM_wspeed = float(windRPM) / WIND_RPM_TO_KNOTS;
  buf[0] = '\0';
  dtostrf(WM_wspeed,1,FLOAT_DECIMAL_PLACES, buf);
  mqttClient.publish(wind_spd_topic, buf);
}

void winddirection_measurement()
{
  // wind direction
  float WM_wdirection = get_wind_direction();  // should return a -1 is disconnected
  buf[0] = '\0';
  dtostrf(WM_wdirection,1,FLOAT_DECIMAL_PLACES, buf);
  mqttClient.publish(wind_dir_topic, buf);
}

void rainfall_measurement()
{
  // rainfall unit conversion
  float WM_rainfall = rain * RAIN_BUCKETS_TO_MM;
  buf[0] = '\0';
  dtostrf(WM_rainfall,1,FLOAT_DECIMAL_PLACES, buf);
  mqttClient.publish(rainfall_topic, buf);
}
#endif


#if ENABLE_WEATHER_METERS
void rainIRQ()
// if the Weather Meters are attached, count rain gauge bucket tips as they occur
// activated by the magnet and reed switch in the rain gauge, attached to input D2
{
  raintime     = micros();              // grab current time
  raininterval = raintime - rainlast;   // calculate interval between this and last event

    if (raininterval > 100)               // ignore switch-bounce glitches less than 100uS after initial edge
  {
    rain++;                             // increment bucket counter
    rainlast = raintime;                // set up for next event
  }
}

void wspeedIRQ()
// if the Weather Meters are attached, measure anemometer RPM (2 ticks per rotation), set flag if RPM is updated
// activated by the magnet in the anemometer (2 ticks per rotation), attached to input D3

// this routine measures RPM by measuring the time between anemometer pulses
// windintcount is the number of pulses we've measured - we need two to measure one full rotation (eliminates any bias between the position of the two magnets)
// when windintcount is 2, we can calculate the RPM based on the total time from when we got the first pulse
// note that this routine still needs an outside mechanism to zero the RPM if the anemometer is stopped (no pulses occur within a given period of time)
{
  windtime = micros(); // grab current time
  if ((windintcount == 0) || ((windtime - windlast) > 10000)) // ignore switch-bounce glitches less than 10ms after the reed switch closes
  {
    if (windintcount == 0) // if we're starting a new measurement, reset the interval
      windinterval = 0;  
    else
      windinterval += (windtime - windlast); // otherwise, add current interval to the interval timer

    if (windintcount == 2) // we have two measurements (one full rotation), so calculate result and start a new measurement
    {
      tempwindRPM = (60000000ul / windinterval); // calculate RPM (temporary since it may change unexpectedly)
      windintcount = 0;
      windinterval = 0;  
      gotwspeed = true; // set flag for main loop
    }

    windintcount++;    
    windlast = windtime; // save the current time so that we can calculate the interval between now and the next interrupt
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


