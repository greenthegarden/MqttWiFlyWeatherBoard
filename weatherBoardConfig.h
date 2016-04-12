#ifndef MQTTWIFLYWEATHERBOARD_WEATHERBOARDCONFIG_H_
#define MQTTWIFLYWEATHERBOARD_WEATHERBOARDCONFIG_H_


// external sensor libraries
#include <SHT1x.h>              // SHT15 temperature/humidity sensor library
#include <Wire.h>               // I2C library (necessary for pressure sensor)
#include <SFE_BMP085.h>         // BMP085 pressure sensor library


// Weather Board Digital I/O pin definitions

#define   RAIN         2
#define   WSPEED       3
#define   STATUS_LED   4
#define   RF_CTS       5
#define   RF_RTS       6
#define   EOC          8
#define   XCLR         9

// (the following three are predefined)
// #define MOSI        11
// #define MISO        12
// #define SCK         13

// analog I/O pins
#define   WDIR         A0
#define   SHT1x_DATA   A4
#define   SHT1x_CLOCK  A5
#define   BATT_LVL     A6
#define   LIGHT        A7


// global variable definitions
unsigned long previousMeasurementMillis = 0;
unsigned long previousWindDirMillis     = 0;
boolean  pressureSensorStatus         = false;

// BMP085 status messages

const char BMP085_INIT_SUCCESS[]          PROGMEM = "BMP085: Init success";
const char BMP085_INIT_FAILURE[]          PROGMEM = "BMP085: init failure";
const char BMP085_ERROR_PRESSURE_START[]  PROGMEM = "BMP085: Pressure Start error";
const char BMP085_ERROR_PRESSURE_GET[]    PROGMEM = "BMP085: Pressure Get error";
const char BMP085_ERROR_TEMP_START[]      PROGMEM = "BMP085: Temperature Start error";
const char BMP085_ERROR_TEMP_GET[]        PROGMEM = "BMP085: Temperature Get error";

PGM_P const BMP085_STATUS_MESSAGES[]      PROGMEM = { BMP085_INIT_SUCCESS,          // idx = 0
                                                      BMP085_INIT_FAILURE,          // idx = 1
                                                      BMP085_ERROR_PRESSURE_START,  // idx = 2
                                                      BMP085_ERROR_PRESSURE_GET,    // idx = 3
                                                      BMP085_ERROR_TEMP_START,      // idx = 4
                                                      BMP085_ERROR_TEMP_GET,        // idx = 5
                                                    };




// initialisation of remaining sensor objects
SHT1x humiditySensor(SHT1x_DATA, SHT1x_CLOCK);
SFE_BMP085 pressureSensor(BMP_ADDR);

void weatherboard_sensors_initialisaton()
{
  // Configure sensors
  // set up inputs and outputs
  pinMode(XCLR, OUTPUT);                // output to BMP085 reset (unused)
  digitalWrite(XCLR, HIGH);             // make pin high to turn off reset

  pinMode(EOC, INPUT);                  // input from BMP085 end of conversion (unused)
  digitalWrite(EOC, LOW);               // turn off pullup

  // Reset the humidity sensor connection so that the I2C bus can be accessed
  TWCR &= ~(_BV(TWEN));                 // turn off I2C enable bit so we can access the SHT15 humidity sensor
  digitalWrite(XCLR, LOW);              // disable the BMP085 while resetting humidity sensor
  humiditySensor.connectionReset();     // reset the humidity sensor connection
  TWCR |= _BV(TWEN);                    // turn on I2C enable bit so we can access the BMP085 pressure sensor
  digitalWrite(XCLR, HIGH);             // enable BMP085
  delay(10);                            // wait for the BMP085 pressure sensor to become ready after reset

  progBuffer[0] = '\0';
  strcpy_P(progBuffer, (char*)pgm_read_word(&(STATUS_TOPICS[6])));
  if (pressureSensor.begin()) {        // initialize the BMP085 pressure sensor (important to get calibration values stored on the device)
    pressureSensorStatus = true;
    if (mqttClient.connected()) {
      messBuffer[0] = '\0';
      strcpy_P(messBuffer, (char*)pgm_read_word(&(BMP085_STATUS_MESSAGES[0])));
      mqttClient.publish(progBuffer, messBuffer);
    }
  } else {
    if (mqttClient.connected()) {
      messBuffer[0] = '\0';
      strcpy_P(messBuffer, (char*)pgm_read_word(&(BMP085_STATUS_MESSAGES[1])));
      mqttClient.publish(progBuffer, messBuffer);
    }
  }
}



#if ENABLE_WEATHER_METERS

// Global variables 
unsigned int windRpm                    = 0;
unsigned int windRpmMax                = 0;
unsigned int windStopped                    = 0;
// volatiles are subject to modification by IRQs
volatile unsigned long tempWindRpm      = 0, windTime = 0, windLast = 0, windInterval = 0;
volatile unsigned char windIntCount;
volatile boolean       gotWindSpeed;
volatile unsigned long rainTime         = 0, rainLast = 0, rainInterval = 0, rain = 0;

// Function definitions
float get_wind_direction();
// interrupt routines (these are called by the hardware interrupts, not by the main code)
void rain_irq();
float get_wind_direction();

#if ENABLE_WIND_DIR_AVERAGING
#include "RunningAverage.h"
byte WIND_DIR_AVERAGING_SIZE    = 10;
unsigned long WIND_DIR_INTERVAL = 1000;
RunningAverage wind_dir_avg(WIND_DIR_AVERAGING_SIZE);
#endif


// Constant conversion factors
//const float WIND_RPM_TO_MPH  = 22.686745;         // divide RPM by this for velocity
const float WIND_RPM_TO_MPS    = 50.748803;         // divide RPM by this for meters per second
// 1 m/s = 1.943844492 knots
const float WIND_RPM_TO_KNOTS  = WIND_RPM_TO_MPS / 1.943844492;
//const float RAIN_BUCKETS_TO_INCHES = 0.014815;    // multiply bucket tips by this for inches
const float RAIN_BUCKETS_TO_MM = 0.376296;          // multiply bucket tips by this for mm
const unsigned int ZERODELAY   = 4000;              // ms, zero RPM if no result for this time period (see irq below)


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

byte wind_direction_measurement()
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
    strcpy_P(progBuffer, (char*)pgm_read_word(&(MEASUREMENT_TOPICS[8])));
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
  strcpy_P(progBuffer, (char*)pgm_read_word(&(MEASUREMENT_TOPICS[9])));
  mqttClient.publish(progBuffer, buf);

  // publish maximum wind speed since last report
  windSpeedMeasurement = float(windRpmMax) / WIND_RPM_TO_KNOTS;
  buf[0] = '\0';
  dtostrf(windSpeedMeasurement,1,FLOAT_DECIMAL_PLACES, buf);
  progBuffer[0] = '\0';
  strcpy_P(progBuffer, (char*)pgm_read_word(&(MEASUREMENT_TOPICS[10])));
  mqttClient.publish(progBuffer, buf);
}

void rainfall_measurement()
{
  // rainfall unit conversion
  float rainfallMeasurement = rain * RAIN_BUCKETS_TO_MM;
  
  buf[0] = '\0';
  dtostrf(rainfallMeasurement,1,FLOAT_DECIMAL_PLACES, buf);
  progBuffer[0] = '\0';
  strcpy_P(progBuffer, (char*)pgm_read_word(&(MEASUREMENT_TOPICS[11])));
  mqttClient.publish(progBuffer, buf);

  // reset value of rain to zero
  rain = 0;
}

void weather_meter_measurement()
{
  // take wind-direction measurement first
  // if returns -1 then treat as sensors not connected
  if (wind_direction_measurement()) {
    windspeed_measurement();
    rainfall_measurement();
  } else {
    progBuffer[0] = '\0';
    strcpy_P(progBuffer, (char*)pgm_read_word(&(STATUS_TOPICS[1])));
    messBuffer[0] = '\0';
    strcpy_P(messBuffer, (char*)pgm_read_word(&(MQTT_PAYLOADS[1])));
    mqttClient.publish(progBuffer, messBuffer);
  }
}

void weatherboard_meters_initialisation()
{
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
}

#endif  /* ENABLE_WEATHER_METERS */


#endif  /* MQTTWIFLYWEATHERBOARD_WEATHERBOARDCONFIG_H_ */

