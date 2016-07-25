#ifndef MQTTWIFLYWEATHERBOARD_WATCHDOGCONFIG_H_
#define MQTTWIFLYWEATHERBOARD_WATCHDOGCONFIG_H_


#define USE_HARDWARE_WATCHDOG       false


#if USE_HARDWARE_WATCHDOG

const int RESET_WATCHDOG1         = 10;
unsigned long previousMillis      = 0;              // will store last time LED was updated
const long watchdog_interval      = 60UL * 1000UL;  // interval at which to blink (milliseconds)

void ResetWatchdog1()
{
 pinMode(RESET_WATCHDOG1, OUTPUT);
 delay(200);
 pinMode(RESET_WATCHDOG1, INPUT);
#if DEBUG
 Serial.println("Watchdog1 Reset");
#endif
}

#endif


#endif  /* MQTTWIFLYWEATHERBOARD_WATCHDOGCONFIG_H_ */

