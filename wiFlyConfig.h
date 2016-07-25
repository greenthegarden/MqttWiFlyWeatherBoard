#ifndef MQTTWIFLYWEATHERBOARD_WIFLYCONFIG_H_
#define MQTTWIFLYWEATHERBOARD_WIFLYCONFIG_H_


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

set ip dhcp 1
set wlan ssid xxx
set wlan phrase xxx
set wlan join 1

save
reboot
*/

// WiFly setup and connection routines

// WiFly libraries
#include <SPI.h>
#include <WiFly.h>

#include "networkConfig.h"

boolean wiflyConnected = false;

WiFlyClient wiflyClient;

void wifly_configure()
{
  Serial.begin(BAUD_RATE);      // Start hardware Serial for the RN-XV
  WiFly.setUart(&Serial);       // Tell the WiFly library that we are not using the SPIUart
}

byte wifly_connect()
{
#if USE_STATUS_LED
  digitalWrite(STATUS_LED, HIGH);
#endif

  DEBUG_LOG(1, "initialising wifly");

  WiFly.begin();
  delay(5000);  // time for WiFly to start

  DEBUG_LOG(1, "joining network");

  if (!WiFly.join(SSID, PASSPHRASE, mode)) {
    wiflyConnected = false;
    DEBUG_LOG(1, "  connection failed");
  } else {
    wiflyConnected = true;
    DEBUG_LOG(1, "  connected");
#if USE_STATUS_LED
    digitalWrite(STATUS_LED, LOW);
#endif
    return 1;
  }
  return 0;
}

byte wifly_disconnect()
{
  if (wiflyConnected) {
    WiFly.leave();
    wiflyConnected = false;
    return 1;
  }
  return 0;
}

const unsigned long SLEEP_TIMER_DELAY_SECS = 10UL;  // seconds delay to sleep WiFly
const unsigned long WAKE_TIMER_DELTA_SECS  = 20UL;  // seconds early to wake WiFly

void wifly_sleep()
{
  wifly_disconnect();
  
  DEBUG_LOG(1, "WiFly: setting sleep timer");
  WiFly.setSleepTimer(SLEEP_TIMER_DELAY_SECS);

  DEBUG_LOG(1, "WiFly: setting wake timer");
  WiFly.setWakeTimer(MEASUREMENT_INTERVAL_SECS - WAKE_TIMER_DELTA_SECS);
}


#endif  /* MQTTWIFLYWEATHERBOARD_WIFLYCONFIG_H_ */

