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
  delay(5000);                  // lots of time for the WiFly to start up
  Serial.begin(BAUD_RATE);      // Start hardware Serial for the RN-XV
  WiFly.setUart(&Serial);       // Tell the WiFly library that we are not using the SPIUart
}

byte wifly_connect()
{
#if USE_STATUS_LED
  digitalWrite(STATUS_LED, HIGH);
#endif

  WiFly.begin();

  if (!WiFly.join(SSID, PASSPHRASE, mode)) {
    wiflyConnected = false;
    return 0;
  } else {
    wiflyConnected = true;
#if USE_STATUS_LED
    digitalWrite(STATUS_LED, LOW);
#endif
    return 1;
  }
}

void wifly_disconnect()
{
  if (wiflyConnected) {
    WiFly.leave();
    wiflyConnected = false;
  }
}

void wifly_sleep()
{
  wifly_disconnect();
  
  DEBUG_LOG(1, "WiFly: setting sleep timer");
  WiFly.setSleepTimer(10);

  DEBUG_LOG(1, "WiFly: setting wake timer");
  WiFly.setWakeTimer(MEASUREMENT_INTERVAL_SECS - 20);
}


#endif  /* MQTTWIFLYWEATHERBOARD_WIFLYCONFIG_H_ */

