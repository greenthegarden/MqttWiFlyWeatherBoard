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
    IP:  192.168.1.56
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

// WiFly setup and connection routines

// WiFly libraries
#include <SPI.h>
#include <WiFly.h>

#define USE_WIFLY_SLEEP true

#include "networkConfig.h"

boolean wiflyConnectedToNetwork = false;

WiFlyClient networkClient;      // creates a WiFly instance

void wifly_init()
{
  DEBUG_LOG(1, "  setting interface");
#if DEBUG_LEVEL == 0
  Serial.begin(BAUD_RATE);      // Start hardware Serial for the RN-XV
#endif
  WiFly.setUart(&Serial);       // Tell the WiFly library that we are not using the SPIUart

  DEBUG_LOG(1, "  configuring");
  WiFly.disableTimers();
  WiFly.begin();
}


#if USE_WIFLY_SLEEP
#endif

const byte NETWORK_CONNECT_ATTEMPTS       = 5;

byte wifly_connect_to_network()
{
#if USE_STATUS_LED
  digitalWrite(STATUS_LED, HIGH);
#endif

  DEBUG_LOG(1, "  joining network");

  for (byte i = 0; i < NETWORK_CONNECT_ATTEMPTS; i++) {
    DEBUG_LOG(1, "  ATTEMPT:");
    DEBUG_LOG(1, i + 1);
    if (!WiFly.join(SSID, PASSPHRASE, mode)) {
      wiflyConnectedToNetwork = false;
      DEBUG_LOG(1, "  connection failed");
    } else {
      wiflyConnectedToNetwork = true;
      DEBUG_LOG(1, "  connected");
#if USE_STATUS_LED
      digitalWrite(STATUS_LED, LOW);
#endif
      return 1;
    }
  }
  return 0;
}

byte wifly_disconnect_from_network()
{
  if (wiflyConnectedToNetwork) {
    WiFly.leave();
    wiflyConnectedToNetwork = false;
    DEBUG_LOG(1, "    DISCONNECTED");
    return 1;
  }
  DEBUG_LOG(1, "    NOT CONNECTED");
  return 0;
}

#if USE_WIFLY_SLEEP
boolean wiflyAfterSleep = false;
boolean wiflyReady      = false;
boolean wiflyAwake      = false;

const unsigned long AFTER_WAKE_DELAY   = 2000UL; // milliseconds

void wifly_after_wake()
{
//  delay(AFTER_WAKE_DELAY);
  DEBUG_LOG(1, "after waking");
  WiFly.disableTimers();
  WiFly.begin();
}

const unsigned long RTS_TIMEOUT_MILLIS    = 500UL;

const unsigned long WAKE_TIMER_DELTA_SECS = 1UL;  // seconds early to wake WiFly

void wifly_sleep()
{
  DEBUG_LOG(1, "sleeping wifly");

  // close tcp connection
  wifly_disconnect_from_network();

  delay(500);

  WiFly.sleep(MEASUREMENT_INTERVAL_SECS - WAKE_TIMER_DELTA_SECS);

  wiflyAfterSleep = true;
}
#endif  /* USE_WIFLY_SLEEP */


#endif  /* MQTTWIFLYWEATHERBOARD_WIFLYCONFIG_H_ */

