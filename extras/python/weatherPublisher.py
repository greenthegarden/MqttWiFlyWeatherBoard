#!/usr/bin/env python

#---------------------------------------------------------------------------------------
# Load configuration values
#
#---------------------------------------------------------------------------------------

# see https://wiki.python.org/moin/ConfigParserShootout
from configobj import ConfigObj
config = ConfigObj('weatherPublisher.cfg')

print("{0}".format("MQTT BoM WOW and Twitter uploader"))


#---------------------------------------------------------------------------------------
# Configure global variables
#
#---------------------------------------------------------------------------------------

# interval (minutes) at which a new report is published
REPORT_INTERVAL = config['REPORT_INTERVAL']
assert REPORT_INTERVAL > 2, "REPORT_INTERVAL must be greater than interval between measurements: %r" % 5

print("Reports published every {0} minutes".format(REPORT_INTERVAL))

# global variables
tempc           = float(config['var_init']['tempc'])
# to keep track of daily data (midnight to midnight)
tempc_daily_max = float(config['var_init']['tempc_daily_max'])
tempc_daily_min = float(config['var_init']['tempc_daily_min'])
rainmmdaily     = float(config['var_init']['rainmmdaily'])
# global variables to keep track of day/night data (9am to 9am)
tempc_to9_max   = float(config['var_init']['tempc_to9_max'])
tempc_to9_min   = float(config['var_init']['tempc_to9_min'])
rainmm9am       = float(config['var_init']['rainmm9am'])
# global variables to keep track of hourly data
rainmm          = float(config['var_init']['rainmm'])

#for key, value in config['var_init'].items() :
#	print ("{0}: {1}".format(key, value))

#---------------------------------------------------------------------------------------
# Modules and details to support Twitter feed
#
#---------------------------------------------------------------------------------------

import tweepy

# for information about setting up Twitter see
# http://raspi.tv/2014/tweeting-with-python-tweepy-on-the-raspberry-pi-part-2-pi-twitter-app-series

# OAuth process, using the keys and tokens
auth = tweepy.OAuthHandler(config['twitter_cfg']['CONSUMER_KEY'],
                           config['twitter_cfg']['CONSUMER_SECRET'],
                           )
auth.set_access_token(config['twitter_cfg']['ACCESS_TOKEN'],
                      config['twitter_cfg']['ACCESS_TOKEN_SECRET'],
                      )

# Creation of the actual interface, using authentication
api = tweepy.API(auth)

TWITTER_PREFIX = config['twitter_cfg']['TWITTER_PREFIX']

twitter_report = {}

def send_data_to_twitter() :

	global twitter_report

	twitter_str = TWITTER_PREFIX
	cnt = 0

	if len(twitter_report) > 1 :
		for key, value in twitter_report.iteritems() :
			twitter_str += key + ": " + value
			cnt += 1
			if cnt < len(twitter_report) :
				twitter_str += ", "

		assert len(twitter_str) < 140, "Twitter messages must be less than 140 characters!"

		print("Twitter string: {0}".format(twitter_str))

		try :
			api.update_status(status=twitter_str)
		except :
			print "Twitter post error"

	twitter_report = {}	# reset report

def publish_twitter_string() :

	global twitter_report

	twitter_str = TWITTER_PREFIX
	cnt = 0

	if len(twitter_report) > 1 :
		for key, value in twitter_report.iteritems() :
			twitter_str += key + ": " + value
			cnt += 1
			if cnt < len(twitter_report) :
				twitter_str += ", "

		assert len(twitter_str) < 140, "Twitter messages must be less than 140 characters!"

		print("Twitter string: {0}".format(twitter_str))

		client.publish("weather/twitter/feed", str(twitter_str))

	twitter_report = {}	# reset report


#---------------------------------------------------------------------------------------
# Modules and details to support Bom WoW feed
#
#---------------------------------------------------------------------------------------

import requests
import json

# sends a report to the BoM WOW in format

# Weather Data (from http://wow.metoffice.gov.uk/support/dataformats)

# The following is a list of items of weather data that can be uploaded to WOW.
# Provide each piece of information as a key/value pair, e.g. winddir=225.5 or tempf=32.2.
# Note that values should not be quoted or escaped.
# Key           Value                                                              Unit
# winddir       Instantaneous Wind Direction                                       Degrees (0-360)
# windspeedmph  Instantaneous Wind Speed                                           Miles per Hour
# windgustdir   Current Wind Gust Direction (using software specific time period)  0-360 degrees
# windgustmph   Current Wind Gust (using software specific time period)            Miles per Hour
# humidity      Outdoor Humidity                                                   0-100 %
# dewptf        Outdoor Dewpoint                                                   Fahrenheit
# tempf         Outdoor Temperature                                                Fahrenheit
# rainin        Accumulated rainfall in the past 60 minutes                        Inches
# dailyrainin   Inches of rain so far today                                        Inches
# baromin       Barometric Pressure (see note)                                     Inches
# soiltempf     Soil Temperature                                                   Fahrenheit
# soilmoisture  % Moisture                                                         0-100 %
# visibility    Visibility                                                         Nautical Miles

# http://wow.metoffice.gov.uk/automaticreading?siteid=123456&siteAuthenticationKey=654321&dateutc=2011-02-02+10%3A32%3A55&winddir=230&windspeedmph=12&windgustmph=12& windgustdir=25&humidity=90&dewptf=68.2&tempf=70&rainin=0&dailyrainin=5&baromin=29.1&soiltempf=25&soilmoisture=25&visibility=25&softwaretype=weathersoftware1.0

# details for Bom WoW site
BOM_WOW_URL             = config['bom_wow_cfg']['BOM_WOW_URL']
#SITE_ID                 = config['bom_wow_cfg']['SITE_ID']
#SITE_AUTHENTICATION_KEY = config['bom_wow_cfg']['SITE_AUTHENTICATION_KEY']	# 6 digit number

# payload initialised with BoM WoW siteid and siteAuthenticationKey
payload = {'siteid': config['bom_wow_cfg']['SITE_ID'],
           'siteAuthenticationKey': config['bom_wow_cfg']['SITE_AUTHENTICATION_KEY'],
           }

print("Uploading to Site ID {0}".format(payload.get('siteid')))
print("Using Site Authentication Key {0}".format(payload.get('siteAuthenticationKey')))

def send_data_to_wow() :

	# add time to report
	# The date must be in the following format: YYYY-mm-DD HH:mm:ss,
	# where ':' is encoded as %3A, and the space is encoded as either '+' or %20.
	# An example, valid date would be: 2011-02-29+10%3A32%3A55, for the 2nd of Feb, 2011 at 10:32:55.
	# Note that the time is in 24 hour format.
	# Also note that the date must be adjusted to UTC time - equivalent to the GMT time zone.
	format = "%Y-%m-%d+%H:%M:%S"
	datestr = msg_arrival_time_utc.strftime(format)
	datestr = datestr.replace(':', '%3A')
	payload['dateutc'] = datestr

	# send report

	print("payload local time: {0}".format(msg_arrival_time_local))
	print("payload to be sent: {0}".format(payload))

	# POST with form-encoded data1
#	r = requests.post(BOM_WOW_URL, data=payload)

	# All requests will return a status code.
	# A success is indicated by 200.
	# Anything else is a failure.
	# A human readable error message will accompany all errors in JSON format.
#	print("POST request status code: {0}".format(r.json))

def publish_wow_data_string() :

	# add time to report
	# The date must be in the following format: YYYY-mm-DD HH:mm:ss,
	# where ':' is encoded as %3A, and the space is encoded as either '+' or %20.
	# An example, valid date would be: 2011-02-29+10%3A32%3A55, for the 2nd of Feb, 2011 at 10:32:55.
	# Note that the time is in 24 hour format.
	# Also note that the date must be adjusted to UTC time - equivalent to the GMT time zone.
	format = "%Y-%m-%d+%H:%M:%S"
	datestr = msg_arrival_time_utc.strftime(format)
	datestr = datestr.replace(':', '%3A')
	payload['dateutc'] = datestr

	# publish report

	print("payload local time: {0}".format(msg_arrival_time_local))
	print("payload to be sent: {0}".format(payload))

	client.publish("weather/bom_wow/feed", str(payload))


#---------------------------------------------------------------------------------------
# Modules and methods for processing weather data
#
#---------------------------------------------------------------------------------------

import numericalunits as nu
nu.reset_units()

# conversion of degrees Celcius to degrees
def degCtoF(tempc) :
	return( float(tempc) * (9/5.0) + 32 )

def dewpoint_calc(tempc, humidity) :
	# calculate dewpoint based on temperature and humidity
	from math import log
	if (tempc > 0.0) :
		Tn = 243.12
		m = 17.62
	else :
		Tn = 272.62
		m = 22.46
	dewpoint = (Tn*(log(humidity/100.0)+((m*tempc)/(Tn+tempc)))/(m-log(humidity/100.0)-((m*tempc)/(Tn+tempc))))
#	print("dewpoint: {0}".format(dewpoint))
	return dewpoint

def wind_degrees_to_direction(degrees) :
	if degree == 0 :
		return "N"
	if degrees == 22.5 :
		return "NNE"
	if degrees == 45 :
		return "NE"
	if degrees == 67.5 :
		return "ENE"
	if degrees == 90 :
		return "E"
	if degrees == 112.5 :
		return "ESE"
	if degrees == 135 :
		return "SE"
	if degrees == 157.5 :
		return "SSE"
	if degrees == 180 :
		return "S"
	if degrees == 202.5 :
		return "SSW"
	if degrees == 225 :
		return "SW"
	if degrees == 247.5 :
		return "WSW"
	if degrees == 270 :
		return "W"
	if degrees == 292.5 :
		return "WNW"
	if degrees == 315 :
		return "NW"
	if degrees == 337.5 :
		return "NNW"


#---------------------------------------------------------------------------------------
# Modules and methods to support MQTT
#
#---------------------------------------------------------------------------------------

import paho.mqtt.client as mqtt

# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc) :

	print("Connected with result code "+str(rc))

	# Subscribing in on_connect() means that if the connection is lost
	# the subscriptions will be renewed when reconnecting.

	# weather station measurement topics
	for topic in config['mqtt_topics']['MEASUREMENT_TOPICS'] :
		client.subscribe(topic)

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg) :

	global msg_arrival_time_local, msg_arrival_time_utc

	msg_arrival_time_local = datetime.datetime.now()	# local time
	msg_arrival_time_utc = datetime.datetime.utcnow()

	global tempc_msg_arrival_time, tempc
	global tempc_daily_max, tempc_daily_min, rainfall_local_9am
	global rainmm, dailyrainmm

	print(msg.topic+" "+str(msg.payload))

	global twitter_report

	bom_wow_report = {}

  # temperature data
	if msg.topic == config['mqtt_data_topics']['TEMPERATURE_TOPIC'] :
  	# in degrees Celcius
		tempc_msg_arrival_time = msg_arrival_time_local
		twitter_report['Temperature'] = msg.payload	# add as string rather than float
   	# convert to degrees Fahrenheit for Bom WoW
		tempc = float(msg.payload)
		bom_wow_report['tempf'] = '{0:.1f}'.format(degCtoF(tempc))
		payload.update(bom_wow_report)
		if (tempc > tempc_daily_max) :
			tempc_daily_max = tempc
			client.publish("weather/temperature/daily_max", str(tempc_daily_max))
			client.publish("weather/temperature/daily_max_time", str(msg_arrival_time_local))
		if (tempc < tempc_daily_min) :
			tempc_daily_min = tempc
			client.publish("weather/temperature/daily_min", str(tempc_daily_min))
			client.publish("weather/temperature/daily_min_time", str(msg_arrival_time_local))
	if msg.topic == config['mqtt_data_topics']['HUMIDITY_TOPIC'] :
  	# as a percentage
		twitter_report['Humidity'] = msg.payload	# add as string rather than float
		humidity = float(msg.payload)
		bom_wow_report['humidity'] = str(humidity)
		payload.update(bom_wow_report)
		if ( msg_arrival_time_local - tempc_msg_arrival_time ) < datetime.timedelta(seconds=2) :
			dewpoint = dewpoint_calc(float(bom_wow_report.get('tempc',tempc)), humidity)
			dewpoint_str = '{0:.1f}'.format(dewpoint)
			client.publish("weather/dewpoint/SHT15_dewpoint", dewpoint_str)
#			bom_wow_report['dewptf'] = dewpoint_str
#			payload.update(bom_wow_report)
	# weather station will not report measurements from pressure sensor
	# if error code generated when sensor is initialised, or
	# if error code generated when taking reading
	if msg.topic == config['mqtt_data_topics']['PRESSURE_TOPIC'] :
		# in mbar
		twitter_report['Pressure'] = msg.payload
		# convert to inches for Bom WoW
		# 1 millibar (or hectopascal/hPa), is equivalent to 0.02953 inches of mercury (Hg).
		# source: http://weatherfaqs.org.uk/node/72
#		bom_wow_report['baromin'] = '{0:.1f}'.format(float(msg.payload) * 0.02953)
#		payload.update(bom_wow_report)
	# weather station will not report measurements from the weather sensors
	# (wind and rain) if error code generated by wind direction reading
	if msg.topic == config['mqtt_data_topics']['WIND_DIR_TOPIC'] :
		# in degrees
		twitter_report['Wind_Dir'] = wind_degrees_to_direction(msg.payload)
#		bom_wow_report['winddir'] = msg.payload
#		payload.update(bom_wow_report)
	if msg.topic == config['mqtt_data_topics']['WIND_SPEED_TOPIC'] :
  	# in knots
  	# convert to miles per hour
		twitter_report['Wind_Spd'] = msg.payload	# add as string rather than float
#		bom_wow_report['windspeedmph'] = '{0:.1f}'.format(float(msg.payload) * 1.15078)
#		payload.update(bom_wow_report)
	if msg.topic == config['mqtt_data_topics']['RAIN_TOPIC'] :
  	# in millimetres
  	# convert to inches
  	# resets automatically on hour
		rainmm += float(msg.payload)
		bom_wow_report['rainin'] = (rainmm*nu.mm)/nu.inch
		payload.update(bom_wow_report)
		# rain_9 is the rain since 9am - value is reset at 9am
		rainmm9am += float(msg.payload)
		twitter_report['Rain'] = str(rainmm9am)
		client.publish("weather/rainfall/since9am", str(rainmmdaily))
		# need to zero at midnight (occurs in main loop - value here will have already been reset)
		rainmmdaily += float(msg.payload)
		bom_wow_report['dailyrainin'] = (rainmmdaily*nu.mm)/nu.inch
		payload.update(bom_wow_report)
		client.publish("weather/rainfall/sincemidnight", str(rainmmdaily))
	if msg.topic == config['mqtt_data_topics']['BATTERY_VOLTAGE_TOPIC'] :
		twitter_report['Battery_Voltage'] = msg.payload
	if msg.topic == config['mqtt_data_topics']['SOLAR_VOLTAGE_TOPIC'] :
		twitter_report['Solar_Voltage'] = msg.payload
	if msg.topic == config['mqtt_data_topics']['OUTPUT_VOLTAGE_TOPIC'] :
		twitter_report['Output_Voltage'] = msg.payload

# Definition of MQTT client and connection to MQTT Broker

client = mqtt.Client()

client.connect(config['mqtt_configuration']['MQTT_BROKER_IP'],
               int(config['mqtt_configuration']['MQTT_BROKER_PORT']),
               int(config['mqtt_configuration']['MQTT_BROKER_PORT_TIMEOUT'])
               )

print("Connected to MQTT broker at {0}".format(config['mqtt_configuration']['MQTT_BROKER_IP']))

# link to callback functions
client.on_connect = on_connect
client.on_message = on_message

client.loop_start()


#---------------------------------------------------------------------------------------
# Initialisation of time  and schedules
#
#---------------------------------------------------------------------------------------

import datetime

msg_arrival_time_local = datetime.datetime.min    # keep track of the time corresponding to the first data for a new report
msg_arrival_time_utc   = datetime.datetime.min
tempc_msg_arrival_time = datetime.datetime.min		# used to ensure tempc measurement is not too old for dewpoint calculation
sent_report_with_time  = datetime.datetime.now()	# keep track of the time a report was last sent

import schedule
import time

# define schedule callbacks

def zero_data_on_hour() :
	print("data reset on hour")
	global rainmm
	rainmm = 0

def zero_data_at_9() :
	print("data reset at 9:00")
	global rainmm9am
	rainmm9am = 0

def zero_data_at_midnight() :
	print("data reset at midnight")
	global rainmmdaily
	rainmmdaily = 0

def publish_weather() :

		global sent_report_with_time

		# ensure the report data is more up-to-date than previously sent message
		# should prevent repots being sent if sensor is off
		if ( msg_arrival_time_local > sentreportwithtime ) :

#			send_data_to_wow()
#			send_data_to_twitter()

			publish_wow_data_string()
			publish_twitter_string()

		sent_report_with_time = msg_arrival_time_local

# define schedules

schedule.every(int(REPORT_INTERVAL)).minutes.do(publish_weather)
schedule.every().hour.at(':00').do(zero_data_on_hour)
schedule.every().day.at("9:00").do(zero_data_at_9)
schedule.every().day.at("0:00").do(zero_data_at_midnight)


#---------------------------------------------------------------------------------------
# Program loops continuously from here
#
#---------------------------------------------------------------------------------------

while True :

	schedule.run_pending()
