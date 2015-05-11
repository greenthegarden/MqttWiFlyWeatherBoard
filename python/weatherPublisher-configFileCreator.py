# config file creator

from configobj import ConfigObj
config = ConfigObj()
config.filename = 'weatherPublisher.cfg'

#
config['REPORT_INTERVAL'] = 15
config['measurement_interval'] = 2

# global variable initialisation
var_init = {
	'tempc'           : -100,
	'tempc_daily_max' : -100,
	'tempc_daily_min' : 100,
	'rainmmdaily'     : 0,
	'tempc_to9_max'   : -100,
	'tempc_to9_min'   : -100,
	'rainmm9am'       : 0,
	'rainmm'          : 0,
	}
config['var_init'] = var_init

# twitter configuration
twitter_cfg = {
	'CONSUMER_KEY'        : "Replace with consumer key!",
	'CONSUMER_SECRET'     : "Replace with consumer secret!",
	'ACCESS_TOKEN'        : "Replace with access token!",
	'ACCESS_TOKEN_SECRET' : "Replace with access token secret!",
	'TWITTER_PREFIX'      : "Weather @ Home => ",
	}
config['twitter_cfg'] = twitter_cfg

# BoM WoW configuration
bom_wow_cfg = {
	'BOM_WOW_URL'             : 'http://wow.metoffice.gov.uk/automaticreading?',
	'SITE_ID'                 : 'Replace with site id!',
	'SITE_AUTHENTICATION_KEY' : 'Replace with site authentication key!',
	}
config['bom_wow_cfg'] = bom_wow_cfg

# mqtt configuration
mqtt_configuration = {
	'MQTT_BROKER_IP'           : "192.168.1.55",
	'MQTT_BROKER_PORT'         : "1883",
	'MQTT_BROKER_PORT_TIMEOUT' : "60",
	}
config['mqtt_configuration'] = mqtt_configuration

# Weather station measurement topics
config['mqtt_topics'] = {}
config['mqtt_topics']['MEASUREMENT_TOPICS'] = ['weather/measurement/#', 'weather/sunairplus/#']

# Specific data measurement topics
mqtt_data_topics = {
	'TEMPERATURE_TOPIC'     : 'weather/measurement/SHT15_temp',
	'HUMIDITY_TOPIC'        : 'weather/measurement/SHT15_humidity',
	'PRESSURE_TOPIC'        : 'weather/measurement/BMP085_pressure',
	'WIND_DIR_TOPIC'        : 'weather/measurement/wind_dir',
	'WIND_SPEED_TOPIC'      : 'weather/measurement/wind_spd',
	'RAIN_TOPIC'            : 'weather/measurement/rain',
	'BATTERY_VOLTAGE_TOPIC' : 'weather/sunairplus/battery_voltage',
	'SOLAR_VOLTAGE_TOPIC'   : 'weather/sunairplus/solar_voltage',
	'OUTPUT_VOLTAGE_TOPIC'  : 'weather/sunairplus/output_voltage',
	}
config['mqtt_data_topics'] = mqtt_data_topics


config.write()
