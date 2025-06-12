# The MIT License (MIT)
# Copyright (c) 2019 Mike Teachman
# https://opensource.org/licenses/MIT
#
# Example MicroPython and CircuitPython code showing how to use the MQTT protocol to  
# publish data to an Adafruit IO feed
#
# Tested using the releases:
#   ESP8266
#       MicroPython 1.9.3
#       MicroPython 1.9.4
#       MicroPython 1.10
#       CircuitPython 2.3.1     (needs addition of CircuitPython specific umqtt module)
#       CircuitPython 3.0.0     (needs addition of CircuitPython specific umqtt module)
#   ESP32
#       MicroPython 1.9.4       (needs addition of MicroPython umqtt module)
#       MicroPython 1.10
#
# Tested using the following boards:
#   Adafruit Feather HUZZAH ESP8266
#   Adafruit Feather HUZZAH ESP32
#   WeMos D1 Mini
#
# User configuration parameters are indicated with "ENTER_".  

import network
import time
from umqtt.robust import MQTTClient
import os
#import gc
import sys

import tcs34725
import read_temp
from machine import I2C, Pin, ADC

def color_rgb_bytes(color_raw):
    """Read the RGB color detected by the sensor.  Returns a 3-tuple of
    red, green, blue component values as bytes (0-255).
    NOTE: These values are normalized against 'clear', remove the division
    by 'clear' if you need the raw values.
    """
    r, g, b, clear = color_raw
    # Avoid divide by zero errors ... if clear = 0 return black
    if clear == 0:
        return (0, 0, 0)
    red   = int(pow((int((r/clear) * 256) / 255), 2.5) * 255)
    green = int(pow((int((g/clear) * 256) / 255), 2.5) * 255)
    blue  = int(pow((int((b/clear) * 256) / 255), 2.5) * 255)
    # Handle possible 8-bit overflow
    if red > 255:
        red = 255
    if green > 255:
        green = 255
    if blue > 255:
        blue = 255
    return (red, green, blue)

### OD stuff
jakobs_led = Pin(12, Pin.OUT)
jakobs_led.value(1)
led = Pin(27, Pin.OUT)
# Turn it ON (set HIGH)
led.value(1)
pwr = Pin(14, Pin.OUT)
pwr.value(1)  # Turn GPIO14 HIGH to "power" sensor

# Define I2C
i2c = I2C(0, scl=Pin(22), sda=Pin(23))  # Adjust pins as needed
sensor = tcs34725.TCS34725(i2c)

led_ctrl = Pin(33, Pin.OUT)
led_ctrl.value(0)

###Web

def cb(topic, msg):
    try:
        slider_value = msg.decode('utf-8')
        print('Slider value received:', slider_value)

        # You can use slider_value to control something (e.g., LED brightness, motor speed)
        
    except Exception as e:
        print('Error processing slider value:', e)

# WiFi connection information
WIFI_SSID = 'iPhone'
WIFI_PASSWORD = '123456798'

# turn off the WiFi Access Point
ap_if = network.WLAN(network.AP_IF)
ap_if.active(False)

# connect the device to the WiFi network
wifi = network.WLAN(network.STA_IF)
wifi.active(True)
wifi.connect(WIFI_SSID, WIFI_PASSWORD)

# wait until the device is connected to the WiFi network
MAX_ATTEMPTS = 20
attempt_count = 0

print('Connecting..')
while not wifi.isconnected() and attempt_count < MAX_ATTEMPTS:
    attempt_count += 1
    time.sleep(1)

if attempt_count == MAX_ATTEMPTS:
    print('could not connect to the WiFi network')
    sys.exit()
else:
    print('Connected to WiFi')

# create a random MQTT clientID 
random_num = int.from_bytes(os.urandom(3), 'little')
mqtt_client_id = bytes('client_'+str(random_num), 'utf-8')

# connect to Adafruit IO MQTT broker using unsecure TCP (port 1883)
# 
# To use a secure connection (encrypted) with TLS: 
#   set MQTTClient initializer parameter to "ssl=True"
#   Caveat: a secure connection uses about 9k bytes of the heap
#         (about 1/4 of the micropython heap on the ESP8266 platform)
ADAFRUIT_IO_URL = b'io.adafruit.com' 
ADAFRUIT_USERNAME = b'MC65'
ADAFRUIT_IO_KEY = b'aio_lNBG33noC0OfwNIsl2QYZkvkVQbQ'

client = MQTTClient(client_id=mqtt_client_id, 
                    server=ADAFRUIT_IO_URL, 
                    user=ADAFRUIT_USERNAME, 
                    password=ADAFRUIT_IO_KEY,
                    ssl=False)
try:       
    client.connect()
    print('Connected to client')
except Exception as e:
    print('could not connect to MQTT server {}{}'.format(type(e).__name__, e))
    sys.exit()

# publish free heap statistics to Adafruit IO using MQTT
#
# format of feed name:  
#   "ADAFRUIT_USERNAME/feeds/ADAFRUIT_IO_FEEDNAME"
mqtt_feedname_temp = bytes('{:s}/feeds/{:s}'.format(ADAFRUIT_USERNAME, b'temperature'), 'utf-8')
mqtt_feedname_OD_r = bytes('{:s}/feeds/{:s}'.format(ADAFRUIT_USERNAME, b'od.red'), 'utf-8')
mqtt_feedname_OD_g = bytes('{:s}/feeds/{:s}'.format(ADAFRUIT_USERNAME, b'od.green'), 'utf-8')
mqtt_feedname_OD_b = bytes('{:s}/feeds/{:s}'.format(ADAFRUIT_USERNAME, b'od.blue'), 'utf-8')
mqtt_feedname_out = bytes('{:s}/feeds/{:s}'.format(ADAFRUIT_USERNAME, b'out-test'), 'utf-8')
client.set_callback(cb)
client.subscribe(mqtt_feedname_out)
PUBLISH_PERIOD_IN_SEC = 10

###init thermistor
temp_sens = read_temp.init_temp_sensor()

###loop
print('Publishing..')
while True:
    try:
        # Read temperature
        temp = read_temp.read_temp(temp_sens)

        # Read color sensor
        r,g,b = color_rgb_bytes(sensor.read(True))
        
        client.publish(mqtt_feedname_temp, bytes(str(temp), 'utf-8'), qos=0)
        client.publish(mqtt_feedname_OD_r, bytes(str(r), 'utf-8'), qos=0)
        client.publish(mqtt_feedname_OD_g, bytes(str(g), 'utf-8'), qos=0)
        client.publish(mqtt_feedname_OD_b, bytes(str(b), 'utf-8'), qos=0)
        print('this runs')

        client.check_msg()
        
        time.sleep(PUBLISH_PERIOD_IN_SEC)
    except KeyboardInterrupt:
        print('Ctrl-C pressed...exiting')
        client.disconnect()
        sys.exit()