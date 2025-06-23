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
import urequests
import time
import uasyncio
from umqtt.robust import MQTTClient
import os
import sys
import tcs34725
import cooling
import calculations
from machine import I2C, Pin, ADC, PWM

async def measure_OD(measurement_count, measurement_interval, pump_duration):
    print("Measuring OD")
    # Buffers for RGB readings
    rgb_buffer = []

    for i in range(measurement_count):
        r_raw, g_raw, b_raw, clear = rgb_sensor.read(True)
        rgb_buffer.append((r_raw, g_raw, b_raw, clear))

        await uasyncio.sleep(measurement_interval)

    # Compute averages
    avg_r     = sum(x[0] for x in rgb_buffer) / measurement_count
    avg_g     = sum(x[1] for x in rgb_buffer) / measurement_count
    avg_b     = sum(x[2] for x in rgb_buffer) / measurement_count
    avg_clear = sum(x[3] for x in rgb_buffer) / measurement_count

    print(f"Publishing r: {avg_r}, g: {avg_g}, b: {avg_b}, clear: {avg_clear}")
    client.publish(mqtt_feedname_OD_r, bytes(str(avg_r), 'utf-8'), qos=0)
    client.publish(mqtt_feedname_OD_g, bytes(str(avg_g), 'utf-8'), qos=0)
    client.publish(mqtt_feedname_OD_b, bytes(str(avg_b), 'utf-8'), qos=0)
    client.publish(mqtt_feedname_OD_clear, bytes(str(avg_clear), 'utf-8'), qos=0)

    c_M = Conc_from_OD(avg_b)
    dV, predicted_cM, iterations = calculations.solve_for_dV(c_A, c_M, T, goal_c_M, V_A, V_M, alpha, beta)

    pump_volume(PUMP_B, dV)


async def pump_volume(pump, vol):
    # pump speed is 8.2 ml/second at pwm 50000 (mid power)
    print("Feeding")
    pump.duty_u16(50000)
    await uasyncio.sleep(vol * 0.12195)
    print("Stopping feedstock")
    pump.duty_u16(0)

# From Jacob
def Conc_from_OD(Blue):
    return -114.9880 * Blue + 248932.3866

### OD stuff
# Initialize control pins
led = Pin(27, Pin.OUT)
led_ctrl = Pin(33, Pin.OUT)
pwr_rgb = Pin(14, Pin.OUT)
blue_led = Pin(21, Pin.OUT)

# Power up sensors
led_ctrl.value(0)
led.value(1)
pwr_rgb.value(1)
blue_led.value(1)

###Pumps

PUMP_A = PWM(Pin(26, Pin.OUT)) # A0
PUMP_B = PWM(Pin(25, Pin.OUT)) # A1

def set_pump_rate(pump, rate):
    # Scale rate to account for 40% minimum threshold
    # rate should be 0-65535, but effective range is 40-100%
    if rate <= 26214:  # 40% of 65535
        pump.duty_u16(0)  # Turn off pump below 40%
    else:
        pump.duty_u16(rate)

###Web

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
ADAFRUIT_IO_KEY = b'abc'

client = MQTTClient(client_id=mqtt_client_id, 
                    server=ADAFRUIT_IO_URL, 
                    user=ADAFRUIT_USERNAME, 
                    password=ADAFRUIT_IO_KEY,
                    ssl=False,
                    port=1883)
try:       
    client.connect()
    print('Connected to client')
except Exception as e:
    print('could not connect to MQTT server {}{}'.format(type(e).__name__, e))
    sys.exit()

# publish free heap statistics to Adafruit IO using MQTT

def get_feed_value(feed_key):
    url = f'https://io.adafruit.com/api/v2/{ADAFRUIT_USERNAME.decode()}/feeds/{feed_key}/data/last'
    headers = {'X-AIO-Key': ADAFRUIT_IO_KEY}
    response = urequests.get(url, headers=headers)
    value = response.json()['value']
    response.close()
    try:
        if '.' in value:
            return float(value)
        else:
            return int(value)
    except ValueError:
        raise ValueError("Input string is not a valid int or float")

def cb(topic, msg):
    global pump_duration, wait, measurement_count, measurement_interval
    try:
        slider_value = msg.decode('utf-8')
        print('Slider value received:', slider_value)

        print("Requesting parameters..")
        pump_duration, wait, measurement_count, measurement_interval = [get_feed_value(x) for x in ['out.pump-duration', 'wait', 'out.measurement-count', 'out.measurement-duration']]
        
    except Exception as e:
        print('Error processing slider value:', e)

# format of feed name:  
#   "ADAFRUIT_USERNAME/feeds/ADAFRUIT_IO_FEEDNAME"
mqtt_feedname_temp = bytes('{:s}/feeds/{:s}'.format(ADAFRUIT_USERNAME, b'temperature'), 'utf-8')
mqtt_feedname_OD_r = bytes('{:s}/feeds/{:s}'.format(ADAFRUIT_USERNAME, b'od.red'), 'utf-8')
mqtt_feedname_OD_g = bytes('{:s}/feeds/{:s}'.format(ADAFRUIT_USERNAME, b'od.green'), 'utf-8')
mqtt_feedname_OD_b = bytes('{:s}/feeds/{:s}'.format(ADAFRUIT_USERNAME, b'od.blue'), 'utf-8')
mqtt_feedname_OD_clear = bytes('{:s}/feeds/{:s}'.format(ADAFRUIT_USERNAME, b'od.clear'), 'utf-8')
mqtt_feedname_set = bytes('{:s}/feeds/{:s}'.format(ADAFRUIT_USERNAME, b'out.set'), 'utf-8')
client.set_callback(cb)
client.subscribe(mqtt_feedname_set)

###init thermistor
#temp_sens = read_temp.init_temp_sensor()

###cooling
cooler = cooling.CoolingSystem(cooling.TemperatureSystem)
set_pump_rate(PUMP_A, 65535)
is_cooling = False

with open("data.txt", "a") as file:
    file.write("\t".join(["Time", "PID", "P", "I", "Mode", "Temp"])+"\n")

###sensor setup
i2c = I2C(0, scl=Pin(22), sda=Pin(23))
rgb_sensor = tcs34725.TCS34725(i2c)
rgb_sensor.gain(60)
rgb_sensor.integration_time(154)

###concentration calculations
V_A = 2000 # change this possibly (mL)
V_M = 5000 # mL
alpha = 1130
beta = 4.45/60*1000  # mL/min
c_A = 15000
T = 12   # min
goal_c_M = alpha

###loop
async def main_loop():
    global measurement_count, measurement_interval, pump_duration
    accum_time_a = COOLING_PERIOD_IN_SEC
    accum_time_b = 0
    accum_time_c = OD_PERIOD_IN_SEC
    start_time = time.time()
    while True:
        try:
            current_time = time.time()
            if accum_time_a>=COOLING_PERIOD_IN_SEC: #publish temperature and run cooling
                # Read temperature and adjust PID
                pid_out, P_out, I_out, power_mode_out, temperature = cooler.run_cooling_system(target_temperature)
                #print("Recorded temperature:", temperature)
                with open("data.txt", "a") as file:
                    file.write("\t".join(str(x) for x in [time.time()-start_time, pid_out, P_out, I_out, power_mode_out, temperature])+"\n")

                client.publish(mqtt_feedname_temp, bytes(str(temperature), 'utf-8'), qos=0)

                accum_time_a = -TICK_PERIOD_IN_SEC
                accum_time_b = -TICK_PERIOD_IN_SEC
                is_cooling = True

            if accum_time_b>=COOLING_PERIOD_IN_SEC*cooler.duty_cycle and is_cooling: #turn off cooling after some time
                cooler.disable_cooling_system()
                accum_time_b = -TICK_PERIOD_IN_SEC
                is_cooling = False

            if accum_time_c>=wait+pump_duration+measurement_count*measurement_interval:
                uasyncio.create_task(measure_OD(measurement_count, measurement_interval, pump_duration))
                #measure_OD(measurement_count, measurement_interval, pump_duration)
                accum_time_c = -TICK_PERIOD_IN_SEC
            
            client.check_msg()
            
            await uasyncio.sleep(TICK_PERIOD_IN_SEC)
            accum_time = time.time()-current_time
            accum_time_a += accum_time
            accum_time_b += accum_time
            accum_time_c += accum_time
            print(accum_time_c)
        except KeyboardInterrupt:
            print('Ctrl-C pressed...exiting')
            client.disconnect()
            sys.exit()

print("Requesting parameters..")
pump_duration, wait, measurement_count, measurement_interval = [get_feed_value(x) for x in ['out.pump-duration', 'out.wait', 'out.measurement-count', 'out.measurement-duration']]
target_temperature = 18
COOLING_PERIOD_IN_SEC = 10
OD_PERIOD_IN_SEC = wait
TICK_PERIOD_IN_SEC = 0.5 

print('Publishing..')
uasyncio.run(main_loop())
