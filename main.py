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
import machine


async def measure_OD(measurement_count, measurement_interval, pump_duration):
    global is_measuring_od_rn, last_feeding_time, c_M
    is_measuring_od_rn = True
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

    c_A = Conc_from_OD(avg_b)
    client.publish(mqtt_feedname_concentration, bytes(str(c_A), 'utf-8'), qos=0)
    print(f"t = {time.ticks_ms()/1000}, Red = {avg_r}, Blue = {avg_b}, c(Algae) = {c_A}")
    file_od.write("\t".join(str(x) for x in [time.ticks_ms(), avg_b, c_A])+"\n")
    file_od.flush()
    if time.ticks_ms() - last_feeding_time > (T*60000):
        if not auto_control: # manual
            print(f"t = {time.ticks_ms()/1000}, Feeding: dV = {pump_duration / 0.054}")
            await pump_volume(PUMP_B, pump_duration / 0.054)
        else:
            dV, immediate_cM, predicted_cM, iterations = calculations.solve_for_dV(c_A, c_M, T, goal_c_M, V_A, V_M, alpha, beta)
            print(f"t = {time.ticks_ms()/1000}, Feeding: dV = {dV}, immediate_cM = {immediate_cM}, predicted_cm = {predicted_cM}, period = {T*60} s")
            if dV>0 and dV<300000:
                file_od.write("\t".join(str(x) for x in [time.ticks_ms(), avg_b, c_A, c_M, immediate_cM, dV])+"\n")
                file_od.flush()
                await pump_volume(PUMP_B, dV)
                c_M = predicted_cM # set global to predicted
            else:
                print("simulation failed: got negative result")
            last_feeding_time = time.ticks_ms()
    is_measuring_od_rn = False


async def pump_volume(pump, vol):
    # pump speed is 18.5 ml/second at max power
    print("Feeding")
    pump.duty_u16(65535)
    await uasyncio.sleep(vol * 0.054)
    print("Stopping feedstock")
    pump.duty_u16(0)

def Conc_from_OD(Blue):
    return ( -114.9880 * Blue + 248932.3866 ) * 0.2067604 # Last minute scaling factor

def set_pump_rate(pump, rate):
    # Scale rate to account for 40% minimum threshold
    # rate should be 0-65535, but effective range is 40-100%
    if rate <= 26214:  # 40% of 65535
        pump.duty_u16(0)  # Turn off pump below 40%
    else:
        pump.duty_u16(rate)

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

#Pumps
#PUMP_A = PWM(Pin(26, Pin.OUT)) # A0
PUMP_B = PWM(Pin(25, Pin.OUT)) # A1
PUMP_B.duty_u16(0)

# Led strip
led_strip = PWM(Pin(17, Pin.OUT))
#led_strip.duty_u16(16384) # 25 percent strength

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
    global pump_duration, T, auto_control
    try:
        msg_value = msg.decode('utf-8')
        #print('Message received:', msg_value)

        if topic == f'{ADAFRUIT_USERNAME.decode()}/feeds/out.manual'.encode():
            if msg_value==0:
                print("Switching to manual control")
            else:
                print("Switching to auto feeding")
            auto_control = msg_value==1
        elif topic == f'{ADAFRUIT_USERNAME.decode()}/feeds/out.pump-duration'.encode():
            pump_duration = int(msg_value)
            print(f"Setting `pump_duration` to {pump_duration}")
        elif topic == f'{ADAFRUIT_USERNAME.decode()}/feeds/out.wait'.encode():
            T = int(msg_value)
            print(f"Setting `T` to {T}")
        elif topic == f'{ADAFRUIT_USERNAME.decode()}/feeds/out.light-strip'.encode():
            light_duty = round(int(msg_value) * 655.35)
            led_strip.duty_u16(light_duty)
            print(f"Setting `light_duty` to {light_duty}")
        else: #error
            print("unrecognized topic")
            print("Topic:", topic)
        
    except Exception as e:
        print('Error processing slider value:', e)

# format of feed name:
mqtt_feedname_temp = bytes('{:s}/feeds/{:s}'.format(ADAFRUIT_USERNAME, b'temperature'), 'utf-8')
mqtt_feedname_OD_r = bytes('{:s}/feeds/{:s}'.format(ADAFRUIT_USERNAME, b'od.red'), 'utf-8')
mqtt_feedname_OD_g = bytes('{:s}/feeds/{:s}'.format(ADAFRUIT_USERNAME, b'od.green'), 'utf-8')
mqtt_feedname_OD_b = bytes('{:s}/feeds/{:s}'.format(ADAFRUIT_USERNAME, b'od.blue'), 'utf-8')
mqtt_feedname_concentration = bytes('{:s}/feeds/{:s}'.format(ADAFRUIT_USERNAME, b'od.concentration'), 'utf-8')
#mqtt_feedname_OD_clear = bytes('{:s}/feeds/{:s}'.format(ADAFRUIT_USERNAME, b'od.clear'), 'utf-8')
#mqtt_feedname_set = bytes('{:s}/feeds/{:s}'.format(ADAFRUIT_USERNAME, b'out.set'), 'utf-8')
mqtt_feedname_pump_duration = bytes('{:s}/feeds/{:s}'.format(ADAFRUIT_USERNAME, b'out.pump-duration'), 'utf-8')
mqtt_feedname_pump_wait = bytes('{:s}/feeds/{:s}'.format(ADAFRUIT_USERNAME, b'out.wait'), 'utf-8')
mqtt_feedname_manual = bytes('{:s}/feeds/{:s}'.format(ADAFRUIT_USERNAME, b'out.manual'), 'utf-8')
mqtt_feedname_light = bytes('{:s}/feeds/{:s}'.format(ADAFRUIT_USERNAME, b'out.light-strip'), 'utf-8')

# subscribe
client.set_callback(cb)
client.subscribe(mqtt_feedname_pump_duration)
client.subscribe(mqtt_feedname_pump_wait)
client.subscribe(mqtt_feedname_manual)
client.subscribe(mqtt_feedname_light)

###cooling
cooler = cooling.CoolingSystem(cooling.TemperatureSystem)
is_cooling = False

file_pid = open("data_pid.txt", "a")
file_pid.write("\t".join(["Time", "PID", "P", "I", "Mode", "Temp"])+"\n")

file_od = open("data_od.txt", "a")
file_od.write("\t".join(["Time", "B_sensor", "cA", "cM_calculated", "cM_postfeed", "dV"])+"\n")

###sensor setup
i2c = I2C(0, scl=Pin(22), sda=Pin(23))
rgb_sensor = tcs34725.TCS34725(i2c)
rgb_sensor.gain(60)
rgb_sensor.integration_time(154)

###concentration calculations
V_A = 4000 # change this possibly (mL)
V_M = 4000 # mL
alpha = 1130
beta = 4.45/60*1000  # mL/min
c_M = 1000 # initially zero
goal_c_M = alpha - 100

auto_control = True
pump_duration = 4
T = 30   # min
light_duty = 16384

is_measuring_od_rn = False
last_feeding_time = -(T*60000)

###loop
async def main_loop():
    global measurement_count, measurement_interval, pump_duration, is_measuring_od_rn
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

                file_pid.write("\t".join(str(x) for x in [time.time()-start_time, pid_out, P_out, I_out, power_mode_out, temperature])+"\n")
                file_pid.flush()

                print("publishing temperature:", temperature)
                client.publish(mqtt_feedname_temp, bytes(str(temperature), 'utf-8'), qos=0)

                accum_time_a = -TICK_PERIOD_IN_SEC
                accum_time_b = -TICK_PERIOD_IN_SEC
                is_cooling = True

            if accum_time_b>=COOLING_PERIOD_IN_SEC*cooler.duty_cycle and is_cooling: #turn off cooling after some time
                cooler.disable_cooling_system()
                accum_time_b = -TICK_PERIOD_IN_SEC
                is_cooling = False

            if accum_time_c>=wait+pump_duration+measurement_count*measurement_interval and not is_measuring_od_rn: #run feeding pump and publish OD measurements
                is_measuring_od_rn = True
                uasyncio.create_task(measure_OD(measurement_count, measurement_interval, pump_duration))
                accum_time_c = -TICK_PERIOD_IN_SEC
            
            client.check_msg()
            
            await uasyncio.sleep(TICK_PERIOD_IN_SEC)
            accum_time = time.time()-current_time
            accum_time_a += accum_time
            accum_time_b += accum_time
            accum_time_c += accum_time
            #print(accum_time_c)
        except KeyboardInterrupt:
            print('Ctrl-C pressed...exiting')
            client.disconnect()
            sys.exit()

print("Requesting parameters..")
try:
    pump_duration = get_feed_value('out.pump-duration')
    T = get_feed_value('out.wait')
    auto_control = get_feed_value('out.manual')
    light_duty = round(get_feed_value('out.light-strip') * 655.35)
except Exception as e:
    print("Failed requesting parameters. Defaulting to placeholder values")
    print("Error report:", e)
    auto_control = True
    pump_duration = 4
    T = 30   # min
    light_duty = 16384

led_strip.duty_u16(light_duty)
wait = 60
measurement_count = 20
measurement_interval = 0.1
target_temperature = 18
COOLING_PERIOD_IN_SEC = 10
OD_PERIOD_IN_SEC = wait
TICK_PERIOD_IN_SEC = 0.5 

print('Publishing..')
uasyncio.run(main_loop())
