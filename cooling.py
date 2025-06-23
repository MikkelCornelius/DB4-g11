from machine import Pin, PWM, ADC
from math import log
from micropython import const
import time

PUMP_A = PWM(Pin(26, Pin.OUT)) # A0
PUMP_B = PWM(Pin(25, Pin.OUT)) # A1
PUMP_B.duty_u16(0)

# Modified function for 12V diaphragm pumps
def set_pump_rate(pump, rate):
    # Scale rate to account for 50% minimum threshold
    # rate should be 0-65535, but effective range is 40-100%
    if rate <= 48214:  # 50% of 65535
        pump.duty_u16(0)  # Turn off pump below 40%
    else:
        pump.duty_u16(rate)

TEMPERATURE_PIN = 32

class TemperatureSystem:
    def __init__(self):
        self.TEMP_SENS_ADC_PIN_NO = TEMPERATURE_PIN
        self.NOM_RES = 11310 # Thermistor resistance at room temperature
        self.SER_RES = 10000  # Serial resistor resistance
        self.TEMP_NOM = 25
        self.NUM_SAMPLES = 50
        self.THERM_B_COEFF = 3950
        self.ADC_MAX = 1023
        self.ADC_Vmax = 3.15

        self.adc_V_lookup = [0.01235294, 0.01852941, 0.03705883, 0.04117647, 0.04529412, 0.04941177, 0.0525, 0.05558824, 0.05867648, 0.06176471, 0.06588235, 0.07, 0.07411765, 0.07720589, 0.08029412, 0.08338236, 0.08647059, 0.08894118, 0.09141177, 0.09388235, 0.09635295, 0.09882354, 0.1029412, 0.1070588, 0.1111765, 0.1142647, 0.117353, 0.1204412, 0.1235294, 0.1266177, 0.1297059, 0.1327941, 0.1358824, 0.14, 0.1441177, 0.1482353, 0.1513235, 0.1544118, 0.1575, 0.1605882, 0.1636765, 0.1667647, 0.1698529, 0.1729412, 0.1770588, 0.1811765, 0.1852941, 0.1877647, 0.1902353, 0.1927059, 0.1951765, 0.1976471, 0.2007353, 0.2038235, 0.2069118, 0.21, 0.2130883, 0.2161765, 0.2192647, 0.222353, 0.2264706, 0.2305882, 0.2347059, 0.2377941, 0.2408824, 0.2439706, 0.2470588, 0.2501471, 0.2532353, 0.2563236, 0.2594118, 0.2625, 0.2655883, 0.2686765, 0.2717647, 0.2758824, 0.28, 0.2841177, 0.2872059, 0.2902942, 0.2933824, 0.2964706, 0.2995588, 0.3026471, 0.3057353, 0.3088235, 0.3119118, 0.315, 0.3180882, 0.3211765, 0.3252941, 0.3294118, 0.3335294, 0.3366177, 0.3397059, 0.3427941, 0.3458824, 0.3489706, 0.3520588, 0.3551471, 0.3582353, 0.3613235, 0.3644118, 0.3675, 0.3705883, 0.3736765, 0.3767647, 0.379853, 0.3829412, 0.3891177, 0.3952941, 0.3983824, 0.4014706, 0.4045588, 0.4076471, 0.4107353, 0.4138236, 0.4169118, 0.42, 0.4230883, 0.4261765, 0.4292647, 0.432353, 0.4364706, 0.4405883, 0.4447059, 0.4467648, 0.4488235, 0.4508824, 0.4529412, 0.455, 0.4570589, 0.4611764, 0.4652942, 0.4694118, 0.4714706, 0.4735294, 0.4755883, 0.4776471, 0.4797059, 0.4817647, 0.4879412, 0.4941177, 0.4982353, 0.502353, 0.5064706, 0.5095589, 0.5126471, 0.5157353, 0.5188236, 0.5219118, 0.525, 0.5280883, 0.5311765, 0.5342648, 0.537353, 0.5404412, 0.5435295, 0.547647, 0.5517647, 0.5558824, 0.5589706, 0.5620589, 0.5651471, 0.5682353, 0.5713236, 0.5744118, 0.5775001, 0.5805883, 0.5836765, 0.5867648, 0.589853, 0.5929412, 0.5960294, 0.5991177, 0.6022058, 0.6052941, 0.6083823, 0.6114706, 0.6145588, 0.6176471, 0.6207353, 0.6238235, 0.6269117, 0.63, 0.6330882, 0.6361765, 0.6392647, 0.642353, 0.6464706, 0.6505883, 0.6547059, 0.6577941, 0.6608824, 0.6639706, 0.6670588, 0.670147, 0.6732353, 0.6763235, 0.6794118, 0.6825, 0.6855883, 0.6886765, 0.6917647, 0.6948529, 0.6979412, 0.7010294, 0.7041177, 0.7072059, 0.7102942, 0.7133823, 0.7164706, 0.7205883, 0.7247059, 0.7288236, 0.7319118, 0.735, 0.7380882, 0.7411765, 0.7452941, 0.7494118, 0.7535295, 0.7555883, 0.757647, 0.7597059, 0.7617648, 0.7638235, 0.7658824, 0.77, 0.7741177, 0.7782353, 0.7813235, 0.7844118, 0.7875, 0.7905883, 0.7936765, 0.7967648, 0.7998529, 0.8029412, 0.8060294, 0.8091177, 0.8122059, 0.8152942, 0.8183824, 0.8214706, 0.8245588, 0.8276471, 0.8301176, 0.8325883, 0.8350588, 0.8375295, 0.8400001, 0.8441177, 0.8482353, 0.852353, 0.8548235, 0.8572942, 0.8597647, 0.8622354, 0.8647059, 0.8677941, 0.8708824, 0.8739706, 0.8770589, 0.8801471, 0.8832354, 0.8863235, 0.8894118, 0.8935295, 0.8976471, 0.9017648, 0.9042353, 0.906706, 0.9091764, 0.9116471, 0.9141177, 0.9182354, 0.9223529, 0.9264707, 0.9295588, 0.9326471, 0.9357353, 0.9388236, 0.9419118, 0.9450001, 0.9480883, 0.9511765, 0.9552942, 0.9594117, 0.9635295, 0.9666177, 0.969706, 0.9727942, 0.9758824, 0.9783529, 0.9808236, 0.9832941, 0.9857648, 0.9882354, 0.9913236, 0.9944118, 0.9975, 1.000588, 1.003677, 1.006765, 1.009853, 1.012941, 1.017059, 1.021176, 1.025294, 1.028382, 1.031471, 1.034559, 1.037647, 1.039706, 1.041765, 1.043824, 1.045882, 1.047941, 1.05, 1.056177, 1.062353, 1.065441, 1.068529, 1.071618, 1.074706, 1.077794, 1.080882, 1.083971, 1.087059, 1.091177, 1.095294, 1.099412, 1.101882, 1.104353, 1.106824, 1.109294, 1.111765, 1.115882, 1.12, 1.124118, 1.127206, 1.130294, 1.133382, 1.136471, 1.139559, 1.142647, 1.145735, 1.148824, 1.151912, 1.155, 1.158088, 1.161177, 1.164265, 1.167353, 1.170441, 1.17353, 1.179706, 1.185882, 1.188353, 1.190824, 1.193294, 1.195765, 1.198235, 1.202353, 1.206471, 1.210588, 1.213676, 1.216765, 1.219853, 1.222941, 1.226029, 1.229118, 1.232206, 1.235294, 1.238382, 1.241471, 1.244559, 1.247647, 1.250735, 1.253824, 1.256912, 1.26, 1.272353, 1.273897, 1.275441, 1.276985, 1.278529, 1.280074, 1.281618, 1.283162, 1.284706, 1.287794, 1.290882, 1.293971, 1.297059, 1.300147, 1.303235, 1.306324, 1.309412, 1.3125, 1.315588, 1.318676, 1.321765, 1.325882, 1.33, 1.334118, 1.337206, 1.340294, 1.343382, 1.346471, 1.350588, 1.354706, 1.358824, 1.361294, 1.363765, 1.366235, 1.368706, 1.371176, 1.374265, 1.377353, 1.380441, 1.383529, 1.386618, 1.389706, 1.392794, 1.395882, 1.398971, 1.402059, 1.405147, 1.408235, 1.410706, 1.413177, 1.415647, 1.418118, 1.420588, 1.426765, 1.432941, 1.435412, 1.437882, 1.440353, 1.442824, 1.445294, 1.449412, 1.453529, 1.457647, 1.460118, 1.462588, 1.465059, 1.46753, 1.47, 1.476177, 1.482353, 1.484824, 1.487294, 1.489765, 1.492235, 1.494706, 1.498824, 1.502941, 1.507059, 1.510147, 1.513235, 1.516324, 1.519412, 1.5225, 1.525588, 1.528677, 1.531765, 1.534853, 1.537941, 1.541029, 1.544118, 1.547206, 1.550294, 1.553382, 1.556471, 1.560588, 1.564706, 1.575, 1.577059, 1.585294, 1.593529, 1.596618, 1.599706, 1.602794, 1.605882, 1.608971, 1.612059, 1.615147, 1.618235, 1.622353, 1.626471, 1.630588, 1.633059, 1.635529, 1.638, 1.640471, 1.642941, 1.647059, 1.651177, 1.655294, 1.658382, 1.661471, 1.664559, 1.667647, 1.670735, 1.673824, 1.676912, 1.68, 1.682471, 1.684941, 1.687412, 1.689882, 1.692353, 1.695441, 1.698529, 1.701618, 1.704706, 1.707794, 1.710882, 1.713971, 1.717059, 1.720147, 1.723235, 1.726324, 1.729412, 1.733529, 1.737647, 1.741765, 1.744853, 1.747941, 1.751029, 1.754118, 1.756588, 1.759059, 1.76153, 1.764, 1.766471, 1.770588, 1.774706, 1.778824, 1.782941, 1.787059, 1.791177, 1.793647, 1.796118, 1.798588, 1.801059, 1.80353, 1.806618, 1.809706, 1.812794, 1.815882, 1.818971, 1.822059, 1.825147, 1.828235, 1.832353, 1.836471, 1.840588, 1.843677, 1.846765, 1.849853, 1.852941, 1.85603, 1.859118, 1.862206, 1.865294, 1.868382, 1.871471, 1.874559, 1.877647, 1.880735, 1.883824, 1.886912, 1.89, 1.894118, 1.898235, 1.902353, 1.905441, 1.908529, 1.911618, 1.914706, 1.917794, 1.920882, 1.923971, 1.927059, 1.930147, 1.933235, 1.936324, 1.939412, 1.943529, 1.947647, 1.951765, 1.954853, 1.957941, 1.96103, 1.964118, 1.968235, 1.972353, 1.976471, 1.980588, 1.984706, 1.988824, 1.991294, 1.993765, 1.996235, 1.998706, 2.001177, 2.005294, 2.009412, 2.01353, 2.016618, 2.019706, 2.022794, 2.025882, 2.028971, 2.032059, 2.035147, 2.038235, 2.041324, 2.044412, 2.0475, 2.050588, 2.053677, 2.056765, 2.059853, 2.062941, 2.067059, 2.071177, 2.075294, 2.077765, 2.080235, 2.082706, 2.085176, 2.087647, 2.091765, 2.095882, 2.1, 2.112353, 2.113897, 2.115441, 2.116985, 2.11853, 2.120074, 2.121618, 2.123162, 2.124706, 2.127794, 2.130883, 2.133971, 2.137059, 2.141176, 2.145294, 2.149412, 2.1525, 2.155588, 2.158677, 2.161765, 2.164235, 2.166706, 2.169177, 2.171647, 2.174118, 2.177206, 2.180294, 2.183383, 2.186471, 2.189559, 2.192647, 2.195735, 2.198824, 2.201912, 2.205, 2.208088, 2.211177, 2.215294, 2.219412, 2.22353, 2.226618, 2.229706, 2.232794, 2.235883, 2.238971, 2.242059, 2.245147, 2.248235, 2.251324, 2.254412, 2.2575, 2.260588, 2.263677, 2.266765, 2.269853, 2.272941, 2.27603, 2.279118, 2.282206, 2.285294, 2.288383, 2.291471, 2.294559, 2.297647, 2.300735, 2.303824, 2.306912, 2.31, 2.314118, 2.318235, 2.322353, 2.326471, 2.330588, 2.334706, 2.337177, 2.339647, 2.342118, 2.344588, 2.347059, 2.34953, 2.352, 2.354471, 2.356941, 2.359412, 2.3625, 2.365588, 2.368677, 2.371765, 2.374853, 2.377941, 2.381029, 2.384118, 2.387206, 2.390294, 2.393382, 2.396471, 2.399559, 2.402647, 2.405735, 2.408823, 2.412941, 2.417059, 2.421176, 2.423647, 2.426118, 2.428588, 2.431059, 2.433529, 2.436, 2.438471, 2.440941, 2.443412, 2.445882, 2.45, 2.454118, 2.458235, 2.461323, 2.464412, 2.4675, 2.470588, 2.473059, 2.475529, 2.478, 2.480471, 2.482941, 2.487059, 2.491177, 2.495294, 2.497765, 2.500235, 2.502706, 2.505177, 2.507647, 2.510118, 2.512588, 2.515059, 2.517529, 2.52, 2.523088, 2.526176, 2.529265, 2.532353, 2.535441, 2.538529, 2.541618, 2.544706, 2.547794, 2.550882, 2.553971, 2.557059, 2.55953, 2.562, 2.564471, 2.566941, 2.569412, 2.5725, 2.575588, 2.578676, 2.581765, 2.584235, 2.586706, 2.589177, 2.591647, 2.594118, 2.597206, 2.600294, 2.603382, 2.606471, 2.609559, 2.612647, 2.615735, 2.618824, 2.620883, 2.622941, 2.625, 2.627059, 2.629118, 2.631176, 2.635294, 2.639412, 2.643529, 2.646618, 2.649706, 2.652794, 2.655882, 2.658971, 2.662059, 2.665147, 2.668235, 2.670706, 2.673177, 2.675647, 2.678118, 2.680588, 2.683676, 2.686765, 2.689853, 2.692941, 2.695412, 2.697882, 2.700353, 2.702824, 2.705294, 2.708382, 2.711471, 2.714559, 2.717647, 2.720118, 2.722588, 2.725059, 2.72753, 2.73, 2.732471, 2.734941, 2.737412, 2.739882, 2.742353, 2.745441, 2.748529, 2.751618, 2.754706, 2.757794, 2.760882, 2.763971, 2.767059, 2.770147, 2.773235, 2.776324, 2.779412, 2.781471, 2.78353, 2.785588, 2.787647, 2.789706, 2.791765, 2.794235, 2.796706, 2.799177, 2.801647, 2.804118, 2.806588, 2.809059, 2.81153, 2.814, 2.816471, 2.818941, 2.821412, 2.823883, 2.826353, 2.828824, 2.831294, 2.833765, 2.836236, 2.838706, 2.841177, 2.842941, 2.844706, 2.846471, 2.848235, 2.85, 2.851765, 2.853529, 2.856, 2.858471, 2.860941, 2.863412, 2.865882, 2.867941, 2.87, 2.872059, 2.874118, 2.876177, 2.878235, 2.880706, 2.883177, 2.885647, 2.888118, 2.890588, 2.892647, 2.894706, 2.896765, 2.898824, 2.900883, 2.902941, 2.905412, 2.907882, 2.910353, 2.912824, 2.915294, 2.917765, 2.920235, 2.922706, 2.925177, 2.927647, 2.929412, 2.931177, 2.932941, 2.934706, 2.936471, 2.938235, 2.94, 2.941765, 2.94353, 2.945294, 2.947059, 2.948823, 2.950588, 2.952353, 2.954412, 2.956471, 2.958529, 2.960588, 2.962647, 2.964706, 2.966471, 2.968235, 2.97, 2.971765, 2.973529, 2.975294, 2.977059, 2.979118, 2.981177, 2.983235, 2.985294, 2.987353, 2.989412, 2.991471, 2.99353, 2.995588, 2.997647, 2.999706, 3.001765, 3.00353, 3.005294, 3.007059, 3.008824, 3.010588, 3.012353, 3.014118, 3.016177, 3.018235, 3.020294, 3.022353, 3.024412, 3.026471, 3.028235, 3.03, 3.031765, 3.03353, 3.035294, 3.037059, 3.038824, 3.040588, 3.042353, 3.044118, 3.045882, 3.047647, 3.049412, 3.051177, 3.053236, 3.055294, 3.057353, 3.059412, 3.061471, 3.063529, 3.065074, 3.066618, 3.068162, 3.069706, 3.07125, 3.072794, 3.074338, 3.075882, 3.077647, 3.079412, 3.081177, 3.082941, 3.084706, 3.086471, 3.088235, 3.092353, 3.096471, 3.125294]

        self.temp_sensor = ADC(Pin(self.TEMP_SENS_ADC_PIN_NO))
        self.temp_sensor.atten(ADC.ATTN_11DB)
        self.temp_sensor.width(ADC.WIDTH_10BIT)

    def read_temp(self):
        raw_read = []

        # Collect NUM_SAMPLES
        for i in range(1, self.NUM_SAMPLES+1):
            raw_read.append(self.temp_sensor.read())

        # Average of the NUM_SAMPLES and look it up in the table
        raw_average = sum(raw_read)/self.NUM_SAMPLES
        #print('raw_avg = ' + str(raw_average))
        #print('V_measured = ' + str(adc_V_lookup[round(raw_average)]))

        # Convert to resistance
        raw_average = self.ADC_MAX * self.adc_V_lookup[round(raw_average)]/self.ADC_Vmax
        resistance = (self.SER_RES * raw_average) / (self.ADC_MAX - raw_average)
        #print('Thermistor resistance: {} ohms'.format(resistance))

        # Convert to temperature
        steinhart  = log(resistance / self.NOM_RES) / self.THERM_B_COEFF
        steinhart += 1.0 / (self.TEMP_NOM + 273.15)
        steinhart  = (1.0 / steinhart) - 273.15
        return steinhart

# Pins
RELAY1_PIN = const(13)
RELAY2_PIN = const(12)

# Constants
MIN_DUTY_CYCLE = const(0.1)  # Minimum duty cycle (10%)
MAX_DUTY_CYCLE = const(1.0)  # Maximum duty cycle (100%)

# Cooling power thresholds (as % of max cooling capacity)
LOW_POWER_MAX = const(0.5)   # Switch to high power when needed power > 50%
HIGH_POWER_MIN = const(0.3)  # Switch back to low power when needed power < 30%

MAX_CONTINUOUS_RUN_TIME = const(3600*100)  # 1 hour in milliseconds
MIN_DEFROST_TIME = const(180*100)        # 5 minutes minimum off time

# PID Constants
KP = const(1)
KI = const(0.0001)
KD = const(0)
Imax = 1

class CoolingSystem(TemperatureSystem):
    def __init__(self, TemperatureSystem):
        self.relay1 = Pin(RELAY1_PIN, Pin.OUT)
        self.relay2 = Pin(RELAY2_PIN, Pin.OUT)

        # Cooling state
        self.current_power_mode = 'off'  # 'off', 'low', 'high'
        self.duty_cycle = 0
        self.last_switch_time = 0
        self.min_mode_time = 5000  # Minimum time in a power mode (ms)

        self.last_error = 0
        self.integral = 0
        self.last_time = 0

        # self.last_run_start = 0
        # self.continuous_run_time = 0
        # self.force_off_until = 0
        # self.anti_freeze_active = False

        #for the running function
        TemperatureSystem.__init__(self)

    def pid_control(self, current_temp, target_temp):
        """Calculate required cooling power (0-1) using PID"""
        now = time.ticks_ms()
        dt = time.ticks_diff(now, self.last_switch_time) / 1000.0  # Convert to seconds
        if dt <= 0:
            dt = 0.01  # Prevent division by zero

        error = current_temp - target_temp
       # Proportional term
        P = KP * error

        # Integral term (only active near setpoint + clamped to ±1)
        if abs(error) <= 1.2:  # Only integrate if within 1.2°C of target
            self.integral += error * dt
            self.integral = max(min(self.integral, 1.0), -1.0)  # Hard clamp to ±1
        else:
            self.integral = 0  # Reset if outside the range

        I = KI * self.integral

        # Derivative term
        D = KD * (error - self.last_error) / dt

        # Calculate output (0-1 range)
        PID = P + I + D
        output = max(-0.5, min(1, PID))  # Clamp to 0-1 range

        #P_values.append(P)
        #I_values.append(I)
        #D_values.append(D)

        # Update state
        self.last_error = error
        self.last_time = now
        #print(f"PID constants {KP, KI, KD}")
        #print(f"PID value {output}, P + I + D {P, I, D, PID}")

        return output, PID, P, I
    
    def disable_cooling_system(self):
        for relay in self.relays_on:
            relay.off()

    def apply_duty_cycle(self):
        """Apply the current duty cycle to the relays"""
        if self.duty_cycle <= 0:
            self.relay1.off()
            self.relay2.off()
            return

        # Determine which relays should be on based on power mode
        relays_on = []
        if self.current_power_mode == 'low':
            relays_on = [self.relay1]
        elif self.current_power_mode == 'high' or 'eco':
            relays_on = [self.relay1, self.relay2]

        # Simple duty cycle implementation (for real application, use timers)
        # This is a simplified version - in practice you'd want to use proper timing
        cycle_time = 10  # seconds (adjust based on your needs)
        on_time = cycle_time * self.duty_cycle

        for relay in relays_on:
            relay.on()

    def set_cooling(self, required_power, PID):
        #power_mode_values.append(self.current_power_mode)

        """Set cooling system with anti-freeze protection"""
        now = time.ticks_ms()

        # # Check if we're in forced off period
        # if now < self.force_off_until:
        #     if self.current_power_mode != 'off':
        #         self.relay1.off()
        #         self.relay2.off()
        #         self.current_power_mode = 'off'
        #         print("Anti-freeze: Cooling forced off")
        #     return

        # Track continuous run time
        # if self.current_power_mode != 'off':
        #     self.continuous_run_time = time.ticks_diff(now, self.last_run_start)
        #     if self.continuous_run_time >= MAX_CONTINUOUS_RUN_TIME:
        #         self.anti_freeze_active = True
        #         self.force_off_until = now + MIN_DEFROST_TIME
        #         self.relay1.off()
        #         self.relay2.off()
        #         self.current_power_mode = 'off'
        #         print(f"Anti-freeze: Max run time reached ({MAX_CONTINUOUS_RUN_TIME/3600000}h), cooling disabled for {MIN_DEFROST_TIME/3600000}h")
        #         return
        # else:
        #     self.last_run_start = now
        #     self.continuous_run_time = 0
        #     self.anti_freeze_active = False

        # Original power mode selection logic
        new_power_mode = self.current_power_mode
        if time.ticks_diff(now, self.last_switch_time) > self.min_mode_time:
            if required_power > LOW_POWER_MAX and self.current_power_mode != 'high':
                if PID > 2.3:
                    new_power_mode = 'high'
            elif required_power > LOW_POWER_MAX and self.current_power_mode != 'eco':
                if PID <= 2.3:
                    new_power_mode = 'eco'
            elif required_power < HIGH_POWER_MIN and self.current_power_mode != 'low':
                new_power_mode = 'low'
            elif required_power <= -0.3:
                new_power_mode = 'off'

        # Apply new power mode if changed
        if new_power_mode != self.current_power_mode:
            self.current_power_mode = new_power_mode
            self.last_switch_time = now
            print(f"Switching to {self.current_power_mode} power mode")

#CHANGES HERE
        # Calculate and apply duty cycle (from previous implementation)
        if self.current_power_mode == 'off':
            effective_power = 0
            self.duty_cycle = 0
            set_pump_rate(PUMP_A, 0)
            print("pump off")
        # DON'T CHANGE THE PUMP RATES
        elif self.current_power_mode == 'low':
            effective_power = min(required_power / LOW_POWER_MAX, 1.0)
            self.duty_cycle = max(MIN_DUTY_CYCLE, effective_power)
            set_pump_rate(PUMP_A, 58982)
            print(f"pump on | low power | duty cycle {self.duty_cycle}")
        elif self.current_power_mode == 'eco':
            effective_power = (required_power - LOW_POWER_MAX) / (1 - LOW_POWER_MAX)
            self.duty_cycle = max(MIN_DUTY_CYCLE, effective_power)
            set_pump_rate(PUMP_A, 64225)
            print(f"pump on | eco power | duty cycle {self.duty_cycle}")
        else:  # high power mode
            effective_power = (required_power - LOW_POWER_MAX) / (1 - LOW_POWER_MAX)
            self.duty_cycle = max(MIN_DUTY_CYCLE, effective_power)
            set_pump_rate(PUMP_A, 65535)
            print(f"pump on | high power | duty cycle {self.duty_cycle}")

        self.apply_duty_cycle()

        return self.current_power_mode

    def run_cooling_system(self, target_temperature):

        current_temperature = self.read_temp()
        if  current_temperature - target_temperature <= -0.5:
            set_pump_rate(PUMP_A, 0)
            print("pump off")

        output, pid, P_values, I_values = self.pid_control(current_temperature, target_temperature)
        power_mode_values = self.set_cooling(round(output, 1), pid)

        # Record data
        #time_points.append(elapsed)
        #pid_values.append(output)
        #temperature_points.append(current_temperature)

        return output, P_values, I_values, power_mode_values, current_temperature

