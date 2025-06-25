from machine import Pin, PWM, ADC
from math import log
from micropython import const
import time

PUMP_A = PWM(Pin(26, Pin.OUT)) # A0

# Modified function for 12V diaphragm pumps
def set_pump_rate(pump, rate):
    # Scale rate to account for 50% minimum threshold
    # rate should be 0-65535, but effective range is 40-100%
    if rate <= 45214:  # 50% of 65535
        pump.duty_u16(0)  # Turn off pump below 40%
    else:
        pump.duty_u16(rate)

TEMPERATURE_PIN = 32

class TemperatureSystem:
    def __init__(self):
        self.TEMP_SENS_ADC_PIN_NO = TEMPERATURE_PIN
        self.NOM_RES = 9700 # Thermistor resistance at room temperature
        self.SER_RES = 10000  # Serial resistor resistance
        self.TEMP_NOM = 25
        self.NUM_SAMPLES = 50
        self.THERM_B_COEFF = 4900
        self.ADC_MAX = 1023
        self.ADC_Vmax = 3.15

        self.adc_V_lookup = [0.01235294, 0.03705883, 0.04117647, 0.04529412, 0.04941177, 0.0525, 0.05558824, 0.05867648, 0.06176471, 0.06485295, 0.06794118, 0.07102942, 0.07411765, 0.07720589, 0.08029412, 0.08338236, 0.08647059, 0.08955883, 0.09264707, 0.0957353, 0.09882354, 0.1029412, 0.1070588, 0.1111765, 0.1136471, 0.1161176, 0.1185882, 0.1210588, 0.1235294, 0.1276471, 0.1317647, 0.1358824, 0.1389706, 0.1420588, 0.1451471, 0.1482353, 0.1523529, 0.1564706, 0.1605882, 0.1630588, 0.1655294, 0.168, 0.1704706, 0.1729412, 0.1770588, 0.1811765, 0.1852941, 0.1883824, 0.1914706, 0.1945588, 0.1976471, 0.2007353, 0.2038235, 0.2069118, 0.21, 0.2130883, 0.2161765, 0.2192647, 0.222353, 0.2254412, 0.2285294, 0.2316177, 0.2347059, 0.2377941, 0.2408824, 0.2439706, 0.2470588, 0.2511765, 0.2552941, 0.2594118, 0.2625, 0.2655883, 0.2686765, 0.2717647, 0.274853, 0.2779412, 0.2810294, 0.2841177, 0.2872059, 0.2902942, 0.2933824, 0.2964706, 0.3005883, 0.3047059, 0.3088235, 0.3112941, 0.3137647, 0.3162353, 0.3187059, 0.3211765, 0.3252941, 0.3294118, 0.3335294, 0.3366177, 0.3397059, 0.3427941, 0.3458824, 0.3489706, 0.3520588, 0.3551471, 0.3582353, 0.362353, 0.3664706, 0.3705883, 0.3730588, 0.3755294, 0.378, 0.3804706, 0.3829412, 0.3891177, 0.3952941, 0.3983824, 0.4014706, 0.4045588, 0.4076471, 0.4117647, 0.4158824, 0.42, 0.4230883, 0.4261765, 0.4292647, 0.432353, 0.4354412, 0.4385294, 0.4416177, 0.4447059, 0.4477942, 0.4508824, 0.4539706, 0.4570589, 0.4595294, 0.462, 0.4644706, 0.4669412, 0.4694118, 0.4725, 0.4755883, 0.4786765, 0.4817647, 0.4858823, 0.4900001, 0.4941177, 0.4972059, 0.5002942, 0.5033824, 0.5064706, 0.5095589, 0.5126471, 0.5157353, 0.5188236, 0.5219118, 0.525, 0.5280883, 0.5311765, 0.5352941, 0.5394118, 0.5435295, 0.5466177, 0.5497059, 0.5527942, 0.5558824, 0.5589706, 0.5620589, 0.5651471, 0.5682353, 0.5713236, 0.5744118, 0.5775001, 0.5805883, 0.5836765, 0.5867648, 0.589853, 0.5929412, 0.5960294, 0.5991177, 0.6022058, 0.6052941, 0.6083823, 0.6114706, 0.6145588, 0.6176471, 0.6207353, 0.6238235, 0.6269117, 0.63, 0.6341177, 0.6382353, 0.642353, 0.6454412, 0.6485294, 0.6516176, 0.6547059, 0.6577941, 0.6608824, 0.6639706, 0.6670588, 0.670147, 0.6732353, 0.6763235, 0.6794118, 0.6825, 0.6855883, 0.6886765, 0.6917647, 0.6948529, 0.6979412, 0.7010294, 0.7041177, 0.7072059, 0.7102942, 0.7133823, 0.7164706, 0.7205883, 0.7247059, 0.7288236, 0.7319118, 0.735, 0.7380882, 0.7411765, 0.7442647, 0.747353, 0.7504412, 0.7535295, 0.7566176, 0.7597059, 0.7627941, 0.7658824, 0.7689706, 0.7720589, 0.7751471, 0.7782353, 0.7813235, 0.7844118, 0.7875, 0.7905883, 0.7936765, 0.7967648, 0.7998529, 0.8029412, 0.8060294, 0.8091177, 0.8122059, 0.8152942, 0.8183824, 0.8214706, 0.8245588, 0.8276471, 0.8317648, 0.8358824, 0.8400001, 0.8430882, 0.8461765, 0.8492647, 0.852353, 0.8548235, 0.8572942, 0.8597647, 0.8622354, 0.8647059, 0.8677941, 0.8708824, 0.8739706, 0.8770589, 0.8795294, 0.8820001, 0.8844706, 0.8869412, 0.8894118, 0.8935295, 0.8976471, 0.9017648, 0.904853, 0.9079412, 0.9110294, 0.9141177, 0.9172059, 0.9202942, 0.9233824, 0.9264707, 0.9305883, 0.9347058, 0.9388236, 0.9419118, 0.9450001, 0.9480883, 0.9511765, 0.9542647, 0.957353, 0.9604412, 0.9635295, 0.9666177, 0.969706, 0.9727942, 0.9758824, 0.9783529, 0.9808236, 0.9832941, 0.9857648, 0.9882354, 0.9923531, 0.9964705, 1.000588, 1.003677, 1.006765, 1.009853, 1.012941, 1.016029, 1.019118, 1.022206, 1.025294, 1.028382, 1.031471, 1.034559, 1.037647, 1.040735, 1.043824, 1.046912, 1.05, 1.054118, 1.058235, 1.062353, 1.064824, 1.067294, 1.069765, 1.072235, 1.074706, 1.077794, 1.080882, 1.083971, 1.087059, 1.091177, 1.095294, 1.099412, 1.1025, 1.105588, 1.108677, 1.111765, 1.114853, 1.117941, 1.121029, 1.124118, 1.127206, 1.130294, 1.133382, 1.136471, 1.139559, 1.142647, 1.145735, 1.148824, 1.151912, 1.155, 1.158088, 1.161177, 1.164265, 1.167353, 1.170441, 1.17353, 1.179706, 1.185882, 1.188971, 1.192059, 1.195147, 1.198235, 1.201324, 1.204412, 1.2075, 1.210588, 1.213676, 1.216765, 1.219853, 1.222941, 1.227059, 1.231176, 1.235294, 1.237765, 1.240235, 1.242706, 1.245177, 1.247647, 1.251765, 1.255882, 1.26, 1.266176, 1.272353, 1.273897, 1.275441, 1.276985, 1.278529, 1.280074, 1.281618, 1.283162, 1.284706, 1.287794, 1.290882, 1.293971, 1.297059, 1.301177, 1.305294, 1.309412, 1.3125, 1.315588, 1.318676, 1.321765, 1.324853, 1.327941, 1.331029, 1.334118, 1.337206, 1.340294, 1.343382, 1.346471, 1.349559, 1.352647, 1.355735, 1.358824, 1.362941, 1.367059, 1.371176, 1.374265, 1.377353, 1.380441, 1.383529, 1.386, 1.388471, 1.390941, 1.393412, 1.395882, 1.4, 1.404118, 1.408235, 1.411324, 1.414412, 1.4175, 1.420588, 1.423676, 1.426765, 1.429853, 1.432941, 1.436029, 1.439118, 1.442206, 1.445294, 1.449412, 1.453529, 1.457647, 1.460118, 1.462588, 1.465059, 1.46753, 1.47, 1.474118, 1.478235, 1.482353, 1.485441, 1.488529, 1.491618, 1.494706, 1.497794, 1.500882, 1.503971, 1.507059, 1.510147, 1.513235, 1.516324, 1.519412, 1.5225, 1.525588, 1.528677, 1.531765, 1.535882, 1.54, 1.544118, 1.547206, 1.550294, 1.553382, 1.556471, 1.564706, 1.572941, 1.581177, 1.568824, 1.575, 1.581177, 1.587353, 1.593529, 1.597647, 1.601765, 1.605882, 1.608971, 1.612059, 1.615147, 1.618235, 1.622353, 1.626471, 1.630588, 1.633677, 1.636765, 1.639853, 1.642941, 1.646029, 1.649118, 1.652206, 1.655294, 1.658382, 1.661471, 1.664559, 1.667647, 1.671765, 1.675882, 1.68, 1.682059, 1.684118, 1.686177, 1.688235, 1.690294, 1.692353, 1.696471, 1.700588, 1.704706, 1.707176, 1.709647, 1.712118, 1.714588, 1.717059, 1.721177, 1.725294, 1.729412, 1.7325, 1.735588, 1.738677, 1.741765, 1.745882, 1.75, 1.754118, 1.756588, 1.759059, 1.76153, 1.764, 1.766471, 1.769559, 1.772647, 1.775735, 1.778824, 1.781912, 1.785, 1.788088, 1.791177, 1.794265, 1.797353, 1.800441, 1.80353, 1.807647, 1.811765, 1.815882, 1.818971, 1.822059, 1.825147, 1.828235, 1.831324, 1.834412, 1.8375, 1.840588, 1.843677, 1.846765, 1.849853, 1.852941, 1.85603, 1.859118, 1.862206, 1.865294, 1.868382, 1.871471, 1.874559, 1.877647, 1.881765, 1.885882, 1.89, 1.893088, 1.896177, 1.899265, 1.902353, 1.905441, 1.908529, 1.911618, 1.914706, 1.917794, 1.920882, 1.923971, 1.927059, 1.931176, 1.935294, 1.939412, 1.9425, 1.945588, 1.948677, 1.951765, 1.954853, 1.957941, 1.96103, 1.964118, 1.968235, 1.972353, 1.976471, 1.980588, 1.984706, 1.988824, 1.991912, 1.995, 1.998088, 2.001177, 2.004265, 2.007353, 2.010441, 2.01353, 2.016618, 2.019706, 2.022794, 2.025882, 2.028971, 2.032059, 2.035147, 2.038235, 2.042353, 2.046471, 2.050588, 2.053677, 2.056765, 2.059853, 2.062941, 2.06603, 2.069118, 2.072206, 2.075294, 2.078382, 2.081471, 2.084559, 2.087647, 2.091765, 2.095882, 2.1, 2.112353, 2.113897, 2.115441, 2.116985, 2.11853, 2.120074, 2.121618, 2.123162, 2.124706, 2.127177, 2.129647, 2.132118, 2.134588, 2.137059, 2.141176, 2.145294, 2.149412, 2.1525, 2.155588, 2.158677, 2.161765, 2.164853, 2.167941, 2.17103, 2.174118, 2.176588, 2.179059, 2.18153, 2.184, 2.186471, 2.190588, 2.194706, 2.198824, 2.201912, 2.205, 2.208088, 2.211177, 2.214265, 2.217353, 2.220441, 2.22353, 2.226618, 2.229706, 2.232794, 2.235883, 2.238971, 2.242059, 2.245147, 2.248235, 2.251324, 2.254412, 2.2575, 2.260588, 2.264706, 2.268824, 2.272941, 2.275412, 2.277882, 2.280353, 2.282824, 2.285294, 2.289412, 2.29353, 2.297647, 2.300735, 2.303824, 2.306912, 2.31, 2.313088, 2.316177, 2.319265, 2.322353, 2.325441, 2.32853, 2.331618, 2.334706, 2.337794, 2.340883, 2.343971, 2.347059, 2.350147, 2.353235, 2.356324, 2.359412, 2.3625, 2.365588, 2.368677, 2.371765, 2.374235, 2.376706, 2.379177, 2.381647, 2.384118, 2.387206, 2.390294, 2.393382, 2.396471, 2.399559, 2.402647, 2.405735, 2.408823, 2.412941, 2.417059, 2.421176, 2.423647, 2.426118, 2.428588, 2.431059, 2.433529, 2.437647, 2.441765, 2.445882, 2.448353, 2.450824, 2.453294, 2.455765, 2.458235, 2.462353, 2.466471, 2.470588, 2.473059, 2.475529, 2.478, 2.480471, 2.482941, 2.486029, 2.489118, 2.492206, 2.495294, 2.497765, 2.500235, 2.502706, 2.505177, 2.507647, 2.510735, 2.513824, 2.516912, 2.52, 2.523088, 2.526176, 2.529265, 2.532353, 2.535441, 2.538529, 2.541618, 2.544706, 2.547177, 2.549647, 2.552118, 2.554588, 2.557059, 2.55953, 2.562, 2.564471, 2.566941, 2.569412, 2.5725, 2.575588, 2.578676, 2.581765, 2.584853, 2.587941, 2.591029, 2.594118, 2.597206, 2.600294, 2.603382, 2.606471, 2.608941, 2.611412, 2.613883, 2.616353, 2.618824, 2.621912, 2.625, 2.628088, 2.631176, 2.633647, 2.636118, 2.638588, 2.641059, 2.643529, 2.647647, 2.651765, 2.655882, 2.658353, 2.660824, 2.663294, 2.665765, 2.668235, 2.671324, 2.674412, 2.6775, 2.680588, 2.683059, 2.685529, 2.688, 2.690471, 2.692941, 2.695412, 2.697882, 2.700353, 2.702824, 2.705294, 2.709412, 2.71353, 2.717647, 2.719706, 2.721765, 2.723824, 2.725883, 2.727941, 2.73, 2.733088, 2.736176, 2.739265, 2.742353, 2.744824, 2.747294, 2.749765, 2.752235, 2.754706, 2.758824, 2.762941, 2.767059, 2.769118, 2.771177, 2.773235, 2.775294, 2.777353, 2.779412, 2.781882, 2.784353, 2.786824, 2.789294, 2.791765, 2.794235, 2.796706, 2.799177, 2.801647, 2.804118, 2.807206, 2.810294, 2.813382, 2.816471, 2.81853, 2.820588, 2.822647, 2.824706, 2.826765, 2.828824, 2.831294, 2.833765, 2.836236, 2.838706, 2.841177, 2.842941, 2.844706, 2.846471, 2.848235, 2.85, 2.851765, 2.853529, 2.856, 2.858471, 2.860941, 2.863412, 2.865882, 2.867941, 2.87, 2.872059, 2.874118, 2.876177, 2.878235, 2.880706, 2.883177, 2.885647, 2.888118, 2.890588, 2.892647, 2.894706, 2.896765, 2.898824, 2.900883, 2.902941, 2.905, 2.907059, 2.909118, 2.911177, 2.913235, 2.915294, 2.917765, 2.920235, 2.922706, 2.925177, 2.927647, 2.929706, 2.931765, 2.933824, 2.935883, 2.937941, 2.94, 2.941765, 2.94353, 2.945294, 2.947059, 2.948823, 2.950588, 2.952353, 2.954118, 2.955883, 2.957647, 2.959412, 2.961176, 2.962941, 2.964706, 2.966765, 2.968824, 2.970882, 2.972941, 2.975, 2.977059, 2.979118, 2.981177, 2.983235, 2.985294, 2.987353, 2.989412, 2.991177, 2.992941, 2.994706, 2.996471, 2.998235, 3.0, 3.001765, 3.003824, 3.005883, 3.007941, 3.01, 3.012059, 3.014118, 3.015882, 3.017647, 3.019412, 3.021177, 3.022941, 3.024706, 3.026471, 3.02853, 3.030588, 3.032647, 3.034706, 3.036765, 3.038824, 3.040588, 3.042353, 3.044118, 3.045882, 3.047647, 3.049412, 3.051177, 3.052941, 3.054706, 3.056471, 3.058235, 3.06, 3.061765, 3.063529, 3.065294, 3.067059, 3.068824, 3.070588, 3.072353, 3.074118, 3.075882, 3.077647, 3.079412, 3.081177, 3.082941, 3.084706, 3.086471, 3.088235, 3.092353, 3.096471, 3.125294]

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

        # Convert to resistance
        raw_average = self.ADC_MAX * self.adc_V_lookup[round(raw_average)]/self.ADC_Vmax
        resistance = (self.SER_RES * raw_average) / (self.ADC_MAX - raw_average)

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

# PID Constants
KP = const(1)
KI = const(0.0001)
KD = const(0) #set to 0 because of the high level of noise from the temperature sensor
Imax = 1
target_temperature = 17

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

        self.relays_on = []

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
        else:
            self.integral = 0

        I = KI * self.integral
        I = max(min(I, 1.0), -1.0) # Hard clamp to ±1
        I = KI * self.integral

        # Derivative term
        D = KD * (error - self.last_error) / dt

        # Calculate output (0-1 range)
        PID = P + I + D
        output = max(-0.5, min(1, PID))  # Clamp to 0-1 range

        # Update state
        self.last_error = error
        self.last_time = now

        print(f"output = {output}, PID = {PID}, I  is {I}")

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
        self.relays_on = []
        if self.current_power_mode == 'low':
            self.relays_on = [self.relay1]
        elif self.current_power_mode == 'high' or 'eco':
            self.relays_on = [self.relay1, self.relay2]

        # Duty cycle implementation
        cycle_time = 10  # seconds
        on_time = cycle_time * self.duty_cycle

        for relay in self.relays_on:
            relay.on()

    def set_cooling(self, required_power, PID):

        now = time.ticks_ms()

        # Original power mode selection logic
        new_power_mode = self.current_power_mode
        if time.ticks_diff(now, self.last_switch_time) > self.min_mode_time:
            if required_power > LOW_POWER_MAX and self.current_power_mode != 'high':
                print("high statement correct")
                if PID > 3.8:
                    new_power_mode = 'high'
            if required_power > LOW_POWER_MAX and self.current_power_mode != 'eco':
                print("eco statement correct")

                if PID <= 3.8:
                    new_power_mode = 'eco'
            if required_power < HIGH_POWER_MIN and self.current_power_mode != 'low':
                print("low statement correct")

                new_power_mode = 'low'
            if required_power <= -0.3:
                print("off statement correct")

                new_power_mode = 'off'

        # Apply new power mode if changed
        if new_power_mode != self.current_power_mode:
            self.current_power_mode = new_power_mode
            self.last_switch_time = now

        # Calculate and apply duty cycle (from previous implementation)
        if self.current_power_mode == 'off':
            effective_power = 0
            self.duty_cycle = 0
            set_pump_rate(PUMP_A, 0)
            print("pump off")
        if self.current_power_mode == 'low':
            effective_power = min(required_power / LOW_POWER_MAX, 1.0)
            self.duty_cycle = max(MIN_DUTY_CYCLE, effective_power)
            set_pump_rate(PUMP_A, 58982)
            print(f"pump on | low power | duty cycle {self.duty_cycle}")
        if self.current_power_mode == 'eco':
            effective_power = (required_power - LOW_POWER_MAX) / (1 - LOW_POWER_MAX)
            self.duty_cycle = max(MIN_DUTY_CYCLE, effective_power)
            set_pump_rate(PUMP_A, 64225)
            print(f"pump on | eco power | duty cycle {self.duty_cycle}")
        if self.current_power_mode == "high":  # high power mode
            effective_power = (required_power - LOW_POWER_MAX) / (1 - LOW_POWER_MAX)
            self.duty_cycle = max(MIN_DUTY_CYCLE, effective_power)
            set_pump_rate(PUMP_A, 65535)
            print(f"pump on | high power | duty cycle {self.duty_cycle}")

        self.apply_duty_cycle()

        return self.current_power_mode

    def run_cooling_system(self, target_temperature):

        current_temperature = self.read_temp()
        print(f"current temperature is {current_temperature}")
        if  current_temperature - target_temperature <= -0.5:
            set_pump_rate(PUMP_A, 0)
            print("pump off")

        output, pid, P_values, I_values = self.pid_control(current_temperature, target_temperature)
        power_mode_values = self.set_cooling(round(output, 1), pid)

        return pid, P_values, I_values, power_mode_values, current_temperature



duration = 72000 #s
start_time = time.time()
elapsed = time.time() - start_time
time_points = []
pid_values = []
P_values = []
I_values = []
D_values = []
power_mode_values = []
temperature_points = []
cooler = CoolingSystem(TemperatureSystem)

with open("data.txt", "a") as file:
    file.write("\t".join(["Time", "PID", "P", "I", "Mode", "Temp", "Target"])+"\n")

if __name__ == "__main__":
    print(f"Running for {duration} s")
    while elapsed < duration:
        try:
            print(f"Elapsed time: {elapsed:.1f}s")

            time.sleep(1)

            elapsed = time.time() - start_time
            pid_out, P_out, I_out, power_mode_out, temperature_out = cooler.run_cooling_system(target_temperature)

            with open("data.txt", "a") as file:
                file.write("\t".join(str(x) for x in [pid_out, P_out, I_out, power_mode_out, temperature_out, target_temperature])+"\n")
        except Exception as e:
            print(e)
            with open("error_log.txt", "a") as file:
                file.write("Timepoint: "+str(time.localtime())+"\nElapsed: "+str(elapsed)+"\n"+str(e))
            break