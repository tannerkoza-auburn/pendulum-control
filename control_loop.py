import time
import RPi.GPIO as GPIO
import Adafruit_ADS1x15

# Define the ADC Instance
adc = Adafruit_ADS1x15.ADS1115()
GAIN = 2/3
adc.start_adc(0, gain=GAIN)  # change to corresponding input channel

# Define Sampling Frequency
fs = 60
T = 1/fs  # this defines the time between samples

# Define Control Input/Output
signalPin = 0
controlPin = 0
GPIO.Input()

# Define Control Gains
Kp = 0  # proportional gain
Kd = 0  # derivative gain
Ki = 0  # integral gain

# Define z-Transform Coefficients
A = Kp + ((2*Kd)/T) + ((T*Ki)/2)
B = (T*Ki) - ((4*Kd)/T)
C = ((2*Kd)/T) + ((T*Ki)/2) - Kp

# Define Control Loop Values
l_error = 0
l_l_error = 0
l_control = 0
l_l_control = 0


# Define Method for Set Point Definition
def set_point():
    input('Place Pendulum to Desired Set Point and Press ENTER.')
    sp = adc.get_last_result()
    return sp


# Define Function for Specified Loop Rates
def loop_rate(period):
    t = time.time()
    delta_t = period
    while True:
        if time.time() > (t + delta_t):
            break


des = set_point()
while True:
    pos = adc.get_last_result()   # retrieves last position voltage value from potentiometer
    error = des - pos
    control_sig = l_l_control + A*error + B*l_error + C*l_l_error
    # Insert Output Code Here
    l_l_error = l_error
    l_error = error
    l_l_control = l_control
    l_control = control_sig
    loop_rate(T)


