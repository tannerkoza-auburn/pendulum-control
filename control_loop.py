import time
import Adafruit_ADS1x15
import RPi.GPIO as GPIO
import pandas as pd
import plotly.express as px


# Define the ADC Instance
adc = Adafruit_ADS1x15.ADS1115()
GAIN = 2/3
adc.start_adc(0, gain=GAIN)  # change to corresponding input channel

# Define Sampling Frequency
fs = 100
T = 1/fs  # this defines the time between samples

# Define Control Output
controlPIN = 0

# Define ADC Voltage Range & Max Value
range_adc = 4.096
max_adc = 32767

# Define Control Gains
Kp = 0.2002  # proportional gain
Kd = 0.0138  # derivative gain
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

# Define Logging Lists
pos_values = list()
sig_values = list()


# Define Method for Set Point Definition
def set_point(rng, mx):
    input('Place Pendulum to Desired Set Point and Press ENTER.')
    dig_pos = adc.get_last_result()
    sp = rng/(mx/dig_pos)  # initial analog voltage
    return sp


# Define Function for Specified Loop Rates
def loop_rate(period, t):
    delta_t = period
    while True:
        if time.time() > (t + delta_t):
            break


des = set_point(range_adc, max_adc)
input('\nStart Control?')

try:
    while True:
        now = time.time()
        dig_pos = adc.get_last_result()   # retrieves last position voltage value from potentiometer
        if dig_pos != 0:
            pos = range_adc/(max_adc/dig_pos)  # determines analog voltage from digital voltage
        else:
            pos = dig_pos
            
        pos_values.append(pos)
        error = des - pos   # defines error
        control_sig = l_l_control + A*error + B*l_error + C*l_l_error
        sig_values.append(control_sig)
        # Insert Output Code Here
        l_l_error = l_error
        l_error = error
        l_l_control = l_control
        l_control = control_sig
        loop_rate(T, now)
except KeyboardInterrupt:
    data = pd.DataFrame(pos_values, columns=['Position Voltage'])
    data['Commanded Voltage'] = sig_values
    data['Time'] = data.index/fs
    data.to_csv('data.csv')
    fig = px.line(data, x='Time', y=['Position Voltage', 'Commanded Voltage'])
    fig.show()

