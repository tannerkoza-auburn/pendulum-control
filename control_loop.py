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
fs = 21
T = 1/fs  # defines sample period

# Define Control Output
controlPIN = 0

# Define ADC Voltage Range & Max Value
range_adc = 6.144
max_adc = 32767

# Define Control Gains
Kp = 0.2002  # proportional gain
Kd = 0.0138  # derivative gain
Ki = 0  # integral gain

# Define Controller z-Transform Coefficients
A = Kp + ((2*Kd)/T) + ((T*Ki)/2)
B = (T*Ki) - ((4*Kd)/T)
C = ((2*Kd)/T) + ((T*Ki)/2) - Kp

# Define Filter 2nd Order z-Transform Coefficients
a = 0.009259
b = 0.2222
c = 0.8219
d = 0.1653

# Define Control Loop Values
l_error = 0
l_l_error = 0
l_control = 0
l_l_control = 0

# Define Filter Loop Value
lf_out = 0

# Define Logging Lists
pos_values = list()
sig_values = list()
filt_sig_values = list()


# Define Function for Set Point Definition
def set_point(rng, mx):
    input('Place Pendulum to Desired Set Point and Press ENTER.')
    dig = adc.get_last_result()
    sp = rng/(mx/dig)  # initial analog voltage
    return sp


# Define Function for Specified Loop Rates
def loop_rate(period, t):
    delta_t = period
    while True:
        if time.time() > (t + delta_t):
            break


# Define Set Point
des = set_point(range_adc, max_adc)
input('\nStart Control?')

# Control Loop
try:
    while True:
        now = time.time()   # sets time at start of the loop to maintain loop rate
        dig_pos = adc.get_last_result()   # retrieves last position voltage value from potentiometer
        
        if dig_pos != 0:
            pos = range_adc/(max_adc/dig_pos)  # determines analog voltage from filtered digital voltage value
        else:
            pos = dig_pos   # sets analog voltage to 0 if digital voltage is 0
            
        pos_values.append(pos)  # logs position

        error = des - pos   # defines error
        control_sig = l_l_control + A*error + B*l_error + C*l_l_error   # determines control signal
        sig_values.append(control_sig)
        
        filt_sig = a*control_sig + a*l_control - b*lf_out  # filters control signal (bw is approx. 10 hz)
        filt_sig_values.append(filt_sig)
        
        # Insert Output Code Here (filt_sig)

        # Variable Management
        l_l_error = l_error
        l_error = error
        l_l_control = l_control
        l_control = control_sig
        
        lf_out = filt_pos

        # Loop Rate Function
        loop_rate(T, now)

except KeyboardInterrupt:
    sp_values = [des]*len(pos_values)
    data = pd.DataFrame(pos_values, columns=['Position Voltage'])
    data['Set Point'] = sp_values
    data['Commanded Voltage'] = sig_values
    data['Filtered Commanded Voltage'] = filt_sig_values
    data['Time'] = data.index/fs
    data.to_csv('data.csv')
    fig = px.line(data, x='Time', y=['Position Voltage', 'Set Point', 'Commanded Voltage', 'Filtered Commanded Voltage'])
    fig.show()
