# Import Statements
import time
import Adafruit_ADS1x15
import RPi.GPIO as GPIO
import pandas as pd
import plotly.express as px


# Define the ADC Instance
adc = Adafruit_ADS1x15.ADS1115()
adc.data_rate = 860

GAIN = 2/3
adc.start_adc(0, gain=GAIN)  # change to corresponding input channel

# Define Sampling Frequency
fs = 70
T = 1/fs  # defines sample period

# Define PWM Frequency
pwm_f = 8000

# Define PWM Control Output
GPIO.setmode(GPIO.BCM)
positivePIN = 17
negativePIN = 23
GPIO.setup(positivePIN, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(negativePIN, GPIO.OUT, initial=GPIO.LOW)

p_pwm = GPIO.PWM(positivePIN, pwm_f)  # positive voltage output pin
n_pwm = GPIO.PWM(negativePIN, pwm_f)  # negative voltage output pin


# Define ADC Voltage Range & Max Value
range_adc = 6.144  # specified in ADS1115 datasheet
max_adc = 32768  # maximum number represented by 16 bits for this range

# Define Control Gains
Kp = 1.3 # proportional gain (empirically tuned)
Kd = 0.135  # derivative gain (empirically tuned)
Ki = 0  # integral gain

# Define Controller z-Transform Coefficients
A = Kp + ((2*Kd)/T) + ((T*Ki)/2)
B = (T*Ki) - ((4*Kd)/T)
C = ((2*Kd)/T) + ((T*Ki)/2) - Kp

# Define Filter 1st Order z-Transform Coefficients
a = 0.4521 
b = 0.09589 

# Define Control Loop Values
l_error = 0
l_l_error = 0
l_control = 0
l_l_control = 0

# Define Logging Lists
pos_values = list()
filt_pos_values = list()
sig_values = list()
filt_sig_values = list()
duty = list()


# Define Functions for Set Point Definition
def set_point(rng, mx):
    input('Place Pendulum to Desired Set Point and Press ENTER.')
    dig = adc.get_last_result()
    sp = rng/(mx/dig)  # initial analog voltage
    return sp, dig

# Define Function for Specified Loop Rates
def loop_rate(period, t):
    delta_t = period
    while True:
        if time.time() > (t + delta_t):
            break


# Define Set Point
[des, dig_des] = set_point(range_adc, max_adc)
input('\nStart Control?')

# Define Filter Loop Value
lf_pos = dig_des
lf_sig = 0
lf_out_pos = dig_des
lf_out_sig = 0

# Start PWM Pins
p_pwm.start(0)
n_pwm.start(0)

# Control Loop
try:
    while True:
        now = time.time()   # sets time at start of the loop to maintain loop rate
        dig_pos = adc.get_last_result()   # retrieves last position voltage value from potentiometer
        
        if dig_pos != 0:
            pos = range_adc/(max_adc/dig_pos)  # determines analog voltage from digital voltage value
        else:
            pos = dig_pos   # sets analog voltage to 0 if digital voltage is 0
               
        pos_values.append(pos)  # logs position

        error = des - pos   # defines error
        control_sig = l_l_control + A*error + B*l_error + C*l_l_error   # determines control signal
        sig_values.append(control_sig)  # logs control signal
        
        filt_sig = a*control_sig + a*lf_sig - b*lf_out_sig  # filters control signal (bw is approx. 10 hz   
        filt_sig = (filt_sig + lf_out_sig)/2  # moving average after filter
        filt_sig_values.append(filt_sig)  # logs filtered control signal
        
        duty_cycle = round(abs((filt_sig)/3.3)*100, 2)  # determining pwm duty cycle for commanded voltage
       
        duty_cycle = min(max(duty_cycle, 0), 100)  # digital saturation
        
        # PWM Pin State Handling
        if filt_sig >= 0:
            p_pwm.ChangeDutyCycle(duty_cycle)  # if positive set positive pin pwm signal
            n_pwm.ChangeDutyCycle(0)
        else:
            p_pwm.ChangeDutyCycle(0)  # if negative set negative pin pwm signal
            n_pwm.ChangeDutyCycle(duty_cycle)
        
        # Variable Management
        l_l_error = l_error
        l_error = error
        l_l_control = l_control
        l_control = control_sig
        
        lf_pos = dig_pos
        lf_sig = control_sig
        lf_out_pos = filt_pos
        lf_out_sig = filt_sig

        # Loop Rate Function
        loop_rate(T, now)

except KeyboardInterrupt:
    # Cleanup
    GPIO.cleanup()
    
    # Log Data Manipulation
    sp_values = [des]*len(pos_values)
    data = pd.DataFrame(pos_values, columns=['Position Voltage'])
    data['Set Point'] = sp_values
    data['Commanded Voltage'] = sig_values
    data['Filtered Command Voltage'] = filt_sig_values
    data['Time'] = data.index/fs
    
    # CSV Creation
    data.to_csv('data_' + str(fs) + 'hz.csv')
    
    # Graphing
    fig = px.line(data, x='Time', y=['Position Voltage', 'Set Point', 'Commanded Voltage',
        'Filtered Command Voltage'], title='Pendulum Performance: ' + str(fs) + 'Hz')
    fig.update_layout(xaxis_title='Time (s)', yaxis_title='Voltage (V)')
    fig.show()
    