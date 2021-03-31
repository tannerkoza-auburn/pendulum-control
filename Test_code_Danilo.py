import time
# Import the ADS1x15 module.
import Adafruit_ADS1x15


# Create an ADS1115 ADC (16-bit) instance.
adc = Adafruit_ADS1x15.ADS1115()

# Or create an ADS1015 ADC (12-bit) instance.
#adc = Adafruit_ADS1x15.ADS1015()

# Choose a gain of 1 for reading voltages from 0 to 4.09V.
# Or pick a different gain to change the range of voltages that are read:
#  - 2/3 = +/-6.144V
#  -   1 = +/-4.096V
#  -   2 = +/-2.048V
#  -   4 = +/-1.024V
#  -   8 = +/-0.512V
#  -  16 = +/-0.256V
# See table 3 in the ADS1015/ADS1115 datasheet for more info on gain.
GAIN = 1

# Start continuous ADC conversions on channel 0 using the previously set gain
# value.  Note you can also pass an optional data_rate parameter, see the simpletest.py
# example and read_adc function for more information.
adc.start_adc(0, gain=GAIN)

# Define Function for Specified Loop Rates
def loop_rate(freq):
    t = time.time()
    delta_t = 1 / freq
    control_law()  # Control Law Method
    while True:
        if time.time() > (t + delta_t):
            break

# Calibrating for a desired position
print('Set the Pendulum to the desired position')

des = adc.get_last_result()

# Define Control Law Method
Kp =
Kd =

A = Kp + (2*Kd/delta_t)
B = -4*Kd/delta_t
C = (2*Kd/delta_t)-Kp
Last_error = 0
Last_last_error = 0
Last_control = 0
Last_last_control = 0


while True:
    loop_rate(60)
    position = adc.get_last_result()
    error =