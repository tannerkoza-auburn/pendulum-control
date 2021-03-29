import time


# Define Function for Specified Loop Rates
def loop_rate(freq):
    t = time.time()
    delta_t = 1 / freq
    control_law()  # Control Law Method
    while True:
        if time.time() > (t + delta_t):
            break


# Define Control Law Method
def control_law():
    print(time.time())


while True:
    loop_rate(20)
