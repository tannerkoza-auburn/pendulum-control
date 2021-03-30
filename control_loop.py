import time


# Define Function for Specified Loop Rates
def loop_rate(freq):
    t = time.time()
    delta_t = 1 / freq
    while True:
        if time.time() > (t + delta_t):
            break


# Define Control Law Method
def control_law():
    print(time.time())


while True:
    print(time.time())
    loop_rate(500)
