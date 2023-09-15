import time

from board import SCL, SDA
import busio

from adafruit_motor import servo
from adafruit_pca9685 import PCA9685

import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

import matplotlib.pyplot as plt



i2c = busio.I2C(SCL, SDA)

ads = ADS.ADS1115(i2c)

chan = AnalogIn(ads, ADS.P0)

pca = PCA9685(i2c)

pca.frequency = 50


servo = servo.Servo(pca.channels[0], min_pulse=500, max_pulse=2600, actuation_range=270)

servo.angle = 0

time.sleep(5)

print("Initial Channel value:", chan.value)

def map_range(x, in_min, in_max, out_min, out_max):
    """re-maps a number from one range to another."""
    mapped = (x-in_min) * (out_max - out_min) / (in_max-in_min) + out_min
    if out_min <= out_max:
        return max(min(mapped, out_max), out_min)
    return min(max(mapped, out_max), out_min)

prev_read = map_range(chan.value, 2048, 24150, 0, 270)
readings = []


try:
    while True:
        desiredPos=float(input("Degrees (0-270): "))
        servo.angle = desiredPos

        curr_read = map_range(chan.value, 2048, 24150, 0, 270)

        while abs(desiredPos - curr_read) > 0.5:
            readings.append(chan.voltage)
            curr_read = map_range(chan.value, 2048, 24150, 0, 270)
            print(curr_read)

        
        Position = curr_read
        print("     Channel value:", chan.value)
        print("     Current Position:", Position)
except:
    plt.plot(readings)
    plt.ylabel('response')
    plt.show()

