#!/usr/bin/env python

import RPi.GPIO as gpio

# Constants
FIRE_ENABLE = 3
AZIMUTH_ENABLE = 19
ELEVATION_ENABLE = 11

def main():
    # Setup gpio
    gpio.setmode(gpio.BOARD)
    gpio.setwarnings(False)
    
    gpio.setup(FIRE_ENABLE, gpio.OUT)
    gpio.setup(AZIMUTH_ENABLE, gpio.OUT)
    gpio.setup(ELEVATION_ENABLE, gpio.OUT)
    
    # Disable all steppers
    gpio.output(FIRE_ENABLE, gpio.LOW)
    gpio.output(AZIMUTH_ENABLE, gpio.LOW)
    gpio.output(ELEVATION_ENABLE, gpio.LOW)

if __name__ == '__main__':
    main()
