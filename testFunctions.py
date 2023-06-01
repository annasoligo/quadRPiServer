# functions to be called from RPi python terminal to test the servo, LED strip and power monitoring

import board
import busio
import adafruit_ina219
from rpi_ws281x import Color, PixelStrip
import time
import RPi.GPIO as GPIO
import pigpio
import os

fr = 800000
dma = 10
brightness = 255
invert = False
channel = 0

nLED = 144
nPin = 18

def setUpCharge(LEDcount, pin):
    # initiates LED strip
    i2c = busio.I2C(board.SCL, board.SDA)
    ina = adafruit_ina219.INA219(i2c)
    strip = PixelStrip(LEDcount, pin, fr, dma, invert, brightness, channel)
    strip.begin()
    return ina, strip

def setLEDs(strip,LEDcount,R,G,B):
    # lights all LEDs in RGB coour
    for i in range(LEDcount):
        strip.setPixelColor(i, Color(R, G, B))
    strip.show()

def inaData(ina):
    # fetches INA power sensor data
    voltage = ina.bus_voltage
    current = ina.current/1000
    power = ina.power
    power2 = voltage*current
    return [round(voltage,2), round(current,2), round(power,2), round(power2,2)]


def colorWipe(strip, r, g, b, wait_ms=50):
        # Wipes color across display a pixel at a time
        color = Color(r,g,b)
        for i in range(strip.numPixels()):
                strip.setPixelColor(i, color)
                strip.show()
                time.sleep(wait_ms / 1000.0)

def setUpServo(servoPin):
    # clears previous GPIO settings and closes pigpio daemon
    # restarts pigpio daemon and sets up servo
    try:
        GPIO.cleanup()
    except:
        print('Nothing to clean :)')
    
    os.system("sudo killall pigpiod")
    os.system("sudo pigpiod")

    pwm = pigpio.pi()
    pwm.set_mode(servoPin, pigpio.OUTPUT)

def setAngle(servoPin, pwm, pw):
        # writes pwm to the servo pin
        pwm.set_servo_pulsewidth(servoPin, pw)
        time.sleep(1)
        # turns off servo pin once angle has been set
        pwm.set_servo_pulsewidth(servoPin, 0)
        pwm.set_PWM_dutycycle(servoPin,0)
