import os
from flask import Flask, render_template, Response
import datetime
from time import sleep
from dronekit import connect
import dronekit as dk
import RPi.GPIO as GPIO
import pigpio
import board
import busio
import adafruit_ina219
from rpi_ws281x import Color, PixelStrip


os.system("sudo killall pigpiod")
os.system("sudo pigpiod")

# clears existing GPIO settings
try:
        GPIO.cleanup()
except:
        print('Nothing to clean :)')

# sets up LED strip and INA219 sensor
fr = 800000
dma = 10
brightness = 255
invert = False
channel = 0

def setUpCharge(LEDcount, pin):
    i2c = busio.I2C(board.SCL, board.SDA)
    ina = adafruit_ina219.INA219(i2c)
    strip = PixelStrip(LEDcount, pin, fr, dma, invert, brightness, channel)
    strip.begin()
    return ina, strip

def setLEDs(strip,LEDcount,R,G,B):
    for i in range(LEDcount):
        strip.setPixelColor(i, Color(R, G, B))
    strip.show()

def inaData(ina):
    voltage = ina.bus_voltage
    current = ina.current/1000
    power = ina.power
    power2 = voltage*current
    return [round(voltage,2), round(current,2), round(power,2), round(power2,2)]

LEDpin = 18
nLEDs = 144

global inaConnEcted
inaConnected = True
global LEDson
LEDson = False

try:
    ina, strip = setUpCharge(nLEDs, LEDpin)
except:
       print('No I2C sensor detected')
       inaConnected = False

global dkConnect
dkConnect = False

# checks if dronekit is already connected, and if not, connects
try:
        print(vehicle)
except NameError:
        print('Attempting to connect...')
        try:
                vehicle = connect('/dev/serial0', wait_ready=True, baud=115200)
                print('Successfully connected :)')
                dkConnect = True
        except:
                print('Connection Unsuccesful')

# initiates servo pin and pwm settings
servoPin = 17
pwm = pigpio.pi()
pwm.set_mode(servoPin, pigpio.OUTPUT)

def setAngle(pw):
        # writes pwm to the servo pin
        pwm.set_servo_pulsewidth(servoPin, pw)
        sleep(1)
        # turns off servo pin once angle has been set
        pwm.set_servo_pulsewidth(servoPin, 0)
        pwm.set_PWM_dutycycle(servoPin,0)


# initiates global variable to track data updates
global count
count = 0

def getData():
        # using dronekit to get drone data, retrieves ina sensor data, and incr>
        global count
        global LEDson
        global inaConnected
        if LEDson:
                setLEDs(strip,nLEDs,255,255,255)
        if inaConnected:
                WPTdata = inaData(ina)
        else:
                WPTdata = [0,0,0]
        count+=1
        now=datetime.datetime.now()
        timeString=now.strftime("%d-%m-%Y %H:%M")
        templateData= {
                'time': timeString,
                'location': [vehicle.location.global_relative_frame.lat,
                        vehicle.location.global_relative_frame.lon,
                        vehicle.location.global_frame.alt],
                'velocity': vehicle.velocity,
                'battery': round(vehicle.battery.voltage,2),
                'mode': vehicle.mode.name,
                'armed': vehicle.armed,
                'charge': WPTdata,
                'refreshCount': count,
        }
        return templateData

def getDCData():
        # default disconnect data, increases update count
        global count
        count+=1
        now=datetime.datetime.now()
        timeString=now.strftime("%d-%m-%Y %H:%M")
        templateData= {
                'time': timeString,
                'location': [0,0,0],
                'velocity': [0,0,0],
                'battery': [0],
                'mode': 'Mode Unknown',
                'armed': 'Aircraft Disconnected',
                'charge':[0.00,0.00,0.00],
                'refreshCount': count,
        }
        return templateData

#print(getData()) # debugging

#______________________FLASK SERVER______________________

app = Flask(__name__)

# on page loading
@app.route('/')

# loads page HTML
def index():
        return render_template('webInterface.html')

# on page button press
@app.route('/<actionid>')

def handleRequest(actionid):
        print("Button pressed:{}".format(actionid)) # debugging
        global LEDson
        # sets servo angle to lift magnets
        if actionid == "toggle-up":
                print('toggled up')
                setAngle(1600)
        # dets servo angle to lower magnets
        elif actionid =="toggle-down":
                print('toggled down')
                setAngle(1000)
        elif actionid =="LED-toggle-down":
                print('LEDs On')
                LEDson = True
                setLEDs(strip, nLEDs, 255,255,255)
        elif actionid == "LED-toggle-up":
                print('LEDs Off')
                LEDson = False
                setLEDs(strip, nLEDs, 0,0,0)
        return "OK 200"

# on update request
@app.route('/update')

def update():
        # fetches current drone data
        if dkConnect == True:
                templateData = getData()
        else:
                templateData = getDCData()
        print(templateData)
        return templateData

# on running file
if __name__=='__main__':
        # clears cache
        os.system("sudo rm -r ~/.cache/chromium/Default/Cache/*")
        # runs server, debug=false to prevent server reload and drone reconnect>
        app.run(debug=False, port = 5000, host= '0.0.0.0', threaded=True)

