# Quadcopter Raspberry-Pi Server

> The code and set-up instructions for a Raspberry-Pi-hosted web-server displaying drone telemetry-data and control options.


## Description

This code runs on a Raspberry Pi to host a web-page on a flask server. The Raspberry PI connects to a flight controller running the Ardupilot autopilot via a serial interface, and communicates using the MavLink protocol. The server can be reached from any device connected to the same wireless network, and in this example displays drone telemetry (battery voltage, GPS status, altitude, location, flight mode, arm status) along with the status of electronics connected directly to the Pi (a servo payload release mechanism, onboard lights and a power-monitor for a wireless power transfer system). 

These options can be customised through changes to the server-code (RPwebUI.py) for fetching data using Mavlink messages (https://dronekit-python.readthedocs.io/en/latest/guide/mavlink_messages.html) and the html and css files for the interface dislay.

![A screenshot of the server UI](relative%20path/images/RPinterface.jpg)

## Set-Up

This section briefly describes the configuration of Ardupilot on the flight-controller, and explains the necessary Raspberry-Pi settings and dependencies.


### Ardupilot Configuration

A Pixhawk-Mini flight controller (FC) was used in the original build, but can be replaced for any capable of running Ardupilot. Set Up is well documented on the Ardupilot site:

1.	Install Mission Planner here: https://ardupilot.org/planner/docs/mission-planner-installation.html
2.	Install the latest stable Arducopter release onto the Pixhawk: https://ardupilot.org/planner/docs/common-loading-firmware-onto-pixhawk.html 
3.	Calibrate the accelerometer: https://ardupilot.org/planner/docs/common-accelerometer-calibration.html#common-accelerometer-calibration 
4.	And the radio-controller: https://ardupilot.org/planner/docs/common-rc-transmitter-flight-mode-configuration.html

** In this build the use of a magnetic payload mechanism caused the onboard magnetometer (in the flight-controller) to drift, so this was disabled through mission-planner , and only the externel GPS magnetomoeter is used) **

5.	Calibrate the (internal or external or both) GPS compass, making sure it is fixed in its frame position: https://ardupilot.org/planner/docs/common-compass-calibration-in-mission-planner.html#common-compass-calibration-in-mission-planner 
6.	Configure the battery sensing: https://ardupilot.org/copter/docs/common-power-module-configuration-in-mission-planner.html. 

At this point, you can already manually fly the drone It’s important to note though, that without adding a telemetry radio, or a companion computer there is no way of receiving key data like battery voltage without directly plugging the FC into a computer.

The default settings spin the motors slowly on arming as a visual confirmation, but for early testing/debugging it can be safer to disable this by changing the MOT_SPIN_ARM parameter to 0.


### Raspberry-Pi Configuration

1.	Install the headless Raspbian OS onto an empty SD Card. When doing this, you can directly enable SSH communication, add WiFi details and set a password through the RP imager. To be able to use the RP in multiple locations, it is best to use a mobile-hotspot.

2.	SSH into the RP using the hostname pi@raspberrypi.local from a device connected to the same WiFi network (use puTTY or from the command line simply run: ssh p@raspberrypi.local)

3.	Install the dependencies by running the following lines on the Pi:

**Core:**
sudo apt-get update
sudo apt-get upgrade
sudo apt-get install python3-pip
sudo apt-get install python3-dev

**Dronekit for MavLink communication:**
sudo apt-get install screen python3-wxgtk4.0 python3-lxml
sudo pip install future pyserial dronekit MAVProxy

**LED strip control:**
sudo pip install rpi_ws281x

- Additionally disable audio (which cannot be used at the same timeas these libraries) by editing the snd-blacklist and configuration files. Run sudo nano /etc/modprobe.d/snd-blacklist.conf and add the line blacklist snd_bcm2835. Then run sudo nano /boot/config.txt and comment out the line dtparam=audio=on.

**GPIO pin control (servos):**
sudo apt-get install python3-pigpio python3-rpi.gpio
sudo apt-get remove modemmanager

**INA219 Power Monitor:**
sudo pip install adafruit-circuitpython-ina219 

- Also enable the RP I2C interface through the interfacing options in the configuration utility, and reboot the Pi for this to take effect.

4. Install the files in this repo onto the Raspberry-Pi

The server can now be launched by running RPwebUI.py from the Pi command line. As it launches, the IP-address it can be accessed from appears in the command window. 


## Warnings!
This is a demonstration system, not a secure method of communicating with an aircraft! For safety, no features currently enable control of the aircraft through the RP (e.g. arming the aircraft or changing the flight-mode), but these can easily be added. Also note that if the Pi or ground device lose connection to the wireless network, the server wil stop updating and has to be relaunched on reconnection. 


## Change History

* 01/06
    * Initial Commit
* 02/06
    * README added