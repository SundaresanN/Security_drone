#Libraries
import time
import serial
import socket
from pynmea import nmea
import RPi.GPIO as GPIO

#Variables
gps_dev = "/dev/ttyUSB0" #For linux TTL GPS module
gps_baud = 9600
gpgga = nmea.GPGGA()
trig = 12 #HC-05
echo = 11 #HC-05

#Configurations
GPIO.setmode(GPIO.BOARD)
GPIO.setup(trig, GPIO.OUT)
GPIO.setup(echo, GPIO.IN)
ser = serial.Serial(gps_dev , gps_baud , timeout = 0.5)
#ser.open()

#Functions
def distance():
    # set Trigger to HIGH
    GPIO.output(trig, True)
 
    # set Trigger after 0.01ms to LOW
    time.sleep(0.00001)
    GPIO.output(trig, False)
 
    startTime = time.time()
    stopTime = time.time()
 
    # save StartTime
    while GPIO.input(echo) == 0:
        startTime = time.time()
 
    # save time of arrival
    while GPIO.input(echo) == 1:
        stopTime = time.time()
 
    # time difference between start and arrival
    timeElapsed = stopTime - startTime
    # multiply with the sonic speed (34300 cm/s)
    # and divide by 2, because there and back
    distance = (timeElapsed * 34300) / 2
 
    return distance 

def send_gps():
    data = ser.readline()
    if data[0:6] == '$GPGGA':
        d = gpgga.parse(data)
        lat = gpgga.latitude
        lon = gpgga.longitude
        dlat = int(lat[0:2])
        dlon = int(lon[0:3])
        mlat = float(lat[2:])
        mlon = float(lon[3:])
        lat = dlat + mlat / 60
        lon = dlon + mlon / 60
        #print ("Latitude of intruder is ", "%.5f" % lat)
        #print ("Longitude of intruder is ", "%.5f" % lon)
        time.sleep(0.5)
        return lat,lon

#Main Program
try:
    while True:
	dist = distance()
	time.sleep(.5)
	dist = distance()
	if 0 <= dist >= 1000 :
        print ("Measured Distance = %.1f cm" % dist)
		print ("No Intrusion")
	else:
		print("Alert! Intruder detected!")
		print("Sending Location to drone!")
		while 1:
            send_gps()
            break

		break

except KeyboardInterrupt:
    print("Measurement stopped by User")
    GPIO.cleanup()

