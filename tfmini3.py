from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import serial

import argparse
parser = argparse.ArgumentParser()
parser.add_argument('--connect', default='127.0.0.1:14550')
args = parser.parse_args()

baud_rate = 57600

ser = serial.Serial("/dev/ttyUSB0", 115200)

#--- Connect the vehicle
print 'Connecting the vehicle on: %s' % args.connect
vehicle = connect(args.connect, baud=baud_rate, wait_ready = True)

def getTFminiData():
    #while True:
        count = ser.in_waiting
        if count > 8:
            recv = ser.read(9)
            ser.reset_input_buffer()
            if recv[0] == 'Y' and recv[1] == 'Y': # 0x59 is 'Y'
                low = int(recv[2].encode('hex'), 16)
                high = int(recv[3].encode('hex'), 16)
                distance = low + high * 256
                return distance

def activateSensor():
    try:
        if ser.is_open == False:
            ser.open()
        while True:
            data = getTFminiData()
            if data > 50 and data != None:
                print data
            if data < 50 and data != None:
                print "threshold reached"

    except KeyboardInterrupt:
        if ser != None:
            ser.close()
            print "Landing"
            vehicle.mode = VehicleMode("LAND")
            vehicle.close()

#--- Create a function for arming and takeoff after that
def arm_and_takeoff(setTargetAltitude):

	print "Basic pre-arm checks"
	# Do not let the user arm until autpilot is ready
	while not vehicle.is_armable:
		print "Waiting for vehicle to initialise ..."
        print activateSensor()
		time.sleep(1)

	print "Arming motors"
	# Set the quadrotor into GUIDED Mode
	vehicle.mode = VehicleMode("GUIDED")
	vehicle.armed = True

	while not vehicle.armed:
		print " Waiting for arming ..."
		time.sleep(1)

	print "Taking off!!"
	vehicle.simple_takeoff(setTargetAltitude) #taking off with desired altitude


	# Check the altitude of the quadrotor already reached
	while True:
		print "Altitude: ", vehicle.location.global_relative_frame.alt
		# Break and return from function below
		if vehicle.location.global_relative_frame.alt >= setTargetAltitude*0.95:
			print "Reached target altitude"
			break
		time.sleep(1)

# initialize takeoff altitude 5 meters
arm_and_takeoff(2)

print("Take off complete")
print activateSensor()
# hovering for 10 seconds
time.sleep(10)

# landing
print("Landing....")
vehicle.mode = VehicleMode("LAND")

# Close vehicle object
vehicle.close()
