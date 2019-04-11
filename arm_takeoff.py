#---- Declare the dependencies
from dronekit import VehicleMode, connect, LocationGlobal
from pymavlink import mavutil

import math, sys, socket, time
import argparse

baudRate = 57600

#---- Set the quadrotor arm and takeoff
def arm_and_takeoff(setTargetHeight):

	while not vehicle.is_armable:
		print "Initialize the quadrotor"
		time.sleep(1)
	
	#--- Set the quadrotor mode in GUIDED mode
	vehicle.mode  = VehicleMode("GUIDED")
	vehicle.armed = True

	while not vehicle.armed:
		print "Waiting for quadrotor arming"
		time.sleep(1)


	#--- TAKEOFF!!
	print "Take off!"
	vehicle.simple_takeoff(setTargetHeight)

	
	#--- Wait the takeoff finish
	#--  Check the altitude is reached
	while True:
		print "Altitude: ", vehicle.location.global_relative_frame.alt
		
		#-- Trigger to reach the altitude
		if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: 
            		print "Reached target altitude"
            		break
        	time.sleep(1)

#---- Get the distance information
def get_distance(xLocation, yLocation):
	dis_latitude  = yLocation.lat - xLocation.lat
	dis_longitude = yLocation.lon - xLocation.lon
	return math.sqrt((dis_latitude*dis_latitude) + (dis_longitude*dis_longitude)) * 1.113195e5

#---- Define connection method
#---  Also define the ip address, let say we use localhost 127.0.0.1:14550
 connection_string = "127.0.0.1:14550"
# connection_string = "/dev/ttyAMA0"

#--- Connect the quadrotor
vehicle = connect(connection_string, baud = baudRate, wait_ready = True)

#--- Arm and takeoff at desire altitude (meter)
arm_and_takeoff(10)

#--- Set the hovering state 20 seconds
time.sleep(20)
print "Now quadrotor is hovering"

#--- Set the quadrotor landing
vehicle.mode = VehicleMode("LAND")
print "Landing"
#--- close
vehicle.close()
