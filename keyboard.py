"""
Controlling drone using arrow keys
"""

#---- Call the dependencies
import time

from dronekit import connect, LocationGlobalRelative, VehicleMode, LocationGlobal
from pymavlink import mavutil

#---- Import keyboard dependencies
import Tkinter as tk

#---- Connect to the vehicle
connection_string = "127.0.0.1:14550"
baudRate = 57600
vehicle = connect(connection_string, baud = baudRate, wait_ready = True)

#---- Speed Command Setup
quadSpeed = 5 				#-- Define the speed [m/s]

#---- Define arm and take off quadrotor
def arm_and_takeoff(altitude):

#	while not vehicle.is_armable:
#		print("waiting to be armable")
#		time.sleep(1)

	print("Arming motors")
	vehicle.mode  = VehicleMode("GUIDED")
	vehicle.armed = True

	while not vehicle.armed:
	time.sleep(1)

	print("takeoff")
	vehicle.simple_takeoff(altitude)

	while True:
	veloAltitude = vehicle.location.global_relative_frame.alt
	print("Altitude: %.1f m" % veloAltitude)

	if veloAltitude >= altitude - 1.0:
		print("Target altitude reached")
		break
	time.sleep(1)

#---- Define the function to send mavlink velocity command in body frame
def set_velocity_body(vehicle, vx, vy, vz):
	""" vz positive is downward
	"""
	msg = vehicle.message_factory.set_position_target_local_ned_encode(
		0,
		0, 0,
		mavutil.mavlink.MAV_FRAME_BODY_NED,
		0b0000111111000111 		#-- bitmask
		0, 0, 0,			#-- Position
		vx, vy, vz,			#-- velocity
		0, 0, 0,			#-- acceleration
		0, 0)
	vehicle.send_mavlink(msg)
	vehicle.flush()


#---- Keyboard event
def key(event):
	if event.char == event.keysym: 	#-- standard keys
		if event.keysym == 'r':
			print("Set vehicle to RTL")
			vehicle.mode = VehicleMode("RTL")

	else:  				#-- non-standard keys
		if event.keysym == 'Up':
			set_velocity_body(vehicle, quadSpeed, 0, 0)
		elif event.keysym == 'Down':
			set_velocity_body(vehicle, -quadSpeed, 0, 0)
		elif event.keysym == 'Left':
			set_velocity_body(vehicle, 0, -quadSpeed, 0)
		elif event.keysym == 'Right':
			set_velocity_body(vehicle, 0, quadSpeed, 0)

#---- Main Function
#---  Take off
arm_and_takeoff(10)

#--- Read keyboard input
root = tk.Tk()
print("Control the quadrotor using arrows keys. Press r to change to RTL mode")
root.bind_all('<key>', key)
root.mainloop()






