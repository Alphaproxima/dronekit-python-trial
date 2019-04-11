from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import argparse

parser = argparse.ArgumentParser(description='commands')
parser.add_argument('--connect')
args = parser.parse_args()
connection_string = "/dev/ttyAMA0"
#connection_string = args.connect
print("Connection to the vehicle on %s" %connection_string)
vehicle = connect(connection_string, wait_ready = True)

def arm_and_takeoff(targetAltitude):
	print("Arming the motors")
	
	while not vehicle.is_armable:
		time.sleep(1)

	vehicle.mode = VehicleMode("GUIDED")
	vehicle.armed = True

	print("Take off is initialized")
	time.sleep(1)
	print("Take off!")
	vehicle.simple_takeoff(targetAltitude)

	while True:
		print "Altitude: ", vehicle.location.global_relative_frame.alt
		# Break and return from function below
		if vehicle.location.global_relative_frame.alt >= targetAltitude*0.95:
			print "Reached target altitude"
			break
		time.sleep(1)

arm_and_takeoff(10)
print("Take off complete")

vehicle.airspeed = 7
print("Flying")
wpl = LocationGlobalRelative(-34.364114, 149.166022, 30)
vehicle.simple_goto(wpl)

time.sleep(30)
vehicle.mode = VehicleMode("RTL")
time.sleep(20)

print("Landing....")
vehicle.mode = VehicleMode("LAND")

vehicle.close()

