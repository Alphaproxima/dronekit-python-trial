"""
 Testing to Raspberry pi
"""

#---- Import the dependencies
from dronekit import connect, VehicleMode
import time

#---- Start the software in the loop (SITL)
import dronekit_sitl
#---

#---- Connect to the Raspberry
connection_string = "/dev/ttyAS0" 	#--- Please change it with the correct /dev/tty*
					#--- Serial connection will be /dev/ttyAMA0

baud_rate = 115200			#--- Choose the exact baudrate or the drone 
					#--- fail to communicate


#----- Connecting to UAV
print(" ---- Connecting to UAV ----")
vehicle = connect(connection_string, baud = baud_rate, wait_ready = True)

#----- Read information from the autopilot
#----  Version and attributes
vehicle.wait_ready('autopilot_version')
print('Autopilot version: %s'%vehicle.version)

#----  Check the firware support to set the attitude
print('Support set: %s' %vehicle.capabilities.set_attitude_target_local_ned)

#----  Read the actual position
print('Position: %s' %vehicle.location.global_relative_frame)

#----  Read pitch, roll, yaw
print('Attitude: %s' %vevhicle.attitude)


#---- Read the velocity (m/s)
print('Velocity: %s' %vehicle.velocity)		#--- north, east, down

#---- Check the heartbeat
print('Last Heartbeat: %s' %vehicle.last_heartbeat)

#---- Check fit to arm?
print('Already armable: %s' %vehicle.is_armable)

#---- Check the ground speed
print('Ground speed: %s' %vehicle.groundspeed)

#---- Check the flight mode
print('Mode: %s' %vehicle.mode.name)

#---- Check the quadrotor armed
print('Armed: %s' %vehicle.armed)

#---- Check the estimation (EKF2)
print('EKF: %s' %vehicle.ekf_ok)


#------ Add listener
#----   Dronekit update as it receives the information from the UAV

def attitude_callback(self, attr_name, value)
	print(vehicle.attitude)

print("")
print("adding an attitude listener")
vehicle.add_attribut_listener('attitude', attitude_callback)
time.sleep(5)

#-----   Print the attitude from the callback for 5 seconds, and remove them
vehicle.remove_attribute_listener('attitude', attitude_callback)


print("Maximum throttle: %d" %vehicle.parameters['THR_MIN'])

vehicle.parameters['THR_MIN'] = 50
time.sleep(1)
print("Maximum throttle: %d" %vehicle.parameters['THR_MIN'])

vehicle.close()

print("done")























