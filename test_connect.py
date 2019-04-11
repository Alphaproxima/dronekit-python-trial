from dronekit import connect, VehicleMode
import time

# --- start software in the loop (SITL)
import dronekit_sitl

connection_string = "/dev/ttyACM0" # -- for serial port connection
					# -- "dev/ttyAMA0

baudRate = 115200


print(" Connecting to UAV ")
vehicle = connect(connection_string, baud = baudRate, wait_ready = True)

vehicle.wait_ready('autopilot version')
print('Autopilot version: %s' %vehicle.version)

print('Supports set attitude from companion: %s' %vehicle.capabilities.set_attitude_target_local_ned)

print('Position: %s' %vehicle.location.global_relative_frame)

print('Attitude: %s' %vehicle.attitude)

print('Velocity: %s' %vehicle.velocity)

print('Last Heartbeat: %s' %vehicle.last_heartbeat)

print('Is the vehicle armable: %s' %vehicle.is_armable)

print('Groundspeed: %s' %vehicle.groundspeed)

print('Mode: %s' %vehicle.mode.name)

print('Armed: %s' %vehicle.armed)

print('EKF ok: %s' %vehicle.ekf_ok)


def attitude_callback(self, attr_name, value):
	print(vehicle.attitude)

print("")
print("Adding an attitude listener")
vehicle.add_attribute_listener('attitude', attitude_callback)
time_sleep(5)

vehicle.remove_attribute_listener('attitude', attitude_callback)


print("Maximum Throttle: %d" %vehicle.parameters['THR_MIN'])

vehicle.parameter['THR_MIN'] = 50
time.sleep(1)
print("Maximum Throttle: %d" %vehicle.parameters['THR_MIN'])

vehicle.close()

print("done")


