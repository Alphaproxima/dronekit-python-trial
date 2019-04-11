from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import serial
import math

import argparse
parser = argparse.ArgumentParser()
parser.add_argument('--connect', default='127.0.0.1:14550')
args = parser.parse_args()

connection_string = args.connect
sitl = None
#--- Set serial connection for tfmini lidar
#--- Note: baudrate of the sensor should be 115200
ser = serial.Serial("/dev/ttyUSB0", 115200)

#--- Start dronekit-sitl if no connection declared
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()

#--- Connect the vehicle
print 'Connecting the vehicle on: %s' % args.connect
vehicle = connect(connection_string, baud=57600, wait_ready = True)

"""
    This code below will control the data stream from tfmini Lidar
    We should note, that this sensor is attached to raspberry
    through the usb cable
    So, define the serial communication should be checked carefully
    use ls -l /dev/tty* to check the active port
"""
#--- Create function for TFMini Lidar
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

#--- Because tfmini lidar read is depending on "active serial"
#--- I still find an efficient way to get the data in python
def activateSensor():
    try:
        if ser.is_open == False:
            ser.open()
        while True:
            data = getTFminiData()
            if data > 200 and data != None:
                print data
            if data < 200 and data != None:
                print "threshold reached"
                set_attitude(pitch_angle = 5, thrust = 0.5)

    except KeyboardInterrupt:
        if ser != None:
            ser.close()


"""
    Define global velocity
    in the tutorial it says that this function control
    the behavior of the velocity of the quadrotor
"""
def send_global_velocity(velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    msg = vehicle.message_factory.set_position_target_global_int_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, # lat_int - X Position in WGS84 frame in 1e7 * meters
        0, # lon_int - Y Position in WGS84 frame in 1e7 * meters
        0, # alt - Altitude in meters in AMSL altitude(not WGS84 if absolute or relative)
        # altitude above terrain if GLOBAL_TERRAIN_ALT_INT
        velocity_x, # X velocity in NED frame in m/s
        velocity_y, # Y velocity in NED frame in m/s
        velocity_z, # Z velocity in NED frame in m/s
        0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    # send command to vehicle on 1 Hz cycle
    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)

"""
    This is the function to control the heading of quadrotor
    on the perspective of yaw
"""
def condition_yaw(heading, relative=False):
    if relative:
        is_relative=1 #yaw relative to direction of travel
    else:
        is_relative=0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)

"""
    send the ned velocity
    let's say, if there is an obstacle detected in the front of
    quadrotor
    This function will enable us to move backward etc

    # Set up velocity mappings
    # velocity_x > 0 => fly North
    # velocity_x < 0 => fly South
    # velocity_y > 0 => fly East
    # velocity_y < 0 => fly West
    # velocity_z < 0 => ascend
    # velocity_z > 0 => descend

    UP is negative!
"""
def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)


    # send command to vehicle on 1 Hz cycle
    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)

"""
    We want to activate the arming and takeoff function
    This function is supported with a controlable Thrust
    But, in nogps environment. This function is not work
    Be sure to update the dronekit into the latest version
"""
#--- Create a function for arming and takeoff after that
def arm_and_takeoff(setTargetAltitude):

    #--- Try to control the thrust power
    DEFAULT_TAKEOFF_THRUST = 0.7
    SMOOTH_TAKEOFF_THRUST = 0.6

    print "Basic pre-arm checks"
	# Do not let the user arm until autpilot is ready
	while not vehicle.is_armable:
        print "Waiting for vehicle to initialise ..."
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

    #--- Set the thrust
    thrust = DEFAULT_TAKEOFF_THRUST

	# Check the altitude of the quadrotor already reached
	while True:
        current_altitude = vehicle.location.global_relative_frame.alt
        print ("Altitude: %f Target: %f") %(current_altitude, setTargetAltitude)

        # Break and return from function below
		if current_altitude >= setTargetAltitude*0.95:
			print "Reached target altitude"
			break

        elif current_altitude >= setTargetAltitude*0.6:
            thrust = SMOOTH_TAKEOFF_THRUST
        set_attitude(thrust = thrust)
		time.sleep(0.2)

"""
    Send attitude target function will be the gate to control
    our quadrotor attitude based on quaternions
    we can control the movement as well using this code

    This function works in GUIDED and GUIDED_NOGPS Mode
    be careful to set the VehicleMode in the arm_and_takeoff function
"""

def send_attitude_target(roll_angle = 0.0, pitch_angle = 0.0,
                         yaw_angle = None, yaw_rate = 0.0, use_yaw_rate = False,
                         thrust = 0.5):
    """
    use_yaw_rate: the yaw can be controlled using yaw_angle OR yaw_rate.
                  When one is used, the other is ignored by Ardupilot.
    thrust: 0 <= thrust <= 1, as a fraction of maximum vertical thrust.
            Note that as of Copter 3.5, thrust = 0.5 triggers a special case in
            the code for maintaining current altitude.
    """
    if yaw_angle is None:
        # this value may be unused by the vehicle, depending on use_yaw_rate
        yaw_angle = vehicle.attitude.yaw
    # Thrust >  0.5: Ascend
    # Thrust == 0.5: Hold the altitude
    # Thrust <  0.5: Descend
    msg = vehicle.message_factory.set_attitude_target_encode(
        0, # time_boot_ms
        1, # Target system
        1, # Target component
        0b00000000 if use_yaw_rate else 0b00000100,
        to_quaternion(roll_angle, pitch_angle, yaw_angle), # Quaternion
        0, # Body roll rate in radian
        0, # Body pitch rate in radian
        math.radians(yaw_rate), # Body yaw rate in radian/second
        thrust  # Thrust
    )
    vehicle.send_mavlink(msg)

"""
    Using this code we will set the heading of the quadrotor
"""
def set_attitude(roll_angle =0.0, pitch_angle = 0.0, yaw_angle = None,
                 yaw_rate = 0.0, use_yaw_rate = False, thrust = 0.5, duration = 0)

        send_attitude_target(roll_angle,pitch_angle,yaw_angle,yaw_rate,False,
                            thrust)
        start = time.time()
        while time.time() - start < duration:
            send_attitude_target(roll_angle,pitch_angle,yaw_angle,yaw_rate,
                                 False, thrust)
            time.sleep(0.1)
        send_attitude_target(0,0,0,0,True,thrust)

"""
    Quaternion helps us to control the attitude of quadrotor
"""

def to_quaternion(roll = 0.0, pitch = 0.0, yaw = 0.0):
    """
    Convert degrees to quaternions
    """
    t0 = math.cos(math.radians(yaw * 0.5))
    t1 = math.sin(math.radians(yaw * 0.5))
    t2 = math.cos(math.radians(roll * 0.5))
    t3 = math.sin(math.radians(roll * 0.5))
    t4 = math.cos(math.radians(pitch * 0.5))
    t5 = math.sin(math.radians(pitch * 0.5))

    w = t0 * t2 * t4 + t1 * t3 * t5
    x = t0 * t3 * t4 - t1 * t2 * t5
    y = t0 * t2 * t5 + t1 * t3 * t4
    z = t1 * t2 * t4 - t0 * t3 * t5

    return [w, x, y, z]

"""
    example of code
    (1) We arm and take off the quadrotor at the desired Altitude
    (2) Move Forward
    (3) backward
    (4) Actually this will be landing.
"""
#--- takeoff altitude is 2 meters
arm_and_takeoff(2)

print("Hold the Position for 3 seconds")
set_attitude(duration = 3)

print("Forward for 3 seconds")
set_attitude(pitch_angle = -5, thrust = 0.5, duration = 3.21)

print("backward for 3 seconds")
set_attitude(pitch_angle = 5, thrust = 0.5, duration = 3.21)

print("LAND")
vehicle.mode = VechileMode("LAND")
time.sleep(1)

print("Close the vehicle object")
vehicle.close()

if  sitl is not None:
    sitl.stop()

print("Done")
