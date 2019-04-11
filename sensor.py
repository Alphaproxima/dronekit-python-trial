"""
 We aim to fully read and control the sensors which attached to quadrotor
 The first target is read the data from distance sensor	(tfmini or ultrasonic)
"""

from dronekit import connect, VehicleMode, LocationGlobalRelative
from mavlink import mavutil
import time


@vehicle.on_message('RANGEFINDER')
def listener(self, name, message)
	print 'distance: %s' % message.distance
	print 'voltage: %s' % message.voltage




def distanceSensor(tempValue)
	print "Distance sensor initialize"
	print 


