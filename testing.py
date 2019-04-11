from pymavlink import mavutil
from dronekit import connect, VehicleMode, GlobalPosition, LocationGlobalRelative

import time
import argparse
import RPi.GPIO as GPIO

parser = argparse.ArgumentParser()
parser.add_argument('--connect', default='192.168.208.96:14550')
args = parser.parse_args()

#-- Define baudrate of the pixhawk
baudRate = 115200

connection_string = arg.connects

def readSensordata(self)
	print "Enter the data sensor"
	


def callbackDataSensor(self, name, msg):
	pass

@vehicle.on_message('*')
def listener(self, name, message):
	print 'message: %s' %message
