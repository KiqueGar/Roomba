#Robot Class
#
#Python interace for roomba
#
# Zach Dodds	dodds@cs.hmc.edu
# updated for SIGCSE 3/9/07
#
# modified by Sean Luke Oct 20 2007
#
# modified by James O'Beirne Dec 9 2009
#	1. Two new functions (senseFunc and sleepTill).
#	2. getPose fixed (call to r.sensors([POSE]) added to stop()).
#	3. move() changed to accept parameters in centimeters instead of milimeters
#		for consistency with printSensors/getPose.
#
# modified by Martin Schaef Feb 2016
# https://github.com/martinschaef/roomba
#	1. Added support for dirt sensor, encoders, and light bump.
#	2. Adjusted the size of the Roomba in WHEEL_SPAN
#	3. getPose seems to be broken.
#
# modified by Enrique April 2017

import serial
from math import *
import time

#some module-level definitions for the robot commands
# As per "iRobot_Roomba_500_Open_Interface_Spec.pdf" page 29
START = 128	 # already converted to bytes...
BAUD = 129	  # + 1 byte
CONTROL = 130  # deprecated for Create
SAFE = 131
FULL = 132
POWER = 133
SPOT = 134	  # Same for the Roomba and Create
CLEAN = 135	 # Clean button - Roomba
MAX = 136		# Roomba
DRIVE = 137	 # + 4 bytes
MOTORS = 138	# + 1 byte
LEDS = 139	  # + 3 bytes
SONG = 140	  # + 2N+2 bytes, where N is the number of notes
PLAY = 141	  # + 1 byte
QUERY = 142  # + 1 byte
#SENSORS = chr(142)  # + 1 byte
FORCESEEKINGDOCK = 143  # same on Roomba and Create
# the above command is called "Cover and Dock" on the Create
PWMMOTORS = 144	# [MainBrush, SideBrush, Vaccum] in PWM values
DRIVEDIRECT = 145		 # Create only
DRIVEPWM = 146	#
STREAM = 148		 # Create only
QUERYLIST = 149		 # Create only
DOSTREAM = 150		 # Create only
#PAUSERESUME = chr(150)		 # Create only

# the four SCI modes
# the code will try to keep track of which mode the system is in,
# but this might not be 100% trivial...
OFF_MODE = 0
PASSIVE_MODE = 1
SAFE_MODE = 2
FULL_MODE = 3

mode_list=[[POWER, OFF_MODE], [START, PASSIVE_MODE], [SAFE, SAFE_MODE], [FULL, FULL_MODE]]

#Sensors:
# As per "iRobot_Roomba_500_Open_Interface_Spec.pdf" page 55
BUMPS_AND_WHEEL_DROPS = 7
WALL_IR_SENSOR = 8
CLIFF_LEFT = 9
CLIFF_FRONT_LEFT = 10
CLIFF_FRONT_RIGHT = 11
CLIFF_RIGHT = 12
VIRTUAL_WALL = 13
LSD_AND_OVERCURRENTS = 14
DIRT_DETECTED = 15
INFRARED_BYTE = 17
BUTTONS = 18
DISTANCE = 19
ANGLE = 20
CHARGING_STATE = 21
VOLTAGE = 22
CURRENT = 23
BATTERY_TEMP = 24
BATTERY_CHARGE = 25
BATTERY_CAPACITY = 26
WALL_SIGNAL = 27
CLIFF_LEFT_SIGNAL = 28
CLIFF_FRONT_LEFT_SIGNAL = 29
CLIFF_FRONT_RIGHT_SIGNAL = 30
CLIFF_RIGHT_SIGNAL = 31
CARGO_BAY_DIGITAL_INPUTS = 32
CARGO_BAY_ANALOG_SIGNAL = 33
CHARGING_SOURCES_AVAILABLE = 34
OI_MODE = 35
SONG_NUMBER = 36
SONG_PLAYING = 37
NUM_STREAM_PACKETS = 38
REQUESTED_VELOCITY = 39
REQUESTED_RADIUS = 40
REQUESTED_RIGHT_VELOCITY = 41
REQUESTED_LEFT_VELOCITY = 42
ENCODER_LEFT = 43
ENCODER_RIGHT = 44
LIGHTBUMP = 45
LIGHTBUMP_LEFT = 46
LIGHTBUMP_FRONT_LEFT = 47
LIGHTBUMP_CENTER_LEFT = 48
LIGHTBUMP_CENTER_RIGHT = 49
LIGHTBUMP_FRONT_RIGHT = 50
LIGHTBUMP_RIGHT = 51

def _bitOfByte( bit, byte ):
	 """ returns a 0 or 1: the value of the 'bit' of 'byte' """
	 if bit < 0 or bit > 7:
		  print ('Your bit of', bit, 'is out of range (0-7)')
		  print ('returning 0')
		  return 0
	 return ((byte >> bit) & 0x01)

def modeStr(mode):
	""" prints a string representing the input SCI mode """
	if mode == OFF_MODE: return 'OFF_MODE'
	if mode == PASSIVE_MODE: return 'PASSIVE_MODE'
	if mode == SAFE_MODE: return 'SAFE_MODE'
	if mode == FULL_MODE: return 'FULL_MODE'
	print ('Warning: unknown mode', mode, 'seen in modeStr')
	return 'UNKNOWN_MODE'

def _toTwosComplement2Bytes( value ):
	"""Returns 2 bytes from int in two's complements"""
	if value >=0:
		eqBitVal = value
	#For negative:
	else:
		eqBitVal = (1<<16) + value
	return ( (eqBitVal >> 8) & 0xFF, eqBitVal & 0xFF )

class robot:
	"""iRobot Roomba Open interface and robot class from udacity"""
	def __init__(self, length=500):
		"""Creates a robot and intiliazes location/orientation to zeros"""

		### THIS MAY NEED TO BE CHANGES AS ROBOT MODEL MIGHT BE DIFFERENT ###
		self.x = 0.0
		self.y = 0.0
		self.orientation = 0.0
		self.length = length
		self.steering_noise	 = 0.0
		self.distance_noise	 = 0.0
		self.measurement_noise = 0.0
		self.num_collisions	 = 0
		self.num_steps			= 0
		self.WHEEL_SPAN = 233.0

		# Setting OI mode
		self.sciMode = OFF_MODE
		self._debug=False

		  #self.leftEncoder = -1
		  #self.rightEncoder = -1

	def OIsetMode(self, mode):
		time.sleep(.3)
		#print("Writing to serial: ", hex(int(mode_list[mode][0])))
		self._write(mode_list[mode][0])
		time.sleep(.3)
		#print("Internal state to: ", mode_list[mode][1])
		self.sciMode = mode_list[mode][1]
		print(modeStr(mode_list[mode][1]))
		#return
	
	### SERIAL functions
	def _write(self, byte):
		if self._debug==True:
			print (hex(byte))
		#self.ser.write(byte.encode())
		self.ser.write(bytes([byte]))
	
	def _writeBytes(self, list):
		for i in list:
			self._write(i)

	def connect(self, PORT, BAUD_RATE=115200, startingMode=SAFE_MODE):
		""" the constructor which tries to open the
		  connection to the robot at port PORT
		  """
		print("Using port", PORT)
		if type(PORT) == type('string'):
				if PORT == 'sim':
					 print ('In simulated mode...')
					 self.ser = 'sim'; # SRSerial('mapSquare.txt')
				else:
					 # for Mac/Linux - use whole port name
					 # print 'In Mac/Linux mode...'
					self.ser = serial.Serial(PORT, baudrate=BAUD_RATE, timeout=0.5)
		  # otherwise, we try to open the numeric serial port...
		else:
			# print 'In Windows mode...'
			self.ser = serial.Serial(PORT-1, baudrate=BAUD_RATE, timeout=0.5)
		# did the serial port actually open?
		if self.ser != 'sim' and self.ser.isOpen():
			print ('Serial port did open, presumably to a roomba...')
		else:
			print ('Serial port did NOT open, check the')
			print ('  - port number')
			print ('  - physical connection')
			print ('  - baud rate of the roomba (it\'s _possible_, if unlikely,')
			print ('				  that it might be set to 19200 instead')
			print ('				  of the default 57600 - removing and')
			print ('				  reinstalling the battery should reset it.')
		  
		#Start OI
		self.OIsetMode(PASSIVE_MODE)
		if self.sciMode != startingMode:
			self.OIsetMode(startingMode)

	def disconnect(self):
		print("Shutting down robot...")
		self.OIsetMode(1)
		print("Closing port...")
		self.ser.close()

	def _drive(self, roomba_mm_sec, roomba_radius_mm, turn_inplace=False):
		"""Drives the robot as linear speed, radius to center, note special
		cases"""
		#Cast to int, if not
		if type(roomba_mm_sec) != type(10):
			roomba_mm_sec=int(roomba_mm_sec)
		if type(roomba_radius_mm) != type(10):
			roomba_radius_mm = int(roomba_radius_mm)

		#Prune invalid values
		if roomba_mm_sec < -500:
			roomba_mm_sec = -500
		if roomba_mm_sec > 500:
			roomba_mm_sec = 500
		#If too big a radius, is a straight line!
		if ((roomba_radius_mm < -2000)|(roomba_radius_mm > 2000)):
			roomba_radius_mm = 32768

		velHighVal, velLowVal = _toTwosComplement2Bytes( roomba_mm_sec )

		if turn_inplace:
			if roomba_radius_mm==0:
				roomba_radius_mm= 1
			radiusHighVal, radiusLowVal = _toTwosComplement2Bytes( roomba_radius_mm )
		else:
			if (roomba_radius_mm == 0 | roomba_radius_mm == 1):
				roomba_radius_mm = 2
			if roomba_radius_mm == -1:
				roomba_radius_mm = -2
			radiusHighVal, radiusLowVal = _toTwosComplement2Bytes( roomba_radius_mm )

		if self._debug:
			print("Driving with Vel and radius Values: ")

		out=[DRIVE, chr(velHighVal), chr(velLowVal), chr(radiusHighVal), chr(radiusLowVal)]
		#self._writeBytes(out)
		self._write( DRIVE )
		self._write(velHighVal)
		self._write(velLowVal)
		self._write(radiusHighVal)
		self._write(radiusLowVal)


	def setLeds(self, color, intensity, play, advance):
		raise NotImplemented

	### Other things

	def set(self, new_x, new_y, new_orientation):
		"""Sets a new position and orientation in world """
		self.x = float(new_x)
		self.y = float(new_y)
		self.orientation = float(new_orientation) % (2.0 * pi)

	def move(self, distance, steering, max_steering=pi/4):
		raise NotImplemented
