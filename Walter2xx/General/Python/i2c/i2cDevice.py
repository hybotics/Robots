#!/usr/bin/env python3
import fcntl
from i2cBus import i2cBus

class i2cDevice ():
	address = None
	bus = None
	buffer = [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ]

	name = None

	debug = False
		
	def __init__ (self, name, addr=None, busNr=None):
		self.bus = i2cBus(busNr)
		self.bus.setDebug(True)

		self.name = name

		self.setDebug(True)

		if (addr != None):
			#	We are a Slave or Master
			if self.debug:
				print ("We are a Slave OR Master, called '" + self.name + "' with address")

			self.address = addr
		else:
			#	We are a Master
			if self.debug:
				print ("We are a Master, called '" + self.name + "' (no address)")
				
			self.address = None
		
		return None

	def readByte (self):
		status = True
		byte = 0xFF
		
		return ( byte, status )

	def sendByte (self, byte):
		status = True
		
		return status

	def writeByte (self, byte):
		status = True
		
		return status

	def readList (self):
		status = True
		byteList = [ 0xFF ]
		
		return ( byteList, status )

	def sendList (self, byteList):
		status = True
		
		return status

	def writeList (self, byteList):
		status = True

		return status
		
	def setDebug (self, debug):
		status = True
		self.debug = debug
		
		return status

if __name__=="__main__":
	arduinoAddress = 0x50

	arduino = i2cDevice("Arduino", arduinoAddress)
	arduino.bus.beginTransmission()

	arduino.bus.endTransmission()
