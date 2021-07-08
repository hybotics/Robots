#!/usr/bin/env python3
import fcntl

class i2cBus ():
	bus = None
	busFileName = None
	busFileHandle = None

	debug = False
	
	def __init__ (self, busNr=None):
		if (busNr != None):
			self.bus = busNr
		else:
			self.bus = self.detect()

		self.busFileName = "/dev/i2c-" + str(self.bus)
		
		self.setDebug(False)
		
		return None

	'''
		Detect the I2C bus
		
		Will replace with real bus detection code
	'''
	def detect (self):
		status = True
		bus = 1
		
		return bus

	def beginTransmission (self):
		status = True

		if self.debug:
			print ("Opening " + self.busFileName + " for write")

		self.busFileHandle = open(self.busFileName, "w")
		
		return status
		
	def endTransmission (self):
		status = True

		if self.debug:
			print ("Closing I2C bus " + self.busFileName)

		self.busFileHandle.close()
		
		return status

	def setDebug (self, debug):
		self.debug = debug

		return None

if __name__=="__main__":
	arduinoAddress = 0x50

