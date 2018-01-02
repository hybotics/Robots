#!/usr/bin/env python3
import fcntl

from i2cDevice import i2cDevice

class i2cSlave (i2cDevice):
	device = None

	def __init__ (self, name, addr, busNr=None):
		self.address = addr
		self.device = i2cDevice(name, addr, busNr)
		self.setDebug(True)
		
		return None

if __name__=="__main__":
	arduinoAddress = 0x50

	arduino = i2cSlave("ArduinoSlave", arduinoAddress)

	arduino.device.bus.beginTransmission()
	
	arduino.device.bus.endTransmission()
