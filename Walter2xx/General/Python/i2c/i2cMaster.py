#!/usr/bin/env python3
import fcntl
from i2cBus import i2cBus
from i2cDevice import i2cDevice

class i2cMaster (i2cDevice):
	device = None
	
	def __init__ (self, name, addr=None, busNr=None):
		self.device = i2cDevice(name, addr, busNr)

		self.setDebug(True)
	
		return None

if __name__=="__main__":
	masterAddress = 0x60

	master = i2cDevice("Master")

	master.bus.beginTransmission()

	master.bus.endTransmission()

