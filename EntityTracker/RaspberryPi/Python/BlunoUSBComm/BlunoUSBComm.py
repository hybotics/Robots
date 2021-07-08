#!/usr/bin/env python3

'''
	Program:		BlunoUSBComm.py
	Version:		0.0.2
	Date:			02-Sep-2014
	Purpose:		Main script to communicate with the Bluno for services

	Dependencies:	None
	
	Copyright (2014) by Dale Weber, hybotics.pdx@gmail.com, @hybotics on App.Net

	Changes:
		02-Sep-2014:
		Added code from my old SSC-32 Python example program on the Lynxmotion forums.

	ALL Rights Reserved.
'''

'''
	Imports
'''
import serial

EOL = "\r"
command = ""

# Set the SSC-32 port - "COM1:" for Windows, "/dev/ttyS0" for Linux
blunoPort = "/dev/ttyACM0"

LeftMChan = 0
LeftMStop = 1520

RightMChan = 1
RightMStop = 1525

PanChan = 2
PanHome = 1530
PanREnd = 2150
PanLEnd = 900

TiltChan = 3
TiltHome = 1430
TiltUEnd = 500
TiltDEnd = 1550

# Reads single characters until a CR is read
def Response (port):
   ich = ""
   resp = ""

   while (ich != '\r'):
      ich = port.read(1)
      
      if (ich != '\r'):
         resp = resp + ich
   return resp

# Converts a servo position in degrees to uS for the SSC-32
def To_Degrees (uS):
   result = 0
   
   result = (uS - 1500) / 10
   
   return result

# Converts an SSC-32 servo position in uS to degrees
def To_uS (degrees):
   result = 0
   
   result = (degrees * 10) + 1500
   
   return result

# Wait for a servo move to be completed
def Wait_for_Servos (port):
   
   ich = ""
   
   while (ich != "."):
      Send_Command (port, "Q", True)
      
      ich = port.read(1)
   
   return

# Send EOL to the SSC-32 to start a command executing
def Send_EOL (port):

   result = port.write(EOL)
   
   return result

# Send a command to the SSC-32 with or without an EOL
def Send_Command (port, cmd, sendeol):
   
   result = port.write (cmd)
   
   if (sendeol):
      Send_EOL (port)
   
   return result
   
#   Open the port at 115200 Bps - defaults to 8N1
bluno = serial.Serial(blunoPort, 115200)

#   Motors All Stop - Left = Ch0, Right = Ch1
command = "#" + str(LeftMChan) + " P" + str(LeftMStop) + " #" + str(RightMChan) + " P" + str(RightMStop)
Send_Command (bluno, command, True)

Wait_for_Servos (bluno)

#   Home the PING Pan/Tilt Base
command = "#" + str(PanChan) + " P" + str(PanHome) + " #" + str(TiltChan) + " P" + str(TiltHome)
Send_Command (bluno, command, True)

Wait_for_Servos (bluno)

#   Send the version command
command = "ver"
Send_Command (bluno, command, False)
Send_EOL (bluno)

#   Read the response
inp = Response(bluno)

#   Show what we got back
print (inp)

command = "#" + str(PanChan) + "P1900" + "#" + str(TiltChan) + "P750 T5000"
Send_Command (bluno, command, True)

Wait_for_Servos (bluno)

#   Close the port
bluno.close()

# Test the conversion functions
print ("600 uS = %d degrees.", To_Degrees(600))
print ("2400 uS = %d degrees.", To_Degrees(2400))
print ("1500 uS = %d degrees.", To_Degrees (1500))

print ("-90 degrees = %d uS.", To_uS(-90))
print ("90 degrees = %d uS.", To_uS(90))
print ("0 degrees = %d uS.", To_uS(0))
