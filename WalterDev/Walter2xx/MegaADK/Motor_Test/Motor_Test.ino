/*
	W.A.L.T.E.R. 2.0: Motor and Controller Test sketch.
	Copyright (C) 2014 Dale A. Weber <hybotics.pdx@gmail.com>

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/*
	Program:		W.A.L.T.E.R. 2.0, Motor and Controller Test sketch
	Date:			01-Jul-2014
	Version:		0.1.1 Arduino Mega ADK - ALPHA

	Purpose:		Test Motors and Motor Controllers
						
	Dependencies:	Adafruit libraries:
						RTClib for the DS1307

					Other libraries:
						None

	Comments:		Credit is given, where applicable, for code I did not originate.
						This sketch started out as an Adafruit tutorial for the electret
						microphones being used for sound detection. I've also pulled
						code for the GP2Y0A21YK0F IR and PING sensors from the Arduino
	    				Playground, which I have modified to suit my needs.
*/

#include <Wire.h>

#include <RTClib.h>

#include <BMSerial.h>
#include <RoboClaw.h>


/*
	Additional libraries and local includes
*/

#include "Motor_Test.h"
#include "Pitches.h"

/********************************************************************/
/*	Global objects													*/
/********************************************************************/

RTC_DS1307 clock;

/********************************************************************/
/*	Initialize global variables										*/
/********************************************************************/

/*
	Time control variables
*/
uint8_t currentMinute = 0;
uint8_t lastMinute = -1;
long minuteCount = 0;						//	Count the time, in minutes, since we were last restarted

//	Enable run once only loop initialization code to run for special cases
bool firstLoop = true;

/*
	Setup all our serial devices
*/

//	Hardware Serial: Console and debug (replaces Serial.* routines)
BMSerial console(SERIAL_CONSOLE_RX_PIN, SERIAL_CONSOLE_TX_PIN, 10000);

//	Hardware Serial1: RoboClaw 3x5 Motor Controller
RoboClaw roboClaw(SERIAL_ROBOCLAW_RX_PIN, SERIAL_ROBOCLAW_TX_PIN, 10000, false);

//	Hardware Serial2: SSC-32 Servo Controller
BMSerial ssc32(SERIAL_SSC32_RX_PIN, SERIAL_SSC32_TX_PIN);

//	Hardware Serial3: XBee Mesh Wireless
BMSerial xbee(SERIAL_XBEE_RX_PIN, SERIAL_XBEE_TX_PIN);

//	We only have one RoboClaw 2x5 right now
uint8_t roboClawControllers = ROBOCLAW_CONTROLLERS - 1;
uint8_t	roboClawBaseAddress = ROBOCLAW_SERIAL_BASE_ADDR;
uint8_t roboClawAddress1 = ROBOCLAW_SERIAL_BASE_ADDR;
uint8_t roboClawAddress2 = ROBOCLAW_SERIAL_BASE_ADDR + 1;

//	Left side motor (M1) - RoboClaw 2x5 Controller #1 (0x80)
GearMotor leftMotorM2;

//	Right side motor (M2) - RoboClaw 2x5 Controller #1 (0x80)
GearMotor rightMotorM1;

//	Front motor (M1) - RoboClaw 2x5 Controller #2 (0x81)
GearMotor frontMotorM1;

//	Back motor (M2) - RoboClaw 2x5 Controller #2 (0x81)
GearMotor backMotorM2;

/*
	Code starts here
*/

/********************************************************************/
/*	Utility routines 												*/
/********************************************************************/

/*
    Process error conditions
*/
void processError (uint16_t err, String routine, String message) {
	console.print(F("ERROR: "));
	console.print(message);
	console.print(F(", Code = "));
	console.print(err);
	console.print(F(", in routine '"));
	console.print(routine);
	console.println(F("' !"));
}

/*
	Wait for a number of seconds to allow time to read the Console Serial Monitor log
*/
void wait (uint8_t nrSeconds) {
	uint8_t count;

	console.print(F("Waiting"));

	for (count = 0; count < nrSeconds; count++) {
		console.print(F("."));
		delay(1000);
	}

	console.println();
}

/*
    Pulses a digital pin for a duration in ms
*/
void pulseDigital(int pin, int duration) {
	digitalWrite(pin, HIGH);			// Turn the ON by making the voltage HIGH (5V)
	delay(duration);					// Wait for duration ms
	digitalWrite(pin, LOW);				// Turn the pin OFF by making the voltage LOW (0V)
	delay(duration);					// Wait for duration ms
}

/********************************************************************/
/*	Display routines 												*/
/********************************************************************/

/*
	Display the data for a given motor
*/
void displayGearMotor (GearMotor *gearMotor) {
	console.print(gearMotor->name);
	console.println(F(" Gear Motor:"));

	//	Using Packet Serial
	console.print(F("Encoder is valid: "));

	if (gearMotor->encoderValid) {
		console.print(F("Yes, Status: "));
		console.print(gearMotor->encoderStatus);
		console.print(F(", Value: "));
		console.println(gearMotor->encoder);
	} else {
		console.println(F("No"));
	}

	console.print(F("Speed is valid: "));

	if (gearMotor->speedValid) {
		console.print(F("Yes, Status: "));
		console.print(gearMotor->speedStatus);
		console.print(F(", Speed: "));
		console.println(gearMotor->speed);
	} else {
		console.println(F("No"));
	}

	console.print(F("Moving "));

	if (gearMotor->forward) {
		console.print(F("Forward"));
	} else {
		console.print(F("Reverse"));
	}

	console.print(F("Distance is valid: "));

	if (gearMotor->distanceValid) {
		console.print(F("Yes, Distance: "));
		console.println(gearMotor->distance);
	} else {
		console.println(F("No"));
	}
}

/*
	Display data from the RoboClaw 2x5 motor controller
*/
void displayRoboClawData (uint8_t address, GearMotor *rightMotorM1, GearMotor *leftMotorM2) {
	char version[32];

	roboClaw.ReadVersion(address, version);

	console.print(F("RoboClaw 2x5 status (version "));
	console.print(version);
	console.println(F(")"));
	console.println();

	if (rightMotorM1->encoderValid) {
		console.print(F("Right Motor Encoder = "));
		console.print(rightMotorM1->encoder, DEC);
		console.print(F(", Status = "));
		console.print(rightMotorM1->encoderStatus, HEX);
		console.println();
	}

	if (rightMotorM1->speedValid) {
		console.print(F("Right Motor Speed = "));
		console.print(rightMotorM1->speed, DEC);
		console.println();
	}

    if (leftMotorM2->encoderValid) {
		console.print(F("Left Motor Encoder = "));
		console.print(leftMotorM2->encoder, DEC);
		console.print(F(", Status =  "));
		console.print(leftMotorM2->encoderStatus, HEX);
		console.println();
	}

	if (leftMotorM2->speedValid) {
		console.print(F("Left Motor Speed = "));
		console.print(leftMotorM2->speed, DEC);
		console.println();
	}
	
	console.println();
}

/*
	Display the data in a Servo struct
*/
void displayServo (Servo *servo) {
	console.println();
	console.print(servo->name);
	console.println(F(" Servo:"));
	console.println();

	console.print(F("Pin = "));
	console.print(servo->pin);
	console.print(F(", msPulse = "));
	console.print(servo->msPulse);
	console.print(F(", Offset = "));
	console.println(servo->offset);

	console.print(F("Home Pos = "));
	console.print(servo->homePos);
	console.print(F(", Angle = "));
	console.println(servo->angle);

	console.print(F("Min Pw = "));
	console.print(servo->minPulse);
	console.print(F(", Max Pw = "));
	console.print(servo->maxPulse);
	console.print(F(", Max Deg = "));
	console.print(servo->maxDegrees);
	console.print(F(", Error = "));
	console.println(servo->error);
	console.println();
}

/*
	Read current data from the RoboClaw 2x5 Motor Controller
*/
uint16_t readRoboClawData (uint8_t address, GearMotor *rightM1, GearMotor *leftM2) {
	//	Error control
	uint16_t errorStatus = 0;

	bool valid;
	uint8_t status;
	String errorMsg;

	console.println(F("Reading Right Motor Encoder.."));

	rightM1->encoder = roboClaw.ReadEncM2(address, &status, &valid);
	rightM1->encoderStatus = status;
	rightM1->encoderValid = valid;

	console.println(F("Reading Right Motor Speed.."));

	rightM1->speed = roboClaw.ReadSpeedM2(address, &status, &valid);
	rightM1->speedStatus = status;
	rightM1->speedValid = valid;

	console.println(F("Reading Left Motor Encoder.."));

	leftM2->encoder = roboClaw.ReadEncM1(address, &status, &valid);
	leftM2->encoderStatus = status;
	leftM2->encoderValid = valid;

	console.println(F("Reading Left Motor Speed.."));

	leftM2->speed = roboClaw.ReadSpeedM1(address, &status, &valid);
	leftM2->speedStatus = status;
	leftM2->speedValid = valid;

	return errorStatus;
}

/********************************************************************/
/*	Miscellaneous routines 											*/
/********************************************************************/

/********************************************************************/
/*	RoboClaw 2x5 Motor Controller routines 							*/
/********************************************************************/

uint16_t setGearMotorSpeed (uint8_t address, GearMotor *gearMotor, short spd) {
	uint16_t errorStatus = 0;

	if (gearMotor->location == Left) {
		//	Set left motor speed
		if (spd >= 0) {
			roboClaw.ForwardM1(address, spd);
			gearMotor->forward = true;
		} else {
			roboClaw.BackwardM1(address, -spd);
			gearMotor->forward = false;
		}
	} else if (gearMotor->location == Right) {
		//	Set right motor speed
		if (spd >= 0) {
			roboClaw.ForwardM2(address, spd);
			gearMotor->forward = true;
		} else {
			roboClaw.BackwardM2(address, -spd);
			gearMotor->forward = false;
		}
	} else {
		errorStatus = 601;
	}

	return errorStatus;
}

/*
	Set motor speeds

	The left and right motor speeds may be different.
*/
void setGearMotors (uint8_t address, GearMotor *rightM1, short rightSpd, GearMotor *leftM2, short leftSpd) {
	uint16_t errorStatus = 0;
	bool leftDir, rightDir;

	console.println(F("Setting motor speeds.."));

	errorStatus = setGearMotorSpeed(address, rightM1, rightSpd);
	errorStatus = setGearMotorSpeed(address, leftM2, leftSpd);

	updateGearMotors(address, rightM1, leftM2);
}

/*
	Update motor data
*/
void updateGearMotors (uint8_t address, GearMotor *rightM1, GearMotor *leftM2) {
        bool valid;
  	uint8_t speedStatus;
	uint32_t speed;

	console.println(F("Updating motors.."));

	//	Update right motor data
	speed = roboClaw.ReadSpeedM1(address, &speedStatus, &valid);

	rightM1->speed = speed;
	rightM1->speedStatus = speedStatus;
	rightM1->speedValid = valid;

	//	Update left motor data
	speed = roboClaw.ReadSpeedM2(address, &speedStatus, &valid);

	leftM2->speed = speed;
	leftM2->speedStatus = speedStatus;
	leftM2->speedValid = valid;
}

/********************************************************************/
/*	Initialization routines 										*/
/********************************************************************/

/*
	Initialize the RoboClaw 2x5 motor controller
*/
void initRoboClaw (uint8_t address, uint16_t bps, GearMotor *rightM1, GearMotor *leftM2) {
	console.print(F("Initializing the RoboClaw 2x5 Motor Controller at address "));
	console.print(address, HEX);
	console.print(F(", for "));
	console.print(bps);
	console.println(F(" Bps communication."));

	roboClaw.begin(bps);

	//	Set the RoboClaw motor constants
	roboClaw.SetM1VelocityPID(address, ROBOCLAW_KD, ROBOCLAW_KP, ROBOCLAW_KI, ROBOCLAW_QPPS);
	roboClaw.SetM2VelocityPID(address, ROBOCLAW_KD, ROBOCLAW_KP, ROBOCLAW_KI, ROBOCLAW_QPPS);

	//	For Packet Serial modes
	rightM1->name = ROBOCLAW_MOTOR_RIGHT_NAME;
	rightM1->location = Right;
	rightM1->encoder = 0;
	rightM1->encoderStatus = 0;
	rightM1->encoderValid = false;
	rightM1->speed = 0;
	rightM1->speedStatus = 0;
	rightM1->speedValid = false;
	rightM1->forward = true;
	rightM1->distance = 0;
	rightM1->distanceValid = false;		    

	//	For Packet Serial modes
	leftM2->name = ROBOCLAW_MOTOR_LEFT_NAME;
	leftM2->location = Left;
	leftM2->encoder = 0;
	leftM2->encoderStatus = 0;
	leftM2->encoderValid = false;
	leftM2->speed = 0;
	leftM2->speedStatus = 0;
	leftM2->speedValid = false;
	leftM2->forward = true;
	leftM2->distance = 0;
	leftM2->distanceValid = false;		    
}

/*
	Initialize sensors
*/
void initSensors (void) {
	console.println(F("Initializing Sensors.."));

	console.println(F("     DS1307 Real Time Clock.."));

	//	Check to be sure the RTC is running
//	if (! clock.isrunning()) {
//		console.println(F("The Real Time Clock is NOT running!"));
//		while(1);
//	}
}


/*
	Runs once to initialize everything
*/
void setup (void) {
	uint16_t errorStatus;

	//  Start up the Wire library
	Wire.begin();

	//  Initialize the console port
	console.begin(9600);

	console.println();
	console.print(F("W.A.L.T.E.R. 2.0 Motor and Controller Test, version "));
	console.print(BUILD_VERSION);
	console.print(F(" on "));
	console.print(BUILD_DATE);
	console.print(F(" for the "));
	console.print(BUILD_BOARD);
	console.println(F(" board"));

	console.println();

	console.println(F("Initializing Digital Pins.."));

	//  Initialize the LED pin as an output.
	pinMode(HEARTBEAT_LED, OUTPUT);
	digitalWrite(HEARTBEAT_LED, LOW);

	console.println();

	if (ROBOCLAW_CONTROLLERS > 0) {
		//	Initialize the RoboClaw 2x5 motor controller port
		initRoboClaw(roboClawAddress1, 38400, &rightMotorM1, &leftMotorM2);

		//	Let's see if we can move the motors forward
		console.println(F("Moving the motors forward for 10 seconds in setup.."));
		setGearMotors(roboClawAddress1, &rightMotorM1, WALTER_DEFAULT_MOVE_SPEED, &leftMotorM2, WALTER_DEFAULT_MOVE_SPEED);
		displayRoboClawData(roboClawAddress1, &rightMotorM1, &leftMotorM2);

		//	Wait 10 seconds
		wait(2);

		console.println(F("Data after moving forward.."));
		displayRoboClawData(roboClawAddress1, &rightMotorM1, &leftMotorM2);

		//	Wait 2 seconds
		wait(2);

		//	Stop the motors
		console.println(F("Stopping the motors in setup.."));
		setGearMotors(roboClawAddress1, &rightMotorM1, 0, &leftMotorM2, 0);

		//	Wait 2 seconds
		wait(2);

		//	Let's see if we can move the motors in reverse
		console.println(F("Moving the motors in reverse for 10 seconds in setup.."));
		setGearMotors(roboClawAddress1, &rightMotorM1, -WALTER_DEFAULT_MOVE_SPEED, &leftMotorM2, -WALTER_DEFAULT_MOVE_SPEED);
		displayRoboClawData(roboClawAddress1, &rightMotorM1, &leftMotorM2);

		//	Wait 10 seconds
		wait(2);

		console.println(F("Data after moving in reverse.."));
		displayRoboClawData(roboClawAddress1, &rightMotorM1, &leftMotorM2);

		//	Wait 2 seconds
		wait(2);

		//	Stop the motors
		console.println(F("Stopping the motors in setup.."));
		setGearMotors(roboClawAddress1, &rightMotorM1, 0, &leftMotorM2, 0);

		//	Wait 5 seconds
		wait(5);
	}
}

/*
	Runs forever
*/
void loop (void) {
	//	Error control
	uint16_t errorStatus = 0;
	String errorMsg;

	//	The current date and time from the DS1307 real time clock
	DateTime now = clock.now();

	//	Display related variables
	bool amTime, pitchRollValid = false, headingValid = false;
	uint8_t displayNr = 0, count = 0;
	uint8_t readingNr = 0, areaClosestReadingPING = 0, areaClosestReadingIR = 0;
	uint8_t areaFarthestReadingPING = 0, areaFarthestReadingIR = 0;
	uint8_t currentHour = now.hour(), nrDisplays = 0;
	uint16_t displayInt;

	uint8_t roboClawStatus;
	bool roboClawValid;

	uint8_t analogPin = 0;
	uint8_t digitalPin = 0;

	/*
		Code starts here
	*/

	// Pulse the heartbeat LED
	pulseDigital(HEARTBEAT_LED, 500);

	currentMinute = now.minute();

	/*
		This is code that only runs one time, to initialize
			special cases.
	*/
	if (firstLoop) {
		lastMinute = currentMinute;

		firstLoop = false;
	}

	if (ROBOCLAW_CONTROLLERS > 0) {
		/*
			Read the RoboClaw 2x5 motor data
		*/
		console.println(F("Reading RoboClaw.."));

		errorStatus = readRoboClawData(roboClawAddress1, &rightMotorM1, &leftMotorM2);

		if (errorStatus != 0) {
			processError(errorStatus, "readRoboClawData", "Unhandled problem");
		} else {
			displayRoboClawData(roboClawAddress1, &rightMotorM1, &leftMotorM2);
		}

		/*
			Read the RoboClaw 2x5 motor data
		*/
		console.println(F("Reading RoboClaw.."));

		errorStatus = readRoboClawData(roboClawAddress1, &rightMotorM1, &leftMotorM2);

		if (errorStatus != 0) {
			processError(errorStatus, "readRoboClawData", "Unhandled problem");
		} else {
			displayRoboClawData(roboClawAddress1, &rightMotorM1, &leftMotorM2);
		}
	}

	console.println();

	//	Count the minutes
	if (currentMinute != lastMinute) {
		minuteCount += 1;
		lastMinute = currentMinute;
	}

	wait(WAIT_DELAY_SECONDS);

	console.println();
}
