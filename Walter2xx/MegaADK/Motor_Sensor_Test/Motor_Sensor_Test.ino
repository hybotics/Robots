/*
	W.A.L.T.E.R. 2.0: Motor and Sensor Test sketch.
	Copyright (C) 2013 Dale A. Weber <hybotics.pdx@gmail.com>

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
	Program:		W.A.L.T.E.R. 2.0, Motor and Sensor Test sketch
	Date:			16-Apr-2014
	Version:		0.0.2 Arduino Mega ADK - ALPHA

	Purpose:		To test motors and sensors for W.A.L.T.E.R. 2.0
						
	Dependencies:	Adafruit libraries:
						Adafruit_Sensor and Adafruit_L3GD20 libraries.

					Hybotics libraries:
						Hybotics_10DOF_Unified (forked from the Adafruit_10DOF library)
						Hybotics_LSM303DLHC_Unified (forked from the Adafruit_LSM303 library)

					Other libraries:
						RTClib for the DS1307 (Adafruit version),
						KalmanFilter

	Comments:		Credit is given, where applicable, for code I did not originate.
						This sketch started out as an Adafruit tutorial for the electret
						microphones being used for sound detection. I've also pulled
						code for the GP2Y0A21YK0F IR and PING sensors from the Arduino
	    				Playground, which I have modified to suit my needs.
*/

#include <Wire.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_L3GD20.h>
#include <Hybotics_LSM303DLHC_Unified.h>
#include <Hybotics_10DOF_Unified.h>
#include <KalmanFilter.h>

#include <RTClib.h>

#include <BMSerial.h>
#include <RoboClaw.h>

/*
	Additional libraries
*/

#include "Motor_Sensor_Test.h"
#include "Pitches.h"

/********************************************************************/
/*	Global objects													*/
/********************************************************************/

/*
	Initialize our sensors

	We have:
		These are all on a single small board from Adafruit
			http://www.adafruit.com/products/1604
				A BMP180 temperature and pressure sensor
				An L3GD20 Gyroscope
				An LSM303 3-Axis Accelerometer / 3-Axis Magnetometer (compass)

		These are also from Adafruit:
			http://www.adafruit.com/products/1334 (TCS34725 RGB Color sensor)
			http://www.adafruit.com/products/1296 (TMP006 Heat sensor)
			http://www.adafruit.com/products/264 (DS1307 Realtime Clock)

		From other sources:
			GP2Y0A21YK0F IR Ranging sensors (4)
			PING Ultrasonic Ranging sensors (3)
*/

Adafruit_L3GD20 gyroscope;
Hybotics_LSM303DLHC_Accel_Unified accelerometer = Hybotics_LSM303DLHC_Accel_Unified(10002);
Hybotics_LSM303DLHC_Mag_Unified compass = Hybotics_LSM303DLHC_Mag_Unified(10003);
Hybotics_10DOF_Unified imu = Hybotics_10DOF_Unified();

RTC_DS1307 clock;

/********************************************************************/
/*	Initialize global variables										*/
/********************************************************************/

//	Total number of area readings taken, or -1 if data is not valid
int nrAreaScanReadings;

//  These are where the range sensor readings are stored.
int ping[MAX_NUMBER_PING];
float ir[MAX_NUMBER_IR];

bool areaScanValid = false, hasMoved = false;

//	Readings for full area scans
AreaScanReading areaScan[MAX_NUMBER_AREA_READINGS];

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
BMSerial console(SERIAL_CONSOLE_RX_PIN, SERIAL_CONSOLE_TX_PIN);

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
Motor leftMotorM1;

//	Right side motor (M2) - RoboClaw 2x5 Controller #1 (0x80)
Motor rightMotorM2;

//	Front motor (M1) - RoboClaw 2x5 Controller #2 (0x81)
Motor frontMotorM1;

//	Back motor (M2) - RoboClaw 2x5 Controller #2 (0x81)
Motor backMotorM2;

/********************************************************************/
/*	Initialize servos												*/
/********************************************************************/

Servo gripLift, gripWrist, gripGrab, pan, tilt;

/*
	Code starts here
*/

/********************************************************************/
/*	Utility routines 												*/
/********************************************************************/

/*
    Left zero pad a numeric string
*/
String leftZeroPadString (String st, uint8_t nrPlaces) {
	uint8_t i, len;
	String newStr = st;
  
	if (newStr.length() < nrPlaces) {
		len = st.length();
  
		for (i = len; i < nrPlaces; i++) {
			newStr = String("0" + newStr);
		}
	}

	return newStr;
}

/*
    Convert a pulse width in ms to inches
*/
long microsecondsToInches (long microseconds) {
	/*
		According to Parallax's datasheet for the PING))), there are
			73.746 microseconds per inch (i.e. sound travels at 1130 feet per
			second).  This gives the distance travelled by the ping, outbound
			and return, so we divide by 2 to get the distance of the obstacle.

		See: http://www.parallax.com/dl/docs/prod/acc/28015-PING-v1.3.pdf
	*/
	
	return microseconds / 74 / 2;
}

/*
    Convert a pulse width in ms to a distance in cm
*/
long microsecondsToCentimeters (long microseconds) {
	/*
		The speed of sound is 340 m/s or 29 microseconds per centimeter.

		The ping travels out and back, so to find the distance of the
			object we take half of the distance travelled.
	*/

	return microseconds / 29 / 2;
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

/*
	Convert a temperature in Celsius to Fahrenheit
*/
float toFahrenheit (float celsius) {
	return (celsius * 1.8) + 32;
}

/*
    Trim trailing zeros from a numeric string
*/
String trimTrailingZeros (String st) {
	uint8_t newStrLen = 0;
	String newStr = st;

	newStrLen = newStr.length();

	while (newStr.substring(newStrLen - 1) == "0") {
		newStrLen -= 1;
		newStr = newStr.substring(0, newStrLen);
	}

	return newStr;
}

/*
    Display the GP2Y0A21YK0F IR sensor readings (cm)
*/
void displayIR (void) {
	int sensorNr = 0;
  
	console.println(F("IR Sensor readings:"));

	while (sensorNr < MAX_NUMBER_IR) {
		console.print(F("IR #"));
		console.print(sensorNr + 1);
		console.print(F(" range = "));
		console.print(ir[sensorNr]);
		console.println(F(" cm"));

		sensorNr += 1;
	}

	console.println();
}

/*
	Display the readings from the PING Ultrasonic sensors
*/
void displayPING (void) {
	int sensorNr = 0;
  
	console.println(F("PING Ultrasonic Sensor readings:"));
  
	//	Display PING sensor readings (cm)
	while (sensorNr < MAX_NUMBER_PING) {
		console.print(F("Ping #"));
		console.print(sensorNr + 1);
		console.print(F(" range = "));
		console.print(ping[sensorNr]);
		console.println(F(" cm"));

		sensorNr += 1;
	}
 
	console.println();
}

/*
	Display the readings from the IMU (Accelerometer, Magnetometer [Compass], Gyroscope,
		and Orientation (if valid)
*/
void displayIMUReadings (sensors_event_t *accelEvent, sensors_event_t *compassEvent, sensors_vec_t *orientation, bool pitchRollValid, bool headingValid, bool temperatureValid, float celsius, float fahrenheit, int gyroX, int gyroY, int gyroZ) {
	//	LMS303DLHC Accelerometer readings
	console.println(F("Accelerometer Readings: X = "));
	console.print(accelEvent->acceleration.x);
	console.print(F(", Y = "));
	console.print(accelEvent->acceleration.y);
	console.print(F(", Z = "));
	console.println(accelEvent->acceleration.z);
	console.println();

	//	LMS303DLHC Magnetometer (Compass) readings
	console.println(F("Magnetometer (Compass) Readings: X = "));
	console.print(compassEvent->magnetic.x);
	console.print(F(", Y = "));
	console.print(compassEvent->magnetic.y);
	console.print(F(", Z = "));
	console.println(compassEvent->magnetic.z);
	console.println();

	//	L3DG20 Gyroscope readings
	console.println(F("Gyroscope Readings: Gyro: X = "));
	console.print(gyroX);
	console.print(F(", Y = "));
	console.print(gyroY);
	console.print(F(", Z = "));
	console.println(gyroZ);
	console.println();

	//	BMP180 Temperature readings
	if (temperatureValid) {
		console.print(F("Room Temperature = "));
		console.print(fahrenheit);
		console.print(F(" F, "));
		console.print(celsius);
		console.println(F(" C."));
		console.println();
	}

	if (pitchRollValid || headingValid) {
		console.println(F("Orientation Readings:"));
	}
	
	//	Orientation readings - Pitch, Roll, and Heading
	if (pitchRollValid) {
		console.print(F("Roll: "));
		console.print(orientation->roll);
		console.print(F("; "));
		console.print(F("Pitch: "));
		console.print(orientation->pitch);
	}

	if (headingValid) {
		if (pitchRollValid) {
			console.print(F(", "));
		}

		console.print(F("Heading: "));
		console.println(orientation->heading);
	}

	console.println();
}

/*
	Display the data for a given motor
*/
void displayMotor (Motor *motor, String name) {
	console.print(motor->name);
	console.println(F(" Motor:"));

	//	Using Packet Serial
	console.print(F("Encoder is valid: "));

	if (motor->encoderValid) {
		console.print(F("Yes, Status: "));
		console.print(motor->encoderStatus);
		console.print(F(", Value: "));
		console.println(motor->encoder);
	} else {
		console.println(F("No"));
	}

	console.print(F("Speed is valid: "));

	if (motor->speedValid) {
		console.print(F("Yes, Status: "));
		console.print(motor->speedStatus);
		console.print(F(", Speed: "));
		console.println(motor->speed);
	} else {
		console.println(F("No"));
	}

	console.print(F("Moving "));

	if (motor->forward) {
		console.print(F("Forward"));
	} else {
		console.print(F("Reverse"));
	}

	console.print(F("Distance is valid: "));

	if (motor->distanceValid) {
		console.print(F("Yes, Distance: "));
		console.println(motor->distance);
	} else {
		console.println(F("No"));
	}
}

/*
	Display data from the RoboClaw 2x5 motor controller
*/
void displayRoboClawData (uint8_t address, Motor *leftMotorM1, Motor *rightMotorM2) {
	char version[32];

	roboClaw.ReadVersion(address, version);

	console.print(F("RoboClaw 2x5 status (version "));
	console.print(version);
	console.print(F("): "));
	console.println();

    if (leftMotorM1->encoderValid) {
		console.print(F("Left Motor Encoder = "));
		console.print(leftMotorM1->encoder, DEC);
		console.print(F(", Status =  "));
		console.print(leftMotorM1->encoderStatus, HEX);
		console.println();
	}

	if (leftMotorM1->speedValid) {
		console.print(F("Left Motor Speed = "));
		console.print(leftMotorM1->speed, DEC);
		console.println();
	}

	if (rightMotorM2->encoderValid) {
		console.print(F("Right Motor Encoder = "));
		console.print(rightMotorM2->encoder, DEC);
		console.print(F(", Status = "));
		console.print(rightMotorM2->encoderStatus, HEX);
		console.println();
	}

	if (rightMotorM2->speedValid) {
		console.print(F("Right Motor Speed = "));
		console.print(rightMotorM2->speed, DEC);
		console.println();
	}
	
	console.println();
}

/*
	Display the data in a Servo struct
*/
void displayServo (Servo *servo, String servoName) {
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
	Function to read a value from a GP2Y0A21YK0F infrared distance sensor and return a
		distance value in centimeters.

	This sensor should be used with a refresh rate of 36ms or greater.

	Javier Valencia 2008

	float readIR(byte pin)

	It can return -1 if something has gone wrong.

	TODO: Make several readings over a time period, and average them
		for the final reading.

	NOTE: This code is for the older Sharp GP2D12 IR sensor, and will no
		doubt have to be adjusted to work correctly with the newer sensor.
*/
float readIR (byte sensorNr) {
	byte pin = sensorNr + IR_PIN_BASE;
	int tmp;

	tmp = analogRead(pin);

	if (tmp < 3) {
		return -1;                                  // Invalid value
	} else {
		return (6787.0 /((float)tmp - 3.0)) - 4.0;  // Distance in cm
	}
}

/*
	Ping))) Sensor 

	This routine reads a PING))) ultrasonic rangefinder and returns the
		distance to the closest object in range. To do this, it sends a pulse
		to the sensor to initiate a reading, then listens for a pulse
		to return.  The length of the returning pulse is proportional to
		the distance of the object from the sensor.

	The circuit:
		* +V connection of the PING))) attached to +5V
		* GND connection of the PING))) attached to ground
		* SIG connection of the PING))) attached to digital pin 7

	http://www.arduino.cc/en/Tutorial/Ping

	Created 3 Nov 2008
		by David A. Mellis

	Modified 30-Aug-2011
		by Tom Igoe

	Modified 09-Aug-2013
		by Dale Weber

		Set units = true for cm, and false for inches
*/
int readPING (byte sensorNr, bool units=true) {
	byte pin = sensorNr + PING_PIN_BASE;
	long duration;
	int result;

	/*
		The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
		Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
	*/
	pinMode(pin, OUTPUT);
	digitalWrite(pin, LOW);
	delayMicroseconds(2);
	digitalWrite(pin, HIGH);
	delayMicroseconds(5);
	digitalWrite(pin, LOW);

	/*
		The same pin is used to read the signal from the PING))): a HIGH
		pulse whose duration is the time (in microseconds) from the sending
		of the ping to the reception of its echo off of an object.
	*/
	pinMode(pin, INPUT);
	duration = pulseIn(pin, HIGH);

	//  Convert the duration into a distance
	if (units) {
		//	Return result in cm
		result = microsecondsToCentimeters(duration);
	} else {
		//  Return result in inches.
		result = microsecondsToInches(duration);
	}
 
	delay(100);
  
	return result;
}

/*
	Read current data from the RoboClaw 2x5 Motor Controller
*/
uint16_t readRoboClawData (uint8_t address, Motor *leftM1, Motor *rightM2) {
	bool valid;
	uint8_t status;

	//	Error control
	uint16_t errorStatus;
	String errorMsg;

	errorStatus = 0;

	console.println(F("Reading Left Motor Encoder.."));

	leftM1->encoder = roboClaw.ReadEncM1(address, &status, &valid);
	leftM1->encoderStatus = status;
	leftM1->encoderValid = valid;

	console.println(F("Reading Left Motor Speed.."));

	leftM1->speed = roboClaw.ReadSpeedM1(address, &status, &valid);
	leftM1->speedStatus = status;
	leftM1->speedValid = valid;

	console.println(F("Reading Right Motor Encoder.."));

	rightM2->encoder = roboClaw.ReadEncM2(address, &status, &valid);
	rightM2->encoderStatus = status;
	rightM2->encoderValid = valid;

	console.println(F("Reading Right Motor Speed.."));

	rightM2->speed = roboClaw.ReadSpeedM2(address, &status, &valid);
	rightM2->speedStatus = status;
	rightM2->speedValid = valid;

	return errorStatus;
}

/********************************************************************/
/*	Miscellaneous routines 											*/
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
	Wait for a bit to allow time to read the Console Serial Monitor log
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

/********************************************************************/
/*	Lynxmotion SSC-32 Servo Controller routines 					*/
/********************************************************************/

/*
	Convert a servo position in degrees to a pulse width in uS
*/
uint16_t degreesToPulseUS (Servo *servo, int degrees) {
	uint16_t result, pulseAdjustmentUS, servoPulseUS;

	//	Error control
	uint16_t errorStatus = 0;
	String errorMsg;

	pulseAdjustmentUS = servoPulseAdjustmentUS(servo);

	if (servo->maxDegrees == 90) {
		servoPulseUS = SERVO_CENTER_MS + (degrees * 11) + servo->offset;

		if (degrees < 0) {
			servoPulseUS -= pulseAdjustmentUS;
		} else {
			servoPulseUS += pulseAdjustmentUS;
		}
	} else if (servo->maxDegrees == 180) {
		servoPulseUS = (degrees * 11) + servo->offset + pulseAdjustmentUS;
	} else {
		errorStatus = 4002;
		errorMsg = String(F("(degreesToPulseUS) Servo maximum degrees is invalid"));
	}

	if (errorStatus != 0) {
		result = errorStatus;
	} else {
		result = servoPulseUS;
	}

	return result;
}

void displayDistanceReading (DistanceReading *reading) {
	if (reading->dist == Closest) {
		console.print(F("Closest"));
	} else {
		console.print(F("Farthest"));
	}

	console.println(F(" object:"));

	console.print(F("     IR: Reading = "));
	console.print(reading->irDistance);
	console.print(F(" cm, Position = "));
	console.print(reading->irPositionDeg);
	console.println(F(" degrees"));

	console.print(F("     PING: Reading = "));
	console.print(reading->pingDistance);
	console.print(F(" cm, Position = "));
	console.print(reading->pingPositionDeg);
	console.println(F(" degrees"));

	console.println();
}

DistanceReading getClosestObject () {
	uint8_t irNr = 0, pingNr = 0, readingNr;

	DistanceReading distReading;

	console.println(F("Finding the closest object.."));

	//	Find the closest object
	for (readingNr = 0; readingNr < nrAreaScanReadings; readingNr++) {
		//	Check for the closest object
		if (areaScan[readingNr].ping < areaScan[pingNr].ping) {
			pingNr = readingNr;
		}

		if (areaScan[readingNr].ir <  areaScan[irNr].ir) {
			irNr = readingNr;
		}
	}

	distReading.dist = Closest;

	distReading.irNr = irNr;
	distReading.irDistance = areaScan[irNr].ir;
	distReading.irPositionDeg = areaScan[irNr].positionDeg;

	distReading.pingNr = pingNr;
	distReading.pingDistance = areaScan[pingNr].ping;
	distReading.pingPositionDeg = areaScan[pingNr].positionDeg;

	return distReading;
}

DistanceReading getFarthestObject () {
	uint8_t irNr = 0, pingNr = 0, readingNr;

	DistanceReading distReading;

	console.println(F("Finding the farthest object.."));

	//	Find the farthest object
	for (readingNr = 0; readingNr < nrAreaScanReadings; readingNr++) {
		//	Check for the farthest object
		if (areaScan[readingNr].ping > areaScan[pingNr].ping) {
			pingNr = readingNr;
		}

		if (areaScan[readingNr].ir > areaScan[irNr].ir) {
			irNr = readingNr;
		}
	}

	distReading.dist = Farthest;

	distReading.irNr = irNr;
	distReading.irDistance = areaScan[irNr].ir;
	distReading.irPositionDeg = areaScan[irNr].positionDeg;

	distReading.pingNr = pingNr;
	distReading.pingDistance = areaScan[pingNr].ping;
	distReading.pingPositionDeg = areaScan[pingNr].positionDeg;

	return distReading;
}

/*
	Convert a servo pulse width in ms to an angle in degrees (0 to 180)
*/
uint16_t pulseToDegrees (Servo *servo, uint16_t pulseWidthUS) {
	uint16_t degrees = (pulseWidthUS - SERVO_CENTER_MS) / 11;
	uint16_t result;

	//	Error control
	uint16_t errorStatus = 0;
	String errorMsg;

/*
	console.print(F("(pulseToDegrees) degrees = "));
	console.print(degrees);
	console.print(F(", pulseWidthUS = "));
	console.println(pulseWidthUS);
*/
	if ((pulseWidthUS < servo->minPulse) || (pulseWidthUS > servo->maxPulse)) {
		errorStatus = 3001;
		errorMsg = String(F("Servo pulse is out of range"));
	}

	if (errorStatus != 0) {
		result = 999;
		processError(errorStatus, "pulseToDegrees", errorMsg);
	} else {
		result = degrees;
	}

	return result;
}

/*
	Return the next valid pulse (in micro seconds) from a range (in degrees), or 0 (zero) if there are
		no more valid pulses within the range.
*/
uint16_t nextValidPositionDegrees (Servo *servo, int positionDeg, int startDeg, int stopDeg, int incrDeg) {
	uint16_t adjustmentUS, result;
	int currPositionDeg = SERVO_INVALID_DEGREES;
	uint16_t servoPulseUS;

	//	Error control
	uint16_t errorStatus = 0;
	String errorMsg;

	adjustmentUS = servoPulseAdjustmentUS(servo);

	if (errorStatus != 0) {
		processError(errorStatus, "nextValidPositionDegrees", "Unhandled Problem");
	} else {
		currPositionDeg = positionDeg;
		servoPulseUS = degreesToPulseUS(servo, currPositionDeg);

		if ((servoPulseUS < servo->minPulse) && (servoPulseUS < servo->maxPulse)) {
			//	Find the next valid pulse within the current range
			while ((servoPulseUS < servo->minPulse) && (currPositionDeg <= stopDeg)) {
				currPositionDeg = currPositionDeg + incrDeg;
				servoPulseUS = degreesToPulseUS(servo, currPositionDeg);
			}
		} else if (servoPulseUS > servo->maxPulse) {
			//	There aren't any more valid pulses in the current range - NOT FATAL
			errorStatus = 3000;
			currPositionDeg = SERVO_INVALID_DEGREES;
		}
	}

	servo->error = errorStatus;

	if (errorStatus != 0) {
		result = errorStatus;
	} else {
		//	Make sure the return value is in the range 0 to 180 degrees
		if (currPositionDeg < 0) {
			currPositionDeg += 90;
		}

		result = currPositionDeg;
	}

	return result;
}

/*
    Move a servo by pulse width in ms (500ms - 2500ms) - Modified to use HardwareSerial2()
*/
uint16_t moveServoPw (Servo *servo, int servoPosition, bool term, int moveSpeed = 0, int moveTime = 0) {
	//	Error control
	uint16_t errorStatus = 0;
	String errorMsg;
  
	if ((servoPosition >= servo->minPulse) && (servoPosition <= servo->maxPulse)) {
		ssc32.print(F("#"));
		ssc32.print(servo->pin);
		ssc32.print(F(" P"));
		ssc32.print(servoPosition + servo->offset);

		servo->msPulse = servoPosition;
		servo->angle = (servoPosition - SERVO_CENTER_MS) / 11;

		if (servo->maxDegrees == 90) {
			servo->angle += 90;
		}
	} else if ((servoPosition < servo->minPulse) || (servoPosition > servo->maxPulse)) {
		errorStatus = 3001;
		errorMsg = String(F("Servo pulse is out of range"));
	}
 
	if (errorStatus != 0) {
		processError(errorStatus, F("moveServoPw"), errorMsg);
	} else {
		//  Add servo move speed
		if (moveSpeed != 0) {
			ssc32.print(F(" S"));
			ssc32.print(moveSpeed);
		}
    
		//  Terminate the command
		if (term) {
			if (moveTime != 0) {
				ssc32.print(F(" T"));
				ssc32.print(moveTime);
			}

			ssc32.println();
		}
  	}

	servo->error = errorStatus;

  	return errorStatus;
}

/*
    Move a servo by degrees (-90 to 90) or (0 - 180) - Modified to use BMSerial
*/
uint16_t moveServoDegrees (Servo *servo, int servoDegrees, bool term, int moveSpeed = 0, int moveTime = 0) {
	int servoPulse;
	int pulseAdjustment;

	//	Error control
	uint16_t errorStatus = 0;
	String errorMsg;
  
	//  Convert degrees to ms for the servo move
	if (servo->maxDegrees == 90) {
		if (servoDegrees < 0) {
			pulseAdjustment = -10;
		} else {
			pulseAdjustment = 10;
		}
	} else if (servo->maxDegrees == 180) {
		pulseAdjustment = 500;
	} else {
		errorStatus = 3002;
		errorMsg = String(F("Servo position (degrees) is invalid"));
	}

	if (errorStatus != 0) {
		processError(errorStatus, F("moveServoDegrees #1"), errorMsg);
	} else {
		servoPulse = degreesToPulseUS(servo, servoDegrees);

		if (errorStatus != 0) {
			processError(errorStatus, F("moveServoDegrees #2, degreesToPulseUS"), F("Unhandled Problem"));
		} else {
			if ((servoPulse >= servo->minPulse) && (servoPulse <= servo->maxPulse)) {
				errorStatus = moveServoPw(servo, servoPulse, true);

				if (errorStatus != 0) {
					processError(errorStatus, F("moveServoDegrees #3, moveServoPw"), errorMsg);
				}
			} else {
				errorStatus = 3001;
				errorMsg = String(F("Servo pulse is out of range"));
				processError(errorStatus, F("moveServoDegrees #4"), errorMsg);
			}
		}
	}
/*
	console.print(F("(moveServoDegrees #1) servoDegrees = "));
	console.print(servoDegrees);
	console.print(F(",  servoPulse = "));
	console.print(servoPulse);
	console.print(F(", servo->minPulse = "));
	console.print(servo->minPulse);
	console.print(F(", servo->maxPulse = "));
	console.println(servo->maxPulse);
*/
	if (errorStatus != 0) {
		processError(errorStatus, F("moveServoDegrees"), errorMsg);
	}

	servo->error = errorStatus;

	return errorStatus;
}

/*
	Scan the area for objects
*/
uint16_t scanArea (Servo *servo, int startDeg, int stopDeg, int incrDeg) {
	int readingNr, positionDeg;
	int nrReadings, totalRangeDeg = 0;
	uint16_t servoPulseUS;

	//	Error control
	uint16_t errorStatus = 0;
	String errorMsg;

/*
	console.println(F("Checking parameters.."));
*/
	//	Check the parameters
	if (startDeg > stopDeg) {
		//	Start can't be greater than stop
		errorStatus = 4001;
		errorMsg = String(F("Start position is > ending position"));
	} else if (((SERVO_MAX_DEGREES == 90) && ((startDeg < -90) || (stopDeg > 90))) || ((SERVO_MAX_DEGREES == 180) && ((startDeg < 0) || (stopDeg > 180)))) {
		//	One or more parameters is outside of the valid range
		errorStatus = 4002;
		errorMsg = String(F("Servo maximum degrees is invalid"));
	} else {
		//	Calculate the total range, in degrees
		totalRangeDeg = abs(stopDeg - startDeg);

		//	Calculate the number of readings we need room for
		nrReadings = abs(totalRangeDeg / incrDeg);

		//	More error checking
		if (nrReadings > MAX_NUMBER_AREA_READINGS) {
			//	We can't take this many readings
			errorStatus = 4003;
			errorMsg = String(F("Number of readings is invalid"));
		} else if (totalRangeDeg > 180) {
			//	Servos can only move up to 180 degrees
			errorStatus = 4004;
			errorMsg =  String(F("Requested servo move is invalid"));
		} else if (incrDeg > (totalRangeDeg / nrReadings)) {
			//	Increment is too large for our range
			errorStatus = 4005;
			errorMsg = String(F("Move increment is too large for movement range"));
		} else {
			//	Continue normal processing
			readingNr = 0;
			positionDeg = startDeg;
/*
			console.print(F("(scanArea #1) startDeg = "));
			console.print(startDeg);
			console.print(F(", stopDeg = "));
			console.print(stopDeg);
			console.print(F(", incrDeg = "));
			console.print(incrDeg);
			console.print(F(", positionDeg = "));
			console.print(positionDeg);
			console.print(F(", readingNr = "));
			console.println(readingNr);
*/
			servoPulseUS = degreesToPulseUS(servo, positionDeg);
/*
			console.print(F("(scanArea #2) servoPulseUS = "));
			console.println(servoPulseUS);
*/
			if (errorStatus != 0) {
				processError(errorStatus, "scanArea #1", errorMsg);			
			} else {
/*
				console.print(F("(scanArea #7) servoPulseUS = "));
				console.println(servoPulseUS);
*/
				//	Get the lowest position (in degrees) within the servo's valid pulse width range
				if (servoPulseUS < servo->minPulse) {
					positionDeg = nextValidPositionDegrees(servo, positionDeg, startDeg, stopDeg, incrDeg);
/*
					console.print(F("(scanArea #8) positionDeg = "));
					console.println(positionDeg);
					console.print(F(", servoPulseUS = "));
					console.print(servoPulseUS);
*/
					servoPulseUS = degreesToPulseUS(servo, positionDeg);
/*
					console.print(F("scanArea #9) servoPulseUS = "));
					console.println(servoPulseUS);
*/
				}

				if (errorStatus != 0) {
					processError(errorStatus, F("scanArea #2"), errorMsg);
				} else {
					while ((positionDeg <= stopDeg) && (servoPulseUS <= servo->maxPulse)) {
/*
						console.print(F("(scanArea #10) positionDeg = "));
						console.print(positionDeg);
						console.print(F(", servoPulseUS = "));
						console.print(servoPulseUS);
						console.print(F(", readingNr = "));
						console.println(readingNr);
*/
						errorStatus = moveServoDegrees(servo, positionDeg, true, 1000, 0);

						if (errorStatus != 0) {
							processError(errorStatus, F("scanArea #3"), F("There was a problem moving the pan servo"));
							break;
						} else {
							delay(SCAN_SENSOR_DELAY);
/*
							console.print(F("(scanArea #11) positionDeg = "));
							console.println(positionDeg);
*/
							//	Take a reading from each pan/tilt sensor in cm
							areaScan[readingNr].ping = readPING(PING_FRONT_CENTER, true);
							areaScan[readingNr].ir = readIR(IR_FRONT_CENTER);
							areaScan[readingNr].positionDeg = positionDeg;

							readingNr += 1;
							positionDeg += incrDeg;
							servoPulseUS = degreesToPulseUS(servo, positionDeg);
/*
							console.print(F("(scanArea #11) servoPulseUS = "));
							console.println(servoPulseUS);
*/
							//	Wait a bit to allow time between sensor readings
							delay(SCAN_SENSOR_DELAY);
						}
					}
				}
			}
		}
	}

	if (errorStatus != 0) {
		nrAreaScanReadings = -1;
		areaScanValid = false;
		servo->error = errorStatus;
	} else {
		//	Set the number of readings taken
		nrAreaScanReadings = readingNr;
		areaScanValid = true;
	}

	servo->error = errorStatus;

	return errorStatus;
}

/*
	Return the servo pulse adjustment, in uS, for a given servo
*/
uint16_t servoPulseAdjustmentUS (Servo *servo)  {
	uint16_t result, adjustmentUS;

	//	Error control
	uint16_t errorStatus = 0;
	String errorMsg;

	if (servo->maxDegrees == 90) {
		adjustmentUS = 10;
	} else if (servo->maxDegrees == 180) {
		adjustmentUS = 500;
	} else {
		//	servo->maxDegrees is invalid
		errorStatus = 3003;
		errorMsg = String(F("maxDegrees is invalid for this servo"));
	}

	if (errorStatus != 0) {
		processError(errorStatus, "servoPulseAdjustmentMS", errorMsg);
		result = errorStatus;
	} else {
		result = adjustmentUS;
	}

	servo->error = errorStatus;

	return result;
}

/********************************************************************/
/*	RoboClaw 2x5 Motor Controller routines 							*/
/********************************************************************/

/*
	Set motor speeds

	The left and right motor speeds may be different.
*/
void setMotors (uint8_t address, short leftSpd, short rightSpd, Motor *leftM1, Motor *rightM2) {
	bool leftDir, rightDir;

	console.println(F("Setting motor speeds.."));

	leftDir = setMotorSpeed(address, leftSpd, leftM1);
	rightDir = setMotorSpeed(address, rightSpd, rightM2);

	updateMotors(address, leftDir, rightDir, leftM1, rightM2);
}

bool setMotorSpeed (uint8_t address, short spd, Motor *motor) {
	bool direction;

	if (motor->location == Left) {
		//	Set left motor speed
		if (spd >= 0) {
			roboClaw.ForwardM1(address, spd);
			direction = true;
		} else {
			roboClaw.BackwardM1(address, -spd);
			direction = false;
		}
	} else {
		//	Set right motor speed
		if (spd >= 0) {
			roboClaw.ForwardM2(address, spd);
			direction = true;
		} else {
			roboClaw.BackwardM2(address, -spd);
			direction = false;
		}
	}

	return direction;
}

/*
	Update motor data
*/
void updateMotors (uint8_t address, bool leftDir, bool rightDir, Motor *leftM1, Motor *rightM2) {
	bool direction, valid;
	uint8_t speedStatus;
	uint32_t speed;

	console.println(F("Updating motors.."));

	//	Update left motor data
	speed = roboClaw.ReadSpeedM1(address, &speedStatus, &valid);

	leftM1->speed = speed;
	leftM1->speedStatus = speedStatus;
	leftM1->speedValid = valid;
	leftM1->forward = direction;

	//	Update right motor data
	speed = roboClaw.ReadSpeedM2(address, &speedStatus, &valid);

	rightM2->speed = speed;
	rightM2->speedStatus = speedStatus;
	rightM2->speedValid = valid;
	rightM2->forward = direction;
}

/********************************************************************/
/*	Initialization routines 										*/
/********************************************************************/

/*
	Set the Pan/Tilt to Home Position
*/
void initPanTilt (Servo *pan, Servo *tilt) {
	console.println(F("Initializing Pan/Tilt"));

	//  Put the front pan/tilt at home position
	moveServoPw(pan, SERVO_CENTER_MS, false);
	moveServoPw(tilt, SERVO_CENTER_MS, true);
//	moveServoDegrees(pan, 0, false);
//	moveServoDegrees(tilt, 0, true);
}

/*
	Initialize the RoboClaw 2x5 motor controller
*/
void initRoboClaw (uint8_t address, uint16_t bps, Motor *leftM1, Motor *rightM2) {
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
	leftM1->name = ROBOCLAW_MOTOR_LEFT_NAME;
	leftM1->location = Left;
	leftM1->encoder = 0;
	leftM1->encoderStatus = 0;
	leftM1->encoderValid = false;
	leftM1->speed = 0;
	leftM1->speedStatus = 0;
	leftM1->speedValid = false;
	leftM1->forward = true;
	leftM1->distance = 0;
	leftM1->distanceValid = false;		    

	//	For Packet Serial modes
	rightM2->name = ROBOCLAW_MOTOR_RIGHT_NAME;
	rightM2->location = Right;
	rightM2->encoder = 0;
	rightM2->encoderStatus = 0;
	rightM2->encoderValid = false;
	rightM2->speed = 0;
	rightM2->speedStatus = 0;
	rightM2->speedValid = false;
	rightM2->forward = true;
	rightM2->distance = 0;
	rightM2->distanceValid = false;		    
}

/*
	Initialize sensors
*/
void initSensors (void) {
	console.println(F("Initializing Sensors.."));

	//	Initialize the accelerometer
	console.println(F("     LSM303DLHC Accelerometer.."));

	if (! accelerometer.begin()) {
		/* There was a problem detecting the LSM303DLHC ... check your connections */
		console.println(F("Ooops, no LSM303DLHC detected ... Check your wiring!"));
		while(1);
	}

	console.println(F("     LSM303DLHC Magnetometer (Compass).."));

	//	Initialize the magnetometer (compass) sensor
	if (! compass.begin()) {
		/*	There was a problem detecting the LSM303DLHC ... check your connections */
		console.println(F("Ooops, no LSM303DLHC detected ... Check your wiring!"));
		while(1);
	}

	console.println(F("     L3GD20 Gyroscope.."));

	//	Initialize and warn if we couldn't detect the gyroscope chip
	if (! gyroscope.begin(gyroscope.L3DS20_RANGE_250DPS)) {
		console.println(F("Oops ... unable to initialize the L3GD20. Check your wiring!"));
		while (1);
	}

	console.println(F("     10 DOF Inertial Measurement Unit.."));

	imu.begin();

	console.println(F("     DS1307 Real Time Clock.."));

	//	Check to be sure the RTC is running
//	if (! clock.isrunning()) {
//		console.println(F("The Real Time Clock is NOT running!"));
//		while(1);
//	}
}

/*
	Initialize servos to defaults
*/
void initServos (Servo *lift, Servo *wrist, Servo *grab, Servo *pan, Servo *tilt) {
	lift->pin = SERVO_GRIP_LIFT_PIN;
	lift->name = SERVO_GRIP_LIFT_NAME;
	lift->offset = SERVO_GRIP_LIFT_OFFSET;
	lift->homePos = SERVO_GRIP_LIFT_HOME;
	lift->msPulse = 0;
	lift->angle = 0;
	lift->minPulse = SERVO_GRIP_LIFT_MIN;
	lift->maxPulse = SERVO_GRIP_LIFT_MAX;
	lift->maxDegrees = SERVO_MAX_DEGREES;
	lift->error = 0;

	wrist->pin = SERVO_GRIP_WRIST_PIN;
	wrist->name = SERVO_GRIP_WRIST_NAME;
	wrist->offset = SERVO_GRIP_WRIST_OFFSET;
	wrist->homePos = SERVO_GRIP_WRIST_HOME;
	wrist->msPulse = 0;
	wrist->angle = 0;
	wrist->minPulse = SERVO_GRIP_WRIST_MIN;
	wrist->maxPulse = SERVO_GRIP_WRIST_MAX;
	wrist->maxDegrees = SERVO_MAX_DEGREES;
	wrist->error = 0;

	grab->pin = SERVO_GRIP_GRAB_PIN;
	grab->name = SERVO_GRIP_GRAB_NAME;
	grab->offset = SERVO_GRIP_GRAB_OFFSET;
	grab->homePos = SERVO_GRIP_GRAB_HOME;
	grab->msPulse = 0;
	grab->angle = 0;
	grab->minPulse = SERVO_GRIP_GRAB_MIN;
	grab->maxPulse = SERVO_GRIP_GRAB_MAX;
	grab->maxDegrees = SERVO_MAX_DEGREES;
	grab->error = 0;

	pan->pin = SERVO_PAN_PIN;
	pan->name = SERVO_PAN_NAME;
	pan->offset = SERVO_PAN_OFFSET;
	pan->homePos = SERVO_PAN_HOME;
	pan->msPulse = 0;
	pan->angle = 0;
	pan->minPulse = SERVO_PAN_LEFT_MIN;
	pan->maxPulse = SERVO_PAN_RIGHT_MAX;
	pan->maxDegrees = SERVO_MAX_DEGREES;
	pan->error = 0;

	tilt->pin = SERVO_TILT_PIN;
	tilt->name = SERVO_TILT_NAME;
	tilt->offset = SERVO_TILT_OFFSET;
	tilt->homePos = SERVO_TILT_HOME;
	tilt->msPulse = 0;
	tilt->angle = 0;
	tilt->minPulse = SERVO_TILT_DOWN_MIN;
	tilt->maxPulse = SERVO_TILT_UP_MAX;
	tilt->maxDegrees = SERVO_MAX_DEGREES;
	tilt->error = 0;
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
	console.print(F("W.A.L.T.E.R. 2.0 Motor and Sensor Tester, version "));
	console.print(BUILD_VERSION);
	console.print(F(" on "));
	console.print(BUILD_DATE);
	console.print(F(" for the "));
	console.print(BUILD_BOARD);
	console.println(F(" board"));

	console.println();
	console.println(F("Initializing Serial Ports.."));

	//	Initialize the SSC-32 servo controller port
	ssc32.begin(115200);

	//	Initialize the XBee communication port (BMSerial)
	xbee.begin(115200);

	console.println(F("Initializing Digital Pins.."));

	//  Initialize the LED pin as an output.
	pinMode(HEARTBEAT_LED, OUTPUT);
	digitalWrite(HEARTBEAT_LED, LOW);

	//	Initialize all sensors
//	initSensors();

	//	Initialize all servos
	initServos(&gripLift, &gripWrist, &gripGrab, &pan, &tilt);

	//	Set the Pan/Tilt to home position
	initPanTilt(&pan, &tilt);


	//	Scan the entire 180 degree range and take sensor readings every 10 degrees
	console.println(F("Doing initial area scan.."));

	errorStatus = scanArea(&pan, -90, 90, 10);

	if (errorStatus != 0) {
		processError(errorStatus, F("Setup, scanArea"), F("Unhandled problem"));
	} else {
		//	Put the pan/tilt back at home position, facing forward
		initPanTilt(&pan, &tilt);
	}

	wait(WAIT_DELAY_SECONDS);

	console.println();

	/*
		Do we have any RoboClaw motor controllers to setup?
	*/
	if (ROBOCLAW_CONTROLLERS > 0) {
		//	Initialize the first RoboClaw 2x5 motor controller at address 0x80
		initRoboClaw(roboClawAddress1, 38400, &leftMotorM1, &rightMotorM2);

		//	Start the motors, both forward.
		console.println(F("Both motors forward"));
		setMotors(roboClawAddress1, 100, 100, &leftMotorM1, &rightMotorM2);
		delay(5000);

		//	Stop the motors!
		setMotors(roboClawAddress1, 0, 0, &leftMotorM1, &rightMotorM2);
		delay(2000);

		//	Left motor foward, right motor reverse
		console.println(F("Left motor forward, right motor reverse"));
		setMotors(roboClawAddress1, 100, -100, &leftMotorM1, &rightMotorM2);
		delay(5000);

		//	Stop the motors!
		setMotors(roboClawAddress1, 0, 0, &leftMotorM1, &rightMotorM2);
		delay(2000);

		//	Left motor reverse, right motor forward
		console.println(F("Left motor reverse, right motor forward"));
		setMotors(roboClawAddress1, -100, 100, &leftMotorM1, &rightMotorM2);
		delay(5000);

		//	Stop the motors!
		setMotors(roboClawAddress1, 0, 0, &leftMotorM1, &rightMotorM2);
		delay(2000);

		//	Both motors reverse
		console.println(F("Both motors reverse"));
		setMotors(roboClawAddress1, -100, -100, &leftMotorM1, &rightMotorM2);
		delay(5000);

		//	Stop the motors!
		setMotors(roboClawAddress1, 0, 0, &leftMotorM1, &rightMotorM2);
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

	//	IMU variables
	float celsius, fahrenheit, altitude;
	float accelX, accelY, accelZ;
	float compassX, compassY, compassZ;
	float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;

	int gyroX = 0, gyroY = 0, gyroZ = 0;

	sensors_event_t accelEvent, compassEvent, tempEvent;
	sensors_vec_t orientation;

	DistanceReading closestObject, farthestObject;

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

		errorStatus = readRoboClawData(roboClawAddress1, &leftMotorM1, &rightMotorM2);

		if (errorStatus != 0) {
			processError(errorStatus, "readRoboClawData", "Unhandled problem");
		} else {
			displayRoboClawData(roboClawAddress1, &leftMotorM1, &rightMotorM2);
		}
	}

	/*
		Put accelerometer and Gyro reactive behaviors HERE
	*/

//	console.println(F("Getting Distance Sensor readings.."));

	//	Get readings from all the GP2Y0A21YK0F Analog IR range sensors, if any, and store them
	if (MAX_NUMBER_IR > 0) {
		for (analogPin = 0; analogPin < MAX_NUMBER_IR; analogPin++) { 
			ir[analogPin] = readIR(analogPin);
		}

		displayIR();
	}

	//	Get readings from all the Parallax PING Ultrasonic range sensors, if any, and store them
	if (MAX_NUMBER_PING > 0) {
		for (digitalPin = 0; digitalPin < MAX_NUMBER_PING; digitalPin++) {
			ping[digitalPin] = readPING(digitalPin);
		}

		displayPING();
	}

	if (ROBOCLAW_CONTROLLERS > 0) {
		/*
			Read the RoboClaw 2x5 motor data
		*/
		console.println(F("Reading RoboClaw.."));

		errorStatus = readRoboClawData(roboClawAddress1, &leftMotorM1, &rightMotorM2);

		if (errorStatus != 0) {
			processError(errorStatus, "readRoboClawData", "Unhandled problem");
		} else {
			displayRoboClawData(roboClawAddress1, &leftMotorM1, &rightMotorM2);
		}
	}

	/*
		Put distance related reactive behaviors HERE
	*/

	//	Find the closest and farthest objects
	if (areaScanValid) {
		closestObject = getClosestObject();
		farthestObject = getFarthestObject();

		displayDistanceReading(&closestObject);
		displayDistanceReading(&farthestObject);
	} else {
		console.println(F("Area scan is not valid.."));
		console.println();
	}

	console.println();

	//	Count the minutes
	if (currentMinute != lastMinute) {
		minuteCount += 1;
		lastMinute = currentMinute;
	}

//	displayIMUReadings (&accelEvent, &compassEvent, &orientation, pitchRollValid, headingValid, tempEvent.pressure, celsius, fahrenheit, gyroX, gyroY, gyroZ);

	wait(WAIT_DELAY_SECONDS);

	console.println();
}
