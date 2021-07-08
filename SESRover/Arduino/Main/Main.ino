/*
	SES Rover: Main code
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
	Program:		SES Rover, Main navigation and reactive behaviors sketch
	Date:			06-Mar-2014
	Version:		0.1.0 ALPHA Lynxmotion's BotBoarduino and SSC-32

	Platform:		Lynxmotion's BotBoarduino (Arduino),
						with Lynxmotion's SSC-32 Servo Controller,
						and a 3DOF (Raise/Lower, Wrist Rotate, Open/Close) Little Grip gripper

	Purpose:		To experiment with various sensor configurations, tracking objects (heat or
						color based), course following, manipulation of the environment, and to
						test code that will later be used on W.A.L.T.E.R. 2.0.

					-------------------------------------------------------------------------------
					v0.0.1 ALPHA 17-Feb-2014:
					Initial build - basic two wheeled chassis, with a Lynxmotion
						BotBoarduino (Arduino), a Lynxmotion SSC-32 Servo
						Controller, and two Lynxmotion CR servos for locomotion.

					This build was inspired by Lynxmotion's Two Wheeled Servo
						based rover.
					-------------------------------------------------------------------------------
					v0.0.2 ALPHA 19-Feb-2014:
					Added Pan/Tilt sensor platform, with a Parallax PING ultrasonic
						range sensor, and a Sharp GP2Y0A21YK0F IR range sensor.

					Tweaked all servo home positions to be right, by adusting
						the servo offset values.
					-------------------------------------------------------------------------------
					v0.0.3 ALPHA 19-Feb-2014:

					-------------------------------------------------------------------------------
					v0.0.4 ALPHA 04-Mar-2014:
					I put all the print strings in F(), like F("string"), and this may
						have saved me enough dynamic memory that I can
						continue developing on this platform.
					--------------------------------------------------------------------------------
					v0.0.9 ALPHA 05-Mar-2014:
					Added more error checking in moveServoPw() and moveServoDegrees().

					I think I fixed problem where servoMoveDegrees() was using the wrong servo pulse
						width. More testing is needed to be sure.

					Extra bump of version number because yesterday's commit was a major change.

					Added license notice here and to header file.
					--------------------------------------------------------------------------------
					v0.1.0 ALPHA 06-Mar-2014:
					Finally got the scanArea() range right! It now scans up to a full 180 degree range.
					--------------------------------------------------------------------------------

	Dependencies:	Adafruit libraries:
						Adafruit_Sensor, Adafruit_TCS34725, Adafruit_TMP006, RTClib (Adafruit version)

					Hybotics libraries:
						None

	Comments:		Credit is given, where applicable, for code I did not originate.
*/

/*
	Required Arduino System
*/
#include <Wire.h>
#include <SoftwareSerial.h>

/*
	Additional sensors
*/
#include <Adafruit_Sensor.h>
#include <Adafruit_TCS34725.h>
#include <Adafruit_TMP006.h>

/*
	Additional libraries
*/
#include <RTClib.h>

/*
	Local stuff
*/
#include "Main.h"
#include "Pitches.h"

/************************************************************/
/*	Global variables										*/
/************************************************************/

/*
	Initialize our sensors

	We have:
		These are from Adafruit:
			http://www.adafruit.com/products/1334 (TCS34725 RGB Color sensor)
			http://www.adafruit.com/products/1296 (TMP006 Heat sensor)
			http://www.adafruit.com/products/264 (DS1307 Realtime Clock)

		From other sources:
			GP2Y0A21YK0F IR Ranging sensors (1)
			PING Ultrasonic Ranging sensors (1)
*/

Adafruit_TCS34725 rgb = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
Adafruit_TMP006 heat = Adafruit_TMP006();
RTC_DS1307 clock;

/************************************************************/
/*	Initialize global variables								*/
/************************************************************/

/*
	These variables control the display of various information
		on the seven segment and matrix displays.
*/

//	Date display
boolean displayDate = true;
uint8_t dateMinuteCount = 0;

//	Time display
boolean displayTime = true;
uint8_t timeMinuteCount = 0;

//	Temperature display
boolean displayTemperature = true;
uint8_t temperatureMinuteCount = 0;

/*
	Time control variables
*/
uint8_t currentMinute = 0;
uint8_t lastMinute = -1;
long minuteCount = 0;						//	Count the time, in minutes, since we were last restarted

//	Enable run once only loop code to run
boolean firstLoop = true;

//	Error control
uint16_t errorStatus;
String errorMsg;

/*
	Setup all our serial devices
*/

//	Hardware Serial: Console and debug (replaces Serial.* routines)
SoftwareSerial console(SERIAL_CONSOLE_RX_PIN, SERIAL_CONSOLE_TX_PIN);

//	Software Serial: SSC-32 Controller
SoftwareSerial ssc32(SERIAL_SSC32_RX_PIN, SERIAL_SSC32_TX_PIN);

//	Software Serial: XBee Mesh Wireless
SoftwareSerial xbee(SERIAL_XBEE_RX_PIN, SERIAL_XBEE_TX_PIN);

/************************************************************/
/*	Initialize Servos										*/
/************************************************************/

Servo gripLift, gripWrist, gripGrab, pan, tilt;

/************************************************************/
/*	Initialize Sensors										*/
/************************************************************/

//	Total number of area readings taken, or -1 if data is not valid
int nrAreaScanReadings;

//  These are where the range sensor readings are stored.
int ping[MAX_NUMBER_PING];
float ir[MAX_NUMBER_IR];

boolean areaScanValid = false, hasMoved = false;

//	Readings for full area scans
AreaScanReading areaScan[MAX_NUMBER_AREA_READINGS];

//	The TCS34725 RGB Color Sensor
ColorSensor colorData = {
	0,
	0,
	0,
	0,
	0
};

//	The TMP006 Heat sensor
HeatSensor heatData = {
	0.0,
	0.0
};

/************************************************************/
/*	Bitmaps for the matrix 8x8 display drawBitMap() routine	*/
/************************************************************/

static const uint8_t PROGMEM
	hpa_bmp[] = {
		B10001110,
		B10001001,
		B11101110,
		B10101000,
		B00000100,
		B00001010,
		B00011111,
		B00010001
	},

	c_bmp[] = {
		B01110000,
		B10001000,
		B10000000,
		B10001000,
		B01110000,
		B00000000,
		B00000000,
		B00000000
	},

	f_bmp[] = {
		B11111000,
		B10000000,
		B11100000,
		B10000000,
		B10000000,
		B00000000,
		B00000000,
		B00000000
	},

	m_bmp[] = {
		B00000000,
		B00000000,
		B00000000,
		B00000000,
		B11101110,
		B10111010,
		B10010010,
		B10000010
	},

	date_bmp[] = {
		B10110110,
		B01001001,
		B01001001,
		B00000100,
		B00000100,
		B01111100,
		B10000100,
		B01111100
	},

	year_bmp[] = {
		B00000000,
		B10001000,
		B10001000,
		B01110000,
		B00101011,
		B00101100,
		B00101000,
		B00000000
	},

	am_bmp[] = {
		B01110000,
		B10001010,
		B10001010,
		B01110100,
		B00110110,
		B01001001,
		B01001001,
		B01001001
	},

	pm_bmp[] = {
		B01111100,
		B10000010,
		B11111100,
		B10000000,
		B10110110,
		B01001001,
		B01001001,
		B01001001
	},

	allon_bmp[] = {
		B11111111,
		B11111111,
		B11111111,
		B11111111,
		B11111111,
		B11111111,
		B11111111,
		B11111111
	};

/*
	Code starts here
*/

/************************************************************/
/*	Utility routines										*/
/************************************************************/

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

/************************************************************/
/*	Display routines										*/
/************************************************************/

/*
	Display the TCS34725 RGB color sensor readings
*/
void displayColorSensorReadings (ColorSensor *colorData) {
	console.print(F("Color Temperature: "));
	console.print(colorData->colorTemp, DEC);
	console.print(F(" K - Lux: "));
	console.print(colorData->lux, DEC);
	console.print(F(" - Red: "));
	console.print(colorData->red, DEC);
	console.print(F(", Green: "));
	console.print(colorData->green, DEC);
	console.print(F(", Blue: "));
	console.print(colorData->blue, DEC);
	console.print(F(", C: "));
	console.println(colorData->c, DEC);
	console.println();
}

/*
	Display the TMP006 heat sensor readings
*/
void displayHeatSensorReadings (HeatSensor *heatData) {
	float objCelsius = heatData->objectTemp;
	float objFahrenheit = toFahrenheit(objCelsius);
	float dieCelsius = heatData->dieTemp;
	float dieFahrenheit = toFahrenheit(dieCelsius);

	console.print(F("Object Temperature: "));
	console.print(objFahrenheit);
	console.print(F(" F, "));
	console.print(objCelsius);
	console.println(F(" C"));
	console.print(F("Die Temperature: "));
	console.print(dieFahrenheit);
	console.print(F(" F, "));
	console.print(dieCelsius);
	console.println(F(" C"));
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

/************************************************************/
/*	Sensor reading routines									*/
/************************************************************/

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
int readPING (byte sensorNr, boolean units=true) {
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

/********************************************************************/
/*	Lynxmotion BotBoarduino specific routines						*/
/********************************************************************/

/********************************************************************/
/*	Lynxmotion SSC-32 Servo Controller routines						*/
/********************************************************************/

/*
    Move a servo by pulse width in ms (500ms - 2500ms) - Modified to use HardwareSerial2()
*/
uint16_t moveServoPw (Servo *servo, int servoPosition, boolean term, int moveSpeed = 0, int moveTime = 0) {
	errorStatus = 0;
	servo->error = 0;
  
	if ((servoPosition >= servo->minPulse) && (servoPosition <= servo->maxPulse)) {
		ssc32.print("#");
		ssc32.print(servo->pin);
		ssc32.print(" P");
		ssc32.print(servoPosition + servo->offset);

		servo->msPulse = servoPosition;
		servo->angle = ((servoPosition - SERVO_CENTER_MS) / 10);
    
		if (servo->maxDegrees == 180) {
			servo->angle += 90;
		}
	} else if ((servoPosition < servo->minPulse) || (servoPosition > servo->maxPulse)) {
		servo->error = 201;
		errorMsg = String("(moveServoPw) Servo pulse is out of range");
	}
 
	if (errorStatus != 0) {
		processError(errorStatus, errorMsg);
	} else {
		//  Add servo move speed
		if (moveSpeed != 0) {
			ssc32.print(" S");
			ssc32.print(moveSpeed);
		}
    
		//  Terminate the command
		if (term) {
			if (moveTime != 0) {
				ssc32.print(" T");
				ssc32.print(moveTime);
			}

			ssc32.println();
		}
  	}

	if (errorStatus != 0) {
		servo->error = errorStatus;
	}

  	return errorStatus;
}

/*
    Move a servo by degrees (-90 to 90) or (0 - 180) - Modified to use BMSerial
*/
uint16_t moveServoDegrees (Servo *servo, int servoDegrees, boolean term, int moveSpeed = 0, int moveTime = 0) {
	int servoPulse;

	errorStatus = 0;
	servo->error = 0;
  
	//  Convert degrees to ms for the servo move
	if (servo->maxDegrees == 90) {
		servoPulse = SERVO_CENTER_MS + (servoDegrees * 11) + servo->offset + 50;
	} else if (servo->maxDegrees == 180) {
		servoPulse = SERVO_CENTER_MS + ((servoDegrees -90) * 11) + servo->offset + 50;
	} else {
		errorStatus = 202;
		errorMsg = String("(moveServoDegrees) Servo position (degrees) is invalid");
	}

	if (errorStatus != 0) {
		processError(errorStatus, errorMsg);
	} else {
		if ((servoPulse >= servo->minPulse) && (servoPulse <= servo->maxPulse)) {
			moveServoPw(servo, servoPulse, true);
		} else {
			errorStatus = 201;
			errorMsg = String("(moveServoDegrees) Servo pulse is out of range");
		}
	}

	console.print(F("(moveServoDegrees #1) servoDegrees = "));
	console.print(servoDegrees);
	console.print(F(",  servoPulse = "));
	console.print(servoPulse);
	console.print(F(", servo->minPulse = "));
	console.print(servo->minPulse);
	console.print(F(", servo->maxPulse = "));
	console.println(servo->maxPulse);

	if (errorStatus != 0) {
		processError(errorStatus, errorMsg);
		servo->error = errorStatus;
	}

	return errorStatus;
}

/*
	Scan the area for objects
*/
uint16_t scanArea (Servo *pan, int startDeg, int stopDeg, int incrDeg) {
	int readingNr, positionDeg = startDeg;
	int nrReadings, totalRangeDeg = 0;

	errorStatus = 0;
	pan->error = 0;

	console.println(F("(scanArea #1) Checking parameters.."));

	//	Check the parameters
	if (startDeg > stopDeg) {
		//	Start can't be greater than stop
		errorStatus = 401;
		errorMsg = String(F("(scanArea #1) Start position is > ending position"));
	} else if (((SERVO_MAX_DEGREES == 90) && ((startDeg < -90) || (stopDeg > 90))) || ((SERVO_MAX_DEGREES == 180) && ((startDeg < 0) || (stopDeg > 180)))) {
		//	One or more parameters is outside of the valid range
		errorStatus = 402;
		errorMsg = String(F("(scanArea #2) Servo maximum degrees is invalid"));
	} else {
		//	Calculate the total range, in degrees
		totalRangeDeg = abs(stopDeg - startDeg);

		//	Calculate the number of readings we need room for
		nrReadings = abs(totalRangeDeg / incrDeg);

		//	More error checking
		if (nrReadings > MAX_NUMBER_AREA_READINGS) {
			//	We can't take this many readings
			errorStatus = 403;
			errorMsg = String(F("(scanArea #3) Number of readings is invalid"));
		} else if (totalRangeDeg > 180) {
			//	Servos can only move up to 180 degrees
			errorStatus = 404;
			errorMsg =  String(F("(scanArea #4) Requested servo move is invalid"));
		} else if (incrDeg > (totalRangeDeg / nrReadings)) {
			//	Increment is too large for our range
			errorStatus = 405;
			errorMsg = String(F("(scanArea #5) Move increment is too large for movement range"));
		} else {
			//	Continue normal processing
			readingNr = 0;

			console.println(F("Scanning the area.."));

			console.print(F("(scanArea #6) startDeg = "));
			console.print(startDeg);
			console.print(F(", stopDeg = "));
			console.print(stopDeg);
			console.print(F(", incrDeg = "));
			console.print(incrDeg);
			console.print(F(", readingNr = "));
			console.println(readingNr);

			readingNr = 0;

			while (positionDeg <= stopDeg) {
				console.print(F("(scanArea #7) positionDeg = "));
				console.print(positionDeg);
				console.print(F(", readingNr = "));
				console.println(readingNr);

				errorStatus = moveServoDegrees(pan, positionDeg, true, 5000, 5000);

				if (errorStatus != 0) {
					processError(errorStatus, F("(scanArea #8) There was a problem moving the pan servo"));
					break;
				} else {
					//	Take a reading from each pan/tilt sensor in cm
					areaScan[readingNr].ping = readPING(PING_FRONT_CENTER, true);
					areaScan[readingNr].ir = readIR(IR_FRONT_CENTER);
					areaScan[readingNr].positionDeg = positionDeg;

					readingNr += 1;
					positionDeg += incrDeg;
				}
			}

			if (!errorStatus) {
				initPanTilt(pan, &tilt);
			}
		}
	}

	if (errorStatus != 0) {
		nrAreaScanReadings = -1;
		areaScanValid = false;
		pan->error = errorStatus;
	} else {
		//	Set the number of readings taken
		nrAreaScanReadings = readingNr;
		areaScanValid = true;
		pan->error = 0;
	}

	return errorStatus;
}

/********************************************************/
/*	Initialization routines								*/
/********************************************************/

void initGripper (Servo *raise, Servo *wrist, Servo *grab) {
	console.println(F("Initializing Gripper Position.."));
  
	//  Put the 3DOF gripper at home position
	moveServoPw(grab, SERVO_GRIP_GRAB_HOME, 0, 0, false);
	moveServoPw(raise, SERVO_GRIP_LIFT_HOME + 150, 0, 0, false);
	moveServoPw(wrist, SERVO_GRIP_WRIST_HOME, 0, 0, true);
}

/*
	Set the Pan/Tilt to Home Position
*/
void initPanTilt (Servo *pan, Servo *tilt) {
	console.println(F("Initializing Pan/Tilt Position.."));
  
	//  Put the front pan/tilt at home position
	moveServoPw(pan, SERVO_PAN_HOME, 0, 0, false);
	moveServoPw(tilt, SERVO_TILT_HOME, 0, 0, true);
}

/*
	Initialize sensors
*/
void initSensors (void) {
	console.println(F("Initializing Sensors.."));
/*
	console.println(F("     TMP006 Heat..");

	//	Initialize the TMP006 heat sensor
	if (! heat.begin()) {
		console.println(F("There was a problem initializing the TMP006 heat sensor .. check your wiring or I2C ADDR!"));
		while(1);
	}
	
	console.println(F("     TCS34725 RGB Color.."));

	//	Initialize the TCS34725 color sensor
	if (! rgb.begin()) {
		console.println(F("There was a problem initializing the TCS34725 RGB color sensor .. check your wiring or I2C ADDR!"));
		while(1);
	}
*/
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
	lift->offset = SERVO_GRIP_LIFT_OFFSET;
	lift->homePos = SERVO_GRIP_LIFT_HOME;
	lift->msPulse = 0;
	lift->angle = 0;
	lift->minPulse = SERVO_GRIP_LIFT_MIN;
	lift->maxPulse = SERVO_GRIP_LIFT_MAX;
	lift->maxDegrees = SERVO_MAX_DEGREES;
	lift->error = 0;

	wrist->pin = SERVO_GRIP_WRIST_PIN;
	wrist->offset = SERVO_GRIP_WRIST_OFFSET;
	wrist->homePos = SERVO_GRIP_WRIST_HOME;
	wrist->msPulse = 0;
	wrist->angle = 0;
	wrist->minPulse = SERVO_GRIP_WRIST_MIN;
	wrist->maxPulse = SERVO_GRIP_WRIST_MAX;
	wrist->maxDegrees = SERVO_MAX_DEGREES;
	wrist->error = 0;

	grab->pin = SERVO_GRIP_GRAB_PIN;
	grab->offset = SERVO_GRIP_GRAB_OFFSET;
	grab->homePos = SERVO_GRIP_GRAB_HOME;
	grab->msPulse = 0;
	grab->angle = 0;
	grab->minPulse = SERVO_GRIP_GRAB_MIN;
	grab->maxPulse = SERVO_GRIP_GRAB_MAX;
	grab->maxDegrees = SERVO_MAX_DEGREES;
	grab->error = 0;

	pan->pin = SERVO_PAN_PIN;
	pan->offset = SERVO_PAN_OFFSET;
	pan->homePos = SERVO_PAN_HOME;
	pan->msPulse = 0;
	pan->angle = 0;
	pan->minPulse = SERVO_PAN_LEFT_MIN;
	pan->maxPulse = SERVO_PAN_RIGHT_MAX;
	pan->maxDegrees = SERVO_MAX_DEGREES;
	pan->error = 0;

	tilt->pin = SERVO_TILT_PIN;
	tilt->offset = SERVO_TILT_OFFSET;
	tilt->homePos = SERVO_TILT_HOME;
	tilt->msPulse = 0;
	tilt->angle = 0;
	tilt->minPulse = SERVO_TILT_DOWN_MIN;
	tilt->maxPulse = SERVO_TILT_UP_MAX;
	tilt->maxDegrees = SERVO_MAX_DEGREES;
	tilt->error = 0;
}

/************************************************************/
/*	Miscelaneous routines									*/
/************************************************************/

/*
    Process error conditions
*/
void processError (byte err, String msg) {
	console.print(F("Error "));
	console.print(err);
	console.print(F(": "));
	console.print(msg);
	console.println(F("!"));
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

/*
	Runs once to initialize everything
*/
void setup (void) {
	//  Start up the Wire library
	Wire.begin();

	//  Initialize the console port
	console.begin(115200);

	console.println();
	console.print(F("SES Rover, version "));
	console.print(BUILD_VERSION);
	console.print(F(" on "));
	console.print(BUILD_DATE);
	console.print(F(" for the "));
	console.print(BUILD_BOARD);
	console.println(F("."));

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

	//	Initialize and turn off the TCS34725 RGB Color sensor's LED
	pinMode(COLOR_SENSOR_LED, OUTPUT);
	digitalWrite(COLOR_SENSOR_LED, LOW);
	delay(250);
	digitalWrite(COLOR_SENSOR_LED, HIGH);
	delay(250);
	digitalWrite(COLOR_SENSOR_LED, LOW);

	//	Initialize all sensors
	initSensors();

 	//  Initialize all servos
 	initServos(&gripLift, &gripWrist, &gripGrab, &pan, &tilt);

	//	Set the Pan/Tilt to home position
	initPanTilt(&pan, &tilt);

	//	Set the Gripper to home position
	initGripper(&gripLift, &gripWrist, &gripGrab);

	console.println();
}

/*
	Runs forever
*/
void loop (void) {
	//	The current date and time from the DS1307 real time clock
	DateTime now = clock.now();

	//	Display related variables
	boolean amTime;
	uint8_t displayNr = 0, count = 0;
	uint8_t currentHour = now.hour(), nrDisplays = 0;

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
/*
		//	Scan the entire 180 degree range and take readings
		console.println(F("Doing initial area scan.."));

		errorStatus = scanArea(&pan, -90, 90, 10);

		if (errorStatus != 0) {
			processError(errorStatus, "Main, firstLoop");
		}

		wait(LOOP_DELAY_SECONDS);
*/
		firstLoop = false;
	}

	console.println(F("Getting Distance Sensor readings.."));

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

	/*
		Put distance related reactive behaviors HERE
	*/

	/*
		Read the TCS34725 RGB color sensor
	*/

/*
	rgb.getRawData(&colorData.red, &colorData.green, &colorData.blue, &colorData.c);
	colorData.colorTemp = rgb.calculateColorTemperature(colorData.red, colorData.green, colorData.blue);
	colorData.lux = rgb.calculateLux(colorData.red, colorData.green, colorData.blue);

	displayColorSensorReadings(&colorData);
*/

	/*
		Read the TMP006 heat sensor
	*/

/*
	heatData.dieTemp = heat.readDieTempC();
	heatData.objectTemp = heat.readObjTempC();
	
	displayHeatSensorReadings(&heatData);

	console.println();
*/

	if (errorStatus != 0) {
		processError(errorStatus, "(MAIN) Unknown Error");
	}

	//	Count the minutes
	if (currentMinute != lastMinute) {
		if (DISPLAY_INFORMATION) {
			dateMinuteCount += 1;
			temperatureMinuteCount += 1;
			timeMinuteCount += 1;
		}

		minuteCount += 1;
		lastMinute = currentMinute;
	}

	/*
		Update the information display control variables
	*/

	if (DISPLAY_INFORMATION) {
		displayDate = (dateMinuteCount == DISPLAY_DATE_FREQ_MIN);
		displayTemperature = (temperatureMinuteCount == DISPLAY_TEMPERATURE_FREQ_MIN);
		displayTime = (timeMinuteCount == DISPLAY_TIME_FREQ_MIN);
	}

	/*
		Delay a bit, to allow time to read the Serial Monitor information log
	*/
	wait(LOOP_DELAY_SECONDS);

	console.println();
}
