/*
	Program:      	SES Rover, Motor_Servo_Test.ino - Motor experimentation and test sketch
	Date:         	17-Apr-2014
	Version:      	0.0.3 ALPHA Lynxmotion's BotBoarduino and SSC-32

	Platform:		Lynxmotion's BotBoarduino (Arduino),
						with Lynxmotion's SSC-32 Servo Controller,
						and a 3DOF (Raise/Lower, Wrist Rotate, Open/Close) Little Grip gripper

	Purpose:		To experiment with various sensor configurations, tracking objects (heat or
						color based), course following, manipulation of the environment, and to
						test code that will later be used on W.A.L.T.E.R. 2.0.

					-------------------------------------------------------------------------------
					v0.0.1 ALPHA 19-Feb-2014:
					Initial build from W.A.L.T.E.R. 2.0 code
					-------------------------------------------------------------------------------
					v0.0.2 ALPHA 20-Feb-2014:
					Adding routines to scan the area and move the robot.

					Rewrote the ServoMotor struct to use naming like the Servo struct.
					-------------------------------------------------------------------------------
					v0.0.3 ALPHA 17-Apr-2014:
					Switched processor to the Arduino Mega 2560 R3 board, because there is not enough
						memory on a BotBoarduino (Arduino Duemilanove) for what I need to do.

					Making adjustments to take advantage of the extra hardware serial ports.
					-------------------------------------------------------------------------------

	Dependencies:	Adafruit libraries:
						Adafruit_Sensor, for the Unified Sensor Model,
						Adafruit_TCS34725, for the TCS34725 RGB Color sensor,
						Adafruit_TMP006, for the TMP006 heat sensor, and
						RTClib, for the DS1307 real time clock.

					Hybotics libraries:
						None

	Comments:		Credit is given, where applicable, for code I did not originate.
*/
#include <Wire.h>
#include <SoftwareSerial.h>

/*
	Additional sensors
*/

#include <RTClib.h>

#include "Motor_Servo_Test.h"

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
byte error = 0;

/************************************************************/
/*	Initialize Objects										*/
/************************************************************/

RTC_DS1307 clock;

/*
	Setup all our serial devices
*/

//	Hardware Serial: Console and debug (replaces Serial.* routines)
SoftwareSerial console(SERIAL_CONSOLE_RX_PIN, SERIAL_CONSOLE_TX_PIN);

//	Software Serial: SSC-32 Servo Controller
SoftwareSerial ssc32(SERIAL_SSC32_RX_PIN, SERIAL_SSC32_TX_PIN);

/************************************************************/
/*	Initialize Servos										*/
/************************************************************/

Servo gripLift, gripWrist, gripGrab, pan, tilt;

ServoMotor leftMotor = {
	SERVO_MOTOR_LEFT_PIN,
	SERVO_MOTOR_LEFT_OFFSET,
	SERVO_MOTOR_LEFT_NEUTRAL,
	SERVO_MOTOR_LEFT_MIN,
	SERVO_MOTOR_LEFT_MAX,
	0,
	0
};

ServoMotor rightMotor = {
	SERVO_MOTOR_RIGHT_PIN,
	SERVO_MOTOR_RIGHT_OFFSET,
	SERVO_MOTOR_RIGHT_NEUTRAL,
	SERVO_MOTOR_RIGHT_MIN,
	SERVO_MOTOR_RIGHT_MAX,
	0,
	0
};

//	Total number of area readings taken, or -1 if data is not valid
int nrAreaReadings;

//	PING Ultrasonic range sensor readings
int ping[MAX_NUMBER_PING];

//	Sharp GP2Y0A21YK0F IR range sensor readings
float ir[MAX_NUMBER_IR];

AreaDistanceReading areaScan[MAX_NUMBER_AREA_READINGS];

/************************************************************/
/*	Display routines										*/
/************************************************************/

/*
	Display the TCS34725 RGB color sensor readings
*/
void displayColorSensorReadings (ColorSensor *colorData) {
	console.print("Color Temperature: ");
	console.print(colorData->colorTempC, DEC);
	console.print(" K - ");
	console.print("Lux: ");
	console.print(colorData->lux, DEC);
	console.print(" - ");
	console.print("Red: ");
	console.print(colorData->red, DEC);
	console.print(" ");
	console.print("Green: ");
	console.print(colorData->green, DEC);
	console.print(" ");
	console.print("Blue: ");
	console.print(colorData->blue, DEC);
	console.print(" ");
	console.print("C: ");
	console.println(colorData->c, DEC);
	console.println();
}

/*
	Display the TMP006 heat sensor readings
*/
void displayHeatSensorReadings (HeatSensor *heatData) {
	float objCelsius = heatData->objectTempC;
	float objFahrenheit = toFahrenheit(objCelsius);
	float dieCelsius = heatData->dieTempC;
	float dieFahrenheit = toFahrenheit(dieCelsius);

	console.print("Object Temperature: ");
	console.print(objFahrenheit);
	console.print(" F, ");
	console.print(objCelsius);
	console.println(" C");
	console.print("Die Temperature: ");
	console.print(dieFahrenheit);
	console.print(" F, ");
	console.print(dieCelsius);
	console.println(" C");
}

/*
    Display the GP2Y0A21YK0F IR sensor readings (cm)
*/
void displayIR (void) {
	int sensorNr = 0;
  
	console.println("IR Sensor readings:");

	while (sensorNr < MAX_NUMBER_IR) {
		console.print("IR #");
		console.print(sensorNr + 1);
		console.print(" range = ");
		console.print(ir[sensorNr]);
		console.println(" cm");

		sensorNr += 1;
	}

	console.println();
}

/*
	Display the readings from the PING Ultrasonic sensors
*/
void displayPING (void) {
	int sensorNr = 0;
  
	console.println("PING Ultrasonic Sensor readings:");
  
	//	Display PING sensor readings (cm)
	while (sensorNr < MAX_NUMBER_PING) {
		console.print("Ping #");
		console.print(sensorNr + 1);
		console.print(" range = ");
		console.print(ping[sensorNr]);
		console.println(" cm");

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
/*	Lynxmotion SSC-32 Servo Controller routines						*/
/********************************************************************/

/*
    Move a servo by pulse width in ms (500ms - 2500ms) - Modified to use HardwareSerial2()
*/
uint16_t moveServoPw (Servo *servo, int servoPosition, boolean term, int moveSpeed = 0, int moveTime = 0) {
	uint16_t errorStatus = 0;
	String errorMsg;

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

	uint16_t errorStatus = 0;
	String errorMsg;

	servo->error = 0;
  
	//  Convert degrees to ms for the servo move
	if (servo->maxDegrees == 90) {
		servoPulse = SERVO_CENTER_MS + (servoDegrees * 11) + servo->offset;
	} else if (servo->maxDegrees == 180) {
		servoPulse = SERVO_CENTER_MS + ((servoDegrees -90) * 11) + servo->offset;
	} else {
		errorStatus = 202;
		errorMsg = String("(moveServoDegrees) Servo position (degrees) is invalid");
	}

	console.print(F("(moveServoDegrees #1) servoPulse = "));
	console.print(servoPulse);
	console.print(F(", servoDegrees = "));
	console.println(servoDegrees);

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
	uint16_t error = 0, readingNr = 0, nrReadings = 0;
	uint16_t positionDeg = 0;
        int totalRangeDeg = 0;

	console.println("(scanArea #1) Checking parameters..");

	//	Check the parameters
	if (startDeg > stopDeg) {
		//	Start can't be greater than stop
		error = 400;
	} else if (((SERVO_MAX_DEGREES == 90) && ((startDeg < -90) || (stopDeg > 90))) || ((SERVO_MAX_DEGREES == 180) && ((startDeg < 0) || (stopDeg > 180)))) {
		//	One or more parameters is outside of the valid range
		error = 401;
	} else if ((startDeg < pan->minPulse) || (stopDeg > pan->maxPulse)) {
		//	Out of range for the pan servo
		error = 402;
	} else {
		//	Calculate the total range, in degrees
		totalRangeDeg = abs(stopDeg - startDeg);

		//	Calculate the number of readings we need room for
		nrReadings = totalRangeDeg / incrDeg;

		//	More error checking
		if (totalRangeDeg > 180) {
			//	Servos can only move up to 180 degrees
			error = 403;
		} else if (nrReadings > MAX_NUMBER_AREA_READINGS) {
			//	We can't take this many readings
			error = 404;
		} else if (incrDeg > totalRangeDeg) {
			//	Increment is too large for our range
			error = 405;
		} else {
			//	Continue normal processing
			readingNr = 0;

			console.println("Scanning the area..");

			for (positionDeg = startDeg; positionDeg < stopDeg; positionDeg += incrDeg) {
				moveServoDegrees(pan, positionDeg, 0, 0, true);

				//	Take a reading from each pan/tilt sensor in cm
				areaScan[readingNr].ping = readPING(PING_FRONT_CENTER, true);
				areaScan[readingNr].ir = readIR(IR_FRONT_CENTER);
				areaScan[readingNr].positionDeg = positionDeg;

				readingNr += 1;
			}
		}
	}

	if (error != 0) {
		processError(error, "scanArea");
		nrAreaReadings = -1;
	} else {
		//	Set the number of readings taken
		nrAreaReadings = readingNr;
	}

	return error;
}

/*
	Set the motor speed
*/
uint16_t setMotorSpeed (ServoMotor *servoMotor, int spd, bool term) {
	uint16_t errorStatus = 0, pulse = SERVO_MOTOR_NEUTRAL;
	int motorSpeed = spd;

	if ((spd < SERVO_MOTOR_MIN_SPEED) or (spd > SERVO_MOTOR_MAX_SPEED)) {
		errorStatus = 501;
		processError(errorStatus, F("(setMotorSpeed) Speed is out of range"));
	} else {
		Servo servo;

		servo.pin = servoMotor->pin;
		servo.offset = servoMotor->offset;
		servo.homePos = servoMotor->neutralPos;
		servo.minPulse = servoMotor->minPulse;
		servo.maxPulse = servoMotor->maxPulse;
		servo.msPulse = servoMotor->pulse;
		servo.error = 0;

		pulse += servoMotor->offset;

		if (servoMotor->direction == false) {
			motorSpeed *= -1;
		}

		pulse += motorSpeed;

		errorStatus = moveServoPw(&servo, pulse, term);

		if (errorStatus != 0) {
			servoMotor->error = servo.error;
			processError(errorStatus, F("(setMotorSpeed) Could not set the motor speed"));
		}
	}

	return errorStatus;
}

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
/*	Miscelaneous routines									*/
/************************************************************/

/*
    Process error conditions
*/
void processError (byte err, String routine) {
	console.print("Error in routine ");
	console.print(routine);
	console.print(", Code: ");
	console.print(err);
	console.println("!");
}

/*
	Wait for a bit to allow time to read the Console Serial Monitor log
*/
void wait (uint8_t nrSeconds) {
	uint8_t count;

	console.print("Waiting");

	for (count = 0; count < nrSeconds; count++) {
		console.print(".");
		delay(1000);
	}

	console.println();
}

/********************************************************/
/*	Initialization routines								*/
/********************************************************/

void initGripper (Servo *lift, Servo *wrist, Servo *grab) {
	console.println("Initializing Gripper Position..");
  
	//  Put the 3DOF gripper at home position
	moveServoPw(grab, SERVO_GRIP_GRAB_HOME, 0, 0, false);
	moveServoPw(lift, SERVO_GRIP_LIFT_HOME + 150, 0, 0, false);
	moveServoPw(wrist, SERVO_GRIP_WRIST_HOME, 0, 0, true);
}

/*
	Set the Pan/Tilt to Home Position
*/
void initPanTilt (Servo *pan, Servo *tilt) {
	console.println("Initializing Pan/Tilt Position..");
  
	//  Put the front pan/tilt at home position
	moveServoPw(pan, SERVO_PAN_HOME, 0, 0, false);
	moveServoPw(tilt, SERVO_TILT_HOME, 0, 0, true);
}

/*
	Initialize the motors.
*/
uint16_t initMotors (ServoMotor *left, ServoMotor *right) {
	uint16_t errorStatus = 0;

	console.println("Initializing Motors..");

	errorStatus = setMotorSpeed(left, SERVO_MOTOR_LEFT_NEUTRAL, false);

	if (errorStatus != 0) {
		processError(errorStatus, F("(initMotors) Could not initialze the LEFT motor"));
	} else {
		left->direction = SERVO_MOTOR_LEFT_DIRECTION;

		errorStatus = setMotorSpeed(right, SERVO_MOTOR_RIGHT_NEUTRAL, true);

		if (errorStatus != 0) {
			processError(errorStatus, F("(initMotors) Could not initialize the RIGHT motor"));
		} else {
			right->direction = SERVO_MOTOR_RIGHT_DIRECTION;
		}
	}

	return errorStatus;
}

/*
	Initialize sensors
*/
void initSensors (void) {
	console.println("Initializing Sensors..");
	console.println("     DS1307 Real Time Clock..");

	//	Check to be sure the RTC is running
//	if (! clock.isrunning()) {
//		console.println("The Real Time Clock is NOT running!");
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
/*	Runs once to initialize everything						*/
/************************************************************/
void setup (void) {
	uint16_t errorStatus = 0;

	//  Initialize the console port
	console.begin(115200);

	console.println();
	console.print("SES Rover Motor Servo Test, version ");
	console.print(BUILD_VERSION);
	console.print(" on ");
	console.println(BUILD_DATE);
	console.print("for ");
	console.print(BUILD_BOARD);
	console.println(".");

	console.println();
	console.println("Initializing Serial Ports..");

	//	Initialize the SSC-32 servo controller port
	ssc32.begin(115200);

	console.println("Initializing Digital Pins..");

	//  Initialize the LED pin as an output.
	pinMode(HEARTBEAT_LED, OUTPUT);
	digitalWrite(HEARTBEAT_LED, LOW);

	//	Initialize all sensors
	initSensors();

 	//  Initialize all servos
 	initServos(&gripLift, &gripWrist, &gripGrab, &pan, &tilt);

	//	Set the Pan/Tilt to home position
	initPanTilt(&pan, &tilt);

	//	Set the Gripper to home position
	initGripper(&gripLift, &gripWrist, &gripGrab);

	//	Initialize the motors
	errorStatus = initMotors(&leftMotor, &rightMotor);

	if (errorStatus != 0) {
		processError(errorStatus, F("(SETUP) There was a problem initializing the motors"));
	} else {
		console.println();

		//	Start the motors, forward
		console.println(F("Starting the motors, forward"));
		setMotorSpeed(&leftMotor, 500, false);
		setMotorSpeed(&rightMotor, 500, true);

		delay(5000);

		//	Stop the motors
		console.println(F("Stopping the motors"));
		setMotorSpeed(&leftMotor, 0, false);
		setMotorSpeed(&rightMotor, 0, true);

		delay(2000);

		//	Start the motors, reverse
		console.println(F("Starting the motors, reverse"));
		setMotorSpeed(&leftMotor, -500, false);
		setMotorSpeed(&rightMotor, -500, true);

		delay(5000);

		//	Stop the motors
		console.println(F("Stopping the motors"));
		setMotorSpeed(&leftMotor, 0, false);
		setMotorSpeed(&rightMotor, 0, true);
	}
}

/************************************************************/
/*	Runs forever											*/
/************************************************************/
void loop (void) {
	//	The current date and time from the DS1307 real time clock
	DateTime now = clock.now();

	//	Display related variables
	boolean amTime, areaScanValid = false, hasMoved = false;
	uint8_t displayNr = 0, count = 0;
	uint8_t readingNr = 0, areaClosestReadingPING = 0, areaClosestReadingIR = 0;
	uint8_t areaFarthestReadingPING = 0, areaFarthestReadingIR = 0;
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
		console.println("Entering the main loop..");

		lastMinute = currentMinute;

		areaScanValid = false;

		//	Scan the entire 180 degree range and take readings
		console.println("Doing initial area scan..");

		error = scanArea(&pan, -90, 90, 10);

		if (error != 0) {
			processError(error, "Main, firstLoop");
		} else {
			areaScanValid = true;
		}

		firstLoop = false;
	}

	console.println("Getting Distance Sensor readings..");

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

	//	Find the closest and farthest objects
	if (areaScanValid) {
		areaClosestReadingPING = 0;
		areaClosestReadingIR = 0;
		areaFarthestReadingPING = 0;
		areaFarthestReadingIR = 0;

		console.println("Finding the closest and farthest objects..");

		//	Find the closest and farthest objects
		for (readingNr = 0; readingNr < nrAreaReadings; readingNr++) {
			//	Check for the closest object
			if (areaScan[readingNr].ping < areaScan[areaClosestReadingPING].ping) {
				areaClosestReadingPING = readingNr;
			}

			if (areaScan[readingNr].ir <  areaScan[areaClosestReadingIR].ir) {
				areaClosestReadingIR = readingNr;
			}

			//	Check for the farthest object
			if (areaScan[readingNr].ping > areaScan[areaFarthestReadingPING].ping) {
				areaFarthestReadingPING = readingNr;
			}

			if (areaScan[readingNr].ir > areaScan[areaFarthestReadingIR].ir) {
				areaFarthestReadingIR = readingNr;
			}
		}
	} else {
		console.println("Area scan is not valid..");
	}

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

	//	Move the motors
	console.println("Moving the motors..");
	setMotorSpeed(&leftMotor, 100, false);
	setMotorSpeed(&rightMotor, 100, true);

	/*
		Delay a bit, to allow time to read the Serial Monitor information log
	*/
	wait(LOOP_DELAY_SECONDS);

	console.println();
}
