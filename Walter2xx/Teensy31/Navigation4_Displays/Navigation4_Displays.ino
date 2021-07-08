/*
	Program:		W.A.L.T.E.R. 2.0, Main navigation and reactive behaviors sketch
	Date:			13-Feb-2014
	Version:		0.2.6 Teensy 3.1 ALPHA

	Purpose:		Added two enum definitions for SensorLocation and MotorLocation. I'm
						not sure the sensor locations are going to work out.

					Added BMSerial ports for the SSC-32 and RoboClaw 2x5 controllers;
						defined a Motor struct to hold information about motors; added motor
						definitions and initialization; modified moveServoPw() and moveServoDegrees()
						to work with a BMSerial port.

					Converted to using a BMSerial() port for the hardware serial console port

					Converted to running the RoboClaw 2x5 motor controller in Packet Serial mode,
						with all the goodies - encoders, speed, acceleration, and distance.

					Modified moveServoPw() and moveServoDegrees() to use a pointer to the port

					-------------------------------------------------------------------------------------
					v0.1.7 ALPHA 13-Jan-2014:
					Added ColorSensor struct for RGB color sensor data; added code to read the TCS34725
						RGB color and TMP006 heat sensors.

					Added a control pin (COLOR_SENSOR_LED, pin 4) so the LED can be turned on and off.

					-------------------------------------------------------------------------------------
					v0.1.8 ALPHA 15-Jan 2014:
					Fixed a bug in readPING() - was not getting the duration, because code was in a comment.

					Now displaying readings from all sensors in the main loop. RGB color and Heat sensors are
						working. IMU seems to be working so far - still need to get useful information from it.

					The SSC-32 doesn't seem to like SoftwareSerial ports.

					I'm thinking more seriously about moving to the Arduino Mega ADK board. There is only about
						5Kb of program memory left, RAM is low, etc. I don't think I have a choice here.

					-------------------------------------------------------------------------------------
					v0.1.9 ALPHA 18-Jan-2014:
					Starting migration from the Arduino (BotBoarduino) to the Arduino Mega ADK board

					-------------------------------------------------------------------------------------
					v0.2.0 ALPHA 22-Jan-2014:
					Adding display driver code from IMU_Multi_Display_Test.ino

					I decided to keep the displays, because they can be useful for displaying status and
						error information from the robot.

					-------------------------------------------------------------------------------------
					v0.2.1 ALPHA 24-Jan-2014:
					Reorganized code, grouped similar kinds of routines together.

					I discovered the problem with the I2C is with the DFRobots Sensor Shield I have for the
						Arduino Mega ADK board. I removed the shield, rewired everything, and it's all
						working except the DS1307 real time clock. I have another one I can build when I
						get more solder.

					-------------------------------------------------------------------------------------
					v0.2.2 ALPHA 26-Jan-2014:
					Added the Hybotics_10DOF_Unified library to get orientation information - pitch, roll,
						and heading from the raw accelerometer and magnetometer (compass) data

					-------------------------------------------------------------------------------------
					v0.2.3 Teensy 3.1 ALPHA 09-Feb-2014:
					Beginning converstion to run on the Teensy 3.1 board:

					Set header definitions for the Teensy 3.1 hardware serial ports

					Removed all references to the BMSerial and RoboClaw libraries, because they just aren't
						compatible with the Teensy 3.1 right now. I am not sure how I want to or should proceed
						with this right now.

					Fixed a problem with the Hybotics_10DOF_Unified library where it was not able to find the
						Adafruit_BMP085_Unified.h file. I switched to using my version of this library
						(Hybotics_BMP180_Unified), and everything seems OK - more testing is needed. I should
						probably rename this library to Hybotics_BMP180_Unified to show there are differences
						from the Adafruit version.

					Commented out the code that checks to see if the DS1307 RTC is running, because there is some
						kind of bug in the isrunning() routine that causes it to return a failed check when the rtc
						is running.

					Added delay at the end of the main loop() to allow time to read the Serial Monitor log.

					This version builds cleanly for the Teensy 3.1. Testing begins!
					-------------------------------------------------------------------------------------
					v0.2.4 ALPHA 11-Feb-2014:
					Testing current code with Teensy 3.1 board.

					Minor changes required to bring code into sync with the Arduino Mega ADK code

					I adjusted the display routines a bit to give the combined output a nicer and easier to
						read look.

					The BMP180 Temperature/Pressure sensor is working fine.

					The TCS34725 RGB color and TMP006 heat sensors are working fine.

					The L3GD20 Gyro does not seem to work, apparently it is not detected - commented all Gyro
						related code out. I will look at the Adafruit_L3GD20 library and see if I can find any
						issues.

					I am getting readings from the LSM303DLHC Accelerometer and LSM303DLHC Magnetometer (Compass),
						but am not sure they are correct. More testing and validation is needed.

					-------------------------------------------------------------------------------------
					v0.2.5 ALPHA 12-Feb-2014:
					Changed the COLOR_SENSOR_LED from pin 4 to pin 11. Pin 4 conflicted with the
						location of the PING sensors on pins 4 to 6.

					Added the build version, date, and board to the heading announcement.

					I've activated all RoboClaw related code now, and everything builds and loads into the
						Teensy 3.1 now.

					Starting to test Paul's fixes to the BMSerial and RoboClaw libraries. I have communication
						on Serial2 (pins 9, 10) and Serial3 (pins 7, 8) at 115200 with the SSC-32, but nothing
						on Serial (pins 0, 1).

					I've added many console.println() statements to trace how far the code is executing

					This is the version running on my Teensy 3.1 now.

					---------------------------------------------------------------------------------------

	Dependencies:	Adafruit libraries:
						Adafruit_Sensor, Adafruit_L3GD20, Adafruit_TMP006, and Adafruit_TCS34725 libraries.
						Adafruit_LEDBackpack and Adafruit_GFX libraries (for the displays)

					Hybotics libraries:
						Hybotics_BMP180_Unified (forked from the Adafruit_BMP085 library)
						Hybotics_10DOF_Unified (forked from the Adafruit_10DOF library)
						Hybotics_LSM303DLHC_Unified (forked from the Adafruit_LSM303 library)

					Other libraries:
						RTClib for the DS1307 (Adafruit version)
						KalmanFilter

	Comments:		Credit is given, where applicable, for code I did not originate.
						This sketch started out as an Adafruit tutorial for the electret
						microphones being used for sound detection. I've also pulled
						code for the GP2Y0A21YK0F IR and PING sensors from the Arduino
						Playground, which I have modified to suit my needs.

					Copyright (C) 2013 Dale Weber <hybotics.pdx@gmail.com>.
*/

#include <Wire.h>
#include <Adafruit_LEDBackpack.h>
#include <Adafruit_GFX.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_L3GD20.h>
#include <Hybotics_LSM303DLHC_Unified.h>
#include <Hybotics_BMP180_Unified.h>
#include <Hybotics_10DOF_Unified.h>
#include <KalmanFilter.h>

#include <RTClib.h>

#include <BMSerial.h>
#include <RoboClaw.h>

/*
	Additional sensors
*/
#include <Adafruit_TCS34725.h>
#include <Adafruit_TMP006.h>

/*
	Additional libraries
*/

#include "Navigation4_Displays.h"
#include "Pitches.h"

/***************************************************************************
  This sketch uses the 10DOF IMU from Adafruit, which has a
    BMP180 Temperature/Barometric pressure sensor, a
    LMS303DLHC Three-axis accelerometer / Three-axis magnetometer (compass),
    and a L3GD20 Three-axis Gyroscope.
    
    Adafruit product http://www.adafruit.com/products/1604
***************************************************************************/

/*
    Global variables
*/

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

Hybotics_BMP180_Unified temperature = Hybotics_BMP180_Unified(10001);
Hybotics_LSM303DLHC_Accel_Unified accelerometer = Hybotics_LSM303DLHC_Accel_Unified(10002);
Hybotics_LSM303DLHC_Mag_Unified compass = Hybotics_LSM303DLHC_Mag_Unified(10003);
Adafruit_L3GD20 gyroscope;
Hybotics_10DOF_Unified imu = Hybotics_10DOF_Unified();

Adafruit_TCS34725 rgbColor = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
Adafruit_TMP006 heat = Adafruit_TMP006();
RTC_DS1307 clock;

/*
    Initialize global variables
*/

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

//  Support for multiple 7 segment displays
Adafruit_7segment sevenSeg[MAX_NUMBER_7SEG_DISPLAYS];

Adafruit_8x8matrix matrix8x8 = Adafruit_8x8matrix();

/*
	Setup all our serial devices
*/

//	Hardware Serial: Console and debug (replaces Serial.* routines)
BMSerial console(SERIAL_CONSOLE_RX_PIN, SERIAL_CONSOLE_TX_PIN);

//	Hardware Serial2: SSC-32 Servo Controller
BMSerial ssc32(SERIAL_SSC32_RX_PIN, SERIAL_SSC32_TX_PIN);

//	Hardware Serial3: RoboClaw 3x5 Motor Controller
RoboClaw roboClaw(SERIAL_ROBOCLAW_RX_PIN, SERIAL_ROBOCLAW_TX_PIN, 0, false);

//	This is a software serial port
BMSerial xbee(SERIAL_XBEE_RX_PIN, SERIAL_XBEE_TX_PIN);

//	We only have one RoboClaw 2x5 right now
uint8_t roboClawControllers = ROBOCLAW_CONTROLLERS - 1;
uint8_t	roboClawBaseAddress = ROBOCLAW_SERIAL_BASE_ADDR;
uint8_t roboClawAddress = ROBOCLAW_SERIAL_BASE_ADDR;
char *roboClawVersion;

//	RoboClaw 2x5 motor M1
Motor leftMotorM1;
//	RoboClaw 2x5 motor M2
Motor rightMotorM2;

/*
    Initialize servos
*/
Servo panServo = {
	SERVO_PAN_PIN,
	SERVO_PAN_ADJUST,
	0,
	0,
	SERVO_PAN_LEFT_MAX,
	SERVO_PAN_RIGHT_MAX,
	SERVO_MAX_DEGREES,
	0
};

Servo tiltServo = {
	SERVO_TILT_PIN,
	SERVO_TILT_ADJUST,
	0,
	0,
	SERVO_TILT_DOWN_MAX,
	SERVO_TILT_UP_MAX,
	SERVO_MAX_DEGREES,
	0
};

//  These are where the range sensor readings are stored.
int ping[MAX_NUMBER_PING];
float ir[MAX_NUMBER_IR];

ColorSensor colorData = {
	0,
	0,
	0,
	0,
	0
};

HeatSensor heatData = {
	0.0,
	0.0
};

/*
	Bitmaps for the drawBitMap() routine
*/
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

/********************************************************/
/*	Initialization routines								*/
/********************************************************/

/*
	Initialize displays

	Multiple 7 segment displays will be supported. The displays
		should be on the breadboard, starting at the right with
		the lowest addressed display and going to the left.

*/
void initDisplays (uint8_t totalDisplays) {
	uint8_t nrDisp = 0;
	uint8_t address;

	console.println("Initializing Displays..");

	while (nrDisp < totalDisplays) {
		sevenSeg[nrDisp] = Adafruit_7segment();
		address = SEVEN_SEG_BASE_ADDR + nrDisp;
		sevenSeg[nrDisp].begin(address);
		sevenSeg[nrDisp].setBrightness(5);
		sevenSeg[nrDisp].drawColon(false);

		nrDisp += 1;
	}

	/*
		The matrix display address is one higher than the last
			seven segment display, based on the number of seven
			seven segment displays that are configured.
	*/
	matrix8x8.begin(MATRIX_DISPLAY_ADDR);
	matrix8x8.setBrightness(5);
	matrix8x8.setRotation(3);
}

/*
	Set the Pan/Tilt to Home Position
*/
void initPanTilt (void) {
	console.println("Initializing Pan/Tilt");
  
	//  Put the front pan/tilt at home position
	moveServoPw(&panServo, SERVO_CENTER_MS, 0, 0, false);
	moveServoPw(&tiltServo, SERVO_CENTER_MS, 0, 0, true);
//	moveServoDegrees(&panS, moveDegrees, moveSpeed, moveTime, false);
//	moveServoDegrees(&tiltS, moveDegrees, moveSpeed, moveTime, true);
}

/*
	Initialize the RoboClaw 2x5 motor controller
*/
void initRoboClaw (uint8_t address, uint16_t bps, Motor *leftMotorM1, Motor *rightMotorM2) {
	console.print("Initializing the RoboClaw 2x5 Motor Controller at address ");
	console.print(address, HEX);
	console.print(", for ");
	console.print(bps);
	console.println(" Bps communication");

	roboClaw.begin(bps);

	//	Set the RoboClaw motor constants
	roboClaw.SetM1VelocityPID(address, ROBOCLAW_KD, ROBOCLAW_KP, ROBOCLAW_KI, ROBOCLAW_QPPS);
	roboClaw.SetM2VelocityPID(address, ROBOCLAW_KD, ROBOCLAW_KP, ROBOCLAW_KI, ROBOCLAW_QPPS);

	//	For R/C (PWM) modes
	leftMotorM1->pin = 0;
	leftMotorM1->pulseWidthMin = 1000;
	leftMotorM1->pulseWidthMax = 2000;
	leftMotorM1->pulseWidthOffset = 0;

	//	For Packet Serial modes
	leftMotorM1->encoder = 0;
	leftMotorM1->encoderStatus = 0;
	leftMotorM1->encoderValid = false;
	leftMotorM1->speed = 0;
	leftMotorM1->speedStatus = 0;
	leftMotorM1->speedValid = false;
	leftMotorM1->forward = true;
	leftMotorM1->distance = 0;
	leftMotorM1->distanceValid = false;		    

	//	For R/C (PWM) modes
	rightMotorM2->pin = 0;
	rightMotorM2->pulseWidthMin = 1000;
	rightMotorM2->pulseWidthMax = 2000;
	rightMotorM2->pulseWidthOffset = 0;

	//	For Packet Serial modes
	rightMotorM2->encoder = 0;
	rightMotorM2->encoderStatus = 0;
	rightMotorM2->encoderValid = false;
	rightMotorM2->speed = 0;
	rightMotorM2->speedStatus = 0;
	rightMotorM2->speedValid = false;
	rightMotorM2->forward = true;
	rightMotorM2->distance = 0;
	rightMotorM2->distanceValid = false;		    
}

/*
	Initialize sensors
*/
void initSensors (void) {
	console.println("Initializing Sensors..");

	//	Initialize the accelerometer
	console.println("     LSM303DLHC Accelerometer..");

	if (! accelerometer.begin()) {
		/* There was a problem detecting the LSM303DLHC ... check your connections */
		console.println("Ooops, no LSM303DLHC detected ... Check your wiring!");
		while(1);
	}

	console.println("     LSM303DLHC Magnetometer (Compass)..");

	//	Initialize the magnetometer (compass) sensor
	if (! compass.begin()) {
		/*	There was a problem detecting the LSM303DLHC ... check your connections */
		console.println("Ooops, no LSM303DLHC detected ... Check your wiring!");
		while(1);
	}

	console.println("     L3GD20 Gyroscope..");

	//	Initialize and warn if we couldn't detect the gyroscope chip
	if (! gyroscope.begin(gyroscope.L3DS20_RANGE_250DPS)) {
		console.println("Oops ... unable to initialize the L3GD20. Check your wiring!");
		while (1);
	}

	console.println("     10 DOF Inertial Measurement Unit..");

	imu.begin();

	console.println("     BMP180 Temperature/Pressure..");

	//	Initialize the BMP180 temperature sensor
	if (! temperature.begin()) {
		//  There was a problem detecting the BMP180 ... check your connections
		console.println("Ooops, no BMP180 detected ... Check your wiring or I2C ADDR!");
		while(1);
	}
	
	console.println("     TMP006 Heat..");

	//	Initialize the TMP006 heat sensor
	if (! heat.begin()) {
		console.println("There was a problem initializing the TMP006 heat sensor .. check your wiring or I2C ADDR!");
		while(1);
	}
	
	console.println("     TCS34725 RGB Color..");

	//	Initialize the TCS34725 color sensor
	if (! rgbColor.begin()) {
		console.println("There was a problem initializing the TCS34725 RGB color sensor .. check your wiring or I2C ADDR!");
		while(1);
	}

	console.println("     DS1307 Real Time Clock..");

	//	Check to be sure the RTC is running
//	if (! clock.isrunning()) {
//		console.println("The Real Time Clock is NOT running!");
//		while(1);
//	}
}
  
/*
    Write a floating point value to the 7-Segment display, such as the 0.56"
      4 digit displays with I2C backpacks, sold by Adafruit.

    Multiple 7 segment displays are supported automatically. You just have to
      set the number of displays in the IMU_Multi_Display.h and set the proper
      I2C addresses for the displays. The base address is 0x70, as shipped by
      Adafruit. Up to 8 of these displays are supported, with the 
      being
      highest addressed display farthest to the left, and decreasing addresses
      moving to the right. The lowest addressed (0x70) display has to be at the
      far right for this to work.
*/

/*
	Write a number (integer or floating point) to a 7-Segment display
*/
void writeNumber (uint8_t displayNr, uint16_t value, uint8_t decimal = 2, boolean noblank = false) {
  uint8_t digitCount = 1, temp = 0;
  boolean decimalPoint = false;

  temp = value / 100;
/*  
  console.print("(writeNumber) value = ");
  console.print(value);
  console.print(", temp = ");
  console.println(temp);
*/

  //	Set first digit of the integer portion
  if ((noblank) or (temp > 9)) {
/*    
    console.print("(writeNumber) digit = ");
    console.println(digit);
*/

    decimalPoint = ((digitCount) == decimal);
    sevenSeg[displayNr].writeDigitNum(0, int(temp / 10), decimalPoint);  //  Tens
  } else {
    sevenSeg[displayNr].clear();
  }

  //	Set the second digit of the integer portion
  digitCount += 1;
  decimalPoint = ((digitCount) == decimal);
  sevenSeg[displayNr].writeDigitNum(1, temp % 10, decimalPoint);         //  Ones

  //	Set the first digit of the decimal portion
  temp = value % 100;
  digitCount += 1;
  decimalPoint = ((digitCount) == decimal);
  sevenSeg[displayNr].writeDigitNum(3, int(temp / 10), decimalPoint);    //  Tens

  //	Set the second digit of the decimal portion
  digitCount += 1;
  decimalPoint = ((digitCount) == decimal);
  sevenSeg[displayNr].writeDigitNum(4, temp % 10, decimalPoint);         //  Ones
}

/*
	Clear all the seven segment and matrix displays
*/
void clearDisplays (void) {
	uint8_t nrDisp = 0;

	while (nrDisp < MAX_NUMBER_7SEG_DISPLAYS) {
		sevenSeg[nrDisp].clear();
		sevenSeg[nrDisp].drawColon(false);
		sevenSeg[nrDisp].writeDisplay();

		nrDisp += 1;
	}

	matrix8x8.clear();
	matrix8x8.writeDisplay();
}

/*
	Display the TCS34725 RGB color sensor readings
*/
void displayColorSensorReadings (ColorSensor *colorData) {
	console.print("Color Temperature: ");
	console.print(colorData->colorTemp, DEC);
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
	float objCelsius = heatData->objectTemp;
	float objFahrenheit = toFahrenheit(objCelsius);
	float dieCelsius = heatData->dieTemp;
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

/*
	Display the readings from the IMU (Accelerometer, Magnetometer [Compass], Gyroscope,
		and Orientation (if valid)
*/
void displayIMUReadings (sensors_event_t *accelEvent, sensors_event_t *compassEvent, sensors_vec_t *orientation, boolean pitchRollValid, boolean headingValid, boolean temperatureValid, float celsius, float fahrenheit, int gyroX, int gyroY, int gyroZ) {
	//	LMS303DLHC Accelerometer readings
	console.println("Accelerometer Readings:");
	console.print("X = ");
	console.print(accelEvent->acceleration.x);
	console.print(", Y = ");
	console.print(accelEvent->acceleration.y);
	console.print(", Z = ");
	console.println(accelEvent->acceleration.z);
	console.println();

	//	LMS303DLHC Magnetometer (Compass) readings
	console.println("Magnetometer (Compass) Readings:");
	console.print("X = ");
	console.print(compassEvent->magnetic.x);
	console.print(", Y = ");
	console.print(compassEvent->magnetic.y);
	console.print(", Z = ");
	console.println(compassEvent->magnetic.z);
	console.println();

	//	BMP180 Temperature readings
	if (temperatureValid) {
		console.print("Room Temperature = ");
		console.print(fahrenheit);
		console.print(" F, ");
		console.print(celsius);
		console.println(" C.");
		console.println();
	}

	//	L3DG20 Gyroscope readings
	console.println("Gyroscope Readings:");
	console.print("X = ");
	console.print(gyroX);
	console.print(", Y = ");
	console.print(gyroY);
	console.print(", Z = ");
	console.println(gyroZ);

	if (pitchRollValid || headingValid) {
		console.println("Orientation Readings:");
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
			console.print(", ");
		}

		console.print("Heading: ");
		console.println(orientation->heading);
	}

	console.println();
}

/*
	Display the data for a given motor
*/
void displayMotor (Motor *motor, String name, boolean serialPWM = true) {
	console.print(name);
	console.println(" Motor:");

	if (serialPWM) {
		//	Using Packet Serial
		console.print("Encoder is valid: ");

		if (motor->encoderValid) {
			console.print("Yes");
			console.print(", Status: ");
			console.print(motor->encoderStatus);
			console.print(", Value: ");
			console.println(motor->encoder);
		} else {
			console.println("No");
		}

		console.print("Speed is valid: ");

		if (motor->speedValid) {
			console.print("Yes");
			console.print(", Status: ");
			console.print(motor->speedStatus);
			console.print(", Speed: ");
			console.println(motor->speed);
		} else {
			console.println("No");
		}

		console.print("Moving ");

		if (motor->forward) {
			console.print("forward");   
		} else {
			console.print("in reverse");
		}

		console.print("Distance is valid: ");

		if (motor->distanceValid) {
			console.print("Yes");
			console.print(", Distance: ");
			console.println(motor->distance);
		} else {
			console.println("No");
		}
	} else {
		//	Using R/C PWM control
		console.print("Pin: ");
		console.print(motor->pin);
		console.print(", Minimum pulse: ");
		console.print(motor->pulseWidthMin);
		console.print(", Maximum pulse: ");
		console.print(motor->pulseWidthMax);
		console.print(", Offset: ");
		console.println(motor->pulseWidthOffset);
	}
}

/*
	Display data from the RoboClaw 2x5 motor controller
*/
void displayRoboClawEncoderSpeed (uint8_t address, Motor *leftMotorM1, Motor *rightMotorM2) {
	char version[32];

	roboClaw.ReadVersion(address, version);

	console.print("RoboClaw 2x5 status (version ");
	console.print(version);
	console.println("): ");

    if (leftMotorM1->encoderValid) {
		console.print("Left Motor Encoder = ");
		console.print(leftMotorM1->encoder, DEC);
		console.print(", Status =  ");
		console.print(leftMotorM1->encoderStatus, HEX);
		console.println();
	}

	if (leftMotorM1->speedValid) {
		console.print("Left Motor Speed = ");
		console.print(leftMotorM1->speed, DEC);
		console.println();
	}

	if (rightMotorM2->encoderValid) {
		console.print("Right Motor Encoder = ");
		console.print(rightMotorM2->encoder, DEC);
		console.print(", Status = ");
		console.print(rightMotorM2->encoderStatus, HEX);
		console.println();
	}

	if (rightMotorM2->speedValid) {
		console.print("Right Motor Speed = ");
		console.print(rightMotorM2->speed, DEC);
		console.println();
	}
	
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

/*
	Read current data from the RoboClaw 2x5 Motor Controller
*/
uint16_t readRoboClaw (uint8_t address, Motor *leftMotorM1, Motor *rightMotorM2) {
	uint16_t error = 0;
	bool valid;
	uint8_t status;

	console.println("Reading Left Encoder..");
	leftMotorM1->encoder = roboClaw.ReadEncM1(address, &status, &valid);
	leftMotorM1->encoderStatus = status;
	leftMotorM1->encoderValid = valid;

	console.println("Reading Left Speed..");
	leftMotorM1->speed = roboClaw.ReadSpeedM1(address, &status, &valid);
	leftMotorM1->speedStatus = status;
	leftMotorM1->speedValid = valid;

	console.println("Reading Right Encoder..");
	rightMotorM2->encoder = roboClaw.ReadEncM2(address, &status, &valid);
	rightMotorM2->encoderStatus = status;
	rightMotorM2->encoderValid = valid;

	console.println("Reading Right Speed..");
	rightMotorM2->speed = roboClaw.ReadSpeedM2(address, &status, &valid);
	rightMotorM2->speedStatus = status;
	rightMotorM2->speedValid = valid;

	return error;
}

/********************************************************************/
/*	The following routines deal with the SSC-32 servo controller	*/
/********************************************************************/

/*
    Move a servo by pulse width in ms (500ms - 2500ms) - Modified to use HardwareSerial2()
*/
void moveServoPw (Servo *servo, int servoPosition, int moveSpeed, int moveTime, boolean term) {
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
		servo->error = 200;
	}
 
	if (servo->error == 0) {
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
}

/*
    Move a servo by degrees (-90 to 90) or (0 - 180) - Modified to use BMSerial
*/
void moveServoDegrees (Servo *servo, int servoDegrees, int moveSpeed, int moveTime, boolean term) {
	int servoPulse = SERVO_CENTER_MS + servo->offset;

	servo->error = 0;
  
	//  Convert degrees to ms for the servo move
	if (servo->maxDegrees == 90) {
		servoPulse = (SERVO_CENTER_MS + servo->offset) + (servoDegrees * 10);
	} else if (servo->maxDegrees == 180) {
		servoPulse = (SERVO_CENTER_MS + servo->offset) + ((servoDegrees - 90) * 10);
	}

	if ((servoPulse >= servo->minPulse) && (servoPulse <= servo->maxPulse)) {
		ssc32.print("#");
		ssc32.print(servo->pin);
		ssc32.print(" P");
		ssc32.print(servoPulse);

		servo->msPulse = (servoDegrees * 10) + SERVO_CENTER_MS;
		servo->angle = servoDegrees;
    
		if (servo->maxDegrees == 180) {
			servo->angle += 90;
		}
	} else if ((servoPulse < servo->minPulse) || (servoPulse > servo->maxPulse)) {
		servo->error = 200;
	}
  
	if (servo->error == 0) {
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
}

/********************************************/
/*	End of SSC-32 servo controller routines	*/
/********************************************/

/*
    Process error conditions
*/
void processError (byte err) {
	console.print("Error: ");
	console.print(err);
	console.println("");
}

/*
	Test all the displays
*/
void testDisplays (uint8_t totalDisplays) {
	uint8_t nrDisp = 0;

	console.println("Testing All Displays");

	while (nrDisp < totalDisplays) {
		sevenSeg[nrDisp].print(8888);
		sevenSeg[nrDisp].drawColon(true);
		sevenSeg[nrDisp].writeDisplay();

		nrDisp += 1;
	}

	matrix8x8.drawBitmap(0, 0, allon_bmp, 8, 8, LED_ON);
	matrix8x8.writeDisplay();

	delay(2000);

	clearDisplays();
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
	console.print("W.A.L.T.E.R. 2.0 Navigation, version ");
	console.print(BUILD_VERSION);
	console.print(" on ");
	console.print(BUILD_DATE);
	console.print(" for the ");
	console.print(BUILD_BOARD);
	console.println(" board");

	console.println();
	console.println("Initializing Serial Ports..");

	//	Initialize the SSC-32 servo controller port
	ssc32.begin(115200);

	//	Initialize the XBee communication port (BMSerial)
	xbee.begin(115200);

	console.println("Initializing Digital Pins..");

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

	//	Initialize the displays
	initDisplays(MAX_NUMBER_7SEG_DISPLAYS);

	//	Test the displays
	testDisplays(MAX_NUMBER_7SEG_DISPLAYS);

	//	Initialize all sensors
	initSensors();

	//	Initialize the RoboClaw 2x5 motor controller port
	initRoboClaw(roboClawAddress, 38400, &leftMotorM1, &rightMotorM2);

	//	Set the Pan/Tilt to home position
	initPanTilt();

	console.println();
}

/*
	Runs forever
*/
void loop (void) {
	//	The current date and time from the DS1307 real time clock
	DateTime now = clock.now();

	//	Display related variables
	boolean amTime, pitchRollValid = false, headingValid = false;
	uint8_t displayNr = 0, count = 0;
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

	clearDisplays();

	//  Display the date, if it's time
	if (displayDate && DISPLAY_INFORMATION) {
		console.println("Displaying the date (month/day)..");
		displayInt = (now.month() * 100) + now.day();  

		//  Month and day
		writeNumber(0, displayInt, 0, true);
		matrix8x8.drawBitmap(0, 0, date_bmp, 8, 8, LED_ON);

		sevenSeg[0].writeDisplay();
		matrix8x8.writeDisplay();

		delay(5000);

		sevenSeg[0].clear();
		matrix8x8.clear();  

		//  Year
		console.println("Displaying the date (year)..");
		writeNumber(0, now.year(), 0, false);
		matrix8x8.drawBitmap(0, 0, year_bmp, 8, 8, LED_ON);

		sevenSeg[0].writeDisplay();
		matrix8x8.writeDisplay();

		delay(5000);
    
		dateMinuteCount = 0;
		clearDisplays();
	}
  
	if (displayTime && DISPLAY_INFORMATION) {
		console.println("Displaying the time..");

		if (currentHour > 12) {
			amTime = false;
			currentHour = currentHour - 12;
		} else {
			amTime = true;
		}
	  
		displayInt = (currentHour * 100) + now.minute();  

		//  Display the current time on the 7 segment display
		writeNumber(0, displayInt, 0, false);
		sevenSeg[0].drawColon(true);
	  
		matrix8x8.clear();
	  
		if (amTime) {
			matrix8x8.drawBitmap(0, 0, am_bmp, 8, 8, LED_ON);
		} else {
			matrix8x8.drawBitmap(0, 0, pm_bmp, 8, 8, LED_ON);
		}
	  
		sevenSeg[0].writeDisplay();
		matrix8x8.writeDisplay();
	  
		delay(5000);

		timeMinuteCount = 0;
		clearDisplays();
	}

	console.println("Getting Accelerometer Readings..");

	/*
		Get accelerometer readings
	*/
	accelerometer.getEvent(&accelEvent);
 
	accelX = accelEvent.acceleration.x;
	accelY = accelEvent.acceleration.y;
	accelZ = accelEvent.acceleration.z;

	/*
		Get compass readings
	*/
	console.println("Getting Magnetometer (Compass) Readings..");

	compass.getEvent(&compassEvent);

	compassX = compassEvent.magnetic.x;
	compassY = compassEvent.magnetic.y;
	compassZ = compassEvent.magnetic.z;

	/*
		Get gyro readings
	*/
	console.println("Getting Gyroscope Readings..");

	gyroscope.read();

	gyroX = (int)gyroscope.data.x;
	gyroY = (int)gyroscope.data.y;
	gyroZ = (int)gyroscope.data.z;

	console.println("Getting Orientation readings..");

	/*
		Get pitch, roll, and heading information
	*/

	//	Calculate pitch and roll from the raw accelerometer data
	pitchRollValid = imu.accelGetOrientation(&accelEvent, &orientation);

	//	Calculate the heading using the magnetometer (compass)
	headingValid = imu.magGetOrientation(SENSOR_AXIS_Z, &compassEvent, &orientation);

	/*
		Put accelerometer and Gyro reactive behaviors HERE
	*/
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

	/*
		Read the RoboClaw 2x5 motor controller encoders
	*/
	console.println("Reading RoboClaw..");

	error = readRoboClaw (roboClawAddress, &leftMotorM1, &rightMotorM2);

	if (error != 0) {
		processError(error);
	} else {
		displayRoboClawEncoderSpeed(roboClawAddress, &leftMotorM1, &rightMotorM2);
	}

	console.println("Getting Temperature and Pressure readings..");

	//	Get a new sensor event
	temperature.getEvent(&tempEvent);

	if (tempEvent.pressure) {
		console.println("Temperature and Pressure are valid..");
	} else {
		console.println("Temperature and Pressure are NOT valid..");
	}
  
	if (tempEvent.pressure) {
		// Calculating altitude with reasonable accuracy requires pressure
		// sea level pressure for your position at the moment the data is
		// converted, as well as the ambient temperature in degress
		// celcius.  If you don't have these values, a 'generic' value of
		// 1013.25 hPa can be used (defined as SENSORS_PRESSURE_SEALEVELHPA
		// in sensors.h), but this isn't ideal and will give variable
		// results from one day to the next.
		//
		// You can usually find the current SLP value by looking at weather
		// websites or from environmental information centers near any major
		// airport.
		//
		// For example, for Paris, France you can check the current mean
		// pressure and sea level at: http://bit.ly/16Au8ol

		//  First we get the current temperature from the BMP180 in celsius and fahrenheit
		temperature.getTemperature(&celsius);
		fahrenheit = toFahrenheit(celsius);

		//	Convert the atmospheric pressure, SLP and temp to altitude in meters
		altitude = temperature.pressureToAltitude(seaLevelPressure, tempEvent.pressure, celsius); 

		if (displayTemperature && DISPLAY_INFORMATION) {
			//  Display the temperature in Fahrenheit
			console.println("Displaying temperature..");

			writeNumber(0, int(fahrenheit * 100), 2, false);
			sevenSeg[0].writeDisplay();

			matrix8x8.clear();
			matrix8x8.drawBitmap(0, 0, f_bmp, 8, 8, LED_ON);
			matrix8x8.writeDisplay();

			delay(5000);

			//  Display the temperature in Celsius
			writeNumber(0, int(celsius * 100), 2, false);
			sevenSeg[0].writeDisplay();

			matrix8x8.clear();
			matrix8x8.drawBitmap(0, 0, c_bmp, 8, 8, LED_ON);
			matrix8x8.writeDisplay();

			delay(5000);
			temperatureMinuteCount = 0;
			clearDisplays();
		}
	}

	displayIMUReadings (&accelEvent, &compassEvent, &orientation, pitchRollValid, headingValid, tempEvent.pressure, celsius, fahrenheit, gyroX, gyroY, gyroZ);

	/*
		Read the TCS34725 RGB color sensor
	*/
	rgbColor.getRawData(&colorData.red, &colorData.green, &colorData.blue, &colorData.c);
	colorData.colorTemp = rgbColor.calculateColorTemperature(colorData.red, colorData.green, colorData.blue);
	colorData.lux = rgbColor.calculateLux(colorData.red, colorData.green, colorData.blue);

	displayColorSensorReadings(&colorData);
                    	
	/*
		Read the TMP006 heat sensor
	*/
	heatData.dieTemp = heat.readDieTempC();
	heatData.objectTemp = heat.readObjTempC();
	
	displayHeatSensorReadings(&heatData);

	console.println();
                                   
	if (error != 0) {
		processError(error);
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
		Delay to allow time to read the Serial Monitor information log
	*/
	console.print("Waiting");

	for (count = 0; count < LOOP_DELAY_SECONDS; count++) {
		console.print(".");
		delay(1000);
	}

	console.println();
	console.println();
}
