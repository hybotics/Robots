/*
	Program:	IMU_Multi_Display_Test.ino - Inertial Measurement Unit testing, 
					with multiple 7 segment display support.

	Date:		07-Feb-2014
	Version:	0.1.7t ALPHA

	Purpose:	To allow experimentation and testing with various IMUs, including
					the Adafruit 10 DOF IMU with BMP180 temperature/preasure, LMS303DLHC
					3-axis accelerometer/3-axis Magnetometer (compass), and L3GD20
					3-axis Gyro that I'm tinkering with now.
									
					This sketch will support multiple 7 segment displays using I2C
						backpacks ( http://www.adafruit.com/products/1427 ) from Adafruit.
									
					These are the 1.2" four digit 7 segment displays:
						http://www.adafruit.com/products/878 (Red)
						http://www.adafruit.com/products/879 (Yellow)
						http://www.adafruit.com/products/880 (Green)
						http://www.adafruit.com/products/881 (Blue)
						http://www.adafruit.com/products/1002 (White)

					-----------------------------------------------------------------------
					v0.1.6 ALPHA 24-Jan-2014
					Merged appropriate code from Navigation3_Displays.ino - I feel a library
						coming on here.

					------------------------------------------------------------------------
					v0.1.7 ALPHA 26-Jan-2014
					Added the Adafruit_10DOF_Unified library to get orientation information - pitch, roll,
						and heading from the raw accelerometer and magnetometer (compass) data

					------------------------------------------------------------------------
					v0.1.7t ALPHA 07-Feb-2014
					Converted to run on a Teensy3.1
					I removed the 10DOF stuff, because there is some sort of unknown problem with it,
						which I will investigate later. Compiles clean now. The next step is to connect the
						Teensy3.1 up to the I2C display and sensor circuit and see what, if anything, works
						now.

					--------------------------------------------------------------------------

	Dependencies:	Adafruit libraries:
						LSM303DLHC, L3GD20, TMP006, TCS34727, RTClib for the DS1307

					Hybotics libraries:
						BMP180 (modified from Adafruit's BMP085 library)

	Comments:     Credit is given, where applicable, for code I did not originate.
						This sketch started out as an Adafruit tutorial for the electret
						microphones being used for sound detection. I've also pulled
						code for the GP2D12 IR and PING sensors from the Arduino
						Playground, which I have modified to suit my needs.

					Copyright (C) 2013 Dale Weber <hybotics.pdx@gmail.com>.
*/

#include <Wire.h>
#include <Adafruit_LEDBackpack.h>
#include <Adafruit_GFX.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BMP180_Unified.h>
#include <Adafruit_LSM303DLHC_Unified.h>
#include <Adafruit_L3GD20.h>

#include <KalmanFilter.h>

/*
	Additional sensors
*/
#include <Adafruit_TCS34725.h>
#include <Adafruit_TMP006.h>

#include <RTClib.h>
//#include <BMSerial.h>
//#include <RoboClaw.h>

#include "IMU_Multi_Display_Test.h"

/************************************************************
	This sketch uses the 10DOF IMU from Adafruit, which has a
		BMP180 Temperature/Barometric pressure sensor, a
		LMS303 Three-axis accelerometer, and a
		L3GD20 Gyro.
		
		Adafruit product http://www.adafruit.com/products/1604
*************************************************************/

/*
		Global variables
*/

//	Hardware Serial console (replaces console.* routines)
HardwareSerial console = HardwareSerial();

//  Support for multiple 7 segment displays
Adafruit_7segment sevenSeg[MAX_NUMBER_7SEG_DISPLAYS];

Adafruit_8x8matrix matrix8x8 = Adafruit_8x8matrix();

Adafruit_BMP180_Unified temperature = Adafruit_BMP180_Unified(10001);
Adafruit_LSM303_Accel_Unified accelerometer = Adafruit_LSM303_Accel_Unified(10002);
Adafruit_LSM303_Mag_Unified compass = Adafruit_LSM303_Mag_Unified(10003);
Adafruit_L3GD20 gyro;

Adafruit_TCS34725 rgbColor = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
Adafruit_TMP006 heat = Adafruit_TMP006();
RTC_DS1307 clock;

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

ColorSensor colorData = {
	0,
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
	Convert the Celsius temperature to Fahrenheit
*/
float tempFahrenheit (float celsius) {
	return (celsius * 1.8) + 32;
}

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
  
	console.println("------------------------------------");
	console.println("IR Sensor readings");
	console.println("------------------------------------");

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
  
	console.println("------------------------------------");
	console.println("PING Ultrasonic Sensor readings");
	console.println("------------------------------------");
  
	//	Display PING sensor readings (cm)
	while (sensorNr < MAX_NUMBER_PING) {
		console.print("Ping #");
		console.print(sensorNr + 1);
		console.print(" range = ");
		console.print(ping[sensorNr]);
		console.println(" cm");

		sensorNr += 1;
	}
 
	console.println("");
}

/*
	Display the readings from the IMU (Accelerometer, Magnetometer [Compass], and Gyro
*/
void displayIMUReadings (sensors_event_t *accelEvent, sensors_event_t *compassEvent, sensors_vec_t *orientation, float celsius, float fahrenheit, int gyroX, int gyroY, int gyroZ) {
	//	Accelerometer readings
	console.print("Accelerometer readings:");
	console.print("X = ");
	console.print(accelEvent->acceleration.x);
	console.print(", Y = ");
	console.print(accelEvent->acceleration.y);
	console.print(", Z = ");
	console.println(accelEvent->acceleration.z);

	//	Magnetometer (Compass) readings
	console.print("Magnetometer (Compass) readings:")
	console.print("X = ");
	console.print(compassEvent->magnetic.x);
	console.print(", Y = ");
	console.print(compassEvent->magnetic.y);
	console.print(", Z = ");
	console.println(compassEvent->magnetic.z);

	//	Gyro readings
	console.print("Gyroscope readings:");
	console.print("X = ");
	console.print(gyroX);
	console.print(", Y = ");
	console.print(gyroY);
	console.print(", Z = ");
	console.println(gyroZ);

	//	Temperature readings
	console.print("Temperature and Preasure readings:")
	console.print("Room Temperature = ");
	console.print(fahrenheit);
	console.print(" F, ");
	console.print(celsius);
	console.print(" C.");

	//	Orientation readings
	console.println("Orientation readings:");
	console.print("Roll = ");
	console.print(orientation->roll);
	console.print(", Pitch = ");
	console.print(orientation->pitch);
	console.print(", Heading = ");
	console.println(orientation->heading);
	
	console.println();
}

/*
	Display data from the RoboClaw 2x5 motor controller
*/
void displayRoboClawEncoderSpeed (uint8_t address, Motor *leftMotorM1, Motor *rightMotorM2) {
	char *version;

	roboClaw.ReadVersion(address, version);

	console.print("RoboClaw 2x5 status (version ");
	console.print(version);
	console.println("): ");
	console.println();

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
	
	console.println("");
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
	console.print(colorData->c, DEC);
	console.println();
}

/*
	Display the TMP006 heat sensor readings
*/
void displayHeatSensorReadings (HeatSensor *heatData) {
	console.print("Object Temperature: ");
	console.print(heatData->objectTemp);
	console.println(" C");
	console.print("Die Temperature: ");
	console.print(heatData->dieTemp);
	console.println(" C");
}

/********************************************************/
/*  Initialization routines               */
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

	//  This is not needed for the mini 8x8 matrix displays
	matrix8x8.setRotation(3);
}

/*
	Initialize all sensors
*/
void initSensors (void) {
	console.println("Initializing Sensors..");

	//  Initialize the accelerometer
	console.println("     LSM303 Accelerometer..");

	if (! accelerometer.begin()) {
		/* There was a problem detecting the LSM303 ... check your connections */
		console.println("Ooops, no LSM303 detected ... Check your wiring!");
		while(1);
	}

	console.println("     LSM303 Magnetometer (Compass)..");

	//  Initialize the magnetometer (compass) sensor
	if (! compass.begin()) {
		/*  There was a problem detecting the LSM303 ... check your connections */
		console.println("Ooops, no LSM303 detected ... Check your wiring!");
		while(1);
	}

	console.println("     L3GD20 Gyroscope..");

	//  Initialize and warn if we couldn't detect the gyroscope chip
	if (! gyro.begin(gyro.L3DS20_RANGE_250DPS)) {
		console.println("Oops ... unable to initialize the L3GD20. Check your wiring!");
		while (1);
	}

	console.println("     BMP180 Temperature/Pressure..");

	//  Initialize the BMP180 temperature sensor
	if (! temperature.begin()) {
		//  There was a problem detecting the BMP180 ... check your connections
		console.println("Ooops, no BMP180 detected ... Check your wiring or I2C ADDR!");
		while(1);
	}
	
	console.println("     TMP006 Heat..");

	//  Initialize the TMP006 heat sensor
	if (! heat.begin()) {
		console.println("There was a problem initializing the TMP006 heat sensor .. check your wiring or I2C ADDR!");
		while(1);
	}
	
	console.println("     TCS34725 RGB Color..");

	//  Initialize the TCS34725 color sensor
	if (! rgbColor.begin()) {
		console.println("There was a problem initializing the TCS34725 RGB color sensor .. check your wiring or I2C ADDR!");
		while(1);
	}

	console.println("     DS1307 Real Time Clock..");

	//  Check to be sure the RTC is running
	if (! clock.isrunning()) {
		console.println("The Real Time Clock is NOT running!");
		while(1);
	}
}
	
/*
	Write a floating point value to the 7-Segment display, such as the 0.56"
		4 digit displays with I2C backpacks, sold by Adafruit.

	Multiple 7 segment displays are supported automatically. You just have to
		set the number of displays in the IMU_Multi_Display.h and set the proper
		I2C addresses for the displays. The base address is 0x70, as shipped by
		Adafruit. Up to 8 of these displays are supported, with the setup being
		highest addressed display farthest to the left, and decreasing addresses
		moving to the right. The lowest addressed (0x70) display has to be at the
		far right for this to work.
*/

/*
boolean writeFloat (uint8_t displayNr, double value, uint8_t decimal = 2, uint8_t nrDisplayDigits = 4, boolean noblank = false) {
	boolean exitStatus = true;
	uint8_t nrDisplays = 1;

	long integerPart = 0, decimalPart = 0;
	double sign = 1.0, newValue = 0.0;
	uint8_t digitCount = 1, temp = 0, dotPosition = 0, totalDigits = 0;
	uint8_t intPartLen = 0, decPartLen = 0, valueLen = 0, nrDigits = 0; 
	boolean decimalPoint = false;
	String decPartStr, intPartStr, valueStr;
	
	//	Store the sign
	if (value < 0) {
		sign = -1.0;
	} else {
		sign = 1.0;
	}

	newValue = abs(value);

	valueStr = String(newValue, DEC);  
	valueLen = valueStr.length();
	dotPosition = valueStr.indexOf(".");

	intPartStr = valueStr.substring(0, dotPosition);
	intPartLen = intPartStr.length();
//  integerPart = int(value);

	decPartStr = trimTrailingZeros(valueStr.substring(dotPosition + 1));
	decPartLen = decPartStr.length();
//  decimalPart = value - integerPart;
	
	console.print("(writeFloat) Integer part string = '");
	console.print(intPartStr);
	console.print("' (");
	console.print(intPartLen);
	console.println(")");

	console.print("Decimal part string = '");
	console.print(decPartStr);
	console.print("' (");
	console.print(decPartLen);
	console.println(")");

	console.print("sign = ");
	console.println(sign);

	console.print("(writeFloat) value string = '");
	console.print(valueStr);
	console.print("' (");
	console.print(valueLen);
	console.println(")");

	//	Find out how many digits we have to display
	totalDigits = intPartLen + decPartLen;
	
	if (sign < 0) {
		totalDigits += 1;
	}

	//	Check to be sure we can display the entire value
	if ((totalDigits > nrDisplayDigits) || (totalDigits > (NUMBER_DISPLAYS * 4))) {
		exitStatus = false;
	}

	if (exitStatus) {
		nrDisplays = totalDigits / 4;

		temp = value / 100;
 
		console.print("(writeNumber) value = ");
		console.print(value);
		console.print(", temp = ");
		console.println(temp);

		//	Set first digit of the integer portion
		if ((noblank) or (temp > 9)) {
			decimalPoint = ((digitCount) == decimal);
			sevenSeg[0].writeDigitNum(0, int(temp / 10), decimalPoint);  //  Tens
		} else {
			sevenSeg[0].clear();
		}

		//	Set the second digit of the integer portion
		digitCount += 1;
		decimalPoint = ((digitCount) == decimal);
		sevenSeg[0].writeDigitNum(1, temp % 10, decimalPoint);         //  Ones

		//	Set the first digit of the decimal portion
		temp = int(value / 100);
		
		digitCount += 1;
		decimalPoint = ((digitCount) == decimal);
		sevenSeg[0].writeDigitNum(3, int(temp / 10), decimalPoint);    //  Tens

		//	Set the second digit of the decimal portion
		digitCount += 1;
		decimalPoint = ((digitCount) == decimal);
		sevenSeg[0].writeDigitNum(4, temp % 10, decimalPoint);         //  Ones
	}
	
	return exitStatus;
}
*/

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

void setup () {
	//  Start up the Wire (I2C)
	Wire.begin();
	
	//  Initialize serial port communication
	console.begin(115200);
	console.println("IMU Time/Temperature Test");
	
	//  Setup and turn off the Color sensor's LED
	pinMode(COLOR_SENSOR_LED, OUTPUT);
	digitalWrite(COLOR_SENSOR_LED, LOW);
	delay(250);
	digitalWrite(COLOR_SENSOR_LED, HIGH);
	delay(250);
	digitalWrite(COLOR_SENSOR_LED, LOW);

	initDisplays(MAX_NUMBER_7SEG_DISPLAYS);

	testDisplays(MAX_NUMBER_7SEG_DISPLAYS);

	initSensors();
}

void loop () {
	boolean amTime;
	uint8_t displayNr = 0;
	DateTime now = clock.now();
	
	uint8_t hour = now.hour(), nrDisplays = 0;
	uint16_t displayInt;

	//	IMU variables
	float celsius, fahrenheit, altitude;
	float accelX, accelY, accelZ;
	float compassX, compassY, compassZ;
	float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;

	int gyroX, gyroY, gyroZ;

	sensors_event_t accelEvent, compassEvent, tempEvent;
	sensors_vec_t orientation;

/*
	String displayString;
	String timeString;
	String currMonth = leftZeroPadString(String(now.month()), 2);
	String currDay = leftZeroPadString(String(now.day()), 2);
	String currYear = leftZeroPadString(String(now.year()), 4);
	String currMinute = leftZeroPadString(String(now.minute()), 2);
	String currSecond = leftZeroPadString(String(now.second()), 2);
*/

	//  Clear the displays
	for (displayNr = 0; displayNr < MAX_NUMBER_7SEG_DISPLAYS; displayNr ++) {
		sevenSeg[displayNr].clear();
		sevenSeg[displayNr].drawColon(false);
	}

	matrix8x8.clear();

/*
	console.print("Month = ");
	console.print(now.month());
	console.print(", Day = ");
	console.print(now.day());
	console.print(", Year = ");
	console.println(now.year());
*/

/*  
	console.print("displayInt = ");
	console.println(displayInt);
*/  

	//	Display the date, if it's time
	if (displayDate) {
		displayInt = (now.month() * 100) + now.day();  

		//	Month and day
		writeNumber(0, displayInt, 0, true);
		matrix8x8.drawBitmap(0, 0, date_bmp, 8, 8, LED_ON);

		sevenSeg[0].writeDisplay();
		matrix8x8.writeDisplay();

		delay(5000);

		sevenSeg[0].clear();
		matrix8x8.clear();  

		//	Year
		writeNumber(0, now.year(), 0, false);
		matrix8x8.drawBitmap(0, 0, year_bmp, 8, 8, LED_ON);

		sevenSeg[0].writeDisplay();
		matrix8x8.writeDisplay();

		delay(5000);
		
		minuteCount = 0;
	}
	
	sevenSeg[0].clear();
	matrix8x8.clear();  

	/*
		Display the time
	*/

	if (hour > 12) {
		amTime = false;
		hour = hour - 12;
	} else {
		amTime = true;
	}
	
	displayInt = (hour * 100) + now.minute();  

	//	Display the current time on the 7 segment display
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
	
	delay(45000);

	sevenSeg[0].drawColon(false);

/*
	//	Display the current time
	console.print("Current date: ");
	console.print(currMonth);
	console.print('/');
	console.print(currDay);
	console.print('/');
	console.println(currYear);
	
	console.print("Current time: (");
	console.print(timeString);
	console.print(") ");
	console.print(currHour);
	console.print(':');
	console.print(currMinute);
	console.print(':');
	console.print(currSecond);
	console.print(" ");
	console.println(amPM);
	console.println();
*/

	/*
		Get Accelerometer, Magnetometer (Compass), and Gyro readings
	*/
	accelerometer.getEvent(&accelEvent);
 
	accelX = accelEvent.acceleration.x;
	accelY = accelEvent.acceleration.y;
	accelZ = accelEvent.acceleration.z;

	/*
		Get compass readings
	*/
	compass.getEvent(&compassEvent);

	compassX = compassEvent.magnetic.x;
	compassY = compassEvent.magnetic.y;
	compassZ = compassEvent.magnetic.z;

	/*
		Get gyro readings
	*/
	gyroscope.read();

	gyroX = (int)gyroscope.data.x;
	gyroY = (int)gyroscope.data.y;
	gyroZ = (int)gyroscope.data.z;

	/*
		Get pitch, roll, and heading information
	*/

	//	Calculate pitch and roll from the raw accelerometer data
	if (imu.accelGetOrientation(&accelEvent, &orientation)) {
		//	'orientation' should have valid .roll and .pitch fields
		console.print(F("Roll: "));
		console.print(orientation.roll);
		console.print(F("; "));
		console.print(F("Pitch: "));
		console.print(orientation.pitch);
		console.println(F("; "));
	}

	//	Calculate the heading using the magnetometer (compass)
	if (imu.magGetOrientation(SENSOR_AXIS_Z, &compassEvent, &orientation)) {
		//	'orientation' should have valid .heading data now
		console.print(F("Heading: "));
		console.print(orientation.heading);
		console.println(F("; "));
	}

	displayIMUReadings(&accelEvent, &compassEvent, &orientation, celsius, fahrenheit, gyroX, gyroY, gyroZ) {

	/*
		Get readings from the BMP180 Temperature/Preasure sensor.
	*/

	//	Get a new sensor event */ 
	temperature.getEvent(&tempEvent);
	
	//	Display the barometric pressure in hPa
	if (tempEvent.pressure) {
		/*
			Calculating altitude with reasonable accuracy requires pressure
				sea level pressure for your position at the moment the data is
		 		converted, as well as the ambient temperature in degress
		 		celcius.  If you don't have these values, a 'generic' value of 
		 		1013.25 hPa can be used (defined as SENSORS_PRESSURE_SEALEVELHPA
		 		in sensors.h), but this isn't ideal and will give variable
		 		results from one day to the next.

			You can usually find the current SLP value by looking at weather
				websites or from environmental information centers near any major
		 		airport.

			For example, for Paris, France you can check the current mean
		 		pressure and sea level at: http://bit.ly/16Au8ol

		//	First we get the current temperature from the BMP180 in celsius and fahrenheit
		temperature.getTemperature(&celsius);
		fahrenheit = tempFahrenheit(celsius);

		//	Convert the atmospheric pressure, SLP and temp to altitude in meters
		altitude = temperature.pressureToAltitude(seaLevelPressure, tempEvent.pressure, celsius); 
/*
		//	Display the barometric pressure in hPa
		sevenSeg.print(event.pressure);
		sevenSeg.writeDisplay();

		matrix8x8.clear();
		matrix8x8.drawBitmap(0, 0, hpa_bmp, 8, 8, LED_ON);
		matrix8x8.writeDisplay();

		delay(5000);
*/

		//	Display the temperature in Fahrenheit
		writeNumber(0, int(fahrenheit * 100), 2, false);
		sevenSeg[0].writeDisplay();

		matrix8x8.clear();
		matrix8x8.drawBitmap(0, 0, f_bmp, 8, 8, LED_ON);
		matrix8x8.writeDisplay();

		delay(7500);

		//	Display the temperature in Celsius
		writeNumber(0, int(celsius * 100), 2, false);
		sevenSeg[0].writeDisplay();

		matrix8x8.clear();
		matrix8x8.drawBitmap(0, 0, c_bmp, 8, 8, LED_ON);
		matrix8x8.writeDisplay();

/*    
		delay(5000);
		
		//	Display altitude in meters
		sevenSeg.print(altitude);
		sevenSeg.writeDisplay();

		matrix8x8.clear();
		matrix8x8.drawBitmap(0, 0, m_bmp, 8, 8, LED_ON);
		matrix8x8.writeDisplay();

		for (int8_t x=7; x>=-36; x--) {
			matrix8x8.clear();
			matrix8x8.setCursor(x,0);
			matrix8x8.print("m");
			matrix8x8.writeDisplay();
			delay(100);
		}

		//	Display atmospheric pressue in hPa
		console.print("Pressure:    ");
		console.print(event.pressure);
		console.println(" hPa");
		
		//	Display the temperature in Celsius and Fahrenheit
		console.print("Temperature: ");
		console.print(celsius);
		console.print(" C, ");
		console.print(fahrenheit);
		console.println(" F");
		
		//	Display our altitude in meters
		console.print("Altitude:    ");
		console.print(altitude); 
		console.println(" m");
		console.println("");
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
		minuteCount += 1;
		displayDate = (minuteCount == dateDisplayFreq);
		
		console.println();
		delay(7500);
	}
}
