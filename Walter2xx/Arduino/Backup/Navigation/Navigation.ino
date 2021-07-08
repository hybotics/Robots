/*
	Program:      	W.A.L.T.E.R. 2.0, Main navigation and reactive behaviors
	Date:         	13-Jan-2014
	Version:      	0.1.4 ALPHA

	Purpose:      	Added new displaySoundDirection() routine. I've started to add some
						error handling code, as well as code to handle Wire (I2C) slave
						operation.

	Dependencies:	Adafruit libraries:
                  		LSM303DLHC, L3GD20, TMP006, TCS34727, RTClib for the DS1307

					Hybotics libraries:
						BMP180 (modified from Adafruit's BMP085 library)

	Comments:		Credit is given, where applicable, for code I did not originate.
						This sketch started out as an Adafruit tutorial for the electret
						microphones being used for sound detection. I've also pulled
						code for the GP2D12 IR and PING sensors from the Arduino
						Playground, which I have modified to suit my needs.

					Copyright (C) 2013 Dale Weber <hybotics.pdx@gmail.com>.
*/

#include <Wire.h>

#include <Adafruit_GFX.h>
#include <Adafruit_LEDBackpack.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BMP180_Unified.h>
#include <Adafruit_LSM303DLHC_Unified.h>
#include <Adafruit_L3GD20.h>

#include <KalmanFilter.h>

#include <RTClib.h>
#include <RoboClaw.h>

/*
	Additional sensors
*/
#include <Adafruit_TCS34725.h>
#include <Adafruit_TMP006.h>

/*
	Additional libraries
*/
#include <SoftI2CMaster.h>

#include "Navigation.h"

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
				An L3GD20 Gyro
				An LSM303 3-Axis accelerometer / magnetometer (compass)

		These are also from Adafruit:
			http://www.adafruit.com/products/1334 (TCS34725 RGB Color sensor)
			http://www.adafruit.com/products/1296 (TMP006 Heat sensor)
			http://www.adafruit.com/products/264 (DS1307 Realtime Clock)

		From other sources:
			GP2D12 IR Ranging sensors (3)
			PING Ultrasonic Ranging sensors (3)
*/

Adafruit_BMP180_Unified temperature = Adafruit_BMP180_Unified(10001);
Adafruit_LSM303_Accel_Unified accelerometer = Adafruit_LSM303_Accel_Unified(10002);
Adafruit_LSM303_Mag_Unified compass = Adafruit_LSM303_Mag_Unified(10003);
Adafruit_L3GD20 gyro;

Adafruit_TCS34725 color = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
Adafruit_TMP006 heat;
RTC_DS1307 clock;

/*
    Initialize global variables
*/

/*
	Displays
*/

Adafruit_8x8matrix matrix8x8 = Adafruit_8x8matrix();

//  Support for multiple 7 segment displays
Adafruit_7segment sevenSeg[NUMBER_DISPLAYS];

/*
    We have to use a software I2C connection, because the master controller
      uses the main I2C bus to communicate with us.
*/

SoftI2CMaster i2c = SoftI2CMaster(SOFT_I2C_SDA_PIN, SOFT_I2C_SCL_PIN, 0);

/*
    Initialize servos
*/

Servo panS = {
	SERVO_PAN_PIN,
	SERVO_PAN_ADJUST,
	0,
	0,
	SERVO_PAN_LEFT_MAX,
	SERVO_PAN_RIGHT_MAX,
	SERVO_MAX_DEGREES,
	0
};

Servo tiltS = {
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
int ping[MAX_PING];
float gp2d12[MAX_GP2D12];

boolean displayDate = true;
uint8_t dateDisplayFreq = 15;              //  How often to display the date, in minutes
uint8_t minuteCount = 0;                   //  Count the minutes

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
boolean writeFloat (double value, uint8_t decimal = 2, uint8_t nrDisplayDigits = 4, boolean noblank = false) {
	boolean exitStatus = true;
	uint8_t nrDisplays = NUMBER_DISPLAYS;

	long integerPart = 0, decimalPart = 0;
	double sign = 1.0, newValue = 0.0;
	uint8_t digitCount = 1, temp = 0, dotPosition = 0, totalDigits = 0;
	uint8_t intPartLen = 0, decPartLen = 0, valueLen = 0, nrDigits = 0; 
	boolean decimalPoint = false;
	String decPartStr, intPartStr, valueStr;
  
	//  Store the sign
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
//	integerPart = int(value);

	decPartStr = trimTrailingZeros(valueStr.substring(dotPosition + 1));
	decPartLen = decPartStr.length();
//	decimalPart = value - integerPart;

	Serial.print("(writeFloat) Integer part string = '");
	Serial.print(intPartStr);
	Serial.print("' (");
	Serial.print(intPartLen);
	Serial.println(")");

	Serial.print("Decimal part string = '");
	Serial.print(decPartStr);
	Serial.print("' (");
	Serial.print(decPartLen);
	Serial.println(")");

	Serial.print("sign = ");
	Serial.println(sign);

	Serial.print("(writeFloat) value string = '");
	Serial.print(valueStr);
	Serial.print("' (");
	Serial.print(valueLen);
	Serial.println(")");

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
 
		Serial.print("(writeNumber) value = ");
		Serial.print(value);
		Serial.print(", temp = ");
		Serial.println(temp);

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
	Write a number (integer or floating point) to a 7-Segment display
*/
void writeNumber (uint8_t displayNr, uint16_t value, uint8_t decimal = 2, boolean noblank = false) {
	uint8_t digitCount = 1, temp = 0;
	boolean decimalPoint = false;

	temp = value / 100;
/*  
	Serial.print("(writeNumber) value = ");
	Serial.print(value);
	Serial.print(", temp = ");
	Serial.println(temp);
*/

	//	Set first digit of the integer portion
	if ((noblank) or (temp > 9)) {
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

float tempFahrenheit (float celsius) {
	return (celsius * 1.8) + 32;
}

/*
	Display current accelerometer readings
*/
void displayAccelerometerReadings (sensors_event_t *accelEvent) {
	Serial.print("X: ");
	Serial.print(accelEvent->acceleration.x);
	Serial.print("  ");
	Serial.print("Y: ");
	Serial.print(accelEvent->acceleration.y);
	Serial.print("  ");
	Serial.print("Z: ");
	Serial.print(accelEvent->acceleration.z);
	Serial.print("  ");
	Serial.println("m/s^2 ");
}

/*
    Display accelerometer settings
*/
void displayAccelerometerSettings (sensor_t *accelSettings) {
	Serial.println("------------------------------------");
	Serial.println("Accelerometer settings:");
	Serial.println("------------------------------------");
	Serial.print  ("Sensor:       ");
	Serial.println(accelSettings->name);
	Serial.print  ("Driver Ver:   ");
	Serial.println(accelSettings->version);
	Serial.print  ("Unique ID:    ");
	Serial.println(accelSettings->sensor_id);
	Serial.print  ("Max Value:    ");
	Serial.print(accelSettings->max_value);
	Serial.println(" m/s^2");
	Serial.print  ("Min Value:    ");
	Serial.print(accelSettings->min_value);
	Serial.println(" m/s^2");
	Serial.print  ("Resolution:   ");
	Serial.print(accelSettings->resolution);
	Serial.println(" m/s^2");  
	Serial.println("------------------------------------");
}

/*
  Display Compass (magnetometer) readings
  
  The magnetic vector values are in micro-Tesla (uT)
*/
void displayCompassReadings (sensors_event_t *compassEvent) {
	Serial.println("------------------------------------");
	Serial.println("Compass (magnetometer) readings");
	Serial.println("------------------------------------");
	Serial.print("X: ");
	Serial.print(compassEvent->magnetic.x);
	Serial.print("  ");
	Serial.print("Y: ");
	Serial.print(compassEvent->magnetic.y);
	Serial.print("  ");
	Serial.print("Z: ");
	Serial.print(compassEvent->magnetic.z);
	Serial.print("  ");
	Serial.println("uT");
}

/*
	Display Gyroscope readings
*/
void displayGyroReadings (int gyroX, int gyroY, int gyroZ) {
	Serial.println("------------------------------------");
	Serial.println("Gyroscope readings");
	Serial.println("------------------------------------");
	Serial.print("X: ");
	Serial.print(gyroX);
	Serial.print(" ");
	Serial.print("Y: ");
	Serial.print(gyroY);
	Serial.print(" ");
	Serial.print("Z: ");
	Serial.println(gyroZ);
}

/*
    Display the GP2D12 sensor readings (cm)
*/
void displayGP2D12 (void) {
	int sensorNr;
  
	Serial.println("------------------------------------");
	Serial.println("GPD12 IR Sensor readings");
	Serial.println("------------------------------------");

	for (sensorNr = 0; sensorNr < MAX_GP2D12; sensorNr++) { 
		Serial.print("gp2d12 #");
		Serial.print(sensorNr + 1);
		Serial.print(" range = ");
		Serial.print(gp2d12[sensorNr]);
		Serial.println(" cm");
	}

	Serial.println();  
}

/* 
	Function that reads a value from GP2D12 infrared distance sensor and returns a
		value in centimeters.

	This sensor should be used with a refresh rate of 36ms or greater.

	Javier Valencia 2008

	float read_gp2d12(byte pin)

	It can return -1 if something gone wrong.
    
	TODO: Make several readings over a time period, and average them
		for the final reading.
*/
float readGP2D12 (byte pin) {
	int tmp;

	tmp = analogRead(pin);

	if (tmp < 3) {
		return -1;                                  // Invalid value
	} else {
		return (6787.0 /((float)tmp - 3.0)) - 4.0;  // Distance in cm
	}
}

/*
    Convert a pulse width in ms to inches
*/
long microsecondsToInches(long microseconds) {
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
long microsecondsToCentimeters(long microseconds) {
	/*
		The speed of sound is 340 m/s or 29 microseconds per centimeter.

		The ping travels out and back, so to find the distance of the
			object we take half of the distance travelled.
	*/
	return microseconds / 29 / 2;
}

void displayPING (void) {
	int sensorNr;
  
	//	Display PING sensor readings (cm)
	for (sensorNr = 0; sensorNr < MAX_PING; sensorNr++) {
		Serial.print("Ping #");
		Serial.print(sensorNr + 1);
		Serial.print(" range = ");
		Serial.print(ping[sensorNr]);
		Serial.println(" cm");
	}
 
	Serial.println("");
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

		Set units = true for inches, and false for cm
*/
int readPING (byte pingPin, boolean units) {
	long duration;
	int result;

	/*
		The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
		Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
	*/
	pinMode(pingPin, OUTPUT);
	digitalWrite(pingPin, LOW);
	delayMicroseconds(2);
	digitalWrite(pingPin, HIGH);
	delayMicroseconds(5);
	digitalWrite(pingPin, LOW);

	/*
		The same pin is used to read the signal from the PING))): a HIGH
		pulse whose duration is the time (in microseconds) from the sending
		of the ping to the reception of its echo off of an object.

		pinMode(pingPin, INPUT);
		duration = pulseIn(pingPin, HIGH);
	*/

	//  Convert the time into a distance
	if (units) {
		//  Return result in inches.
		result = microsecondsToInches(duration);
	} else {
		//	Return result in cm
		result = microsecondsToCentimeters(duration);
	}
 
	delay(100);
  
	return result;
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
    Move a servo by pulse width in ms (500ms - 2500ms)
*/
void moveServoPw (Servo *servo, int servoPosition, int moveSpeed, int moveTime, boolean term) {
	servo->error = 0;
  
	if ((servoPosition >= servo->minPulse) && (servoPosition <= servo->maxPulse)) {
		Serial.print("#");
		Serial.print(servo->pin);
		Serial.print(" P");
		Serial.print(servoPosition + servo->offset);

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
			Serial.print(" S");
			Serial.print(moveSpeed);
		}
    
		//  Terminate the command
		if (term) {
			if (moveTime != 0) {
				Serial.print(" T");
				Serial.print(moveTime);
			}

			Serial.println();
		}
  	}
}

/*
    Move a servo by degrees (-90 to 90) or (0 - 180)
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
		Serial.print("#");
		Serial.print(servo->pin);
		Serial.print(" P");
		Serial.print(servoPulse);

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
			Serial.print(" S");
			Serial.print(moveSpeed);
		}
    
		//  Terminate the command
		if (term) {
			if (moveTime != 0) {
				Serial.print(" T");
				Serial.print(moveTime);
			}
      
			Serial.println();
		}
	}
}

/*
    Process error conditions
*/
void processError (byte err) {
	Serial.print("Error: ");
	Serial.print(err);
	Serial.println("");
}

/*
    Called when a request from an I2C (Wire) Master comes in
*/
void wireRequestEvent (void) {
  
}

/*
    Called when the I2C (Wire) Slave receives data from an I2C (Wire) Master
*/
void wireReceiveData (int nrBytesRead) {

}

void setup() {
	uint8_t nrDisp;
  
	//  Initialize serial port communication
	Serial.begin(115200);
	Serial.println("W.A.L.T.E.R. 2.0 Navigation");

	//  Start up the Wire library as a slave device at address 0xE0
	Wire.begin(NAV_I2C_ADDRESS);

	//  Register event handlers
	Wire.onRequest(wireRequestEvent);
	Wire.onReceive(wireReceiveData);

	//  Initialize the LED pin as an output.
	pinMode(HEARTBEAT_LED, OUTPUT);
  
	/*  Initialize the accelerometer */
	if (! accelerometer.begin()) {
		/* There was a problem detecting the LSM303 ... check your connections */
		Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
		while(1);
	}
  
	/*  Initialize the compass (magnetometer) sensor */
	if (! compass.begin()) {
		/*	There was a problem detecting the LSM303 ... check your connections */
		Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
		while(1);
	}

	/*  Initialize and warn if we couldn't detect the gyroscope chip */
	if (! gyro.begin(gyro.L3DS20_RANGE_250DPS)) {
//	if (!gyro.begin(gyro.L3DS20_RANGE_500DPS)) {
//	if (!gyro.begin(gyro.L3DS20_RANGE_2000DPS)) {
		Serial.println("Oops ... unable to initialize the L3GD20. Check your wiring!");
		while (1);
	}

	//	Initialize the BMP180 temperature sensor
	if(!temperature.begin()) {
		//  There was a problem detecting the BMP180 ... check your connections
		Serial.print("Ooops, no BMP180 detected ... Check your wiring or I2C ADDR!");
		while(1);
	}

	//	Check to be sure the RTC is running
	if (! clock.isrunning()) {
		Serial.println("RTC is NOT running!");
		while(1);
	}

	/*
		Multiple 7 segment displays will be supported.
	*/

	//	Initialize the 7-Segment display(s)
	for (nrDisp = 0; nrDisp < NUMBER_DISPLAYS; nrDisp++) {
		sevenSeg[nrDisp] = Adafruit_7segment();
		sevenSeg[nrDisp].begin(SEVEN_SEG_BASE_ADDR + nrDisp);
		sevenSeg[nrDisp].setBrightness(1);
		sevenSeg[nrDisp].drawColon(false);
	}

	//	Initialize the 8x8 matrix display
	matrix8x8.begin(MATRIX_DISPLAY_ADDR);
	matrix8x8.setBrightness(1);
	matrix8x8.setRotation(3);

	//  Test all the displays
	for (nrDisp = 0; nrDisp < NUMBER_DISPLAYS; nrDisp++) {
		sevenSeg[nrDisp].print(8888);
		sevenSeg[nrDisp].drawColon(true);
	}

	matrix8x8.drawBitmap(0, 0, allon_bmp, 8, 8, LED_ON);
	sevenSeg[0].writeDisplay();
	matrix8x8.writeDisplay();
 
	delay(2000);
  
	//  Put the front pan/tilt at home position
	moveServoPw(&panS, SERVO_CENTER_MS, 0, 0, false);
	moveServoPw(&tiltS, SERVO_CENTER_MS, 0, 0, true);
//	moveServoDegrees(&panS, moveDegrees, moveSpeed, moveTime, false);
//	moveServoDegrees(&tiltS, moveDegrees, moveSpeed, moveTime, true);
  
/*  
	writeFloat(163.90127, 2, 4, false);
	writeFloat(-23.0159, 2, 4, false);
*/
}

void loop() {
	DateTime now = clock.now();

	byte error = 0;

	boolean amTime;

	uint8_t analogPin = 0;
	uint8_t digitalPin = 0;
	uint8_t displayNr = 0;
	uint8_t hour = now.hour();

	float celsius, fahrenheit, altitude;
	float accelX, accelY, accelZ;
	float compassX, compassY, compassZ;
	float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;

	int gyroX, gyroY, gyroZ;

	sensors_event_t accelEvent, compassEvent, tempEvent;
  
	uint16_t displayInt;

	String displayString;
	String timeString;
	String currMonth = leftZeroPadString(String(now.month()), 2);
	String currDay = leftZeroPadString(String(now.day()), 2);
	String currYear = leftZeroPadString(String(now.year()), 4);
	String currMinute = leftZeroPadString(String(now.minute()), 2);
	String currSecond = leftZeroPadString(String(now.second()), 2);

	/*
		Code starts here
	*/
  
	// Pulse the heartbeat LED
	pulseDigital(HEARTBEAT_LED, 500);
  
	//  Clear the displays
	for (displayNr = 0; displayNr < NUMBER_DISPLAYS; displayNr ++) {
		sevenSeg[displayNr].clear();
		sevenSeg[displayNr].drawColon(false);
	}

	matrix8x8.clear();

	Serial.print("Month = ");
	Serial.print(now.month());
	Serial.print(", Day = ");
	Serial.print(now.day());
	Serial.print(", Year = ");
	Serial.println(now.year());

	//  Display the date, if it's time
	if (displayDate) {
		displayInt = (now.month() * 100) + now.day();  

		//  Month and day
		writeNumber(0, displayInt, 0, true);
		matrix8x8.drawBitmap(0, 0, date_bmp, 8, 8, LED_ON);

		//	Update the displays
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
	}	//	End date display
  
	sevenSeg[0].clear();
	matrix8x8.clear();

	if (hour == 0) {
		hour = 12;
		amTime = true;
	} else if (hour == 12) {
		amTime = false;
	} else if (hour > 12) {
		amTime = false;
		hour = hour - 12;
	} else {
		amTime = true;
	}
  
	displayInt = (hour * 100) + now.minute();  
	timeString = leftZeroPadString(String((hour * 100) + now.minute()), 4);

	//  Display the current time on the 7 segment display
	writeNumber(0, displayInt, 0, false);
	sevenSeg[0].drawColon(true);
  
	matrix8x8.clear();
  
	if (amTime) {
		matrix8x8.drawBitmap(0, 0, am_bmp, 8, 8, LED_ON);
	} else {
		matrix8x8.drawBitmap(0, 0, pm_bmp, 8, 8, LED_ON);
	}

	//	Update the displays
	sevenSeg[0].writeDisplay();
	matrix8x8.writeDisplay();
  
	delay(45000);

	sevenSeg[0].drawColon(false);

/*
	//  Display the current time
	Serial.print("Current date: ");
	Serial.print(currMonth);
	Serial.print('/');
	Serial.print(currDay);
	Serial.print('/');
	Serial.println(currYear);

	Serial.print("Current time: (");
	Serial.print(timeString);
	Serial.print(") ");
	Serial.print(currHour);
	Serial.print(':');
	Serial.print(currMinute);
	Serial.print(':');
	Serial.print(currSecond);
	Serial.print(" ");
	Serial.println(amPM);
	Serial.println();
*/
	/*
		Get accelerometer readings
	*/
	accelerometer.getEvent(&accelEvent);
 
	//	Display the results (acceleration is measured in m/s^2)
	accelX = accelEvent.acceleration.x;
	accelY = accelEvent.acceleration.y;
	accelZ = accelEvent.acceleration.z;

	Serial.print("Accelerometer: X = ");
	Serial.print(accelX);
	Serial.print(", Y = ");
	Serial.print(accelY);
	Serial.print(", Z = ");
	Serial.print(accelZ);
	Serial.println(" m/s^2");

	/*
		Get compass readings
	*/
	compass.getEvent(&compassEvent);

	compassX = compassEvent.magnetic.x;
	compassY = compassEvent.magnetic.y;
	compassZ = compassEvent.magnetic.z;

	Serial.print("Compass: X = ");
	Serial.print(compassX);
	Serial.print(", Y = ");
	Serial.print(compassY);
	Serial.print(", Z = ");
	Serial.println(compassZ);

	/*
		Get gyro readings
	*/
	gyro.read();

	gyroX = (int)gyro.data.x;
	gyroY = (int)gyro.data.y;
	gyroZ = (int)gyro.data.z;

	Serial.print("Gyroscope: X = ");
	Serial.print(gyroX);
	Serial.print(", Y = ");
	Serial.print(gyroY);
	Serial.print(", Z = ");
	Serial.println(gyroZ);

	/*
		Accelerometer and Gyro reactive behaviors HERE
	*/

	//	Get readings from all the GP2D12 Analog IR range sensors, if any, and store them
	if (MAX_GP2D12 > 0) {
		for (analogPin = 0; analogPin < MAX_GP2D12; analogPin++) { 
			gp2d12[analogPin] = readGP2D12(analogPin);
		}
	}

	//	Get readings from all the Parallax PING Ultrasonic range sensors, if any, and store them
	if (MAX_PING > 0) {
		for (digitalPin = 0; digitalPin < MAX_PING; digitalPin++) {
			ping[digitalPin] = readPING(digitalPin + DIGITAL_PIN_BASE, false);
		}
	}

	/*
		Distance related reactive behaviors HERE
	*/

	//	Get a new sensor event
	temperature.getEvent(&tempEvent);
  
	if (tempEvent.pressure) {
		/* Calculating altitude with reasonable accuracy requires pressure    *
		 * sea level pressure for your position at the moment the data is     *
		 * converted, as well as the ambient temperature in degress           *
		 * celcius.  If you don't have these values, a 'generic' value of     *
		 * 1013.25 hPa can be used (defined as SENSORS_PRESSURE_SEALEVELHPA   *
		 * in sensors.h), but this isn't ideal and will give variable         *
		 * results from one day to the next.                                  *
		 *                                                                    *
		 * You can usually find the current SLP value by looking at weather   *
		 * websites or from environmental information centers near any major  *
		 * airport.                                                           *
		 *                                                                    *
		 * For example, for Paris, France you can check the current mean      *
		 * pressure and sea level at: http://bit.ly/16Au8ol                   */

		//  First we get the current temperature from the BMP180 in celsius and fahrenheit
		temperature.getTemperature(&celsius);
		fahrenheit = tempFahrenheit(celsius);
    
		Serial.print("Temperature: ");
		Serial.print(fahrenheit);
		Serial.print(" F, ");
		Serial.print(celsius);
		Serial.println(" C.");

		//	Convert the atmospheric pressure, SLP and temp to altitude in meters
		altitude = temperature.pressureToAltitude(seaLevelPressure, tempEvent.pressure, celsius); 

		//  Display the temperature in Fahrenheit
		writeNumber(0, int(fahrenheit * 100), 2, false);
		sevenSeg[0].writeDisplay();

		matrix8x8.clear();
		matrix8x8.drawBitmap(0, 0, f_bmp, 8, 8, LED_ON);
		matrix8x8.writeDisplay();

		delay(7500);

		//  Display the temperature in Celsius
		writeNumber(0, int(celsius * 100), 2, false);
	    sevenSeg[0].writeDisplay();

		matrix8x8.clear();
		matrix8x8.drawBitmap(0, 0, c_bmp, 8, 8, LED_ON);
		matrix8x8.writeDisplay();
	}

	minuteCount += 1;
	displayDate = (minuteCount == dateDisplayFreq);

	if (error != 0) {
		processError(error);
	}

	Serial.println(".");
	delay(7500);
}
