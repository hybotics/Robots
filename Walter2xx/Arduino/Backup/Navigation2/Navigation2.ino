/*
	Program:		W.A.L.T.E.R. 2.0, Main navigation and reactive behaviors sketch
	Date:			16-Jan-2014
	Version:		0.1.6 ALPHA

	Purpose:		Added two enum definitions for SensorLocation and MotorLocation. I'm
						not sure the sensor locations are going to work out.

					Added SoftwareSerial ports for the SSC-32 and RoboClaw 2x5 controllers

	Dependencies:	Adafruit libraries:
						LSM303DLHC, L3GD20, TMP006, TCS34727, RTClib for the DS1307

					Hybotics libraries:
						BMP180 (modified from Adafruit's BMP085 library)

	Comments:		Credit is given, where applicable, for code I did not originate.
						This sketch started out as an Adafruit tutorial for the electret
						microphones being used for sound detection. I've also pulled
						code for the GP2Y0A21YK0F IR and PING sensors from the Arduino
						Playground, which I have modified to suit my needs.

					Copyright (C) 2013 Dale Weber <hybotics.pdx@gmail.com>.
*/

#include <Wire.h>

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
#include <SoftwareSerial.h>
#include <SoftI2CMaster.h>

#include "Navigation2.h"

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
			GP2Y0A21YK0F IR Ranging sensors (4)
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
    We have to use a software I2C connection, because the master controller
      uses the main I2C bus to communicate with us.
*/
SoftI2CMaster i2c = SoftI2CMaster(SOFT_I2C_SDA_PIN, SOFT_I2C_SCL_PIN, 0);

SoftwareSerial ssc32 = SoftwareSerial(SOFTSER_SSC32_RX_PIN, SOFTSER_SSC32_TX_PIN);
SoftwareSerial roboclaw = SoftwareSerial(SOFTSER_ROBOCLAW_RX_PIN, SOFTSER_ROBOCLAW_TX_PIN);

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
int ping[MAX_NUMBER_PING];
float ir[MAX_NUMBER_IR];

boolean displayDate = true;					//	Whether to display the date or not
uint8_t dateDisplayFreq = 15;				//  How often to display the date, in minutes
uint8_t minuteCount = 0;					//  Count the minutes

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

float tempFahrenheit (float celsius) {
	return (celsius * 1.8) + 32;
}

/*
    Display the GP2Y0A21YK0F IR sensor readings (cm)
*/
void displayIR (void) {
	int sensorNr;
  
	Serial.println("------------------------------------");
	Serial.println("IR Sensor readings");
	Serial.println("------------------------------------");

	for (sensorNr = 0; sensorNr < MAX_NUMBER_IR; sensorNr++) { 
		Serial.print("IR #");
		Serial.print(sensorNr + 1);
		Serial.print(" range = ");
		Serial.print(ir[sensorNr]);
		Serial.println(" cm");
	}

	Serial.println();  
}

/*
	Display the readings from the PING Ultrasonic sensors
*/
void displayPING (void) {
	int sensorNr;
  
	//	Display PING sensor readings (cm)
	for (sensorNr = 0; sensorNr < MAX_NUMBER_PING; sensorNr++) {
		Serial.print("Ping #");
		Serial.print(sensorNr + 1);
		Serial.print(" range = ");
		Serial.print(ping[sensorNr]);
		Serial.println(" cm");
	}
 
	Serial.println("");
}

/* 
	Function that reads a value from GP2Y0A21YK0F infrared distance sensor and returns a
		value in centimeters.

	This sensor should be used with a refresh rate of 36ms or greater.

	Javier Valencia 2008

	float readIR(byte pin)

	It can return -1 if something gone wrong.

	TODO: Make several readings over a time period, and average them
		for the final reading.
*/
float readIR (byte pin) {
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

	//	Initialize SoftwareSerial ports
	ssc32.begin(115200);
	roboclaw.begin(38400);
  
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
	if (! temperature.begin()) {
		//  There was a problem detecting the BMP180 ... check your connections
		Serial.print("Ooops, no BMP180 detected ... Check your wiring or I2C ADDR!");
		while(1);
	}

	//	Check to be sure the RTC is running
	if (! clock.isrunning()) {
		Serial.println("RTC is NOT running!");
		while(1);
	}
  
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
	compass.getEvent(&compassEvent);

	compassX = compassEvent.magnetic.x;
	compassY = compassEvent.magnetic.y;
	compassZ = compassEvent.magnetic.z;

	/*
		Get gyro readings
	*/
	gyro.read();

	gyroX = (int)gyro.data.x;
	gyroY = (int)gyro.data.y;
	gyroZ = (int)gyro.data.z;

	/*
		Accelerometer and Gyro reactive behaviors HERE
	*/

	//	Get readings from all the GP2Y0A21YK0F Analog IR range sensors, if any, and store them
	if (MAX_NUMBER_IR > 0) {
		for (analogPin = 0; analogPin < MAX_NUMBER_IR; analogPin++) { 
			ir[analogPin] = readIR(analogPin);
		}
	}

	//	Get readings from all the Parallax PING Ultrasonic range sensors, if any, and store them
	if (MAX_NUMBER_PING > 0) {
		for (digitalPin = 0; digitalPin < MAX_NUMBER_PING; digitalPin++) {
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

		//	Convert the atmospheric pressure, SLP and temp to altitude in meters
		altitude = temperature.pressureToAltitude(seaLevelPressure, tempEvent.pressure, celsius); 
	}

	if (error != 0) {
		processError(error);
	}
}
