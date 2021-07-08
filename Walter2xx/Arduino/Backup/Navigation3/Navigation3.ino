/*
	Program:		W.A.L.T.E.R. 2.0, Main navigation and reactive behaviors sketch
	Date:			18-Jan-2014
	Version:		0.1.8 ALPHA

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
					v0.1.7:
					Added ColorSensor struct for RGB color sensor data; added code to read the TCS34725
						RGB color and TMP006 heat sensors.
						
					Added a control pin (COLOR_SENSOR_LED, pin 4) so the LED can be turned on and off.

					-------------------------------------------------------------------------------------
					v0.1.8:
					Fixed a bug in readPING() - was not getting the duration, because code was in a comment.

					Now displaying readings from all sensors in the main loop. RGB color and Heat sensors are
						working. IMU seems to be working so far - still need to get useful information from it.

					The SSC-32 doesn't seem to like SoftwareSerial ports.
					
					I'm thinking more seriously about moving to the Arduino Mega ADK board. There is only about
						5Kb of program memory left, RAM is low, etc. I don't think I have a choice here.

	Dependencies:	Adafruit libraries:
						LSM303DLHC, L3GD20, TMP006, TCS34725, RTClib for the DS1307

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
#include <SoftI2CMaster.h>

#include "Navigation3.h"

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

Adafruit_TCS34725 rgbColor = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
Adafruit_TMP006 heat = Adafruit_TMP006();
RTC_DS1307 clock;

/*
    Initialize global variables
*/

//	Hardware Serial console (replaces Serial.* routines)
BMSerial console(HARDWARE_SERIAL_RX_PIN, HARDWARE_SERIAL_TX_PIN);

/*
    We have to use a software I2C connection, because the master controller
      uses the main I2C bus to communicate with us.
*/
SoftI2CMaster i2c = SoftI2CMaster(SOFT_I2C_SDA_PIN, SOFT_I2C_SCL_PIN, 0);

/*
	BMSerial Ports - Keep the hardware serial port for programming and debugging
*/
BMSerial ssc32(BMSERIAL_SSC32_RX_PIN, BMSERIAL_SSC32_TX_PIN);
RoboClaw roboClaw(BMSERIAL_ROBOCLAW_RX_PIN, BMSERIAL_ROBOCLAW_TX_PIN);

//	We only have one RoboClaw 2x5 right now
uint8_t roboClawControllers = ROBOCLAW_CONTROLLERS - 1;
uint8_t	roboClawBaseAddress = ROBOCLAW_SERIAL_BASE_ADDR;
uint8_t roboClawAddress = ROBOCLAW_SERIAL_BASE_ADDR;

/*
	Initialize motors
*/

//	RoboClaw 2x5 motor M1
Motor leftMotor = {
	//	These four parameters are for PWM (R/C) control modes
	SERVO_MOTOR_LEFT_PIN,
	SERVO_MOTOR_LEFT_MIN,
	SERVO_MOTOR_LEFT_MAX,
	SERVO_MOTOR_LEFT_ADJUST,

	0,								//	Motor Encoder value
	0,								//	Motor Speed
	true,							//	Motor direction: Forward = true, Reverse = false
	0,								//	Distance traveled
	0								//	Encoder ticks
};

//	RoboClaw 2x5 motor M2
Motor rightMotor = {
	//	These four parameters are for PWM (R/C) control modes
	SERVO_MOTOR_RIGHT_PIN,
	SERVO_MOTOR_RIGHT_MIN,
	SERVO_MOTOR_RIGHT_MAX,
	SERVO_MOTOR_RIGHT_ADJUST,

	0,								//	Motor Encoder value
	0,								//	Motor Speed
	true,							//	Motor direction: Forward = true, Reverse = false
	0,								//	Distance traveled
	0								//	Encoder ticks
};
                            
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
	Code starts here
*/

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
	Convert a temperature in Celsius to Fahrenheit
*/
float tempFahrenheit (float celsius) {
	return (celsius * 1.8) + 32;
}

/*
	Display the TCS34725 RGB color sensor readings
*/
void displayColorSensorReadings (ColorSensor *colorData) {
	console.print("Color Temperature: ");
	console.print(colorData->colorTemp, DEC);
	console.print(" K, ");
	console.print("Lux: ");
	console.print(colorData->lux, DEC);
	console.print(", ");
	console.print("Red: ");
	console.print(colorData->red, DEC);
	console.print(", ");
	console.print("Green: ");
	console.print(colorData->green, DEC);
	console.print(", ");
	console.print("Blue: ");
	console.print(colorData->blue, DEC);
	console.print(", ");
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

/*
	Display data from the RoboClaw 2x5 motor controller
*/
void displayRoboClawEncoderSpeedAccelDistance (Motor *leftMotor, Motor *rightMotor) {
	uint8_t roboClawStatus;
	bool roboClawValid;

	console.println("RoboClaw 2x5 status:");
	console.println();

	leftMotor->encoder = roboClaw.ReadEncM1(roboClawAddress, &roboClawStatus, &roboClawValid);
	
    if (roboClawValid) {
		console.print("Left Motor Encoder = ");
		console.print(leftMotor->encoder, DEC);
		console.print(", Status =  ");
		console.print(roboClawStatus, HEX);
		console.println();
	}

	leftMotor->speed = roboClaw.ReadSpeedM1(roboClawAddress, &roboClawStatus, &roboClawValid);

	if (roboClawValid) {
		console.print("Left Motor Speed = ");
		console.print(leftMotor->speed, DEC);
		console.println();
	}

	rightMotor->encoder = roboClaw.ReadEncM2(roboClawAddress, &roboClawStatus, &roboClawValid);

	if (roboClawValid) {
		console.print("Right Motor Encoder = ");
		console.print(rightMotor->encoder, DEC);
		console.print(", Status = ");
		console.print(roboClawStatus, HEX);
		console.println();
	}

	rightMotor->speed = roboClaw.ReadSpeedM2(roboClawAddress, &roboClawStatus, &roboClawValid);

	if (roboClawValid) {
		console.print("Right Motor Speed = ");
		console.print(rightMotor->speed, DEC);
		console.println();
	}
	
	console.println("");
}

/*
    Display the GP2Y0A21YK0F IR sensor readings (cm)
*/
void displayIR (void) {
	int sensorNr;
  
	console.println("------------------------------------");
	console.println("IR Sensor readings");
	console.println("------------------------------------");

	for (sensorNr = 0; sensorNr < MAX_NUMBER_IR; sensorNr++) { 
		console.print("IR #");
		console.print(sensorNr + 1);
		console.print(" range = ");
		console.print(ir[sensorNr]);
		console.println(" cm");
	}

	console.println();  
}

/*
	Display the readings from the PING Ultrasonic sensors
*/
void displayPING (void) {
	int sensorNr;
  
	console.println("------------------------------------");
	console.println("PING Ultrasonic Sensor readings");
	console.println("------------------------------------");
  
	//	Display PING sensor readings (cm)
	for (sensorNr = 0; sensorNr < MAX_NUMBER_PING; sensorNr++) {
		console.print("Ping #");
		console.print(sensorNr + 1);
		console.print(" range = ");
		console.print(ping[sensorNr]);
		console.println(" cm");
	}
 
	console.println("");
}

/*
	Display the readings from the IMU (Accelerometer, Magnetometer [Compass], and Gyro
*/
void displayIMUReadings (sensors_event_t *accelEvent, sensors_event_t *compassEvent, float celsius, float fahrenheit, int gyroX, int gyroY, int gyroZ) {
	//	Accelerometer readings
	console.print("Accelerometer: X = ");
	console.print(accelEvent->acceleration.x);
	console.print(", Y = ");
	console.print(accelEvent->acceleration.y);
	console.print(", Z = ");
	console.println(accelEvent->acceleration.z);

	//	Magnetometer (Compass) readings
	console.print("Magnetometer (Compass): X = ");
	console.print(compassEvent->magnetic.x);
	console.print(", Y = ");
	console.print(compassEvent->magnetic.y);
	console.print(", Z = ");
	console.println(compassEvent->magnetic.z);

	//	Gyro readings
	console.print("Gyro: X = ");
	console.print(gyroX);
	console.print(", Y = ");
	console.print(gyroY);
	console.print(", Z = ");
	console.println(gyroZ);

	//	Temperature readings
	console.print("Room Temperature = ");
	console.print(fahrenheit);
	console.print(" F, ");
	console.print(celsius);
	console.print(" C.");
	
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
int readPING (byte pingPin, boolean units=true) {
	byte realPin = pingPin + PING_PIN_BASE;
	long duration;
	int result;

	/*
		The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
		Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
	*/
	pinMode(realPin, OUTPUT);
	digitalWrite(realPin, LOW);
	delayMicroseconds(2);
	digitalWrite(realPin, HIGH);
	delayMicroseconds(5);
	digitalWrite(realPin, LOW);

	/*
		The same pin is used to read the signal from the PING))): a HIGH
		pulse whose duration is the time (in microseconds) from the sending
		of the ping to the reception of its echo off of an object.
	*/
	pinMode(realPin, INPUT);
	duration = pulseIn(realPin, HIGH);

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
    Pulses a digital pin for a duration in ms
*/
void pulseDigital(int pin, int duration) {
	digitalWrite(pin, HIGH);			// Turn the ON by making the voltage HIGH (5V)
	delay(duration);					// Wait for duration ms
	digitalWrite(pin, LOW);				// Turn the pin OFF by making the voltage LOW (0V)
	delay(duration);					// Wait for duration ms
}

/*
    Move a servo by pulse width in ms (500ms - 2500ms) - Modified to use BMSerial
*/
void moveServoPw (BMSerial *port, Servo *servo, int servoPosition, int moveSpeed, int moveTime, boolean term) {
	servo->error = 0;
  
	if ((servoPosition >= servo->minPulse) && (servoPosition <= servo->maxPulse)) {
		port->print("#");
		port->print(servo->pin);
		port->print(" P");
		port->print(servoPosition + servo->offset);

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
			port->print(" S");
			port->print(moveSpeed);
		}
    
		//  Terminate the command
		if (term) {
			if (moveTime != 0) {
				port->print(" T");
				port->print(moveTime);
			}

			port->println();
		}
  	}
}

/*
    Move a servo by degrees (-90 to 90) or (0 - 180) - Modified to use BMSerial
*/
void moveServoDegrees (BMSerial *port, Servo *servo, int servoDegrees, int moveSpeed, int moveTime, boolean term) {
	int servoPulse = SERVO_CENTER_MS + servo->offset;

	servo->error = 0;
  
	//  Convert degrees to ms for the servo move
	if (servo->maxDegrees == 90) {
		servoPulse = (SERVO_CENTER_MS + servo->offset) + (servoDegrees * 10);
	} else if (servo->maxDegrees == 180) {
		servoPulse = (SERVO_CENTER_MS + servo->offset) + ((servoDegrees - 90) * 10);
	}

	if ((servoPulse >= servo->minPulse) && (servoPulse <= servo->maxPulse)) {
		port->print("#");
		port->print(servo->pin);
		port->print(" P");
		port->print(servoPulse);

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
			port->print(" S");
			port->print(moveSpeed);
		}
    
		//  Terminate the command
		if (term) {
			if (moveTime != 0) {
				port->print(" T");
				port->print(moveTime);
			}
      
			port->println();
		}
	}
}

/*
    Process error conditions
*/
void processError (byte err) {
	console.print("Error: ");
	console.print(err);
	console.println("");
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

void setup () {
	//  Start up the Wire library as a slave device at address 0xE0
//	Wire.begin(NAV_I2C_ADDRESS);

	//  Register event handlers
//	Wire.onRequest(wireRequestEvent);
//	Wire.onReceive(wireReceiveData);

	//  Initialize the LED pin as an output.
	pinMode(HEARTBEAT_LED, OUTPUT);

	//	Initialize and turn off the TCS34725 RGB Color sensor's LED
	pinMode(COLOR_SENSOR_LED, OUTPUT);
	digitalWrite(COLOR_SENSOR_LED, LOW);

	/*
		Initialize BMSerial ports
	*/
	//  Initialize serial port communication (BMSerial)
	console.begin(115200);
	console.println("W.A.L.T.E.R. 2.0 Navigation");

	//	Initialize the SSC-32 servo controller port
	ssc32.begin(115200);
	
	//	Initialize the RoboClaw 2x5 motor controller port
	roboClaw.begin(38400);

	//	Set the RoboClaw motor constants
	roboClaw.SetM1Constants(roboClawAddress , ROBOCLAW_KD, ROBOCLAW_KP, ROBOCLAW_KI, ROBOCLAW_QPPS);
	roboClaw.SetM2Constants(roboClawAddress , ROBOCLAW_KD, ROBOCLAW_KP, ROBOCLAW_KI, ROBOCLAW_QPPS);

	displayRoboClawEncoderSpeedAccelDistance(&leftMotor, &rightMotor);

	//	Initialize the accelerometer
	if (! accelerometer.begin()) {
		/* There was a problem detecting the LSM303 ... check your connections */
		console.println("Ooops, no LSM303 detected ... Check your wiring!");
		while(1);
	}
  
	//	Initialize the magnetometer (compass) sensor
	if (! compass.begin()) {
		/*	There was a problem detecting the LSM303 ... check your connections */
		console.println("Ooops, no LSM303 detected ... Check your wiring!");
		while(1);
	}

	//	Initialize and warn if we couldn't detect the gyroscope chip
	if (! gyro.begin(gyro.L3DS20_RANGE_250DPS)) {
//	if (!gyro.begin(gyro.L3DS20_RANGE_500DPS)) {
//	if (!gyro.begin(gyro.L3DS20_RANGE_2000DPS)) {
		console.println("Oops ... unable to initialize the L3GD20. Check your wiring!");
		while (1);
	}

	//	Initialize the BMP180 temperature sensor
	if (! temperature.begin()) {
		//  There was a problem detecting the BMP180 ... check your connections
		console.print("Ooops, no BMP180 detected ... Check your wiring or I2C ADDR!");
		while(1);
	}
	
	//	Initialize the TMP006 heat sensor
	if (! heat.begin()) {
		console.print("There was a problem initializing the TMP006 heat sensor .. check your wiring or I2C ADDR!");
		while(1);
	}
	
	//	Initialize the TCS34725 color sensor
	if (! rgbColor.begin()) {
		console.print("There was a problem initializing the TCS34725 RGB color sensor .. check your wiring or I2C ADDR!");
		while(1);
	}

	//	Check to be sure the RTC is running
	if (! clock.isrunning()) {
		console.println("Real Time Clock is NOT running!");
		while(1);
	}
  
	//  Put the front pan/tilt at home position
	moveServoPw(&ssc32, &panServo, SERVO_CENTER_MS, 0, 0, false);
	moveServoPw(&ssc32, &tiltServo, SERVO_CENTER_MS, 0, 0, true);
//	moveServoDegrees(&ssc32, &panS, moveDegrees, moveSpeed, moveTime, false);
//	moveServoDegrees(&ssc32, &tiltS, moveDegrees, moveSpeed, moveTime, true);
}

void loop () {
	//	The current date and time from the DS1307 real time clock
	DateTime now = clock.now();

	uint8_t roboClawStatus;
	bool roboClawValid;
    
	byte error = 0;

	boolean amTime;

	uint8_t analogPin = 0;
	uint8_t digitalPin = 0;

	float celsius, fahrenheit, altitude;
	float accelX, accelY, accelZ;
	float compassX, compassY, compassZ;
	float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;

	int gyroX, gyroY, gyroZ;

	sensors_event_t accelEvent, compassEvent, tempEvent;

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

	displayIR();

	//	Get readings from all the Parallax PING Ultrasonic range sensors, if any, and store them
	if (MAX_NUMBER_PING > 0) {
		for (digitalPin = 0; digitalPin < MAX_NUMBER_PING; digitalPin++) {
			ping[digitalPin] = readPING(digitalPin);
		}
	}

	displayPING();

	/*
		Distance related reactive behaviors HERE
	*/

	/*
		Read the RoboClaw 2x5 motor controller encoders
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

		displayIMUReadings(&accelEvent, &compassEvent, celsius, fahrenheit, gyroX, gyroY, gyroZ);
	}

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
}
