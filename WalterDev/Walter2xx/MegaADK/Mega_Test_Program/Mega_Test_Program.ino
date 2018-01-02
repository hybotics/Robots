/*
    Arduino Mega Test Program
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

#include "Mega_Test_Program.h"

#define  BOARD_LED        13
#define  TOGGLE_TIME_MS   500

//	We only have one RoboClaw 2x5 right now
uint8_t roboClawControllers = ROBOCLAW_CONTROLLERS - 1;
uint8_t	roboClawBaseAddress = ROBOCLAW_SERIAL_BASE_ADDR;
uint8_t roboClawAddress = ROBOCLAW_SERIAL_BASE_ADDR;

Adafruit_BMP180_Unified temperature = Adafruit_BMP180_Unified(10001);
Adafruit_LSM303_Accel_Unified accelerometer = Adafruit_LSM303_Accel_Unified(10002);
Adafruit_LSM303_Mag_Unified compass = Adafruit_LSM303_Mag_Unified(10003);
Adafruit_L3GD20 gyro;

Adafruit_TCS34725 rgbColor = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
Adafruit_TMP006 heat = Adafruit_TMP006();
RTC_DS1307 clock;

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
    SSC-32 works on Serial1
    RoboClaw 2x5 works on Serial2
*/

//	Hardware Serial console (replaces Serial.* routines)
BMSerial console = BMSerial(HARDWARE_SERIAL_RX_PIN, HARDWARE_SERIAL_TX_PIN);

/*
	BMSerial Ports - Hardware serial ports on the Arduino Mega ADK
*/
//	Hardware Serial1
BMSerial ssc32 = BMSerial(SERIAL_SSC32_RX_PIN, SERIAL_SSC32_TX_PIN);
//	Hardware Serial2
RoboClaw roboClaw = RoboClaw(SERIAL_ROBOCLAW_RX_PIN, SERIAL_ROBOCLAW_TX_PIN);
//	Hardware Serial3
BMSerial xbee = BMSerial(SERIAL_XBEE_RX_PIN, SERIAL_XBEE_TX_PIN);

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
    Pulses a digital pin for a duration in ms
*/
void pulseDigital(int pin, int duration) {
	digitalWrite(pin, HIGH);			// Turn the ON by making the voltage HIGH (5V)
	delay(duration);					// Wait for duration ms
	digitalWrite(pin, LOW);				// Turn the pin OFF by making the voltage LOW (0V)
	delay(duration);					// Wait for duration ms
}

void setup () {
	//  Start up the Wire library as a slave device at address 0xE0
	Wire.begin();

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

void loop() {
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
  
}
