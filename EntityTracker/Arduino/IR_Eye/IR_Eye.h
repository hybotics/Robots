/*
	IR_Eye.ino - Test sketch for the Dagu Compound IR Sensor
		(http://www.robotshop.com/en/dagu-compound-infrared-sensor.html)

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
	Program:      	Dagu Compound IR Sensor test sketch
						(http://www.robotshop.com/en/dagu-compound-infrared-sensor.html)
	Date:         	15-Mar-2014
	Version:      	0.0.1 Bluno - ALPHA

	Purpose:		Test the Dagu Compound IR Sensor

	Dependencies:	None

	Comments:		Credit is given, where applicable, for code I did not originate.
						This sketch started out as an Adafruit tutorial for the electret
						microphones being used for sound detection. I've also pulled
						code for the GP2D12 IR and PING sensors from the Arduino
						Playground, which I have modified to suit my needs.
*/
#ifndef	__IR_EYE_H__
#define	__IR_EYE_H__

#define	BUILD_VERSION					"0.0.1"
#define	BUILD_DATE 						"01-Sep-2014"
#define	BUILD_BOARD						"Bluno"

#define	SPEAKER_OUT						5			//	Digital 5
#define	HEARTBEAT_LED					13			//	Digital 13

//	Display constants
#define	MAX_NUMBER_7SEG_DISPLAYS		0
#define	SEVEN_SEG_BASE_ADDR				0x70

#define	MATRIX_DISPLAY_ADDR				SEVEN_SEG_BASE_ADDR + MAX_NUMBER_7SEG_DISPLAYS

#define	WAIT_DELAY_SECONDS				10

/*
	These settings control whether standard information is displayed
		on the seven segment and matrix displays or not, and how
		often, in minutes.
*/
#define	DISPLAY_INFORMATION				false

#define	DISPLAY_DATE_FREQ_MIN			15
#define	DISPLAY_TIME_FREQ_MIN			15
#define	DISPLAY_TEMPERATURE_FREQ_MIN	15

//  For SoftI2CMaster
#define	SOFT_I2C_SDA_PIN				2			//	Digital 2
#define	SOFT_I2C_SCL_PIN				3			//	Digital 3

/*
	Dagu Compound IR Sensor Settings
*/

#define	IR_EYE_ANALOG_TOP				0			//	Analog 0
#define	IR_EYE_ANALOG_LEFT				1			//	Analog 1
#define	IR_EYE_ANALOG_BOTTOM			2			//	Analog 2
#define	IR_EYE_ANALOG_RIGHT				3			//	Analog 3

#define IR_EYE_DIGITAL					4			//	Digital 4

/*
	Hardware Serial ports
*/

//	Serial:	Console and Debug port
#define	SERIAL_CONSOLE_RX_PIN			0
#define	SERIAL_CONSOLE_TX_PIN			1


//	Servos:	Pins and Settings
#define	SERVO_MAX_DEGREES				90
#define SERVO_INVALID_DEGREES			9999
#define	SERVO_CENTER_MS					1500

#define	SERVO_ARM_PAN_PIN				6			//	Digital 6
#define SERVO_ARM_PAN_NAME				"ArmPan"
#define	SERVO_ARM_PAN_HOME				650
#define	SERVO_ARM_PAN_OFFSET			-90
#define	SERVO_ARM_PAN_MIN				500
#define	SERVO_ARM_PAN_MAX				2500

#define	SERVO_ARM_SHOULDER_PIN			7			//	Digital 7
#define SERVO_ARM_SHOULDER_NAME			"ArmShoulder"
#define	SERVO_ARM_SHOULDER_HOME			600
#define	SERVO_ARM_SHOULDER_OFFSET		0
#define	SERVO_ARM_SHOULDER_MIN			500
#define	SERVO_ARM_SHOULDER_MAX			2500

#define	SERVO_ARM_ELBOW_PIN				8			//	Digital 8
#define SERVO_ARM_ELBOW_NAME			"ArmElbow"
#define	SERVO_ARM_ELBOW_HOME			2500
#define	SERVO_ARM_ELBOW_OFFSET			0
#define	SERVO_ARM_ELBOW_MIN				500
#define	SERVO_ARM_ELBOW_MAX				2500

#define	SERVO_GRIP_WRIST_PIN			9			//	Digital 9
#define SERVO_GRIP_WRIST_NAME			"GripWrist"
#define SERVO_GRIP_WRIST_HOME			SERVO_CENTER_MS
#define	SERVO_GRIP_WRIST_OFFSET			0
#define	SERVO_GRIP_WRIST_LEFT_MIN		500
#define	SERVO_GRIP_WRIST_RIGHT_MAX		2400

#define	SERVO_GRIP_GRAB_PIN				10			//	Digital 10
#define SERVO_GRIP_GRAB_NAME			"GripGrab"
#define SERVO_GRIP_GRAB_HOME			SERVO_CENTER_MS
#define	SERVO_GRIP_GRAB_OFFSET			-135
#define	SERVO_GRIP_GRAB_OPEN_MIN		500
#define	SERVO_GRIP_GRAB_CLOSE_MAX		2000

/*********************************************************
	Structs for data we store on various onboard devices
*********************************************************/

struct AreaScanReading {
	float ir;
	uint8_t ping;
	struct ColorSensor *colorData;
	struct HeatSensor *heatData;

	int positionDeg;
};

enum Distance {
	Closest,
	Farthest,
};

struct DistanceReading {
	Distance dist;

	uint8_t irNr;
	float irDistance;
	uint8_t irPositionDeg;

	uint8_t pingNr;
	uint8_t pingDistance;
	uint8_t pingPositionDeg;
};

struct ColorSensor {
	uint16_t colorTemp;
	uint16_t lux;

	uint16_t red;
	uint16_t green;
	uint16_t blue;

	uint16_t c;
};

struct HeatSensor {
	float dieTemp;
	float objectTemp;
};

enum MotorLocation {
	Left,
	Right,
};

struct Motor {
	String name;
	MotorLocation location;

	/*
		R/C PWM control mode parameters
	*/
	uint8_t pin;

	int offset;
	uint16_t msPulse;
	uint16_t minPulse;
	uint16_t maxPulse;

	/*
		Packet Serial control mode parameters
	*/

	//	True, if using encoders
	bool usingEncoders;

	uint32_t encoder;
	uint8_t encoderStatus;
	bool encoderValid;

	uint32_t speed;
	uint8_t speedStatus;
	bool speedValid;

	bool forward;

	long distance;
	bool distanceValid;
};

struct Servo {
	uint8_t pin;
	String name;

	int offset;
	uint16_t homePos;
	uint16_t msPulse;
	uint8_t angle;
	uint16_t minPulse;
	uint16_t maxPulse;
	uint8_t maxDegrees;

	uint16_t error;
};

struct ServoMotor {
	/*
		R/C PWM control mode parameters
	*/
	uint8_t pin;
	String name;

	int offset;
	uint16_t msPulse;
	uint16_t minPulse;
	uint16_t maxPulse;

	//	True, if using encoders
	bool usingEncoders;

	uint32_t encoder;
	uint8_t encoderStatus;
	bool encoderValid;

	uint32_t speed;
	uint8_t speedStatus;
	bool speedValid;

	bool forward;

	long distance;
	bool distanceValid;
};

#endif
