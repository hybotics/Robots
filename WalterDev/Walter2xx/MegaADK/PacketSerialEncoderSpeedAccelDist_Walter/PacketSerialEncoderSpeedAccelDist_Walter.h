/*
	W.A.L.T.E.R. 2.0: Motor Test sketch.
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
	Program:      	W.A.L.T.E.R. 2.0, Motor Test sketch, header file
	Date:         	24-Jun-2014
	Version:      	0.1.0 Arduino Mega R3 - ALPHA

	Purpose:		Added definitions for SoftwareSerial ports for the SSC-32 and RoboClaw 2x5

					Added constants for RoboClaw motor controllers

	Dependencies:	Adafruit libraries:
						LSM303DLHC, L3GD20, TMP006, TCS34727, RTClib for the DS1307

					Hybotics libraries:
						BMP180 (modified from Adafruit's BMP085 library)

	Comments:		Credit is given, where applicable, for code I did not originate.
						This sketch started out as an Adafruit tutorial for the electret
						microphones being used for sound detection. I've also pulled
						code for the GP2D12 IR and PING sensors from the Arduino
						Playground, which I have modified to suit my needs.
*/
#ifndef	__MOTOR_TEST_H__
#define	__MOTOR_TEST_H__

#define	NAV_I2C_ADDRESS					0x50

#define	BUILD_VERSION					"0.1.0"
#define	BUILD_DATE 						"24-Jun-2014"
#define	BUILD_BOARD						"Arduino Mega R3"

#define	COLOR_SENSOR_LED				53

#define	SPEAKER_OUT						52
#define	HEARTBEAT_LED					13

//	Display constants
#define	MAX_NUMBER_7SEG_DISPLAYS		1
#define	SEVEN_SEG_BASE_ADDR				0x70

#define	MATRIX_DISPLAY_ADDR				SEVEN_SEG_BASE_ADDR + MAX_NUMBER_7SEG_DISPLAYS

#define	WAIT_DELAY_SECONDS				10

/*
	These settings control whether standard information is displayed
		on the seven segment and matrix displays or not, and how
		often, in minutes.
*/
#define	DISPLAY_INFORMATION				true

#define	DISPLAY_DATE_FREQ_MIN			15
#define	DISPLAY_TIME_FREQ_MIN			15
#define	DISPLAY_TEMPERATURE_FREQ_MIN	15

/*
	Sensor settings
*/

#define	MAX_NUMBER_AREA_READINGS		36

#define PIR_MOTION_PIN_BASE				30 			//	Digital 30
#define MAX_NUMBER_PIR_MOTION			0

#define	PIR_FRONT_LEFT					0
#define	PIR_FRONT_RIGHT					1
#define	PIR_BACK_LEFT					2
#define	PIR_BACK_RIGHT					3

#define	PING_PIN_BASE					24			//	Digital 24
#define	MAX_NUMBER_PING					1

#define	PING_FRONT_CENTER				0
#define	PING_FRONT_LEFT					1
#define	PING_FRONT_RIGHT				2

#define	MAX_NUMBER_IR					1
#define	IR_PIN_BASE						6			//	Analog 6

#define	IR_FRONT_CENTER					0
#define	IR_BACK_CENTER					1
#define	IR_BACK_LEFT					2
#define	IR_BACK_RIGHT					3

//	RoboClaw 2x5 Motor Controller Packet Serial constants
#define	ROBOCLAW_CONTROLLERS			1
#define	ROBOCLAW_SERIAL_BASE_ADDR		0x80

#define	ROBOCLAW_KP						0x00010000
#define	ROBOCLAW_KI						0x00008000
#define	ROBOCLAW_KD						0x00004000
#define	ROBOCLAW_QPPS					44000

#define ROBOCLAW_MOTOR_LEFT_NAME		"Left"
#define ROBOCLAW_MOTOR_RIGHT_NAME		"Right"
#define ROBOCLAW_MOTOR_FRONT_NAME		"Front"
#define ROBOCLAW_MOTOR_BACK_NAME		"Back"

//  For SoftI2CMaster
#define	SOFT_I2C_SDA_PIN				2
#define	SOFT_I2C_SCL_PIN				3

/*
	Hardware Serial ports
*/
//	Serial:	Console and Debug port
#define	SERIAL_CONSOLE_RX_PIN			0
#define	SERIAL_CONSOLE_TX_PIN			1

//	Serial1: RoboClaw 2x5 Motor Controller
#define	SERIAL_ROBOCLAW_RX_PIN			19
#define	SERIAL_ROBOCLAW_TX_PIN			18

//	Serial2: SSC-32 Servo Controller
#define	SERIAL_SSC32_RX_PIN				17
#define	SERIAL_SSC32_TX_PIN				16

//	Serial3: XBee Wireless Mesh Networking
#define	SERIAL_XBEE_RX_PIN				15
#define	SERIAL_XBEE_TX_PIN				14

/*
	The following settings apply to the SSC-32 servo controller
*/
#define SCAN_SENSOR_DELAY				350

#define	SERVO_MOTOR_LEFT_PIN			4
#define	SERVO_MOTOR_LEFT_ADJUST	        0
#define	SERVO_MOTOR_LEFT_MIN			1000
#define	SERVO_MOTOR_LEFT_MAX			2000

#define	SERVO_MOTOR_RIGHT_PIN	        5
#define	SERVO_MOTOR_RIGHT_ADJUST        0
#define	SERVO_MOTOR_RIGHT_MIN			1000
#define	SERVO_MOTOR_RIGHT_MAX			2000

#define	SERVO_MOTOR_NEUTRAL				1500

#define	SERVO_MAX_DEGREES				90
#define SERVO_INVALID_DEGREES			9999
#define	SERVO_CENTER_MS					1500

#define	SERVO_GRIP_LIFT_PIN				0
#define SERVO_GRIP_LIFT_NAME			"GripLift"
#define	SERVO_GRIP_LIFT_HOME			650
#define	SERVO_GRIP_LIFT_OFFSET			-90
#define	SERVO_GRIP_LIFT_MIN				500
#define	SERVO_GRIP_LIFT_MAX				2500

#define	SERVO_GRIP_WRIST_PIN			1
#define SERVO_GRIP_WRIST_NAME			"GripWrist"
#define	SERVO_GRIP_WRIST_HOME			600
#define	SERVO_GRIP_WRIST_OFFSET			0
#define	SERVO_GRIP_WRIST_MIN			500
#define	SERVO_GRIP_WRIST_MAX			2500

#define	SERVO_GRIP_GRAB_PIN				2
#define SERVO_GRIP_GRAB_NAME			"GripGrab"
#define	SERVO_GRIP_GRAB_HOME			2500
#define	SERVO_GRIP_GRAB_OFFSET			0
#define	SERVO_GRIP_GRAB_MIN				500
#define	SERVO_GRIP_GRAB_MAX				2500

#define	SERVO_PAN_PIN					0
#define SERVO_PAN_NAME					"Pan"
#define SERVO_PAN_HOME					SERVO_CENTER_MS
#define	SERVO_PAN_OFFSET				0
#define	SERVO_PAN_LEFT_MIN				500
#define	SERVO_PAN_RIGHT_MAX				2400

#define	SERVO_TILT_PIN					1
#define SERVO_TILT_NAME					"Tilt"
#define SERVO_TILT_HOME					SERVO_CENTER_MS
#define	SERVO_TILT_OFFSET				-135
#define	SERVO_TILT_DOWN_MIN				500
#define	SERVO_TILT_UP_MAX				2000

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
